"""Read-only, shared ROS TF receiver; ROS imports stay in a separate process."""
from collections import deque
import atexit
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import threading
import time


class TransformStore:
    def __init__(self, retention=60, max_edges=2000):
        self.retention, self.max_edges = retention, max_edges
        self.edges = {}

    def ingest(self, parent, child, static, publisher, stamp, translation, rotation, now):
        if not parent or not child or not all(math.isfinite(v) for v in (*translation, *rotation, stamp)):
            return
        key = (parent, child, static, publisher)
        if key not in self.edges:
            if len(self.edges) >= self.max_edges:
                return
            self.edges[key] = {'parent': parent, 'child': child, 'static': static,
                               'publisher_gid': publisher, 'samples': deque(maxlen=500)}
        edge = self.edges[key]
        edge.update(stamp=stamp, translation=list(translation), rotation=list(rotation), last_seen=now)
        edge['samples'].append(now)

    def snapshot(self, now, publishers=None):
        publishers = publishers or {}
        rows = []
        for key, edge in list(self.edges.items()):
            age = max(0, now-edge['last_seen'])
            if not edge['static'] and age > self.retention:
                del self.edges[key]
                continue
            samples = [t for t in edge['samples'] if t >= now-5]
            rate = (len(samples)-1)/(samples[-1]-samples[0]) if len(samples)>1 and samples[-1]>samples[0] else None
            rows.append({k: v for k, v in edge.items() if k not in ('samples', 'last_seen')})
            rows[-1].update(age_seconds=age, stale=not edge['static'] and age>3,
                            rate_hz=None if edge['static'] else rate,
                            publisher=publishers.get(edge['publisher_gid'], '配信元を確認中'))
        rows.sort(key=lambda e: (e['parent'], e['child'], e['static'], e['publisher_gid']))
        frames = sorted({e[k] for e in rows for k in ('parent', 'child')})
        children = {e['child'] for e in rows}
        sources = {}
        for edge in rows:
            sources.setdefault(edge['child'], set()).add((edge['parent'], edge['publisher_gid'], edge['static']))
        # Detect actual cycle members, rather than all descendants of a cycle.
        adjacency = {f: [] for f in frames}
        for edge in rows:
            adjacency[edge['parent']].append(edge['child'])
        visited, cyclic = set(), set()
        for root in frames:
            if root in visited: continue
            path, active = [root], {root: 0}
            visited.add(root)
            stack = [(root, iter(adjacency[root]))]
            while stack:
                frame, child_iter = stack[-1]
                child = next(child_iter, None)
                if child is None:
                    stack.pop(); active.pop(frame); path.pop()
                elif child in active:
                    cyclic.update(path[active[child]:])
                elif child not in visited:
                    visited.add(child); active[child] = len(path); path.append(child)
                    stack.append((child, iter(adjacency[child])))
        return {'frames': frames, 'edges': rows, 'roots': sorted(set(frames)-children),
                'conflicts': sorted(f for f, src in sources.items() if len(src)>1),
                'cycles': sorted(cyclic), 'retention_seconds': self.retention,
                'at_capacity': len(self.edges)>=self.max_edges}


class TFMonitor:
    def __init__(self):
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = self._proc = self._latest = None
        self._domain = ''
        self._error = ''
        atexit.register(self.close)

    def snapshot(self, domain):
        with self._lock:
            if self._thread is None:
                self._domain = str(domain)
                self._thread = threading.Thread(target=self._run, daemon=True, name='sirius-tf-monitor')
                self._thread.start()
            data = dict(self._latest or {})
            fresh = bool(data) and time.time()-data['timestamp'] < 5
            return {**data, 'available': fresh, 'domain': self._domain,
                    'error': self._error, 'receiver_alive': self._proc is not None and self._proc.poll() is None}

    def _run(self):
        script = Path(__file__).resolve()
        workspace = script.parents[2]
        command = ('source /opt/ros/jazzy/setup.bash && '
                   f'source {shlex.quote(str(workspace/"install/setup.bash"))} && '
                   f'exec python3 {shlex.quote(str(script))} --receive')
        env = {**os.environ, 'ROS_DOMAIN_ID': self._domain, 'PYTHONUNBUFFERED': '1'}
        while not self._stop.is_set():
            try:
                with open('/tmp/sirius_launcher_tf_monitor.log', 'w') as log:
                    proc = subprocess.Popen(['bash', '-c', command], env=env,
                                            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=log, text=True)
                    with self._lock: self._proc = proc
                    for line in proc.stdout:
                        try:
                            data = json.loads(line)
                            if 'edges' not in data: continue
                        except (ValueError, TypeError): continue
                        with self._lock:
                            self._latest, self._error = {**data, 'timestamp': time.time()}, ''
                    proc.wait()
                if not self._stop.is_set():
                    error = Path('/tmp/sirius_launcher_tf_monitor.log').read_text(errors='replace')[-500:]
                    with self._lock: self._error = error.strip() or 'TF受信プロセスが終了しました'
            except OSError as error:
                with self._lock: self._error = str(error)
            self._stop.wait(5)

    def close(self):
        self._stop.set()
        proc = self._proc
        if proc and proc.poll() is None:
            if proc.stdin: proc.stdin.close()
            try: proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try: proc.wait(timeout=2)
                except subprocess.TimeoutExpired: proc.kill(); proc.wait()


def receive():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from tf2_msgs.msg import TFMessage
    rclpy.init()
    node = rclpy.create_node('sirius_launcher_tf_monitor')
    store, publishers, stop = TransformStore(), {}, threading.Event()
    # Parent/server closure closes the pipe, including exec/self-restarts.
    threading.Thread(target=lambda: (sys.stdin.read(), stop.set()), daemon=True).start()
    def callback(message, info, static):
        now = time.monotonic()
        raw_gid = info.get('publisher_gid') if isinstance(info, dict) else getattr(info, 'publisher_gid', None)
        gid = bytes(raw_gid).hex() if raw_gid is not None and any(raw_gid) else 'unknown:'+('/tf_static' if static else '/tf')
        for tf in message.transforms:
            t, q, stamp = tf.transform.translation, tf.transform.rotation, tf.header.stamp
            store.ingest(tf.header.frame_id, tf.child_frame_id, static, gid,
                         stamp.sec+stamp.nanosec/1e9, (t.x,t.y,t.z), (q.x,q.y,q.z,q.w), now)
    node.create_subscription(TFMessage, '/tf', lambda msg, info: callback(msg, info, False),
                             QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(TFMessage, '/tf_static', lambda msg, info: callback(msg, info, True),
                             QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def publish_snapshot():
        for topic in ('/tf','/tf_static'):
            candidates = []
            for endpoint in node.get_publishers_info_by_topic(topic):
                name = endpoint.node_namespace.rstrip('/')+'/'+endpoint.node_name
                publishers[bytes(endpoint.endpoint_gid).hex()] = name
                candidates.append(name)
            publishers['unknown:'+topic] = '候補: '+', '.join(sorted(set(candidates))) if candidates else '配信元を特定できません'
        print(json.dumps(store.snapshot(time.monotonic(), publishers), allow_nan=False), flush=True)
    node.create_timer(1.0, publish_snapshot)
    try:
        while rclpy.ok() and not stop.is_set(): rclpy.spin_once(node, timeout_sec=.2)
    except (KeyboardInterrupt, BrokenPipeError): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    receive()
