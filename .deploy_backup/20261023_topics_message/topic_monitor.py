"""Read-only ROS 2 topic graph and on-demand rate monitor.

ROS imports stay in a separate process; the web server only handles snapshots
and JSON commands.
"""
import atexit
import json
import os
from pathlib import Path
import queue
import re
import shlex
import subprocess
import sys
import threading
import time

STALE_SECONDS = 5.0
MEASURE_RETENTION_SECONDS = 30.0
DEFAULT_MEASURE_SECONDS = 3.0
MIN_MEASURE_SECONDS = 1.0
MAX_MEASURE_SECONDS = 10.0
LOG_PATH = '/tmp/sirius_launcher_topic_monitor.log'
MONITOR_NODE = 'sirius_launcher_topic_monitor'


def topic_name(value):
    value = str(value or '').strip()
    if len(value) > 512 or not re.fullmatch(r'/[^\s]+', value):
        return None
    return value


def node_name(namespace, name):
    namespace = str(namespace or '/').rstrip('/')
    name = str(name or '').lstrip('/')
    return f'{namespace}/{name}' if namespace else f'/{name}'


def enum_name(value):
    name = getattr(value, 'name', None)
    if name:
        return str(name)
    return str(value).rsplit('.', 1)[-1]


def qos_dict(qos):
    if qos is None:
        return {}
    data = {
        'reliability': enum_name(getattr(qos, 'reliability', '')),
        'durability': enum_name(getattr(qos, 'durability', '')),
        'history': enum_name(getattr(qos, 'history', '')),
    }
    depth = getattr(qos, 'depth', None)
    if isinstance(depth, int):
        data['depth'] = depth
    return data


def endpoint_dict(endpoint):
    raw_gid = getattr(endpoint, 'endpoint_gid', b'')
    try:
        gid = bytes(raw_gid).hex()
    except (TypeError, ValueError):
        gid = ''
    namespace = getattr(endpoint, 'node_namespace', '/')
    name = getattr(endpoint, 'node_name', '')
    return {
        'node': node_name(namespace, name),
        'namespace': str(namespace or '/'),
        'name': str(name or ''),
        'type': str(getattr(endpoint, 'topic_type', '') or ''),
        'gid': gid,
        'qos': qos_dict(getattr(endpoint, 'qos_profile', None)),
    }


def unique_endpoints(endpoints):
    seen, result = set(), []
    for endpoint in endpoints:
        key = endpoint.get('node') or endpoint.get('gid')
        if key in seen:
            continue
        seen.add(key)
        result.append(endpoint)
    return result


def rate_from_timestamps(timestamps):
    if len(timestamps) < 2:
        return None
    first, last = timestamps[0], timestamps[-1]
    if last <= first:
        return None
    return (len(timestamps) - 1) / (last - first)


class TopicMonitor:
    def __init__(self):
        self._lock = threading.RLock()
        self._stdin_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._proc = None
        self._latest = None
        self._domain = ''
        self._error = ''
        atexit.register(self.close)

    def _ensure_started_locked(self, domain):
        domain = str(domain)
        if self._thread is not None:
            return self._domain == domain
        self._domain = domain
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name='sirius-topic-monitor')
        self._thread.start()
        return True

    def snapshot(self, domain):
        with self._lock:
            same_domain = self._ensure_started_locked(domain)
            data = dict(self._latest or {})
            fresh = bool(data) and time.time() - data.get('timestamp', 0) < STALE_SECONDS
            proc = self._proc
            return {
                **data,
                'available': fresh and same_domain,
                'domain': self._domain,
                'error': self._error if same_domain else 'ROSドメインが変更されました',
                'receiver_alive': bool(proc and proc.poll() is None),
            }

    def measure(self, domain, topic, duration=DEFAULT_MEASURE_SECONDS):
        topic = topic_name(topic)
        if topic is None:
            return {'started': False, 'error': 'invalid topic', 'status': 400}
        try:
            duration = float(duration)
        except (TypeError, ValueError):
            duration = DEFAULT_MEASURE_SECONDS
        duration = min(MAX_MEASURE_SECONDS, max(MIN_MEASURE_SECONDS, duration))
        with self._lock:
            same_domain = self._ensure_started_locked(domain)
            proc = self._proc
            if not same_domain:
                return {'started': False, 'error': 'ROSドメインが変更されました', 'status': 503}
        if proc is None or proc.poll() is not None:
            return {'started': False, 'error': 'topic receiver is not running', 'status': 503}
        command = json.dumps({'action': 'measure', 'topic': topic, 'duration': duration})
        try:
            with self._stdin_lock:
                proc.stdin.write(command + '\n')
                proc.stdin.flush()
        except (BrokenPipeError, OSError, ValueError) as error:
            with self._lock:
                self._error = str(error)
            return {'started': False, 'error': 'topic receiver is not running', 'status': 503}
        return {'started': True, 'topic': topic, 'duration': duration}

    def _run(self):
        script = Path(__file__).resolve()
        workspace = script.parents[2]
        command = ('source /opt/ros/jazzy/setup.bash && '
                   f'source {shlex.quote(str(workspace / "install/setup.bash"))} && '
                   f'exec python3 {shlex.quote(str(script))} --receive')
        env = {**os.environ, 'ROS_DOMAIN_ID': self._domain, 'PYTHONUNBUFFERED': '1'}
        while not self._stop.is_set():
            try:
                with open(LOG_PATH, 'w') as log:
                    proc = subprocess.Popen(['bash', '-c', command], env=env,
                                            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=log, text=True, bufsize=1)
                    with self._lock:
                        self._proc = proc
                        self._error = ''
                    for line in proc.stdout:
                        try:
                            data = json.loads(line)
                        except (ValueError, TypeError):
                            continue
                        if not isinstance(data, dict) or 'topics' not in data:
                            continue
                        with self._lock:
                            self._latest = {**data, 'timestamp': time.time()}
                    proc.wait()
                if not self._stop.is_set():
                    try:
                        error = Path(LOG_PATH).read_text(errors='replace')[-500:]
                    except OSError:
                        error = ''
                    with self._lock:
                        self._error = error.strip() or 'トピック受信プロセスが終了しました'
            except OSError as error:
                with self._lock:
                    self._error = str(error)
            self._stop.wait(5)

    def close(self):
        self._stop.set()
        proc = self._proc
        if proc and proc.poll() is None:
            if proc.stdin:
                try:
                    proc.stdin.close()
                except OSError:
                    pass
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()


def _measurement_public(topic, measurement):
    state = measurement['state']
    data = {
        'topic': topic,
        'state': state,
        'type': measurement.get('type', ''),
        'duration': measurement.get('duration', DEFAULT_MEASURE_SECONDS),
        'started': measurement.get('started'),
        'finished': measurement.get('finished'),
        'count': len(measurement.get('timestamps', [])),
        'rate_hz': measurement.get('rate_hz'),
    }
    if measurement.get('error'):
        data['error'] = measurement['error']
    return data


def receive():
    import rclpy
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = rclpy.create_node(MONITOR_NODE)
    commands = queue.Queue()
    stop = threading.Event()
    measurements = {}

    def command_reader():
        for line in sys.stdin:
            try:
                command = json.loads(line)
            except (ValueError, TypeError):
                continue
            if isinstance(command, dict):
                commands.put(command)
        stop.set()

    threading.Thread(target=command_reader, daemon=True, name='sirius-topic-commands').start()

    def finish_due(now):
        for topic, measurement in list(measurements.items()):
            if measurement['state'] != 'measuring' or now < measurement['deadline']:
                continue
            stamps = measurement.get('timestamps', [])
            measurement['rate_hz'] = rate_from_timestamps(stamps)
            measurement['state'] = 'done'
            measurement['finished'] = time.time()
            if not stamps:
                measurement['error'] = 'メッセージを受信できませんでした'
            sub = measurement.pop('sub', None)
            if sub is not None:
                try:
                    node.destroy_subscription(sub)
                except Exception:
                    pass

    def prune(now):
        for topic, measurement in list(measurements.items()):
            if measurement['state'] == 'measuring':
                continue
            if now - (measurement.get('deadline') or now) > MEASURE_RETENTION_SECONDS:
                measurements.pop(topic, None)

    def start_measure(topic, duration):
        topic = topic_name(topic)
        if topic is None:
            return
        try:
            duration = float(duration)
        except (TypeError, ValueError):
            duration = DEFAULT_MEASURE_SECONDS
        duration = min(MAX_MEASURE_SECONDS, max(MIN_MEASURE_SECONDS, duration))
        current = measurements.get(topic)
        if current and current['state'] == 'measuring':
            return
        try:
            types = dict(node.get_topic_names_and_types()).get(topic, [])
        except Exception as error:
            measurements[topic] = {'state': 'error', 'error': str(error), 'timestamps': [],
                                   'duration': duration, 'started': time.time(),
                                   'finished': time.time(), 'deadline': time.monotonic()}
            return
        if not types:
            measurements[topic] = {'state': 'error', 'error': 'トピックが見つかりません',
                                   'timestamps': [], 'duration': duration,
                                   'started': time.time(), 'finished': time.time(),
                                   'deadline': time.monotonic()}
            return
        type_name = types[0]
        timestamps = []
        try:
            message_type = get_message(type_name)
            qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            sub = node.create_subscription(message_type, topic,
                                           lambda _message: timestamps.append(time.monotonic()),
                                           qos)
        except Exception as error:
            measurements[topic] = {'state': 'error', 'type': type_name, 'error': str(error),
                                   'timestamps': [], 'duration': duration,
                                   'started': time.time(), 'finished': time.time(),
                                   'deadline': time.monotonic()}
            return
        now = time.monotonic()
        measurements[topic] = {
            'state': 'measuring',
            'type': type_name,
            'duration': duration,
            'started': time.time(),
            'deadline': now + duration,
            'timestamps': timestamps,
            'sub': sub,
            'rate_hz': None,
        }

    def topic_rows():
        rows = []
        try:
            named_types = node.get_topic_names_and_types()
        except Exception as error:
            return rows, str(error)
        for name, types in sorted(named_types, key=lambda item: item[0]):
            try:
                publishers = unique_endpoints([endpoint_dict(endpoint)
                                               for endpoint in node.get_publishers_info_by_topic(name)
                                               if endpoint.node_name != MONITOR_NODE])
            except Exception:
                publishers = []
            try:
                subscribers = unique_endpoints([endpoint_dict(endpoint)
                                                for endpoint in node.get_subscriptions_info_by_topic(name)
                                                if endpoint.node_name != MONITOR_NODE])
            except Exception:
                subscribers = []
            rows.append({
                'name': name,
                'type': types[0] if types else '',
                'types': list(types),
                'publishers': publishers,
                'subscribers': subscribers,
                'publisher_count': len(publishers),
                'subscriber_count': len(subscribers),
            })
        return rows, ''

    def emit():
        now = time.monotonic()
        finish_due(now)
        prune(now)
        rows, error = topic_rows()
        data = {
            'domain': os.environ.get('ROS_DOMAIN_ID', ''),
            'topics': rows,
            'measurements': {topic: _measurement_public(topic, measurement)
                             for topic, measurement in sorted(measurements.items())},
        }
        if error:
            data['error'] = error
        try:
            print(json.dumps(data, allow_nan=False), flush=True)
        except (BrokenPipeError, OSError):
            stop.set()

    emit()
    last_emit = time.monotonic()
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
            while True:
                try:
                    command = commands.get_nowait()
                except queue.Empty:
                    break
                if command.get('action') == 'measure':
                    start_measure(command.get('topic'), command.get('duration'))
            now = time.monotonic()
            finish_due(now)
            if now - last_emit >= 1.0:
                emit()
                last_emit = now
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    if '--receive' in sys.argv:
        receive()
