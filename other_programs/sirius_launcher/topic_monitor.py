"""Read-only ROS 2 topic graph and on-demand rate monitor.

ROS imports stay in a separate process; the web server only handles snapshots
and JSON commands.
"""
import atexit
import json
import math
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
WATCH_LEASE_SECONDS = 10.0
MIN_WATCH_LEASE_SECONDS = 3.0
MAX_WATCH_LEASE_SECONDS = 30.0
MESSAGE_MAX_CHARS = 12000
MESSAGE_MAX_ITEMS = 24
MESSAGE_MAX_DEPTH = 5
MESSAGE_MAX_STRING = 400
MESSAGE_MAX_BYTES = 32
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


def _is_sequence(value):
    if isinstance(value, (str, bytes, bytearray, dict)):
        return False
    if hasattr(value, 'get_fields_and_field_types'):
        return False
    try:
        len(value)
    except (TypeError, ValueError):
        return False
    return hasattr(value, '__getitem__')


class _MessagePreview:
    """Convert a ROS message into a small, JSON-safe preview.

    Images and point clouds can contain megabytes of binary/array data. Never
    serialize the whole payload; show a bounded prefix and retain its length.
    """

    def __init__(self):
        self.remaining = MESSAGE_MAX_CHARS
        self.truncated = False

    def _text(self, value, limit=MESSAGE_MAX_STRING):
        text = str(value)
        if len(text) > limit:
            text = text[:limit] + '…'
            self.truncated = True
        if self.remaining <= 0:
            self.truncated = True
            return '…'
        if len(text) > self.remaining:
            text = text[:max(0, self.remaining)] + '…'
            self.truncated = True
        self.remaining -= len(text)
        return text

    def _number(self, value):
        self.remaining -= 8
        return value

    def value(self, value, depth=0):
        if self.remaining <= 0:
            self.truncated = True
            return '…'
        if depth > MESSAGE_MAX_DEPTH:
            self.truncated = True
            return '…'
        if value is None:
            return None
        if isinstance(value, bool):
            return value
        if isinstance(value, int):
            return self._number(value)
        if isinstance(value, float):
            if math.isfinite(value):
                return self._number(value)
            self.truncated = True
            return str(value)
        if isinstance(value, str):
            return self._text(value)
        if isinstance(value, (bytes, bytearray)):
            data = bytes(value)
            if len(data) <= MESSAGE_MAX_BYTES:
                return {'__type__': 'bytes', 'length': len(data), 'hex': data.hex()}
            self.truncated = True
            return {'__type__': 'bytes', 'length': len(data)}
        if hasattr(value, 'get_fields_and_field_types'):
            result = {}
            try:
                fields = value.get_fields_and_field_types()
            except Exception:
                fields = {}
            for name in fields:
                if self.remaining <= 0:
                    self.truncated = True
                    break
                try:
                    field = getattr(value, name)
                except Exception:
                    continue
                result[name] = self.value(field, depth + 1)
            return result
        if isinstance(value, dict):
            result = {}
            for key, item in value.items():
                if self.remaining <= 0:
                    self.truncated = True
                    break
                result[str(key)] = self.value(item, depth + 1)
            return result
        if _is_sequence(value):
            try:
                length = len(value)
            except (TypeError, ValueError):
                length = 0
            limit = min(length, MESSAGE_MAX_ITEMS)
            items = []
            for index in range(limit):
                if self.remaining <= 0:
                    self.truncated = True
                    break
                try:
                    items.append(self.value(value[index], depth + 1))
                except (IndexError, KeyError, TypeError):
                    break
            if len(items) < length:
                self.truncated = True
                return {'__type__': 'array', 'length': length, 'items': items}
            return items
        if hasattr(value, 'item') and callable(value.item):
            try:
                return self.value(value.item(), depth)
            except Exception:
                pass
        return self._text(value)


def message_preview(message):
    """Return a bounded JSON text preview and whether anything was omitted."""
    preview = _MessagePreview()
    data = preview.value(message)
    try:
        text = json.dumps(data, ensure_ascii=False, allow_nan=False, indent=2)
    except (TypeError, ValueError):
        preview.truncated = True
        text = json.dumps(str(data), ensure_ascii=False)
    if len(text) > MESSAGE_MAX_CHARS:
        text = text[:MESSAGE_MAX_CHARS].rstrip() + '\n…'
        preview.truncated = True
    return text, preview.truncated


def _watch_public(watch):
    if not watch:
        return None
    return {
        'topic': watch.get('topic', ''),
        'type': watch.get('type', ''),
        'state': watch.get('state', 'stopped'),
        'count': watch.get('count', 0),
        'received_at': watch.get('received_at'),
        'text': watch.get('text'),
        'truncated': bool(watch.get('truncated')),
        'error': watch.get('error', ''),
        'expired': bool(watch.get('expired')),
    }


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

    def watch(self, domain, topic, lease=WATCH_LEASE_SECONDS):
        topic = topic_name(topic)
        if topic is None:
            return {'started': False, 'error': 'invalid topic', 'status': 400}
        try:
            lease = float(lease)
        except (TypeError, ValueError):
            lease = WATCH_LEASE_SECONDS
        lease = min(MAX_WATCH_LEASE_SECONDS, max(MIN_WATCH_LEASE_SECONDS, lease))
        with self._lock:
            same_domain = self._ensure_started_locked(domain)
            proc = self._proc
            if not same_domain:
                return {'started': False, 'error': 'ROSドメインが変更されました', 'status': 503}
        if proc is None or proc.poll() is not None:
            for _ in range(20):
                time.sleep(0.05)
                with self._lock:
                    proc = self._proc
                if proc is not None and proc.poll() is None:
                    break
        if proc is None or proc.poll() is not None:
            return {'started': False, 'error': 'topic receiver is not running', 'status': 503}
        command = json.dumps({'action': 'watch', 'topic': topic, 'lease': lease})
        try:
            with self._stdin_lock:
                proc.stdin.write(command + '\n')
                proc.stdin.flush()
        except (BrokenPipeError, OSError, ValueError) as error:
            with self._lock:
                self._error = str(error)
            return {'started': False, 'error': 'topic receiver is not running', 'status': 503}
        return {'started': True, 'topic': topic, 'lease': lease}

    def unwatch(self, domain):
        with self._lock:
            if self._thread is None:
                return {'stopped': True}
            same_domain = self._ensure_started_locked(domain)
            proc = self._proc
            if not same_domain:
                return {'stopped': False, 'error': 'ROSドメインが変更されました', 'status': 503}
        if proc is None or proc.poll() is not None:
            return {'stopped': False, 'error': 'topic receiver is not running', 'status': 503}
        try:
            with self._stdin_lock:
                proc.stdin.write(json.dumps({'action': 'unwatch'}) + '\n')
                proc.stdin.flush()
        except (BrokenPipeError, OSError, ValueError) as error:
            with self._lock:
                self._error = str(error)
            return {'stopped': False, 'error': 'topic receiver is not running', 'status': 503}
        return {'stopped': True}

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
    watch = {}

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

    def watch_qos(topic):
        from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
        try:
            endpoints = node.get_publishers_info_by_topic(topic)
        except Exception:
            endpoints = []
        reliable = bool(endpoints)
        transient = bool(endpoints)
        for endpoint in endpoints:
            profile = getattr(endpoint, 'qos_profile', None)
            if profile is None:
                reliable = transient = False
                break
            if getattr(profile, 'reliability', None) != ReliabilityPolicy.RELIABLE:
                reliable = False
            if getattr(profile, 'durability', None) != DurabilityPolicy.TRANSIENT_LOCAL:
                transient = False
        return QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE,
        )

    def stop_watch(expired=False):
        sub = watch.pop('sub', None)
        if sub is not None:
            try:
                node.destroy_subscription(sub)
            except Exception:
                pass
        if not watch:
            return
        if watch.get('state') in ('starting', 'watching'):
            watch['state'] = 'stopped'
        watch['expired'] = bool(expired)
        watch['deadline'] = 0.0
        if expired and watch.get('count', 0) == 0 and not watch.get('error'):
            watch['error'] = 'メッセージを受信できませんでした'

    def start_watch(topic, lease):
        topic = topic_name(topic)
        if topic is None:
            return
        try:
            lease = float(lease)
        except (TypeError, ValueError):
            lease = WATCH_LEASE_SECONDS
        lease = min(MAX_WATCH_LEASE_SECONDS, max(MIN_WATCH_LEASE_SECONDS, lease))
        if watch.get('topic') == topic and watch.get('state') in ('starting', 'watching'):
            watch['deadline'] = time.monotonic() + lease
            return
        stop_watch()
        watch.clear()
        try:
            types = dict(node.get_topic_names_and_types()).get(topic, [])
        except Exception as error:
            watch.update(topic=topic, state='error', error=str(error), count=0,
                         received_at=None, text=None, truncated=False, expired=False)
            return
        if not types:
            watch.update(topic=topic, state='error', error='トピックが見つかりません',
                         count=0, received_at=None, text=None, truncated=False,
                         expired=False)
            return
        type_name = types[0]
        watch.update(topic=topic, type=type_name, state='starting', count=0,
                     received_at=None, text=None, truncated=False, error='',
                     expired=False, deadline=time.monotonic() + lease)
        try:
            message_type = get_message(type_name)
            qos = watch_qos(topic)

            def on_message(message):
                if watch.get('topic') != topic:
                    return
                text, truncated = message_preview(message)
                watch['text'] = text
                watch['truncated'] = truncated
                watch['received_at'] = time.time()
                watch['count'] = watch.get('count', 0) + 1
                watch['state'] = 'watching'

            sub = node.create_subscription(message_type, topic, on_message, qos)
        except Exception as error:
            watch.update(state='error', error=str(error))
            return
        watch['sub'] = sub
        watch['state'] = 'watching'

    def finish_watch_due(now):
        if (watch.get('state') in ('starting', 'watching') and
                watch.get('deadline', 0) and now > watch['deadline']):
            stop_watch(expired=True)

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
        finish_watch_due(now)
        prune(now)
        rows, error = topic_rows()
        data = {
            'domain': os.environ.get('ROS_DOMAIN_ID', ''),
            'topics': rows,
            'measurements': {topic: _measurement_public(topic, measurement)
                             for topic, measurement in sorted(measurements.items())},
            'watch': _watch_public(watch),
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
                action = command.get('action')
                if action == 'measure':
                    start_measure(command.get('topic'), command.get('duration'))
                elif action == 'watch':
                    start_watch(command.get('topic'), command.get('lease'))
                elif action == 'unwatch':
                    stop_watch()
            now = time.monotonic()
            finish_due(now)
            finish_watch_due(now)
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
