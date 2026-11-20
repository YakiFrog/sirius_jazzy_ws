"""Topic graph helpers, on-demand rate math, and authenticated API access."""
import importlib
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from topic_monitor import (MESSAGE_MAX_CHARS, TopicMonitor, endpoint_dict,
                           message_preview, node_name, qos_dict,
                           rate_from_timestamps, topic_name, unique_endpoints)


class FakeQoS:
    class Reliability:
        name = 'RELIABLE'
    class Durability:
        name = 'TRANSIENT_LOCAL'
    class History:
        name = 'KEEP_LAST'

    reliability = Reliability()
    durability = Durability()
    history = History()
    depth = 5


class FakeEndpoint:
    node_namespace = '/sirius'
    node_name = 'perception'
    topic_type = 'sensor_msgs/msg/LaserScan'
    endpoint_gid = b'\x01\xab'
    qos_profile = FakeQoS()


def test_rate_uses_receive_window_and_requires_two_messages():
    assert rate_from_timestamps([]) is None
    assert rate_from_timestamps([1.0]) is None
    assert math.isclose(rate_from_timestamps([1.0, 1.1, 1.2]), 10.0)


def test_topic_and_node_names_are_normalized_and_bounded():
    assert node_name('/', 'node') == '/node'
    assert node_name('/ns/', '/node') == '/ns/node'
    assert topic_name('/scan') == '/scan'
    assert topic_name('scan') is None
    assert topic_name('/has space') is None
    assert topic_name('/' + 'x' * 512) is None


def test_endpoint_serializes_node_type_gid_and_qos():
    data = endpoint_dict(FakeEndpoint())
    assert data['node'] == '/sirius/perception'
    assert data['namespace'] == '/sirius'
    assert data['name'] == 'perception'
    assert data['type'] == 'sensor_msgs/msg/LaserScan'
    assert data['gid'] == '01ab'
    assert data['qos'] == {'reliability': 'RELIABLE', 'durability': 'TRANSIENT_LOCAL',
                           'history': 'KEEP_LAST', 'depth': 5}
    assert qos_dict(None) == {}


def test_endpoints_are_unique_per_node():
    first = endpoint_dict(FakeEndpoint())
    second = {**first, 'gid': '02cd'}
    assert unique_endpoints([first, second]) == [first]


class FakeHeader:
    frame_id = 'map'
    stamp = 1.5

    def get_fields_and_field_types(self):
        return {'stamp': 'builtin_interfaces/Time', 'frame_id': 'string'}


class FakeDiagnosticMessage:
    header = FakeHeader()
    ranges = [1.0] * 100
    data = b'\x00' * 100
    note = 'x' * 1000
    value = float('nan')

    def get_fields_and_field_types(self):
        return {'header': 'std_msgs/Header', 'ranges': 'double[]',
                'data': 'uint8[]', 'note': 'string', 'value': 'double'}


def test_message_preview_bounds_large_arrays_bytes_and_strings():
    text, truncated = message_preview(FakeDiagnosticMessage())
    assert truncated
    assert '"__type__": "array"' in text
    assert '"length": 100' in text
    assert '"__type__": "bytes"' in text
    assert '"length": 1000' not in text
    assert 'nan' in text
    assert len(text) <= MESSAGE_MAX_CHARS + 2


def test_invalid_watch_is_rejected_without_starting_ros():
    monitor = TopicMonitor()
    assert monitor.watch('53', 'scan') == {'started': False, 'error': 'invalid topic', 'status': 400}
    assert monitor._thread is None


def test_topics_message_viewer_is_wired_to_watch_api():
    root = Path(__file__).resolve().parents[1]
    script = (root / 'static/topics.js').read_text(encoding='utf-8')
    style = (root / 'static/topics.css').read_text(encoding='utf-8')
    assert '/api/topics/watch' in script
    assert "TOPIC_VIEW.watchTopic" in script
    assert "topicMessageSection" in script
    assert '.topics-message-text' in style
    assert 'topics-splitter' in script
    assert 'sirius-topics-inspector-width' in script
    assert '.topics-splitter' in style
    assert '--topics-inspector-width' in style


def test_invalid_measurement_is_rejected_without_starting_ros():
    monitor = TopicMonitor()
    result = monitor.measure('53', 'scan')
    assert result == {'started': False, 'error': 'invalid topic', 'status': 400}
    assert monitor._thread is None


def test_topics_api_auth_snapshot_and_measure(monkeypatch):
    web = importlib.import_module('web_server')

    class FakeMonitor:
        def __init__(self):
            self.measured = None
            self.watched = None
            self.unwatched = False

        def snapshot(self, domain):
            assert domain == '53'
            return {'available': True, 'domain': domain, 'topics': [], 'measurements': {}}

        def measure(self, domain, topic, duration):
            assert domain == '53'
            self.measured = (topic, duration)
            return {'started': True, 'topic': topic, 'duration': 3.0}

        def watch(self, domain, topic, lease):
            assert domain == '53'
            self.watched = (topic, lease)
            return {'started': True, 'topic': topic, 'lease': 10.0}

        def unwatch(self, domain):
            assert domain == '53'
            self.unwatched = True
            return {'stopped': True}

    fake = FakeMonitor()
    monkeypatch.setattr(web, 'TOPIC_MONITOR', fake)
    monkeypatch.setattr(web, 'DEFAULT_DOMAIN', '53')
    monkeypatch.setattr(web, 'PASSWORD', 'test-password')
    client = web.app.test_client()
    assert client.get('/api/topics').status_code == 401
    assert client.post('/api/topics/measure', json={'topic': '/scan'}).status_code == 401
    assert client.post('/api/topics/watch', json={'topic': '/scan'}).status_code == 401
    client.set_cookie('pw', 'test-password')
    response = client.get('/api/topics')
    assert response.json['available'] and response.json['domain'] == '53'
    assert response.headers['Cache-Control'] == 'no-store'
    response = client.post('/api/topics/measure', json={'topic': '/scan', 'duration': 3})
    assert response.status_code == 200
    assert response.json['started'] is True
    assert fake.measured == ('/scan', 3)
    response = client.post('/api/topics/watch', json={'action': 'start', 'topic': '/scan', 'lease': 10})
    assert response.status_code == 200
    assert response.json['started'] is True
    assert fake.watched == ('/scan', 10)
    response = client.post('/api/topics/watch', json={'action': 'stop'})
    assert response.status_code == 200
    assert response.json['stopped'] is True
    assert fake.unwatched is True
