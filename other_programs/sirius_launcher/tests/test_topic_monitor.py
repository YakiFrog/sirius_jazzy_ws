"""Topic graph helpers, on-demand rate math, and authenticated API access."""
import importlib
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from topic_monitor import (TopicMonitor, endpoint_dict, node_name,
                           qos_dict, rate_from_timestamps, topic_name,
                           unique_endpoints)


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

        def snapshot(self, domain):
            assert domain == '53'
            return {'available': True, 'domain': domain, 'topics': [], 'measurements': {}}

        def measure(self, domain, topic, duration):
            assert domain == '53'
            self.measured = (topic, duration)
            return {'started': True, 'topic': topic, 'duration': 3.0}

    fake = FakeMonitor()
    monkeypatch.setattr(web, 'TOPIC_MONITOR', fake)
    monkeypatch.setattr(web, 'DEFAULT_DOMAIN', '53')
    monkeypatch.setattr(web, 'PASSWORD', 'test-password')
    client = web.app.test_client()
    assert client.get('/api/topics').status_code == 401
    assert client.post('/api/topics/measure', json={'topic': '/scan'}).status_code == 401
    client.set_cookie('pw', 'test-password')
    response = client.get('/api/topics')
    assert response.json['available'] and response.json['domain'] == '53'
    assert response.headers['Cache-Control'] == 'no-store'
    response = client.post('/api/topics/measure', json={'topic': '/scan', 'duration': 3})
    assert response.status_code == 200
    assert response.json['started'] is True
    assert fake.measured == ('/scan', 3)
