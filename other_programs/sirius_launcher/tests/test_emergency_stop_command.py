import importlib
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import emergency_stop_command as command


@pytest.fixture
def web(monkeypatch):
    module = importlib.import_module('web_server')
    monkeypatch.setattr(module, 'PASSWORD', '')
    monkeypatch.setattr(module, 'DEFAULT_DOMAIN', '53')
    return module


@pytest.mark.parametrize('body', [{}, {'active': 1}, {'active': 'false'}, ['on']])
def test_invalid_request_never_sends(web, monkeypatch, body):
    monkeypatch.setattr(web, 'send_emergency_stop', lambda *a: pytest.fail('must not send'))
    assert web.app.test_client().post('/api/emergency_stop', json=body).status_code == 400


def test_auth_and_release_confirmation(web, monkeypatch):
    monkeypatch.setattr(web, 'send_emergency_stop', lambda *a: pytest.fail('must not send'))
    client = web.app.test_client()
    monkeypatch.setattr(web, 'PASSWORD', 'secret')
    assert client.post('/api/emergency_stop', json={'active': True}).status_code == 401
    client.set_cookie('pw', 'secret')
    assert client.post('/api/emergency_stop', json={'active': False}).status_code == 400


@pytest.mark.parametrize('state,available,age,active', [
    ('stale', True, 6, True), ('live', False, 0, True),
    ('live', True, 0, None), ('live', True, 0, False), ('live', True, 6, True),
])
def test_release_requires_current_confirmed_electronic_stop(web, monkeypatch, state, available, age, active):
    monkeypatch.setattr(web, 'send_emergency_stop', lambda *a: pytest.fail('must not send'))
    monkeypatch.setattr(web.TF_MONITOR, 'snapshot', lambda domain: {
        'available': available, 'timestamp': time.time(), 'battery': {
            'driver': {'state': state, 'age_seconds': age, 'emergency_stop': active}}})
    assert web.app.test_client().post('/api/emergency_stop', json={
        'active': False, 'confirmed': True}).status_code == 409


def test_stop_and_release_send_explicit_boolean_on_monitor_domain(web, monkeypatch):
    sent = []
    monkeypatch.setattr(web, 'send_emergency_stop', lambda active, domain:
                        sent.append((active, domain)) or {'sent': True, 'active': active})
    monkeypatch.setattr(web.TF_MONITOR, 'snapshot', lambda domain: {
        'available': True, 'timestamp': time.time(), 'battery': {
            'driver': {'state': 'live', 'age_seconds': .1, 'emergency_stop': True}}})
    client = web.app.test_client()
    assert client.post('/api/emergency_stop', json={'active': True}).json['sent']
    assert client.post('/api/emergency_stop', json={'active': False, 'confirmed': True}).json['sent']
    assert sent == [(True, '53'), (False, '53')]


def test_send_failure_is_not_reported_as_success(web, monkeypatch):
    monkeypatch.setattr(web, 'send_emergency_stop', lambda *a: {'sent': False, 'error': 'offline'})
    assert web.app.test_client().post('/api/emergency_stop', json={'active': True}).status_code == 502


def test_ble_subscriber_alone_is_insufficient():
    node = SimpleNamespace(get_subscriptions_info_by_topic=lambda topic: [
        SimpleNamespace(node_name='sirius_ble_gateway')])
    publisher = SimpleNamespace(get_subscription_count=lambda: 1)
    with pytest.raises(RuntimeError, match='Roboteq'):
        command.wait_for_driver(node, publisher, lambda *a, **k: None, timeout=.001)


def test_ros_command_publishes_once_and_checks_ack(monkeypatch):
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', SimpleNamespace(Bool=SimpleNamespace))
    node = SimpleNamespace(get_subscriptions_info_by_topic=lambda topic: [
        SimpleNamespace(node_name='roboteq_ros2_driver')])
    messages = []
    publisher = SimpleNamespace(get_subscription_count=lambda: 1,
                                publish=lambda msg: messages.append(msg.data),
                                wait_for_all_acked=lambda duration: True)
    command.publish_stop(node, publisher, True, lambda *a, **k: None, lambda **k: None)
    assert messages == [True]
    publisher.wait_for_all_acked = lambda duration: False
    with pytest.raises(RuntimeError, match='応答'):
        command.publish_stop(node, publisher, False, lambda *a, **k: None, lambda **k: None)
    assert messages == [True, False]


@pytest.mark.parametrize('active', [True, False])
def test_live_driver_feedback_confirms_on_and_off_with_dds_ack(monkeypatch, active):
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', SimpleNamespace(Bool=SimpleNamespace))
    node = SimpleNamespace(get_subscriptions_info_by_topic=lambda topic: [
        SimpleNamespace(node_name='roboteq_ros2_driver')])
    messages, feedback = [], {'active': active}
    publisher = SimpleNamespace(get_subscription_count=lambda: 1,
                                publish=lambda msg: messages.append(msg.data),
                                wait_for_all_acked=lambda duration: True)
    def spin(*args, **kwargs):
        assert not feedback  # Old feedback must be discarded before sending.
        feedback['active'] = active
    command.publish_stop(node, publisher, active, spin, lambda **kwargs: None,
                         feedback=feedback)
    assert messages == [active]


def test_release_waits_for_mux_and_retries_explicit_request(monkeypatch):
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', SimpleNamespace(Bool=SimpleNamespace))
    clock = [0.0]
    monkeypatch.setattr(command.time, 'monotonic', lambda: clock[0])
    node = SimpleNamespace(get_subscriptions_info_by_topic=lambda topic: [
        SimpleNamespace(node_name='roboteq_ros2_driver'),
        SimpleNamespace(node_name='twist_mux')])
    sent, feedback = [], {}
    publisher = SimpleNamespace(get_subscription_count=lambda: 2,
        publish=lambda msg: sent.append(msg.data), wait_for_all_acked=lambda duration: True)
    def spin(*args, **kwargs):
        clock[0] += .1
        feedback['active'] = False
        feedback['mux_active'] = len(sent) < 2
    command.publish_stop(node, publisher, False, spin, lambda **kwargs: None, feedback=feedback)
    assert sent == [False, False]


def test_release_does_not_report_success_with_mux_still_locked(monkeypatch):
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', SimpleNamespace(Bool=SimpleNamespace))
    clock = [0.0]
    monkeypatch.setattr(command.time, 'monotonic', lambda: clock[0])
    node = SimpleNamespace(get_subscriptions_info_by_topic=lambda topic: [
        SimpleNamespace(node_name='roboteq_ros2_driver'), SimpleNamespace(node_name='twist_mux')])
    sent, feedback = [], {}
    publisher = SimpleNamespace(get_subscription_count=lambda: 2,
        publish=lambda msg: sent.append(msg.data), wait_for_all_acked=lambda duration: True)
    def spin(*args, **kwargs):
        clock[0] += .1
        feedback.update(active=False, mux_active=True)
    with pytest.raises(RuntimeError, match='twist_mux'):
        command.publish_stop(node, publisher, False, spin, lambda **kwargs: None, feedback=feedback)
    assert sent and not any(sent)
