import importlib
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)) if str(ROOT) not in sys.path else None
from teleop_control import VelocityControl

OWNER = 'a'*32
OTHER = 'b'*32


def control():
    model = VelocityControl()
    assert model.command('arm', OWNER, 1, 10.) == ''
    return model


def test_scn_steps_and_axis_independent_resets():
    m = control()
    for seq, key in enumerate((65, 65, 68, 44), 2):
        assert not m.command('key', OWNER, seq, 10., key)
    assert m.target_linear == 0 and m.target_angular == .157
    assert not m.command('key', OWNER, 6, 10., 66)
    assert not m.command('key', OWNER, 7, 10., 46)
    assert m.target_linear == -.1 and m.target_angular == 0
    assert not m.command('key', OWNER, 8, 10., 67)
    assert m.target_angular == -.157


def test_limits_do_not_overshoot_and_scn_acceleration_is_applied():
    m = control()
    for seq in range(2, 50):
        m.command('key', OWNER, seq, 10., 65)
    assert m.target_linear == 2.
    m.tick(10.1, .1)
    assert m.linear == pytest.approx(.08)
    for seq in range(50, 100):
        m.command('key', OWNER, seq, 10.1, 68)
    assert m.target_angular == 3.14
    m.tick(10.2, .1)
    assert m.angular == pytest.approx(.1)


def test_timeout_stops_immediately_and_late_inputs_cannot_resume_motion():
    m = control()
    m.command('key', OWNER, 2, 10., 65)
    m.tick(10.5, .1)
    assert m.linear > 0
    m.tick(11.01, .1)
    assert not m.owner and m.linear == m.target_linear == 0
    assert m.command('heartbeat', OWNER, 3, 11.1)
    assert m.command('key', OWNER, 4, 11.2, 65)
    assert m.linear == m.target_linear == 0


def test_other_tab_and_old_commands_cannot_change_current_owner():
    m = control()
    assert m.command('arm', OTHER, 1, 10.)
    assert m.command('key', OTHER, 2, 10., 65)
    assert m.target_linear == 0
    m.command('key', OWNER, 4, 10., 65)
    assert m.command('key', OWNER, 3, 10., 65)
    assert m.target_linear == .1
    m.command('stop', OTHER, 3, 10.)
    assert not m.owner and m.target_linear == 0
    assert m.command('key', OWNER, 5, 10., 65)


def test_own_normal_stop_zeros_immediately_and_keeps_session_ready():
    m=control()
    m.command('key',OWNER,2,10.,65)
    m.command('key',OWNER,3,10.,68)
    m.tick(10.1,.1)
    assert m.linear>0 and m.angular>0
    assert not m.command('stop',OWNER,4,10.2)
    assert m.owner==OWNER and m.snapshot()['enabled']
    assert m.linear==m.angular==m.target_linear==m.target_angular==0
    assert not m.command('key',OWNER,5,10.3,65)
    assert m.target_linear==.1 and m.target_angular==0
    m.tick(11.31,.1)
    assert not m.owner and m.linear==0  # A stop does not remove the timeout.


def test_axis_resets_keep_owner_and_other_axis():
    m=control()
    m.command('key',OWNER,2,10.,65)
    m.command('key',OWNER,3,10.,68)
    assert not m.command('key',OWNER,4,10.,44)
    assert m.owner==OWNER and m.target_linear==0 and m.target_angular==.157
    assert not m.command('key',OWNER,5,10.,46)
    assert m.owner==OWNER and m.target_linear==m.target_angular==0


def test_stop_cannot_enable_inactive_control_or_ignore_an_interlock():
    m=VelocityControl()
    assert not m.command('stop',OWNER,1,10.)
    assert not m.owner
    assert not m.command('arm',OWNER,2,10.)
    assert not m.command('stop',OWNER,3,10.1,blocked='電子緊急停止がONです')
    assert not m.owner and m.target_linear==m.target_angular==0


def test_emergency_or_missing_telemetry_blocks_arm_and_resets_targets():
    m = VelocityControl()
    assert m.command('arm', OWNER, 1, 10., blocked='電子緊急停止がONです')
    assert not m.owner
    m.command('arm', OWNER, 2, 10.)
    m.command('key', OWNER, 3, 10., 65)
    m.tick(10.1, .1, blocked='状態が途絶しました')
    assert not m.owner and m.linear == m.target_linear == 0
    m.tick(10.2, .1)
    assert not m.owner


def test_web_validation_auth_and_acknowledged_state(monkeypatch):
    web = importlib.import_module('web_server')
    class FakeBridge:
        def __init__(self):
            self.model = VelocityControl()
            self.calls = 0
        def command(self, data, domain):
            self.calls += 1
            error = self.model.command(data['action'], data['session'], data['seq'], 10., data.get('key'))
            return {**self.model.snapshot(), 'ok': not error, 'error': error}
        def status(self, session=None):
            return {**self.model.snapshot(), 'owned': self.model.owner == session,
                    'topic': '/cmd_vel', 'alive': True}
    fake = FakeBridge()
    monkeypatch.setattr(web, 'TELEOP', fake)
    monkeypatch.setattr(web, 'PASSWORD', '')
    client = web.app.test_client()
    for payload in ({'key': 65}, {'action':'arm','session':OWNER,'seq':True},
                    {'action':'key','session':OWNER,'seq':1,'key':True},
                    {'action':'arm','session':OWNER,'seq':1,'domain':'53;bad'}):
        assert client.post('/api/teleop', json=payload).status_code == 400
    assert fake.calls == 0
    response = client.post('/api/teleop', json={'action':'arm','session':OWNER,'seq':1})
    assert response.status_code == 200 and response.json['owned']
    assert client.post('/api/teleop', json={'action':'key','session':OTHER,'seq':2,'key':65}).status_code == 409
    assert fake.model.target_linear == 0
    monkeypatch.setattr(web, 'PASSWORD', 'test-password')
    assert client.post('/api/teleop', json={'action':'arm','session':OWNER,'seq':3}).status_code == 401


@pytest.mark.parametrize('mode,outputs,inputs,topic,blocked', [
    ('mux', set(), set(), '/cmd_vel_web', True),
    ('mux', {'twist_mux'}, set(), '/cmd_vel_web', True),
    ('mux', set(), {'twist_mux'}, '/cmd_vel_web', True),
    ('mux', {'twist_mux'}, {'twist_mux'}, '/cmd_vel_web', False),
    ('auto', set(), set(), '/cmd_vel', False),
    ('auto', {'twist_mux'}, {'twist_mux'}, '/cmd_vel_teleop', False),
])
def test_mux_only_never_falls_back_to_direct_output(mode, outputs, inputs, topic, blocked):
    from teleop_bridge import velocity_route
    actual_topic, reason = velocity_route(mode, outputs, inputs)
    assert actual_topic == topic
    assert bool(reason) == blocked


def test_web_mux_mode_forwarding_and_validation(monkeypatch):
    web = importlib.import_module('web_server')
    calls = []
    class Bridge:
        def command(self, data, domain):
            calls.append(data)
            return {'ok': True}
        def status(self, session=None):
            return {}
    monkeypatch.setattr(web, 'TELEOP', Bridge())
    monkeypatch.setattr(web, 'PASSWORD', '')
    client = web.app.test_client()
    request = dict(action='arm', session=OWNER, seq=1, route_mode='mux')
    assert client.post('/api/teleop', json=request).status_code == 200
    assert calls[-1]['route_mode'] == 'mux'
    for invalid in ('direct', None, [], 1):
        assert client.post('/api/teleop', json={**request, 'route_mode': invalid}).status_code == 400
    assert client.post('/api/teleop', json={**request, 'action': 'heartbeat'}).status_code == 400
    assert len(calls) == 1
