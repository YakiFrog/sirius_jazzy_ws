import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from process_health import LogHealth, battery_health

NOW = 1792575000.0


def write(path, text, modified=NOW):
    path.write_text(text)
    os.utime(path, (modified, modified))


def test_old_error_survives_as_history_not_current_error(tmp_path):
    path = tmp_path/'log'
    write(path, f'[ERROR] [{NOW-300}] failed\n[INFO] [{NOW}] alive\n')
    state = LogHealth().snapshot(path, NOW)
    assert state['state'] == 'unknown'
    assert state['error_age_seconds'] == 300
    assert 'failed' in state['last_error']['message']


def test_recent_error_expires_without_claiming_recovery(tmp_path):
    path = tmp_path/'log'
    write(path, f'[ERROR] [{NOW-2}] failed\n')
    monitor = LogHealth()
    assert monitor.snapshot(path, NOW)['state'] == 'error'
    assert monitor.snapshot(path, NOW+40)['state'] == 'unknown'


def test_traceback_without_error_tag_and_reconnect_timestamp(tmp_path):
    path = tmp_path/'log'
    write(path, 'Task exception was never retrieved\nTraceback (most recent call last):\n'
          f'asyncio.exceptions.InvalidStateError: invalid state\n[WARN] [{NOW-200}] reconnecting\n')
    state = LogHealth().snapshot(path, NOW)
    assert state['error_age_seconds'] == 200
    assert 'InvalidStateError' in state['last_error']['message']


def test_new_unstamped_error_and_truncated_log(tmp_path):
    path = tmp_path/'log'
    write(path, '[INFO] running\n')
    monitor = LogHealth()
    assert monitor.snapshot(path, NOW)['state'] == 'ok'
    write(path, '[INFO] running\nRuntimeError: failed\n')
    assert monitor.snapshot(path, NOW)['state'] == 'error'
    write(path, 'started\n', NOW+1)
    assert monitor.snapshot(path, NOW+1)['last_error'] is None


def test_later_unstamped_info_does_not_refresh_error_age(tmp_path):
    path = tmp_path/'log'
    write(path, 'RuntimeError: failed\n')
    monitor = LogHealth()
    assert monitor.snapshot(path, NOW)['state'] == 'error'
    write(path, 'RuntimeError: failed\nworking normally\n', NOW+100)
    assert monitor.snapshot(path, NOW+100)['state'] == 'unknown'
    write(path, path.read_text()+'RuntimeError: failed\n', NOW+101)
    assert monitor.snapshot(path, NOW+101)['state'] == 'error'


def test_child_crash_stays_error_until_restart_is_observed(tmp_path):
    path = tmp_path/'log'
    write(path, f'[ERROR] [{NOW-300}] [ekf_node-1]: process has died [pid 123, exit code 127]\n')
    monitor = LogHealth()
    assert monitor.snapshot(path, NOW)['state'] == 'error'
    write(path, path.read_text()+'[INFO] [ekf_node-1]: process started with pid [456]\n')
    state = monitor.snapshot(path, NOW)
    assert state['state'] == 'unknown'
    assert not state['failed_children']


def test_fresh_battery_does_not_clear_child_crash():
    state = battery_health({**log_error(), 'failed_children': {'gateway': 'died'}}, live(), NOW)
    assert state['state'] == 'error'


def live(age=0.5, **changes):
    return {'available': True, 'timestamp': NOW, 'battery': {'details': {
        'state': 'live', 'present': True, 'age_seconds': age, **changes}}}


def log_error(timestamp=NOW-100):
    return {'state': 'error', 'summary': 'error', 'source': 'log',
            'last_error': {'timestamp': timestamp, 'message': 'InvalidStateError'}}


def test_fresh_connected_solix_data_proves_recovery():
    state = battery_health(log_error(), live(), NOW)
    assert state['state'] == 'ok'
    assert state['source'] == 'solix'
    assert state['last_error']['message'] == 'InvalidStateError'
    assert '復帰' in state['summary']


def test_data_before_error_cannot_clear_it():
    assert battery_health(log_error(NOW-.1), live(age=1), NOW)['state'] == 'error'


def test_stale_and_disconnected_data_are_current_errors():
    for data in (live(age=6), live(state='absent', present=False)):
        assert battery_health({'state': 'ok'}, data, NOW)['state'] == 'error'
    assert battery_health({'state': 'ok'}, live(age=4), NOW+2)['state'] == 'error'


def test_receiver_failure_does_not_claim_device_failure_or_recovery():
    assert battery_health({'state': 'unknown'}, {'available': False}, NOW)['state'] == 'unknown'
    assert battery_health(log_error(), {'available': False}, NOW)['state'] == 'error'


def test_full_gateway_is_not_cleared_by_battery_data(monkeypatch):
    import web_server as web
    monkeypatch.setattr(web.LOG_HEALTH, 'snapshot', lambda path: log_error())
    monkeypatch.setattr(web.TF_MONITOR, 'snapshot', lambda domain: live())
    assert web.program_health('sirius_ble_gateway')['state'] == 'error'
    # Use a timestamp from the live clock for the integration wrapper.
    monkeypatch.setattr(web, 'battery_health', lambda log, data: {'state': 'ok'})
    assert web.has_error('sirius_ble_battery') is False


def test_status_exposes_current_health_and_preserves_history(monkeypatch):
    import web_server as web
    monkeypatch.setattr(web, 'scan_pid_files', lambda: {'sirius_ble_battery'})
    monkeypatch.setattr(web, 'MANAGERS', {})
    monkeypatch.setattr(web, 'running_pid', lambda name: 1234)
    monkeypatch.setattr(web, 'process_uptime', lambda pid: 120)
    monkeypatch.setattr(web, 'managed_sessions', lambda: {})
    monkeypatch.setattr(web, 'external_matches', lambda *args: {})
    monkeypatch.setattr(web, 'program_health', lambda name: {
        'state': 'ok', 'summary': 'バッテリー受信正常（過去エラーから復帰）',
        'last_error': {'message': 'InvalidStateError', 'timestamp': NOW-100}})
    data = web.app.test_client().get('/api/status').get_json()['sirius_ble_battery']
    assert data['running'] is True
    assert data['error'] is False
    assert data['health']['last_error']['message'] == 'InvalidStateError'
