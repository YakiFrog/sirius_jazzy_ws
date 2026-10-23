"""Lost PID files, session detection, and difference-only preset starts."""
import importlib
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@pytest.fixture
def web(monkeypatch):
    module = importlib.import_module('web_server')
    monkeypatch.setattr(module, 'BUTTONS', {
        'roboteq': 'bash /tmp/start_roboteq.sh false',
        'imu': 'ros2 launch sirius_navigation witmotion_hwt905.launch.py',
    })
    monkeypatch.setattr(module, '_SIG_CACHE', {})
    monkeypatch.setattr(module, '_VAR_CACHE', {})
    monkeypatch.setattr(module, '_SCRIPT_SIG_CACHE', None)
    monkeypatch.setattr(module, 'PASSWORD', '')
    return module


def test_managed_sessions_finds_lost_pid_file(web, monkeypatch):
    now = time.time()
    proc = SimpleNamespace(info={
        'pid': 4321,
        'cmdline': ['python3', '/x/command_session.py', '--name', 'roboteq',
                    '--script', '/tmp/launch.sh'],
        'create_time': now - 12,
    })
    monkeypatch.setattr(web.psutil, 'process_iter', lambda attrs: [proc])
    sessions = web.managed_sessions()
    assert list(sessions) == ['roboteq']
    assert sessions['roboteq'][0]['pid'] == 4321
    assert sessions['roboteq'][0]['age'] >= 11


def test_start_skips_a_live_session_without_pid_file(web, monkeypatch, tmp_path):
    manager = SimpleNamespace(
        is_running=lambda: False,
        pid_file=str(tmp_path / 'missing.pid'),
        pid_file_content=None,
    )
    launched = []
    manager.launch_web = lambda domain: launched.append(domain) or True
    monkeypatch.setattr(web, 'get_manager', lambda name: manager)
    monkeypatch.setattr(web, 'managed_sessions',
                        lambda: {'roboteq': [{'pid': 4321, 'age': 12}]})
    monkeypatch.setattr(web, 'proc_rows', lambda: [])
    assert web.start_one('roboteq') == (True, 'already running (session pid 4321)')
    assert launched == []


def test_stop_kills_a_live_session_without_pid_file(web, monkeypatch, tmp_path):
    manager = SimpleNamespace(
        is_running=lambda: False,
        pid_file=str(tmp_path / 'missing.pid'),
        pid_file_content=None,
    )
    killed = []
    monkeypatch.setattr(web, 'get_manager', lambda name: manager)
    monkeypatch.setattr(web, 'managed_sessions',
                        lambda: {'roboteq': [{'pid': 4321, 'age': 12}]})
    monkeypatch.setattr(web, '_kill_pid_tree', lambda pid: killed.append(pid) or True)
    monkeypatch.setattr(web, 'proc_rows', lambda: [])
    assert web.stop_one('roboteq') == (True, 'stopped (session)')
    assert killed == [4321]


def test_script_launch_target_identifies_external_wrapper(web, monkeypatch, tmp_path):
    script = tmp_path / 'start_roboteq.sh'
    script.write_text(
        '#!/usr/bin/env bash\n'
        'exec ros2 launch roboteq_ros2_driver roboteq_ros2_driver.launch.py '
        'pub_odom_tf:=false\n',
        encoding='utf-8')
    monkeypatch.setattr(web, 'BUTTONS', {'roboteq': f'bash {script} false'})
    monkeypatch.setattr(web, '_SIG_CACHE', {})
    monkeypatch.setattr(web, '_VAR_CACHE', {})
    monkeypatch.setattr(web, '_SCRIPT_SIG_CACHE', None)
    assert web.best_alias_for(
        '/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch '
        'roboteq_ros2_driver roboteq_ros2_driver.launch.py pub_odom_tf:=false') == 'roboteq'


def test_preset_starts_only_missing_items(web, monkeypatch):
    calls = []
    def fake_start(name, domain):
        calls.append(name)
        return (True, 'already running' if name == 'roboteq' else 'started')
    monkeypatch.setattr(web, 'PRESETS', [{'name': 'mix', 'items':
                                          ['roboteq', 'imu', 'roboteq', 'imu']}])
    monkeypatch.setattr(web, 'start_one', fake_start)
    monkeypatch.setattr(web.time, 'sleep', lambda seconds: None)
    client = web.app.test_client()
    response = client.post('/api/preset', json={'name': 'mix', 'domain': '42'})
    assert response.status_code == 200
    assert response.json == {'started': ['imu'], 'skipped': ['roboteq', 'roboteq', 'imu'],
                             'failed': []}
    assert calls == ['roboteq', 'imu']


def test_stop_all_includes_unlisted_sessions(web, monkeypatch):
    stopped = []
    monkeypatch.setattr(web, 'MANAGERS', {})
    monkeypatch.setattr(web, 'scan_pid_files', lambda: set())
    monkeypatch.setattr(web, 'managed_sessions',
                        lambda: {'roboteq': [{'pid': 4321, 'age': 12}]})
    monkeypatch.setattr(web, 'external_matches', lambda: {})
    monkeypatch.setattr(web, 'stop_one', lambda name: stopped.append(name) or (True, 'stopped'))
    client = web.app.test_client()
    assert client.post('/api/stop_all', json={}).status_code == 200
    assert stopped == ['roboteq']


def test_external_matches_skips_session_children(web, monkeypatch):
    monkeypatch.setattr(web, 'proc_rows', lambda: [
        {'pid': 4322, 'cmd': 'ros2 launch roboteq_ros2_driver roboteq_ros2_driver.launch.py'},
        {'pid': 555, 'cmd': 'ros2 launch sirius_navigation witmotion_hwt905.launch.py'},
    ])
    monkeypatch.setattr(web, 'session_owner_map',
                        lambda sessions=None: {4321: 'roboteq', 4322: 'roboteq'})
    assert web.external_matches() == {'imu': [555]}
