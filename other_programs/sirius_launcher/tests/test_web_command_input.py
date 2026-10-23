"""Exercise real PTY input, reconnects, interrupts and authenticated Web routes."""
import importlib
import os
from pathlib import Path
import shlex
import signal
import sys
import time
import uuid

import pytest

LAUNCHER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LAUNCHER))
from command_session import session_request, socket_path
from process_manager import ProcessManager


def wait_for(check, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if check():
            return
        time.sleep(.03)
    raise AssertionError('terminal did not reach the expected state')


@pytest.fixture
def command(tmp_path, monkeypatch):
    # Keep tests independent of the user's interactive .bashrc.
    monkeypatch.setenv('HOME', str(tmp_path))
    name = 'test_input_' + uuid.uuid4().hex
    script = tmp_path / 'prompt.py'
    script.write_text(
        'import os,sys\n'
        'assert sys.stdin.isatty() and sys.stdout.isatty()\n'
        'print("DOMAIN="+os.environ["ROS_DOMAIN_ID"],flush=True)\n'
        'try:\n'
        '    first=input("地図番号: ")\n'
        '    print("選択="+first,flush=True)\n'
        '    second=input("Enterで既定値: ")\n'
        '    print("既定値="+repr(second),flush=True)\n'
        '    input("割り込み待機: ")\n'
        'except KeyboardInterrupt:\n'
        '    print("INTERRUPTED",flush=True)\n')
    manager = ProcessManager(name, f'{shlex.quote(sys.executable)} {shlex.quote(str(script))}')
    assert manager.launch_web('224')

    def log():
        path = Path(manager.log_file)
        return path.read_text(errors='replace') if path.exists() else ''

    try:
        wait_for(lambda: '地図番号:' in log())
        yield name, manager, log
    finally:
        if manager.process.poll() is None:
            manager.process.send_signal(signal.SIGTERM)
        try:
            manager.process.wait(timeout=5)
        finally:
            if manager.process.poll() is None:
                manager.process.kill()
                manager.process.wait(timeout=3)
        for path in (manager.log_file, manager.pid_file, socket_path(name), manager.script_path):
            Path(path).unlink(missing_ok=True)


def test_terminal_input_reconnect_and_ctrl_c(command):
    name, manager, log = command
    assert 'DOMAIN=224' in log()
    wait_for(lambda: session_request(name).get('input_wait', {}).get('confirmed'))
    assert session_request(name)['input_wait']['interactive']
    assert session_request(name, '地図 2\n')['ok']
    wait_for(lambda: '選択=地図 2' in log())
    # A new process manager has no in-memory session; use the persisted endpoint.
    reconnected = ProcessManager(name, '')
    assert reconnected.is_running()
    assert reconnected.pid_file_content == manager.process.pid
    assert session_request(name, '\n')['ok']
    wait_for(lambda: "既定値=''" in log())
    assert session_request(name, '\x03')['ok']
    manager.process.wait(timeout=8)
    assert 'INTERRUPTED' in log()
    assert not reconnected.is_running()
    assert not session_request(name, 'after exit\n')['ok']


def test_web_input_auth_limits_and_log(command, monkeypatch):
    name, manager, log = command
    web = importlib.import_module('web_server')
    monkeypatch.setattr(web, 'PASSWORD', 'test-password')
    monkeypatch.setattr(web, 'BUTTONS', {name: manager.command})
    client = web.app.test_client()
    assert client.post('/api/input', json={'name': name, 'data': 'blocked\n'}).status_code == 401
    client.set_cookie('pw', 'test-password')
    result = client.get('/api/log', query_string={'name': name})
    assert result.json['interactive'] and '地図番号:' in result.json['log']
    assert client.post('/api/input', json={'name': '../invalid', 'data': '1\n'}).status_code == 404
    assert client.post('/api/input', json={'name': name, 'data': 'あ'*1400}).status_code == 400
    assert client.post('/api/input', json={'name': name, 'data': '2\n'}).json['ok']
    wait_for(lambda: '選択=2' in log())
    assert client.post('/api/input', json={'name': name, 'data': '\n'}).json['ok']
    wait_for(lambda: '割り込み待機:' in log())
    assert client.post('/api/input', json={'name': name, 'data': '\x03'}).json['ok']
    manager.process.wait(timeout=8)
    assert client.get('/api/log', query_string={'name': name}).json['interactive'] is False
    assert client.post('/api/input', json={'name': name, 'data': '\n'}).status_code == 409
