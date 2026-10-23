import importlib
import os
import pty
import signal
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from input_wait import InputWaitMonitor, inspect_terminal_reader


def eventually(check):
    deadline = time.monotonic()+4
    while time.monotonic() < deadline:
        if check():
            return
        time.sleep(.02)
    raise AssertionError('Terminal state did not change')


@pytest.mark.parametrize('command', [
    ['bash', '-c', 'read -p "番号を入力: " answer; sleep 10'],
    [sys.executable, '-c', 'input("番号を入力: "); import time; time.sleep(10)'],
])
def test_real_terminal_read_and_return_to_work(command, tmp_path):
    pid, master = pty.fork()
    if pid == 0:
        os.execvp(command[0], command)
    try:
        eventually(lambda: (inspect_terminal_reader(pid) or {}).get('confirmed'))
        os.write(master, b'1')
        assert inspect_terminal_reader(pid)['state'] == 'waiting'
        os.write(master, b'\n')
        monitor = InputWaitMonitor()
        eventually(lambda: not inspect_terminal_reader(pid))
        assert monitor.snapshot(pid, tmp_path/'absent.log')['state'] == 'none'
    finally:
        os.killpg(pid, signal.SIGTERM)
        os.waitpid(pid, 0)
        os.close(master)


def test_pipe_read_and_old_prompt_do_not_count_as_terminal_input(tmp_path):
    import subprocess
    process = subprocess.Popen(['bash', '-c', 'read answer'], stdin=subprocess.PIPE)
    log = tmp_path/'log'; log.write_text('番号を入力: ')
    try:
        assert InputWaitMonitor().snapshot(process.pid, log)['state'] == 'none'
    finally:
        process.terminate(); process.wait()


def test_cannot_confirm_is_labeled_possible_and_other_fd_is_rejected(tmp_path, monkeypatch):
    module = importlib.import_module('input_wait')
    base = tmp_path/'123'; (base/'fd').mkdir(parents=True)
    (base/'fd/0').symlink_to('/dev/pts/99')
    (base/'fd/3').symlink_to('/tmp/sensor')
    (base/'stat').write_text('123 (bash) S 1 123 123 100 123 0')
    (base/'comm').write_text('bash')
    (base/'wchan').write_text('wait_woken')
    monkeypatch.setattr(module, 'Path', lambda _: tmp_path)
    assert module.inspect_terminal_reader(123) is None
    assert module.inspect_terminal_reader(123, '番号を入力: ')['state'] == 'possible'
    call = next(iter(module.READ_CALLS))
    (base/'syscall').write_text(f'{call} 0x3 0x0 0x1')
    assert module.inspect_terminal_reader(123, '番号を入力: ') is None
    (base/'syscall').write_text(f'{call} 0x0 0x0 0x1')
    assert module.inspect_terminal_reader(123)['confirmed']


def test_status_exposes_wait_independently_of_log_errors(monkeypatch):
    web = importlib.import_module('web_server')
    monkeypatch.setattr(web, 'MANAGERS', {})
    monkeypatch.setattr(web, 'scan_pid_files', lambda: {'demo'})
    monkeypatch.setattr(web, 'running_pid', lambda name: 123)
    monkeypatch.setattr(web, 'managed_sessions', lambda: {'demo': [{'pid': 123}]})
    monkeypatch.setattr(web, 'process_uptime', lambda pid: 5)
    monkeypatch.setattr(web, 'external_matches', lambda *args: {})
    monkeypatch.setattr(web, 'session_request', lambda name: {'ok': True})
    monkeypatch.setattr(web, 'program_health', lambda *args: dict(state='ok', summary='ログ異常なし'))
    monkeypatch.setattr(web.INPUT_WAIT, 'snapshot', lambda *args, **kwargs:
                        dict(state='waiting', confirmed=True, prompt='番号: ', **kwargs))
    state = web.app.test_client().get('/api/status').json['demo']
    assert state['input_waiting'] and state['input_wait']['interactive']
    assert not state['error'] and state['health']['state'] == 'ok'
