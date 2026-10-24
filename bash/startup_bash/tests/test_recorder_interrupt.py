"""Ensure a background recorder can finalize despite inherited ignored Ctrl+C."""
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import pytest
import yaml

SUPPORT = Path(__file__).resolve().parents[1]/'navigation_recording.py'


def test_real_rosbag_restores_interrupt_and_writes_metadata(tmp_path):
    if not shutil.which('ros2'):
        pytest.skip('requires sourced ROS environment')
    bag = tmp_path/'interrupt_bag'
    log = tmp_path/'recorder.log'
    with log.open('w') as stream:
        process = subprocess.Popen([
            'bash', '-c', 'trap "" INT; exec "$@"', 'bash', sys.executable, str(SUPPORT),
            '--exec-recorder', '-s', 'mcap', '-o', str(bag), '--topics', '/interrupt_test'],
            env={**os.environ, 'ROS_DOMAIN_ID': '231'}, stdout=stream, stderr=stream)
    try:
        deadline = time.monotonic()+10
        while 'Recording...' not in log.read_text() and time.monotonic()<deadline:
            assert process.poll() is None, log.read_text()
            time.sleep(.05)
        assert 'Recording...' in log.read_text(), log.read_text()
        process.send_signal(signal.SIGINT)
        process.wait(timeout=5)
        metadata = yaml.safe_load((bag/'metadata.yaml').read_text())
        assert metadata['rosbag2_bagfile_information']['storage_identifier']=='mcap'
        assert not list(bag.glob('*.tmp'))
    finally:
        if process.poll() is None:
            process.terminate(); process.wait(timeout=5)


def test_web_record_command_stops_with_one_ctrl_c(tmp_path, monkeypatch):
    if not shutil.which('ros2'):
        pytest.skip('requires sourced ROS environment')
    import uuid
    root = SUPPORT.parents[2]
    sys.path.insert(0, str(root/'other_programs/sirius_launcher'))
    from process_manager import ProcessManager
    from command_session import session_request
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.setenv('SIRIUS_WS_DIR', str(root))
    manager = ProcessManager('test_record_'+uuid.uuid4().hex,
                             'bash '+str(SUPPORT.with_name('record_rosbag.sh')))
    assert manager.launch_web('232')
    log = Path(manager.log_file)
    def until(marker):
        deadline = time.monotonic()+15
        while time.monotonic()<deadline:
            content = log.read_text(errors='replace')
            if marker in content:
                return
            assert manager.process.poll() is None, content
            time.sleep(.05)
        raise AssertionError(content)
    try:
        until('Press [Enter]')
        assert session_request(manager.name, '\n\n\n')['ok']
        until('Recording...')
        start = time.monotonic()
        assert session_request(manager.name, '\x03')['ok']
        manager.process.wait(timeout=12)
        assert time.monotonic()-start<12
        bags = list((tmp_path/'rosbag2_data').glob('*/metadata.yaml'))
        assert len(bags)==1, log.read_text(errors='replace')
        mcap = next(bags[0].parent.glob('*.mcap'))
        with mcap.open('rb') as f:
            f.seek(-8, 2)
            assert f.read()==b'\x89MCAP0\r\n'
        assert '✓ 記録が完了しました' in log.read_text(errors='replace')
        assert 'recover ' not in log.read_text(errors='replace')
    finally:
        if manager.process.poll() is None:
            manager.process.terminate();manager.process.wait(timeout=5)
        for p in (manager.log_file, manager.pid_file):
            Path(p).unlink(missing_ok=True)
