"""TF-only launches must not appear as, or be stopped through, RViz aliases."""
import importlib
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@pytest.fixture
def web(monkeypatch):
    module = importlib.import_module('web_server')
    monkeypatch.setattr(module, 'BUTTONS', {
        'rviz2sim': 'src && ros2 launch sirius_description display.launch.py use_sim_time:=true',
        'rviz2real': 'src && ros2 launch sirius_description display.launch.py use_sim_time:=false',
        'tf_real': 'src && ros2 launch sirius_description display.launch.py use_sim_time:=false rviz:=false',
        'tf_sim': 'src && ros2 launch sirius_description display.launch.py use_sim_time:=true rviz:=false use_gui:=false',
    })
    monkeypatch.setattr(module, '_SIG_CACHE', {})
    monkeypatch.setattr(module, '_VAR_CACHE', {})
    return module


@pytest.mark.parametrize('arguments, expected', [
    ('use_sim_time:=false rviz:=false', 'tf_real'),
    ('rviz:=false use_sim_time:=false', 'tf_real'),
    ('use_sim_time:=true', 'rviz2sim'),
    ('use_sim_time:=false', 'rviz2real'),
    ('use_gui:=false rviz:=false use_sim_time:=true', 'tf_sim'),
    ('rviz:=false use_sim_time:=true', ''),
])
def test_display_launch_identity_includes_parameters(web, arguments, expected):
    command = '/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch sirius_description display.launch.py '+arguments
    assert web.best_alias_for(command) == expected


def test_tf_process_is_reported_only_as_tf(web, monkeypatch):
    monkeypatch.setattr(web, 'proc_rows', lambda: [{'pid': 123, 'cmd':
        'ros2 launch sirius_description display.launch.py use_sim_time:=false rviz:=false'}])
    assert web.external_matches() == {'tf_real': [123]}


def test_stopping_rviz_does_not_stop_tf(web, monkeypatch, tmp_path):
    manager = SimpleNamespace(is_running=lambda: False, pid_file=str(tmp_path/'unused.pid'))
    monkeypatch.setattr(web, 'get_manager', lambda name: manager)
    monkeypatch.setattr(web, 'proc_rows', lambda: [{'pid': 123, 'cmd':
        'ros2 launch sirius_description display.launch.py use_sim_time:=false rviz:=false'}])
    killed = []
    monkeypatch.setattr(web, '_kill_pid_tree', lambda pid: killed.append(pid))
    assert web.stop_one('rviz2sim') == (False, 'not started')
    assert web.stop_one('rviz2real') == (False, 'not started')
    assert killed == []
    assert web.stop_one('tf_real') == (True, 'stopped (external)')
    assert killed == [123]
