"""Mode commands must load the overlay even when the ROS CLI already exists."""
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest


@pytest.mark.parametrize('returncode', [0, 7])
def test_existing_ros_cli_still_loads_workspace_and_preserves_result(tmp_path, returncode):
    scripts = tmp_path/'workspace/bash/startup_bash'
    scripts.mkdir(parents=True)
    shutil.copy2(Path(__file__).resolve().parents[1]/'change_nav_mode.sh', scripts)
    install = tmp_path/'workspace/install'
    install.mkdir()
    (install/'setup.bash').write_text('export SIRIUS_TEST_OVERLAY=loaded\n')
    tools = tmp_path/'bin'
    tools.mkdir()
    ros2 = tools/'ros2'
    ros2.write_text('#!/bin/sh\necho /controller_server\n')
    ros2.chmod(0o755)
    (scripts/'change_nav_mode_fast.py').write_text(
        'import os,sys,json\n'
        'print(json.dumps({"overlay":os.environ.get("SIRIUS_TEST_OVERLAY"),'
        '"domain":os.environ.get("ROS_DOMAIN_ID"),"mode":sys.argv[1]}))\n'
        f'raise SystemExit({returncode})\n')
    env = {**os.environ, 'PATH':str(tools)+os.pathsep+os.environ['PATH'],
           'ROS_DOMAIN_ID':'231'}
    env.pop('SIRIUS_TEST_OVERLAY', None)
    result = subprocess.run(['bash', str(scripts/'change_nav_mode.sh'), 'wait_normal'],
                            env=env, text=True, capture_output=True, timeout=10)
    payload = next(json.loads(line) for line in result.stdout.splitlines() if line.startswith('{'))
    assert payload == {'overlay':'loaded', 'domain':'231', 'mode':'wait_normal'}
    assert result.returncode == returncode
