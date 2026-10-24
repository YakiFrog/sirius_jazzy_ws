import os
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_autostart_is_user_scoped_and_removal_keeps_other_entries(tmp_path):
    workspace = tmp_path/'sirius_jazzy_ws'
    starter = workspace/'bash/startup_bash/start_sirius_console.sh'
    starter.parent.mkdir(parents=True); starter.write_text('#!/bin/bash\n')
    config = tmp_path/'config'; autostart = config/'autostart'
    autostart.mkdir(parents=True)
    other = autostart/'other.desktop'; other.write_text('keep me')
    env = dict(os.environ, HOME=str(tmp_path), XDG_CONFIG_HOME=str(config), ROS_DOMAIN_ID='53')
    script = ROOT/'bash/startup_bash/install_sirius_console_autostart.sh'
    subprocess.run(['bash',str(script)],env=env,check=True,capture_output=True)
    target = autostart/'sirius-console.desktop'
    assert 'SIRIUS_WEB_KIOSK=1' in target.read_text()
    assert str(starter) in target.read_text()
    assert 'ROS_DOMAIN_ID=53' in target.read_text()
    subprocess.run(['bash',str(script),'--remove'],env=env,check=True,capture_output=True)
    assert not target.exists() and other.read_text()=='keep me'
