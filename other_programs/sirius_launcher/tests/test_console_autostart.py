import os
import subprocess
import sys
import pytest
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


@pytest.mark.parametrize('rewritten_argv', [False, True])
def test_existing_console_with_changed_title_is_reused_and_fullscreen(tmp_path, rewritten_argv):
    """A page-title change must not open a second console or focus SiriusFace."""
    state=tmp_path/'state';state.mkdir()
    tools=tmp_path/'bin';tools.mkdir()
    calls=tmp_path/'window-calls'
    profile=tmp_path/'.local/share/sirius/console-browser'
    browser_marker=tmp_path/'unexpected-browser'
    if rewritten_argv:
        console=subprocess.Popen(['bash','-c','exec -a "$1" sleep 15','mock-chrome',
            'chrome --app=http://127.0.0.1:8090 --user-data-dir='+str(profile)])
    else:
        console=subprocess.Popen([sys.executable,'-c','import time; time.sleep(15)',
            '--app=http://127.0.0.1:8090','--user-data-dir='+str(profile)])
    # Wait until bash has exec'd the mock Chrome process before inspecting argv.
    if rewritten_argv:
        import time
        for _ in range(100):
            if Path(f'/proc/{console.pid}/cmdline').read_bytes().startswith(b'chrome '):
                break
            time.sleep(.01)
    try:
        scripts={
            'curl':'#!/bin/sh\necho \'{"service":"sirius-launcher"}\'\n',
            'wmctrl':f'''#!/bin/sh
if [ "$1" = "-lp" ]; then
 echo "0x123 0 {console.pid} host SiriusLauncher"
else
 echo "$*" >> "{calls}"
fi
''',
            'google-chrome':f'#!/bin/sh\ntouch "{browser_marker}"\n',
        }
        for name,text in scripts.items():
            path=tools/name;path.write_text(text);path.chmod(0o755)
        env=dict(os.environ,HOME=str(tmp_path),PATH=str(tools)+':'+os.environ['PATH'],
                 XDG_RUNTIME_DIR=str(state),SIRIUS_WEB_KIOSK='1')
        script=ROOT/'bash/startup_bash/start_sirius_console.sh'
        for _ in range(2):
            subprocess.run(['bash',str(script)],env=env,check=True,capture_output=True,timeout=5)
        assert not browser_marker.exists()
        assert calls.read_text().splitlines()==[
            '-i -a 0x123','-i -r 0x123 -b add,fullscreen']*2
    finally:
        console.terminate();console.wait(timeout=5)
