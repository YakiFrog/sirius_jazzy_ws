import json
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
    assert 'SIRIUS_WEB_WORKSPACE=1' in target.read_text()
    assert str(starter) in target.read_text()
    assert 'ROS_DOMAIN_ID=53' in target.read_text()
    subprocess.run(['bash',str(script),'--remove'],env=env,check=True,capture_output=True)
    assert not target.exists() and other.read_text()=='keep me'


@pytest.mark.parametrize('rewritten_argv', [False, True])
@pytest.mark.parametrize('legacy', [False, True])
def test_existing_console_with_changed_title_is_reused_and_fullscreen(tmp_path, rewritten_argv, legacy):
    """A page-title change must not open a second console or focus SiriusFace."""
    state=tmp_path/'state';state.mkdir()
    tools=tmp_path/'bin';tools.mkdir()
    calls=tmp_path/'window-calls'
    fullscreen=tmp_path/'fullscreen'
    profile=tmp_path/'.local/share/sirius/console-browser'
    browser_marker=tmp_path/'unexpected-browser'
    url='http://localhost:8090' if legacy else 'http://127.0.0.1:8090'
    options='--app='+url + ('' if legacy else ' --user-data-dir='+str(profile))
    if rewritten_argv:
        console=subprocess.Popen(['bash','-c','exec -a "$1" sleep 15','mock-chrome',
            'chrome '+options])
    else:
        console=subprocess.Popen([sys.executable,'-c','import time; time.sleep(15)',
            *options.split()])
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
if [ "$1" = "-m" ]; then
 exit 0
elif [ "$1" = "-lpx" ]; then
 echo "0x122 0 {console.pid} google-chrome.Google-chrome host Normal Chrome window"
 echo "0x123 0 {console.pid} {'localhost' if legacy else '127.0.0.1'}.Google-chrome host SiriusLauncher"
else
 echo "$*" >> "{calls}"
 if [ "$2" = "-r" ]; then touch "{fullscreen}"; fi
fi
''',
            'google-chrome':f'#!/bin/sh\ntouch "{browser_marker}"\n',
            'xprop':f'#!/bin/sh\nif test -f "{fullscreen}"; then echo _NET_WM_STATE_FULLSCREEN; fi\n',
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
            '-i -a 0x123','-i -r 0x123 -b add,fullscreen','-i -a 0x123']
    finally:
        console.terminate();console.wait(timeout=5)


def test_new_console_uses_direct_app_route_and_waits_for_window_state(tmp_path):
    tools = tmp_path / 'bin'
    tools.mkdir()
    browser_args = tmp_path / 'browser-args'
    fullscreen = tmp_path / 'fullscreen'
    workspace = tmp_path / 'workspace'
    focused = tmp_path / 'focused'
    scripts = {
        'curl': '#!/bin/sh\necho \'{"service":"sirius-launcher"}\'\n',
        'wmctrl': f'''#!/bin/sh
case "$1" in
 -m) exit 0 ;;
 -lpx) if test -f "{browser_args}"; then echo "0x123 0 99999 127.0.0.1__console.SiriusLauncher-8090 host Loading"; fi ;;
 -i) case "$2 $4" in
       '-r -b') touch "{fullscreen}" ;;
       '-r -t') echo "$5" > "{workspace}" ;;
       '-a ') touch "{focused}" ;;
     esac ;;
esac
''',
        'xprop': f'''#!/bin/sh
case "$3" in
 _NET_WM_DESKTOP) echo "_NET_WM_DESKTOP(CARDINAL) = $(cat "{workspace}" 2>/dev/null || echo 0)" ;;
 _NET_WM_STATE) if test -f "{fullscreen}"; then echo _NET_WM_STATE_FULLSCREEN; fi ;;
esac
''',
        'google-chrome': f'#!/bin/sh\nprintf "%s\\n" "$@" > "{browser_args}"\n',
    }
    for name, content in scripts.items():
        path = tools / name
        path.write_text(content)
        path.chmod(0o755)
    env = dict(os.environ, HOME=str(tmp_path), PATH=str(tools)+':'+os.environ['PATH'],
               XDG_RUNTIME_DIR=str(tmp_path), SIRIUS_WEB_PORT='8090', SIRIUS_WEB_KIOSK='1',
               SIRIUS_WEB_WORKSPACE='1')
    script = ROOT/'bash/startup_bash/start_sirius_console.sh'
    subprocess.run(['bash', str(script)], env=env, check=True, capture_output=True, timeout=8)
    args = browser_args.read_text().splitlines()
    assert '--app=http://127.0.0.1:8090/console' in args
    assert '--class=SiriusLauncher-8090' in args
    assert '--password-store=basic' in args
    prefs = json.loads((tmp_path / '.local/share/sirius/console-browser/Default/Preferences').read_text())
    assert prefs['credentials_enable_service'] is False
    assert prefs['profile']['password_manager_enabled'] is False
    assert '--kiosk' not in args
    assert fullscreen.exists()
    assert workspace.read_text().strip() == '1'
    assert not focused.exists()
