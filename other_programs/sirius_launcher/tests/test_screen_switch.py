import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from face_bridge import FaceBridge, FaceError
import screen_switch


@pytest.mark.parametrize('joined', [False, True])
def test_identifies_only_dedicated_launcher_despite_renamed_title(tmp_path, monkeypatch, joined):
    profile = str(tmp_path/'.local/share/sirius/console-browser')
    command = ['chrome', '--app=http://127.0.0.1:8090', '--user-data-dir='+profile]
    cmdline = (' '.join(command)+'\0').encode() if joined else ('\0'.join(command)+'\0').encode()
    original = Path.read_bytes
    monkeypatch.setattr(Path, 'read_bytes', lambda path: cmdline if str(path)=='/proc/123/cmdline'
                        else b'chrome\0--app=http://other\0--user-data-dir=/other\0'
                        if str(path)=='/proc/456/cmdline' else original(path))
    monkeypatch.setattr('face_bridge.shutil.which', lambda _: '/usr/bin/wmctrl')
    monkeypatch.setattr('face_bridge.subprocess.run', lambda *a, **k: SimpleNamespace(
        stdout='0x01 0 123 host Changed title\n0x02 1 456 host SiriusLauncher\n', returncode=0))
    bridge = FaceBridge(tmp_path)
    assert bridge.launcher_windows() == [dict(id='0x01', pid=123)]


def test_bidirectional_switch_uses_existing_windows_without_face_api(tmp_path, monkeypatch):
    bridge = FaceBridge(tmp_path)
    monkeypatch.setattr(bridge, 'windows', lambda _: [dict(id='0x11')])
    monkeypatch.setattr(bridge, 'launcher_windows', lambda: [dict(id='0x22')])
    monkeypatch.setattr(bridge, 'status', lambda: pytest.fail('Switch must not wait for Face API'))
    monkeypatch.setattr(bridge, 'gui_env', lambda: {})
    calls=[]
    monkeypatch.setattr('face_bridge.subprocess.run', lambda args, **kw:
                        (calls.append(args) or SimpleNamespace(returncode=0)))
    monkeypatch.setattr('face_bridge.subprocess.Popen', lambda *a, **kw: pytest.fail('Duplicate app'))
    assert bridge.action(dict(action='focus'))['screen']=='face'
    assert bridge.action(dict(action='focus_launcher'))['screen']=='launcher'
    assert calls == [['wmctrl', '-i', '-a', '0x11'], ['wmctrl', '-i', '-a', '0x22']]
    monkeypatch.setattr(bridge, 'launcher_windows', lambda: [])
    with pytest.raises(FaceError, match='Launcher'):
        bridge.action(dict(action='focus_launcher'))


def test_toggle_returns_from_face_and_goes_to_face_from_launcher(monkeypatch):
    choices=[]
    bridge=SimpleNamespace(shortcut=lambda: {}, windows=lambda _: [dict(id='0x11')],
                           focus_screen=choices.append)
    monkeypatch.setattr(screen_switch, 'active_window', lambda _: 0x11)
    screen_switch.toggle(bridge)
    monkeypatch.setattr(screen_switch, 'active_window', lambda _: 0x22)
    screen_switch.toggle(bridge)
    assert choices==['launcher', 'face']


def test_install_is_idempotent_and_keeps_other_gnome_shortcuts(tmp_path, monkeypatch):
    import ast
    paths=['/custom/existing/']
    commands=[]
    def run(args, **kw):
        nonlocal paths
        if args[1]=='get':
            return SimpleNamespace(stdout=repr(paths))
        commands.append(args)
        if args[3]=='custom-keybindings':
            paths=ast.literal_eval(args[4])
        return SimpleNamespace(stdout='')
    monkeypatch.setattr(screen_switch.subprocess, 'run', run)
    bridge=SimpleNamespace(home=tmp_path, gui_env=lambda: {})
    screen_switch.install(bridge)
    screen_switch.install(bridge)
    assert len(paths)==3 and paths[0]=='/custom/existing/'
    assert len(set(paths))==3
    assert any(cmd[-1]=='<Control><Alt>f' for cmd in commands)
    assert any(cmd[-1]=='<Control><Alt>w' for cmd in commands)
    assert 'overlay' in (tmp_path/'.config/autostart/sirius-screen-switch.desktop').read_text()
