"""Launcher integration for the user's desktop SiriusFace shortcut."""
import configparser
import json
import os
from pathlib import Path
import shlex
import shutil
import socket
import subprocess
import threading
import time

from flask import Blueprint, jsonify, request

EXPRESSIONS = {'normal', 'happy', 'angry', 'sad', 'surprised', 'cat', 'wink', 'pien',
               'sleeping', 'battery', 'failure', 'hurt'}
GAZE = {'center': (.5, .5), 'left': (.15, .5), 'right': (.85, .5),
        'up': (.5, .15), 'down': (.5, .85)}


class FaceError(Exception):
    pass


class FaceBridge:
    def __init__(self, home=None):
        self.home = Path(home or Path.home())
        self.root = Path(os.environ.get('SIRIUS_FACE_HOME') or self.home/'sirius_face_anim2')
        self.lock = threading.RLock()
        self.cached = None
        self.cached_at = 0
        self.launch_until = 0
        self.gui = None

    def shortcut(self):
        explicit = os.environ.get('SIRIUS_FACE_SHORTCUT')
        paths = [Path(explicit)] if explicit else [
            self.home/'Desktop/sirius_face.desktop', self.home/'デスクトップ/sirius_face.desktop',
            self.home/'.local/share/applications/sirius_face.desktop']
        for path in paths:
            if not path.is_file():
                continue
            config = configparser.ConfigParser(interpolation=None)
            try:
                config.read(path)
            except (OSError, configparser.Error):
                continue
            if 'Desktop Entry' not in config:
                continue
            entry = config['Desktop Entry']
            try:
                args = shlex.split(entry.get('Exec', ''))
            except ValueError:
                continue
            if entry.get('Type') == 'Application' and args:
                return dict(path=path, executable=args[0], name=entry.get('Name', 'Sirius Face UI'),
                            wm_class=entry.get('StartupWMClass', 'sirius-face-2').lower())
        return None

    def gui_env(self):
        if self.gui is not None:
            return self.gui
        env = dict(os.environ)
        if not env.get('DISPLAY'):
            try:
                result = subprocess.run(['pgrep', '-u', str(os.getuid()), '-x', 'gnome-shell'],
                                        capture_output=True, text=True, timeout=.5)
                for pid in result.stdout.split():
                    for item in Path('/proc', pid, 'environ').read_bytes().split(b'\0'):
                        key, _, value = item.partition(b'=')
                        if key.decode() in {'DISPLAY', 'XAUTHORITY', 'XDG_RUNTIME_DIR',
                                            'DBUS_SESSION_BUS_ADDRESS', 'WAYLAND_DISPLAY'}:
                            env[key.decode()] = value.decode()
                    break
            except (OSError, subprocess.TimeoutExpired):
                pass
        self.gui = env
        return env

    def windows(self, shortcut):
        if not shortcut or not shutil.which('wmctrl'):
            return []
        try:
            result = subprocess.run(['wmctrl', '-lpGx'], env=self.gui_env(), capture_output=True,
                                    text=True, timeout=.6)
            windows = []
            for line in result.stdout.splitlines():
                fields = line.split(None, 9)
                if len(fields) >= 9 and shortcut['wm_class'] in fields[7].lower().split('.'):
                    windows.append(dict(id=fields[0], pid=int(fields[2]),
                                        width=int(fields[5]), height=int(fields[6])))
            return windows
        except (OSError, ValueError, subprocess.TimeoutExpired):
            return []

    def processes(self, shortcut):
        if not shortcut:
            return []
        try:
            result = subprocess.run(['ps', '-u', str(os.getuid()), '-o', 'pid=,args='],
                                    capture_output=True, text=True, timeout=.6)
            found = []
            for line in result.stdout.splitlines():
                fields = line.strip().split(None, 1)
                if len(fields) == 2:
                    try:
                        if shortcut['executable'] in shlex.split(fields[1]):
                            found.append(int(fields[0]))
                    except ValueError:
                        continue
            return found
        except (OSError, ValueError, subprocess.TimeoutExpired):
            return []

    def launcher_windows(self):
        """Identify our dedicated browser, including Chrome's space-joined argv."""
        if not shutil.which('wmctrl'):
            return []
        profile = str(self.home/'.local/share/sirius/console-browser')
        try:
            result = subprocess.run(['wmctrl', '-lp'], env=self.gui_env(), capture_output=True,
                                    text=True, timeout=1)
        except (OSError, subprocess.TimeoutExpired):
            return []
        found = []
        for line in result.stdout.splitlines():
            fields = line.split(None, 4)
            if len(fields) < 4:
                continue
            try:
                cmd = ' '+Path('/proc', fields[2], 'cmdline').read_bytes().replace(b'\0', b' ').decode()+' '
            except (OSError, UnicodeError):
                continue
            if ' --user-data-dir='+profile+' ' in cmd and ' --app=' in cmd:
                found.append(dict(id=fields[0], pid=int(fields[2])))
        return found

    def focus_screen(self, target):
        windows = self.windows(self.shortcut()) if target == 'face' else self.launcher_windows()
        label = '顔' if target == 'face' else 'Launcher'
        if not windows:
            raise FaceError(label+'のウィンドウが見つかりません')
        result = subprocess.run(['wmctrl', '-i', '-a', windows[0]['id']], env=self.gui_env(),
                                capture_output=True, timeout=1)
        if result.returncode:
            raise FaceError(label+'の画面を表示できませんでした')
        return dict(ok=True, screen=target)

    @staticmethod
    def listening(port):
        try:
            with socket.create_connection(('127.0.0.1', port), timeout=.15):
                return True
        except OSError:
            return False

    def rpc(self, payload):
        python = self.root/'venv/bin/python'
        if not python.exists() or not (self.root/'scripts/stubs/face_control_pb2_grpc.py').exists():
            raise FaceError('SiriusFaceの通信環境が見つかりません')
        try:
            env = {key: value for key, value in os.environ.items() if key not in {'PYTHONPATH', 'PYTHONHOME'}}
            result = subprocess.run([str(python), str(Path(__file__).with_name('face_worker.py')), str(self.root)],
                                    input=json.dumps(payload), capture_output=True, text=True, timeout=4, env=env)
            data = json.loads(result.stdout.splitlines()[-1])
        except (OSError, ValueError, IndexError, subprocess.TimeoutExpired) as error:
            raise FaceError('SiriusFaceの応答を確認できません') from error
        if data.get('error'):
            raise FaceError(data['error'])
        return data

    def status(self, force=False):
        with self.lock:
            now = time.monotonic()
            if not force and self.cached is not None and now-self.cached_at < 2:
                return dict(self.cached)
            shortcut = self.shortcut()
            windows = self.windows(shortcut)
            pids = self.processes(shortcut)
            state = dict(connected=False, running=bool(pids or windows), window_visible=bool(windows),
                         starting=now < self.launch_until, shortcut_available=bool(shortcut),
                         shortcut_name=shortcut['name'] if shortcut else 'Sirius Face UI',
                         speech_ready=False, expression=None, error=None,
                         launcher_visible=bool(self.launcher_windows()),
                         switch_shortcuts=(self.home/'.config/autostart/sirius-screen-switch.desktop').is_file())
            if self.listening(50051):
                try:
                    state.update(self.rpc({'action': 'status'}))
                    state.update(running=True, starting=False)
                    self.launch_until = 0
                except FaceError as error:
                    state['error'] = str(error)
            state['speech_ready'] = state['connected'] and bool(state.get('speech_ready'))
            self.cached, self.cached_at = state, time.monotonic()
            return dict(state)

    def action(self, data):
        if not isinstance(data, dict):
            raise ValueError('操作内容が不正です')
        action = data.get('action')
        if action not in {'launch', 'focus', 'focus_launcher', 'expression', 'gaze', 'subtitle', 'clear_text', 'speak', 'instruction'}:
            raise ValueError('対応していないFace操作です')
        payload = {'action': action}
        if action == 'expression':
            if data.get('expression') not in EXPRESSIONS:
                raise ValueError('表情を一覧から選んでください')
            payload['expression'] = data['expression']
        if action in {'subtitle', 'speak'}:
            text = data.get('text')
            if not isinstance(text, str) or not text.strip() or len(text.strip()) > 140:
                raise ValueError('表示・読み上げる文字を1〜140文字で入力してください')
            if action == 'speak' and ('[' in text or ']' in text):
                raise ValueError('読み上げには制御タグを使えません')
            payload['text'] = text.strip()
        if action == 'instruction':
            text = data.get('text')
            mode = data.get('mode')
            if mode not in {'llm', 'nav'}:
                raise ValueError('行動指示の方式を選んでください')
            if not isinstance(text, str) or not text.strip() or len(text.strip()) > 200:
                raise ValueError('行動指示を1〜200文字で入力してください')
            if '[' in text or ']' in text:
                raise ValueError('行動指示には制御タグを使えません')
            payload.update(mode=mode, text=text.strip())
        if action == 'gaze' and data.get('direction') not in GAZE:
            raise ValueError('視線の方向を選んでください')
        # Window switching must work even when Face's API is unavailable.
        if action in {'focus', 'focus_launcher'}:
            return self.focus_screen('face' if action == 'focus' else 'launcher')
        with self.lock:
            state = self.status()
            if action == 'launch':
                if state['connected'] or state['running'] or state['starting']:
                    return dict(ok=True, already_running=True)
                shortcut = self.shortcut()
                if not shortcut or not shutil.which('gio'):
                    raise FaceError('デスクトップのSirius Face UIショートカットが見つかりません')
                env = self.gui_env()
                if not env.get('DISPLAY') and not env.get('WAYLAND_DISPLAY'):
                    raise FaceError('NUCのデスクトップに接続できません')
                subprocess.Popen(['gio', 'launch', str(shortcut['path'])], env=env,
                                 stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL, start_new_session=True)
                self.launch_until = time.monotonic()+30
                self.cached = None
                return dict(ok=True, starting=True)
            if not state['connected']:
                raise FaceError('SiriusFaceへ接続できません。顔の起動状態を確認してください')
            if action == 'instruction' and not state.get('speech_ready'):
                raise FaceError('SiriusFaceの指示APIへ接続できません。音声APIの状態を確認してください')
            if action == 'gaze':
                windows = self.windows(self.shortcut())
                if not windows:
                    raise FaceError('視線の基準となる顔ウィンドウが見つかりません')
                x, y = GAZE[data['direction']]
                payload.update(x=windows[0]['width']*x, y=windows[0]['height']*y)
            result = self.rpc(payload)
            self.cached = None
            return result


def register_face(app, auth_ok, bridge=None):
    bridge = bridge or FaceBridge()
    bp = Blueprint('face_control', __name__)

    @bp.get('/api/face/status')
    def status():
        if not auth_ok():
            return jsonify(error='auth'), 401
        response = jsonify(bridge.status())
        response.headers['Cache-Control'] = 'no-store'
        return response

    @bp.post('/api/face/action')
    def action():
        if not auth_ok():
            return jsonify(error='auth'), 401
        try:
            return jsonify(bridge.action(request.get_json(silent=True)))
        except ValueError as error:
            return jsonify(error=str(error)), 400
        except (FaceError, OSError, subprocess.SubprocessError) as error:
            return jsonify(error=str(error)), 503

    app.register_blueprint(bp)
