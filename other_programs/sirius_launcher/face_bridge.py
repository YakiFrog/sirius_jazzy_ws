"""Launcher integration for the user's desktop SiriusFace shortcut."""
import configparser
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import socket
import subprocess
import tempfile
import threading
import time
from uuid import uuid4

from flask import Blueprint, Response, jsonify, request
from fullscreen_workspace import ensure_fullscreen
from launcher_windows import launcher_windows

EXPRESSIONS = {'normal', 'happy', 'angry', 'sad', 'surprised', 'cat', 'wink', 'pien',
               'sleeping', 'battery', 'failure', 'hurt'}
SPEECH_LIMIT = 140
SPEECH_PRESET_LIMIT = 50
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
        self.cancel_placement = threading.Event()
        self.gui = None
        self.preview_lock = threading.Lock()
        self.preview_cached = None
        self.preview_at = 0
        self.speech_lock = threading.Lock()
        self.speech_presets_path = Path(
            os.environ.get('SIRIUS_FACE_SPEECH_PRESETS')
            or self.home/'.config/sirius/face_speech_presets.json')

    @staticmethod
    def speech_text(value):
        if not isinstance(value, str):
            raise ValueError('セリフを文字で入力してください')
        text = value.strip()
        if not text or len(text) > SPEECH_LIMIT:
            raise ValueError(f'セリフは1〜{SPEECH_LIMIT}文字で入力してください')
        if '[' in text or ']' in text:
            raise ValueError('読み上げセリフには制御タグを使えません')
        return text

    def _read_speech_presets(self):
        try:
            data = json.loads(self.speech_presets_path.read_text(encoding='utf-8'))
        except FileNotFoundError:
            return []
        except (OSError, ValueError) as error:
            raise FaceError('保存済みセリフプリセットを読み込めません') from error
        rows = data.get('presets') if isinstance(data, dict) else data
        if not isinstance(rows, list):
            raise FaceError('保存済みセリフプリセットの形式が不正です')
        presets, texts = [], set()
        for row in rows:
            if not isinstance(row, dict):
                continue
            try:
                text = self.speech_text(row.get('text'))
            except ValueError:
                continue
            if text in texts:
                continue
            preset_id = row.get('id')
            if not isinstance(preset_id, str) or not preset_id.strip():
                preset_id = uuid4().hex
            presets.append({'id': preset_id, 'text': text})
            texts.add(text)
            if len(presets) >= SPEECH_PRESET_LIMIT:
                break
        return presets

    def _write_speech_presets(self, presets):
        path = self.speech_presets_path
        path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        payload = json.dumps({'version': 1, 'presets': presets}, ensure_ascii=False, indent=2) + '\n'
        with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=path.parent,
                                         prefix='.face-speech-', delete=False) as stream:
            temporary = Path(stream.name)
            try:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
                os.replace(temporary, path)
            finally:
                temporary.unlink(missing_ok=True)
        try:
            os.chmod(path, 0o600)
        except OSError:
            pass

    def speech_presets(self):
        with self.speech_lock:
            return self._read_speech_presets()

    def add_speech_preset(self, text):
        text = self.speech_text(text)
        with self.speech_lock:
            presets = self._read_speech_presets()
            if any(item['text'] == text for item in presets):
                return presets
            if len(presets) >= SPEECH_PRESET_LIMIT:
                raise ValueError(f'セリフプリセットは{SPEECH_PRESET_LIMIT}件までです')
            presets.insert(0, {'id': uuid4().hex, 'text': text})
            self._write_speech_presets(presets)
            return presets

    def update_speech_preset(self, preset_id, text):
        if not isinstance(preset_id, str) or not preset_id:
            raise ValueError('編集するセリフプリセットを指定してください')
        text = self.speech_text(text)
        with self.speech_lock:
            presets = self._read_speech_presets()
            target = next((item for item in presets if item['id'] == preset_id), None)
            if target is None:
                raise FileNotFoundError('編集するセリフプリセットが見つかりません')
            if any(item['id'] != preset_id and item['text'] == text for item in presets):
                raise ValueError('同じセリフがすでに登録されています')
            if target['text'] != text:
                target['text'] = text
                self._write_speech_presets(presets)
            return presets

    def delete_speech_preset(self, preset_id):
        if not isinstance(preset_id, str) or not preset_id:
            raise ValueError('削除するセリフプリセットを指定してください')
        with self.speech_lock:
            presets = self._read_speech_presets()
            remaining = [item for item in presets if item['id'] != preset_id]
            if len(remaining) != len(presets):
                self._write_speech_presets(remaining)
            return remaining

    def preview(self):
        """A small, fresh snapshot of the actual Face window; never the desktop."""
        with self.preview_lock:
            if self.preview_cached is not None and time.monotonic()-self.preview_at < .08:
                return self.preview_cached
            self.preview_cached = None
            windows = self.windows(self.shortcut())
            if not windows:
                raise FaceError('SiriusFaceのウィンドウがありません')
            window = max(windows, key=lambda w:w['width']*w['height'])
            try:
                result = subprocess.run(['/usr/bin/python3',str(Path(__file__).with_name('face_preview_worker.py')),window['id']],
                    capture_output=True, timeout=2, env=self.gui_env())
            except (OSError, subprocess.TimeoutExpired) as error:
                raise FaceError('顔ウィンドウのキャプチャを取得できません') from error
            if result.returncode or not result.stdout.startswith(b'\xff\xd8') or len(result.stdout)>150000:
                raise FaceError('顔のキャプチャは未取得です（表示環境を確認してください）')
            self.preview_cached, self.preview_at = result.stdout, time.monotonic()
            return result.stdout

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
        command = shortcut['executable']
        executable = Path(command if os.path.isabs(command) else shutil.which(command) or '')
        if not executable.is_absolute():
            return []
        executable = executable.resolve()
        # An AppImage's UI runs from a temporary FUSE mount with a different
        # executable path. Match the mount type and the app's executable name.
        mount_type = 'fuse.' + executable.name
        mounts = {
            Path(fields[4].replace(r'\040', ' '))
            for line in Path('/proc/self/mountinfo').read_text().splitlines()
            if (fields := line.split()) and fields[fields.index('-') + 1] == mount_type
        } if executable.suffix.lower() == '.appimage' else set()
        result = subprocess.run(['ps', '-u', str(os.getuid()), '-o', 'pid='],
                                capture_output=True, text=True, timeout=1, check=True)
        found = []
        for pid in map(int, result.stdout.split()):
            try:
                target = Path(os.readlink(f'/proc/{pid}/exe'))
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
            if target == executable or (
                target.name.lower() == shortcut['wm_class'] and target.parent in mounts
            ):
                found.append(pid)
        return found

    def stop(self, shortcut):
        pids = self.processes(shortcut)
        windows = self.windows(shortcut)
        if not pids and (windows or time.monotonic() < self.launch_until):
            raise FaceError('顔のプロセスを確認できません。少し待ってから再試行してください')
        self.cancel_placement.set()
        self.cached = self.preview_cached = None
        self.launch_until = 0
        for window in windows:
            if window['pid'] in pids:
                try:
                    subprocess.run(['wmctrl', '-i', '-c', window['id']], env=self.gui_env(),
                                   capture_output=True, timeout=1)
                except (OSError, subprocess.TimeoutExpired):
                    break

        def wait_for_exit(timeout):
            deadline = time.monotonic() + timeout
            remaining = self.processes(shortcut)
            while remaining and time.monotonic() < deadline:
                time.sleep(.1)
                remaining = self.processes(shortcut)
            return remaining

        remaining = wait_for_exit(2 if windows else 0)
        for sig in (signal.SIGTERM, signal.SIGKILL):
            if not remaining:
                break
            for pid in remaining:
                try:
                    fd = os.pidfd_open(pid)
                except ProcessLookupError:
                    continue
                try:
                    if pid in self.processes(shortcut):
                        signal.pidfd_send_signal(fd, sig)
                except ProcessLookupError:
                    pass
                finally:
                    os.close(fd)
            remaining = wait_for_exit(2)
        if remaining or self.windows(shortcut) or self.listening(50051):
            raise FaceError('SiriusFaceの終了を確認できません')
        return dict(ok=True, stopped=True)

    def launch(self, shortcut, env):
        result = subprocess.run(['gio', 'launch', str(shortcut['path'])], env=env,
                                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL, timeout=3, start_new_session=True)
        if result.returncode:
            raise FaceError('SiriusFaceのショートカットを起動できませんでした')
        self.launch_until = time.monotonic()+30
        self.cached = None
        if env.get('DISPLAY'):
            self.cancel_placement.set()
            self.cancel_placement = threading.Event()
            threading.Thread(target=self.place_window, args=(shortcut, env, self.cancel_placement),
                             daemon=True).start()
        return dict(ok=True, starting=True)

    def place_window(self, shortcut, env, canceled):
        # Follow the same workspace setting as desktop autostart.
        config_dir = Path(env.get('XDG_CONFIG_HOME') or self.home/'.config')
        config = configparser.ConfigParser(interpolation=None)
        try:
            config.read(config_dir/'autostart/sirius-face.desktop')
            args = shlex.split(config.get('Desktop Entry', 'Exec', fallback=''))
        except (OSError, ValueError, configparser.Error):
            args = []
        workspace = None
        for arg in args:
            key, _, value = arg.partition('=')
            if key == 'SIRIUS_FACE_WORKSPACE' and value.isdecimal():
                workspace = int(value)
                break
        deadline = time.monotonic()+30
        while not canceled.is_set() and time.monotonic() < deadline:
            windows = self.windows(shortcut)
            if windows:
                with self.lock:
                    if not canceled.is_set():
                        ensure_fullscreen(windows[0]['id'], workspace, env)
                return
            canceled.wait(.25)

    def launcher_windows(self):
        port = os.environ.get('SIRIUS_WEB_PORT', '8090')
        return launcher_windows(port, self.gui_env())

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
        if action not in {'launch', 'stop', 'restart', 'focus', 'focus_launcher', 'expression',
                          'gaze', 'subtitle', 'clear_text', 'speak', 'instruction'}:
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
            if action == 'launch':
                state = self.status()
                if state['connected'] or state['running'] or state['starting']:
                    return dict(ok=True, already_running=True)
            if action in {'launch', 'stop', 'restart'}:
                shortcut = self.shortcut()
                if not shortcut:
                    raise FaceError('デスクトップのSirius Face UIショートカットが見つかりません')
                if action == 'stop':
                    return self.stop(shortcut)
                env = self.gui_env()
                if not env.get('DISPLAY') and not env.get('WAYLAND_DISPLAY'):
                    raise FaceError('NUCのデスクトップに接続できません')
                if not shutil.which('gio'):
                    raise FaceError('SiriusFaceの起動に必要な gio が見つかりません')
                if action == 'restart':
                    self.stop(shortcut)
                return self.launch(shortcut, env)
            state = self.status()
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

    @bp.get('/api/face/preview')
    def preview():
        if not auth_ok():
            return jsonify(error='auth'), 401
        try:
            response = Response(bridge.preview(), mimetype='image/jpeg')
            response.headers['Cache-Control'] = 'no-store'
            response.headers['X-Content-Type-Options'] = 'nosniff'
            return response
        except FaceError as error:
            return jsonify(error=str(error)), 503

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

    @bp.get('/api/face/speech-presets')
    def speech_presets():
        if not auth_ok():
            return jsonify(error='auth'), 401
        try:
            response = jsonify(presets=bridge.speech_presets())
        except (FaceError, OSError) as error:
            return jsonify(error=str(error)), 503
        response.headers['Cache-Control'] = 'no-store'
        return response

    @bp.post('/api/face/speech-presets')
    def add_speech_preset():
        if not auth_ok():
            return jsonify(error='auth'), 401
        body = request.get_json(silent=True)
        if not isinstance(body, dict):
            return jsonify(error='セリフを入力してください'), 400
        try:
            response = jsonify(presets=bridge.add_speech_preset(body.get('text')))
        except ValueError as error:
            return jsonify(error=str(error)), 400
        except (FaceError, OSError) as error:
            return jsonify(error=str(error)), 503
        response.headers['Cache-Control'] = 'no-store'
        return response, 201

    @bp.route('/api/face/speech-presets/<preset_id>', methods=['PUT', 'PATCH'])
    def update_speech_preset(preset_id):
        if not auth_ok():
            return jsonify(error='auth'), 401
        body = request.get_json(silent=True)
        if not isinstance(body, dict):
            return jsonify(error='セリフを入力してください'), 400
        try:
            response = jsonify(presets=bridge.update_speech_preset(preset_id, body.get('text')))
        except ValueError as error:
            return jsonify(error=str(error)), 400
        except FileNotFoundError as error:
            return jsonify(error=str(error)), 404
        except (FaceError, OSError) as error:
            return jsonify(error=str(error)), 503
        response.headers['Cache-Control'] = 'no-store'
        return response

    @bp.delete('/api/face/speech-presets/<preset_id>')
    def delete_speech_preset(preset_id):
        if not auth_ok():
            return jsonify(error='auth'), 401
        try:
            response = jsonify(presets=bridge.delete_speech_preset(preset_id))
        except ValueError as error:
            return jsonify(error=str(error)), 400
        except (FaceError, OSError) as error:
            return jsonify(error=str(error)), 503
        response.headers['Cache-Control'] = 'no-store'
        return response

    app.register_blueprint(bp)
