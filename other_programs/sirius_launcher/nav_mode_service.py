"""Backend bridge for the Nav2 driving-mode UI.

The ROS definitions and the parameter application remain in
``sirius_navigation.navigation_modes`` and ``change_nav_mode.sh``.  This
module only runs those commands and watches the confirmed ROS state topic.
"""
import json
import os
import subprocess
import threading
import time
from pathlib import Path


class NavModeService:
    def __init__(self, home=None):
        self.home = Path(home or os.path.expanduser('~'))
        self.workspace = self.home / 'sirius_jazzy_ws'
        self.apply_script = self.workspace / 'bash/startup_bash/change_nav_mode.sh'
        self.info_script = self.workspace / 'bash/startup_bash/nav_mode_info.sh'
        self.state_script = self.workspace / 'bash/startup_bash/nav_mode_state.sh'
        self._modes = None
        self._modes_error = ''
        self._modes_lock = threading.Lock()
        self._apply_lock = threading.Lock()
        self._watcher = None
        self._watcher_domain = ''
        self._watcher_lock = threading.Lock()

    @staticmethod
    def domain(value):
        text = str(value or '').strip()
        if not text.isdigit() or not 0 <= int(text) <= 232:
            raise ValueError('invalid ROS domain')
        return text

    def modes(self):
        with self._modes_lock:
            if self._modes is not None:
                return self._modes
            try:
                result = subprocess.run(
                    ['bash', str(self.info_script)],
                    capture_output=True, text=True, timeout=15,
                    env=dict(os.environ),
                )
                if result.returncode:
                    raise RuntimeError(result.stderr.strip() or 'mode info failed')
                payload = json.loads(result.stdout.strip().splitlines()[-1])
                modes = payload.get('modes')
                if not isinstance(modes, list) or not modes:
                    raise RuntimeError('mode list is empty')
                self._modes = modes
                self._modes_error = ''
            except (OSError, ValueError, KeyError, IndexError,
                    subprocess.SubprocessError) as error:
                self._modes_error = str(error)
                self._modes = []
            return self._modes

    def mode_info(self, mode):
        for item in self.modes():
            if item.get('mode') == mode:
                return item
        return None

    def _state_path(self, domain):
        return Path('/tmp') / f'sirius_nav_mode_state_{domain}.json'

    def _applied_path(self, domain):
        return Path('/tmp') / f'sirius_nav_mode_applied_{domain}.json'

    def _pid_path(self, domain):
        return Path('/tmp') / f'sirius_nav_mode_state_{domain}.pid'

    def _pid_alive(self, domain):
        try:
            pid = int(self._pid_path(domain).read_text(encoding='utf-8').strip())
        except (OSError, ValueError):
            return False
        return Path(f'/proc/{pid}').exists()

    def _write_log(self, domain, stream):
        path = Path('/tmp') / f'sirius_nav_mode_state_{domain}.log'
        path.write_text(stream or '', encoding='utf-8', errors='replace')

    def ensure_watcher(self, domain):
        domain = self.domain(domain)
        with self._watcher_lock:
            if self._pid_alive(domain):
                self._watcher_domain = domain
                return
            if (self._watcher is not None and self._watcher.poll() is None
                    and self._watcher_domain == domain):
                return
            if self._watcher is not None and self._watcher.poll() is None:
                self._watcher.terminate()
                try:
                    self._watcher.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self._watcher.kill()
            state_path = self._state_path(domain)
            try:
                state_path.unlink()
            except FileNotFoundError:
                pass
            env = dict(os.environ)
            env['ROS_DOMAIN_ID'] = domain
            self._watcher = subprocess.Popen(
                ['bash', str(self.state_script), '--domain', domain,
                 '--output', str(state_path)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                env=env, start_new_session=True,
            )
            self._watcher_domain = domain
            time.sleep(0.6)

    def _read_state(self, domain):
        path = self._state_path(domain)
        try:
            data = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            return None
        return data if isinstance(data, dict) else None

    def _read_applied(self, domain):
        path = self._applied_path(domain)
        try:
            data = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            return None
        if not isinstance(data, dict) or data.get('status') != 'ok':
            return None
        received_at = data.get('received_at')
        if not received_at or time.time() - float(received_at) > 120:
            return None
        return data

    def state(self, domain):
        domain = self.domain(domain)
        self.ensure_watcher(domain)
        alive = ((self._watcher is not None and self._watcher.poll() is None)
                 or self._pid_alive(domain))
        data = self._read_state(domain)
        applied = self._read_applied(domain)
        if applied and (not data or data.get('status') != 'ok'
                        or float(applied.get('received_at') or 0) >= float(data.get('received_at') or 0)):
            data = applied
        if not alive:
            return {
                'ok': True,
                'status': 'unavailable',
                'mode': None,
                'label': None,
                'speed': None,
                'domain': domain,
                'error': '状態監視プロセスが停止しています',
            }
        if not data or data.get('status') != 'ok' or not data.get('mode'):
            return {
                'ok': True,
                'status': 'waiting',
                'mode': None,
                'label': None,
                'speed': None,
                'domain': domain,
                'received_at': data.get('received_at') if data else None,
            }
        raw_mode = str(data['mode']).strip()
        normalized = 'strict_safe' if raw_mode == 'strict' else raw_mode
        info = self.mode_info(normalized)
        received_at = data.get('received_at')
        age = max(0.0, time.time() - float(received_at)) if received_at else None
        if info is None:
            return {
                'ok': True,
                'status': 'unknown_mode',
                'mode': raw_mode,
                'label': '不明なモード',
                'speed': None,
                'domain': domain,
                'received_at': received_at,
                'age_seconds': age,
            }
        return {
            'ok': True,
            'status': 'ok',
            'mode': normalized,
            'label': info['label'],
            'speed': info['speed'],
            'domain': domain,
            'received_at': received_at,
            'age_seconds': age,
            'source': data.get('source', 'ros'),
        }

    def apply(self, mode, domain):
        domain = self.domain(domain)
        mode = str(mode or '').strip()
        info = self.mode_info(mode)
        if info is None:
            return {'ok': False, 'error': '不明な走行モードです'}, 400
        if not self._apply_lock.acquire(blocking=False):
            return {'ok': False, 'error': '別のモード変更を適用中です'}, 409
        try:
            self.ensure_watcher(domain)
            env = dict(os.environ)
            env['ROS_DOMAIN_ID'] = domain
            try:
                result = subprocess.run(
                    ['bash', str(self.apply_script), mode, '--json'],
                    capture_output=True, text=True, timeout=60, env=env,
                )
            except subprocess.TimeoutExpired:
                return {
                    'ok': False,
                    'mode': mode,
                    'error': 'モード変更がタイムアウトしました',
                    'details': 'Nav2のパラメータサービスが応答しませんでした。',
                    'state': self.state(domain),
                }, 504
            except OSError as error:
                return {
                    'ok': False,
                    'mode': mode,
                    'error': f'モード変更を開始できません: {error}',
                    'state': self.state(domain),
                }, 503
            payload = None
            for line in reversed(result.stdout.splitlines()):
                line = line.strip()
                if line.startswith('{'):
                    try:
                        payload = json.loads(line)
                        break
                    except ValueError:
                        continue
            details = result.stderr.strip().splitlines()[-20:]
            if not isinstance(payload, dict):
                return {
                    'ok': False,
                    'mode': mode,
                    'error': 'モード変更スクリプトの結果を取得できません',
                    'details': details or result.stdout.strip().splitlines()[-20:],
                    'state': self.state(domain),
                }, 502
            if not payload.get('ok') or result.returncode:
                return {
                    **payload,
                    'ok': False,
                    'mode': mode,
                    'error': payload.get('error') or '一部の設定に失敗しました',
                    'details': details,
                    'state': self.state(domain),
                }, 409
            snapshot = self.state(domain)
            if not (snapshot.get('status') == 'ok' and snapshot.get('mode') == mode):
                snapshot = {
                    'ok': True,
                    'status': 'ok',
                    'mode': mode,
                    'label': payload.get('label'),
                    'speed': payload.get('speed'),
                    'domain': domain,
                    'source': 'apply',
                }
            return {
                **payload,
                'ok': True,
                'confirmed': True,
                'state': snapshot,
                'details': details,
            }, 200
        finally:
            self._apply_lock.release()
