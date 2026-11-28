"""Monitor and operate the dedicated NetworkManager access point."""
import ipaddress
import os
import subprocess
import threading
import time

from flask import Blueprint, g, jsonify, request

from hotspot_access import USERNAME
from launcher_access import direct_local_request


class HotspotMonitor:
    def __init__(self, profile=None):
        self.profile = profile or os.environ.get('SIRIUS_HOTSPOT_CONNECTION', 'Sirius-Hotspot')
        self.lock = threading.Lock()
        self.control_lock = threading.Lock()
        self.cached = None
        self.cached_at = 0

    @staticmethod
    def run(*args, timeout=1.5):
        return subprocess.run(['nmcli', '-t', '-e', 'no', *args], capture_output=True,
                              text=True, timeout=timeout, env={**os.environ, 'LC_ALL': 'C'})

    def read(self):
        state = dict(active=False, state='unconfigured', ssid=None, interface=None,
                     address=None, autostart=False, can_start=False, can_stop=False)
        try:
            result = self.run('-g', 'connection.interface-name,connection.autoconnect,'
                              '802-11-wireless.mode,802-11-wireless.ssid,connection.uuid',
                              'connection', 'show', 'id', self.profile)
            if result.returncode:
                state['state'] = 'unconfigured' if result.returncode == 10 else 'error'
                return state
            fields = result.stdout.splitlines()
            if len(fields) != 5 or not fields[0] or fields[2] != 'ap' or not fields[4]:
                return state
            interface, auto, _, ssid, uuid = fields
            state.update(state='inactive', interface=interface, autostart=auto == 'yes',
                         ssid=ssid, _uuid=uuid)
            result = self.run('-f', 'GENERAL.STATE,GENERAL.CON-UUID,WIFI-PROPERTIES.AP,IP4.ADDRESS',
                              'device', 'show', interface)
            if result.returncode:
                state['state'] = 'unavailable' if result.returncode == 10 else 'error'
                return state
            values = dict(line.split(':', 1) for line in result.stdout.splitlines() if ':' in line)
            device_state = values.get('GENERAL.STATE', '').split(' ', 1)[0]
            current = values.get('GENERAL.CON-UUID') or '--'
            state['can_start'] = (device_state == '30' and current == '--'
                                  and values.get('WIFI-PROPERTIES.AP') == 'yes')
            state['can_stop'] = current == uuid
            if current != uuid:
                if device_state == '100':
                    state['state'] = 'device_busy'
                return state
            if device_state != '100':
                state['state'] = 'starting'
                return state
            state['state'] = 'no_address'
            for key, value in values.items():
                if key.startswith('IP4.ADDRESS'):
                    address = ipaddress.ip_interface(value).ip
                    if address.version == 4 and not (address.is_loopback or address.is_unspecified or address.is_multicast):
                        state.update(active=True, state='active', address=str(address))
                        break
        except (OSError, ValueError, subprocess.TimeoutExpired):
            state.update(active=False, state='error', address=None, can_start=False, can_stop=False)
        return state

    def status(self):
        with self.lock:
            if self.cached is None or time.monotonic()-self.cached_at >= 4:
                self.cached = self.read()
                self.cached_at = time.monotonic()
            return {key: value for key, value in self.cached.items() if key != '_uuid'}

    def control(self, action):
        if not self.control_lock.acquire(blocking=False):
            return 'busy', 409
        try:
            state = self.read()
            if action == 'start':
                if not state['can_start']:
                    return 'unavailable', 409
            else:
                if not state['can_stop']:
                    return 'unavailable', 409
                result = self.run('--wait', '8', 'connection', 'down', 'uuid',
                                  state['_uuid'], timeout=10)
                if result.returncode:
                    return 'stop_failed', 503
                if action == 'stop':
                    if self.read()['can_stop']:
                        return 'stop_failed', 503
                    return None, 200
                ready = self.read()
                if (not ready['can_start'] or ready['_uuid'] != state['_uuid']
                        or ready['interface'] != state['interface']):
                    return 'unavailable', 409
            result = self.run('--wait', '15', 'connection', 'up', 'uuid', state['_uuid'],
                              'ifname', state['interface'], timeout=17)
            if result.returncode or not self.read()['active']:
                return 'start_failed', 503
            return None, 200
        except subprocess.TimeoutExpired:
            return 'timeout', 504
        except OSError:
            return 'command_failed', 503
        finally:
            with self.lock:
                self.cached = None
            self.control_lock.release()


def register_hotspot(app, auth_ok, monitor=None):
    monitor = monitor or HotspotMonitor()
    bp = Blueprint('hotspot_status', __name__)

    @bp.route('/api/hotspot/status', methods=['GET', 'POST'])
    def status():
        if not auth_ok():
            return jsonify(error='auth'), 401
        can_control = direct_local_request() or bool(g.get('launcher_tailnet_user'))
        error, code = None, 200
        if request.method == 'POST':
            if (not can_control or request.headers.get('Origin') != request.host_url.rstrip('/')
                    or request.headers.get('Sec-Fetch-Site') not in (None, 'same-origin')):
                return jsonify(error='access'), 403
            body = request.get_json(silent=True)
            action = body.get('action') if isinstance(body, dict) else None
            if action not in ('start', 'stop', 'restart'):
                return jsonify(error='invalid_action'), 400
            error, code = monitor.control(action)
        state = monitor.status()
        state.update(can_control=can_control, busy=monitor.control_lock.locked())
        if error:
            state['error'] = error
        port = app.config.get('SIRIUS_LISTEN_PORT', 8090)
        offline = app.config.get('SIRIUS_HOTSPOT_ACCESS')
        listening = offline.listening if offline else None
        state['offline_login'] = bool(offline)
        available = state['active'] and (not app.config.get('SIRIUS_TAILNET_ACCESS')
                                         or (listening and listening[:2] == (state['address'], state['interface'])))
        state['url'] = f"http://{state['address']}:{port}/console" if available else None
        response = jsonify(state)
        response.headers['Cache-Control'] = 'no-store'
        return response, code

    @bp.get('/api/hotspot/login')
    def offline_login():
        if not auth_ok() or not (direct_local_request() or g.get('launcher_tailnet_user')):
            return jsonify(error='access'), 403
        offline = app.config.get('SIRIUS_HOTSPOT_ACCESS')
        if not offline:
            return jsonify(error='unavailable'), 404
        response = jsonify(username=USERNAME, password=offline.password)
        response.headers['Cache-Control'] = 'no-store'
        return response

    app.register_blueprint(bp)
