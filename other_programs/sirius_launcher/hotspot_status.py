"""Read the dedicated AP profile without changing networking or reading its key."""
import ipaddress
import os
import subprocess
import threading
import time

from flask import Blueprint, jsonify


class HotspotMonitor:
    def __init__(self, profile=None):
        self.profile = profile or os.environ.get('SIRIUS_HOTSPOT_CONNECTION', 'Sirius-Hotspot')
        self.lock = threading.Lock()
        self.cached = None
        self.cached_at = 0

    @staticmethod
    def run(*args):
        return subprocess.run(['nmcli', '-t', '-e', 'no', *args], capture_output=True,
                              text=True, timeout=1.5, env={**os.environ, 'LC_ALL': 'C'})

    def read(self):
        state = dict(active=False, state='unconfigured', ssid=None, interface=None,
                     address=None, autostart=False)
        try:
            result = self.run('-g', 'connection.interface-name,connection.autoconnect,'
                              '802-11-wireless.mode,802-11-wireless.ssid',
                              'connection', 'show', self.profile)
            if result.returncode:
                state['state'] = 'unconfigured' if result.returncode == 10 else 'error'
                return state
            fields = result.stdout.splitlines()
            if len(fields) != 4 or not fields[0] or fields[2] != 'ap':
                return state
            interface, auto, _, ssid = fields
            state.update(state='inactive', interface=interface, autostart=auto == 'yes', ssid=ssid)
            result = self.run('-f', 'GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS',
                              'device', 'show', interface)
            if result.returncode:
                state['state'] = 'unavailable' if result.returncode == 10 else 'error'
                return state
            values = dict(line.split(':', 1) for line in result.stdout.splitlines() if ':' in line)
            connected = values.get('GENERAL.STATE', '').split(' ', 1)[0] == '100'
            if not connected or values.get('GENERAL.CONNECTION') != self.profile:
                return state
            for key, value in values.items():
                if key.startswith('IP4.ADDRESS'):
                    address = ipaddress.ip_interface(value).ip
                    if address.version == 4 and not (address.is_loopback or address.is_unspecified or address.is_multicast):
                        state.update(active=True, state='active', address=str(address))
                        break
        except (OSError, ValueError, subprocess.TimeoutExpired):
            state.update(active=False, state='error', address=None)
        return state

    def status(self):
        with self.lock:
            if self.cached is None or time.monotonic()-self.cached_at >= 4:
                self.cached = self.read()
                self.cached_at = time.monotonic()
            return dict(self.cached)


def register_hotspot(app, auth_ok, monitor=None):
    monitor = monitor or HotspotMonitor()
    bp = Blueprint('hotspot_status', __name__)

    @bp.get('/api/hotspot/status')
    def status():
        if not auth_ok():
            return jsonify(error='auth'), 401
        state = monitor.status()
        port = app.config.get('SIRIUS_LISTEN_PORT', 8090)
        state['url'] = f"http://{state['address']}:{port}/console" if state['active'] else None
        response = jsonify(state)
        response.headers['Cache-Control'] = 'no-store'
        return response

    app.register_blueprint(bp)
