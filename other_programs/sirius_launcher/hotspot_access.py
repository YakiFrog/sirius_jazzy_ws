"""Serve the Launcher on the active AP interface with offline credentials."""

import hmac
import logging
from pathlib import Path
import socket
import threading
import time

from werkzeug.datastructures import Authorization
from werkzeug.serving import make_server
from werkzeug.wrappers import Response


PASSWORD_PATH = Path.home() / '.config/sirius/launcher-hotspot-password'
USERNAME = 'sirius'


def load_hotspot_password(path=PASSWORD_PATH):
    try:
        password = path.read_text(encoding='ascii').strip()
    except FileNotFoundError:
        return None
    if len(password) < 16 or not password.isprintable():
        raise ValueError('AP用パスワードが無効です')
    return password


def offline_gateway(application, address, port, password):
    host = f'{address}:{port}'
    origin = f'http://{host}'

    def serve(environ, start_response):
        if (environ.get('HTTP_HOST') != host or any(environ.get(name) for name in (
                'HTTP_FORWARDED', 'HTTP_X_FORWARDED_FOR', 'HTTP_X_FORWARDED_HOST',
                'HTTP_X_FORWARDED_PROTO', 'HTTP_TAILSCALE_USER_LOGIN'))):
            return Response(status=403)(environ, start_response)
        if environ['REQUEST_METHOD'] not in ('GET', 'HEAD', 'OPTIONS') and (
                environ.get('HTTP_ORIGIN') != origin
                or environ.get('HTTP_SEC_FETCH_SITE') not in (None, 'same-origin')):
            return Response(status=403)(environ, start_response)
        auth = Authorization.from_header(environ.get('HTTP_AUTHORIZATION'))
        if (not auth or auth.type != 'basic' or auth.username != USERNAME
                or not hmac.compare_digest((auth.password or '').encode(), password.encode())):
            response = Response('AP用のユーザー名とパスワードを入力してください。', 401, {
                'WWW-Authenticate': 'Basic realm="SiriusLauncher Wi-Fi", charset="UTF-8"',
                'Cache-Control': 'no-store',
            })
            return response(environ, start_response)
        environ['sirius.hotspot_authorized'] = True
        return application(environ, start_response)

    return serve


class HotspotAccess:
    def __init__(self, application, monitor, port, password):
        self.application = application
        self.monitor = monitor
        self.port = port
        self.password = password
        self.listening = None

    def start(self):
        threading.Thread(target=self._watch, name='sirius-hotspot-access', daemon=True).start()

    def _listen(self, address, interface):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, interface.encode() + b'\0')
            listener.bind((address, self.port))
            listener.listen()
            gateway = offline_gateway(self.application, address, self.port, self.password)
            server = make_server(address, self.port, gateway, threaded=True, fd=listener.fileno())
        threading.Thread(target=server.serve_forever, name='sirius-hotspot-http', daemon=True).start()
        return server

    def _watch(self):
        server = None
        last_error = None
        while True:
            state = self.monitor.status()
            target = None
            if state['active']:
                try:
                    interface = state['interface']
                    target = (state['address'], interface, socket.if_nametoindex(interface))
                except OSError:
                    pass
            if target != self.listening:
                if server:
                    self.listening = None
                    server.shutdown()
                    server.server_close()
                    server = None
                if target:
                    try:
                        server = self._listen(*target[:2])
                        self.listening = target
                        last_error = None
                    except OSError as error:
                        if str(error) != last_error:
                            logging.getLogger(__name__).warning('AP Launcher: %s', error)
                            last_error = str(error)
            time.sleep(2)
