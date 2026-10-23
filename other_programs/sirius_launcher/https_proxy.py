"""Honor reverse-proxy URL headers only from a local TLS terminator."""
from werkzeug.middleware.proxy_fix import ProxyFix


class LocalHTTPSProxy:
    def __init__(self, app):
        self.app = app
        self.local_proxy = ProxyFix(app, x_proto=1, x_host=1)

    def __call__(self, environ, start_response):
        if environ.get('REMOTE_ADDR') in {'127.0.0.1', '::1', '::ffff:127.0.0.1'}:
            return self.local_proxy(environ, start_response)
        return self.app(environ, start_response)
