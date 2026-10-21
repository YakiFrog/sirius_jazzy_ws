"""A local TLS proxy must not produce mixed-content viewer or ROS URLs."""
import importlib
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def test_local_https_proxy_preserves_external_origin():
    web = importlib.import_module('web_server')
    client = web.app.test_client()
    headers = {'X-Forwarded-Proto': 'https', 'X-Forwarded-Host': 'robot.example.ts.net'}
    for address in ['127.0.0.1', '::1']:
        response = client.get('/api/visualization/config', base_url='http://127.0.0.1:8090',
                              headers=headers, environ_overrides={'REMOTE_ADDR': address})
        data = response.get_json()
        assert data['viewer_url'] == 'https://robot.example.ts.net:8443/'
        assert data['bridge_url'] == 'wss://robot.example.ts.net:8767'
        assert data['layout_url'] == 'https://robot.example.ts.net/api/visualization/layout'
    response = client.get('/api/visualization/layouts', headers=headers,
                          environ_overrides={'REMOTE_ADDR': '127.0.0.1'})
    assert response.get_json()['layouts'][0]['url'].startswith('https://robot.example.ts.net/')


def test_external_requests_cannot_supply_proxy_headers():
    web = importlib.import_module('web_server')
    response = web.app.test_client().get(
        '/api/visualization/config', base_url='http://100.107.5.14:8090',
        headers={'X-Forwarded-Proto': 'https', 'X-Forwarded-Host': 'spoofed.invalid'},
        environ_overrides={'REMOTE_ADDR': '100.100.100.100'})
    assert response.get_json()['viewer_url'] == 'http://100.107.5.14:8091/'
    assert response.get_json()['bridge_url'] == 'ws://100.107.5.14:8766'


def test_https_setup_refuses_to_replace_another_service():
    from enable_https import validate_existing
    import pytest
    validate_existing({}, 'robot.ts.net')
    validate_existing({'Web': {'robot.ts.net:443': {'Handlers': {'/': {'Proxy': 'http://127.0.0.1:8090'}}}}}, 'robot.ts.net')
    with pytest.raises(ValueError):
        validate_existing({'Web': {'robot.ts.net:443': {'Handlers': {'/': {'Proxy': 'http://127.0.0.1:9999'}}}}}, 'robot.ts.net')
    with pytest.raises(ValueError):
        validate_existing({'TCP': {'443': {'TCPForward': 'localhost:22'}}}, 'robot.ts.net')
