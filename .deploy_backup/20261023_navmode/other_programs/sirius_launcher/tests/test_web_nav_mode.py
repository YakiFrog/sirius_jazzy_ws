import importlib
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
web = importlib.import_module('web_server')

MODES = [
    {'mode': 'normal', 'label': '通常', 'description': '標準', 'speed': 0.9},
    {'mode': 'wait_normal', 'label': '待機優先', 'description': '待機', 'speed': 0.6},
]


def test_modes_endpoint_returns_shared_modes(monkeypatch):
    monkeypatch.setattr(web.NAV_MODE, 'modes', lambda: MODES)
    client = web.app.test_client()
    response = client.get('/api/navigation_modes')
    assert response.status_code == 200
    assert response.get_json()['modes'] == MODES


def test_state_endpoint_does_not_guess_when_waiting(monkeypatch):
    monkeypatch.setattr(web.NAV_MODE, 'state', lambda domain: {
        'ok': True, 'status': 'waiting', 'mode': None, 'label': None,
        'speed': None, 'domain': domain,
    })
    client = web.app.test_client()
    response = client.get('/api/navigation_mode?domain=220')
    assert response.status_code == 200
    data = response.get_json()
    assert data['status'] == 'waiting'
    assert data['mode'] is None


def test_apply_endpoint_returns_failure_details(monkeypatch):
    payload = {
        'ok': False, 'mode': 'wait_normal',
        'error': '一部の設定に失敗しました',
        'details': ['velocity_smoother: service missing'],
        'state': {'status': 'waiting', 'mode': None},
    }
    monkeypatch.setattr(web.NAV_MODE, 'apply', lambda mode, domain: (payload, 409))
    client = web.app.test_client()
    response = client.post('/api/navigation_mode', json={'mode': 'wait_normal', 'domain': '220'})
    assert response.status_code == 409
    data = response.get_json()
    assert data['ok'] is False
    assert data['details'] == ['velocity_smoother: service missing']


def test_apply_endpoint_rejects_invalid_domain(monkeypatch):
    called = []
    monkeypatch.setattr(web.NAV_MODE, 'apply', lambda mode, domain: called.append((mode, domain)))
    client = web.app.test_client()
    response = client.post('/api/navigation_mode', json={'mode': 'normal', 'domain': '999'})
    assert response.status_code == 400
    assert not called
