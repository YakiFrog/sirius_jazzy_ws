import json
import subprocess
from pathlib import Path
from unittest import mock

import pytest
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from nav_mode_service import NavModeService


MODES = [
    {'mode': 'normal', 'label': '通常', 'description': '標準', 'speed': 0.9},
    {'mode': 'wait_normal', 'label': '待機優先', 'description': '待機', 'speed': 0.6},
]


class Watcher:
    def __init__(self, alive=True):
        self.alive = alive

    def poll(self):
        return None if self.alive else 1

    def terminate(self):
        self.alive = False

    def wait(self, timeout=None):
        return 0

    def kill(self):
        self.alive = False


@pytest.fixture
def service(tmp_path):
    svc = NavModeService(home=tmp_path)
    svc._modes = list(MODES)
    svc._modes_error = ''
    svc.ensure_watcher = lambda domain: None
    return svc


def test_modes_are_reused_from_navigation_definitions():
    sys.path.insert(0, str(ROOT.parents[1] / 'src/sirius/sirius_navigation'))
    from sirius_navigation.navigation_modes import (
        NAVIGATION_MODE_PRESENTATION,
        navigation_mode_speech,
    )

    assert len(NAVIGATION_MODE_PRESENTATION) == 10
    assert navigation_mode_speech('wait_normal') == '待機優先モード、秒速0.6メートルに変更しました'


def test_state_waits_without_mode_instead_of_guessing(service, tmp_path, monkeypatch):
    state_path = tmp_path / 'state.json'
    monkeypatch.setattr(service, '_state_path', lambda domain: state_path)
    service._watcher = Watcher(True)
    service._watcher_domain = '201'
    state_path.write_text(json.dumps({'status': 'waiting', 'mode': None, 'received_at': 1}))
    result = service.state('201')
    assert result['status'] == 'waiting'
    assert result['mode'] is None
    assert result['label'] is None


def test_state_unavailable_when_watcher_stops(service, tmp_path, monkeypatch):
    monkeypatch.setattr(service, '_state_path', lambda domain: tmp_path / 'state.json')
    service._watcher = Watcher(False)
    service._watcher_domain = '202'
    result = service.state('202')
    assert result['status'] == 'unavailable'
    assert result['mode'] is None


def test_unknown_mode_is_not_mapped_to_normal(service, tmp_path, monkeypatch):
    state_path = tmp_path / 'state.json'
    monkeypatch.setattr(service, '_state_path', lambda domain: state_path)
    service._watcher = Watcher(True)
    service._watcher_domain = '203'
    state_path.write_text(json.dumps({'status': 'ok', 'mode': 'mystery', 'received_at': 1}))
    result = service.state('203')
    assert result['status'] == 'unknown_mode'
    assert result['mode'] == 'mystery'
    assert result['label'] == '不明なモード'


def test_apply_success_keeps_audio_result_and_confirms_state(service, monkeypatch):
    payload = {
        'ok': True, 'mode': 'wait_normal', 'label': '待機優先', 'speed': 0.6,
        'audio': {'ok': False, 'error': 'Face unavailable'}, 'errors': [],
    }
    completed = subprocess.CompletedProcess([], 0, json.dumps(payload), '')
    monkeypatch.setattr('nav_mode_service.subprocess.run', lambda *args, **kwargs: completed)
    monkeypatch.setattr(service, 'state', lambda domain: {
        'ok': True, 'status': 'ok', 'mode': 'wait_normal',
        'label': '待機優先', 'speed': 0.6, 'domain': domain,
    })
    result, status = service.apply('wait_normal', '204')
    assert status == 200
    assert result['ok'] is True
    assert result['confirmed'] is True
    assert result['audio']['ok'] is False
    assert result['state']['mode'] == 'wait_normal'


def test_apply_partial_failure_returns_details(service, monkeypatch):
    payload = {
        'ok': False, 'mode': 'wait_normal',
        'errors': ['velocity_smoother: service missing'], 'audio': None,
    }
    completed = subprocess.CompletedProcess([], 1, json.dumps(payload), 'detail line')
    monkeypatch.setattr('nav_mode_service.subprocess.run', lambda *args, **kwargs: completed)
    monkeypatch.setattr(service, 'state', lambda domain: {
        'ok': True, 'status': 'waiting', 'mode': None, 'domain': domain,
    })
    result, status = service.apply('wait_normal', '205')
    assert status == 409
    assert result['ok'] is False
    assert result['details'] == ['detail line']
    assert result['errors'] == ['velocity_smoother: service missing']


def test_apply_timeout_is_reported(service, monkeypatch):
    def timeout(*args, **kwargs):
        raise subprocess.TimeoutExpired(args[0], 60)

    monkeypatch.setattr('nav_mode_service.subprocess.run', timeout)
    monkeypatch.setattr(service, 'state', lambda domain: {
        'ok': True, 'status': 'waiting', 'mode': None, 'domain': domain,
    })
    result, status = service.apply('wait_normal', '206')
    assert status == 504
    assert result['ok'] is False
    assert 'タイムアウト' in result['error']


def test_invalid_domain_and_mode_are_rejected(service):
    with pytest.raises(ValueError):
        service.state('999')
    result, status = service.apply('not-a-mode', '207')
    assert status == 400
    assert result['ok'] is False
