import importlib
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def setup_store(monkeypatch, tmp_path):
    web = importlib.import_module('web_server')
    monkeypatch.setattr(web, 'LAUNCH_SPECS_FILE', tmp_path/'launch.json')
    monkeypatch.setattr(web, 'LAUNCH_SPECS', {})
    monkeypatch.setattr(web, 'LAUNCH_SPECS_LOADED', True)
    return web


def test_shared_process_origins_persist_without_duplicates_and_ignore_pid_reuse(monkeypatch, tmp_path):
    web = setup_store(monkeypatch, tmp_path)
    identity = [123, 1.5]
    monkeypatch.setattr(web, 'program_identity', lambda name: identity.copy())
    monkeypatch.setattr(web, 'process_identity', lambda pid: identity.copy())
    for preset in ['地図生成', '自律移動', '地図生成']:
        web.record_preset_origin('imu', preset)
    monkeypatch.setattr(web, 'LAUNCH_SPECS', {})
    monkeypatch.setattr(web, 'LAUNCH_SPECS_LOADED', False)
    status = {'imu': {'pid': 123}}
    web.attach_preset_origins(status)
    assert status['imu']['started_by_presets'] == ['地図生成', '自律移動']
    identity[1] = 2.5
    web.attach_preset_origins(status)
    assert status['imu']['started_by_presets'] == []
    web.record_preset_origin('imu', '新しい実験')
    web.attach_preset_origins(status)
    assert status['imu']['started_by_presets'] == ['新しい実験']


def test_preset_route_tracks_started_and_shared_but_not_failed(monkeypatch, tmp_path):
    web = setup_store(monkeypatch, tmp_path)
    monkeypatch.setattr(web, 'PRESETS', [{'name': '実験', 'items': ['new', 'shared', 'bad', 'new']}])
    monkeypatch.setattr(web, 'start_one', lambda name, domain: {
        'new': (True, 'started'), 'shared': (True, 'already running'), 'bad': (False, 'failed')}[name])
    monkeypatch.setattr(web.time, 'sleep', lambda _: None)
    calls = []
    monkeypatch.setattr(web, 'record_preset_origin', lambda name, preset: calls.append((name, preset)))
    response = web.app.test_client().post('/api/preset', json={'name': '実験'})
    assert response.status_code == 200
    assert calls == [('new', '実験'), ('shared', '実験')]
