import importlib
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def test_process_uptime_uses_process_create_time(monkeypatch):
    web = importlib.import_module('web_server')
    monkeypatch.setattr(web.psutil, 'Process',
                        lambda pid: type('Proc', (), {'create_time': lambda self: 100.0})())
    monkeypatch.setattr(web.time, 'time', lambda: 225.9)
    assert web.process_uptime(123) == 125


def test_status_includes_uptime_for_launcher_processes(monkeypatch):
    web = importlib.import_module('web_server')
    monkeypatch.setattr(web, 'scan_pid_files', lambda: {'alpha'})
    monkeypatch.setattr(web, 'running_pid', lambda name: 4321)
    monkeypatch.setattr(web, 'process_uptime', lambda pid: 125)
    monkeypatch.setattr(web, 'has_error', lambda name: False)
    monkeypatch.setattr(web, 'managed_sessions', lambda: {})
    monkeypatch.setattr(web, 'external_matches', lambda *args: {})
    client = web.app.test_client()
    response = client.get('/api/status')
    assert response.status_code == 200
    assert response.get_json()['alpha']['uptime'] == 125
