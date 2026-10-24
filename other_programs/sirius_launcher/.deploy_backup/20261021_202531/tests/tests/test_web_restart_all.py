import importlib
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def test_restart_all_restores_recorded_domain_and_command(monkeypatch):
    web = importlib.import_module('web_server')
    monkeypatch.setattr(web, 'BUTTONS', {'alpha': 'ros2 launch alpha', 'beta': 'ros2 launch beta'})
    monkeypatch.setattr(web, 'running_program_names', lambda: ['alpha', 'beta'])
    monkeypatch.setattr(web, 'launch_specs_snapshot', lambda: {
        'alpha': {'domain': '12', 'command': 'ros2 launch alpha old:=true'},
    })
    calls = []

    def fake_stop(name):
        calls.append(('stop', name))
        return True, 'stopped'

    def fake_start(name, domain='', command=None):
        calls.append(('start', name, domain, command))
        return True, 'started'

    monkeypatch.setattr(web, 'stop_one', fake_stop)
    monkeypatch.setattr(web, 'start_one', fake_start)

    result = web.restart_all_programs('99', restore=True)

    assert result['count'] == 2
    assert result['stopped'] == ['alpha', 'beta']
    assert result['started'] == ['alpha', 'beta']
    assert result['restored'] == ['alpha']
    assert result['failed'] == []
    assert ('start', 'alpha', '12', 'ros2 launch alpha old:=true') in calls
    assert ('start', 'beta', '99', 'ros2 launch beta') in calls


def test_restart_all_without_restore_uses_current_inputs(monkeypatch):
    web = importlib.import_module('web_server')
    monkeypatch.setattr(web, 'BUTTONS', {'alpha': 'ros2 launch alpha'})
    monkeypatch.setattr(web, 'running_program_names', lambda: ['alpha'])
    monkeypatch.setattr(web, 'launch_specs_snapshot', lambda: {
        'alpha': {'domain': '12', 'command': 'ros2 launch alpha old:=true'},
    })
    starts = []

    def fake_start(name, domain='', command=None):
        starts.append((name, domain, command))
        return True, 'started'

    monkeypatch.setattr(web, 'stop_one', lambda name: (True, 'stopped'))
    monkeypatch.setattr(web, 'start_one', fake_start)

    result = web.restart_all_programs('99', restore=False)

    assert result['restored'] == []
    assert starts == [('alpha', '99', 'ros2 launch alpha')]


def test_restart_all_route_returns_result(monkeypatch):
    web = importlib.import_module('web_server')
    monkeypatch.setattr(web, 'restart_all_programs', lambda domain, restore: {
        'count': 1, 'stopped': ['alpha'], 'started': ['alpha'],
        'restored': ['alpha'] if restore else [], 'failed': [],
        'domain': domain,
    })
    client = web.app.test_client()
    response = client.post('/api/restart_all', json={'domain': '42', 'restore': True})
    assert response.status_code == 200
    assert response.get_json()['domain'] == '42'
    assert response.get_json()['restored'] == ['alpha']
