"""Monitoring must stay read-only, shared and responsive while sampling."""
import importlib
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from resource_monitor import ResourceMonitor
from web_server import classify_process_rows


def test_process_classification_marks_hidden_and_unrelated():
    rows = [
        {'pid': 1, 'related': True, 'command': 'managed-session'},
        {'pid': 2, 'related': False, 'command': 'tracked-alias'},
        {'pid': 3, 'related': False, 'command': 'duplicate-alias'},
        {'pid': 4, 'related': False, 'command': 'external-alias'},
        {'pid': 5, 'related': False, 'command': 'unrelated-tool'},
        {'pid': 6, 'related': False, 'command': ''},
    ]
    result = classify_process_rows(
        rows,
        aliases={2: 'alias_a', 3: 'alias_a', 4: 'alias_b'},
        managed_pids={1, 2},
        tracked_aliases={'alias_a': 2, 'alias_b': None},
        owners={1: 'session_a'},
    )
    by_pid = {row['pid']: row for row in result}
    assert by_pid[1]['managed'] and by_pid[1]['session'] == 'session_a'
    assert not by_pid[1]['hidden'] and not by_pid[1]['unrelated']
    assert by_pid[2]['alias'] == 'alias_a' and not by_pid[2]['hidden']
    assert by_pid[3]['hidden'] and by_pid[3]['hidden_kind'] == 'duplicate'
    assert by_pid[4]['hidden'] and by_pid[4]['hidden_kind'] == 'external'
    assert by_pid[5]['unrelated'] and not by_pid[5]['hidden']
    assert not by_pid[6]['unrelated'] and not by_pid[6]['hidden']  # kernel thread


def test_requests_do_not_wait_for_an_in_progress_sample(monkeypatch):
    monitor = ResourceMonitor(interval=.01, history_size=3)
    entered, release = threading.Event(), threading.Event()
    def collect():
        entered.set(); release.wait(timeout=2)
        return {'timestamp': time.time(), 'cpu_percent': 10, 'memory_percent': 20,
                'processes': [], 'hostname': 'test-host'}
    monkeypatch.setattr(monitor, '_collect', collect)
    monitor.start()
    try:
        assert entered.wait(timeout=2)
        start = time.monotonic()
        for _ in range(30):
            assert monitor.snapshot()['warming_up']
        assert time.monotonic()-start < .2
        release.set()
        deadline = time.monotonic()+2
        while time.monotonic() < deadline and not monitor.snapshot().get('available'):
            time.sleep(.01)
        assert monitor.snapshot()['available']
        time.sleep(.06)
        assert len(monitor.snapshot()['history']) == 3
    finally:
        release.set(); monitor.close()


def test_resource_auth_and_summary_preserve_the_shared_cache(monkeypatch):
    web = importlib.import_module('web_server')
    monitor = ResourceMonitor()
    monitor._latest = {'hostname': 'test-host', 'timestamp': time.time(),
                       'processes': [{'pid': 123, 'related': False, 'command': '<script>'}]}
    monkeypatch.setattr(monitor, 'start', lambda: None)
    monkeypatch.setattr(web, 'RESOURCES', monitor)
    monkeypatch.setattr(web, 'PASSWORD', 'test-password')
    client = web.app.test_client()
    assert client.get('/api/resources').status_code == 401
    assert client.get('/api/health').json['service'] == 'sirius-launcher'
    client.set_cookie('pw', 'test-password')
    summary = client.get('/api/resources?summary=1')
    assert summary.status_code == 200
    assert 'processes' not in summary.json and 'history' not in summary.json
    full = client.get('/api/resources')
    assert full.headers['Cache-Control'] == 'no-store'
    assert full.json['processes'][0]['command'] == '<script>'
    assert len(monitor._latest['processes']) == 1


def test_new_ui_keeps_program_input_and_dedicated_navigation():
    web = importlib.import_module('web_server')
    assert all('data-page="'+name+'"' in web.PAGE for name in ('home','terminal','resources','settings'))
    assert 'id="login"' in web.PAGE and 'id="termin"' in web.PAGE
    assert 'id="cpu-chart"' in web.PAGE and 'id="memory-chart"' in web.PAGE
