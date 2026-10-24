"""コマンドバー: ページ内マークアップと、入力コマンドを対話ターミナルで実行する経路。"""
import importlib
import sys
import time
import uuid
from pathlib import Path

LAUNCHER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LAUNCHER))


def wait_for(check, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if check():
            return
        time.sleep(.05)
    raise AssertionError('terminal did not reach the expected state')


def test_page_contains_command_bar():
    web = importlib.import_module('web_server')
    page = web.PAGE
    for key in ('id="cmdin"', 'id="sugg"', 'function runCmd',
                'function buildCmdIndex', 'function renderSugg', 'runInTerminal'):
        assert key in page
    assert '/api/start' in page and '/api/term/open' in page


def test_run_in_terminal_does_not_force_open_panel():
    web = importlib.import_module('web_server')
    frag = web.PAGE.split('async function runInTerminal', 1)[1].split('async function runCmd', 1)[0]
    assert 'toggleTerm' not in frag
    assert "classList.contains('open')" in frag


def test_typed_command_runs_in_interactive_terminal(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    web = importlib.import_module('web_server')
    client = web.app.test_client()
    name = 'cmd_' + uuid.uuid4().hex
    result = client.post('/api/term/open', json={'cmd': '', 'name': name})
    assert result.status_code == 200 and result.get_json()['name'] == name
    try:
        sent = client.post('/api/term/write', json={'name': name, 'data': 'echo $((6*7))\n'})
        assert sent.json['ok']

        def output():
            read = client.get('/api/term/read',
                              query_string={'name': name, 'offset': 0}).json
            return read.get('data', '')

        wait_for(lambda: '42' in output())
    finally:
        client.post('/api/term/close', json={'name': name})
