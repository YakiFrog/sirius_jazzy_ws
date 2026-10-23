import importlib
import sys
import time
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_program_console_stream_keeps_ansi_utf8_and_detects_restart(tmp_path, monkeypatch):
    web = importlib.import_module('web_server')
    log = tmp_path/'program.log'
    monkeypatch.setattr(web, 'BUTTONS', {'demo': 'test'})
    monkeypatch.setattr(web, 'log_path', lambda _: str(log))
    monkeypatch.setattr(web, 'session_request', lambda _: {'ok': True})
    client = web.app.test_client()
    payload = '\x1b[31m赤\x1b[0m\r\n'.encode()
    log.write_bytes(payload[:-1] + b'\xe3\x81')
    first = client.get('/api/program_terminal/read?name=demo&offset=0').json
    assert first['interactive'] and '\x1b[31m赤' in first['data']
    assert first['offset'] == len(payload)-1
    with log.open('ab') as stream:stream.write(b'\x82')
    second = client.get('/api/program_terminal/read', query_string={'name':'demo','offset':first['offset']}).json
    assert second['data'] == 'あ'
    log.write_bytes(b'new session\r\n')
    reset = client.get('/api/program_terminal/read', query_string={'name':'demo','offset':second['offset']}).json
    assert reset['reset'] and 'new session' in reset['data']
    assert client.get('/api/program_terminal/read?name=unknown').status_code == 404
    assert client.get('/api/program_terminal/read?name=demo&offset=bad').status_code == 400


def test_shell_console_raw_ansi_and_resize(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME',str(tmp_path))
    web=importlib.import_module('web_server');client=web.app.test_client();name='console_test_'+uuid.uuid4().hex
    assert client.post('/api/term/open',json={'name':name}).status_code==200
    try:
        assert client.post('/api/term/resize',json={'name':name,'cols':97,'rows':28}).json['ok']
        assert client.post('/api/term/resize',json={'name':name,'cols':0,'rows':28}).status_code==400
        client.post('/api/term/write',json={'name':name,'data':"printf '\\033[31mANSI_OK\\033[0m\\n'; stty size\n"})
        end=time.monotonic()+5
        while time.monotonic()<end:
            raw=client.get('/api/term/read',query_string={'name':name,'raw':1,'offset':0}).json['data']
            if '\x1b[31mANSI_OK\x1b[0m' in raw and '28 97' in raw:break
            time.sleep(.03)
        else:raise AssertionError(raw)
        plain=client.get('/api/term/read',query_string={'name':name,'offset':0}).json['data']
        assert '\x1b[31m' not in plain
    finally:client.post('/api/term/close',json={'name':name})
