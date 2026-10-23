import importlib
import json
import sys
from pathlib import Path

import pytest
from flask import Flask

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from documentation_notes import NoteConflict, NoteStore, register_documentation_notes


@pytest.fixture
def client(tmp_path):
    app = Flask(__name__)
    app.config['TESTING'] = True
    register_documentation_notes(app, lambda: True, tmp_path / 'notes')
    return app.test_client()


def test_markdown_persists_across_store_instances_and_full_text_search(tmp_path):
    store = NoteStore(tmp_path)
    body = '# 地図撮り\n- [ ] velodyne\n```bash\nrecord_3dmap\n```\n' + 'x' * 200 + '\n深い位置の検索語'
    note = store.save({'title': '実験手順', 'body': body})
    other = NoteStore(tmp_path)
    assert other.get(note['id'])['body'] == body
    assert other.list('検索語')[0]['id'] == note['id']
    assert other.list('存在しない') == []
    assert note['format'] == 'markdown'


def test_stale_update_and_delete_cannot_overwrite_another_device(tmp_path):
    store = NoteStore(tmp_path)
    note = store.save({'title': 'A', 'body': 'first'})
    newer = store.save({'title': 'B', 'body': 'second', 'revision': 1}, note['id'])
    with pytest.raises(NoteConflict):
        store.save({'title': 'C', 'body': 'stale', 'revision': 1}, note['id'])
    with pytest.raises(NoteConflict):
        store.delete(note['id'], 1)
    assert store.get(note['id']) == newer
    store.delete(note['id'], 2)
    assert store.list() == []
    assert json.loads(next((tmp_path / '.trash').glob('*.json')).read_text())['body'] == 'second'


def test_failed_atomic_replace_preserves_existing_note(tmp_path, monkeypatch):
    store = NoteStore(tmp_path)
    note = store.save({'title': 'A', 'body': 'keep'})
    module = importlib.import_module('documentation_notes')
    def fail(*args):
        raise OSError('disk full')
    monkeypatch.setattr(module.os, 'replace', fail)
    with pytest.raises(OSError):
        store.save({'title': 'B', 'body': 'lost', 'revision': 1}, note['id'])
    assert store.get(note['id'])['body'] == 'keep'
    assert not list(tmp_path.glob('*.tmp'))


def test_api_crud_validation_and_conflict(client):
    assert client.post('/api/documentation/notes', json=[]).status_code == 400
    assert client.post('/api/documentation/notes', json={'title': '', 'body': ''}).status_code == 400
    note = client.post('/api/documentation/notes', json={'title': '地図撮り', 'body': '- [ ] imu'}).get_json()['note']
    path = '/api/documentation/notes/' + note['id']
    assert client.get(path).get_json()['note'] == note
    assert client.put(path, json={'title': '更新', 'body': 'x', 'revision': 1}).status_code == 200
    assert client.put(path, json={'title': '古い', 'body': 'x', 'revision': 1}).status_code == 409
    assert client.delete(path, json={'revision': 1}).status_code == 409
    assert client.delete(path, json={'revision': 2}).status_code == 200
    assert client.get(path).status_code == 404
    assert client.get('/api/documentation/notes/invalid').status_code == 400
    assert client.post('/api/documentation/notes', data=b'x' * 500001).status_code == 400


def test_auth_applies_to_reads_and_writes(tmp_path):
    app = Flask(__name__)
    register_documentation_notes(app, lambda: False, tmp_path)
    client = app.test_client()
    for method, path in [('get', '/api/documentation/notes'), ('post', '/api/documentation/notes'),
                         ('get', '/api/documentation/notes/' + 'a' * 32)]:
        assert getattr(client, method)(path).status_code == 401
    assert not list(tmp_path.glob('*.json'))
