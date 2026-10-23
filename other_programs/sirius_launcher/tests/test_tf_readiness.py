"""Sirius TF readiness classification."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tf_readiness import tf_readiness


def edge(parent, child, static=False, stale=False):
    return {'parent': parent, 'child': child, 'static': static, 'stale': stale}


def snapshot(edges, **extra):
    frames = sorted({e[k] for e in edges for k in ('parent', 'child')})
    return {'available': True, 'frames': frames, 'edges': edges,
            'conflicts': [], 'cycles': [], **extra}


def nuc_tree():
    return snapshot([
        edge('sirius3/odom', 'camera_init', static=True),
        edge('camera_init', 'body'),
        edge('body', 'sirius3/base_footprint', static=True),
        edge('sirius3/base_footprint', 'sirius3/base_link', static=True),
    ])


def test_nuc_style_tree_is_ready():
    result = tf_readiness(nuc_tree())
    assert result['state'] == 'ok' and result['ready']
    assert result['dynamic_path'] and not result['missing'] and not result['disconnected']


def test_unavailable_and_empty_snapshots_are_not_ready():
    assert tf_readiness(None)['state'] == 'unknown'
    result = tf_readiness({'available': True, 'frames': [], 'edges': []})
    assert result['state'] == 'waiting' and result['missing']


def test_conflicts_and_cycles_take_precedence():
    data = nuc_tree()
    data['conflicts'] = ['sirius3/base_link']
    assert tf_readiness(data)['state'] == 'error'
    data = nuc_tree()
    data['cycles'] = ['sirius3/base_link']
    assert tf_readiness(data)['state'] == 'error'


def test_missing_required_frame_waits():
    data = nuc_tree()
    data['edges'] = [e for e in data['edges'] if e['child'] != 'sirius3/base_link']
    data['frames'] = [f for f in data['frames'] if f != 'sirius3/base_link']
    result = tf_readiness(data)
    assert result['state'] == 'waiting'
    assert result['missing'] == ['sirius3/base_link']


def test_disconnected_base_waits():
    data = snapshot([
        edge('sirius3/odom', 'camera_init', static=True),
        edge('camera_init', 'body'),
        edge('body', 'sirius3/base_footprint', static=True),
        edge('elsewhere', 'sirius3/base_link', static=True),
    ])
    data['frames'].append('elsewhere')
    result = tf_readiness(data)
    assert result['state'] == 'waiting'
    assert result['disconnected'] == [['sirius3/base_footprint', 'sirius3/base_link']]


def test_only_static_odom_path_waits_for_dynamic_tf():
    data = snapshot([
        edge('sirius3/odom', 'sirius3/base_footprint', static=True),
        edge('sirius3/base_footprint', 'sirius3/base_link', static=True),
    ])
    result = tf_readiness(data)
    assert result['state'] == 'waiting'
    assert result['dynamic_path'] is False


def test_stale_dynamic_edge_does_not_satisfy_readiness():
    data = nuc_tree()
    next(e for e in data['edges'] if not e['static'])['stale'] = True
    result = tf_readiness(data)
    assert result['state'] == 'waiting'
    assert result['dynamic_path'] is False


def test_tf_api_includes_readiness(monkeypatch):
    import importlib
    web = importlib.import_module('web_server')
    data = nuc_tree()
    class FakeMonitor:
        def snapshot(self, domain):
            assert domain == '53'
            return dict(data)
    monkeypatch.setattr(web, 'TF_MONITOR', FakeMonitor())
    monkeypatch.setattr(web, 'DEFAULT_DOMAIN', '53')
    monkeypatch.setattr(web, 'PASSWORD', 'test-password')
    client = web.app.test_client()
    client.set_cookie('pw', 'test-password')
    result = client.get('/api/tf')
    assert result.status_code == 200
    assert result.json['readiness']['state'] == 'ok'
