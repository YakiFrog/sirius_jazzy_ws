import sys
from pathlib import Path
from types import SimpleNamespace
import pytest
from flask import Flask

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from lifecycle_protocol import REQUIRED_NODES, validate_command
from lifecycle_service import register_lifecycle,LifecycleService


def client(auth=True,service=None):
    app=Flask(__name__);app.config['TESTING']=True
    register_lifecycle(app,lambda:auth,lambda:'53',lambda:{'available':True,'navigation':{'lifecycle':{'nodes':{}}}},service)
    return app.test_client()


@pytest.mark.parametrize('data',[
    None,{},dict(node='roboteq',action='activate',expected_state=2),
    dict(node='amcl',action='shutdown',expected_state=2),
    dict(node='amcl',action='activate',expected_state=True),
    dict(node='amcl',action='activate',expected_state=3)])
def test_only_scoped_legal_requests(data):
    with pytest.raises(ValueError):validate_command(data)


def test_auth_and_origin_before_any_operation():
    assert client(False).get('/api/lifecycle').status_code==401
    assert client(False).post('/api/lifecycle/transition',json={}).status_code==401
    assert client().post('/api/lifecycle/transition',json={},headers={'Origin':'https://elsewhere.invalid'}).status_code==403
    assert client(False).post('/api/lifecycle/auto-activation', json={'enabled': True}).status_code == 401
    assert client().post('/api/lifecycle/auto-activation', json={'enabled': True},
                         headers={'Sec-Fetch-Site': 'cross-site'}).status_code == 403


def test_read_only_get_and_explicit_post():
    calls=[]
    service=SimpleNamespace(snapshot=lambda:[],auto_status=lambda: {'enabled': False},
                            start=lambda data,domain:(calls.append((data,domain)) or {'id':'job'}))
    c=client(service=service)
    assert c.get('/api/lifecycle').json['available'] and not calls
    r=c.post('/api/lifecycle/transition',json=dict(node='amcl',action='activate',expected_state=2))
    assert r.status_code==202 and calls[0][1]=='53'


def test_parallel_operations_are_rejected_without_retries(monkeypatch):
    started=[]
    monkeypatch.setattr('lifecycle_service.threading.Thread',lambda **kwargs:SimpleNamespace(start=lambda:started.append(kwargs)))
    s=LifecycleService();cmd=dict(node='amcl',action='activate',expected_state=2)
    assert s.start(cmd,'53')['status']=='running'
    with pytest.raises(RuntimeError):s.start(cmd,'53')
    assert len(started)==1


def test_worker_result_is_retained(monkeypatch):
    monkeypatch.setattr('lifecycle_service.subprocess.run',lambda *a,**k:SimpleNamespace(stdout='{"ok":true,"state":{"id":3,"label":"active"}}\n'))
    s=LifecycleService();job=dict(node='amcl',action='activate',expected_state=2,domain='228',status='running')
    s._run(job);assert job['status']=='done' and job['result']['state']['id']==3


def test_auto_activation_is_explicit_and_off_by_default(monkeypatch):
    started = []
    monkeypatch.setattr('lifecycle_service.threading.Thread',
                        lambda **kwargs: SimpleNamespace(start=lambda: started.append(kwargs)))
    c = client()
    assert not c.get('/api/lifecycle').json['auto_activation']['enabled']
    path = '/api/lifecycle/auto-activation'
    assert c.post(path, json={'enabled': 'yes'}).status_code == 400
    assert c.post(path, json={'enabled': True}).json['auto_activation']['enabled']
    assert c.post(path, json={'enabled': True}).status_code == 200
    assert len(started) == 1
    assert not c.post(path, json={'enabled': False}).json['auto_activation']['enabled']


def test_automatic_activation_uses_fresh_stable_required_states_once(monkeypatch):
    monkeypatch.setattr('lifecycle_service.threading.Thread', lambda **kwargs: SimpleNamespace(start=lambda: None))
    s = LifecycleService()
    rows = {name: {'fresh': True, 'value': {'id': 3}, 'observed_seconds': 10}
            for name in REQUIRED_NODES}
    controller = rows['controller_server']
    controller.update(value={'id': 2}, observed_seconds=4)
    rows['docking_server'] = {'fresh': True, 'value': {'id': 2}, 'observed_seconds': 10}
    data = {'available': True, 'domain': '53', 'navigation': {'lifecycle': {'nodes': rows}}}
    assert s.poll_auto(data, '53') is None
    s.set_auto_mode(True, lambda: data, lambda: '53')
    assert s.poll_auto(data, '53') is None
    controller['observed_seconds'] = 5
    assert s.poll_auto(data, '56') is None
    controller['fresh'] = False
    assert s.poll_auto(data, '53') is None
    controller['fresh'] = True
    job = s.poll_auto(data, '53')
    assert (job['node'], job['action'], job['source']) == ('controller_server', 'activate', 'automatic')
    assert s.poll_auto(data, '53') is None
    s.operations[0]['status'] = 'done'
    assert s.poll_auto(data, '53') is None
    controller['value'] = {'id': 3}
    assert s.poll_auto(data, '53') is None
    controller['value'] = {'id': 2}
    assert s.poll_auto(data, '53')['id'] != job['id']


def test_failed_automatic_operation_and_manual_deactivation_turn_mode_off(monkeypatch):
    monkeypatch.setattr('lifecycle_service.threading.Thread', lambda **kwargs: SimpleNamespace(start=lambda: None))
    monkeypatch.setattr('lifecycle_service.subprocess.run',
                        lambda *args, **kwargs: SimpleNamespace(stdout='{"ok":false,"error":"timeout"}\n'))
    s = LifecycleService()
    s.set_auto_mode(True, lambda: {}, lambda: '53')
    s._run(dict(node='amcl', action='activate', expected_state=2, domain='53', source='automatic'))
    assert not s.auto_status()['enabled']
    s.set_auto_mode(True, lambda: {}, lambda: '53')
    s.start(dict(node='amcl', action='deactivate', expected_state=3), '53')
    assert not s.auto_status()['enabled']
