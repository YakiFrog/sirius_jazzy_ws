import sys
from pathlib import Path
from types import SimpleNamespace
import pytest
from flask import Flask

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from lifecycle_protocol import validate_command
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


def test_read_only_get_and_explicit_post():
    calls=[]
    service=SimpleNamespace(snapshot=lambda:[],start=lambda data,domain:(calls.append((data,domain)) or {'id':'job'}))
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
