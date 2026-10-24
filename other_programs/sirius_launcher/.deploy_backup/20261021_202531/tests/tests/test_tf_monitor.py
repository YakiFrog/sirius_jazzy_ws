"""TF state, bounded retention, conflicts, cycles and authenticated access."""
import importlib
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tf_monitor import TransformStore


def add(store, parent, child, now=0, static=False, publisher='one'):
    store.ingest(parent, child, static, publisher, 123.5, (1,2,3), (0,0,0,1), now)


def test_rates_static_retention_and_stale_receive_time():
    store = TransformStore()
    for t in (0,.1,.2): add(store,'map','odom',t)
    add(store,'base','laser',static=True)
    data = store.snapshot(.3, {'one':'/tf_source'})
    edge = next(e for e in data['edges'] if not e['static'])
    assert math.isclose(edge['rate_hz'],10)
    assert edge['publisher']=='/tf_source' and not edge['stale']
    assert len(store.edges)==2
    data = store.snapshot(4)
    assert next(e for e in data['edges'] if not e['static'])['stale']
    assert not next(e for e in data['edges'] if e['static'])['stale']
    assert [e['child'] for e in store.snapshot(61)['edges']]==['laser']


def test_multiple_publishers_and_real_cycle_members():
    store = TransformStore()
    add(store,'a','b');add(store,'b','a');add(store,'b','c')
    add(store,'a','b',publisher='two')
    data=store.snapshot(1)
    assert data['conflicts']==['b']
    assert data['cycles']==['a','b']  # c is a descendant, not part of the cycle.
    assert len(data['edges'])==4
    assert data['roots']==[]


def test_deep_tree_and_invalid_or_excess_transforms_are_bounded():
    store=TransformStore(max_edges=1500)
    for i in range(1502): add(store,str(i),str(i+1))
    store.ingest('bad','nan',False,'one',0,(float('nan'),0,0),(0,0,0,1),0)
    add(store,'','empty')
    data=store.snapshot(1)
    assert len(data['edges'])==1500 and data['at_capacity']
    assert data['cycles']==[] and data['roots']==['0']


def test_tf_api_auth_domain_and_cache_policy(monkeypatch):
    web=importlib.import_module('web_server')
    class FakeMonitor:
        def snapshot(self,domain):
            assert domain=='53'
            return {'available':True,'domain':domain,'edges':[]}
    monkeypatch.setattr(web,'TF_MONITOR',FakeMonitor())
    monkeypatch.setattr(web,'DEFAULT_DOMAIN','53')
    monkeypatch.setattr(web,'PASSWORD','test-password')
    client=web.app.test_client()
    assert client.get('/api/tf').status_code==401
    client.set_cookie('pw','test-password')
    result=client.get('/api/tf')
    assert result.json['available'] and result.json['domain']=='53'
    assert result.headers['Cache-Control']=='no-store'


def test_warming_receiver_does_not_block_http_snapshots(monkeypatch):
    import threading
    import time
    from tf_monitor import TFMonitor
    monitor=TFMonitor()
    entered=threading.Event()
    def wait_for_close():
        entered.set();monitor._stop.wait(2)
    monkeypatch.setattr(monitor,'_run',wait_for_close)
    try:
        assert not monitor.snapshot('53')['available']
        assert entered.wait(1)
        start=time.monotonic()
        for _ in range(30): assert monitor.snapshot('53')['domain']=='53'
        assert time.monotonic()-start<.2
    finally:
        monitor.close()
        monitor._thread.join(1)
