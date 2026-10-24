import importlib
import math
import sys
from pathlib import Path
from types import SimpleNamespace as Message

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from battery_monitor import BatteryStore,normalize


def standard(**values):
    return Message(voltage=values.get('voltage',24.2),percentage=values.get('percentage',float('nan')),
                   current=values.get('current',0),temperature=values.get('temperature',float('nan')),
                   power_supply_status=values.get('status',0),present=values.get('present',True))


def motor(**values):
    return Message(voltage=values.get('voltage',5.8),voltage_internal=13.8,
                   battery_current_ch1=-.1,battery_current_ch2=-.1,temperature=32,
                   fault_flags=values.get('flags',16))


def test_unknown_percent_stays_unknown_and_estop_is_separate():
    data=normalize(motor(),'RoboteqStatus')
    assert math.isclose(data['voltage'],5.8)
    assert data['percentage'] is None and data['charging'] is None
    assert data['emergency_stop'] is True and data['fault_flags']==16
    assert math.isclose(data['current'],-.2)


def test_standard_percentage_nan_invalid_and_zero_percent():
    assert normalize(standard(),'BatteryState')['percentage'] is None
    assert normalize(standard(percentage=0),'BatteryState')['percentage']==0
    assert normalize(standard(percentage=.78),'BatteryState')['percentage']==78
    assert normalize(standard(percentage=1.2,voltage=float('inf')),'BatteryState')['voltage'] is None
    assert normalize(standard(percentage=-1),'BatteryState')['percentage'] is None
    assert normalize(standard(status=1),'BatteryState')['charging']=='充電中'


def test_missing_invalid_stale_and_reconnect_states():
    store=BatteryStore()
    assert store.snapshot(0)['state']=='waiting'
    store.ingest('/roboteq/status','RoboteqStatus',motor(voltage=0),0)
    data=store.snapshot(1)
    assert data['state']=='invalid' and data['voltage'] is None and data['current'] is None
    store.ingest('/roboteq/status','RoboteqStatus',motor(),2)
    assert store.snapshot(3)['state']=='live'
    data=store.snapshot(8)
    assert data['state']=='stale' and data['age_seconds']==6 and data['voltage']==5.8
    store.ingest('/roboteq/status','RoboteqStatus',motor(flags=0),9)
    assert store.snapshot(9)['state']=='live' and not store.snapshot(9)['emergency_stop']


def test_source_priority_falls_back_to_fresh_driver():
    store=BatteryStore()
    store.ingest('/battery','BatteryState',standard(percentage=.75),0)
    store.ingest('/robot/roboteq/status','RoboteqStatus',motor(),1)
    assert store.snapshot(2)['topic']=='/battery'
    store.ingest('/robot/roboteq/status','RoboteqStatus',motor(),7)
    assert store.snapshot(7)['topic']=='/robot/roboteq/status'
    store.ingest('/battery','BatteryState',standard(present=False),8)
    assert store.snapshot(8)['topic']=='/robot/roboteq/status'
    single=BatteryStore();single.ingest('/battery','BatteryState',standard(present=False),0)
    assert single.snapshot(0)['state']=='absent'


def test_battery_api_auth_and_dead_receiver_is_never_live(monkeypatch):
    web=importlib.import_module('web_server')
    class Monitor:
        def snapshot(self,domain):
            return {'available':False,'domain':domain,'error':'receiver stopped',
                    'battery':{'state':'live','voltage':24.2}}
    monkeypatch.setattr(web,'TF_MONITOR',Monitor());monkeypatch.setattr(web,'PASSWORD','test')
    monkeypatch.setattr(web,'DEFAULT_DOMAIN','53')
    client=web.app.test_client()
    assert client.get('/api/battery').status_code==401
    client.set_cookie('pw','test');result=client.get('/api/battery')
    assert result.json['state']=='receiver_error' and result.json['voltage']==24.2
    assert result.json['domain']=='53' and result.headers['Cache-Control']=='no-store'
