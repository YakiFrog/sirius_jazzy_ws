import importlib
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace as Message

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from battery_monitor import BatteryStore,BatteryReceiver,normalize


def standard(**values):
    return Message(voltage=values.get('voltage',24.2),percentage=values.get('percentage',float('nan')),
                   current=values.get('current',0),temperature=values.get('temperature',float('nan')),
                   power_supply_status=values.get('status',0),present=values.get('present',True))


def motor(**values):
    return Message(voltage=values.get('voltage',5.8),voltage_internal=values.get('internal',13.8),
                   battery_current_ch1=values.get('current',-.1),battery_current_ch2=values.get('current',-.1),temperature=32,
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
    store.ingest('/roboteq/status','RoboteqStatus',motor(voltage=math.nan,internal=math.nan,current=math.nan,flags=None),0)
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


def test_battery_and_driver_keep_separate_measurements_and_freshness():
    store=BatteryStore()
    store.ingest('/battery_state','BatteryState',standard(percentage=1,voltage=math.nan,current=math.nan),0)
    store.ingest('/roboteq/status','RoboteqStatus',motor(),1)
    data=store.snapshot(2)
    assert data['battery']['percentage']==100 and data['battery']['voltage'] is None
    assert data['driver']['voltage']==5.8 and data['driver']['internal_voltage']==13.8
    assert data['driver']['emergency_stop'] is True
    store.ingest('/roboteq/status','RoboteqStatus',motor(flags=0),6)
    data=store.snapshot(6)
    assert data['battery']['state']=='stale' and data['driver']['state']=='live'
    assert data['battery']['percentage']==100 and data['driver']['emergency_stop'] is False
    store.ingest('/battery_state','BatteryState',standard(percentage=.98),12)
    data=store.snapshot(12)
    assert data['battery']['state']=='live' and data['driver']['state']=='stale'


def test_zero_or_missing_drive_voltage_does_not_hide_estop():
    store=BatteryStore()
    for voltage in (0,math.nan):
        store.ingest('/roboteq/status','RoboteqStatus',motor(voltage=voltage),0)
        driver=store.snapshot(0)['driver']
        assert driver['state']=='live' and driver['emergency_stop'] is True
        assert driver['internal_voltage']==13.8
    store.ingest('/roboteq/status','RoboteqStatus',motor(voltage=0),1)
    assert store.snapshot(1)['driver']['voltage']==0


def test_both_sources_remain_visible_before_first_message():
    data=BatteryStore().snapshot(0)
    assert data['battery']['state']==data['driver']['state']=='waiting'


def solix(**values):
    return Message(data=json.dumps({'status':'connected','battery_level':75,'total_input':0,
                                   'total_output':64,'time_remaining':2.25,'stamp':1000,**values}))


def test_solix_duration_ports_and_charge_estimate():
    data=normalize(solix(ports={'ac_output':64,'dc_output':0,'usb_c1':-1}),'SolixStatus')
    assert data['runtime_hours']==2.25 and data['charge_remaining_hours'] is None
    assert data['ports']['dc_output']==0 and data['ports']['usb_c1'] is None
    data=normalize(solix(total_input=100,total_output=10,time_remaining=1.5),'SolixStatus')
    assert data['charge_remaining_hours']==1.5 and data['runtime_hours'] is None
    assert data['charge_complete_at']==6400
    for remaining in [6553.5,-1,math.nan,math.inf,0]:
        data=normalize(solix(time_remaining=remaining),'SolixStatus')
        assert data['runtime_hours'] is None and data['charge_remaining_hours'] is None


def test_solix_is_independent_and_never_replaces_battery_or_driver():
    store=BatteryStore()
    store.ingest('/battery_state','BatteryState',standard(percentage=.75),0)
    store.ingest('/roboteq/status','RoboteqStatus',motor(),0)
    store.ingest('/sirius/battery_status','SolixStatus',solix(battery_level=100),1)
    data=store.snapshot(2)
    assert data['source_type']=='BatteryState' and data['battery']['percentage']==75
    assert data['details']['percentage']==100 and data['driver']['emergency_stop'] is True
    store.ingest('/battery_state','BatteryState',standard(percentage=.74),7)
    data=store.snapshot(7)
    assert data['battery']['state']=='live' and data['details']['state']=='stale'


def test_bad_json_and_non_finite_fields_do_not_break_shared_receiver():
    store=BatteryStore()
    for raw in ['{','[]','null']:
        store.ingest('/sirius/battery_status','SolixStatus',Message(data=raw),0)
        assert store.snapshot(0)['details']['state']!='live'
    store.ingest('/sirius/battery_status','SolixStatus',solix(total_output=math.inf,temperature=math.nan),0)
    json.dumps(store.snapshot(0),allow_nan=False)
    assert store.snapshot(0)['details']['output_power'] is None


def test_string_discovery_only_subscribes_to_battery_status():
    assert BatteryReceiver.message_type('/sirius/battery_status',['std_msgs/msg/String'])=='std_msgs/msg/String'
    assert BatteryReceiver.message_type('/robot/sirius/battery_status',['std_msgs/msg/String'])=='std_msgs/msg/String'
    assert BatteryReceiver.message_type('/llm_instruction',['std_msgs/msg/String']) is None
    assert BatteryReceiver.message_type('/sirius/battery_status/other',['std_msgs/msg/String']) is None


def test_battery_api_auth_and_dead_receiver_is_never_live(monkeypatch):
    web=importlib.import_module('web_server')
    class Monitor:
        cached={'state':'live','voltage':24.2,'battery':{'state':'live'},'driver':{'state':'live'},'details':{'state':'live'}}
        def snapshot(self,domain):
            return {'available':False,'domain':domain,'error':'receiver stopped',
                    'battery':self.cached}
    monkeypatch.setattr(web,'TF_MONITOR',Monitor());monkeypatch.setattr(web,'PASSWORD','test')
    monkeypatch.setattr(web,'DEFAULT_DOMAIN','53')
    client=web.app.test_client()
    assert client.get('/api/battery').status_code==401
    client.set_cookie('pw','test');result=client.get('/api/battery')
    assert result.json['state']=='receiver_error' and result.json['voltage']==24.2
    assert result.json['domain']=='53' and result.headers['Cache-Control']=='no-store'
    assert result.json['battery']['state']==result.json['driver']['state']=='receiver_error'
    assert Monitor.cached['battery']['state']==Monitor.cached['driver']['state']=='live'
    assert result.json['details']['state']=='receiver_error' and Monitor.cached['details']['state']=='live'
