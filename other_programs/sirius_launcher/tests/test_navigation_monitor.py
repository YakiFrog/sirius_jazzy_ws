import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation_monitor import NavigationState


def test_detects_actual_recorded_lock_mismatch_without_driver_stop():
    s = NavigationState()
    for k, v in [('safe', {'moving': True}), ('lock', True), ('driver', False)]:
        s.ingest(k, v, 10)
    assert s.snapshot(11)['mismatch']
    s.ingest('driver', True, 11)
    assert not s.snapshot(11)['mismatch']
    assert '緊急停止が有効' in s.snapshot(11)['summary']


def test_stale_data_is_not_current_fault_and_zero_motion_is_not_mismatch():
    s = NavigationState()
    for k, v in [('safe', {'moving': True}), ('lock', True), ('driver', False)]:
        s.ingest(k, v, 10)
    assert not s.snapshot(14)['mismatch']
    s.ingest('safe', {'moving': False}, 14)
    assert not s.snapshot(14)['mismatch']


def test_output_is_distinct_from_vehicle_motion():
    s = NavigationState()
    s.ingest('output', {'moving': True}, 10)
    assert '動きは未確認' in s.snapshot(10)['summary']
    s.ingest('motion', True, 10)
    assert '動きを確認' in s.snapshot(10)['summary']


def test_preflight_uses_configured_scan_and_expires_read_only_statuses():
    s = NavigationState()
    s.ingest('lifecycle/bt_navigator', {'active': True, 'label': 'active'}, 10)
    s.ingest('scan_topic', '/scan3', 10)
    s.ingest('laser//scan3', {'samples': 898, 'valid': True, 'stamp_current': True}, 10)
    s.ingest('driver_communication', True, 10)
    s.ingest('driver', True, 10)  # Held stop does not erase preparation evidence.
    data = s.snapshot(11)['preflight']
    assert data['nodes']['bt_navigator']['value']['active']
    assert data['scan']['topic'] == '/scan3' and data['scan']['fresh']
    assert data['communication']['value'] is True
    data = s.snapshot(17)['preflight']
    assert not data['nodes']['bt_navigator']['fresh']
    assert not data['scan']['fresh']
    assert not data['communication']['fresh']


def test_lifecycle_history_records_changes_and_costmap_data_separately():
    s=NavigationState()
    s.ingest('lifecycle/global_costmap/global_costmap',{'id':2,'label':'inactive','active':False},10)
    s.ingest('lifecycle/global_costmap/global_costmap',{'id':13,'label':'activating','active':False},11)
    s.ingest('lifecycle/global_costmap/global_costmap',{'id':3,'label':'active','active':True},12)
    s.ingest('lifecycle/global_costmap/global_costmap',{'id':3,'label':'active','active':True},13)
    d=s.snapshot(14)['lifecycle']
    assert len(d['history'])==3
    assert d['nodes']['global_costmap/global_costmap']['value']['id']==3
    assert not d['costmaps']['global']['fresh']  # Active does not prove receipt.
    s.ingest('costmap/global',{'valid':True,'width':20,'height':30,'frame':'map'},14)
    assert s.snapshot(15)['lifecycle']['costmaps']['global']['fresh']
    assert not s.snapshot(21)['lifecycle']['nodes']['global_costmap/global_costmap']['fresh']
