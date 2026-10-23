from pathlib import Path
import json
import sys

import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import navigation_recording as recording


def test_navigation_topics_cover_lio_tf_semantic_and_costmap_updates():
    assert len(recording.TOPICS)==len(set(recording.TOPICS))
    assert {'/Odometry','/fastlio/diagnostics','/tf','/tf_static','/robot_description',
        '/sam3/static_colored_map_grid','/sam3/static_semantic_cost_grid',
        '/global_costmap/costmap_raw_updates','/local_costmap/costmap_updates',
        '/cmd_vel_web','/cmd_vel_nav'}<=set(recording.TOPICS)
    assert not {'/velodyne_points','/velodyne_points_leveled','/cloud_registered'}&set(recording.TOPICS)
    root=Path(__file__).resolve().parents[3]
    qos=yaml.safe_load((root/'params/rosbag_navigation_qos.yaml').read_text())
    for topic in ['/tf_static','/map','/sam3/static_colored_map_grid','/sam3/static_semantic_cost_grid',
                  '/global_costmap/costmap','/local_costmap/costmap_raw']:
        assert qos[topic]['durability']=='transient_local'


def test_map_assets_are_preserved_and_recorded_with_runtime_parameters(tmp_path,monkeypatch):
    maps=tmp_path/'maps';maps.mkdir()
    prefix=maps/'semantic';pgm=Path(str(prefix)+'.colored.pgm');pgm.write_bytes(b'P5\n1 1\n255\n\x07')
    palette=Path(str(prefix)+'.colored.json');palette.write_text('{"palette":[[0,0,0]]}')
    image=maps/'structural.pgm';image.write_bytes(b'P5\n1 1\n255\n\xff')
    map_yaml=maps/'structural.yaml';map_yaml.write_text('image: structural.pgm\nresolution: 0.05\n')
    def query(*args):
        if args==('node','list'):return '/sam3_colored_map_loader\n/map_server\n/laser_mapping'
        if args[:2]==('param','dump'):return args[2]+':\n  ros__parameters: {}'
        if args[-1]=='map_path':return str(prefix)
        if args[-1]=='yaml_filename':return str(map_yaml)
    monkeypatch.setattr(recording,'query',query)
    report=recording.snapshot(tmp_path/'capture',tmp_path/'workspace')
    rows=[row for item in report['maps'] for row in item['files']]
    assert len(rows)==4 and not report['warnings']
    assert len(report['parameters'])==3
    for row in rows:assert Path(row['source']).read_bytes()==Path(row['copy']).read_bytes()
    assert json.loads((tmp_path/'capture/recording_context.json').read_text())['topics']==recording.TOPICS


def test_no_active_map_loader_is_reported_without_inventing_assets(tmp_path,monkeypatch):
    monkeypatch.setattr(recording,'query',lambda *args:None)
    report=recording.snapshot(tmp_path/'capture',tmp_path/'workspace')
    assert report['warnings'] and not report['maps']
