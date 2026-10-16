"""Exercise bag deserialization, calibration and weak-axis protection end to end."""
import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

SCRIPT = Path(__file__).resolve().parents[1]/'offline_3d_mapping.py'
sys.path.insert(0, str(SCRIPT.parent))
spec = importlib.util.spec_from_file_location('offline_3d_mapping', SCRIPT)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def read_ply(path):
    raw = path.read_bytes()
    header, body = raw.split(b'end_header\n', 1)
    dtype = [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]
    if b'property uchar red' in header:
        dtype += [('red', 'u1'), ('green', 'u1'), ('blue', 'u1')]
    return np.frombuffer(body, dtype=dtype)


def test_interpolation_does_not_extrapolate_or_bridge_gaps():
    poses = [m.transform(translation=[0, 0, 0]), m.transform(translation=[2, 0, 0])]
    assert m.interpolate_pose(.5, np.array([0., 1.]), poses, .2) is None
    assert m.interpolate_pose(-.01, np.array([0., 1.]), poses, 2) is None
    assert m.interpolate_pose(1.01, np.array([0., 1.]), poses, 2) is None
    np.testing.assert_allclose(m.interpolate_pose(.5, np.array([0., 1.]), poses, 1)[:3, 3], [1, 0, 0])


@pytest.mark.parametrize('wheel_fusion', ['auto', 'already-fused'])
def test_recorded_fastlio_is_used_and_shared_imu_is_not_counted_twice(synthetic_bag, wheel_fusion):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from nav_msgs.msg import Odometry
    import shutil
    bag = synthetic_bag.parent/'with_lio'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    source = m.reader(synthetic_bag)
    for i, topic in enumerate(source.get_all_topics_and_types()):
        writer.create_topic(rosbag2_py.TopicMetadata(id=i, name=topic.name, type=topic.type, serialization_format='cdr'))
    writer.create_topic(rosbag2_py.TopicMetadata(id=99, name='/Odometry', type='nav_msgs/msg/Odometry', serialization_format='cdr'))
    while source.has_next():
        topic, data, ns = source.read_next()
        writer.write(topic, data, ns)
        if topic == '/odom':
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = 'camera_init', 'body'
            odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(ns, 1_000_000_000)
            odom.pose.pose.orientation.w = 1.
            odom.pose.pose.position.x = 17.+(ns-10_000_000_000)*1e-9
            odom.pose.pose.position.z = .3  # FAST-LIO body is IMU, not footprint.
            writer.write('/Odometry', serialize_message(odom), ns)
    del writer
    shutil.copytree(synthetic_bag/'capture', bag/'capture')
    assert m.main([str(bag), '--lio-wheel-fusion', wheel_fusion, '--no-loop']) == 0
    report = json.loads((bag/'offline_3d/report.json').read_text())
    initial = np.loadtxt(bag/'offline_3d/trajectory_initial.csv', delimiter=',', skiprows=1)
    assert report['lio']['used_for_mapping'] is True
    assert report['imu_edges'] == 0
    np.testing.assert_allclose(initial[-1, 1:4], [2, 0, 0], atol=1e-6)
    assert report['lio']['wheel_already_in_frontend'] == (wheel_fusion == 'already-fused')
    assert report['optimization']['residuals']['wheel']['edges'] == (0 if wheel_fusion == 'already-fused' else report['keyframes']-1)


def test_weak_lidar_axis_cannot_pull_wheel_trajectory_backwards():
    # Parallel wall and floor have no normal component along the travel axis.
    h = np.zeros((120, 6))
    h[:60, 1] = 1
    h[60:, 2] = 1
    sqrt_info, _, weak = m.observable_information(h, .01, .05)
    assert weak >= 1
    assert np.linalg.norm(sqrt_info[:, 0]) < 1e-10
    initial = [m.transform(translation=[i, 0, 0]) for i in range(5)]
    wrong_icp = m.transform(translation=[-10, 0, 0])
    edges = [(i, i+1, m.transform(translation=[1, 0, 0]), (wrong_icp, sqrt_info), None)
             for i in range(4)]
    args = SimpleNamespace(wheel_position_sigma=.03, wheel_rotation_sigma=.03,
                           imu_rotation_sigma=.05, max_iterations=30)
    poses, report = m.optimize_poses(initial, edges, args)
    assert report['success']
    np.testing.assert_allclose([p[0, 3] for p in poses], range(5), atol=1e-6)


def test_observable_information_keeps_translation_beside_far_lever_arms():
    # A small far wall patch and a huge farther floor. Without normalizing the
    # rotation block by a representative length, the far points set the 1% eigen
    # threshold and observable translation directions disappear.
    a, b = np.meshgrid(np.linspace(-2, 2, 40), np.linspace(20, 80, 40))
    floor = np.column_stack([a.ravel(), b.ravel(), np.zeros(a.size)])
    wall = np.column_stack([np.full(200, 10.), np.linspace(-1, 1, 200), np.linspace(0, 2, 200)])
    points = np.vstack([floor, wall])
    normals = np.vstack([np.tile([0., 0., 1.], (len(floor), 1)), np.tile([1., 0., 0.], (len(wall), 1))])
    h = np.column_stack([normals, np.cross(points, normals)])
    sqrt_info, diagnostics, weak = m.observable_information(h, .01, .05, scale=30.)
    # Wall translation x and floor translation z must stay observable.
    assert np.linalg.norm(sqrt_info @ np.array([1., 0., 0., 0., 0., 0.])) > 0
    assert np.linalg.norm(sqrt_info @ np.array([0., 0., 1., 0., 0., 0.])) > 0
    assert 'x' not in diagnostics['dropped_axes'] and 'z' not in diagnostics['dropped_axes']
    # Weights are mapped back to the original meter/radian parameterization.
    e = np.array([.1, -.2, .3, .01, -.02, .03])
    scaled = np.array([1., 1., 1., 30., 30., 30.]) * e
    expected = np.zeros(6)
    keep = diagnostics['keep']
    expected[keep] = (np.sqrt(diagnostics['eigenvalues'][keep])/.05
                      * (diagnostics['vectors'][:, keep].T @ scaled))
    np.testing.assert_allclose(sqrt_info @ e, expected, atol=1e-9)
    assert weak == 6 - int(keep.sum())


def test_loop_closure_detects_revisit_and_skips_recent_pairs(tmp_path, monkeypatch):
    # 片道→同じ場所へ戻る経路。再訪ペアだけがループ候補になる。
    positions = [[k*0.5, 0., 0.] for k in range(21)] + [[(40-k)*0.5, 0.2, 0.] for k in range(21, 40)]
    poses = [m.transform(translation=p) for p in positions]
    times = np.arange(len(poses), dtype=float)  # 1秒間隔
    cache = tmp_path/'cache'
    cache.mkdir()
    for k in range(len(poses)):
        np.savez(cache/f'{k:06d}.npz', xyz=np.zeros((1, 3)), rgb=np.zeros((1, 3)))
    monkeypatch.setattr(m, 'icp_edge', lambda source, target, initial, args:
                        (initial, {'accepted': True, 'fitness': 1.0, 'rmse': 0.0}))
    args = SimpleNamespace(no_loop=False, loop_radius=1.0, loop_min_gap=5.0,
                           loop_min_keyframes=8, max_iterations=30)
    edges, diagnostics = m.find_loop_edges(poses, times, cache, args)
    assert edges, '再訪ペアがループ拘束として採用される'
    for i, j, wheel, lidar, imu in edges:
        assert j - i >= args.loop_min_keyframes and times[j] - times[i] >= args.loop_min_gap
        assert wheel is None and imu is None and lidar is not None
    assert all(d['type'] == 'loop' for d in diagnostics)
    # ノードごとに最大1本（重複候補で同じノードを使い回さない）
    used = [node for edge in edges for node in edge[:2]]
    assert len(used) == len(set(used))
    # --no-loop なら候補ゼロ
    args.no_loop = True
    assert m.find_loop_edges(poses, times, cache, args)[0] == []


def test_rejected_loop_candidate_does_not_consume_its_nodes(tmp_path, monkeypatch):
    # ノード0で最も近い候補(0,8)のICPが不採用でも、同じノード0を使う
    # 後続の再訪ペア(0,9)を検証して拘束にできること。
    positions = [[k*.5, 0., 0.] for k in range(8)] + [[.05*(k-7), .3, 0.] for k in range(8, 12)]
    poses = [m.transform(translation=p) for p in positions]
    times = np.arange(len(poses), dtype=float)
    cache = tmp_path/'cache'
    cache.mkdir()
    for k in range(len(poses)):
        marker = -999. if k == 8 else 1.
        np.savez(cache/f'{k:06d}.npz', xyz=np.array([[marker, 0., 0.]]), rgb=np.zeros((1, 3)))

    def fake_icp(source, target, initial, args):
        if source[0, 0] < 0:
            return None, {'accepted': False, 'fitness': 0.0, 'reason': 'insufficient overlap'}
        return initial, {'accepted': True, 'fitness': 1.0, 'rmse': 0.0}

    monkeypatch.setattr(m, 'icp_edge', fake_icp)
    args = SimpleNamespace(no_loop=False, loop_radius=1.0, loop_min_gap=5.0,
                           loop_min_keyframes=8, max_iterations=30)
    edges, diagnostics = m.find_loop_edges(poses, times, cache, args)
    assert (0, 8) in [(d['from'], d['to']) for d in diagnostics if not d['accepted']]
    assert (0, 9) in [(i, j) for i, j, *_ in edges], '不採用候補がノード0を塞いではいけない'


@pytest.fixture
def synthetic_bag(tmp_path):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu, PointField
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Header
    from sensor_msgs_py.point_cloud2 import create_cloud
    import yaml
    bag = tmp_path/'synthetic'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    for topic, typ in [('/velodyne_points', 'sensor_msgs/msg/PointCloud2'),
                       ('/odom', 'nav_msgs/msg/Odometry'), ('/imu', 'sensor_msgs/msg/Imu')]:
        writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=topic, type=typ, serialization_format='cdr'))
    a, b = np.meshgrid(np.linspace(-3, 8, 28), np.linspace(-2, 2, 14))
    floor = np.column_stack([a.ravel(), b.ravel(), np.zeros(a.size)])
    a, b = np.meshgrid(np.linspace(-3, 8, 28), np.linspace(0, 3, 14))
    wall = np.column_stack([a.ravel(), np.full(a.size, 3.), b.ravel()])
    world = np.concatenate([floor, wall])
    fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(['x', 'y', 'z', 'rgb'])]
    packed = np.uint32(0x36a2e8).view(np.float32)
    for i in range(21):
        ns = 10_000_000_000+i*100_000_000
        header = Header(frame_id='sirius3/lidar_link')
        header.stamp.sec, header.stamp.nanosec = divmod(ns, 1_000_000_000)
        xyz = world - np.array([i*.1+.05, 0, .85])
        cloud = create_cloud(header, fields, np.column_stack([xyz, np.full(len(xyz), packed)]).astype('f4'))
        odom = Odometry()
        odom.header.stamp = header.stamp
        odom.header.frame_id = 'sirius3/odom'
        odom.child_frame_id = 'sirius3/base_footprint'
        odom.pose.pose.position.x = 5+i*.1  # Arbitrary wheel origin must cancel.
        odom.pose.pose.orientation.w = 1.
        odom.twist.twist.linear.x = 1.
        imu = Imu()
        imu.header.stamp = header.stamp
        imu.orientation.w = 1.
        # Receipt order intentionally differs from sensor processing order.
        for topic, msg in [('/velodyne_points', cloud), ('/imu', imu), ('/odom', odom)]:
            writer.write(topic, serialize_message(msg), ns)
    del writer
    capture = bag/'capture'
    capture.mkdir()
    config = {'/**': {'ros__parameters': {
        'mapping': {'extrinsic_T': [.05, 0, .55], 'extrinsic_R': np.eye(3).reshape(-1).tolist()},
        'wheel': {'imu_to_base_translation': [0, 0, -.3],
                  'imu_from_base_rotation': np.eye(3).reshape(-1).tolist(),
                  'child_frame': 'sirius3/base_footprint'}}}}
    (capture/'calibration.yaml').write_text(yaml.safe_dump(config))
    return bag


def test_frozen_lio_falls_back_without_collapsing_a_moving_map(synthetic_bag):
    import rosbag2_py
    from rclpy.serialization import serialize_message, deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2, Imu
    from sensor_msgs_py.point_cloud2 import read_points, create_cloud
    import shutil
    source = m.reader(synthetic_bag, ['/velodyne_points'])
    _, data, _ = source.read_next()
    template = deserialize_message(data, PointCloud2)
    original = read_points(template, skip_nans=False)
    world = np.column_stack([original[n] for n in ('x', 'y', 'z')]) + [.05, 0, .85]
    bag = synthetic_bag.parent/'frozen_lio'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    for i, (topic, typ) in enumerate([('/velodyne_points', 'sensor_msgs/msg/PointCloud2'),
                                    ('/odom', 'nav_msgs/msg/Odometry'), ('/Odometry', 'nav_msgs/msg/Odometry'),
                                    ('/imu', 'sensor_msgs/msg/Imu')]):
        writer.create_topic(rosbag2_py.TopicMetadata(id=i, name=topic, type=typ, serialization_format='cdr'))
    for i in range(201):
        ns = 10_000_000_000+i*50_000_000
        distance = i*.05
        header = template.header
        header.stamp.sec, header.stamp.nanosec = divmod(ns, 1_000_000_000)
        cloud = create_cloud(header, template.fields, np.column_stack([world-[distance+.05, 0, .85], original['rgb']]).astype('f4'))
        wheel = Odometry()
        wheel.header.stamp = header.stamp
        wheel.header.frame_id, wheel.child_frame_id = 'sirius3/odom', 'sirius3/base_footprint'
        wheel.pose.pose.position.x = 5+distance
        wheel.pose.pose.orientation.w = 1.
        lio = Odometry()
        lio.header.stamp = header.stamp
        lio.header.frame_id, lio.child_frame_id = 'camera_init', 'body'
        lio.pose.pose.position.x, lio.pose.pose.position.z = 17., .3
        lio.pose.pose.orientation.w = 1.
        imu = Imu()
        imu.header.stamp = header.stamp
        imu.orientation.w = 1.
        for topic, msg in [('/velodyne_points', cloud), ('/odom', wheel), ('/Odometry', lio), ('/imu', imu)]:
            writer.write(topic, serialize_message(msg), ns)
    del writer
    shutil.copytree(synthetic_bag/'capture', bag/'capture')
    assert m.main([str(bag), '--no-loop']) == 0
    out = bag/'offline_3d'
    report = json.loads((out/'report.json').read_text())
    assert report['lio']['initialization'].startswith('wheel/IMU')
    assert report['lio']['rejected_edges'] == report['keyframes']-1
    trajectory = np.loadtxt(out/'trajectory.csv', delimiter=',', skiprows=1)
    assert abs(trajectory[-1, 1]-10.) < .1


@pytest.mark.parametrize('mode', ['wheel', 'optimize'])
def test_bag_produces_two_maps_with_identical_geometry(synthetic_bag, mode, capsys):
    assert m.main([str(synthetic_bag), '--mode', mode, '--keyframe-distance', '.5']) == 0
    out = synthetic_bag/'offline_3d'
    plain, colored = read_ply(out/'map_3d.ply'), read_ply(out/'map_3d_colored.ply')
    assert len(plain) > 100
    for name in ['x', 'y', 'z']:
        np.testing.assert_array_equal(plain[name], colored[name])
    assert np.all(colored['red'] == 0x36)
    report = json.loads((out/'report.json').read_text())
    assert report['rgb_keyframes'] == report['keyframes']
    trajectory = np.loadtxt(out/'trajectory.csv', skiprows=1, delimiter=',')
    assert abs(trajectory[-1, 1]-2.) < .05
    assert np.all(np.diff(trajectory[:, 1]) >= -.01)
    if mode == 'optimize':
        assert any(e.get('weak_directions', 0) for e in report['edges'])
        assert report['loops'] == 0  # 直線経路に再訪はない
    progress_log = capsys.readouterr().out
    for stage in range(1, 9):
        assert f'[{stage}/8]' in progress_log
    assert '100.0%' in progress_log
    assert ('【完了】' if report['optimization']['success'] else '【保存完了・要確認】') in progress_log
    # Protect existing results against silent overwrites.
    assert m.main([str(synthetic_bag), '--mode', mode]) == 1


def test_numbered_bag_selection_and_repeat_preserve_existing_map(synthetic_bag, monkeypatch, capsys):
    # Exercise the CLI with no bag argument, including input correction and a
    # second generation without asking for an output path or overwriting a map.
    answers = iter(['0', 'not a number', '1', ''])
    monkeypatch.setattr('builtins.input', lambda _: next(answers))
    options = ['--directory', str(synthetic_bag.parent), '--mode', 'wheel']
    assert m.main(options) == 0
    first = synthetic_bag/'offline_3d'/'map_3d_colored.ply'
    original = first.read_bytes()
    assert m.main(options) == 0
    assert first.read_bytes() == original
    second = synthetic_bag/'offline_3d_2'/'map_3d_colored.ply'
    assert second.read_bytes() == original
    from map_selection import map_candidates
    candidates = map_candidates(synthetic_bag.parent)
    assert candidates[0] == second
    assert len(candidates) == 2  # XYZ and RGB files must not duplicate choices.
    output = capsys.readouterr().out
    assert '[1] synthetic' in output and '1〜1の番号' in output


def test_discovery_finds_saved_maps_nested_bags_and_can_retry_empty_list(tmp_path, monkeypatch):
    from map_selection import bag_candidates, map_candidates, choose_numbered
    bags = tmp_path/'bags'
    maps = tmp_path/'maps3d'
    nested = bags/'0927'/'recording'
    nested.mkdir(parents=True)
    (nested/'metadata.yaml').write_text('')
    maps.mkdir()
    saved = maps/'colored_sync_1015_202120.ply'
    saved.write_text('ply\n')
    assert bag_candidates(bags) == [nested]
    assert map_candidates([bags, maps]) == [saved]
    empty = tmp_path/'new_maps'
    empty.mkdir()
    calls = []
    def answer(prompt):
        calls.append(prompt)
        if len(calls) == 1:
            (empty/'newly_saved.ply').write_text('ply\n')
            return ''
        return '1'
    monkeypatch.setattr('builtins.input', answer)
    assert choose_numbered(lambda: map_candidates(empty), '3D地図', empty) == empty/'newly_saved.ply'
    assert len(calls) == 2


def test_planar_encoders_leave_height_roll_pitch_to_lidar_and_imu():
    # A 15 degree uphill path, with 10 degree roll. Encoder odom remains flat.
    rot = m.Rotation.from_euler('xyz', [10, -15, 0], degrees=True).as_matrix()
    mount = m.transform(translation=[.05, 0, .85])
    base = [m.transform(rot, rot @ np.array([i, 0, 0])) for i in range(4)]
    initial = [p @ mount for p in base]
    measured = m.inverse(mount) @ m.transform(translation=[1, 0, 0]) @ mount
    edges = [(i, i+1, measured, (m.inverse(initial[i]) @ initial[i+1], np.eye(6)*50), np.eye(3))
             for i in range(3)]
    args = SimpleNamespace(wheel_model='planar', wheel_position_sigma=.01, wheel_rotation_sigma=.01,
                           imu_rotation_sigma=.01, max_iterations=100)
    poses, report = m.optimize_poses(initial, edges, args, base_from_lidar=mount)
    assert report['success']
    np.testing.assert_allclose(poses, initial, atol=1e-7)
    assert (poses[-1] @ m.inverse(mount))[2, 3] > .7
    # A full SE3 encoder constraint would falsely penalize slope attitude changes.
    relative = m.transform(m.Rotation.from_euler('xy', [12, -8], degrees=True).as_matrix(), [1, 0, .2])
    np.testing.assert_allclose(m.wheel_residual(relative, m.transform(translation=[1, 0, 0]), np.eye(4), args)[:2], 0)


def test_encoder_seed_integrates_distance_with_imu_attitude():
    times = np.arange(5, dtype=float)
    wheel = [m.transform(translation=[5+i, 2, 0]) for i in range(5)]
    rot = m.Rotation.from_euler('y', -20, degrees=True).as_matrix()
    imu = [m.transform(rot) for _ in times]
    seed = m.seed_planar_motion(times, wheel, times, imu, .2, np.eye(3))
    expected = np.array([5, 2, 0]) + rot @ np.array([4, 0, 0])
    np.testing.assert_allclose(seed[-1][:3, 3], expected, atol=1e-10)
    assert seed[-1][2, 3] > 1.3


def test_sloped_bag_uses_planar_encoders_and_keeps_ground_truth_for_evaluation(synthetic_bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import PointCloud2, Imu
    from sensor_msgs_py.point_cloud2 import create_cloud
    from nav_msgs.msg import Odometry
    import shutil
    bag = synthetic_bag.parent/'slope'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    for topic, typ in [('/velodyne_points', 'sensor_msgs/msg/PointCloud2'), ('/imu', 'sensor_msgs/msg/Imu'),
                       ('/odom', 'nav_msgs/msg/Odometry'), ('/ground_truth/odom', 'nav_msgs/msg/Odometry')]:
        writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=topic, type=typ, serialization_format='cdr'))
    r = m.reader(synthetic_bag)
    rot = m.Rotation.from_euler('xyz', [8, -15, 0], degrees=True)
    mount = m.transform(translation=[.05, 0, .85])
    while r.has_next():
        topic, data, ns = r.read_next()
        distance = (ns-10_000_000_000)*1e-9
        base = m.transform(rot.as_matrix(), rot.apply([distance, 0, 0]))
        if topic == '/velodyne_points':
            msg = deserialize_message(data, PointCloud2)
            raw = np.frombuffer(bytes(msg.data), dtype='<f4').reshape(-1, 4).copy()
            world = raw[:, :3] + np.array([distance+.05, 0, .85])
            lidar = base @ mount
            raw[:, :3] = (world-lidar[:3, 3]) @ lidar[:3, :3]
            msg = create_cloud(msg.header, msg.fields, raw)
        elif topic == '/imu':
            msg = deserialize_message(data, Imu)
            q = rot.as_quat()
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = map(float, q)
        else:
            msg = deserialize_message(data, Odometry)
            truth = Odometry()
            truth.header, truth.child_frame_id = msg.header, msg.child_frame_id
            # Deliberately wrong truth: must show up in evaluation, never distort the map.
            truth.pose.pose.position.x = float(base[0, 3]*3)
            truth.pose.pose.position.z = float(base[2, 3]*3)
            q = rot.as_quat()
            truth.pose.pose.orientation.x, truth.pose.pose.orientation.y, truth.pose.pose.orientation.z, truth.pose.pose.orientation.w = map(float, q)
            writer.write('/ground_truth/odom', serialize_message(truth), ns)
        writer.write(topic, serialize_message(msg), ns)
    del writer
    shutil.copytree(synthetic_bag/'capture', bag/'capture')
    assert m.main([str(bag), '--no-loop']) == 0
    out = bag/'offline_3d'
    trajectory = np.loadtxt(out/'trajectory.csv', delimiter=',', skiprows=1)
    np.testing.assert_allclose(trajectory[-1, 1:4], rot.apply([2, 0, 0]), atol=.04)
    plain, colored = read_ply(out/'map_3d.ply'), read_ply(out/'map_3d_colored.ply')
    for name in ['x', 'y', 'z']:
        np.testing.assert_array_equal(plain[name], colored[name])
    report = json.loads((out/'report.json').read_text())
    assert report['ground_truth_evaluation']['used_for_mapping'] is False
    assert report['ground_truth_evaluation']['position_rmse_m'] > 1
    assert (out/'trajectory_ground_truth.csv').exists()
    # The fixed floor remains flat, even though the robot climbs throughout the bag.
    floor = plain[np.abs(plain['y']-3) > .1]
    assert np.quantile(np.abs(floor['z']), .95) < .03


def test_point_time_deskew_restores_a_wall_during_translation_and_rotation():
    times = np.linspace(0, .2, 25)
    poses = [m.transform(m.Rotation.from_euler('z', t).as_matrix(), [2*t, 0, .2*t]) for t in times]
    mount = m.transform(translation=[0, 0, .85])
    offsets = np.linspace(0, .1, 100)
    world = np.column_stack([np.full(100, 5.), np.linspace(-2, 2, 100), np.ones(100)])
    raw = []
    for t, point in zip(offsets, world):
        pose = m.interpolate_pose(t, times, poses, .1) @ mount
        raw.append((point-pose[:3, 3]) @ pose[:3, :3])
    deskewer = m.CloudDeskewer(times, poses, mount, .1)
    corrected = deskewer.apply(np.array(raw), offsets, 0)
    reconstructed = corrected @ mount[:3, :3].T + mount[:3, 3]
    np.testing.assert_allclose(reconstructed, world, atol=1e-10)
    assert np.ptp(np.array(raw)[:, 0]) > .1
    assert deskewer.scans == 1 and deskewer.points == 100
    outside = deskewer.apply(np.array(raw), offsets, 1)
    assert np.isnan(outside).all(), 'never extrapolate unobserved motion'


def test_web_profile_calibration_matches_real_and_ideal_sensor_mounts():
    from web_sensor_profile import apply_profile
    template = {'/**': {'ros__parameters': {'mapping': {}, 'preprocess': {}, 'wheel': {}}}}
    profile = {'profile': 'sirius-real', 'mounts': {'lidar': [0, 0, .85], 'imu': [0, 0, .65]}, 'lidar': {'rolling': True}}
    params = apply_profile(template, profile)['/**']['ros__parameters']
    np.testing.assert_allclose(params['mapping']['extrinsic_T'], [0, 0, .2])
    assert params['preprocess']['lidar_type'] == 2
    assert params['wheel']['imu_to_base_translation'] == [0, 0, -.65]
    assert template['/**']['ros__parameters']['mapping'] == {}, 'do not mutate the caller template'
    profile['profile'] = 'ideal'
    profile['mounts'] = {'lidar': [.05, 0, .85], 'imu': [0, 0, .3]}
    profile['lidar']['rolling'] = False
    params = apply_profile(template, profile)['/**']['ros__parameters']
    np.testing.assert_allclose(params['mapping']['extrinsic_T'], [.05, 0, .55])
    assert params['preprocess']['lidar_type'] == 0


def test_timed_bag_runs_deskew_in_the_map_pipeline(synthetic_bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs_py.point_cloud2 import create_cloud
    import shutil
    bag = synthetic_bag.parent/'timed'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    for topic, typ in [('/velodyne_points', 'sensor_msgs/msg/PointCloud2'), ('/imu', 'sensor_msgs/msg/Imu'), ('/odom', 'nav_msgs/msg/Odometry')]:
        writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=topic, type=typ, serialization_format='cdr'))
    r = m.reader(synthetic_bag)
    while r.has_next():
        topic, data, ns = r.read_next()
        if topic == '/velodyne_points':
            msg = deserialize_message(data, PointCloud2)
            raw = np.frombuffer(bytes(msg.data), dtype='<f4').reshape(-1, 4).copy()
            offsets = np.linspace(0, .09, len(raw), dtype='f4')
            raw[:, 0] -= offsets  # Robot advances during one rotation.
            fields = list(msg.fields)+[PointField(name='time', offset=16, datatype=PointField.FLOAT32, count=1)]
            msg = create_cloud(msg.header, fields, np.column_stack([raw, offsets]))
            data = serialize_message(msg)
        writer.write(topic, data, ns)
    del writer
    shutil.copytree(synthetic_bag/'capture', bag/'capture')
    assert m.main([str(bag), '--no-loop']) == 0
    report = json.loads((bag/'offline_3d/report.json').read_text())
    assert report['deskew']['scans'] > 2 and report['deskew']['points'] > 100
    assert report['deskew']['dropped_points'] > 0, 'last scan has no future odom: drop unsupported points'
    plain = read_ply(bag/'offline_3d/map_3d.ply')
    assert np.quantile(np.abs(plain['z'][np.abs(plain['y']-3) > .1]), .95) < .02
