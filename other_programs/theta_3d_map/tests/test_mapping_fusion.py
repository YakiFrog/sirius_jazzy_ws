"""Signed real sensor times, frame contracts and conservative dynamic removal."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mapping_fusion import rigid, invert, static_lookup, wheel_weights, visibility_filter, motion_consistency

spec = importlib.util.spec_from_file_location('offline_fusion_test', ROOT/'offline_3d_mapping.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def test_static_calibration_traverses_mount_chain_in_both_directions():
    edges = [('base', 'link', rigid()), ('link', 'imu', rigid(position=[0, 0, .65])),
             ('link', 'lidar', rigid(position=[0, 0, .85]))]
    np.testing.assert_allclose(static_lookup(edges, 'base', 'lidar')[:3, 3], [0, 0, .85])
    np.testing.assert_allclose(static_lookup(edges, 'imu', 'base')[:3, 3], [0, 0, -.65])
    assert static_lookup(edges, 'unknown', 'base') is None


def test_signed_point_times_recover_moving_wall_from_legacy_coloring():
    from sensor_msgs.msg import PointField
    from sensor_msgs_py.point_cloud2 import create_cloud
    from std_msgs.msg import Header
    times = np.linspace(0, 1, 21)
    poses = [rigid(position=[t, 0, 0]) for t in times]
    deskew = m.CloudDeskewer(times, poses, np.eye(4), .1)
    offsets = np.linspace(-.05, 0, 10)
    header = Header(frame_id='lidar')
    header.stamp.nanosec = 500_000_000
    raw_xyz = np.column_stack([5-(.5+offsets), np.linspace(-1, 1, 10), np.zeros(10)])
    fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(['x', 'y', 'z', 'time'])]
    raw = create_cloud(header, fields, np.column_stack([raw_xyz, offsets]).astype('f4'))
    colored_fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(['x', 'y', 'z', 'rgb'])]
    rgb = np.full(10, np.uint32(0x36a2e8).view(np.float32))
    colored = create_cloud(header, colored_fields, np.column_stack([raw_xyz, rgb]).astype('f4'))
    xyz, colors, has_color = m.points(colored, SimpleNamespace(min_range=.1, max_range=80), deskew, raw)
    np.testing.assert_allclose(xyz[:, 0], 4.5, atol=1e-6)
    np.testing.assert_array_equal(colors, np.tile([0x36, 0xa2, 0xe8], (10, 1)))
    assert has_color and deskew.scans == 1
    bad = create_cloud(header, colored_fields, np.column_stack([raw_xyz[::-1], rgb]).astype('f4'))
    with pytest.raises(ValueError, match='point order or XYZ'):
        m.points(bad, SimpleNamespace(min_range=.1, max_range=80), deskew, raw)


def test_wheel_disagreement_is_gated_only_with_lidar_geometric_support():
    lio = rigid(position=[1., 0, 0])
    slip = rigid(position=[2., 0, 0])
    strong = (lio, np.eye(6)*20)
    weak = (lio, np.diag([0, 20, 20, 20, 20, 20]))
    strong_weights, _ = wheel_weights(lio, slip, np.eye(4), strong, .03, .03)
    weak_weights, _ = wheel_weights(lio, slip, np.eye(4), weak, .03, .03)
    assert strong_weights[0] < .05
    assert weak_weights[0] == 1  # disagreement alone cannot label a wheel as slipping


def test_frozen_lio_cannot_be_selected_as_initializer_for_sustained_wheel_motion():
    times = np.arange(201)*.05
    wheel = [rigid(position=[t, 0, 0]) for t in times]
    frozen = [rigid() for _ in times]
    ratio, conflict = motion_consistency(times, wheel, times, frozen, m.interpolate_pose, .2)
    assert conflict and ratio == 0.
    ratio, conflict = motion_consistency(times, wheel, times, wheel, m.interpolate_pose, .2)
    assert not conflict and ratio == 1.


def test_valid_lio_cannot_override_observed_floor_height_and_tilt():
    from scipy.spatial.transform import Rotation
    args = SimpleNamespace(wheel_model='planar', max_iterations=80)
    poses = [rigid(Rotation.from_euler('y', .025*i).as_matrix(), [i, 0, .12*i]) for i in range(5)]
    floor = (rigid(position=[1, 0, 0]), np.diag([0, 20, 20, 20, 20, 0]))
    edges = []
    for i in range(4):
        lio = invert(poses[i])@poses[i+1]
        info = m.unobservable_motion_information(floor, np.array([.1]*3+[.03]*3), 4.)
        edges += [(i, i+1, None, floor, None), (i, i+1, None, (lio, info), None)]
    result, _ = m.optimize_poses(poses, edges, args)
    np.testing.assert_allclose([p[2, 3] for p in result], 0., atol=1e-5)
    assert max(Rotation.from_matrix(p[:3, :3]).magnitude() for p in result) < 1e-5
    assert result[-1][0, 3] > 3.9  # the floor's unobservable travel still uses LIO


@pytest.mark.parametrize('slope', [0., .2])
def test_shared_plane_refinement_recovers_height_without_flattening_slope(tmp_path, slope):
    from mapping_bundle import refine_surfaces
    from scipy.spatial.transform import Rotation
    x, y = np.meshgrid(np.linspace(-1, 5, 35), np.linspace(-2, 2, 25))
    world = np.column_stack([x.ravel(), y.ravel(), slope*x.ravel()])
    truth, initial = [], []
    mount = rigid(position=[0, 0, .85])
    for i, distance in enumerate([0., 1., 2., 3., 2., 1., 0.]):
        true = rigid(position=[distance, 0, .85+slope*distance])
        local = (world-true[:3, 3])@true[:3, :3]
        np.savez(tmp_path/f'{i:06d}.npz', xyz=local)
        pose = true.copy()
        if i:
            pose[2, 3] += .10*np.sin(i/6*np.pi)
            pose[:3, :3] = Rotation.from_euler('y', .025*np.sin(i/6*np.pi)).as_matrix()
        truth.append(true)
        initial.append(pose)
    args = SimpleNamespace(voxel=.05, lidar_sigma=.05, max_iterations=40,
                           surface_rounds=2, wheel_model='planar')
    edges = [(i, i+1, None, (invert(initial[i])@initial[i+1],
                             np.diag([5., 5., 5., 10., 10., 10.])), None) for i in range(6)]
    def graph(p):
        return m.pose_graph_residual(p, edges, args, mount)
    result, report = refine_surfaces(initial, tmp_path, edges, args, graph,
        edge_residual=lambda a, b, e: m.pose_graph_residual([a, b], [(0, 1, *e[2:])], args, mount))
    errors = [abs(p[2, 3]-t[2, 3]) for p, t in zip(result, truth)]
    assert max(errors) < .03
    assert report['rounds'][-1]['final_surface_rms_m'] < .01
    if slope:
        assert result[3][2, 3]-result[0][2, 3] > .55


def test_floor_validation_detects_double_layers_independently_of_solver(tmp_path):
    from mapping_quality import floor_overlap_quality
    x, y = np.meshgrid(np.linspace(-2, 2, 45), np.linspace(-2, 2, 45))
    world = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])
    truth = [rigid(position=[0, 0, .85]), rigid(position=[.8, 0, .85])]
    for i, pose in enumerate(truth):
        np.savez(tmp_path/f'{i:06d}.npz', xyz=world-pose[:3, 3])
    mount = rigid(position=[0, 0, .85])
    good = floor_overlap_quality(tmp_path, truth, [0, 60], mount)
    doubled = [p.copy() for p in truth]
    doubled[1][2, 3] += .2
    bad = floor_overlap_quality(tmp_path, doubled, [0, 60], mount)
    assert good['max_pair_p95_height_difference_m'] < 1e-5
    assert bad['max_pair_p95_height_difference_m'] == pytest.approx(.2, abs=1e-5)
    assert not bad['absolute_accuracy_measured']


@pytest.mark.parametrize('surface', ['ramp', 'curve'])
def test_shared_planes_keep_distinct_ramp_and_landings(tmp_path, surface):
    from mapping_bundle import refine_surfaces
    from scipy.spatial.transform import Rotation
    x, y = np.meshgrid(np.linspace(-3, 6, 80), np.linspace(-2, 2, 35))
    height = (lambda p: np.clip(p, 0, 3)*.2) if surface == 'ramp' else (lambda p: .025*np.asarray(p)**2)
    ground = height(x)
    world = np.column_stack([x.ravel(), y.ravel(), ground.ravel()])
    truth, initial = [], []
    for i, distance in enumerate([-1., 0., 1., 2., 3., 4., 3., 2., 1., 0., -1.]):
        true = rigid(position=[distance, 0, .85+height(distance)])
        np.savez(tmp_path/f'{i:06d}.npz', xyz=world-true[:3, 3])
        pose = true.copy()
        if i:
            pose[2, 3] += .05*np.sin(i/10*np.pi)
            pose[:3, :3] = Rotation.from_euler('y', .015*np.sin(i/10*np.pi)).as_matrix()
        truth.append(true); initial.append(pose)
    args = SimpleNamespace(voxel=.05, lidar_sigma=.05, max_iterations=40, surface_rounds=2, wheel_model='planar')
    edges = [(i, i+1, None, (invert(initial[i])@initial[i+1], np.eye(6)*5), None) for i in range(10)]
    mount = rigid(position=[0, 0, .85])
    result, _ = refine_surfaces(initial, tmp_path, edges, args,
        lambda p: m.pose_graph_residual(p, edges, args, mount),
        edge_residual=lambda a, b, e: m.pose_graph_residual([a, b], [(0, 1, *e[2:])], args, mount))
    assert max(abs(p[2, 3]-t[2, 3]) for p, t in zip(result, truth)) < .035
    assert result[5][2, 3]-result[0][2, 3] > height(4.)-height(-1.)-.05


@pytest.mark.parametrize('tilt_degrees', [0., 5.])
def test_stationary_imu_check_rejects_inconsistent_absolute_attitude(tilt_degrees):
    from sensor_consistency import check_imu
    from sensor_msgs.msg import Imu
    from rclpy.serialization import serialize_message
    from scipy.spatial.transform import Rotation
    messages = []
    for i in range(60):
        msg = Imu()
        t = 1.+i*.05
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = round((t-int(t))*1e9)
        q = Rotation.from_euler('x', tilt_degrees, degrees=True).as_quat()
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        msg.linear_acceleration.z = 9.80665
        messages.append(serialize_message(msg))
    class Reader:
        def __init__(self): self.index = 0
        def get_metadata(self):
            return SimpleNamespace(topics_with_message_count=[SimpleNamespace(
                topic_metadata=SimpleNamespace(name='/imu'), message_count=len(messages))])
        def has_next(self): return self.index < len(messages)
        def read_next(self):
            data = messages[self.index]
            self.index += 1
            return '/imu', data, self.index
    times = 1.+np.arange(61)*.05
    report = check_imu(SimpleNamespace(bag='unused', imu_topic='/imu', max_sync_gap=.2),
                       lambda *a: Reader(), times, [rigid() for _ in times])
    assert report['orientation_acceleration_angle_deg'] == pytest.approx(tilt_degrees, abs=1e-5)
    assert report['absolute_attitude_usable'] == (tilt_degrees == 0.)
    assert not report['calibration_applied']


def test_body_normal_motion_prior_prevents_free_height_and_preserves_ramp():
    from scipy.spatial.transform import Rotation
    pitch = -.2
    r = Rotation.from_euler('y', pitch).as_matrix()
    initial = [rigid(r, [0, 0, 0]), rigid(r, [np.cos(pitch), 0, 3.]),
               rigid(r, [2*np.cos(pitch), 0, -2.])]
    args = SimpleNamespace(wheel_model='planar', wheel_position_sigma=.03, wheel_rotation_sigma=.03,
                           wheel_vertical_sigma=.10, imu_rotation_sigma=.05, max_iterations=100)
    edges = [(i, i+1, (rigid(position=[1., 0, 0]), np.ones(3), 0.), None, None) for i in range(2)]
    edges += [(i, i, None, None, {'gravity': r.T@np.array([0., 0., 1.])}) for i in range(3)]
    result, report = m.optimize_poses(initial, edges, args)
    assert report['success']
    np.testing.assert_allclose([p[2, 3] for p in result], -np.arange(3)*np.sin(pitch), atol=1e-4)


def test_absolute_gravity_prior_observes_tilt_but_leaves_heading_free():
    from scipy.spatial.transform import Rotation
    r = Rotation.from_euler('y', -.2).as_matrix()
    expected = {'gravity': r.T@np.array([0., 0., 1.])}
    yawed = Rotation.from_euler('z', 1.3).as_matrix()@r
    args = SimpleNamespace(imu_rotation_sigma=.05)
    np.testing.assert_allclose(m.inertial_residual(rigid(yawed), rigid(), expected, args), 0., atol=1e-8)
    assert np.linalg.norm(m.inertial_residual(rigid(), rigid(), expected, args)) > 1.


def test_visibility_removes_repeatedly_cleared_ghost_and_preserves_occluded_wall():
    ghost, wall, unseen = [2., 0, 0], [5., 0, 0], [0, 5., 0]
    points = np.array([ghost, wall, unseen])
    # Robot remains stationary while a person leaves. Foreground person in the
    # first view must not be interpreted as free space behind the person.
    frames = [np.array([ghost])] + [np.array([wall]) for _ in range(4)]
    args = SimpleNamespace(dynamic_range=30., dynamic_margin=.2, dynamic_min_votes=3)
    keep, report = visibility_filter(points, frames.__getitem__, [rigid() for _ in frames], args)
    np.testing.assert_array_equal(keep, [False, True, True])
    assert report['removed_points'] == 1


def test_visibility_does_not_remove_sparse_points_without_contradicting_returns():
    points = np.array([[5., 0, 0], [0, 5., 0]])
    frames = [np.array([[2., 0, 0]]) for _ in range(5)]
    args = SimpleNamespace(dynamic_range=30., dynamic_margin=.2, dynamic_min_votes=3)
    keep, _ = visibility_filter(points, frames.__getitem__, [rigid() for _ in frames], args)
    assert keep.all()


def test_trajectory_jump_check_flags_spike_but_keeps_slope_and_steady_motion():
    from mapping_quality import trajectory_jump_quality
    times = np.arange(20)*.5
    steady = [rigid(position=[.625*i, 0, .1+.05*i+.001*np.sin(i)]) for i in range(20)]
    result = trajectory_jump_quality(steady, times)
    assert result['jump_count'] == 0
    assert result['max_speed_observed_mps'] == pytest.approx(1.254, abs=.02)
    spiked = [p.copy() for p in steady]
    spiked[10][2, 3] += 30.
    result = trajectory_jump_quality(spiked, times)
    assert result['jump_count'] == 2
    assert all('above_physical_cap' in j['reasons'] for j in result['jumps'])
    assert not result['absolute_accuracy_measured']


def test_trajectory_jump_check_flags_robust_outlier_below_physical_cap():
    from mapping_quality import trajectory_jump_quality
    times = np.arange(12)*.5
    poses = [rigid(position=[.1*i, 0, 0]) for i in range(12)]
    poses[6][2, 3] += .9
    result = trajectory_jump_quality(poses, times)
    assert result['jump_count'] == 2
    assert all('robust_outlier' in j['reasons'] for j in result['jumps'])
    assert max(j['speed_mps'] for j in result['jumps']) < 2.


def test_trajectory_jump_check_accepts_legitimate_acceleration():
    from mapping_quality import trajectory_jump_quality
    speeds_seq = np.concatenate([np.full(14, 1.25), np.full(20, 1.75)])
    times = np.concatenate([[0], np.cumsum(np.concatenate([np.full(14, .5), np.full(20, .4)]))])
    step = speeds_seq*np.diff(times)
    x = np.concatenate([[0], np.cumsum(step)])
    poses = [rigid(position=[xi, .01*np.sin(i), .05*i/len(x)]) for i, xi in enumerate(x)]
    result = trajectory_jump_quality(poses, times)
    assert result['max_speed_observed_mps'] == pytest.approx(1.75, abs=.02)
    assert result['jump_count'] == 0


def test_trajectory_jump_check_ignores_stationary_jitter():
    from mapping_quality import trajectory_jump_quality
    times = np.arange(15)*.5
    poses = [rigid(position=[.01*np.sin(3*i), .01*np.cos(2*i), .002*np.sin(5*i)]) for i in range(15)]
    result = trajectory_jump_quality(poses, times)
    assert result['jump_count'] == 0
    assert result['steps_checked'] == 14
