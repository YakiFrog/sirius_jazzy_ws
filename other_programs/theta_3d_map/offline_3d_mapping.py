#!/usr/bin/env python3
"""Read a bag directly and build matching XYZ/RGB maps with wheel-constrained poses."""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp
import yaml


def transform(rotation=None, translation=None):
    result = np.eye(4)
    if rotation is not None:
        result[:3, :3] = rotation
    if translation is not None:
        result[:3, 3] = translation
    return result


def inverse(pose):
    return transform(pose[:3, :3].T, -pose[:3, :3].T @ pose[:3, 3])


def pose_error(predicted, measured):
    error = predicted @ inverse(measured)
    return np.r_[error[:3, 3], Rotation.from_matrix(error[:3, :3]).as_rotvec()]


def interpolate_pose(stamp, times, poses, max_gap):
    i = int(np.searchsorted(times, stamp))
    if i < len(times) and abs(times[i] - stamp) < 1e-6:
        return poses[i].copy()
    if i == 0 or i == len(times):
        return None
    if max(stamp - times[i-1], times[i] - stamp) > max_gap:
        return None
    alpha = (stamp-times[i-1]) / (times[i]-times[i-1])
    rot = Slerp([0., 1.], Rotation.from_matrix([poses[i-1][:3, :3], poses[i][:3, :3]]))([alpha])[0]
    return transform(rot.as_matrix(), (1-alpha)*poses[i-1][:3, 3] + alpha*poses[i][:3, 3])


def observable_information(jacobian, ratio, sigma):
    """Mean point-to-plane information; discard weak directions rather than boosting them."""
    information = jacobian.T @ jacobian / max(1, len(jacobian))
    values, vectors = np.linalg.eigh(information)
    keep = values > max(1e-9, values[-1] * ratio)
    weights = np.where(keep, np.maximum(values, 0), 0) / sigma**2
    return np.diag(np.sqrt(weights)) @ vectors.T, values, int((~keep).sum())


def icp_edge(source, target, initial, args):
    import open3d as o3d
    def cloud(points):
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)
        pc = pc.voxel_down_sample(args.icp_voxel)
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=args.icp_voxel*4, max_nn=30))
        return pc
    src, dst = cloud(source), cloud(target)
    if min(len(src.points), len(dst.points)) < 30:
        return None, {'accepted': False, 'reason': 'too few points'}
    result = o3d.pipelines.registration.registration_icp(
        src, dst, args.max_correspondence, initial,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(
            o3d.pipelines.registration.HuberLoss(args.lidar_sigma)),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=35))
    moved = np.asarray(src.points) @ result.transformation[:3, :3].T + result.transformation[:3, 3]
    distances, indices = cKDTree(np.asarray(dst.points)).query(moved)
    keep = distances < args.max_correspondence
    meta = {'accepted': False, 'fitness': float(result.fitness), 'rmse': float(result.inlier_rmse)}
    if keep.sum() < 30 or result.fitness < args.min_fitness:
        meta['reason'] = 'insufficient overlap'
        return None, meta
    p = moved[keep]
    normals = np.asarray(dst.normals)[indices[keep]]
    h = np.column_stack([normals, np.cross(p, normals)])
    sqrt_info, eigenvalues, weak = observable_information(h, args.degeneracy_ratio, args.lidar_sigma)
    meta.update(accepted=True, eigenvalues=eigenvalues.tolist(), weak_directions=weak)
    # Project the ICP displacement itself too. An arbitrary weak-axis shift can
    # otherwise leak into strong residuals through finite-rotation coupling.
    correction = pose_error(result.transformation, initial)
    projector = np.linalg.pinv(sqrt_info) @ sqrt_info
    correction = projector @ correction
    measured = transform(Rotation.from_rotvec(correction[3:]).as_matrix(), correction[:3]) @ initial
    return (measured, sqrt_info), meta


def wheel_residual(relative, measured, base_from_lidar, args):
    if getattr(args, 'wheel_model', 'se3') == 'se3':
        return pose_error(relative, measured) / np.r_[np.full(3, args.wheel_position_sigma),
                                                       np.full(3, args.wheel_rotation_sigma)]
    # Wheel encoders observe travel in the body plane and rotation about its Z.
    # Their zero z/roll/pitch fields are NOT measurements of those quantities.
    lidar_from_base = inverse(base_from_lidar)
    predicted = base_from_lidar @ relative @ lidar_from_base
    wheel = base_from_lidar @ measured @ lidar_from_base
    error = Rotation.from_matrix(predicted[:3, :3] @ wheel[:3, :3].T).as_rotvec()
    return np.r_[(predicted[:2, 3] - wheel[:2, 3]) / args.wheel_position_sigma,
                 error[2] / args.wheel_rotation_sigma]


def seed_planar_motion(wt, wp, it, ip, max_gap, imu_from_base):
    """Integrate encoder increments along IMU attitude; never reads ground truth."""
    poses, alignment = [], None
    for k, (t, wheel) in enumerate(zip(wt, wp)):
        imu = interpolate_pose(t, it, ip, max_gap) if len(it) >= 2 else None
        rot = wheel[:3, :3]
        if imu is not None:
            observed = imu[:3, :3] @ imu_from_base
            if alignment is None:
                yaw_w = np.arctan2(rot[1, 0], rot[0, 0])
                yaw_i = np.arctan2(observed[1, 0], observed[0, 0])
                alignment = Rotation.from_euler('z', yaw_w-yaw_i).as_matrix()
            rot = alignment @ observed
        elif poses:
            rot = poses[-1][:3, :3] @ (wp[k-1][:3, :3].T @ wheel[:3, :3])
        if not poses:
            position = np.r_[wheel[:2, 3], 0.]
        else:
            delta = inverse(wp[k-1]) @ wheel
            position = poses[-1][:3, 3] + poses[-1][:3, :3] @ np.r_[delta[:2, 3], 0.]
        poses.append(transform(rot, position))
    return poses


def optimize_poses(initial, edges, args, progress=None, base_from_lidar=None):
    """Jointly fit relative wheel, IMU rotation and observable LiDAR constraints."""
    if len(initial) < 2:
        return initial, {'success': True, 'cost': 0., 'evaluations': 0}
    base_from_lidar = np.eye(4) if base_from_lidar is None else base_from_lidar
    wheel_size = 3 if getattr(args, 'wheel_model', 'se3') == 'planar' else 6
    x0 = np.concatenate([np.r_[p[:3, 3], Rotation.from_matrix(p[:3, :3]).as_rotvec()] for p in initial[1:]])
    def unpack(x):
        return [initial[0]] + [transform(Rotation.from_rotvec(v[3:]).as_matrix(), v[:3])
                               for v in x.reshape(-1, 6)]
    def residual(x):
        if progress is not None:
            progress.evaluated()
        poses = unpack(x)
        values = []
        for i, j, wheel, lidar, imu in edges:
            relative = inverse(poses[i]) @ poses[j]
            if wheel is not None:
                values.extend(wheel_residual(relative, wheel, base_from_lidar, args))
            if lidar is not None:
                measurement, sqrt_info = lidar
                values.extend(sqrt_info @ pose_error(relative, measurement))
            if imu is not None:
                values.extend(Rotation.from_matrix(relative[:3, :3] @ imu.T).as_rotvec() / args.imu_rotation_sigma)
        return np.asarray(values)
    pattern = lil_matrix((len(residual(x0)), len(x0)), dtype=int)
    row = 0
    for i, j, wheel, lidar, imu in edges:
        count = ((wheel_size if wheel is not None else 0) + (6 if lidar is not None else 0)
                 + (3 if imu is not None else 0))
        for node in (i, j):
            if node:
                pattern[row:row+count, (node-1)*6:node*6] = 1
        row += count
    result = least_squares(residual, x0, jac_sparsity=pattern.tocsr(), loss='huber',
                           f_scale=2., max_nfev=args.max_iterations, x_scale='jac')
    if not np.isfinite(result.x).all():
        raise RuntimeError('Optimization produced a nonfinite trajectory')
    return unpack(result.x), {'success': bool(result.success), 'cost': float(result.cost),
                             'evaluations': result.nfev, 'message': result.message}


def find_loop_edges(poses, times, cache, args, progress=None):
    """再訪候補をICPで検証し、ループ閉じ込み拘束 (i, j, None, lidar, None) を作る。

    候補: 現在姿勢間の距離が loop_radius 以下、キーフレーム番号差が
    loop_min_keyframes 以上、時刻差が loop_min_gap 秒以上。
    検証: 近距離の候補から順にICP。既存の受理条件（fitness/重なり/退化除去）を
    そのまま使い、通ったものだけ拘束として採用する。1ノードにつき最大1本。
    """
    edges, diagnostics = [], []
    n = len(poses)
    if args.no_loop or n <= args.loop_min_keyframes:
        return edges, diagnostics
    positions = np.array([p[:3, 3] for p in poses])
    tree = cKDTree(positions)
    candidates = []
    for i in range(n):
        for j in tree.query_ball_point(positions[i], args.loop_radius):
            j = int(j)
            if j - i < args.loop_min_keyframes:
                continue
            if times[j] - times[i] < args.loop_min_gap:
                continue
            candidates.append((float(np.linalg.norm(positions[i]-positions[j])), i, j))
    candidates.sort()
    if progress is not None:
        progress.begin(5, 'ループ閉じ込み候補の確認', len(candidates))
    used = set()
    for number, (distance, i, j) in enumerate(candidates, 1):
        if progress is not None:
            progress.update(number-1, f'採用 {len(edges)}件')
        if i in used or j in used:
            continue
        used.update((i, j))
        with np.load(cache/f'{i:06d}.npz') as target, np.load(cache/f'{j:06d}.npz') as source:
            relative = inverse(poses[i]) @ poses[j]
            lidar, meta = icp_edge(source['xyz'], target['xyz'], relative, args)
        meta.update({'from': i, 'to': j, 'type': 'loop',
                     'distance': distance, 'gap_s': float(times[j]-times[i])})
        diagnostics.append(meta)
        if lidar is not None:
            edges.append((i, j, None, lidar, None))
    return edges, diagnostics


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def reader(path, topics=None):
    import rosbag2_py
    r = rosbag2_py.SequentialReader()
    # Let rosbag identify MCAP or sqlite from metadata; no DDS or /clock playback.
    r.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''), rosbag2_py.ConverterOptions('', ''))
    if topics:
        r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


def load_motion(args, progress=None):
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    r = reader(args.bag, [args.wheel_topic, args.imu_topic])
    if progress is not None:
        total = sum(t.message_count for t in r.get_metadata().topics_with_message_count
                    if t.topic_metadata.name in (args.wheel_topic, args.imu_topic))
        progress.begin(1, '車輪オドメトリ・IMUの読み込み', total)
    processed = 0
    wheel, imu = {}, {}
    frames = set()
    last = {}
    while r.has_next():
        topic, data, _ = r.read_next()
        processed += 1
        if progress is not None:
            progress.update(processed)
        msg = deserialize_message(data, Odometry if topic == args.wheel_topic else Imu)
        t = stamp(msg)
        if topic in last and t < last[topic]-0.5:
            raise ValueError(f'{topic}: timestamp reset in bag; record one simulator run per bag')
        last[topic] = max(t, last.get(topic, t))
        if topic == args.wheel_topic:
            frames.add((msg.header.frame_id, msg.child_frame_id))
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            quat = np.array([q.x, q.y, q.z, q.w])
            if not np.isfinite(quat).all() or np.linalg.norm(quat) < .5:
                continue
            pose = transform(Rotation.from_quat(quat).as_matrix(), [p.x, p.y, p.z])
            if np.isfinite(pose).all():
                wheel[t] = pose
        else:
            q = msg.orientation
            quat = np.array([q.x, q.y, q.z, q.w])
            if msg.orientation_covariance[0] >= 0 and np.isfinite(quat).all() and np.linalg.norm(quat) > .5:
                imu[t] = transform(Rotation.from_quat(quat).as_matrix())
    if len(frames) != 1:
        raise ValueError(f'Wheel odometry frame changed or is absent: {frames}')
    if len(wheel) < 2:
        raise ValueError(f'{args.wheel_topic}: need at least two valid wheel poses')
    wt, it = sorted(wheel), sorted(imu)
    return np.asarray(wt), [wheel[t] for t in wt], np.asarray(it), [imu[t] for t in it], list(frames)[0]


def calibration(path):
    with open(path) as f:
        raw = yaml.safe_load(f)
    params = raw['/**']['ros__parameters']
    mapping, wheel = params['mapping'], params.get('wheel', {})
    imu_from_lidar = transform(np.array(mapping['extrinsic_R']).reshape(3, 3), mapping['extrinsic_T'])
    imu_from_base = transform(np.array(wheel.get('imu_from_base_rotation', np.eye(3))).reshape(3, 3),
                              wheel.get('imu_to_base_translation', [0., 0., -.3]))
    for pose in (imu_from_lidar, imu_from_base):
        rot = pose[:3, :3]
        if not np.isfinite(pose).all() or not np.allclose(rot.T @ rot, np.eye(3), atol=1e-6) or np.linalg.det(rot) < .99:
            raise ValueError('Calibration is not a finite rigid transform')
    return inverse(imu_from_base) @ imu_from_lidar, imu_from_lidar[:3, :3], wheel.get('child_frame')


class CloudDeskewer:
    """Compensate each VLP16 point using encoder/IMU initial motion, never truth."""
    def __init__(self, times, poses, base_from_lidar, max_gap):
        self.times, self.poses = np.asarray(times), np.asarray(poses)
        self.mount, self.max_gap = base_from_lidar, max_gap
        self.rotations = Slerp(self.times, Rotation.from_matrix(self.poses[:, :3, :3]))
        self.scans = self.points = self.dropped = 0

    def apply(self, xyz, offsets, reference_time):
        offsets = np.asarray(offsets).reshape(-1).astype(float)
        if np.any(np.isfinite(offsets) & ((offsets < -.001) | (offsets > .5))):
            raise ValueError('VLP16 time must be seconds from the PointCloud2 header (0..0.5 s)')
        sample_times = reference_time + offsets
        indexes = np.searchsorted(self.times, sample_times)
        indexes = np.clip(indexes, 1, len(self.times)-1)
        before, after = self.times[indexes-1], self.times[indexes]
        valid = (np.isfinite(sample_times) & (sample_times >= self.times[0]) & (sample_times <= self.times[-1])
                 & (sample_times-before <= self.max_gap) & (after-sample_times <= self.max_gap))
        ref_base = interpolate_pose(reference_time, self.times, self.poses, self.max_gap)
        if ref_base is None:
            valid[:] = False
        result = np.full_like(xyz, np.nan)
        if valid.any():
            t = sample_times[valid]
            alpha = ((t-before[valid]) / (after[valid]-before[valid]))[:, None]
            position = (1-alpha)*self.poses[indexes[valid]-1, :3, 3] + alpha*self.poses[indexes[valid], :3, 3]
            in_base = xyz[valid] @ self.mount[:3, :3].T + self.mount[:3, 3]
            world = self.rotations(t).apply(in_base) + position
            ref = ref_base @ self.mount
            result[valid] = (world-ref[:3, 3]) @ ref[:3, :3]
        self.scans += 1; self.points += int(valid.sum()); self.dropped += int((~valid).sum())
        return result


def points(msg, args, deskewer=None):
    from sensor_msgs_py.point_cloud2 import read_points
    p = read_points(msg, skip_nans=False)
    xyz = np.column_stack([p[n].reshape(-1) for n in ('x', 'y', 'z')]).astype(float)
    if deskewer is not None and 'time' in p.dtype.names:
        xyz = deskewer.apply(xyz, p['time'], stamp(msg))
    keep = np.isfinite(xyz).all(axis=1)
    distance = np.linalg.norm(xyz, axis=1)
    keep &= (distance >= args.min_range) & (distance <= args.max_range)
    rgb = np.full((len(xyz), 3), 128., dtype=float)
    field = 'rgb' if 'rgb' in p.dtype.names else 'rgba' if 'rgba' in p.dtype.names else None
    if field:
        packed = np.ascontiguousarray(p[field]).reshape(-1)
        packed = packed.view(np.uint32) if packed.dtype.kind == 'f' else packed.astype(np.uint32)
        rgb = np.column_stack([(packed >> 16) & 255, (packed >> 8) & 255, packed & 255]).astype(float)
    return xyz[keep], rgb[keep], field is not None


def reduce_points(xyz, rgb, voxel):
    if not len(xyz):
        return xyz, rgb
    _, inv = np.unique(np.floor(xyz/voxel).astype(np.int64), axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    count = np.bincount(inv)
    return (np.column_stack([np.bincount(inv, weights=xyz[:, i])/count for i in range(3)]),
            np.column_stack([np.bincount(inv, weights=rgb[:, i])/count for i in range(3)]))


def write_map(path, xyz, rgb=None):
    """Write XYZ identically for both outputs; only RGB fields differ."""
    dtype = [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]
    header = f'ply\nformat binary_little_endian 1.0\nelement vertex {len(xyz)}\n'
    header += 'property float x\nproperty float y\nproperty float z\n'
    if rgb is not None:
        dtype += [('red', 'u1'), ('green', 'u1'), ('blue', 'u1')]
        header += 'property uchar red\nproperty uchar green\nproperty uchar blue\n'
    data = np.empty(len(xyz), dtype=dtype)
    for i, name in enumerate(('x', 'y', 'z')):
        data[name] = xyz[:, i]
    if rgb is not None:
        color = np.clip(np.round(rgb), 0, 255).astype(np.uint8)
        for i, name in enumerate(('red', 'green', 'blue')):
            data[name] = color[:, i]
    with open(path, 'wb') as f:
        f.write((header + 'end_header\n').encode('ascii'))
        f.write(data.tobytes())


def save_trajectory(path, times, poses, base_from_lidar):
    with open(path, 'w') as f:
        out = csv.writer(f)
        out.writerow(['stamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        for t, pose in zip(times, poses):
            base = pose @ inverse(base_from_lidar)
            out.writerow([f'{t:.9f}', *base[:3, 3], *Rotation.from_matrix(base[:3, :3]).as_quat()])


def evaluate_ground_truth(args, times, poses, base_from_lidar, output):
    """Optional evaluation AFTER map generation; never changes fitted poses."""
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    topic = args.ground_truth_topic
    r = reader(args.bag)
    if not any(t.topic_metadata.name == topic and t.message_count for t in r.get_metadata().topics_with_message_count):
        return None
    import rosbag2_py
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    samples = {}
    while r.has_next():
        _, data, _ = r.read_next()
        msg = deserialize_message(data, Odometry)
        q, p = msg.pose.pose.orientation, msg.pose.pose.position
        quat = np.array([q.x, q.y, q.z, q.w])
        if np.isfinite(quat).all() and np.linalg.norm(quat) > .5:
            pose = transform(Rotation.from_quat(quat).as_matrix(), [p.x, p.y, p.z])
            if np.isfinite(pose).all():
                samples[stamp(msg)] = pose
    if not samples:
        return None
    gt_times = np.array(sorted(samples))
    gt_poses = [samples[t] for t in gt_times]
    aligned, selected, predicted, alignment = [], [], [], None
    for t, pose in zip(times, poses):
        truth = interpolate_pose(t, gt_times, gt_poses, args.max_sync_gap)
        if truth is None:
            continue
        base = pose @ inverse(base_from_lidar)
        if alignment is None:
            yaw = lambda p: np.arctan2(p[1, 0], p[0, 0])
            rot = Rotation.from_euler('z', yaw(base)-yaw(truth)).as_matrix()
            alignment = transform(rot, base[:3, 3]-rot @ truth[:3, 3])
        aligned.append(alignment @ truth)
        selected.append(t)
        predicted.append(base)
    if not aligned:
        return None
    save_trajectory(output/'trajectory_ground_truth.csv', selected, aligned, np.eye(4))
    error = np.array([p[:3, 3]-g[:3, 3] for p, g in zip(predicted, aligned)])
    angles = [np.linalg.norm(Rotation.from_matrix(p[:3, :3] @ g[:3, :3].T).as_rotvec()) for p, g in zip(predicted, aligned)]
    return {'topic': topic, 'samples': len(aligned), 'alignment': 'first matched pose: yaw and translation only',
            'used_for_mapping': False, 'position_rmse_m': float(np.sqrt(np.mean(np.sum(error**2, axis=1)))),
            'height_rmse_m': float(np.sqrt(np.mean(error[:, 2]**2))),
            'position_max_m': float(np.max(np.linalg.norm(error, axis=1))),
            'orientation_rmse_deg': float(np.rad2deg(np.sqrt(np.mean(np.square(angles)))))}


def run(args):
    from mapping_progress import MappingProgress
    with MappingProgress() as progress:
        return _run(args, progress)


def _run(args, progress):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    args.bag = Path(args.bag).expanduser().resolve()
    output = Path(args.output).expanduser().resolve() if args.output else args.bag/'offline_3d'
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError(f'Output is not empty: {output}. Use another --output directory.')
    config = Path(args.config).expanduser() if args.config else args.bag/'capture'/'calibration.yaml'
    if not config.exists():
        raise ValueError('Calibration snapshot missing; specify --config <FAST-LIO YAML>')
    base_from_lidar, imu_from_lidar_rot, expected_frame = calibration(config)
    progress.begin(1, '入力データ・校正設定の確認')
    wt, wp, it, ip, frames = load_motion(args, progress)
    progress.finish_stage(f'車輪姿勢 {len(wt)}件・IMU姿勢 {len(it)}件')
    if expected_frame and frames[1] != expected_frame:
        raise ValueError(f'Wheel child frame {frames[1]} differs from calibration {expected_frame}')
    seed = seed_planar_motion(wt, wp, it, ip, args.max_sync_gap,
                              imu_from_lidar_rot @ base_from_lidar[:3, :3].T) if args.wheel_model == 'planar' else wp
    r = reader(args.bag)
    counts = {t.topic_metadata.name: t.message_count for t in r.get_metadata().topics_with_message_count}
    topic = args.cloud_topic or ('/velodyne_points_colored' if counts.get('/velodyne_points_colored', 0) else '/velodyne_points')
    if not counts.get(topic, 0):
        raise ValueError(f'No point cloud messages on {topic}')
    import rosbag2_py
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    output.mkdir(parents=True, exist_ok=True)
    cache = output/'frames'
    cache.mkdir()
    deskewer = CloudDeskewer(wt, seed, base_from_lidar, args.max_sync_gap) if not args.no_deskew else None
    origin, start = None, None
    initial, times, imu_poses, wheel_poses = [], [], [], []
    skipped = colored = 0
    last_candidate = None
    cloud_frame = None
    previous_stamp = None
    progress.begin(2, '点群の読み込み・キーフレーム作成', counts[topic])
    processed = 0
    def add_frame(msg, pose, t):
        nonlocal skipped, colored
        xyz, rgb, has_color = points(msg, args, deskewer)
        if len(xyz) < 30:
            skipped += 1
            return
        if len(initial) >= args.max_keyframes:
            raise ValueError('Reached --max-keyframes; narrow --start/--end or increase the limit')
        xyz, rgb = reduce_points(xyz, rgb, args.voxel)
        np.savez(cache/f'{len(initial):06d}.npz', xyz=xyz.astype('f4'), rgb=rgb.astype('f4'))
        initial.append(pose)
        times.append(t)
        wheel_poses.append(interpolate_pose(t, wt, wp, args.max_sync_gap))
        imu_poses.append(interpolate_pose(t, it, ip, args.max_sync_gap) if len(it) >= 2 else None)
        colored += int(has_color)
    while r.has_next():
        _, data, _ = r.read_next()
        processed += 1
        progress.update(processed, f'キーフレーム {len(initial)}件')
        msg = deserialize_message(data, PointCloud2)
        t = stamp(msg)
        if previous_stamp is not None and t < previous_stamp:
            if t < previous_stamp-.5:
                raise ValueError('Point cloud timestamp reset; record one simulator run per bag')
            skipped += 1
            continue
        previous_stamp = t
        if cloud_frame is not None and cloud_frame != msg.header.frame_id:
            raise ValueError('Point cloud frame changed inside the bag')
        cloud_frame = msg.header.frame_id
        if start is None:
            start = t
        if t-start < args.start or t-start > args.end:
            continue
        base = interpolate_pose(t, wt, seed, args.max_sync_gap)
        if base is None:
            skipped += 1
            continue
        if origin is None:
            if args.wheel_model == 'planar':
                raw_base = interpolate_pose(t, wt, wp, args.max_sync_gap)
                origin = inverse(transform(raw_base[:3, :3], np.r_[raw_base[:2, 3], 0.]))
            else:
                origin = inverse(base)
        pose = origin @ base @ base_from_lidar
        last_candidate = (msg, pose, t)
        if initial:
            delta = inverse(initial[-1]) @ pose
            if np.linalg.norm(delta[:3, 3]) < args.keyframe_distance-1e-9 and np.linalg.norm(Rotation.from_matrix(delta[:3, :3]).as_rotvec()) < np.deg2rad(args.keyframe_angle) and t-times[-1] < args.keyframe_interval:
                continue
        add_frame(msg, pose, t)
        last_candidate = None
    # Include the final partial interval so trajectory distance covers the bag.
    if last_candidate is not None:
        add_frame(*last_candidate)
    if not initial:
        raise ValueError('No usable scans aligned to wheel odometry')
    print(f'keyframes={len(initial)}, skipped={skipped}, RGB frames={colored}', flush=True)
    progress.finish_stage(f'キーフレーム {len(initial)}件・同期欠落 {skipped}件')
    edges, diagnostics = [], []
    progress.begin(3, '隣接する点群の位置合わせ', len(initial)-1)
    for i in range(len(initial)-1):
        relative = inverse(initial[i]) @ initial[i+1]
        lidar, meta = None, {'accepted': False, 'reason': 'wheel baseline mode'}
        if args.mode == 'optimize':
            with np.load(cache/f'{i:06d}.npz') as a, np.load(cache/f'{i+1:06d}.npz') as b:
                lidar, meta = icp_edge(b['xyz'], a['xyz'], relative, args)
        imu = None
        if imu_poses[i] is not None and imu_poses[i+1] is not None:
            imu = imu_from_lidar_rot.T @ imu_poses[i][:3, :3].T @ imu_poses[i+1][:3, :3] @ imu_from_lidar_rot
        wheel = inverse(base_from_lidar) @ inverse(wheel_poses[i]) @ wheel_poses[i+1] @ base_from_lidar
        edges.append((i, i+1, wheel, lidar, imu))
        diagnostics.append({'from': i, 'to': i+1, **meta})
        progress.update(i+1)
        if (i+1) % 20 == 0:
            print(f'registered {i+1}/{len(initial)-1} edges', flush=True)
    progress.finish_stage()
    loop_edges, first_pass = [], None
    if args.mode == 'optimize':
        # 1回目: 隣接拘束のみ。再訪候補の判定とICP初期値にこの姿勢を使う。
        progress.begin(4, '軌跡の最適化（1回目）', detail='終了割合は算出できません。経過時間と計算回数を更新します')
        poses, first_pass = optimize_poses(initial, edges, args, progress, base_from_lidar)
        progress.finish_stage()
        progress.begin(5, 'ループ閉じ込み候補の検索')
        loop_edges, loop_diag = find_loop_edges(poses, times, cache, args, progress)
        progress.finish_stage(f'確認 {len(loop_diag)}件・採用 {len(loop_edges)}件')
        diagnostics.extend(loop_diag)
        print(f'loop candidates={len(loop_diag)} accepted={len(loop_edges)}', flush=True)
        if loop_edges:
            # 2回目: ループ拘束込みで全キーフレームを再最適化。
            edges.extend(loop_edges)
            progress.begin(6, 'ループ補正を含む軌跡の最適化（2回目）', detail='終了割合は算出できません。経過時間と計算回数を更新します')
            poses, optimization = optimize_poses(initial, edges, args, progress, base_from_lidar)
            progress.finish_stage()
        else:
            optimization = first_pass
            progress.begin(6, 'ループ補正を含む軌跡の最適化（2回目）')
            progress.finish_stage('ループ拘束がないためスキップ')
        if not args.no_loop and not loop_edges:
            print('NOTE: no loop-closure edge was accepted '
                  '(no revisit, or overlap too small for ICP)', flush=True)
    else:
        poses, optimization = initial, {'success': True, 'message': 'wheel poses only'}
        for stage, title in [(4, '軌跡の最適化（1回目）'), (5, 'ループ閉じ込み'), (6, '軌跡の最適化（2回目）')]:
            progress.begin(stage, title)
            progress.finish_stage('車輪のみのモードのためスキップ')
    # ループ拘束が最適化後にどれだけ満たされたか（閉じ残差）を記録する。
    loop_residuals = []
    for i, j, _, lidar, _ in loop_edges:
        error = pose_error(inverse(poses[i]) @ poses[j], lidar[0])
        loop_residuals.append({'from': i, 'to': j,
                               'trans_m': float(np.linalg.norm(error[:3])),
                               'rot_deg': float(np.degrees(np.linalg.norm(error[3:])))})
    if loop_residuals:
        print('loop residual after optimization: '
              f'trans mean={np.mean([r["trans_m"] for r in loop_residuals]):.3f} m, '
              f'max={np.max([r["trans_m"] for r in loop_residuals]):.3f} m', flush=True)
    xyz_chunks, rgb_chunks = [], []
    progress.begin(7, '地図点群の合成', len(poses))
    for i, pose in enumerate(poses):
        with np.load(cache/f'{i:06d}.npz') as frame:
            xyz_chunks.append(frame['xyz'] @ pose[:3, :3].T + pose[:3, 3])
            rgb_chunks.append(frame['rgb'])
        progress.update(i+1)
    progress.finish_stage()
    progress.begin(8, '点群の間引き・地図ファイルの保存', detail='通常地図・色付き地図・軌跡・レポートを保存します')
    xyz, rgb = reduce_points(np.concatenate(xyz_chunks), np.concatenate(rgb_chunks), args.voxel)
    progress.update(0, f'間引き完了（{len(xyz):,}点）、ファイル保存中')
    write_map(output/'map_3d.ply', xyz)
    write_map(output/'map_3d_colored.ply', xyz, rgb)
    save_trajectory(output/'trajectory.csv', times, poses, base_from_lidar)
    save_trajectory(output/'trajectory_initial.csv', times, initial, base_from_lidar)
    save_trajectory(output/'trajectory_wheel.csv', times,
                    [origin @ p @ base_from_lidar for p in wheel_poses], base_from_lidar)
    evaluation = evaluate_ground_truth(args, times, poses, base_from_lidar, output)
    if evaluation:
        print(f"3D真値との比較: 位置RMSE={evaluation['position_rmse_m']:.3f} m, 高さRMSE={evaluation['height_rmse_m']:.3f} m", flush=True)
    report = {'ground_truth_evaluation': evaluation, 'bag': str(args.bag), 'config': str(config.resolve()), 'mode': args.mode,
              'cloud_topic': topic, 'wheel_topic': args.wheel_topic, 'imu_topic': args.imu_topic,
              'wheel_frames': frames, 'keyframes': len(initial), 'points': len(xyz),
              'deskew': {'scans': deskewer.scans, 'points': deskewer.points, 'dropped_points': deskewer.dropped} if deskewer else None,
              'cloud_frame': cloud_frame, 'imu_edges': sum(e[4] is not None for e in edges),
              'loops': len(loop_edges), 'loop_residuals': loop_residuals,
              'options': {k: (str(v) if isinstance(v, Path) else None if isinstance(v, float) and not np.isfinite(v) else v)
                          for k, v in vars(args).items()},
              'skipped_scans': skipped, 'rgb_keyframes': colored,
              'optimization_first_pass': first_pass, 'optimization': optimization,
              'edges': diagnostics, 'limitations': [
                  ('Loop closure disabled (--no-loop)' if args.no_loop else
                   f'Loop closure enabled (radius {args.loop_radius:g} m, gap >= {args.loop_min_gap:g} s, '
                   f'min keyframe gap {args.loop_min_keyframes}); accepted edges: {len(loop_edges)}'),
                  'Point time in seconds is deskewed with encoder/IMU seed motion when present; snapshot clouds without time remain unchanged',
                  'IMU orientation constraints are used only when marked valid in the bag',
                  'Ground truth is recorded for evaluation only and is never an optimization input',
                  'Planar wheel mode leaves height, roll and pitch unconstrained by encoders']}
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    import shutil
    shutil.copyfile(config, output/'calibration.yaml')
    if not args.keep_cache:
        shutil.rmtree(cache)
    print(f'Saved: {output}/map_3d.ply and map_3d_colored.ply ({len(xyz)} identical XYZ points)', flush=True)
    if not colored:
        print('WARNING: no RGB in input; colored map is neutral gray', file=sys.stderr)
    if len(initial) > 1 and not any(e[4] is not None for e in edges):
        print('WARNING: no valid IMU orientation constraints; using wheel and LiDAR only', file=sys.stderr)
    if not optimization['success']:
        print('WARNING: optimizer stopped before convergence; inspect report.json', file=sys.stderr)
    progress.finish_stage()
    progress.complete(output, len(xyz), warning=not optimization['success'])
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', nargs='?', help='Recorded bag directory; omit to choose by number')
    parser.add_argument('--directory', default='~/rosbag2_data', help='Directory to list recorded bags')
    parser.add_argument('-o', '--output', help='Empty output directory (default: <bag>/offline_3d)')
    parser.add_argument('--config', help='FAST-LIO YAML; default: bag/capture/calibration.yaml')
    parser.add_argument('--mode', choices=['optimize', 'wheel'], default='optimize')
    parser.add_argument('--cloud-topic')
    parser.add_argument('--wheel-topic', default='/odom')
    parser.add_argument('--wheel-model', choices=['planar', 'se3'], default='planar',
                        help='planar: encoder x/y/yaw only, initialize slope height from IMU; se3: full 3D pose sensor')
    parser.add_argument('--imu-topic', default='/imu')
    parser.add_argument('--ground-truth-topic', default='/ground_truth/odom', help='Optional evaluation only; never used for mapping')
    for name, default in [('voxel', .05), ('icp-voxel', .2), ('keyframe-distance', .5),
                          ('keyframe-angle', 8.), ('keyframe-interval', 2.),
                          ('max-sync-gap', .2), ('max-correspondence', .5), ('min-fitness', .3),
                          ('degeneracy-ratio', .01), ('lidar-sigma', .05),
                          ('wheel-position-sigma', .03), ('wheel-rotation-sigma', .03),
                          ('imu-rotation-sigma', .05), ('min-range', .5), ('max-range', 80.)]:
        parser.add_argument('--'+name, type=float, default=default)
    parser.add_argument('--max-iterations', type=int, default=300)
    parser.add_argument('--max-keyframes', type=int, default=4000)
    parser.add_argument('--start', type=float, default=0., help='Seconds from first cloud')
    parser.add_argument('--end', type=float, default=float('inf'))
    parser.add_argument('--keep-cache', action='store_true')
    parser.add_argument('--no-deskew', action='store_true', help='Disable per-point encoder/IMU motion compensation')
    parser.add_argument('--no-loop', action='store_true', help='Disable loop closure')
    parser.add_argument('--loop-radius', type=float, default=6.0,
                        help='Revisit candidate distance threshold [m]')
    parser.add_argument('--loop-min-gap', type=float, default=10.0,
                        help='Minimum time gap between revisited keyframes [s]')
    parser.add_argument('--loop-min-keyframes', type=int, default=8,
                        help='Minimum keyframe index gap for a revisit candidate')
    args = parser.parse_args(argv)
    positive = [args.voxel, args.icp_voxel, args.keyframe_distance, args.keyframe_angle,
                args.keyframe_interval, args.max_sync_gap, args.max_correspondence,
                args.lidar_sigma, args.wheel_position_sigma, args.wheel_rotation_sigma,
                args.imu_rotation_sigma, args.max_range, args.loop_radius, args.loop_min_gap]
    if not all(np.isfinite(v) and v > 0 for v in positive) or not 0 < args.degeneracy_ratio < 1 or not 0 < args.min_fitness <= 1 or args.start < 0 or args.end < args.start or args.max_iterations < 1 or args.max_keyframes < 1 or args.loop_min_keyframes < 1:
        parser.error('Invalid range, noise, optimizer, loop, or sampling option')
    try:
        if args.bag is None:
            from map_selection import bag_candidates, choose_numbered, next_output_directory
            args.bag = choose_numbered(lambda: bag_candidates(args.directory), '後処理するBAG', args.directory)
            if args.bag is None:
                return 0
            if args.output is None:
                args.output = str(next_output_directory(args.bag))
            print(f'選択したBAG: {args.bag}\n保存先: {args.output}', flush=True)
        return run(args)
    except (ValueError, RuntimeError, ImportError, OSError, EOFError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
