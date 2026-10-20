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
from mapping_fusion import (read_static_calibration, load_lio, wheel_weights,
                            visibility_filter, motion_consistency)


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


def dropped_axis_labels(vectors, keep, to_normalized, threshold=.7):
    """Original-frame axes whose direction is mostly covered by the discarded subspace."""
    dropped = vectors[:, ~keep]
    if not dropped.size:
        return []
    labels = []
    for index, label in enumerate(('x', 'y', 'z', 'roll', 'pitch', 'yaw')):
        direction = np.zeros(6)
        direction[index] = 1.
        direction *= to_normalized
        projected = dropped @ (dropped.T @ direction)
        if np.linalg.norm(projected) / np.linalg.norm(direction) > threshold:
            labels.append(label)
    return labels


def observable_information(jacobian, ratio, sigma, scale=1.):
    """Mean point-to-plane information with rotation/translation unit normalization.

    Rows of `jacobian` are [normal, point x normal] in the sensor frame, so rotation
    columns grow with the lever arm while translation columns stay unitless. Mixing
    them lets distant points inflate the rotation block and hide genuinely observable
    translation directions. `scale` is a representative length: rotations are divided
    by it before the eigen test, then the weights are mapped back to the original
    parameterization so residual units (meters, radians) are unchanged.
    """
    scale = max(float(scale), 1e-6)
    to_original = np.array([1., 1., 1., 1./scale, 1./scale, 1./scale])
    to_normalized = np.array([1., 1., 1., scale, scale, scale])
    scaled = jacobian * to_original
    information = scaled.T @ scaled / max(1, len(jacobian))
    values, vectors = np.linalg.eigh(information)
    keep = values > max(1e-9, values[-1] * ratio)
    weights = np.where(keep, np.maximum(values, 0), 0) / sigma**2
    sqrt_info = (np.diag(np.sqrt(weights)) @ vectors.T) * to_normalized
    diagnostics = {'eigenvalues': values, 'vectors': vectors, 'keep': keep, 'scale': scale,
                   'dropped_axes': dropped_axis_labels(vectors, keep, to_normalized)}
    return sqrt_info, diagnostics, int((~keep).sum())


def icp_edge(source, target, initial, args):
    import open3d as o3d
    from mapping_geometry import registration_cloud
    planar = not getattr(args, 'no_planar_normals', False)
    src, _ = registration_cloud(source, args.icp_voxel, planar)
    dst, _ = registration_cloud(target, args.icp_voxel, planar)
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
    lever = max(float(np.median(np.linalg.norm(p, axis=1))), 1e-3)
    h = np.column_stack([normals, np.cross(p, normals)])
    sqrt_info, degeneracy, weak = observable_information(h, args.degeneracy_ratio, args.lidar_sigma, lever)
    meta.update(accepted=True, eigenvalues=degeneracy['eigenvalues'].tolist(), weak_directions=weak,
                char_length_m=lever, dropped_axes=degeneracy['dropped_axes'],
                eigenvectors=np.round(degeneracy['vectors'], 3).tolist())
    # Project the ICP displacement itself too. The projection runs in the normalized
    # frame so a weak-axis shift cannot leak into strong residuals through
    # finite-rotation coupling, and only physically unobservable components are lost.
    correction = pose_error(result.transformation, initial)
    to_normalized = np.array([1., 1., 1., lever, lever, lever])
    to_original = np.array([1., 1., 1., 1./lever, 1./lever, 1./lever])
    normalized = to_normalized * correction
    dropped = ~degeneracy['keep']
    removed = (degeneracy['vectors'][:, dropped] @ (degeneracy['vectors'][:, dropped].T @ normalized)
               if dropped.any() else np.zeros(6))
    correction = to_original * (normalized - removed)
    meta.update(correction_norm_m=float(np.linalg.norm(normalized)),
                dropped_correction_m=float(np.linalg.norm(removed)))
    measured = transform(Rotation.from_rotvec(correction[3:]).as_matrix(), correction[:3]) @ initial
    return (measured, sqrt_info), meta


def wheel_residual(relative, measured, base_from_lidar, args):
    weights = np.ones(3)
    expected_body_z = 0.
    if isinstance(measured, tuple):
        measured, weights, *kinematics = measured
        if kinematics:
            expected_body_z = kinematics[0]
    if getattr(args, 'wheel_model', 'se3') == 'se3':
        return pose_error(relative, measured) / np.r_[np.full(3, args.wheel_position_sigma),
                                                       np.full(3, args.wheel_rotation_sigma)]
    # Encoder pose has no measured world height. Nonholonomic motion nevertheless
    # bounds displacement normal to the body plane. IMU-integrated increments
    # account for pitch changes inside an interval, preserving actual ramps.
    lidar_from_base = inverse(base_from_lidar)
    predicted = base_from_lidar @ relative @ lidar_from_base
    wheel = base_from_lidar @ measured @ lidar_from_base
    error = Rotation.from_matrix(predicted[:3, :3] @ wheel[:3, :3].T).as_rotvec()
    planar = weights * np.r_[(predicted[:2, 3] - wheel[:2, 3]) / args.wheel_position_sigma,
                             error[2] / args.wheel_rotation_sigma]
    distance = np.linalg.norm(wheel[:2, 3])
    rotation = Rotation.from_matrix(predicted[:3, :3]).magnitude()
    vertical_sigma = getattr(args, 'wheel_vertical_sigma', .10)+.5*distance*rotation
    return np.r_[planar, (predicted[2, 3]-expected_body_z)/vertical_sigma]


def unobservable_motion_information(lidar, sigma, scale=1.):
    """Use the motion prior only where scan geometry cannot observe displacement."""
    if lidar is None:
        return np.diag(1./sigma)
    units = np.array([1., 1., 1., scale, scale, scale])
    _, values, vectors = np.linalg.svd(lidar[1]/units)
    null = vectors[values < max(1e-8, values[0]*1e-8)]
    projection = null.T @ null
    return ((projection*units)/units[:, None])/sigma[:, None]


def inertial_residual(first, second, measurement, args):
    if isinstance(measurement, dict) and 'gravity' in measurement:
        return (first[:3, :3].T@np.array([0., 0., 1.])-measurement['gravity'])/args.imu_rotation_sigma
    relative = first[:3, :3].T@second[:3, :3]
    if isinstance(measurement, dict):
        angle = np.arctan2(relative[1, 0], relative[0, 0])-measurement['yaw']
        return np.array([0., 0., np.arctan2(np.sin(angle), np.cos(angle))])/args.imu_rotation_sigma
    return Rotation.from_matrix(relative@measurement.T).as_rotvec()/args.imu_rotation_sigma


def pose_graph_residual(poses, edges, args, base_from_lidar):
    values = []
    for i, j, wheel, lidar, imu in edges:
        relative = inverse(poses[i]) @ poses[j]
        if wheel is not None:
            values.extend(wheel_residual(relative, wheel, base_from_lidar, args))
        if lidar is not None:
            values.extend(lidar[1] @ pose_error(relative, lidar[0]))
        if imu is not None:
            values.extend(inertial_residual(poses[i], poses[j], imu, args))
    return np.asarray(values)


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
    wheel_size = 4 if getattr(args, 'wheel_model', 'se3') == 'planar' else 6
    x0 = np.concatenate([np.r_[p[:3, 3], Rotation.from_matrix(p[:3, :3]).as_rotvec()] for p in initial[1:]])
    def unpack(x):
        return [initial[0]] + [transform(Rotation.from_rotvec(v[3:]).as_matrix(), v[:3])
                               for v in x.reshape(-1, 6)]
    def residual(x):
        if progress is not None:
            progress.evaluated()
        return pose_graph_residual(unpack(x), edges, args, base_from_lidar)
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
    return unpack(result.x), {'success': bool(result.success), 'status': int(result.status),
                             'cost': float(result.cost), 'evaluations': result.nfev,
                             'optimality': float(result.optimality), 'message': result.message}


def residual_breakdown(poses, edges, args, base_from_lidar):
    """Weighted residual RMS per constraint type at the current poses.

    Least-squares reports one total cost; separating wheel / LiDAR / IMU shows which
    constraint group keeps the trajectory from converging or closing the loop.
    """
    values = {'wheel': [], 'lidar': [], 'imu': []}
    for i, j, wheel, lidar, imu in edges:
        relative = inverse(poses[i]) @ poses[j]
        if wheel is not None:
            values['wheel'].append(wheel_residual(relative, wheel, base_from_lidar, args))
        if lidar is not None:
            values['lidar'].append(lidar[1] @ pose_error(relative, lidar[0]))
        if imu is not None:
            values['imu'].append(inertial_residual(poses[i], poses[j], imu, args))
    report = {}
    for name, rows in values.items():
        parts = np.concatenate(rows) if rows else np.zeros(1)
        report[name] = {'edges': len(rows), 'rms': float(np.sqrt(np.mean(parts**2))),
                        'max_abs': float(np.max(np.abs(parts)))}
    return report


def find_loop_edges(poses, times, cache, args, progress=None):
    """再訪候補をICPで検証し、ループ閉じ込み拘束 (i, j, None, lidar, None) を作る。

    候補: 現在姿勢間の距離が loop_radius 以下、キーフレーム番号差が
    loop_min_keyframes 以上、時刻差が loop_min_gap 秒以上。
    検証: 近距離の候補から順にICP。既存の受理条件（fitness/重なり/退化除去）を
    そのまま使い、通ったものだけ拘束として採用する。1ノードにつき採用は最大1本。
    使用済みの印は採用後にだけ付け、不採用の候補が同じノードを使う別の再訪ペアの
    検証を妨げないようにする。
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
    attempts = np.zeros(n, dtype=int)
    for number, (distance, i, j) in enumerate(candidates, 1):
        if progress is not None:
            progress.update(number-1, f'採用 {len(edges)}件')
        if i in used or j in used:
            continue
        if max(attempts[i], attempts[j]) >= getattr(args, 'loop_max_attempts', 6):
            continue
        attempts[i] += 1
        attempts[j] += 1
        with np.load(cache/f'{i:06d}.npz') as target, np.load(cache/f'{j:06d}.npz') as source:
            relative = inverse(poses[i]) @ poses[j]
            lidar, meta = icp_edge(source['xyz'], target['xyz'], relative, args)
        meta.update({'from': i, 'to': j, 'type': 'loop',
                     'distance': distance, 'gap_s': float(times[j]-times[i])})
        if (lidar is not None and getattr(args, 'verify_loops', False)
                and (meta['rmse'] > max(3*args.lidar_sigma, args.icp_voxel)
                     or meta.get('weak_directions', 6) > 3)):
            lidar = None
            meta.update(accepted=False, reason='loop surface error/observability check failed')
        if lidar is not None and getattr(args, 'verify_loops', False):
            # A high one-way overlap can still align the wrong parallel wall.
            with np.load(cache/f'{i:06d}.npz') as target, np.load(cache/f'{j:06d}.npz') as source:
                reverse, reverse_meta = icp_edge(target['xyz'], source['xyz'], inverse(relative), args)
            cycle = pose_error(lidar[0] @ reverse[0], np.eye(4)) if reverse is not None else np.full(6, np.inf)
            # Error is normalized by the same physical surface/noise scale across bags.
            consistent = (reverse is not None and np.linalg.norm(cycle[:3]) <= 3*args.lidar_sigma
                          and np.linalg.norm(cycle[3:]) <= .05
                          and meta['rmse'] <= max(3*args.lidar_sigma, args.icp_voxel)
                          and meta.get('weak_directions', 6) <= 3)
            meta.update(reverse_fitness=reverse_meta.get('fitness', 0.),
                        cycle_translation_m=float(np.linalg.norm(cycle[:3])) if reverse is not None else None,
                        cycle_rotation_rad=float(np.linalg.norm(cycle[3:])) if reverse is not None else None)
            if not consistent:
                lidar = None
                meta.update(accepted=False, reason='loop reciprocity/geometry check failed')
        diagnostics.append(meta)
        if lidar is None:
            # Do not consume the nodes of a rejected candidate: a later, better
            # revisit pair may still need the same start or end node.
            continue
        used.update((i, j))
        edges.append((i, j, None, lidar, None))
    return edges, diagnostics


def find_covisibility_edges(poses, cache, args, progress=None):
    """Align overlapping scans at several baselines to limit accumulated tilt drift.

    Consecutive scans alone allow a small pitch error to accumulate over a long
    floor. These constraints use actual 3D returns and retain changes of slope.
    """
    if getattr(args, 'no_covisibility', False):
        return [], []
    edges, diagnostics = [], []
    if progress is not None:
        progress.begin(5, '複数の距離で重なる点群の照合', len(poses))
    for i in range(len(poses)):
        with np.load(cache/f'{i:06d}.npz') as frame:
            target = frame['xyz']
        for j in range(i+2, min(i+10, len(poses)), 2):
            relative = inverse(poses[i])@poses[j]
            if not .7 < np.linalg.norm(relative[:3, 3]) < 4.5:
                continue
            with np.load(cache/f'{j:06d}.npz') as frame:
                lidar, meta = icp_edge(frame['xyz'], target, relative, args)
            meta.update({'from': i, 'to': j, 'type': 'covisibility'})
            if lidar is not None and meta['rmse'] > max(3*args.lidar_sigma, args.icp_voxel):
                lidar = None
                meta.update(accepted=False, reason='surface alignment error too large')
            diagnostics.append(meta)
            if lidar is not None:
                edges.append((i, j, None, lidar, None))
        if progress is not None:
            progress.update(i+1, f'採用 {len(edges)}件')
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
        if np.any(np.isfinite(offsets) & (np.abs(offsets) > .5)):
            raise ValueError('Point time must be seconds relative to the PointCloud2 header (-0.5..0.5 s)')
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


def points(msg, args, deskewer=None, timing_msg=None):
    from sensor_msgs_py.point_cloud2 import read_points
    p = read_points(msg, skip_nans=False)
    xyz = np.column_stack([p[n].reshape(-1) for n in ('x', 'y', 'z')]).astype(float)
    offsets = p['time'] if 'time' in p.dtype.names else None
    if offsets is None and timing_msg is not None:
        raw = read_points(timing_msg, skip_nans=False)
        raw_xyz = np.column_stack([raw[n].reshape(-1) for n in ('x', 'y', 'z')])
        if timing_msg.header.frame_id != msg.header.frame_id or abs(stamp(timing_msg)-stamp(msg)) > 1e-6:
            raise ValueError('Raw/colored point cloud frame or timestamp mismatch')
        if xyz.shape != raw_xyz.shape or not np.allclose(xyz, raw_xyz, atol=1e-6, rtol=0, equal_nan=True):
            raise ValueError('Cannot recover point times: colored cloud changed point order or XYZ')
        if 'time' in raw.dtype.names:
            offsets = raw['time']
    if deskewer is not None and offsets is not None:
        xyz = deskewer.apply(xyz, offsets, stamp(msg))
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
    from sensor_consistency import check_imu
    sensor_health = check_imu(args, reader, wt, wp)
    for warning in sensor_health['warnings']:
        print('WARNING: '+warning, flush=True)
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
    if getattr(args, 'color_source', 'auto') == 'images' and not counts.get('/theta/dual_fisheye/image_raw/compressed', 0):
        raise ValueError('Image recoloring requested but the bag has no THETA source images')
    static_lidar, static_imu, static_transforms, sensor_frames = read_static_calibration(
        args.bag, reader, frames[1], topic, args.imu_topic)
    calibration_source = 'capture/config YAML'
    if static_lidar is not None and static_imu is not None:
        base_from_lidar = static_lidar
        imu_from_lidar_rot = static_imu[:3, :3].T @ static_lidar[:3, :3]
        calibration_source = 'recorded /tf_static'
        seed = seed_planar_motion(wt, wp, it, ip, args.max_sync_gap, static_imu[:3, :3].T) if args.wheel_model == 'planar' else wp
    else:
        with open(config) as f:
            wheel_config = yaml.safe_load(f)['/**']['ros__parameters'].get('wheel', {})
        if 'imu_to_base_translation' not in wheel_config:
            raise ValueError('Missing base/IMU calibration: record /tf_static or provide an explicit wheel.imu_to_base_translation')
    wheel_imu_seed = seed
    seed_times = wt
    lio_times, lio_base, lio_report = np.array([]), [], {'available': False}
    if args.trajectory != 'wheel' and args.mode != 'wheel':
        with open(config) as f:
            captured_params = yaml.safe_load(f)['/**']['ros__parameters']
        mapping = captured_params['mapping']
        imu_from_base = inverse(static_imu) if static_imu is not None else (
            transform(np.array(mapping['extrinsic_R']).reshape(3, 3), mapping['extrinsic_T']) @ inverse(base_from_lidar))
        lio_times, lio_base, lio_report = load_lio(args, reader, static_transforms, frames[1], imu_from_base)
    using_lio = len(lio_times) >= 2
    disagreement = False
    runtime_path = Path(args.lio_bag or args.bag).expanduser()/'capture'/'fastlio_runtime.json'
    runtime = json.loads(runtime_path.read_text()) if runtime_path.is_file() else {}
    frontend_wheel = (runtime['wheel_enabled'] if isinstance(runtime.get('wheel_enabled'), bool)
                      else captured_params.get('wheel', {}).get('enabled', False) if using_lio else False)
    lio_has_wheel = using_lio and (args.lio_wheel_fusion == 'already-fused' or (
        args.lio_wheel_fusion == 'auto' and frontend_wheel))
    lio_report['wheel_already_in_frontend'] = bool(lio_has_wheel)
    lio_report['wheel_policy_source'] = ('runtime snapshot' if isinstance(runtime.get('wheel_enabled'), bool)
                                        else 'capture YAML (actual runtime not recorded)')
    if args.trajectory == 'lio' and not using_lio:
        raise ValueError(f'No usable LIO poses on {args.lio_topic}')
    if using_lio:
        # A rigid frame alignment preserves every relative slope and height.
        # LIO's initial sensor frame is not necessarily a gravity-aligned frame.
        match = next(((t, p, interpolate_pose(t, wt, seed, args.max_sync_gap))
                      for t, p in zip(lio_times, lio_base)
                      if interpolate_pose(t, wt, seed, args.max_sync_gap) is not None), None)
        if match is None:
            raise ValueError('LIO and wheel trajectories have no synchronized overlap')
        _, lio_first, wheel_first = match
        yaw = lambda p: np.arctan2(p[1, 0], p[0, 0])
        rot = Rotation.from_euler('z', yaw(wheel_first)-yaw(lio_first)).as_matrix()
        if sensor_health.get('absolute_attitude_usable') and len(it) >= 2:
            rot = wheel_first[:3, :3]@lio_first[:3, :3].T
            lio_report['world_alignment'] = 'full rigid alignment to consistent initial IMU attitude'
        else:
            lio_report['world_alignment'] = 'yaw only; absolute map vertical unvalidated'
        alignment = transform(rot, wheel_first[:3, 3]-rot @ lio_first[:3, 3])
        aligned_lio = [alignment @ p for p in lio_base]
        ratio, disagreement = motion_consistency(wt, wp, lio_times, aligned_lio, interpolate_pose, args.max_sync_gap)
        lio_report.update(median_lio_wheel_travel_ratio=ratio,
                          initialization='wheel/IMU: sustained motion disagreement' if disagreement else 'FastLIO')
        if not disagreement:
            seed, seed_times = aligned_lio, lio_times
    print(f'Trajectory: {"FastLIO + adaptive wheel" if using_lio else "wheel + IMU"}; calibration: {calibration_source}', flush=True)
    import rosbag2_py
    recover_times = topic != args.raw_cloud_topic and counts.get(args.raw_cloud_topic, 0)
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic] + ([args.raw_cloud_topic] if recover_times else [])))
    output.mkdir(parents=True, exist_ok=True)
    cache = output/'frames'
    cache.mkdir()
    deskewer = CloudDeskewer(seed_times, seed, base_from_lidar, args.max_sync_gap) if not args.no_deskew else None
    origin, start = None, None
    initial, times, imu_poses, wheel_poses, lio_poses, kinematic_poses = [], [], [], [], [], []
    skipped = colored = 0
    last_candidate = None
    cloud_frame = None
    previous_stamp = None
    progress.begin(2, '点群の読み込み・キーフレーム作成', counts[topic])
    processed = 0
    recovered_scans = 0
    def add_frame(msg, pose, t, timing_msg=None):
        nonlocal skipped, colored, recovered_scans
        xyz, rgb, has_color = points(msg, args, deskewer, timing_msg)
        if timing_msg is not None and 'time' not in {f.name for f in msg.fields}:
            recovered_scans += 1
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
        kinematic_poses.append(interpolate_pose(t, wt, wheel_imu_seed, args.max_sync_gap))
        imu_poses.append(interpolate_pose(t, it, ip, args.max_sync_gap) if len(it) >= 2 else None)
        lio_poses.append(interpolate_pose(t, lio_times, aligned_lio, args.max_sync_gap) if using_lio else None)
        colored += int(has_color)
    raw_scans = {}
    while r.has_next():
        read_topic, data, _ = r.read_next()
        if read_topic != topic:
            raw = deserialize_message(data, PointCloud2)
            raw_scans[stamp(raw)] = raw
            while len(raw_scans) > 12:
                del raw_scans[next(iter(raw_scans))]
            continue
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
        if t-start > args.end:
            break
        if t-start < args.start:
            continue
        base = interpolate_pose(t, seed_times, seed, args.max_sync_gap)
        wheel_base = interpolate_pose(t, wt, wp, args.max_sync_gap)
        if base is None or wheel_base is None:
            skipped += 1
            continue
        if origin is None:
            if args.wheel_model == 'planar':
                raw_base = interpolate_pose(t, wt, wp, args.max_sync_gap)
                origin = inverse(transform(raw_base[:3, :3], np.r_[raw_base[:2, 3], 0.]))
            else:
                origin = inverse(base)
        pose = origin @ base @ base_from_lidar
        last_candidate = (msg, pose, t, raw_scans.get(t))
        if initial:
            delta = inverse(initial[-1]) @ pose
            if np.linalg.norm(delta[:3, 3]) < args.keyframe_distance-1e-9 and np.linalg.norm(Rotation.from_matrix(delta[:3, :3]).as_rotvec()) < np.deg2rad(args.keyframe_angle) and t-times[-1] < args.keyframe_interval:
                continue
        add_frame(*last_candidate)
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
            if sensor_health.get('absolute_attitude_usable') and not using_lio:
                # Gravity directions below supply absolute roll/pitch. Keep only
                # the relative yaw here rather than counting tilt twice.
                imu = {'yaw': float(np.arctan2(imu[1, 0], imu[0, 0]))}
        wheel = inverse(base_from_lidar) @ inverse(wheel_poses[i]) @ wheel_poses[i+1] @ base_from_lidar
        expected_body_z = (inverse(kinematic_poses[i])@kinematic_poses[i+1])[2, 3]
        if args.wheel_model == 'planar':
            wheel = (wheel, np.ones(3), expected_body_z)
        if using_lio:
            wheel_transform = wheel[0] if isinstance(wheel, tuple) else wheel
            relative_lio = (inverse(lio_poses[i] @ base_from_lidar) @ (lio_poses[i+1] @ base_from_lidar)
                            if lio_poses[i] is not None and lio_poses[i+1] is not None else None)
            discrepancy = (np.linalg.norm(lidar[1] @ pose_error(relative_lio, lidar[0]))
                           if lidar is not None and relative_lio is not None else None)
            imu_discrepancy = (Rotation.from_matrix(relative_lio[:3, :3] @ imu.T).magnitude()
                               if imu is not None and relative_lio is not None else 0.)
            use_lio_edge = (relative_lio is not None and (discrepancy is None or discrepancy <= 3.)
                            and imu_discrepancy <= max(.1, 3*args.imu_rotation_sigma))
            if disagreement and relative_lio is not None:
                wheel_motion = base_from_lidar @ wheel_transform @ inverse(base_from_lidar)
                lio_motion = base_from_lidar @ relative_lio @ inverse(base_from_lidar)
                wheel_distance = np.linalg.norm(wheel_motion[:3, 3])
                if wheel_distance > .05 and not .5 <= np.linalg.norm(lio_motion[:3, 3])/wheel_distance <= 2.:
                    use_lio_edge = False
            trusted = relative_lio if use_lio_edge else (lidar[0] if lidar is not None else relative)
            weights, fusion_meta = wheel_weights(trusted, wheel_transform, base_from_lidar, lidar,
                                                args.wheel_position_sigma, args.wheel_rotation_sigma)
            if args.wheel_model == 'planar':
                wheel = (wheel_transform, weights, expected_body_z)
            distance = np.linalg.norm(trusted[:3, 3])
            sigma = np.r_[np.full(3, args.lio_position_sigma+.01*distance),
                          np.full(3, args.lio_rotation_sigma)]
            if use_lio_edge:
                # Retain the measured surface alignment, including z/roll/pitch.
                # A plausible LIO increment can still contain a small systematic
                # tilt error which accumulates into doubled floors over a route.
                motion_info = unobservable_motion_information(
                    lidar, sigma, meta.get('char_length_m', 1.))
                if lidar is None:
                    lidar = (relative_lio, motion_info)
                elif np.linalg.norm(motion_info) > 1e-8:
                    edges.append((i, i+1, None, (relative_lio, motion_info), None))
                if lio_has_wheel:
                    wheel = None
                imu = None
            meta.update(measurement_source=('ICP + FastLIO nullspace' if use_lio_edge else 'ICP/IMU fallback'),
                        lio_accepted=bool(use_lio_edge),
                        fusion=fusion_meta, lio_geometric_innovation=discrepancy,
                        lio_imu_rotation_difference_rad=float(imu_discrepancy),
                        icp_used_as='surface measurement; accepted LIO supplies unobservable directions only')
        edges.append((i, i+1, wheel, lidar, imu))
        diagnostics.append({'from': i, 'to': i+1, **meta})
        progress.update(i+1)
        if (i+1) % 20 == 0:
            print(f'registered {i+1}/{len(initial)-1} edges', flush=True)
    progress.finish_stage()
    if sensor_health.get('absolute_attitude_usable'):
        for i, imu_pose in enumerate(imu_poses):
            if imu_pose is None:
                continue
            gravity = imu_from_lidar_rot.T@imu_pose[:3, :3].T@np.array([0., 0., 1.])
            edges.append((i, i, None, None, {'gravity': gravity}))
    loop_edges, first_pass = [], None
    if args.mode == 'optimize':
        # 1回目: 隣接拘束のみ。再訪候補の判定とICP初期値にこの姿勢を使う。
        progress.begin(4, '軌跡の最適化（1回目）', detail='終了割合は算出できません。経過時間と計算回数を更新します')
        poses, first_pass = optimize_poses(initial, edges, args, progress, base_from_lidar)
        first_pass['residuals'] = residual_breakdown(poses, edges, args, base_from_lidar)
        save_trajectory(output/'trajectory_first_pass.csv', times, poses, base_from_lidar)
        progress.finish_stage()
        covisibility, covisibility_diag = find_covisibility_edges(poses, cache, args, progress)
        diagnostics.extend(covisibility_diag)
        edges.extend(covisibility)
        print(f'covisibility edges={len(covisibility)}', flush=True)
        progress.begin(5, 'ループ閉じ込み候補の検索')
        loop_edges, loop_diag = find_loop_edges(poses, times, cache, args, progress)
        progress.finish_stage(f'確認 {len(loop_diag)}件・採用 {len(loop_edges)}件')
        diagnostics.extend(loop_diag)
        accepted = [(d['from'], d['to']) for d in loop_diag if d.get('accepted')]
        print(f'loop candidates={len(loop_diag)} accepted={len(loop_edges)} nodes={accepted}', flush=True)
        if loop_edges or covisibility:
            # 2回目: ループ拘束込みで全キーフレームを再最適化。
            # 初期値は生の初期軌道ではなく1回目の収束解を使う。生の初期軌道から
            # 始めると、ループの強い拘束に引っ張られて隣接拘束を壊す局所解へ
            # 落ちやすい（1回目は optimality 4.5e-3 まで収束している）。
            edges.extend(loop_edges)
            progress.begin(6, 'ループ補正を含む軌跡の最適化（2回目）', detail='終了割合は算出できません。経過時間と計算回数を更新します')
            poses, optimization = optimize_poses(poses, edges, args, progress, base_from_lidar)
            optimization['residuals'] = residual_breakdown(poses, edges, args, base_from_lidar)
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
    from mapping_quality import floor_overlap_quality, trajectory_jump_quality
    quality_before = floor_overlap_quality(cache, poses, times, base_from_lidar)
    floor_quality = quality_before
    surface_refinement = {'enabled': False}
    if args.mode == 'optimize' and not getattr(args, 'no_surface_refinement', False):
        from mapping_bundle import refine_surfaces
        progress.begin(6, '共有する面と全姿勢の同時補正', detail='床・壁の観測を複数の走行位置から照合します')
        candidate, surface_refinement = refine_surfaces(
            poses, cache, edges, args,
            lambda p: pose_graph_residual(p, edges, args, base_from_lidar), progress,
            edge_residual=lambda a, b, e: pose_graph_residual(
                [a, b], [(0, 1, *e[2:])], args, base_from_lidar))
        candidate_quality = floor_overlap_quality(cache, candidate, times, base_from_lidar, reference=quality_before)
        key = 'p95_pair_p95_height_difference_m'
        reject = (quality_before.get('revisit_pairs', 0) >= 20 and (
            candidate_quality.get('revisit_pairs', 0) < .9*quality_before['revisit_pairs']
            or candidate_quality.get(key, np.inf) > quality_before[key]+.002
            or candidate_quality.get('max_pair_p95_height_difference_m', np.inf)
               > quality_before['max_pair_p95_height_difference_m']+.005))
        surface_refinement.update(accepted=not reject, floor_quality_before=quality_before,
                                  floor_quality_candidate=candidate_quality)
        if reject:
            save_trajectory(output/'trajectory_surface_rejected.csv', times, candidate, base_from_lidar)
            print('共有面補正は再訪床面の重なりを悪化させたため不採用。補正前の姿勢を保存します。', flush=True)
        else:
            poses, floor_quality = candidate, candidate_quality
        surface_refinement['graph_residuals'] = residual_breakdown(poses, edges, args, base_from_lidar)
        progress.finish_stage()
    floor_refinement = None
    optimization_before_floor = None
    if getattr(args, 'known_single_floor', False):
        from mapping_floor import refine_known_floor
        print('既知の単一平坦床として高さ・傾きを補正・検証します。', flush=True)
        save_trajectory(output/'trajectory_before_floor.csv', times, poses, base_from_lidar)
        poses, floor_refinement = refine_known_floor(cache, poses, times, base_from_lidar,
            [(e['from'], e['to']) for e in diagnostics if e.get('accepted')])
        level = np.asarray(floor_refinement['frame_transform_old_map_to_new_map'])
        initial = [level @ p for p in initial]
        origin = level @ origin
        floor_quality = floor_refinement['floor_quality_candidate']
        optimization_before_floor = optimization
        optimization = dict(success=floor_refinement['success'] and optimization_before_floor['success'], cost=floor_refinement['cost'],
            evaluations=floor_refinement['evaluations'], method=floor_refinement['model'])
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
    dynamic_report = {'enabled': not args.no_dynamic_filter}
    if not args.no_dynamic_filter:
        write_map(output/'map_3d_unfiltered.ply', xyz)
        write_map(output/'map_3d_colored_unfiltered.ply', xyz, rgb)
        def frame_points(i):
            with np.load(cache/f'{i:06d}.npz') as frame:
                return frame['xyz'].copy()
        if getattr(args, 'dynamic_method', 'visibility') == 'dufomap':
            from mapping_dynamic import dufomap_filter
            keep, dynamic_diag = dufomap_filter(xyz, frame_points, poses, args, progress, times=times)
        else:
            keep, dynamic_diag = visibility_filter(xyz, frame_points, poses, args, progress, times=times)
        dynamic_report.update(dynamic_diag)
        from mapping_dynamic import finishing_indices
        fragment_ids = finishing_indices(dynamic_diag)
        write_map(output/'dynamic_fragments.ply', xyz[fragment_ids], rgb[fragment_ids])
        write_map(output/'dynamic_candidates.ply', xyz[~keep], rgb[~keep])
        np.savez_compressed(output/'dynamic_mask.npz', keep=keep)
        xyz, rgb = xyz[keep], rgb[keep]
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
    lio_report['accepted_edges'] = sum(e.get('lio_accepted', False) for e in diagnostics)
    lio_report['rejected_edges'] = sum(e.get('lio_accepted') is False for e in diagnostics)
    jump_quality = trajectory_jump_quality(poses, times, args.max_trajectory_speed)
    jump_warning = jump_quality['jump_count'] > 0
    floor_warning = floor_quality.get('p95_pair_p95_height_difference_m', 0.) > .05
    sensor_warning = bool(sensor_health['warnings'])
    validation_status = ('needs_review' if floor_warning or jump_warning or sensor_warning or not optimization['success'] else
                         'revisit_consistency_checked' if floor_quality.get('revisit_pairs', 0) else 'no_floor_revisit_evidence')
    import hashlib
    implementation = {name: hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()
                      for name in ('offline_3d_mapping.py', 'mapping_fusion.py', 'mapping_dynamic.py', 'mapping_bundle.py', 'mapping_quality.py', 'mapping_geometry.py', 'mapping_progress.py', 'sensor_consistency.py')}
    report = {'ground_truth_evaluation': evaluation, 'bag': str(args.bag), 'config': str(config.resolve()), 'mode': args.mode,
              'implementation_sha256': implementation,
              'calibration_source': calibration_source, 'resolved_base_from_lidar': base_from_lidar.tolist(),
              'sensor_frames': sensor_frames, 'lio': {**lio_report, 'used_for_mapping': using_lio},
              'dynamic_filter': dynamic_report, 'recovered_point_time_scans': recovered_scans,
              'cloud_topic': topic, 'wheel_topic': args.wheel_topic, 'imu_topic': args.imu_topic,
              'wheel_frames': frames, 'keyframes': len(initial), 'points': len(xyz),
              'deskew': {'scans': deskewer.scans, 'points': deskewer.points, 'dropped_points': deskewer.dropped} if deskewer else None,
              'cloud_frame': cloud_frame, 'imu_edges': sum(e[4] is not None for e in edges),
              'gravity_edges': sum(isinstance(e[4], dict) and 'gravity' in e[4] for e in edges),
              'loops': len(loop_edges), 'loop_residuals': loop_residuals,
              'covisibility_edges': sum(e.get('type') == 'covisibility' and e.get('accepted', False) for e in diagnostics),
              'options': {k: (str(v) if isinstance(v, Path) else None if isinstance(v, float) and not np.isfinite(v) else v)
                          for k, v in vars(args).items()},
              'skipped_scans': skipped, 'rgb_keyframes': colored,
              'optimization_first_pass': first_pass, 'optimization': optimization,
              'surface_refinement': surface_refinement,
              'floor_overlap_quality': floor_quality,
              'trajectory_jump_quality': jump_quality,
              'validation_status': validation_status,
              'sensor_consistency': sensor_health,
              'edges': diagnostics, 'limitations': [
                  ('Loop closure disabled (--no-loop)' if args.no_loop else
                   f'Loop closure enabled (radius {args.loop_radius:g} m, gap >= {args.loop_min_gap:g} s, '
                   f'min keyframe gap {args.loop_min_keyframes}); accepted edges: {len(loop_edges)}'),
                  'Signed point-time seconds are deskewed using LIO or wheel/IMU motion; legacy colored scans recover time only after exact raw XYZ/order validation',
                  'Adjacent ICP retains observable surface alignment; accepted LIO supplies only its nullspace and initialization/deskew',
                  'Shared local plane refinement adjusts 3D poses without assuming a globally level floor',
                  'Visibility cleaning preserves unknown/occluded space; objects never observed to move can remain',
                  'IMU orientation constraints are used only when marked valid in the bag',
                  'Ground truth is recorded for evaluation only and is never an optimization input',
                  'Encoders do not measure world height; a soft body-normal motion prior uses wheel/IMU kinematics and allows slope transitions']}
    if floor_refinement is not None:
        report.update(floor_refinement=floor_refinement,
            optimization_before_floor_refinement=optimization_before_floor,
            source_pose_graph_evidence='source graph supplied correction smoothness links; measured factors were not reoptimized')
        report['surface_refinement_before_floor'] = report.pop('surface_refinement')
        report['optimization_first_pass_before_floor_refinement'] = report.pop('optimization_first_pass')
        for name in ('mapping_floor.py',):
            report['implementation_sha256'][name] = hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()
    if getattr(args, 'dynamic_polish', False):
        report['implementation_sha256']['mapping_cleanup.py'] = hashlib.sha256((Path(__file__).parent/'mapping_cleanup.py').read_bytes()).hexdigest()
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    import shutil
    shutil.copyfile(config, output/'calibration.yaml')
    if static_lidar is not None and static_imu is not None:
        with open(config) as f:
            resolved = yaml.safe_load(f)
        params = resolved['/**']['ros__parameters']
        imu_lidar = inverse(static_imu) @ static_lidar
        params['mapping']['extrinsic_R'] = imu_lidar[:3, :3].reshape(-1).tolist()
        params['mapping']['extrinsic_T'] = imu_lidar[:3, 3].tolist()
        params.setdefault('wheel', {}).update(child_frame=frames[1],
            imu_to_base_translation=inverse(static_imu)[:3, 3].tolist(),
            imu_from_base_rotation=inverse(static_imu)[:3, :3].reshape(-1).tolist())
        (output/'resolved_calibration.yaml').write_text(yaml.safe_dump(resolved, sort_keys=False))
    color_mode = getattr(args, 'color_source', 'auto')
    recolored = False
    if color_mode == 'images' or (color_mode == 'auto' and counts.get('/theta/dual_fisheye/image_raw/compressed', 0)
                                  and counts.get(args.raw_cloud_topic, 0)):
        from recolor_3d_map import recolor_new_output
        print('元画像から固定校正・遮蔽判定付きで再着色します。軌跡と地図形状は保持します。', flush=True)
        recolor_new_output(output, args.bag, getattr(args, 'colorization_config', None),
                           getattr(args, 'theta_calibration', None))
        recolored = True
    else:
        report['colorization'] = {'source': 'recorded RGB', 'reason': 'explicit recorded policy or no source image/raw scan pair'}
        (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    if not args.keep_cache:
        shutil.rmtree(cache)
    print(f'Saved: {output}/map_3d.ply and map_3d_colored.ply ({len(xyz)} identical XYZ points)', flush=True)
    if not colored and not recolored:
        print('WARNING: no RGB in input; colored map is neutral gray', file=sys.stderr)
    if len(initial) > 1 and not using_lio and not any(e[4] is not None for e in edges):
        print('WARNING: no valid IMU orientation constraints; using wheel and LiDAR only', file=sys.stderr)
    if not optimization['success']:
        print('WARNING: optimizer stopped before convergence; inspect report.json', file=sys.stderr)
    if floor_quality.get('revisit_pairs', 0):
        print(f"再訪床面の高さ差（ペア内95%点の、ペア間95%値）: {floor_quality['p95_pair_p95_height_difference_m']*100:.1f} cm。絶対精度ではありません。", flush=True)
        if floor_quality['p95_pair_p95_height_difference_m'] > .05:
            print('WARNING: residual floor misalignment exceeds 5 cm; convergence alone does not validate map geometry', file=sys.stderr)
    if jump_warning:
        print(f"WARNING: {jump_quality['jump_count']} abnormal keyframe speed spike(s) detected in the optimized trajectory; inspect trajectory_jump_quality in report.json", file=sys.stderr)
    progress.finish_stage()
    reasons = []
    if floor_warning:
        reasons.append('再訪床面に5cmを超える残差があります。床の重なりを確認してください。')
    if jump_warning:
        reasons.append('最適化後の軌跡に異常な速度の飛びがあります。地図の連続性を確認してください。')
    if sensor_warning:
        reasons.append('IMUの姿勢と重力方向に不整合があります。絶対的な地図の上下方向は未検証です。')
    reason = ' '.join(reasons) or None
    progress.complete(output, len(xyz), warning=floor_warning or jump_warning or sensor_warning or not optimization['success'], warning_reason=reason)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', nargs='?', help='Recorded bag directory; omit to choose by number')
    parser.add_argument('--directory', default='~/rosbag2_data', help='Directory to list recorded bags')
    parser.add_argument('-o', '--output', help='Empty output directory (default: <bag>/offline_3d)')
    parser.add_argument('--config', help='FAST-LIO YAML; default: bag/capture/calibration.yaml')
    parser.add_argument('--mapping-config', default=str(Path(__file__).resolve().parents[2]/'params/offline_3d_mapping.yaml'),
                        help='Shared backend YAML policy; explicit CLI options override it')
    parser.add_argument('--mode', choices=['optimize', 'wheel'], default='optimize')
    parser.add_argument('--known-single-floor', action='store_true',
                        help='実際に同一平面の屋内床と確認できる場合のみ、高さ・傾きを補正して検証する')
    parser.add_argument('--cloud-topic')
    parser.add_argument('--color-source', choices=['auto', 'recorded', 'images'], default='auto',
                        help='auto: recolor from THETA source images after geometry processing when available')
    parser.add_argument('--colorization-config', help='Shared robot LiDAR/THETA extrinsic and visibility policy YAML')
    parser.add_argument('--theta-calibration', help='Override THETA intrinsic calibration for image recoloring')
    parser.add_argument('--raw-cloud-topic', default='/velodyne_points', help='Recover missing colored-cloud point times from identical raw XYZ')
    parser.add_argument('--trajectory', choices=['auto', 'lio', 'wheel'], default='auto',
                        help='auto: recorded FastLIO trajectory + adaptive wheels when available; otherwise wheel/IMU')
    parser.add_argument('--lio-topic', default='/Odometry')
    parser.add_argument('--lio-bag', help='Optional separately recomputed FastLIO pose bag; raw sensor bag remains unchanged')
    parser.add_argument('--frontend', choices=['auto', 'recorded', 'recompute'], default='auto',
                        help='auto: freshly recompute FastLIO for bags captured as real; recorded: use existing poses')
    parser.add_argument('--lio-wheel-fusion', choices=['auto', 'independent', 'already-fused'], default='auto',
                        help='Avoid double-counting wheel factors when capture YAML says FastLIO already fused wheels')
    parser.add_argument('--lio-position-sigma', type=float, default=.10, help='Conservative LIO relative-position floor [m]')
    parser.add_argument('--lio-rotation-sigma', type=float, default=.03, help='Conservative LIO relative-rotation floor [rad]')
    parser.add_argument('--wheel-topic', default='/odom')
    parser.add_argument('--wheel-model', choices=['planar', 'se3'], default='planar',
                        help='planar: encoder x/y/yaw only, initialize slope height from IMU; se3: full 3D pose sensor')
    parser.add_argument('--imu-topic', default='/imu')
    parser.add_argument('--wheel-vertical-sigma', type=float, default=.10,
                        help='Soft body-normal displacement uncertainty [m]; world height remains free on slopes')
    parser.add_argument('--ground-truth-topic', default='/ground_truth/odom', help='Optional evaluation only; never used for mapping')
    for name, default in [('voxel', .05), ('icp-voxel', .2), ('keyframe-distance', .5),
                          ('keyframe-angle', 8.), ('keyframe-interval', 2.),
                          ('max-sync-gap', .2), ('max-correspondence', .5), ('min-fitness', .3),
                          ('degeneracy-ratio', .01), ('lidar-sigma', .05),
                          ('wheel-position-sigma', .03), ('wheel-rotation-sigma', .03),
                          ('imu-rotation-sigma', .05), ('min-range', .5), ('max-range', 80.)]:
        parser.add_argument('--'+name, type=float, default=default)
    parser.add_argument('--max-iterations', type=int, default=300)
    parser.add_argument('--max-trajectory-speed', type=float, default=2.0,
                        help='Abnormal-jump check: physical keyframe-to-keyframe speed cap [m/s]')
    parser.add_argument('--max-keyframes', type=int, default=4000)
    parser.add_argument('--start', type=float, default=0., help='Seconds from first cloud')
    parser.add_argument('--end', type=float, default=float('inf'))
    parser.add_argument('--keep-cache', action='store_true')
    parser.add_argument('--no-deskew', action='store_true', help='Disable per-point encoder/IMU motion compensation')
    parser.add_argument('--no-dynamic-filter', action='store_true', help='Keep accumulated points without visibility cleaning')
    parser.add_argument('--dynamic-method', choices=['visibility', 'dufomap'], default='visibility')
    parser.add_argument('--dynamic-resolution', type=float, default=.1, help='DUFOMap free-space voxel [m], independent of map detail')
    parser.add_argument('--dynamic-surface-margin', type=float, default=.2, help='DUFOMap d_s: hit inflation [m]')
    parser.add_argument('--dynamic-pose-margin', type=int, default=2, help='DUFOMap d_p: unknown-neighborhood inflation [voxels]')
    parser.add_argument('--dynamic-threads', type=int, default=4)
    parser.add_argument('--no-dynamic-fragments', action='store_true', help='Disable local isolated fragment cleanup; use official DUFOMap alone')
    parser.add_argument('--dynamic-polish', action='store_true', help='DUFOMap後に、時刻を離したレーザー通過証拠で浮遊点・残像を追加仕上げ')
    parser.add_argument('--dynamic-fragment-radius', type=float, default=.12)
    parser.add_argument('--dynamic-fragment-max-points', type=int, default=120)
    parser.add_argument('--dynamic-fragment-max-extent', type=float, default=.8)
    parser.add_argument('--dynamic-fragment-seed-radius', type=float, default=.25)
    parser.add_argument('--dynamic-fragment-revisit-gap', type=float, default=8.)
    parser.add_argument('--dynamic-fragment-match-radius', type=float, default=.08)
    parser.add_argument('--dynamic-range', type=float, default=30.)
    parser.add_argument('--dynamic-margin', type=float, default=.20, help='Conservative surface/pose uncertainty margin [m]')
    parser.add_argument('--dynamic-window', type=float, default=8., help='Gap separating direct-observation episodes for the abandonment rule [s]')
    parser.add_argument('--dynamic-min-votes', type=int, default=3)
    parser.add_argument('--dynamic-free-hit-ratio', type=float, default=2.,
                        help='Free-space votes must exceed this multiple of direct hits unless abandonment evidence exists')
    parser.add_argument('--dynamic-hit-cluster-max', type=int, default=2,
                        help='Max direct-observation episodes for the abandonment rule')
    parser.add_argument('--no-loop', action='store_true', help='Disable loop closure')
    parser.add_argument('--no-covisibility', action='store_true', help='Disable overlapping scans at multiple baselines (diagnosis only)')
    parser.add_argument('--no-planar-normals', action='store_true', help='Use only local PCA normals (diagnosis only)')
    parser.add_argument('--no-surface-refinement', action='store_true', help='Disable joint scan/plane refinement (diagnosis only)')
    parser.add_argument('--surface-rounds', type=int, default=3, help='Shared-plane reassociation/refinement passes')
    parser.add_argument('--no-loop-verification', dest='verify_loops', action='store_false',
                        help='Disable reciprocal ICP and geometric quality checks (diagnosis only)')
    parser.add_argument('--loop-radius', type=float, default=6.0,
                        help='Revisit candidate distance threshold [m]')
    parser.add_argument('--loop-min-gap', type=float, default=10.0,
                        help='Minimum time gap between revisited keyframes [s]')
    parser.add_argument('--loop-min-keyframes', type=int, default=8,
                        help='Minimum keyframe index gap for a revisit candidate')
    parser.add_argument('--loop-max-attempts', type=int, default=6, help='Bound ICP trials per node; rejected candidates still allow alternatives')
    pre, _ = parser.parse_known_args(argv)
    policy = Path(pre.mapping_config).expanduser()
    if not policy.is_file():
        parser.error(f'Mapping policy not found: {policy}')
    with policy.open() as f:
        defaults = yaml.safe_load(f).get('parameters', {})
    actions = {action.dest: action for action in parser._actions}
    for key, value in defaults.items():
        if key not in actions or key in {'bag', 'config', 'output', 'directory', 'mapping_config', 'help', 'known_single_floor'}:
            parser.error(f'Unsupported mapping policy key: {key}')
        if actions[key].choices and value not in actions[key].choices:
            parser.error(f'Invalid mapping policy value: {key}={value}')
    parser.set_defaults(**defaults)
    args = parser.parse_args(argv)
    if args.dynamic_polish and (args.no_dynamic_filter or args.dynamic_method != 'dufomap'):
        parser.error('--dynamic-polish requires enabled DUFOMap filtering')
    positive = [args.voxel, args.icp_voxel, args.keyframe_distance, args.keyframe_angle,
                args.keyframe_interval, args.max_sync_gap, args.max_correspondence,
                args.lidar_sigma, args.wheel_position_sigma, args.wheel_rotation_sigma,
                args.imu_rotation_sigma, args.max_range, args.loop_radius, args.loop_min_gap,
                args.lio_position_sigma, args.lio_rotation_sigma, args.dynamic_range, args.dynamic_margin, args.dynamic_window,
                args.wheel_vertical_sigma, args.max_trajectory_speed]
    if not all(np.isfinite(v) and v > 0 for v in positive) or not 0 < args.degeneracy_ratio < 1 or not 0 < args.min_fitness <= 1 or args.start < 0 or args.end < args.start or args.max_iterations < 1 or args.max_keyframes < 1 or args.loop_min_keyframes < 1:
        parser.error('Invalid range, noise, optimizer, loop, or sampling option')
    if args.dynamic_min_votes < 2:
        parser.error('--dynamic-min-votes must be at least 2')
    if args.dynamic_free_hit_ratio < 1:
        parser.error('--dynamic-free-hit-ratio must be at least 1')
    if args.dynamic_hit_cluster_max < 1:
        parser.error('--dynamic-hit-cluster-max must be positive')
    if (not np.isfinite(args.dynamic_resolution) or args.dynamic_resolution <= 0
            or not np.isfinite(args.dynamic_surface_margin) or args.dynamic_surface_margin < 0
            or args.dynamic_pose_margin < 1 or args.dynamic_threads < 1):
        parser.error('Invalid DUFOMap resolution, margins or threads')
    if (args.dynamic_fragment_max_points < 1 or not all(np.isfinite(v) and v > 0 for v in (
            args.dynamic_fragment_radius, args.dynamic_fragment_max_extent, args.dynamic_fragment_seed_radius,
            args.dynamic_fragment_revisit_gap, args.dynamic_fragment_match_radius))):
        parser.error('Invalid fragment cleanup size, distance or time')
    if args.loop_max_attempts < 1:
        parser.error('--loop-max-attempts must be positive')
    if not 1 <= args.surface_rounds <= 10:
        parser.error('--surface-rounds must be between 1 and 10')
    try:
        if not args.no_dynamic_filter and args.dynamic_method == 'dufomap':
            from mapping_dynamic import require_dufomap
            require_dufomap()  # Fail before expensive frontend/trajectory work.
        if args.bag is None:
            from map_selection import bag_candidates, choose_numbered, next_output_directory
            args.bag = choose_numbered(lambda: bag_candidates(args.directory), '後処理するBAG', args.directory)
            if args.bag is None:
                return 0
            if args.output is None:
                args.output = str(next_output_directory(args.bag))
            print(f'選択したBAG: {args.bag}\n保存先: {args.output}', flush=True)
        raw_bag = Path(args.bag).expanduser().resolve()
        destination = Path(args.output).expanduser() if args.output else raw_bag/'offline_3d'
        if destination.exists() and (not destination.is_dir() or any(destination.iterdir())):
            raise ValueError(f'Output is not empty: {destination}. Use another --output directory.')
        capture_record = raw_bag/'capture/recording.json'
        capture_mode = json.loads(capture_record.read_text()).get('mode') if capture_record.is_file() else None
        recompute = args.frontend == 'recompute' or (args.frontend == 'auto' and capture_mode == 'real')
        if recompute and args.lio_bag is None and args.mode != 'wheel' and args.trajectory != 'wheel':
            import subprocess
            pose_bag = raw_bag/'recomputed_lio'
            number = 2
            while pose_bag.exists():
                pose_bag = raw_bag/f'recomputed_lio_{number}'
                number += 1
            print('実機BAG: 新しい地図・初期化状態でFastLIO＋車輪を先に再計算します。元BAGは変更しません。', flush=True)
            command = [sys.executable, str(Path(__file__).with_name('recompute_fastlio.py')),
                       str(raw_bag), '--output', str(pose_bag)]
            if args.config:
                command.extend(['--config', str(args.config)])
            if np.isfinite(args.end):
                command.extend(['--duration', str(args.end+2.)])
            result = subprocess.run(command)
            if result.returncode:
                raise RuntimeError('FastLIO recomputation failed; see the pose bag log. --frontend recorded uses recorded/fallback poses explicitly.')
            args.lio_bag = str(pose_bag)
        return run(args)
    except (ValueError, RuntimeError, ImportError, OSError, EOFError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
