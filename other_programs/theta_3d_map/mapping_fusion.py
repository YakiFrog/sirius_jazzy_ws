"""Offline sensor contracts, conservative wheel/LIO fusion and visibility cleaning.

This is a small backend inspired by LIO-SAM, degeneracy-aware fusion and Dynablox;
it is not a reimplementation of those complete estimators. See MAPPING_METHOD.md.
"""
from collections import deque

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def rigid(rotation=None, position=None):
    p = np.eye(4)
    if rotation is not None:
        p[:3, :3] = rotation
    if position is not None:
        p[:3, 3] = position
    return p


def invert(p):
    return rigid(p[:3, :3].T, -p[:3, :3].T @ p[:3, 3])


def message_pose(msg):
    q, p = msg.pose.pose.orientation, msg.pose.pose.position
    quat = np.array([q.x, q.y, q.z, q.w])
    xyz = np.array([p.x, p.y, p.z])
    if not np.isfinite(np.r_[quat, xyz]).all() or np.linalg.norm(quat) < .5:
        return None
    return rigid(Rotation.from_quat(quat).as_matrix(), xyz)


def static_lookup(transforms, parent, child):
    """Return parent_from_child, following only recorded static rigid transforms."""
    if parent == child:
        return np.eye(4)
    graph = {}
    for a, b, pose in transforms:
        graph.setdefault(a, []).append((b, pose))
        graph.setdefault(b, []).append((a, invert(pose)))
    todo, seen = deque([(parent, np.eye(4))]), {parent}
    while todo:
        node, pose = todo.popleft()
        for nxt, edge in graph.get(node, []):
            if nxt in seen:
                continue
            result = pose @ edge
            if nxt == child:
                return result
            todo.append((nxt, result))
            seen.add(nxt)
    return None


def read_static_calibration(bag, reader, base, cloud_topic, imu_topic):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu, PointCloud2
    from tf2_msgs.msg import TFMessage
    r = reader(bag, ['/tf_static', cloud_topic, imu_topic])
    count = next((t.message_count for t in r.get_metadata().topics_with_message_count
                  if t.topic_metadata.name == '/tf_static'), 0)
    transforms, frames, received = [], {}, 0
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == '/tf_static':
            received += 1
            for t in deserialize_message(data, TFMessage).transforms:
                q, p = t.transform.rotation, t.transform.translation
                transforms.append((t.header.frame_id.lstrip('/'), t.child_frame_id.lstrip('/'),
                                   rigid(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(),
                                         [p.x, p.y, p.z])))
        elif topic not in frames:
            msg = deserialize_message(data, Imu if topic == imu_topic else PointCloud2)
            frames[topic] = msg.header.frame_id.lstrip('/')
        if received >= count and len(frames) == 2:
            break
    lidar = static_lookup(transforms, base.lstrip('/'), frames.get(cloud_topic, ''))
    imu = static_lookup(transforms, base.lstrip('/'), frames.get(imu_topic, ''))
    return lidar, imu, transforms, frames


def load_lio(args, reader, transforms, base_frame, imu_from_base):
    """Read a single continuous LIO trajectory; never infer that zero covariance is exact."""
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    source = getattr(args, 'lio_bag', None) or args.bag
    r = reader(source, [args.lio_topic])
    samples, frames, rejected = {}, None, 0
    for_meta = any(t.topic_metadata.name == args.lio_topic and t.message_count
                   for t in r.get_metadata().topics_with_message_count)
    if not for_meta:
        return np.array([]), [], {'topic': args.lio_topic, 'available': False}
    child_from_base = None
    last_time, last_pose = None, None
    while r.has_next():
        _, data, _ = r.read_next()
        msg = deserialize_message(data, Odometry)
        pair = (msg.header.frame_id.lstrip('/'), msg.child_frame_id.lstrip('/'))
        if frames is None:
            frames = pair
            child_from_base = static_lookup(transforms, pair[1], base_frame.lstrip('/'))
            # FAST-LIO's documented `body` is the IMU frame; other unknown frames fail.
            if child_from_base is None and pair[1] == 'body':
                child_from_base = imu_from_base
            if child_from_base is None:
                raise ValueError(f'Cannot resolve LIO child frame {pair[1]} to {base_frame}')
        if pair != frames:
            raise ValueError('LIO frame changed inside the bag')
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        pose = message_pose(msg)
        if pose is None:
            rejected += 1
            continue
        pose = pose @ child_from_base
        if last_time is not None:
            dt = stamp-last_time
            if dt < -.5:
                raise ValueError('LIO timestamp reset inside the bag')
            if dt <= 0:
                rejected += 1
                continue
            angle = Rotation.from_matrix(last_pose[:3, :3].T @ pose[:3, :3]).magnitude()
            if np.linalg.norm(pose[:3, 3]-last_pose[:3, 3]) > 10.*dt+.5 or angle > 5.*dt+.3:
                raise ValueError('LIO pose jump/reset detected; split the bag or use --trajectory wheel')
        samples[stamp] = pose
        last_time, last_pose = stamp, pose
    times = np.array(sorted(samples))
    return times, [samples[t] for t in times], {
        'topic': args.lio_topic, 'bag': str(source), 'available': len(times) >= 2, 'samples': len(times),
        'frames': frames, 'invalid_messages': rejected,
        'covariance_policy': 'conservative backend floors; correlated frontend covariance is not independent evidence'}


def wheel_weights(relative_lio, wheel, mount, icp, position_sigma, rotation_sigma):
    """Wheel supports weak axes; downweight disagreement only on observable axes.

    LIO and wheel disagreement alone cannot identify which sensor failed. Require
    geometric support for translation slip gating. Yaw has an inertial reference.
    """
    predicted = mount @ relative_lio @ invert(mount)
    measured = mount @ wheel @ invert(mount)
    delta = predicted[:2, 3]-measured[:2, 3]
    angle = Rotation.from_matrix(predicted[:3, :3] @ measured[:3, :3].T).as_rotvec()[2]
    strength = np.zeros(2)
    if icp is not None:
        a = icp[1][:, :3] @ mount[:3, :3].T
        norms = np.linalg.norm(a, axis=0)
        strength = np.clip(norms[:2]*.1, 0., 1.)
    gate = 1./np.sqrt(1.+(delta/(3*position_sigma+.05))**2)
    # In a well-constrained direction LIO leads; weak directions retain encoders.
    weights = np.r_[(1-strength) + strength*.2*gate,
                    .2/np.sqrt(1.+(angle/(3*rotation_sigma+.03))**2)]
    return weights, {'weights': weights.tolist(), 'wheel_lio_translation_m': delta.tolist(),
                     'wheel_lio_yaw_rad': float(angle), 'lidar_axis_strength': strength.tolist()}


def motion_consistency(wt, wp, lt, lp, interpolate, max_gap):
    """Initializer sanity check, not proof that either sensor is correct."""
    ratios = []
    for k in range(0, len(wt)-10, 10):
        p0 = interpolate(wt[k], lt, lp, max_gap)
        p1 = interpolate(wt[k+10], lt, lp, max_gap)
        travel = np.linalg.norm(wp[k+10][:3, 3]-wp[k][:3, 3])
        if p0 is not None and p1 is not None and travel > .15:
            ratios.append(np.linalg.norm(p1[:3, 3]-p0[:3, 3])/travel)
    ratio = float(np.median(ratios)) if ratios else None
    disagreement = len(ratios) >= 10 and not .5 <= ratio <= 2.
    return ratio, disagreement


def visibility_filter(xyz, frame_points, poses, args, progress=None, times=None):
    """Remove contradicted surfaces, preserving unknown/occluded space and kept objects.

    Inspired by range-image map cleaning and the conservative free-space
    principles of Dynablox/DUFOMap: a space observed empty at any time cannot be
    occupied by a static point. A point whose direct observations form a single
    episode followed by later free-space evidence is treated as an object that
    left the scene; objects still observed as a surface late in the session are
    kept even when aliased free votes exist. This does not reproduce those
    systems' full implementations.
    """
    if not len(xyz):
        return np.ones(0, bool), {'removed_points': 0}
    tree = cKDTree(xyz)
    hits = np.zeros(len(xyz), dtype=np.uint16)
    free = np.zeros(len(xyz), dtype=np.uint16)
    hit_clusters = np.zeros(len(xyz), dtype=np.uint16)
    last_hit = np.full(len(xyz), -np.inf)
    previous_hit = np.full(len(xyz), -np.inf)
    # VLP16: use narrow angular bins; missing rays and foreground occlusions are unknown.
    az_step, el_step = np.deg2rad(.10), np.deg2rad(.10)
    n_az, n_el = int(np.ceil(2*np.pi/az_step)), int(np.ceil(np.pi/el_step))
    def bins(p):
        distance = np.linalg.norm(p, axis=1)
        az = np.floor((np.arctan2(p[:, 1], p[:, 0])+np.pi)/az_step).astype(int) % n_az
        el = np.clip(np.floor((np.arcsin(np.clip(p[:, 2]/np.maximum(distance, 1e-9), -1, 1))
                              +np.pi/2)/el_step).astype(int), 0, n_el-1)
        return az*n_el+el, distance
    def scan_table(i):
        ids, distances = bins(frame_points(i))
        table = np.full(n_az*n_el, np.inf, dtype=np.float32)
        np.minimum.at(table, ids, distances)
        return table
    # Time-spaced keyframes prevent long stationary bursts from dominating votes.
    selected = []
    times = np.arange(len(poses), dtype=float) if times is None else np.asarray(times)
    for i, pose in enumerate(poses):
        if not selected or (np.linalg.norm(pose[:3, 3]-poses[selected[-1]][:3, 3]) >= .15
                            or Rotation.from_matrix(poses[selected[-1]][:3, :3].T @ pose[:3, :3]).magnitude() >= .08
                            or times[i]-times[selected[-1]] >= 1.):
            selected.append(i)
    cluster_gap = getattr(args, 'dynamic_window', 8.)
    free_hit_ratio = getattr(args, 'dynamic_free_hit_ratio', 2)
    cluster_max = getattr(args, 'dynamic_hit_cluster_max', 2)
    for number, i in enumerate(selected):
        pose = poses[i]
        table = scan_table(i)
        candidates = np.asarray(tree.query_ball_point(pose[:3, 3], args.dynamic_range), dtype=int)
        local = (xyz[candidates]-pose[:3, 3]) @ pose[:3, :3]
        keys, ranges = bins(local)
        observed = table[keys]
        margin = np.maximum(args.dynamic_margin, ranges*.005)
        hit_ids = candidates[np.abs(observed-ranges) <= margin]
        hits[hit_ids] += 1
        hit_clusters[hit_ids[times[i]-previous_hit[hit_ids] > cluster_gap]] += 1
        previous_hit[hit_ids] = times[i]
        last_hit[hit_ids] = times[i]
        # Any-time free-space vote: a return observed beyond the candidate proves
        # the space was empty then. Votes are not restricted to the candidate's own
        # observation window, so later or loop-closure views clear ghosts left by
        # moving objects (the "observed empty at any time" principle).
        free[candidates[np.isfinite(observed) & (observed-ranges > margin)]] += 1
        if progress is not None:
            progress.update(number+1, f'静的地図の可視性確認 {number+1}/{len(selected)}')
    # Objects that left: free-space votes that arrived strictly after the point's
    # final direct observation, while the direct observations formed one episode.
    free_after = np.zeros(len(xyz), dtype=np.uint16)
    after_all_hits = np.isfinite(last_hit)
    for i in selected:
        pose = poses[i]
        candidates = np.asarray(tree.query_ball_point(pose[:3, 3], args.dynamic_range), dtype=int)
        if not len(candidates):
            continue
        table = scan_table(i)
        local = (xyz[candidates]-pose[:3, 3]) @ pose[:3, :3]
        keys, ranges = bins(local)
        observed = table[keys]
        margin = np.maximum(args.dynamic_margin, ranges*.005)
        later = candidates[after_all_hits[candidates] & (times[i] > last_hit[candidates])
                           & np.isfinite(observed) & (observed-ranges > margin)]
        free_after[later] += 1
    abandoned = (free_after >= args.dynamic_min_votes) & (hit_clusters <= cluster_max)
    remove = (free >= args.dynamic_min_votes) & ((free > free_hit_ratio*hits) | abandoned)
    return ~remove, {'method': 'all-time conservative range visibility with change-point abandonment (inspired, not full Dynablox/DUFOMap)',
                     'tested_keyframes': len(selected), 'removed_points': int(remove.sum()),
                     'retained_points': int((~remove).sum()), 'unknown_space': 'preserved',
                     'min_free_votes': args.dynamic_min_votes, 'free_hit_ratio': free_hit_ratio,
                     'abandoned_rule': 'free-space votes after last direct hit, single direct-observation episode',
                     'hit_cluster_gap_s': cluster_gap, 'hit_cluster_max': cluster_max,
                     'limitation': 'objects still observed as a surface late in the session, missing rays and occlusions cannot be reliably removed'}
