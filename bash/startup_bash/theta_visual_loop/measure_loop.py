"""Estimate and time-align one planar visual revisit edge from the recorded bag."""
import argparse
import json
from pathlib import Path

import cv2
from types import SimpleNamespace
import rosbag2_py
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage, LaserScan
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from tf2_msgs.msg import TFMessage
import yaml

from sirius_navigation.theta_bev_projection import build_blend_maps

SIZE = 720


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def transform(x, y, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0., 0., 1.]])


def components(matrix):
    return np.r_[matrix[:2, 2], np.arctan2(matrix[1, 0], matrix[0, 0])]


class Bag:
    def __init__(self, path):
        self.path = path

    def iter_messages(self, topics, start_time=None, end_time=None):
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(self.path), storage_id=''),
                    rosbag2_py.ConverterOptions('', ''))
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
        if start_time is not None:
            reader.seek(start_time)
        while reader.has_next():
            topic, data, log_time = reader.read_next()
            if end_time is not None and log_time >= end_time:
                break
            yield None, SimpleNamespace(topic=topic), SimpleNamespace(data=data, log_time=log_time)


def read_inputs(bag, first_stamp, last_stamp, args):
    frames, odom, laser_static, camera_static = {}, [], [], []
    reader = Bag(bag)
    first_offsets = np.r_[2.5, 5.5, 8.5, 13.5, 18.5, 22.5]
    last_offsets = np.r_[-4., -2., 0., 1., 3., 6., 11., 14.]
    for _, _, record in reader.iter_messages(topics=['/tf_static']):
        for tf in deserialize_message(record.data, TFMessage).transforms:
            if tf.child_frame_id == args.camera_frame:
                t, q = tf.transform.translation, tf.transform.rotation
                camera_static.append({'parent': tf.header.frame_id, 'child': tf.child_frame_id,
                                      'xyz': [t.x, t.y, t.z], 'quat': [q.x, q.y, q.z, q.w]})
    for group, center, offsets in [('start', first_stamp, first_offsets),
                                    ('end', last_stamp, last_offsets)]:
        lo, hi = center - 5., center + max(offsets) + 2.
        for _, _, record in reader.iter_messages(
                topics=[args.odom_topic], start_time=int(lo*1e9), end_time=int(hi*1e9)):
            msg = deserialize_message(record.data, Odometry)
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            odom.append([stamp(msg.header), p.x, p.y, yaw])
        for i, offset in enumerate(offsets):
            target = int((center + offset)*1e9)
            records = reader.iter_messages(topics=[args.camera_topic], start_time=target-500_000_000,
                                           end_time=target+500_000_000)
            nearest = min(records, default=None, key=lambda item: abs(item[2].log_time-target))
            if nearest is None:
                continue
            msg = deserialize_message(nearest[2].data, CompressedImage)
            image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if image is not None and msg.header.stamp.sec > 0:
                frames[group, i] = stamp(msg.header), image
    if len(odom) < 2:
        raise ValueError(f'No odometry around graph endpoints on {args.odom_topic}')
    odom = np.asarray(sorted(odom))
    odom = odom[np.r_[True, np.diff(odom[:, 0]) > 0]]
    odom[:, 3] = np.unwrap(odom[:, 3])
    return frames, odom, first_stamp, camera_static, laser_static


def odom_pose(odom, time):
    index = np.searchsorted(odom[:, 0], time)
    nearest = min(abs(odom[j, 0]-time) for j in (index-1, index) if 0 <= j < len(odom))
    if nearest > .1:
        raise ValueError(f'No odom samples within 100 ms at {time}')
    return transform(*(np.interp(time, odom[:, 0], odom[:, col]) for col in [1, 2, 3]))


def make_features(frames, calibration, mask_bottom_right=False):
    size = calibration['bev_size'] = SIZE
    extent = calibration['bev_extent_m']
    mask = np.zeros((size, size), np.uint8)
    cv2.circle(mask, (size//2, size//2), round(size*4.4/extent), 255, -1)
    cv2.circle(mask, (size//2, size//2), round(size*1.45/extent), 0, -1)
    if mask_bottom_right:
        mask[round(size*.65):, :] = 0
        mask[round(size*.28):, round(size*.57):] = 0
    sift = cv2.SIFT_create(nfeatures=900)
    clahe = cv2.createCLAHE(clipLimit=2., tileGridSize=(8, 8))
    maps, result = {}, {}
    for key, (time, image) in frames.items():
        shape = image.shape[:2]
        if shape not in maps:
            maps[shape] = build_blend_maps(calibration, shape[1], shape[0])
        ax, ay, bx, by, alpha = maps[shape]
        front = cv2.remap(image, ax, ay, cv2.INTER_LINEAR)
        back = cv2.remap(image, bx, by, cv2.INTER_LINEAR)
        bev = np.round(front*alpha[..., None] + back*(1-alpha[..., None])).astype(np.uint8)
        kp, desc = sift.detectAndCompute(clahe.apply(cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)), mask)
        if desc is not None:
            result[key] = time, bev, kp, desc
    return result


def metric(kp, extent):
    # BEV pixel u points right, v points backward. Robot axes are x forward, y left.
    return (0.5 - (np.asarray([kp.pt[1], kp.pt[0]]) + .5)/SIZE)*extent


def rigid(source, target):
    a, b = source.mean(axis=0), target.mean(axis=0)
    u, _, v = np.linalg.svd((source-a).T @ (target-b))
    r = v.T @ np.diag([1., np.linalg.det(v.T @ u.T)]) @ u.T
    delta = b-r@a
    return transform(*delta, np.arctan2(r[1, 0], r[0, 0]))


def initial_bridge(bag, start, end, scan_topic):
    """Align the first scan to the first wheel odometry time (wheel starts late)."""
    def points_at(reader, time):
        candidates = []
        for _, _, record in reader.iter_messages(
                topics=[scan_topic], start_time=int((time-.35)*1e9), end_time=int((time+.45)*1e9)):
            scan = deserialize_message(record.data, LaserScan)
            candidates.append((abs(stamp(scan.header)-time), scan))
        error, scan = min(candidates, key=lambda item: item[0])
        if error > .06:
            raise ValueError(f'Initial scan timing mismatch: {error}')
        r = np.asarray(scan.ranges)
        angles = scan.angle_min + np.arange(len(r))*scan.angle_increment
        valid = np.isfinite(r) & (r >= 1.0) & (r <= 20.)
        return np.column_stack((r[valid]*np.cos(angles[valid]), r[valid]*np.sin(angles[valid])))

    reader = Bag(bag)
    reference, moving = points_at(reader, start), points_at(reader, end)
    tree = cKDTree(reference)
    fitted = np.eye(3)
    for _ in range(10):
        distances, indices = tree.query(moving @ fitted[:2, :2].T + fitted[:2, 2])
        close = distances < .18
        fitted = rigid(moving[close], reference[indices[close]])
    measured = components(fitted)
    if close.mean() < .6 or np.linalg.norm(measured[:2]) > .12 or abs(measured[2]) > np.deg2rad(2):
        raise ValueError(f'Uncertain initial bridge: {measured}, overlap {close.mean():.2f}')
    return fitted, {'delay_s': end-start, 'relative_pose': measured.tolist(),
                    'overlap_fraction': float(close.mean()), 'median_residual_m': float(np.median(distances[close]))}


def match_pair(start, end, matcher, extent):
    _, _, ak, ad = start
    _, _, bk, bd = end

    def good(descriptors, targets):
        return {m.queryIdx: m for pair in matcher.knnMatch(descriptors, targets, k=2)
                if len(pair) == 2 for m, n in [pair] if m.distance < .72*n.distance}

    backwards = good(bd, ad)
    matches = sorted((m for j, m in good(ad, bd).items()
                      if m.trainIdx in backwards and backwards[m.trainIdx].trainIdx == j),
                     key=lambda m: m.distance)
    cells_a, cells_b, unique = set(), set(), []
    for m in matches:
        qa = tuple(np.floor(metric(ak[m.queryIdx], extent)/.14))
        qb = tuple(np.floor(metric(bk[m.trainIdx], extent)/.14))
        if qa not in cells_a and qb not in cells_b:
            unique.append(m)
            cells_a.add(qa)
            cells_b.add(qb)
    if len(unique) < 8:
        return None
    a = np.asarray([metric(ak[m.queryIdx], extent) for m in unique])
    b = np.asarray([metric(bk[m.trainIdx], extent) for m in unique])
    _, candidate = cv2.estimateAffinePartial2D(b, a, ransacReprojThreshold=.08, confidence=.999)
    if candidate is None:
        return None
    inlier = candidate.ravel().astype(bool)
    if inlier.sum() < 8:
        return None
    for _ in range(2):
        fitted = rigid(b[inlier], a[inlier])
        error = np.linalg.norm(b @ fitted[:2, :2].T + fitted[:2, 2] - a, axis=1)
        inlier = error < .105
        if inlier.sum() < 8:
            return None
    spread = np.linalg.svd(a[inlier]-a[inlier].mean(axis=0), compute_uv=False) / np.sqrt(inlier.sum())
    if spread[1] < .22 or inlier.mean() < .55:
        return None
    return fitted, unique, inlier, spread, float(np.median(error[inlier]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    parser.add_argument('graph_poses', type=Path)
    parser.add_argument('calibration', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--camera-topic', default='/theta/dual_fisheye/image_raw/compressed')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--scan-topic', default='/scan3')
    parser.add_argument('--robot-frame', default='sirius3/base_footprint')
    parser.add_argument('--camera-frame', default='sirius3/theta_link')
    parser.add_argument('--mask-bottom-right', action='store_true',
                        help='Exclude a known crowd/banner in the BEV bottom and right')

    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(2)
    nodes = np.genfromtxt(args.graph_poses, delimiter=',', names=True)
    first, last = nodes[0], nodes[-1]
    frames, odom, bag_start, static_tf, laser_tf = read_inputs(args.bag, first['stamp'], last['stamp'], args)
    calibration = yaml.safe_load(args.calibration.read_text())
    pose_source = 'calibration YAML'
    candidates = [tf for tf in static_tf if tf['parent'] == args.robot_frame]
    if candidates:
        selected = candidates[-1]
        calibration['camera_position'] = selected['xyz']
        rpy = Rotation.from_quat(selected['quat']).as_euler('xyz', degrees=True)
        calibration['rpy_degrees'] = (rpy * [1., -1., 1.]).tolist()
        pose_source = 'bag /tf_static'
    features = make_features(frames, calibration, args.mask_bottom_right)
    matcher = cv2.BFMatcher()
    bridge = None
    if first['stamp'] < odom[0, 0] - .1:
        first_to_odom, bridge = initial_bridge(args.bag, first['stamp'], odom[0, 0], args.scan_topic)
        first_odom = odom_pose(odom, odom[0, 0]) @ np.linalg.inv(first_to_odom)
    else:
        first_odom = odom_pose(odom, first['stamp'])
    last_odom = odom_pose(odom, last['stamp'])
    rows, previews = [], []
    for ak, start in features.items():
        if ak[0] != 'start':
            continue
        for bk, end in features.items():
            if bk[0] != 'end':
                continue
            match = match_pair(start, end, matcher, calibration['bev_extent_m'])
            if match is None:
                continue
            measured, matched, valid, spread, error = match
            # Graph nodes are at scan timestamps. Compose the two short wheel-odom motions.
            aligned = np.linalg.inv(first_odom) @ odom_pose(odom, start[0]) @ measured @ np.linalg.inv(odom_pose(odom, end[0])) @ last_odom
            row = {'start_stamp': start[0], 'end_stamp': end[0],
                   'start_rel_s': start[0]-bag_start, 'end_rel_s': end[0]-bag_start,
                   'n_matches': len(matched), 'n_inliers': int(valid.sum()),
                   'spread_m': spread.tolist(), 'median_residual_m': error,
                   'raw_edge': components(measured).tolist(), 'aligned_edge': components(aligned).tolist(),
                   'group': 'fit' if bk[1] <= 4 and ak[1] <= 3 else 'held_out'}
            rows.append(row)
            previews.append((row, start, end, matched, valid))
    fit = np.array([r['aligned_edge'] for r in rows if r['group'] == 'fit'])
    if len(fit) < 3:
        raise RuntimeError(f'Only {len(fit)} valid fitting pairs')
    edge = np.median(fit, axis=0)
    held = np.array([r['aligned_edge'] for r in rows if r['group'] == 'held_out'])
    if len(held) == 0:
        raise RuntimeError('No held-out matches')
    held_edge = np.median(held, axis=0)
    if (np.linalg.norm(edge[:2]-held_edge[:2]) > .45
            or abs(np.angle(np.exp(1j*(edge[2]-held_edge[2])))) > np.deg2rad(7)):
        raise ValueError('Visual loop geometry disagrees between image pairs')
    if np.linalg.norm(edge[:2]) > 2.0:
        raise ValueError('The requested start/end pair does not have a near-coincident revisit')
    row, a, b, matches, valid = max((r for r in previews if r[0]['group'] == 'fit'), key=lambda r: r[0]['n_inliers'])
    photo = cv2.drawMatches(a[1], a[2], b[1], b[2], [m for m, ok in zip(matches, valid) if ok], None,
                            matchColor=(30, 220, 30), flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.imwrite(str(args.output/'visual_matches.jpg'), cv2.resize(photo, (1200, 600)))
    report = {'bag': str(args.bag), 'camera_pose_source': pose_source,
              'camera_position': calibration['camera_position'], 'camera_rpy_degrees': calibration['rpy_degrees'],
              'laser_static_transforms': laser_tf, 'initial_bridge': bridge, 'frames': len(features),
              'start_node': int(first['id']), 'end_node': int(last['id']),
              'node_stamps': [float(first['stamp']), float(last['stamp'])],
              'edge': edge.tolist(), 'fit_pairs': len(fit), 'held_out_pairs': len(held),
              'fit_spread': np.std(fit, axis=0).tolist(), 'held_out_median': np.median(held, axis=0).tolist(),
              'held_out_spread': np.std(held, axis=0).tolist(),
              'representative': row, 'pairs': rows}
    (args.output/'visual_measurement.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k:v for k,v in report.items() if k != 'pairs'}, indent=2))


if __name__ == '__main__':
    main()
