#!/usr/bin/env python3
"""Overlay real LiDAR depth edges on THETA images; report an uncommitted z/pitch candidate.

An image-edge score is not a calibrated extrinsic: texture, occlusion, lens errors
and camera latency can give false minima. Never silently apply the candidate.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from mapping_fusion import read_static_calibration, static_lookup
from offline_3d_mapping import reader, stamp


def depth_edges(msg):
    from sensor_msgs_py.point_cloud2 import read_points
    p = read_points(msg, skip_nans=False)
    xyz = np.column_stack([p[n].reshape(-1) for n in ('x', 'y', 'z')])
    distance = np.linalg.norm(xyz, axis=1).reshape(msg.height, msg.width)
    edges = np.zeros_like(distance, dtype=bool)
    # Organized VLP16: adjacent rows sample neighboring azimuths. Exclude NaN
    # boundaries; they are not reliable physical depth discontinuities.
    if msg.height > 1:
        jumps = (np.isfinite(distance[1:]) & np.isfinite(distance[:-1])
                 & (np.abs(distance[1:]-distance[:-1]) > .35))
        edges[1:] |= jumps & (distance[1:] < distance[:-1])
        edges[:-1] |= jumps & (distance[:-1] < distance[1:])
    else:
        raise ValueError('Alignment diagnosis currently requires organized real VLP16 scans')
    keep = edges.reshape(-1) & np.isfinite(xyz).all(axis=1)
    keep &= (np.linalg.norm(xyz, axis=1) > 1.) & (np.linalg.norm(xyz, axis=1) < 12.)
    return xyz[keep]


def run(args):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage, PointCloud2
    # Use the repository's projection implementation without starting any ROS nodes.
    import sys
    navigation = Path(__file__).resolve().parents[2]/'src/sirius/sirius_navigation'
    sys.path.insert(0, str(navigation))
    from sirius_navigation.theta_perspective import project_camera_frame_points
    snapshot = Path(args.bag)/'capture/theta_calibration.yaml'
    calibration_path = Path(args.calibration) if args.calibration else (
        snapshot if snapshot.exists() else navigation/'config/theta_calibration_real.yaml')
    calibration = yaml.safe_load(calibration_path.read_text())
    _, _, transforms, frames = read_static_calibration(args.bag, reader, 'sirius3/base_footprint', args.cloud_topic, '/imu')
    lidar_from_camera = static_lookup(transforms, frames[args.cloud_topic], args.camera_frame)
    if lidar_from_camera is None:
        raise ValueError('Recorded static LiDAR/THETA calibration is required')
    r = reader(args.bag, [args.cloud_topic, args.image_topic])
    targets, selected, recent = [0., 45., 120., 210., 330.], [], []
    start = None
    while r.has_next() and len(selected) < len(targets):
        topic, data, _ = r.read_next()
        if topic == args.cloud_topic:
            cloud = deserialize_message(data, PointCloud2)
            recent.append(cloud)
            recent = recent[-4:]
            continue
        image = deserialize_message(data, CompressedImage)
        t = stamp(image)
        if start is None:
            start = t
        if t-start < targets[len(selected)] or not recent:
            continue
        cloud = min(recent, key=lambda m: abs(stamp(m)-t))
        if abs(stamp(cloud)-t) > .1:
            continue
        bgr = cv2.imdecode(np.frombuffer(image.data, np.uint8), cv2.IMREAD_COLOR)
        points = depth_edges(cloud)
        if bgr is None or len(points) < 30:
            continue
        edge = cv2.Canny(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), 60, 140)
        distance = cv2.distanceTransform(255-edge, cv2.DIST_L2, 5)
        selected.append((t, bgr, points, distance, t-stamp(cloud)))
    if not selected:
        raise ValueError('No image/cloud pairs with enough physical depth edges')
    def projection(points, dz, pitch, shape):
        rotation = lidar_from_camera[:3, :3] @ Rotation.from_euler('y', pitch, degrees=True).as_matrix()
        position = lidar_from_camera[:3, 3] + [0., 0., dz]
        return project_camera_frame_points((points-position) @ rotation, calibration, shape)
    z_values, pitch_values = np.linspace(-.15, .15, 13), np.linspace(-3., 3., 13)
    scores = np.zeros((len(z_values), len(pitch_values)))
    per_frame = []
    for t, bgr, points, distance, dt in selected:
        costs = np.empty_like(scores)
        for zi, z in enumerate(z_values):
            for pi, pitch in enumerate(pitch_values):
                uv, valid = projection(points, z, pitch, bgr.shape[:2])
                xy = np.rint(uv[valid]).astype(int)
                values = distance[xy[:, 1], xy[:, 0]] if len(xy) else np.array([100.])
                # Clipped distances reduce influence of occlusion and texture-only edges.
                costs[zi, pi] = np.mean(np.minimum(values, 12.))
        scores += costs/len(selected)
        best = np.unravel_index(np.argmin(costs), costs.shape)
        per_frame.append({'stamp': t, 'image_minus_cloud_s': dt, 'edge_points': len(points),
                          'candidate_dz_m': float(z_values[best[0]]), 'candidate_pitch_deg': float(pitch_values[best[1]]),
                          'nominal_edge_distance_px': float(costs[6, 6]), 'candidate_edge_distance_px': float(costs[best])})
    best = np.unravel_index(np.argmin(scores), scores.shape)
    dz, pitch = z_values[best[0]], pitch_values[best[1]]
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    for i, (_, bgr, points, _, _) in enumerate(selected):
        panels = []
        for z, angle, label in [(0., 0., 'Recorded TF'), (dz, pitch, f'CANDIDATE dz={dz:+.3f}m pitch={pitch:+.1f}deg')]:
            panel = bgr.copy()
            uv, valid = projection(points, z, angle, bgr.shape[:2])
            for x, y in np.rint(uv[valid]).astype(int):
                cv2.circle(panel, (int(x), int(y)), 2, (0, 255, 255), -1)
            cv2.putText(panel, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 255), 2)
            panels.append(panel)
        cv2.imwrite(str(output/f'projection_{i+1}.jpg'), np.hstack(panels))
    report = {'status': 'diagnostic candidate only; calibration NOT changed',
              'calibration': str(calibration_path), 'calibration_is_bag_snapshot': calibration_path == snapshot,
              'nominal_camera_position_in_lidar_m': lidar_from_camera[:3, 3].tolist(),
              'candidate_dz_m': float(dz), 'candidate_pitch_deg': float(pitch),
              'nominal_edge_distance_px': float(scores[6, 6]), 'candidate_edge_distance_px': float(scores[best]),
              'at_search_boundary': bool(best[0] in (0, 12) or best[1] in (0, 12)),
              'frames': per_frame, 'score_grid': scores.tolist(),
              'limitations': ['texture edges need not be LiDAR geometry edges',
                             'camera latency and motion are not estimated in this diagnostic',
                             'z, pitch, lens calibration and time offset can be ambiguous; use static multi-distance calibration targets']}
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('score_grid', 'frames')}, indent=2))
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--output', required=True)
    parser.add_argument('--cloud-topic', default='/velodyne_points')
    parser.add_argument('--image-topic', default='/theta/dual_fisheye/image_raw/compressed')
    parser.add_argument('--camera-frame', default='sirius3/theta_link')
    parser.add_argument('--calibration', help='THETA YAML; prefer capture snapshot, otherwise repository real calibration')
    raise SystemExit(run(parser.parse_args()))
