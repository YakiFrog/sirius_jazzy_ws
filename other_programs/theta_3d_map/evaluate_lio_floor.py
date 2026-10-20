"""Compare unconstrained LIO output against a measured initial floor plane."""
from pathlib import Path
import json
import argparse
import numpy as np
import rosbag2_py
import yaml
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sirius_navigation.map_localizer_node import _pc2_to_xyz

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('bag', type=Path)
parser.add_argument('--replay', action='append', required=True, help='NAME=PATH')
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
bag = args.bag.expanduser().resolve()
out = bag/'analysis_mount_tilt'
initial = json.loads((out/'initial_floor_imu.json').read_text())
n_lidar = np.median([p['normal_lidar'] for p in initial['plane_samples']], axis=0)
n_lidar /= np.linalg.norm(n_lidar)
d_lidar = float(np.median([p['height_lidar_m'] for p in initial['plane_samples']]))
rng = np.random.default_rng(2026)


def fit_floor(points, expected_normal, expected_offset):
    radius = np.linalg.norm(points[:, :2], axis=1)
    projected = points @ expected_normal
    p = points[np.isfinite(points).all(1) & (radius > 2) & (radius < 7)
               & (np.abs(projected-expected_offset) < .30)]
    if len(p) < 50:
        return None
    triplets = p[rng.integers(0, len(p), size=(1000, 3))]
    normals = np.cross(triplets[:, 1]-triplets[:, 0], triplets[:, 2]-triplets[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    good = lengths > 1e-8
    normals, origins = normals[good]/lengths[good, None], triplets[good, 0]
    normals *= np.where(normals @ expected_normal < 0, -1, 1)[:, None]
    good = normals @ expected_normal > .97
    normals, origins = normals[good], origins[good]
    if len(normals) == 0:
        return None
    offsets = np.einsum('ij,ij->i', normals, origins)
    residuals = np.abs(p @ normals.T-offsets)
    counts = (residuals < .025).sum(0)
    q = p[residuals[:, counts.argmax()] < .025]
    if len(q) < 50:
        return None
    center = q.mean(0)
    _, vectors = np.linalg.eigh((q-center).T @ (q-center))
    normal = vectors[:, 0]
    normal *= 1 if normal @ expected_normal > 0 else -1
    return center, normal, len(q)/len(p)


results = {}
for spec in args.replay:
    name, path = spec.split('=', 1)
    source = Path(path).expanduser().resolve()
    rng = np.random.default_rng(2026)
    if not (source/'metadata.yaml').exists():
        continue
    if name == 'recorded':
        params = yaml.safe_load((bag/'capture/fastlio.yaml').read_text())['/laser_mapping']['ros__parameters']
    else:
        params = yaml.safe_load((source/'capture/calibration.yaml').read_text())['/**']['ros__parameters']
    r_il = np.array(params['mapping']['extrinsic_R']).reshape(3, 3)
    t_il = np.array(params['mapping']['extrinsic_T'])
    expected = r_il @ n_lidar
    expected_d = d_lidar + expected @ t_il
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(source), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/Odometry', '/cloud_registered_body']))
    poses, clouds = [], []
    last_cloud = -10**30
    while reader.has_next():
        topic, data, ns = reader.read_next()
        if topic == '/Odometry':
            m = deserialize_message(data, Odometry)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            stamp = m.header.stamp.sec*10**9+m.header.stamp.nanosec
            poses.append([stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        elif ns-last_cloud >= 5e9:
            m = deserialize_message(data, PointCloud2)
            stamp = m.header.stamp.sec*10**9+m.header.stamp.nanosec
            clouds.append((stamp, _pc2_to_xyz(m)))
            last_cloud = ns
    poses = np.array(poses)
    poses = poses[np.argsort(poses[:, 0])]
    rotations = Rotation.from_quat(poses[:, 4:8]).as_matrix()
    rows = []
    reference_normal, reference_offset = None, None
    for stamp, points in clouds:
        fit = fit_floor(points, expected, expected_d)
        if fit is None:
            continue
        i = int(np.argmin(np.abs(poses[:, 0]-stamp)))
        if abs(poses[i, 0]-stamp) > 5e7:
            continue
        center, normal, fraction = fit
        center_world = rotations[i] @ center + poses[i, 1:4]
        normal_world = rotations[i] @ normal
        if reference_normal is None:
            reference_normal = normal_world
            reference_offset = normal_world @ center_world
        rows.append({'t_s': float((stamp-poses[0, 0])/1e9),
                     'floor_height_relative_initial_m': float(reference_normal @ center_world-reference_offset),
                     'floor_normal_change_deg': float(np.degrees(np.arccos(np.clip(reference_normal @ normal_world, -1, 1)))),
                     'floor_inlier_fraction': float(fraction)})
    heights = np.array([r['floor_height_relative_initial_m'] for r in rows])
    tilts = np.array([r['floor_normal_change_deg'] for r in rows])
    holdout = np.array([r['t_s'] >= 150 for r in rows])
    xyz = poses[:, 1:4]
    result = {'poses': len(poses), 'floor_samples': len(rows), 'rows': rows,
              'floor_height_abs_p95_holdout_m': float(np.percentile(np.abs(heights[holdout]), 95)),
              'floor_normal_change_p95_holdout_deg': float(np.percentile(tilts[holdout], 95)),
              'world_z_span_m': float(np.ptp(xyz[:, 2])),
              'floor_height_span_m': float(np.ptp(heights)),
              'floor_height_abs_p95_m': float(np.percentile(np.abs(heights), 95)),
              'floor_normal_change_p95_deg': float(np.percentile(tilts, 95)),
              'floor_fraction_median': float(np.median([r['floor_inlier_fraction'] for r in rows]))}
    results[name] = result
    print(name, json.dumps({k:v for k,v in result.items() if k not in ('rows','trajectory')}), flush=True)

args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps({
    'metric': 'Measured local floor patches transformed by LIO; initial patch defines the comparison plane. Flat-route assumption; not external ground truth. No floor constraints applied to LIO.',
    'results': results}, indent=2))
