"""Recover high-rate camera motion without changing the optimized map poses."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

from mapping_fusion import load_lio
from offline_3d_mapping import (reader, inverse, transform, interpolate_pose,
                                load_motion, calibration, seed_planar_motion)


def read_trajectory(path, mount):
    rows = np.atleast_1d(np.genfromtxt(path, delimiter=',', names=True))
    poses = [transform(Rotation.from_quat([row[k] for k in ('qx','qy','qz','qw')]).as_matrix(),
                       [row[k] for k in ('x','y','z')]) @ mount for row in rows]
    return rows['stamp'], poses


def initial_trajectory_source(source):
    seen = set()
    while source not in seen:
        seen.add(source)
        if (source/'trajectory_initial.csv').is_file():
            return source
        report = json.loads((source/'report.json').read_text())
        previous = report.get('cleaning_source', {}).get('directory')
        if not previous:
            break
        source = Path(previous).expanduser().resolve()
    raise ValueError('元の trajectory_initial.csv がありません。高頻度軌跡への補正を復元できません。')


class ColorTrajectory:
    def __init__(self, source, report, bag, static_transforms, static_imu):
        self.mount = np.asarray(report['resolved_base_from_lidar'])
        self.times, optimized = read_trajectory(source/'trajectory.csv', self.mount)
        original = initial_trajectory_source(source)
        initial_times, initial = read_trajectory(original/'trajectory_initial.csv', self.mount)
        if not np.array_equal(initial_times, self.times):
            raise ValueError('最適化前後の軌跡時刻が一致しません')
        options = {**report['options'], 'bag': bag}
        args = SimpleNamespace(**options)
        self.gap = args.max_sync_gap
        mode = 'wheel/IMU'
        if report.get('lio', {}).get('initialization') == 'FastLIO':
            if static_imu is None:
                raise ValueError('LIO bodyを解決するIMUの固定TFがありません')
            self.seed_times, self.seed, _ = load_lio(args, reader, static_transforms,
                                                   report['wheel_frames'][1], inverse(static_imu))
            mode = 'FastLIO'
        else:
            wt, wp, it, ip, _ = load_motion(args)
            original_mount, imu_rot, _ = calibration(original/'calibration.yaml')
            self.seed_times = wt
            self.seed = (seed_planar_motion(wt, wp, it, ip, self.gap,
                         imu_rot @ original_mount[:3,:3].T) if args.wheel_model == 'planar' else wp)
        first = interpolate_pose(self.times[0], self.seed_times, self.seed, self.gap)
        if first is None:
            raise ValueError('元軌跡と高頻度オドメトリの時刻が重なりません')
        self.alignment = initial[0] @ inverse(first @ self.mount)
        residuals = []
        for t, pose in zip(self.times, initial):
            high = interpolate_pose(t, self.seed_times, self.seed, self.gap)
            if high is None:
                raise ValueError('元軌跡に対応するオドメトリが欠落しています')
            residuals.append(np.max(abs(self.alignment @ high @ self.mount-pose)))
        if max(residuals) > 1.e-4:
            raise ValueError(f'高頻度軌跡が生成時の初期軌跡と一致しません（残差{max(residuals):.4g}）。')
        self.corrections = [p @ inverse(q) for p,q in zip(optimized, initial)]
        self.report = {'motion_source': mode, 'high_rate_samples': len(self.seed_times),
                       'initial_trajectory_max_matrix_residual': max(residuals),
                       'optimized_keyframes': len(self.times), 'ground_truth_used': False,
                       'policy': 'interpolate optimized SE3 corrections on the original high-rate seed; no extrapolation'}

    def at(self, stamp):
        correction = interpolate_pose(stamp, self.times, self.corrections, float('inf'))
        seed = interpolate_pose(stamp, self.seed_times, self.seed, self.gap)
        if correction is None or seed is None:
            return None
        return correction @ self.alignment @ seed @ self.mount


def rotation_observation_weight(trajectory, stamp, scale_deg_s):
    """Downweight motion-sensitive images; do not adjust their pose or timestamp."""
    if scale_deg_s == 0:
        return 1., None
    before = max(float(trajectory.times[0]), stamp-.05)
    after = min(float(trajectory.times[-1]), stamp+.05)
    if after-before < 1.e-6:
        return 0., None
    first, last = trajectory.at(before), trajectory.at(after)
    if first is None or last is None:
        return 0., None
    speed = np.rad2deg(Rotation.from_matrix(first[:3,:3].T @ last[:3,:3]).magnitude())/(after-before)
    return float(1./(1.+(speed/scale_deg_s)**2)), float(speed)


class ColorFusion:
    """Keep several best reliable observations; use median RGB to resist spill."""
    def __init__(self, count, views, weighted_median=False):
        self.weighted_median = weighted_median
        self.scores = np.zeros((count, views), np.float32)
        self.colors = np.zeros((count, views, 3), np.uint8)

    def add(self, ids, rgb, confidence):
        valid = confidence > 0
        ids, rgb, confidence = ids[valid], rgb[valid], confidence[valid]
        slot = np.argmin(self.scores[ids], axis=1)
        better = confidence > self.scores[ids, slot]
        ids, rgb, confidence, slot = ids[better], rgb[better], confidence[better], slot[better]
        self.scores[ids, slot] = confidence
        self.colors[ids, slot] = np.column_stack([(rgb>>16)&255, (rgb>>8)&255, rgb&255])

    def finish(self):
        counts = (self.scores > 0).sum(axis=1)
        rgb = np.full((len(counts),3), 128, np.uint8)
        known = counts > 0
        data = self.colors[known].astype(float)
        if self.weighted_median:
            # Each channel keeps a robust median, with actual observation weight.
            # An exact half-weight tie averages its endpoints, matching the
            # ordinary median for equal weights (including an even view count).
            order = np.argsort(data, axis=1, kind='stable')
            values = np.take_along_axis(data, order, axis=1)
            weights = np.take_along_axis(
                np.broadcast_to(self.scores[known, :, None], data.shape), order, axis=1).astype(float)
            cumulative = np.cumsum(weights, axis=1)
            half = weights.sum(axis=1, keepdims=True)/2
            lower = np.argmax(cumulative >= half, axis=1)
            upper = np.argmax(cumulative > half, axis=1)
            low = np.take_along_axis(values, lower[:, None, :], axis=1)[:, 0, :]
            high = np.take_along_axis(values, upper[:, None, :], axis=1)[:, 0, :]
            rgb[known] = np.round((low+high)/2).astype(np.uint8)
            return rgb, counts
        data[self.scores[known] <= 0] = np.nan
        rgb[known] = np.round(np.nanmedian(data, axis=1)).astype(np.uint8)
        return rgb, counts
