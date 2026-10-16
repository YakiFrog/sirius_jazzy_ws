"""Revisit floor consistency diagnostics, independent of solver convergence.

This measures overlapping local observations; it neither asserts a flat world nor
uses the result as a constraint. It cannot measure absolute accuracy without truth.
"""
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree


def floor_overlap_quality(cache, poses, times, base_from_lidar, reference=None):
    import open3d as o3d
    o3d.utility.random.seed(42)
    observations = []
    previous = None
    reference_views = set(reference['view_indices']) if reference is not None else None
    reference_pairs = {(p['from'], p['to']) for p in reference['pairs']} if reference is not None else None
    for i, pose in enumerate(poses):
        # Avoid counting long stationary intervals as thousands of independent
        # successful revisits, which hides bad floor alignment along the route.
        if reference_views is not None and i not in reference_views:
            continue
        if reference_views is None and previous is not None and np.linalg.norm(pose[:3, 3]-previous[:3, 3]) < .4:
            continue
        previous = pose
        with np.load(Path(cache)/f'{i:06d}.npz') as frame:
            points = frame['xyz'].astype(float)
        base = points@base_from_lidar[:3, :3].T+base_from_lidar[:3, 3]
        points = points[(base[:, 2] > -.65) & (base[:, 2] < .35)
                        & (np.linalg.norm(base[:, :2], axis=1) < 5.)]
        if len(points) < 50:
            continue
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        equation, inliers = pc.segment_plane(.025, 3, 300)
        equation = np.asarray(equation)
        if len(inliers) < 40 or abs((base_from_lidar[:3, :3]@equation[:3])[2]) < .8:
            continue
        n = pose[:3, :3]@equation[:3]
        if n[2] < 0:
            n = -n
        world = points[inliers]@pose[:3, :3].T+pose[:3, 3]
        pc.points = o3d.utility.Vector3dVector(world)
        world = np.asarray(pc.voxel_down_sample(.10).points)
        observations.append((i, pose[:3, 3], n, world, cKDTree(world[:, :2])))
    pairs = []
    for k, (i, position, normal, world, _) in enumerate(observations):
        for j, other, other_normal, other_world, tree in observations[k+1:]:
            if reference_pairs is not None and (i, j) not in reference_pairs:
                continue
            if reference_pairs is None and (times[j]-times[i] < 40 or np.linalg.norm(position-other) > 1.5):
                continue
            distance, indices = tree.query(world[:, :2])
            valid = distance < .10
            if valid.sum() < 20:
                continue
            average = normal+other_normal
            if abs(average[2]) < .5:
                continue
            delta = other_world[indices[valid]]-world[valid]
            error = np.abs(delta@average/average[2])
            angle = np.degrees(np.arccos(np.clip(normal@other_normal, -1, 1)))
            pairs.append({'from': i, 'to': j, 'overlap_points': int(valid.sum()),
                          'median_height_difference_m': float(np.median(error)),
                          'p95_height_difference_m': float(np.percentile(error, 95)),
                          'normal_difference_deg': float(angle)})
    result = {'floor_views': len(observations), 'revisit_pairs': len(pairs),
              'view_indices': [o[0] for o in observations],
              'absolute_accuracy_measured': False, 'pairs': pairs,
              'method': 'spatially distinct revisits; overlapping local floor returns; slope-corrected vertical differences'}
    if pairs:
        differences = [p['p95_height_difference_m'] for p in pairs]
        result.update(median_pair_p95_height_difference_m=float(np.median(differences)),
                      p95_pair_p95_height_difference_m=float(np.percentile(differences, 95)),
                      max_pair_p95_height_difference_m=float(np.max(differences)),
                      p95_normal_difference_deg=float(np.percentile([p['normal_difference_deg'] for p in pairs], 95)))
    return result


def trajectory_jump_quality(poses, times, max_speed_mps=2., robust_sigma=15., min_robust_speed=.8):
    """Flag physically impossible or robustly abnormal speeds between consecutive keyframes.

    Detects optimizer spikes independently of solver convergence. Constant-speed motion
    and continuous slopes stay accepted; this is a review trigger, not a correctness proof.
    """
    positions = np.array([p[:3, 3] for p in poses], dtype=float)
    times = np.asarray(times, dtype=float)
    steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    dt = np.diff(times)
    valid = dt > 1e-3
    speeds = np.zeros(len(steps))
    speeds[valid] = steps[valid]/dt[valid]
    median = float(np.median(speeds[valid])) if valid.any() else 0.
    mad = float(np.median(np.abs(speeds[valid]-median))) if valid.any() else 0.
    # Legitimate speed changes (e.g. 1.2 -> 1.7 m/s) must not be flagged; the robust
    # bound therefore also scales with the median before the MAD term applies.
    robust_threshold = max(min_robust_speed, 2.*median, median + robust_sigma*1.4826*mad)
    jumps = []
    for k in np.flatnonzero(valid):
        reasons = []
        if speeds[k] > max_speed_mps:
            reasons.append('above_physical_cap')
        if speeds[k] > robust_threshold:
            reasons.append('robust_outlier')
        if reasons:
            jumps.append({'from': int(k), 'to': int(k+1), 'step_m': float(steps[k]),
                          'dt_s': float(dt[k]), 'speed_mps': float(speeds[k]), 'reasons': reasons})
    return {'steps_checked': int(valid.sum()), 'absolute_accuracy_measured': False,
            'median_speed_mps': median, 'robust_speed_threshold_mps': float(robust_threshold),
            'max_speed_mps': float(max_speed_mps), 'max_step_m': float(steps.max()) if len(steps) else 0.,
            'max_speed_observed_mps': float(speeds[valid].max()) if valid.any() else 0.,
            'jump_count': len(jumps), 'jumps': jumps[:50],
            'method': 'consecutive optimized keyframe speeds; physical cap plus robust median/MAD outlier check'}
