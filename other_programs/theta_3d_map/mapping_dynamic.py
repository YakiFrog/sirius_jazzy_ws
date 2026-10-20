"""Official DUFOMap adapter: local scans + optimized LiDAR poses -> map mask.

RA-L 2024: https://arxiv.org/abs/2403.01449
Implementation: https://github.com/KTH-RPL/dufomap (dufomap==1.1.1).
The fine output cloud and RGB are never re-voxelized or relocated here.
"""
from importlib.metadata import version

import numpy as np


def require_dufomap():
    try:
        from dufomap import dufomap
    except ImportError as error:
        raise ValueError('DUFOMapがありません。bash/startup_bash/setup_3d_map_cleaner.sh を実行してください。'
                         ' 従来方式は --dynamic-method visibility で選択できます。') from error
    return dufomap


def refine_fragments(xyz, keep, frame_points, poses, times, args):
    """Conservative local heuristic, NOT a component of the DUFOMap paper.

    Extend dynamic seeds only into small disconnected, weakly revisited groups.
    Size alone is never evidence of motion. No world-height/ROI exclusion is used.
    """
    import open3d as o3d
    from scipy.spatial import cKDTree
    diagnostic = {'enabled': False, 'removed_points': 0, 'removed_indices': []}
    if getattr(args, 'no_dynamic_fragments', False) or times is None or not np.any(~keep) or not np.any(keep):
        return keep, diagnostic
    times = np.asarray(times, dtype=float)
    if times.shape != (len(poses),) or not np.isfinite(times).all() or np.any(np.diff(times) < 0):
        raise ValueError('Fragment filtering requires ordered frame timestamps')
    radius = getattr(args, 'dynamic_fragment_radius', .12)
    maximum = getattr(args, 'dynamic_fragment_max_points', 120)
    extent = getattr(args, 'dynamic_fragment_max_extent', .8)
    seed_radius = getattr(args, 'dynamic_fragment_seed_radius', .25)
    revisit_gap = getattr(args, 'dynamic_fragment_revisit_gap', 8.)
    match_radius = getattr(args, 'dynamic_fragment_match_radius', .08)
    ids = np.flatnonzero(keep)
    points = xyz[ids]
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    labels = np.asarray(cloud.cluster_dbscan(eps=radius, min_points=1))
    counts = np.bincount(labels)
    # Cheap pointwise rejection first; avoid retaining all neighbor graph edges.
    small = np.flatnonzero(counts[labels] <= maximum)
    distances = np.full(len(points), np.inf)
    distances[small] = cKDTree(xyz[~keep]).query(points[small])[0]
    groups = []
    order = np.argsort(labels, kind='stable')
    offsets = np.r_[0, np.cumsum(counts)]
    for label in np.unique(labels[small[distances[small] <= seed_radius]]):
        group = order[offsets[label]:offsets[label+1]]
        if (np.max(np.ptp(points[group], axis=0)) <= extent
                and np.mean(distances[group] <= seed_radius) >= .5):
            groups.append(group)
    candidates = np.concatenate(groups) if groups else np.empty(0, int)
    hits = np.zeros(len(candidates), int)
    first = np.full(len(candidates), np.inf)
    last = np.full(len(candidates), -np.inf)
    for i, pose in enumerate(poses) if len(candidates) else []:
        scan = np.asarray(frame_points(i))
        scan = scan[np.isfinite(scan).all(axis=1)]
        if not len(scan):
            continue
        local = (points[candidates]-pose[:3, 3]) @ pose[:3, :3]
        seen = np.isfinite(cKDTree(scan).query(local, distance_upper_bound=match_radius)[0])
        hits += seen
        first[seen] = np.minimum(first[seen], times[i])
        last[seen] = times[i]
    result = keep.copy()
    offset = protected = removed_groups = 0
    for group in groups:
        section = slice(offset, offset+len(group))
        offset += len(group)
        persistent = (hits[section] >= 3) & (last[section]-first[section] >= revisit_gap)
        # A repeatedly seen minority protects the whole group, e.g. pole/wire ends.
        if np.mean(persistent) >= .2 or np.mean(hits[section] > 0) < .5:
            protected += 1
            continue
        result[ids[group]] = False
        removed_groups += 1
    removed_ids = np.flatnonzero(keep & ~result)
    diagnostic.update(enabled=True, method='seed-near disconnected fragments with temporal static protection (local heuristic)',
                      component_radius_m=radius, max_component_points=maximum, max_extent_m=extent,
                      seed_radius_m=seed_radius, revisit_gap_s=revisit_gap, match_radius_m=match_radius,
                      candidate_components=len(groups), protected_components=protected,
                      removed_components=removed_groups, removed_points=len(removed_ids),
                      removed_indices=removed_ids.tolist(),
                      limitation='proximity and weak revisits are not proof of motion; isolated static parts can be misclassified')
    return result, diagnostic


def dufomap_filter(xyz, frame_points, poses, args, progress=None, times=None,
                   observations=None):
    """Build fully-observed free regions, then classify against ALL observations."""
    factory = require_dufomap()
    resolution = getattr(args, 'dynamic_resolution', .1)
    surface_margin = getattr(args, 'dynamic_surface_margin', .2)
    pose_margin = getattr(args, 'dynamic_pose_margin', 2)
    threads = getattr(args, 'dynamic_threads', 4)
    model = factory(resolution, surface_margin, pose_margin, num_threads=threads)
    evidence = observations if observations is not None else (
        (pose, frame_points(i)) for i, pose in enumerate(poses))
    tested_scans = 0
    for pose, scan in evidence:
        points = np.asarray(scan)
        ranges = np.linalg.norm(points, axis=1)
        valid = np.isfinite(points).all(axis=1) & (ranges > 0) & (ranges <= args.dynamic_range)
        if np.any(valid):
            model.run(points[valid], pose, cloud_transform=True)
        tested_scans += 1
        if progress is not None:
            progress.update(tested_scans, f'DUFOMap 自由空間の確認 {tested_scans}スキャン')
    if not tested_scans:
        raise ValueError('DUFOMap requires at least one evidence scan')
    # Author's offline protocol: propagate void flags, disable optional clustering.
    model.oncePropagateCluster(if_propagate=True, if_cluster=False)
    labels = (model.segment(xyz, np.eye(4), cloud_transform=False)
              if len(xyz) else np.empty(0, np.uint8))
    if labels.shape != (len(xyz),) or not np.isin(labels, [0, 1]).all():
        raise ValueError('DUFOMap returned invalid labels')
    base_keep = labels == 0
    keep, fragments = refine_fragments(xyz, base_keep, frame_points, poses, times, args)
    polish = {'enabled': False, 'removed_indices': []}
    if getattr(args, 'dynamic_polish', False):
        from mapping_cleanup import polish_map
        keep, polish = polish_map(xyz, keep, frame_points, poses, times, args, progress)
    return keep, {'method': 'official DUFOMap offline + optional local fragment refinement', 'package_version': version('dufomap'),
                  'tested_keyframes': len(poses), 'tested_scans': tested_scans,
                  'evidence_source': 'all available raw scans' if observations is not None else 'cached keyframes',
                  'removed_points': int((~keep).sum()),
                  'dufomap_removed_points': int((~base_keep).sum()), 'fragment_filter': fragments,
                  'polish_filter': polish,
                  'retained_points': int(keep.sum()), 'resolution_m': resolution,
                  'surface_margin_m': surface_margin, 'pose_margin_voxels': pose_margin,
                  'range_m': args.dynamic_range, 'threads': threads,
                  'unknown_space': 'preserved by DUFOMap; optional refinement uses weakly revisited seed-near islands', 'trajectory_modified': False,
                  'limitation': 'unobserved/occluded space is not evidence of motion; no pointwise ground truth supplied'}


def finishing_indices(diagnostic):
    """Indices for the orange overlay: fragment cleanup plus ray-based finishing."""
    return np.union1d(diagnostic.get('fragment_filter', {}).get('removed_indices', []),
                      diagnostic.get('polish_filter', {}).get('removed_indices', [])).astype(int)
