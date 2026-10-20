#!/usr/bin/env python3
"""Re-clean a saved map with official DUFOMap, keeping every retained XYZ/RGB byte."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from mapping_dynamic import dufomap_filter, require_dufomap, finishing_indices
from offline_3d_mapping import reduce_points, write_map


def raw_scan_observations(source, report, diagnostic):
    """Stream deskewed scans at corrected high-rate poses; never extrapolate."""
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from mapping_color import ColorTrajectory
    from mapping_fusion import read_static_calibration
    from offline_3d_mapping import reader, points, stamp, CloudDeskewer
    bag = Path(report['bag']).expanduser().resolve()
    raw_topic = report['options'].get('raw_cloud_topic', '/velodyne_points')
    _, imu, transforms, frames = read_static_calibration(
        bag, reader, report['wheel_frames'][1], raw_topic, report['imu_topic'])
    trajectory = ColorTrajectory(source, report, bag, transforms, imu)
    motion = CloudDeskewer(trajectory.seed_times, trajectory.seed,
                          trajectory.mount, trajectory.gap)
    args = SimpleNamespace(**report['options'])
    diagnostic.update(trajectory=trajectory.report, scans_read=0,
                      scans_used=0, scans_without_pose=0, empty_scans=0,
                      signed_point_time_scans=0,
                      deskew='original high-rate seed; optimized SE3 correction at each scan header')
    r = reader(bag, [raw_topic])
    expected_frame = frames[raw_topic].lstrip('/')
    while r.has_next():
        _, data, _ = r.read_next()
        msg = deserialize_message(data, PointCloud2)
        diagnostic['scans_read'] += 1
        if msg.header.frame_id.lstrip('/') != expected_frame:
            raise ValueError('Raw scan frame changed within bag')
        pose = trajectory.at(stamp(msg))
        if pose is None:
            diagnostic['scans_without_pose'] += 1
            continue
        if 'time' not in {field.name for field in msg.fields}:
            raise ValueError('全スキャン除去には秒単位の点時刻 time が必要です')
        diagnostic['signed_point_time_scans'] += 1
        xyz, _, _ = points(msg, args, motion)
        if not len(xyz):
            diagnostic['empty_scans'] += 1
            continue
        diagnostic['scans_used'] += 1
        yield pose, xyz
    diagnostic['deskew_points'] = motion.points
    diagnostic['deskew_dropped_points'] = motion.dropped


def load_poses(source, report):
    rows = np.atleast_1d(np.genfromtxt(source/'trajectory.csv', delimiter=',', names=True))
    mount = np.asarray(report['resolved_base_from_lidar'])
    poses = []
    for row in rows:
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_quat([row[n] for n in ('qx', 'qy', 'qz', 'qw')]).as_matrix()
        pose[:3, 3] = [row[n] for n in ('x', 'y', 'z')]
        poses.append(pose @ mount)  # CSV contains base poses; scans use LiDAR poses.
    return poses


def clean(source, output, cache, policy, no_fragments=False, all_scans=False, polish=False):
    import open3d as o3d
    if not polish:
        require_dufomap()
    source, output, cache = map(lambda p: Path(p).expanduser().resolve(), (source, output, cache))
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError(f'保存先は空の別ディレクトリにしてください: {output}')
    original = source/'map_3d_colored_unfiltered.ply'
    if not original.is_file():
        raise ValueError(f'除去前の色付き点群がありません: {original}')
    report = json.loads((source/'report.json').read_text())
    poses = load_poses(source, report)
    files = sorted(cache.glob('*.npz'))
    if len(files) != len(poses) or any(p.name != f'{i:06d}.npz' for i, p in enumerate(files)):
        raise ValueError(f'軌跡とキャッシュのフレーム数・番号が一致しません: {cache}')
    cloud = o3d.io.read_point_cloud(str(original))
    xyz = np.asarray(cloud.points).copy()
    rgb = np.round(np.asarray(cloud.colors)*255).astype(np.uint8)
    if not len(xyz) or rgb.shape != xyz.shape or not np.isfinite(xyz).all():
        raise ValueError('元の点群が空、非有限、またはRGBを含みません')
    with Path(policy).expanduser().open() as stream:
        defaults = yaml.safe_load(stream)['parameters']
    if no_fragments:
        defaults['no_dynamic_fragments'] = True
    # Verify an explicitly supplied older cache by reconstructing the SAME map.
    # Matching frame counts alone would silently accept the wrong bag/calibration.
    chunks, colors = [], []
    for path, pose in zip(files, poses):
        with np.load(path) as frame:
            chunks.append(frame['xyz'] @ pose[:3, :3].T + pose[:3, 3])
            colors.append(frame['rgb'])
    rebuilt, rebuilt_rgb = reduce_points(np.concatenate(chunks), np.concatenate(colors), report['options']['voxel'])
    if (rebuilt.shape != xyz.shape or not np.allclose(rebuilt, xyz, atol=2e-5, rtol=0)
            or not np.array_equal(np.clip(np.round(rebuilt_rgb), 0, 255).astype(np.uint8), rgb)):
        raise ValueError('キャッシュから復元した元地図が一致しません。同じ生成処理のframesを指定してください。')
    del chunks, colors, rebuilt, rebuilt_rgb
    previous_keep = None
    if all_scans or polish:
        # Preserve the selected map's colors, including v10 reprojection. The
        # unfiltered RGB is older and is used only for cache identity checking.
        with np.load(source/'dynamic_mask.npz') as mask:
            previous_keep = mask['keep'].copy()
        if previous_keep.dtype != bool or previous_keep.shape != (len(xyz),):
            raise ValueError('保存済み残像マスクが元点群と一致しません')
        selected = o3d.io.read_point_cloud(str(source/'map_3d_colored.ply'))
        if not np.array_equal(np.asarray(selected.points), xyz[previous_keep]):
            raise ValueError('選択地図のXYZと保存済み残像マスクが一致しません')
        rgb[previous_keep] = np.round(np.asarray(selected.colors)*255).astype(np.uint8)

    def frame_points(i):
        with np.load(files[i]) as frame:
            return frame['xyz'].copy()

    class Progress:
        def update(self, value, message):
            if value % (200 if all_scans else 20) == 0 or value == len(poses):
                print(message, flush=True)

    print(f'元地図・キャッシュ一致: {len(xyz):,}点 / {len(poses)}フレーム。軌跡を固定して残像除去します。', flush=True)
    times = np.atleast_1d(np.genfromtxt(source/'trajectory.csv', delimiter=',', names=True))['stamp']
    evidence_report = {}
    observations = raw_scan_observations(source, report, evidence_report) if all_scans else None
    if polish:
        from mapping_cleanup import polish_map
        keep, finishing = polish_map(xyz, previous_keep, frame_points, poses, times,
                                    SimpleNamespace(**defaults), Progress())
        report['dynamic_filter_before_polish'] = report['dynamic_filter']
        diagnostic = {**report['dynamic_filter'], 'polish_filter': finishing}
    else:
        keep, diagnostic = dufomap_filter(xyz, frame_points, poses, SimpleNamespace(**defaults),
                                          Progress(), times=times, observations=observations)
    if previous_keep is not None:
        keep &= previous_keep
        if all_scans:
            diagnostic['raw_scan_evidence'] = evidence_report
        diagnostic.update(
                          previous_removed_points=int((~previous_keep).sum()),
                          additional_removed_points=int((previous_keep & ~keep).sum()),
                          removed_points=int((~keep).sum()), retained_points=int(keep.sum()),
                          previous_removals_preserved=True)
    output.mkdir(parents=True, exist_ok=True)
    (output/'.mapping-diagnostic').touch()
    write_map(output/'map_3d.ply', xyz[keep])
    write_map(output/'map_3d_colored.ply', xyz[keep], rgb[keep])
    write_map(output/'dynamic_candidates.ply', xyz[~keep], rgb[~keep])
    fragment_ids = finishing_indices(diagnostic)
    if previous_keep is not None and (source/'dynamic_fragments.ply').is_file():
        old_fragments = o3d.io.read_point_cloud(str(source/'dynamic_fragments.ply'))
        old_xyz = np.asarray(old_fragments.points)
        from scipy.spatial import cKDTree
        distance, old_ids = cKDTree(xyz).query(old_xyz)
        if np.any(distance > 1.e-7) or np.any(keep[old_ids]):
            raise ValueError('既存の追加削除点が元点群・削除マスクと一致しません')
        fragment_ids = np.union1d(fragment_ids, old_ids)
    write_map(output/'dynamic_fragments.ply', xyz[fragment_ids], rgb[fragment_ids])
    for name in ('map_3d_unfiltered.ply', 'map_3d_colored_unfiltered.ply', 'trajectory.csv', 'trajectory_initial.csv',
                 'calibration.yaml', 'resolved_calibration.yaml',
                 'colorization_policy.yaml', 'theta_calibration_used.yaml', 'color_image_motion.npz'):
        if (source/name).is_file():
            shutil.copyfile(source/name, output/name)
    shutil.copyfile(policy, output/'cleaning_policy.yaml')
    np.savez_compressed(output/'dynamic_mask.npz', keep=keep)
    if previous_keep is not None and (source/'color_observations.npz').is_file():
        with np.load(source/'color_observations.npz') as observations:
            counts = observations['accepted_views']
        if counts.shape != (int(previous_keep.sum()),):
            raise ValueError('色観測数と選択地図の点数が一致しません')
        counts = counts[keep[previous_keep]]
        np.savez_compressed(output/'color_observations.npz', accepted_views=counts)
        if 'colorization' in report:
            report['colorization'].update(colored_points=int((counts>0).sum()),
                unknown_gray_points=int((counts==0).sum()), points_with_multiple_views=int((counts>=2).sum()),
                retained_colors_source=str(source), subset_after_cleaning=True)
    report['dynamic_filter'] = {'enabled': True, **diagnostic}
    report['points'] = int(keep.sum())
    report['cleaning_source'] = {'directory': str(source), 'cache': str(cache),
                                 'trajectory_sha256': hashlib.sha256((source/'trajectory.csv').read_bytes()).hexdigest(),
                                 'cleaner_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                                 'adapter_sha256': hashlib.sha256(Path(__file__).with_name('mapping_dynamic.py').read_bytes()).hexdigest(),
                                 'geometry_policy': 'exact subset; unchanged retained XYZ/RGB; no trajectory optimization',
                                 'cache_verified_by_map_reconstruction': True,
                                 'no_fragments_override': no_fragments,
                                 'all_scans': all_scans,
                                 'polish_only': polish,
                                 'cleanup_sha256': hashlib.sha256(Path(__file__).with_name('mapping_cleanup.py').read_bytes()).hexdigest() if polish or defaults.get('dynamic_polish') else None,
                                 'semantic_accuracy': 'not measured: no pointwise dynamic/static ground truth'}
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    (output/'.mapping-diagnostic').unlink()
    print(f'完了: {output}\n保持 {keep.sum():,}点 / 削除対象 {(~keep).sum():,}点\n'
          'view_3dmap で選び、/offline_map/removed_points（赤）を重ねて確認できます。', flush=True)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', nargs='?', help='生成済み地図ディレクトリ。省略時は番号選択')
    parser.add_argument('-o', '--output', help='保存先（既定: 元ディレクトリ_dufomap、既存は上書きしない）')
    parser.add_argument('--cache', help='deskew済みframesディレクトリ（既定: 元ディレクトリ/frames）')
    parser.add_argument('--no-fragments', action='store_true', help='追加の輪郭仕上げを省略してDUFOMap単独で比較')
    parser.add_argument('--all-scans', action='store_true', help='最適化補正を高頻度軌跡へ反映し、全生スキャンをdeskewして自由空間を確認。既存の削除・保持点RGBを維持')
    parser.add_argument('--polish', action='store_true', help='既存の削除を維持し、複数時刻のレーザー通過証拠で浮遊点・残像を追加仕上げ。位置・色は変更しない')
    parser.add_argument('--mapping-config', default=str(Path(__file__).resolve().parents[2]/'params/offline_3d_mapping.yaml'))
    args = parser.parse_args(argv)
    if args.polish and (args.all_scans or args.no_fragments):
        parser.error('--polish is additional finishing only; do not combine with --all-scans/--no-fragments')
    try:
        if args.source is None:
            from map_selection import choose_numbered, map_candidates
            value = choose_numbered(lambda: [p for p in map_candidates(Path.home()/'rosbag2_data')
                                            if (p.parent/'map_3d_colored_unfiltered.ply').exists()],
                                    '残像除去する生成済み3D地図', Path.home()/'rosbag2_data')
            if value is None:
                return 0
            args.source = str(value.parent)
        source = Path(args.source).expanduser().resolve()
        cache = args.cache or source/'frames'
        if args.cache is None and not Path(cache).is_dir() and (source/'report.json').is_file():
            # A previous clean-only output references its verified original cache.
            cache = json.loads((source/'report.json').read_text()).get('cleaning_source', {}).get('cache', cache)
        suffix = '_polished' if args.polish else '_all_scans' if args.all_scans else '_dufomap'
        return clean(source, args.output or str(source)+suffix, cache, args.mapping_config,
                     args.no_fragments, args.all_scans, args.polish)
    except (ValueError, OSError, ImportError, KeyError, EOFError) as error:
        print(f'ERROR: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
