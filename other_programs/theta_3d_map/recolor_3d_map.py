#!/usr/bin/env python3
"""Reproject THETA images onto a fixed cleaned map using robot calibration."""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import shutil
import sys

import cv2
import numpy as np
import yaml
from scipy.spatial import cKDTree

from mapping_color import ColorTrajectory, ColorFusion, rotation_observation_weight
from mapping_fusion import read_static_calibration, static_lookup
from offline_3d_mapping import reader, stamp, write_map

NAVIGATION = Path(__file__).resolve().parents[2]/'src/sirius/sirius_navigation'
sys.path.insert(0, str(NAVIGATION))
from sirius_navigation.theta_color_projection import load_color_policy, camera_in_lidar, visible_colors
from sirius_navigation.theta_ply_publisher import read_ply


def recolor(source, output, bag=None, policy_path=None, calibration_path=None, max_images=0,
            observation_callback=None):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage, PointCloud2
    from sensor_msgs_py.point_cloud2 import read_points
    cv2.setNumThreads(2)
    source, output = Path(source).expanduser().resolve(), Path(output).expanduser().resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError(f'保存先は空の別ディレクトリにしてください: {output}')
    report = json.loads((source/'report.json').read_text())
    bag = Path(bag or report['bag']).expanduser().resolve()
    policy_path = Path(policy_path or NAVIGATION/'config/theta_colorization.yaml').expanduser().resolve()
    policy = load_color_policy(policy_path)
    capture = json.loads((bag/'capture/recording.json').read_text()) if (bag/'capture/recording.json').exists() else {}
    default_intrinsics = NAVIGATION/'config'/('theta_calibration.yaml' if capture.get('mode') == 'sim' else 'theta_calibration_real.yaml')
    snapshot = bag/'capture/theta_calibration.yaml'
    calibration_path = Path(calibration_path or (snapshot if snapshot.exists() else default_intrinsics)).expanduser().resolve()
    calibration = yaml.safe_load(calibration_path.read_text())
    raw_topic = report['options'].get('raw_cloud_topic', '/velodyne_points')
    image_topic = '/theta/dual_fisheye/image_raw/compressed'
    _, imu, transforms, frames = read_static_calibration(bag, reader, report['wheel_frames'][1], raw_topic, report['imu_topic'])
    mount = camera_in_lidar(policy)
    mount_source = 'robot calibration override'
    if mount is not None and frames[raw_topic] != policy['lidar_frame']:
        raise ValueError('ロボット色付け校正とLiDAR frameが一致しません')
    if mount is None:
        mount = static_lookup(transforms, frames[raw_topic], policy['camera_frame'])
        mount_source = 'recorded fixed TF'
    if mount is None:
        raise ValueError('LiDARとTHETA間の固定TF/ロボット校正がありません')
    print('色付け 1/3: 固定校正と高頻度の撮影位置を検証中', flush=True)
    trajectory = ColorTrajectory(source, report, bag, transforms, imu)
    xyz, _ = read_ply(source/'map_3d_colored.ply')
    import open3d as o3d
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz)
    cloud.estimate_covariances(o3d.geometry.KDTreeSearchParamHybrid(radius=.2,max_nn=30))
    eigenvalues, eigenvectors = np.linalg.eigh(np.asarray(cloud.covariances))
    reliable = ((eigenvalues[:,0] < .15*eigenvalues[:,1])
                & (eigenvalues[:,1] > .2*eigenvalues[:,2]))
    normals = eigenvectors[:,:,0].copy()
    normals[~reliable] = 0.
    tree, fusion = cKDTree(xyz), ColorFusion(len(xyz), policy['fusion_views'],
                                           weighted_median=policy['fusion_weighted_median'])
    r = reader(bag, [raw_topic, image_topic])
    total_images = sum(t.message_count for t in r.get_metadata().topics_with_message_count if t.topic_metadata.name == image_topic)
    if not total_images:
        raise ValueError('THETAの元画像がありません。既存RGBだけでは再投影できません。')
    recent = deque(maxlen=4)
    read_images = used = skipped_pose = rejected_sync = occluded = accepted = edge_rejected = 0
    last_used = -np.inf
    plane_recovered = 0
    motion_rows = []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == raw_topic:
            recent.append(deserialize_message(data, PointCloud2))
            continue
        image_msg = deserialize_message(data, CompressedImage)
        read_images += 1
        t = stamp(image_msg)+policy['image_time_offset_s']
        if t-last_used < policy['offline_image_interval_s']:
            continue
        if image_msg.header.frame_id.lstrip('/') != policy['camera_frame'].lstrip('/'):
            raise ValueError('THETA画像のframeとロボット校正が一致しません')
        lidar_pose = trajectory.at(t)
        if lidar_pose is None:
            skipped_pose += 1
            continue
        if not recent:
            rejected_sync += 1
            continue
        raw = min(recent, key=lambda p: abs(stamp(p)-t))
        raw_pose = trajectory.at(stamp(raw))
        if raw_pose is None or abs(stamp(raw)-t) > policy['max_image_age_s']:
            rejected_sync += 1
            continue
        image = cv2.imdecode(np.frombuffer(image_msg.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            continue
        camera = lidar_pose @ mount
        ids = np.asarray(tree.query_ball_point(camera[:3,3], policy['max_color_range_m']), dtype=int)
        local = (xyz[ids]-camera[:3,3]) @ camera[:3,:3]
        raw_points = read_points(raw, field_names=['x','y','z'], skip_nans=True).reshape(-1)
        occluders = np.column_stack([raw_points[n] for n in ('x','y','z')])
        occluders = (occluders @ raw_pose[:3,:3].T + raw_pose[:3,3]-camera[:3,3]) @ camera[:3,:3]
        # Raw dynamic surfaces still occlude images even though absent from the clean map.
        colors, confidence, stats = visible_colors(local, image, calibration, policy, occluders,
                                                  normals_cam=normals[ids] @ camera[:3,:3])
        motion_weight, angular_speed = rotation_observation_weight(
            trajectory, t, policy['color_rotation_scale_deg_s'])
        confidence *= motion_weight
        motion_rows.append((t, np.nan if angular_speed is None else angular_speed, motion_weight))
        fusion.add(ids, colors, confidence)
        if observation_callback is not None:
            observation_callback(t, image, camera, ids, colors, confidence)
        used += 1
        last_used = t
        occluded += stats['occluded']; accepted += stats['accepted']; edge_rejected += stats['image_edge_rejected']
        plane_recovered += stats['coplanar_recovered']
        if used % 20 == 0:
            print(f'色付け 2/3: {read_images/total_images:5.1%} / 採用画像{used}枚 / 遮蔽サンプル除外{occluded:,}', flush=True)
        if max_images and used >= max_images:
            break
    if not used:
        raise ValueError('校正・時刻・軌跡が対応する画像がありません')
    print('色付け 3/3: 複数画像の中央値で融合・保存中', flush=True)
    colors, counts = fusion.finish()
    output.mkdir(parents=True, exist_ok=True)
    for path in source.iterdir():
        if path.is_file() and path.name not in {'map_3d_colored.ply', 'report.json', 'VALIDATION.md'} and path.suffix in {'.ply','.csv','.yaml','.npz'}:
            shutil.copyfile(path, output/path.name)
    write_map(output/'map_3d_colored.ply', xyz, colors)
    np.savez_compressed(output/'color_observations.npz', accepted_views=counts)
    motion_rows = np.asarray(motion_rows)
    np.savez_compressed(output/'color_image_motion.npz', stamps=motion_rows[:,0],
                        angular_speed_deg_s=motion_rows[:,1], confidence_weight=motion_rows[:,2])
    shutil.copyfile(policy_path, output/'colorization_policy.yaml')
    shutil.copyfile(calibration_path, output/'theta_calibration_used.yaml')
    diagnostic = {'method': 'fixed robot extrinsic + camera depth/edge visibility + high-rate motion + multi-view median',
                  'color_distance_power': policy['color_distance_power'],
                  'fusion_weighted_median': policy['fusion_weighted_median'],
                  'rotation_confidence': {'scale_deg_s': policy['color_rotation_scale_deg_s'],
                      'model': '1/(1+(angular_speed_deg_s/scale_deg_s)^2); zero scale disables',
                      'images_below_half_weight': int((motion_rows[:,2]<.5).sum()),
                      'images_without_motion_evidence': int((motion_rows[:,2]==0).sum()),
                      'pose_or_timestamp_modified': False},
                  'source': str(source), 'bag': str(bag), 'policy': str(policy_path),
                  'intrinsics': str(calibration_path), 'intrinsics_from_capture_snapshot': calibration_path == snapshot,
                  'camera_in_lidar': mount.tolist(), 'extrinsic_source': mount_source, 'trajectory': trajectory.report,
                  'images_read': read_images, 'images_used': used, 'images_without_pose': skipped_pose,
                  'images_without_synchronized_raw_scan': rejected_sync, 'occluded_samples': occluded,
                  'edge_rejected_samples': edge_rejected, 'accepted_samples': accepted,
                  'coplanar_samples_recovered': plane_recovered,
                  'colored_points': int((counts>0).sum()), 'unknown_gray_points': int((counts==0).sum()),
                  'points_with_multiple_views': int((counts>=2).sum()), 'geometry_modified': False,
                  'partial': bool(max_images),
                  'implementation_sha256': {name: hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()
                                           for name in ('recolor_3d_map.py','mapping_color.py')},
                  'projection_sha256': hashlib.sha256((NAVIGATION/'sirius_navigation/theta_color_projection.py').read_bytes()).hexdigest(),
                  'limitations': ['recorded TF is a nominal mount, not proof of measured calibration',
                                 'USB/HDMI exposure latency is not inferred; configured time offset defaults to zero',
                                 'raw occlusion scans use header poses (up to one scan of residual motion)',
                                 'no pixel/point color ground truth; unknown/occluded colors remain gray']}
    report['colorization'] = diagnostic
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    if max_images:
        (output/'.mapping-diagnostic').touch()
    print(f'色付け完了: {output}\n形状 {len(xyz):,}点を保持 / 色付き {diagnostic["colored_points"]:,}点 / 未確定は灰色', flush=True)
    return diagnostic


def recolor_new_output(output, bag, policy_path=None, calibration_path=None):
    """Finish a newly generated map atomically; retained geometry stays identical."""
    work = output/'.recolor_work'
    diagnostic = recolor(output, work, bag, policy_path, calibration_path)
    original_xyz, _ = read_ply(output/'map_3d.ply')
    colored_xyz, _ = read_ply(work/'map_3d_colored.ply')
    if not np.array_equal(original_xyz, colored_xyz):
        raise ValueError('再着色でXYZが変化しました。地図を置き換えません。')
    shutil.copyfile(output/'map_3d_colored.ply', output/'map_3d_colored_recorded.ply')
    for name in ('map_3d_colored.ply', 'report.json', 'color_observations.npz', 'color_image_motion.npz',
                 'colorization_policy.yaml', 'theta_calibration_used.yaml'):
        (work/name).replace(output/name)
    shutil.rmtree(work)
    return diagnostic


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', nargs='?', help='生成済み地図ディレクトリ（省略時は番号選択）')
    parser.add_argument('-o','--output')
    parser.add_argument('--bag')
    parser.add_argument('--colorization-config')
    parser.add_argument('--calibration')
    parser.add_argument('--max-images', type=int, default=0, help='検証専用の画像数制限。出力を通常の地図候補から隠す')
    args = parser.parse_args()
    try:
        if args.source is None:
            from map_selection import map_candidates, choose_numbered
            root = Path.home()/'rosbag2_data'
            value = choose_numbered(lambda: [p for p in map_candidates(root) if (p.parent/'report.json').is_file()],
                                    '色付けをやり直す生成済み地図', root)
            if value is None:
                return 0
            args.source = str(value.parent)
        source = Path(args.source).expanduser().resolve()
        recolor(source, args.output or str(source)+'_recolored', args.bag, args.colorization_config,
                args.calibration, args.max_images)
        return 0
    except (ValueError, KeyError, OSError, EOFError) as error:
        print(f'ERROR: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
