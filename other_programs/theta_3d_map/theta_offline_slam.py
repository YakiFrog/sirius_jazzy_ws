#!/usr/bin/env python3
"""色付き3D地図 オフラインSLAM（ファイル直読み・DDS不使用）。

KISS-ICP でLiDARオドメトリ（EKFより高精度）→ Open3Dポーズグラフでループ閉じ込め
最適化 → 最適化姿勢で色付きキーフレームを再配置してPLY保存。
ROSの再生/DDS/QoS/時刻同期を使わないため安定して完走する。

Open3D規約: node.pose = W(local->world)、edge(source,target,X) は X = inv(W[target])@W[source]
（=sourceをtargetへ合わせるICP変換）。

必要: rosbag2_py, sensor_msgs_py, numpy, open3d, kiss_icp
"""
import argparse
import sys

import numpy as np

from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
from kiss_icp.kiss_icp import KissICP
from kiss_icp.config import KISSConfig

VOX_OFFSET = 1 << 20
VOX_LIMIT = (1 << 21) - 1


def voxel_reduce(xyz, colors, voxel):
    if xyz.shape[0] == 0:
        return xyz, colors
    q = np.floor(xyz / voxel).astype(np.int64) + VOX_OFFSET
    np.clip(q, 0, VOX_LIMIT, out=q)
    keys = (q[:, 0] << 42) | (q[:, 1] << 21) | q[:, 2]
    uniq, inv = np.unique(keys, return_inverse=True)
    inv = inv.reshape(-1)
    m = uniq.shape[0]
    cnt = np.bincount(inv, minlength=m).astype(np.float64)
    out_xyz = np.empty((m, 3)); out_col = np.empty((m, 3))
    for i in range(3):
        out_xyz[:, i] = np.bincount(inv, weights=xyz[:, i], minlength=m) / cnt
        out_col[:, i] = np.bincount(inv, weights=colors[:, i], minlength=m) / cnt
    return out_xyz, out_col


def write_ply(path, xyz, rgb_u8):
    n = xyz.shape[0]
    header = ("ply\nformat binary_little_endian 1.0\n" f"element vertex {n}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
    dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                      ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')])
    arr = np.empty(n, dtype=dtype)
    arr['x'], arr['y'], arr['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    arr['red'], arr['green'], arr['blue'] = rgb_u8[:, 0], rgb_u8[:, 1], rgb_u8[:, 2]
    with open(path, 'wb') as f:
        f.write(header.encode('ascii')); f.write(arr.tobytes())


def open_reader(bag, topic):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    return r


def read_scan(msg, min_range, max_range):
    pts = pc2.read_points(msg, field_names=None, skip_nans=True)
    xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=-1).astype(np.float64)
    packed = np.asarray(pts['rgb'], dtype=np.float32).view(np.uint32)
    col = np.stack([((packed >> 16) & 255), ((packed >> 8) & 255), (packed & 255)],
                   axis=-1).astype(np.float64) / 255.0
    dist = np.linalg.norm(xyz, axis=-1)
    keep = (dist >= min_range) & (dist <= max_range) & (packed != 0)
    return xyz[keep], col[keep]


def icp_align(src, tgt, init, max_corr):
    est = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    crit = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40)
    return o3d.pipelines.registration.registration_icp(src, tgt, max_corr, init, est, crit)


def global_align(src, tgt):
    """FPFH+RANSACで大域位置合わせ→ICPで精緻化（廊下の反復構造に強い）。"""
    src_d = src.voxel_down_sample(0.3)
    tgt_d = tgt.voxel_down_sample(0.3)
    radius = 0.8
    src_f = o3d.pipelines.registration.compute_fpfh_feature(
        src_d, o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=100))
    tgt_f = o3d.pipelines.registration.compute_fpfh_feature(
        tgt_d, o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=100))
    ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src_d, tgt_d, src_f, tgt_f, True, 1.0,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
        [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
         o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(1.0)],
        o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return icp_align(src, tgt, ransac.transformation, 0.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('-o', '--out', default='colored_map_slam.ply')
    ap.add_argument('--topic', default='/velodyne_points_colored')
    ap.add_argument('--voxel', type=float, default=0.05)
    ap.add_argument('--kf-voxel', type=float, default=0.2)
    ap.add_argument('--keyframe-dist', type=float, default=0.8)
    ap.add_argument('--loop-radius', type=float, default=6.0)
    ap.add_argument('--loop-min-gap', type=float, default=10.0)
    ap.add_argument('--loop-min-move', type=float, default=15.0)
    ap.add_argument('--min-range', type=float, default=0.5)
    ap.add_argument('--max-range', type=float, default=60.0)
    ap.add_argument('--no-loop', action='store_true')
    ap.add_argument('--start', type=float, default=0.0, help='bag先頭からの処理開始[s]')
    ap.add_argument('--end', type=float, default=1e9, help='bag先頭からの処理終了[s]')
    args = ap.parse_args()

    config = KISSConfig()
    config.data.deskew = False
    config.data.max_range = args.max_range
    config.data.min_range = args.min_range
    config.mapping.voxel_size = 1.0  # 1.3.0はNone非対応
    odom = KissICP(config)

    reader = open_reader(args.bag, args.topic)
    kf_pts, kf_col, kf_pose, kf_stamp = [], [], [], []
    last_xy = None
    n = 0
    t0 = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, PointCloud2)
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t0 is None:
            t0 = t_sec
        rel = t_sec - t0
        if rel < args.start or rel > args.end:
            continue
        xyz, col = read_scan(msg, args.min_range, args.max_range)
        if xyz.shape[0] < 50:
            continue
        odom.register_frame(xyz, np.zeros(xyz.shape[0]))
        W = odom.last_pose.copy()
        n += 1
        if last_xy is None or np.linalg.norm(W[:2, 3] - last_xy) >= args.keyframe_dist:
            sx, sc = voxel_reduce(xyz, col, args.kf_voxel)
            kf_pts.append(sx); kf_col.append(sc); kf_pose.append(W); kf_stamp.append(t_sec)
            last_xy = W[:2, 3].copy()
    print(f'scans={n} keyframes={len(kf_pts)}', flush=True)
    if len(kf_pts) < 3:
        print('キーフレーム不足'); return 1
    p0, p1 = kf_pose[0][:2, 3], kf_pose[-1][:2, 3]
    print(f'[KISS] start={np.round(p0,2)} end={np.round(p1,2)} '
          f'closure_gap={np.linalg.norm(p1-p0):.2f}m', flush=True)

    clouds = []
    for sx in kf_pts:
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(sx)
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=1.5, max_nn=30))
        clouds.append(pc)

    pg = o3d.pipelines.registration.PoseGraph()
    for W in kf_pose:
        pg.nodes.append(o3d.pipelines.registration.PoseGraphNode(W.copy()))
    for i in range(len(kf_pose) - 1):
        X = np.linalg.inv(kf_pose[i + 1]) @ kf_pose[i]  # source=i -> target=i+1
        pg.edges.append(o3d.pipelines.registration.PoseGraphEdge(
            i, i + 1, X, np.eye(6), uncertain=False))

    n_loops = 0
    if not args.no_loop:
        for i in range(len(kf_pose)):
            best_j, best_d = None, args.loop_radius
            for j in range(i):
                if kf_stamp[i] - kf_stamp[j] < args.loop_min_gap:
                    continue
                d = np.linalg.norm(kf_pose[i][:2, 3] - kf_pose[j][:2, 3])
                if d < best_d:
                    best_j, best_d = j, d
            if best_j is None:
                continue
            init = np.linalg.inv(kf_pose[best_j]) @ kf_pose[i]  # i -> j
            res = icp_align(clouds[i], clouds[best_j], init, 1.0)
            if res.fitness > 0.5:
                pg.edges.append(o3d.pipelines.registration.PoseGraphEdge(
                    i, best_j, res.transformation, np.eye(6) * 5.0, uncertain=True))
                n_loops += 1
    print(f'loop edges={n_loops}', flush=True)

    o3d.pipelines.registration.global_optimization(
        pg, o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=0.25, edge_prune_threshold=0.25, reference_node=0))
    q0, q1 = pg.nodes[0].pose[:2, 3], pg.nodes[-1].pose[:2, 3]
    print(f'[OPT] start={np.round(q0,2)} end={np.round(q1,2)} '
          f'closure_gap={np.linalg.norm(q1-q0):.2f}m', flush=True)

    all_xyz, all_col = [], []
    for i in range(len(kf_pts)):
        W = pg.nodes[i].pose  # optimized local->world
        pts = kf_pts[i] @ W[:3, :3].T + W[:3, 3]
        all_xyz.append(pts); all_col.append(kf_col[i])
    xyz = np.concatenate(all_xyz); col = np.concatenate(all_col)
    xyz, col = voxel_reduce(xyz, col, args.voxel)
    rgb = np.clip(np.round(col * 255.0), 0, 255).astype(np.uint8)
    write_ply(args.out, xyz, rgb)
    print(f'SUCCESS: {args.out} ({xyz.shape[0]} points, keys={len(kf_pts)}, loops={n_loops})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
