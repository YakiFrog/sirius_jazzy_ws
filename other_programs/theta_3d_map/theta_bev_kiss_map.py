#!/usr/bin/env python3
"""BEV×KISS-ICP ハイブリッド路面マップ（路面のみ・遮蔽除去・シャープ版）。

改善点:
  1) 遮蔽除去: 各THETAフレーム時刻に最も近い LiDAR スキャンから方位ごとの障害物距離を作り、
     その向こう側になる BEV 地面点を除去（壁に貼られた床テクスチャ=壁貫通を消す）。
  2) シャープ化: セルごとに「センサに最も近い観測（分解能が高い）」の色を採用（平均しない）。

- 姿勢: /velodyne_points でKISS-ICP → T_world_lidar(t)
- 色  : /theta/.../compressed をBEV化（既存 theta_bev_projection）
- 出力: ground_surf.png / ground_surf.ply（z=0平面, 路面のみ）
"""
import argparse
import os
import sys

import cv2
import numpy as np
import yaml

from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import PointCloud2, CompressedImage
import sensor_msgs_py.point_cloud2 as pc2
from kiss_icp.kiss_icp import KissICP
from kiss_icp.config import KISSConfig

from ament_index_python.packages import get_package_share_directory
from sirius_navigation.theta_bev_projection import build_blend_maps


def open_reader(bag, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return r


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--out-prefix', default='')
    ap.add_argument('--lidar-topic', default='/velodyne_points')
    ap.add_argument('--image-topic', default='/theta/dual_fisheye/image_raw/compressed')
    ap.add_argument('--cell', type=float, default=0.03)
    ap.add_argument('--bev-stride', type=int, default=0)
    ap.add_argument('--min-radius', type=float, default=1.2, help='自機の足元（中心）をマスク[m]')
    ap.add_argument('--max-radius', type=float, default=7.0)
    ap.add_argument('--min-value', type=int, default=8)
    ap.add_argument('--lidar-height', type=float, default=0.85)
    ap.add_argument('--max-range', type=float, default=60.0)
    ap.add_argument('--gate-margin', type=float, default=0.25)
    ap.add_argument('--min-count', type=int, default=2, help='セルを採用する最小観測フレーム数（動的物体の残像除去）')
    args = ap.parse_args()

    prefix = args.out_prefix or os.path.join(args.bag, 'ground_surf')
    os.makedirs(os.path.dirname(prefix) or '.', exist_ok=True)
    with open(os.path.join(get_package_share_directory('sirius_navigation'),
                           'config', 'theta_calibration_real.yaml')) as f:
        calib = yaml.safe_load(f)
    bev_size, extent = int(calib['bev_size']), float(calib['bev_extent_m'])

    # --- Pass1: KISS-ICP姿勢 + 方位別障害物距離 ---
    config = KISSConfig()
    config.data.deskew = False
    config.data.max_range = args.max_range
    config.mapping.voxel_size = 1.0
    odom = KissICP(config)
    nbins = 720
    ang_inc = 2 * np.pi / nbins
    times, poses, obs_list = [], [], []
    reader = open_reader(args.bag, [args.lidar_topic])
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, PointCloud2)
        pts = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] < 50:
            continue
        xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=-1).astype(np.float64)
        odom.register_frame(xyz, np.zeros(xyz.shape[0]))
        times.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        poses.append(odom.last_pose.copy())
        r = np.hypot(xyz[:, 0], xyz[:, 1])
        wall = (xyz[:, 2] > -0.30) & (xyz[:, 2] < 1.20) & (r > 0.3)  # 地面を除く壁/什器
        obs = np.full(nbins, np.inf, dtype=np.float32)
        if np.any(wall):
            ang = np.arctan2(xyz[wall, 1], xyz[wall, 0])
            idx = np.floor((ang + np.pi) / ang_inc).astype(np.int64) % nbins
            np.minimum.at(obs, idx, r[wall].astype(np.float32))
        obs_list.append(obs)
    if len(poses) < 2:
        print('LiDAR姿勢が不足'); return 2
    times = np.asarray(times); poses = np.asarray(poses)
    print(f'lio poses={len(poses)}', flush=True)

    # --- 出力グリッド（姿勢範囲+max_radius で確保） ---
    xy = poses[:, :2, 3]
    x0, y0 = xy.min(axis=0) - (args.max_radius + 2.0)
    x1, y1 = xy.max(axis=0) + (args.max_radius + 2.0)
    cell = args.cell
    nx = int(np.ceil((x1 - x0) / cell)) + 1
    ny = int(np.ceil((y1 - y0) / cell)) + 1
    best_r = np.full(nx * ny, np.inf, dtype=np.float32)
    best_c = np.zeros((nx * ny, 3), dtype=np.uint8)
    best_count = np.zeros(nx * ny, dtype=np.uint16)

    # --- Pass2: BEV を姿勢で配置（遮蔽除去 + 最近傍観測の色） ---
    stride = args.bev_stride if args.bev_stride > 0 else max(1, bev_size // 192)
    u = np.arange(0, bev_size, stride)
    v = np.arange(0, bev_size, stride)
    uu, vv = np.meshgrid(u, v)
    gx = ((0.5 - (vv + 0.5) / bev_size) * extent)
    gy = ((0.5 - (uu + 0.5) / bev_size) * extent)
    gr = np.hypot(gx, gy)
    gvalid = (gr >= args.min_radius) & (gr <= args.max_radius)
    gx, gy, gr = gx[gvalid], gy[gvalid], gr[gvalid]
    gu, gv = uu[gvalid], vv[gvalid]
    gang = np.arctan2(gy, gx)
    gidx = np.floor((gang + np.pi) / ang_inc).astype(np.int64) % nbins

    T_lidar_base = np.eye(4); T_lidar_base[2, 3] = -args.lidar_height
    shape = None
    n = 0
    reader = open_reader(args.bag, [args.image_topic])
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, CompressedImage)
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        if shape != frame.shape:
            mx_f, my_f, mx_b, my_b, alpha = build_blend_maps(calib, frame.shape[1], frame.shape[0], 6.0)
            shape = frame.shape
        front = cv2.remap(frame, mx_f, my_f, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        back = cv2.remap(frame, mx_b, my_b, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        a = alpha[..., None]
        bev = np.clip(front.astype(np.float32) * a + back.astype(np.float32) * (1 - a), 0, 255).astype(np.uint8)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        si = int(np.argmin(np.abs(times - t)))
        obs = obs_list[si]
        # 障害物より奥(壁貫通) と 障害物の位置そのもの(人/壁) を除去 → 手前の床だけ残す
        ob = obs[gidx]
        keep = np.isinf(ob) | (gr < (ob - args.gate_margin))
        ggx, ggy, grr = gx[keep], gy[keep], gr[keep]
        pix = bev[gv[keep], gu[keep]][:, ::-1].copy()  # BGR -> RGB
        bright = pix.max(axis=1) >= args.min_value
        ggx, ggy, grr, pix = ggx[bright], ggy[bright], grr[bright], pix[bright]
        if pix.shape[0] == 0:
            continue
        W = poses[si] @ T_lidar_base
        xw = W[0, 0] * ggx + W[0, 1] * ggy + W[0, 3]
        yw = W[1, 0] * ggx + W[1, 1] * ggy + W[1, 3]
        cx = np.floor((xw - x0) / cell).astype(np.int64)
        cy = np.floor((yw - y0) / cell).astype(np.int64)
        inside = (cx >= 0) & (cx < nx) & (cy >= 0) & (cy < ny)
        if not np.any(inside):
            continue
        flat = cx[inside] * ny + cy[inside]
        best_count[np.unique(flat)] += 1
        rr = grr[inside]
        cc = pix[inside]
        better = rr < best_r[flat]
        if np.any(better):
            sel = flat[better]
            best_r[sel] = rr[better]
            best_c[sel] = cc[better]
        n += 1
    print(f'bev frames={n}', flush=True)

    valid = np.isfinite(best_r) & (best_count >= args.min_count)
    idx = np.nonzero(valid)[0]
    if idx.size == 0:
        print('有効セルなし'); return 3
    xs = x0 + (idx // ny + 0.5) * cell
    ys = y0 + (idx % ny + 0.5) * cell
    rgb = best_c[idx]
    img = np.full((nx, ny, 3), 255, dtype=np.uint8)
    img[idx // ny, idx % ny] = rgb
    cv2.imwrite(prefix + '.png', img[::-1, :, ::-1])
    write_ply(prefix + '.ply', np.stack([xs, ys, np.zeros_like(xs)], axis=-1).astype(np.float32), rgb)
    print(f'SUCCESS: {prefix}.png ({nx}x{ny}), {prefix}.ply ({idx.size} cells)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
