#!/usr/bin/env python3
"""KISS-ICPで路面(地面)だけをXY投影した色付きマップを作る（実験的）。

bag をファイル直読みし、KISS-ICPの姿勢で各スキャンの地面点をworldへ配置、
XY平面へ投影して色を平均した2Dマップ（PNG）と、z=0に潰した色付き点群(PLY)を出力。
Foxgloveでは PLY を PointCloud2 として配信して3Dで、PNGは Image で見られる。

使い方:
  theta_kiss_ground_map.py <bag> [--out-prefix DIR/name] [--cell 0.05]
      [--ground-z -0.85] [--band 0.3] [--subsample 2]
"""
import argparse
import os
import sys

import numpy as np

from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from kiss_icp.kiss_icp import KissICP
from kiss_icp.config import KISSConfig

VOX_OFFSET = 1 << 20
VOX_LIMIT = (1 << 21) - 1


def open_reader(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
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


def save_png(path, rgb_img):
    """rgb_img: (H,W,3) RGB uint8. cv2優先、無ければPIL。"""
    try:
        import cv2
        cv2.imwrite(path, rgb_img[:, :, ::-1])  # BGRで保存
        return True
    except Exception:
        pass
    try:
        from PIL import Image
        Image.fromarray(rgb_img).save(path)
        return True
    except Exception:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--out-prefix', default='')
    ap.add_argument('--topic', default='/velodyne_points_colored')
    ap.add_argument('--cell', type=float, default=0.05)
    ap.add_argument('--ground-abs', type=float, default=0.85, help='LiDARの地上高[m]（センサ座標で地面判定）')
    ap.add_argument('--band', type=float, default=0.35, help='地面バンド半幅[m]')
    ap.add_argument('--min-range', type=float, default=0.5)
    ap.add_argument('--max-range', type=float, default=30.0)
    ap.add_argument('--subsample', type=int, default=2)
    args = ap.parse_args()

    prefix = args.out_prefix or os.path.join(args.bag, 'ground_map')
    os.makedirs(os.path.dirname(prefix) or '.', exist_ok=True)

    config = KISSConfig()
    config.data.deskew = False
    config.data.max_range = args.max_range
    config.data.min_range = args.min_range
    config.mapping.voxel_size = 1.0
    odom = KissICP(config)

    reader = open_reader(args.bag)
    xs, ys, cs = [], [], []
    n = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != args.topic:
            continue
        msg = deserialize_message(data, PointCloud2)
        pts = pc2.read_points(msg, field_names=None, skip_nans=True)
        if pts.shape[0] < 50:
            continue
        xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=-1).astype(np.float64)
        packed = np.asarray(pts['rgb'], dtype=np.float32).view(np.uint32)
        col = np.stack([((packed >> 16) & 255), ((packed >> 8) & 255), (packed & 255)],
                       axis=-1).astype(np.float64)
        odom.register_frame(xyz, np.zeros(xyz.shape[0]))
        W = odom.last_pose
        # 地面判定はセンサ座標で行う（world z のドリフトに強い）。
        # LiDARは地上 ground_abs にあるので、lidarフレームの地面は z ≈ -ground_abs。
        keep = (xyz[:, 2] >= -(args.ground_abs + args.band)) & \
               (xyz[:, 2] <= -(args.ground_abs - args.band)) & (packed != 0)
        if not np.any(keep):
            continue
        world = xyz[keep] @ W[:3, :3].T + W[:3, 3]
        w = world[::args.subsample]
        c = col[keep][::args.subsample]
        xs.append(w[:, 0]); ys.append(w[:, 1]); cs.append(c)
        n += 1
    print(f'scans={n}', flush=True)
    if not xs:
        print('地面点が取れませんでした'); return 2

    x = np.concatenate(xs); y = np.concatenate(ys); c = np.concatenate(cs)
    print(f'ground points={x.shape[0]}  x {x.min():.1f}..{x.max():.1f}  y {y.min():.1f}..{y.max():.1f}', flush=True)

    cell = args.cell
    ix = np.floor((x - x.min()) / cell).astype(np.int64)
    iy = np.floor((y - y.min()) / cell).astype(np.int64)
    nx, ny = int(ix.max()) + 1, int(iy.max()) + 1
    flat = ix * ny + iy
    total = nx * ny
    cnt = np.bincount(flat, minlength=total).astype(np.float64)
    img = np.full((total, 3), 255.0)  # 背景は白
    for k in range(3):
        s = np.bincount(flat, weights=c[:, k], minlength=total)
        with np.errstate(invalid='ignore', divide='ignore'):
            img[:, k] = np.where(cnt > 0, s / np.maximum(cnt, 1), 255.0)
    img_u8 = np.clip(img, 0, 255).astype(np.uint8).reshape(nx, ny, 3)

    # 画像は上が+yになるよう反転（PLYは座標そのまま）
    png_path = prefix + '.png'
    if save_png(png_path, img_u8[::-1]):
        print(f'SUCCESS png: {png_path} ({nx}x{ny} cells, cell={cell}m)')
    else:
        print('PNG保存に失敗（cv2/PILなし）。PLYのみ出力')

    # z=0へ潰した色付き点群（各セルの平均色）をPLYで保存
    nz = cnt > 0
    cell_idx = np.nonzero(nz)[0]
    cx = x.min() + (cell_idx // ny + 0.5) * cell
    cy = y.min() + (cell_idx % ny + 0.5) * cell
    cz = np.zeros_like(cx)
    ply = np.stack([cx, cy, cz], axis=-1).astype(np.float32)
    pcol = np.clip(img[cell_idx], 0, 255).astype(np.uint8)
    ply_path = prefix + '.ply'
    write_ply(ply_path, ply, pcol)
    print(f'SUCCESS ply: {ply_path} ({np.count_nonzero(nz)} cells)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
