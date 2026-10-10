#!/usr/bin/env python3
"""rosbag(mcap) -> 色付き3D地図(PLY) オフライン生成（open3d非依存 / numpyのみ）。

/velodyne_points_colored（または /velodyne_points）を /tf, /tf_static で
固定フレーム（既定 sirius3/odom）へ変換してボクセル平均で蓄積し、色付きPLYを保存する。
VLP-16は1スキャンが薄い扇なので、走行中の多数スキャンを積んで立体地図にする。

使い方:
  python3 theta_bag_to_ply.py <bag_dir> [-o out.ply] [--frame sirius3/odom]
      [--voxel 0.05] [--min-range 0.5] [--max-range 60] [--drop-uncolored]

必要: rclpy, tf2_ros, rosbag2_py, sensor_msgs_py, numpy
"""
import argparse
import sys

import numpy as np

import rclpy
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rclpy.duration import Duration
import rosbag2_py
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros

VOX_OFFSET = 1 << 20          # 負座標用オフセット（5cmで約52kmまで）
VOX_LIMIT = (1 << 21) - 1     # 各軸の上限（21bit）


def quaternion_to_matrix(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def open_reader(bag_path, topics=None):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    if topics is not None:
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return reader


def load_tf(bag_path, buffer):
    reader = open_reader(bag_path, ['/tf', '/tf_static'])
    types = {t.name for t in reader.get_all_topics_and_types()}
    n_dyn = n_static = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in types:
            continue
        msg = deserialize_message(data, TFMessage)
        for tr in msg.transforms:
            if topic == '/tf_static':
                buffer.set_transform_static(tr, 'bag')
                n_static += 1
            else:
                buffer.set_transform(tr, 'bag')
                n_dyn += 1
    return n_dyn, n_static


def extract_points(msg, drop_uncolored, min_range, max_range):
    pts = pc2.read_points(msg, field_names=None, skip_nans=False)
    xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=-1).astype(np.float64)
    finite = np.isfinite(xyz).all(axis=1)
    if min_range > 0.0 or max_range > 0.0:
        dist = np.linalg.norm(xyz, axis=-1)
        if min_range > 0.0:
            finite &= dist >= min_range
        if max_range > 0.0:
            finite &= dist <= max_range
    colors = np.zeros((xyz.shape[0], 3), dtype=np.float64)
    if 'rgb' in pts.dtype.names:
        packed = np.asarray(pts['rgb'], dtype=np.float32).view(np.uint32)
        colors[:, 0] = ((packed >> 16) & 255) / 255.0
        colors[:, 1] = ((packed >> 8) & 255) / 255.0
        colors[:, 2] = (packed & 255) / 255.0
        if drop_uncolored:
            finite &= (packed != 0)
    return xyz[finite], colors[finite]


def voxel_keys(xyz, voxel):
    q = np.floor(xyz / voxel).astype(np.int64) + VOX_OFFSET
    np.clip(q, 0, VOX_LIMIT, out=q)
    return (q[:, 0] << 42) | (q[:, 1] << 21) | q[:, 2]


def voxel_reduce(xyz, colors, voxel):
    """同一ボクセル内の座標・色を平均して1点にまとめる。"""
    if xyz.shape[0] == 0:
        return xyz, colors
    keys = voxel_keys(xyz, voxel)
    uniq, inv = np.unique(keys, return_inverse=True)
    m = uniq.shape[0]
    cnt = np.bincount(inv, minlength=m).astype(np.float64)
    sx = np.zeros((m, 3), dtype=np.float64)
    sc = np.zeros((m, 3), dtype=np.float64)
    np.add.at(sx, inv, xyz)
    np.add.at(sc, inv, colors)
    return sx / cnt[:, None], sc / cnt[:, None]


def write_ply(path, xyz, rgb_u8):
    n = xyz.shape[0]
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {n}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n"
    )
    dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                      ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')])
    arr = np.empty(n, dtype=dtype)
    arr['x'], arr['y'], arr['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    arr['red'], arr['green'], arr['blue'] = rgb_u8[:, 0], rgb_u8[:, 1], rgb_u8[:, 2]
    with open(path, 'wb') as stream:
        stream.write(header.encode('ascii'))
        stream.write(arr.tobytes())


def main():
    parser = argparse.ArgumentParser(description='rosbag -> colored 3D map (PLY)')
    parser.add_argument('bag')
    parser.add_argument('-o', '--out', default='colored_map.ply')
    parser.add_argument('--frame', default='sirius3/odom', help='出力固定フレーム')
    parser.add_argument('--topic', default='/velodyne_points_colored', help='入力点群トピック')
    parser.add_argument('--voxel', type=float, default=0.05, help='ボクセルサイズ[m]')
    parser.add_argument('--min-range', type=float, default=0.5)
    parser.add_argument('--max-range', type=float, default=60.0)
    parser.add_argument('--drop-uncolored', action='store_true', help='色なし点を除外')
    parser.add_argument('--flush-every', type=int, default=200, help='指定スキャン毎に中間統合')
    args = parser.parse_args()

    rclpy.init()
    try:
        buffer = tf2_ros.Buffer(cache_time=Duration(seconds=1e9))
        n_dyn, n_static = load_tf(args.bag, buffer)
        print(f'TF loaded: dynamic={n_dyn} static={n_static}')

        reader = open_reader(args.bag)
        type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
        if args.topic not in type_map:
            clouds = [k for k, v in type_map.items() if v == 'sensor_msgs/msg/PointCloud2']
            print(f'ERROR: {args.topic} がbagにありません。存在する点群: {clouds}')
            return 2

        reader = open_reader(args.bag, [args.topic])
        acc_xyz, acc_col = [], []
        total_in = used = kept = 0
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic != args.topic:
                continue
            msg = deserialize_message(data, PointCloud2)
            total_in += 1
            xyz, colors = extract_points(msg, args.drop_uncolored, args.min_range, args.max_range)
            if xyz.shape[0] == 0:
                continue
            stamp = Time.from_msg(msg.header.stamp)
            src = msg.header.frame_id or args.frame
            try:
                tr = buffer.lookup_transform(args.frame, src, stamp, Duration(seconds=1.0))
            except Exception as error:  # TransformException 等
                if used == 0:
                    print(f'TF lookup {args.frame} <- {src} 失敗: {error}')
                continue
            t = tr.transform.translation
            q = tr.transform.rotation
            rot = quaternion_to_matrix(q.x, q.y, q.z, q.w)
            xyz_t = xyz @ rot.T + np.array([t.x, t.y, t.z])
            sx, sc = voxel_reduce(xyz_t, colors, args.voxel)
            acc_xyz.append(sx)
            acc_col.append(sc)
            kept += xyz.shape[0]
            used += 1
            if args.flush_every > 0 and used % args.flush_every == 0:
                mx = np.concatenate(acc_xyz)
                mc = np.concatenate(acc_col)
                mx, mc = voxel_reduce(mx, mc, args.voxel)
                acc_xyz, acc_col = [mx], [mc]
                print(f'  {used} scans, {mx.shape[0]} voxels')

        if used == 0:
            print('ERROR: 変換できたスキャンがありません（トピック/TF/frameを確認）')
            return 3
        xyz = np.concatenate(acc_xyz)
        col = np.concatenate(acc_col)
        xyz, col = voxel_reduce(xyz, col, args.voxel)
        rgb_u8 = np.clip(np.round(col * 255.0), 0, 255).astype(np.uint8)
        write_ply(args.out, xyz, rgb_u8)
        print(f'SUCCESS: {args.out} ({xyz.shape[0]} points, {used}/{total_in} scans, '
              f'{kept} points pooled)')
        return 0
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
