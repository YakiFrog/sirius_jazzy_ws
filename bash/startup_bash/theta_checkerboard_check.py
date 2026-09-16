#!/usr/bin/env python3
"""チェッカーボード検出の切り分けツール（THETA dual-fisheye, 実機）.

魚眼歪みに強い findChessboardCornersSB を使い、左右レンズで検出を試す。
指定パターンで失敗したら広い範囲を総当たりして候補を出す。

  - ライブ取得（/theta/dual_fisheye/image_raw/compressed）または --image で既存画像
  - 生フレームを --out にPNG保存
  - 検出できたらコーナー描画画像も保存

使い方:
  theta_calib_check                 # ライブで確認（10x7 と 7x10 を試し、無ければ総当たり）
  theta_calib_check --image ~/theta_calib/frames/front_xxx.png
"""
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

FLAGS = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE


def try_patterns(frame, patterns):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape[:2]
    sides = (('front', 0, w // 2), ('back', w // 2, w))
    wins = []
    for (c, r) in patterns:
        if c < 3 or r < 3:
            continue
        for name, x0, x1 in sides:
            ok, corners = cv2.findChessboardCornersSB(gray[:, x0:x1], (c, r), FLAGS)
            if ok:
                xs, ys = corners[..., 0], corners[..., 1]
                area = float((xs.max() - xs.min()) * (ys.max() - ys.min())) / ((x1 - x0) * h)
                wins.append((area, c, r, name, corners, x0))
    return wins


def analyse(frame, args):
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    cv2.imwrite(args.out, frame)

    wins = try_patterns(frame, [(args.cols, args.rows), (args.rows, args.cols)])
    if not wins:
        print('指定パターンで未検出 -> 総当たりで探索します…')
        wins = try_patterns(frame, [(c, r) for c in range(4, 14) for r in range(3, 12)])

    if not wins:
        print('検出できませんでした。')
        print('チェック: 黒白のマスが角で接する市松模様か / 片方のレンズに大きく写っているか /')
        print('          明るいか（照明の映り込み回避）/ ボケていないか / ボード全体の四隅が写っているか。')
        print(f'取得フレーム: {args.out}')
        return

    wins.sort(key=lambda t: t[0], reverse=True)
    print('検出成功（面積＝半面に対するボード領域の割合。大きいほど全体に近い）:')
    for area, c, r, name, corners, x0 in wins[:10]:
        print(f'  {c}x{r}  lens={name}  面積={area:.3f}')
        vis = frame.copy()
        pts = corners + np.array([[[x0, 0.0]]], np.float32)
        cv2.drawChessboardCorners(vis, (c, r), pts, True)
        cv2.imwrite(args.out.replace('.png', f'_found_{c}x{r}_{name}.png'), vis)
    area, c, r, name, _, _ = wins[0]
    print(f'-> 最大は {c}x{r} (lens={name})。収集は: theta_calib_capture --cols {c} --rows {r}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cols', type=int, default=9)
    ap.add_argument('--rows', type=int, default=6)
    ap.add_argument('--image', default=None, help='ライブ取得の代わりに既存画像で確認')
    ap.add_argument('--out', default=os.path.expanduser('~/theta_calib/check.png'))
    ap.add_argument('--topic', default='/theta/dual_fisheye/image_raw/compressed')
    args = ap.parse_args()

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f'画像を読めません: {args.image}')
            return
        analyse(frame, args)
        return

    rclpy.init()
    node = Node('theta_checkerboard_check')
    got = []
    node.create_subscription(CompressedImage, args.topic,
                             lambda m: got.append(cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)),
                             qos_profile_sensor_data)
    node.get_logger().info('フレーム待ち（ボードをレンズに大きく写してください）…')
    t = time.time()
    while time.time() - t < 5.0 and not got and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    if not got:
        print('画像を受信できませんでした。theta_capture は起動していますか？')
        return
    analyse(got[-1], args)


if __name__ == '__main__':
    main()
