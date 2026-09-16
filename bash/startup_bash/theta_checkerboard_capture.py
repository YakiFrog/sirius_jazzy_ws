#!/usr/bin/env python3
"""チェッカーボード校正用フレーム収集（THETA dual-fisheye, 実機）.

/theta/dual_fisheye/image_raw/compressed を購読し、前後のどちらかのレンズで
チェッカーボードが検出できたフレームだけを <out>/frames/ にPNG保存する。

2秒ごとに受信状況と検出状況をログに出す:
  受信 12枚 (5.0Hz) / 検出 front=○ back=× / 保存 front=3 back=0
「受信」が増えない → theta_capture が動いていない/トピック違い。
「検出」が常に× → ボードが写っていない/暗い/遠い/大きさ違い。

使い方:
  theta_calib_capture                      # 既定: ~/theta_calib, 9x6
終了は Ctrl-C。
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.expanduser('~/theta_calib'), help='保存先ディレクトリ')
    ap.add_argument('--cols', type=int, default=9, help='チェッカーボード内部コーナー列数')
    ap.add_argument('--rows', type=int, default=6, help='チェッカーボード内部コーナー行数')
    ap.add_argument('--square', type=float, default=0.02, help='1マスの実寸[m]')
    ap.add_argument('--interval', type=float, default=0.7, help='保存の最小間隔[秒]')
    ap.add_argument('--status_sec', type=float, default=2.0, help='ステータス表示の間隔[秒]')
    ap.add_argument('--topic', default='/theta/dual_fisheye/image_raw/compressed')
    args = ap.parse_args()

    frames_dir = os.path.join(args.out, 'frames')
    os.makedirs(frames_dir, exist_ok=True)
    with open(os.path.join(args.out, 'board.txt'), 'w') as f:
        f.write(f'cols={args.cols} rows={args.rows} square_m={args.square}\n')

    pattern = (args.cols, args.rows)
    # 魚眼歪みに強いSB検出器を使う（従来のfindChessboardCornersは歪みで失敗しやすい）。
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
    st = {'recv': 0, 't0': time.time(), 'last_save': 0.0,
          'saved_front': 0, 'saved_back': 0,
          'front': None, 'back': None}

    rclpy.init()
    node = Node('theta_checkerboard_capture')

    def on_image(msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        st['recv'] += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]

        # ボードは片方のレンズ内に収める前提なので、左右の半面だけで検出する。
        def detect_half(x0, x1):
            ok, _ = cv2.findChessboardCornersSB(gray[:, x0:x1], pattern, flags)
            return bool(ok)

        st['front'] = detect_half(0, w // 2)
        st['back'] = detect_half(w // 2, w)

        now = time.time()
        for side in ('front', 'back'):
            if st[side] and now - st['last_save'] >= args.interval:
                st['last_save'] = now
                st['saved_' + side] += 1
                path = os.path.join(frames_dir, f'{side}_{now:.3f}.png')
                cv2.imwrite(path, frame)
                node.get_logger().info(
                    f'保存 {side} (front={st["saved_front"]} back={st["saved_back"]}) '
                    f'-> {os.path.basename(path)}')

    def status():
        dt = max(1e-6, time.time() - st['t0'])
        rate = st['recv'] / dt
        mark = lambda b: '○' if b else '×'   # noqa: E731
        if st['recv'] == 0:
            node.get_logger().warning(
                '画像をまだ受信していません。theta_capture は起動していますか？'
                f'（topic={args.topic}）')
            return
        node.get_logger().info(
            f'受信 {st["recv"]}枚 ({rate:.1f}Hz) / '
            f'検出 front={mark(st["front"])} back={mark(st["back"])} / '
            f'保存 front={st["saved_front"]} back={st["saved_back"]}')

    node.create_subscription(CompressedImage, args.topic, on_image, qos_profile_sensor_data)
    node.create_timer(args.status_sec, status)
    node.get_logger().info(
        f'チェッカーボード({args.cols}x{args.rows}, {args.square*1000:.0f}mm) 収集開始。'
        f'Ctrl-Cで終了。保存先: {frames_dir}')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'終了: 保存 front={st["saved_front"]} back={st["saved_back"]} (受信 {st["recv"]}枚)')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
