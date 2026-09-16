#!/usr/bin/env python3
"""THETA S (HDMI->USBキャプチャ) の映像が取得できているか確認する。

使い方:
  check_theta_capture [/dev/theta_capture|/dev/video0]

デバイスが theta_capture に占有されている場合は、公開トピック
(/theta/dual_fisheye/image_raw) から確認する（占有中でも確認可能）。

ボードにより MJPG が一様フレームになる場合があるため YUYV も試し、内容のある方を採用する。
信号が無い（一様）場合は THETA S/ボードのHDMI入力を確認する。
"""
import errno
import os
import sys
import time

import cv2
import numpy as np

TOPIC = '/theta/dual_fisheye/image_raw'


def device_busy(device):
    try:
        fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
        os.close(fd)
        return False
    except OSError as error:
        return error.errno == errno.EBUSY


def read_device(device):
    best = None  # (std, frame, fourcc)
    for fourcc in ('YUYV', 'MJPG'):
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        for _ in range(20):
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                std = float(frame.std())
                if best is None or std > best[0]:
                    best = (std, frame, fourcc)
        cap.release()
    return best


def read_topic(topic, timeout=4.0):
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
    except Exception:
        return None
    rclpy.init()
    node = Node('check_theta_capture')
    got = []
    node.create_subscription(Image, topic, lambda m: got.append(m), 1)
    t = time.time()
    while time.time() - t < timeout and not got and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    best = None
    for m in got[:5]:
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        std = float(img.std())
        if best is None or std > best[0]:
            best = (std, img.copy(), 'topic')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return best


def main():
    device = sys.argv[1] if len(sys.argv) > 1 else '/dev/theta_capture'
    if not os.path.exists(device):
        device = '/dev/video0'
        print(f'/dev/theta_capture が無いので {device} を使用（udev未設定？）')

    source = 'device'
    best = read_device(device)
    if best is None:
        if device_busy(device):
            print(f'{device} は使用中（theta_capture が起動中？）。公開トピックで確認します。')
        else:
            print(f'{device} からフレームを取得できません（theta_capture が使用中の可能性）。'
                  f'公開トピックで確認します。')
        best = read_topic(TOPIC)
        source = 'topic'
    if best is None:
        print(f'✗ {device} / {TOPIC} のどちらからもフレームを取得できません。')
        print('  theta_capture が起動しているか、HDMI信号・USB接続を確認してください。')
        sys.exit(1)

    std, frame, fourcc = best
    mean = float(frame.mean())
    out = '/tmp/theta_capture_probe.png'
    cv2.imwrite(out, frame)
    label = f'fourcc={fourcc} frame={frame.shape}' if source == 'device' else f'topic={TOPIC} frame={frame.shape}'
    print(f'[{source}] {label} mean={mean:.1f} std={std:.1f}')
    print(f'  保存: {out}')
    if std < 5.0:
        print('✗ 一様フレーム＝HDMI信号なし。THETA S/ボードのHDMI入力を確認してください。')
        sys.exit(2)
    print('✓ 映像を取得できています（dual-fisheye）。')


if __name__ == '__main__':
    main()
