#!/usr/bin/env python3
"""THETA S (HDMI->USBキャプチャ) の映像が取得できているか確認する。

使い方:
  python3 check_theta_capture.py [/dev/theta_capture|/dev/video0]

ボードにより MJPG が一様フレームになる場合があるため YUYV も試し、内容のある方を採用する。
信号が無い（一様）場合は THETA S/ボードのHDMI入力を確認する。
"""
import os
import sys
import cv2
import numpy as np

device = sys.argv[1] if len(sys.argv) > 1 else '/dev/theta_capture'
if not os.path.exists(device):
    device = '/dev/video0'
    print(f'/dev/theta_capture が無いので {device} を使用（udev未設定？）')

best = None  # (std, frame, fourcc, shape)
for fourcc in ('YUYV', 'MJPG'):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        continue
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    for _ in range(20):
        ret, frame = cap.read()
        if ret and frame is not None and frame.size > 0:
            std = float(frame.std())
            if best is None or std > best[0]:
                best = (std, frame, fourcc, frame.shape)
    cap.release()

if best is None:
    print(f'✗ {device}: フレームを取得できません')
    sys.exit(1)
std, frame, fourcc, shape = best
mean = float(frame.mean())
out = '/tmp/theta_capture_probe.png'
cv2.imwrite(out, frame)
print(f'{device}: fourcc={fourcc} frame={shape} mean={mean:.1f} std={std:.1f}')
print(f'  保存: {out}')
if std < 5.0:
    print('✗ 一様フレーム＝HDMI信号なし。THETA S/ボードのHDMI入力を確認してください。')
    sys.exit(2)
print('✓ 映像を取得できています（dual-fisheye）。')
