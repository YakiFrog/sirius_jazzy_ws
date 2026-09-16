#!/usr/bin/env python3
"""収集したチェッカーボードフレームからTHETA各レンズをcv2.fisheyeで校正する。

front/back それぞれについて内部パラメータ(center, focal, distortion)を求め、
--out 指定時は実機YAML(theta_calibration_real.yaml)を更新する。
--calib を初期値に使う（CALIB_USE_INTRINSIC_GUESS）ことで安定化する。

背景の格子模様を誤検出したフレームが混ざるとrmsが数十pxになる。これを除くため
「校正→各ビューの再投影誤差→悪いビューを除外→再校正」を反復する（ロバスト化）。

使い方:
  theta_calib_solve                 # 既定 ~/theta_calib/frames, 実機YAML
  theta_calib_solve --out <yaml>    # YAMLを更新（元は .bak に退避）
"""
import argparse
import glob
import os
import shutil

import cv2
import numpy as np
import yaml


def detect(gray, pattern):
    # 魚眼歪みに強いSB検出器（従来のfindChessboardCornersは歪みで失敗しやすい）
    ok, corners = cv2.findChessboardCornersSB(
        gray, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
    if not ok:
        return None
    return corners.astype(np.float32)


def default_calib():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('sirius_navigation'),
                            'config', 'theta_calibration_real.yaml')
    except Exception:
        return os.path.expanduser(
            '~/sirius_jazzy_ws/src/sirius/sirius_navigation/config/theta_calibration_real.yaml')


def calibrate(objpoints, imgpoints, image_size, K, D, flags):
    n = len(objpoints)
    rvecs = [np.zeros((1, 1, 3), np.float64) for _ in range(n)]
    tvecs = [np.zeros((1, 1, 3), np.float64) for _ in range(n)]
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
        objpoints, imgpoints, image_size, K, D, rvecs, tvecs, flags, criteria)
    return rms, K, D, rvecs, tvecs


def view_errors(objpoints, imgpoints, rvecs, tvecs, K, D):
    errs = []
    for op, ip, rv, tv in zip(objpoints, imgpoints, rvecs, tvecs):
        proj, _ = cv2.fisheye.projectPoints(op, rv, tv, K, D)
        errs.append(float(np.sqrt(np.mean((proj.reshape(-1, 2) - ip.reshape(-1, 2)) ** 2))))
    return np.array(errs)


def solve_lens(entries, image_size, base_lens):
    """entries: list of (path, objp, imgp)。ロバスト化して (K,D,inlier_paths,errs) を返す。"""
    objpoints = [e[1] for e in entries]
    imgpoints = [e[2] for e in entries]
    paths = [e[0] for e in entries]
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
    K = np.zeros((3, 3), np.float64)
    D = np.zeros((1, 4), np.float64)
    if base_lens:
        fx, fy = base_lens['focal']
        cx, cy = base_lens['center']
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], np.float64)
        D = np.array(base_lens['distortion'], np.float64).reshape(1, 4)
        flags |= cv2.fisheye.CALIB_USE_INTRINSIC_GUESS

    idx = list(range(len(objpoints)))
    errs = None
    for _ in range(6):
        o = [objpoints[i] for i in idx]
        im = [imgpoints[i] for i in idx]
        rms, K, D, rvecs, tvecs = calibrate(o, im, image_size, K.copy(), D.copy(), flags)
        e = view_errors(o, im, rvecs, tvecs, K, D)
        med = float(np.median(e))
        thr = max(1.0, med + 3.0 * max(1e-6, float(np.median(np.abs(e - med))) * 1.4826), med * 3.0)
        keep = [idx[i] for i in range(len(idx)) if e[i] <= thr]
        errs = e
        print(f'    iter: ビュー{len(idx)} rms={rms:.3f} 中央値={med:.3f} 除外閾値={thr:.2f} -> 残り{len(keep)}')
        if len(keep) == len(idx) or len(keep) < 6:
            idx = keep
            break
        idx = keep
    o = [objpoints[i] for i in idx]
    im = [imgpoints[i] for i in idx]
    rms, K, D, rvecs, tvecs = calibrate(o, im, image_size, K.copy(), D.copy(), flags)
    e = view_errors(o, im, rvecs, tvecs, K, D)
    # 最終的な外れ値ビュー（除外したもの）を報告
    dropped = [paths[i] for i in range(len(paths)) if i not in idx]
    return rms, K, D, [paths[i] for i in idx], e, dropped


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('frames', nargs='?', default=os.path.expanduser('~/theta_calib/frames'),
                    help='フレームディレクトリ（既定 ~/theta_calib/frames）')
    ap.add_argument('--cols', type=int, default=9, help='内部コーナー列数（10x7マスなら9）')
    ap.add_argument('--rows', type=int, default=6, help='内部コーナー行数（10x7マスなら6）')
    ap.add_argument('--square', type=float, default=0.02, help='1マス実寸[m]')
    ap.add_argument('--calib', default=default_calib(), help='初期値/更新元の実機YAML')
    ap.add_argument('--out', default=None, help='結果YAMLの出力先（省略時は表示のみ）')
    args = ap.parse_args()

    pattern = (args.cols, args.rows)
    npts = args.cols * args.rows
    objp = np.zeros((1, npts, 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    objp *= args.square

    base = None
    if args.calib and os.path.exists(args.calib):
        with open(args.calib) as f:
            base = yaml.safe_load(f)

    files = sorted(glob.glob(os.path.join(args.frames, '*.png'))
                   + glob.glob(os.path.join(args.frames, '*.jpg')))
    if not files:
        raise SystemExit(f'フレームがありません: {args.frames}\n'
                         f'先に theta_calib_capture でチェッカーボードを収集してください。')

    per_lens = {'front': [], 'back': []}
    image_size = None
    for path in files:
        img = cv2.imread(path)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]
        image_size = (w, h)
        for side, x0, x1 in (('front', 0, w // 2), ('back', w // 2, w)):
            corners = detect(gray[:, x0:x1], pattern)
            if corners is not None:
                if x0:  # 半面の座標を全画像座標へ（backは+width/2）
                    corners = corners + np.array([[[float(x0), 0.0]]], np.float32)
                per_lens[side].append((path, objp.copy(), corners))

    results = {}
    for side in ('front', 'back'):
        entries = per_lens[side]
        print(f'[{side}] 検出ビュー数: {len(entries)}')
        if len(entries) < 8:
            print('  -> ビュー不足（目安15以上）。スキップ。')
            results[side] = None
            continue
        rms, K, D, inliers, errs, dropped = solve_lens(entries, image_size, base[side] if base else None)
        cx, cy = float(K[0, 2]), float(K[1, 2])
        fx, fy = float(K[0, 0]), float(K[1, 1])
        dist = [float(v) for v in D.ravel()]
        print(f'  最終: rms={rms:.3f}px  使用ビュー={len(inliers)}  除外={len(dropped)}')
        print(f'  center=({cx:.1f},{cy:.1f})  focal=({fx:.1f},{fy:.1f})')
        print(f'  distortion={[round(v, 6) for v in dist]}')
        if dropped:
            print(f'  除外した例: {[os.path.basename(p) for p in dropped[:5]]}')
        results[side] = {'center': [cx, cy], 'focal': [fx, fy], 'distortion': dist}

    if args.out and base:
        out = dict(base)
        for side in ('front', 'back'):
            if results.get(side):
                out[side] = dict(out[side])
                out[side].update(results[side])
        out['image_size'] = [int(image_size[0]), int(image_size[1])]
        if os.path.exists(args.out):
            shutil.copy(args.out, args.out + '.bak')
        with open(args.out, 'w') as f:
            yaml.safe_dump(out, f, sort_keys=False, allow_unicode=True)
        print(f'書き出し: {args.out} (image_size={out["image_size"]}, 元は .bak に退避)')


if __name__ == '__main__':
    main()
