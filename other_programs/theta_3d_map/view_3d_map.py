#!/usr/bin/env python3
"""Select a generated PLY and publish XYZ/RGB maps for Foxglove."""
import argparse
import math
import os
from pathlib import Path
import sys

from map_selection import choose_numbered, map_candidates


def resolve_ply(value):
    path = Path(value).expanduser().resolve()
    if path.is_dir():
        for candidate in (path / 'map_3d_colored.ply', path / 'offline_3d' / 'map_3d_colored.ply',
                          path / 'map_3d.ply', path / 'offline_3d' / 'map_3d.ply'):
            if candidate.is_file():
                return candidate
    elif path.is_file() and path.suffix.lower() == '.ply':
        return path
    raise ValueError(f'生成済みPLYが見つかりません: {path}')


def choose_z_limits():
    print('\nZカット（表示だけ変更。高さは地図座標、単位m）\n'
          '  [1] カットなし（既定）\n'
          '  [2] 上側を隠す（天井）\n'
          '  [3] 下側を隠す（床）\n'
          '  [4] 上下を指定する\n'
          '  q: 中止', flush=True)
    while True:
        try:
            choice = input('番号を入力（Enter=1）: ').strip().lower() or '1'
        except EOFError:
            return -1.e9, 1.e9  # No input: retain the default, uncut view.
        if choice == 'q':
            return None
        if choice == '1':
            return -1.e9, 1.e9
        if choice in ('2', '3', '4'):
            break
        print('1〜4、またはqを入力してください。', flush=True)

    def height(prompt, default):
        while True:
            value = input(f'{prompt}（Enter={default:g}、q=中止）: ').strip().lower()
            if value == 'q':
                return None
            try:
                result = float(value) if value else default
                if math.isfinite(result):
                    return result
            except ValueError:
                pass
            print('有限の数値をm単位で入力してください（例: 1.8）。', flush=True)

    while True:
        lower = height('表示するZの下限[m]（これより下を隠す）', .2) if choice in ('3', '4') else -1.e9
        if lower is None:
            return None
        upper = height('表示するZの上限[m]（これより上を隠す）', 2.) if choice in ('2', '4') else 1.e9
        if upper is None:
            return None
        if lower <= upper:
            return lower, upper
        print('下限は上限以下にしてください。高さを入力し直します。', flush=True)


def main():
    parser = argparse.ArgumentParser(description='Foxglove用に後処理済み3D地図を配信')
    parser.add_argument('path', nargs='?', help='bag・出力ディレクトリ、またはPLYファイル')
    parser.add_argument('--directory', help='選択候補の検索先（既定: rosbag2_dataとmaps_waypoints/maps3d）')
    parser.add_argument('--period', type=float, default=3., help='再配信間隔（秒、最小0.5）')
    parser.add_argument('--z-min', type=float, help='表示するZの下限[m]（指定時はZ対話を省略）')
    parser.add_argument('--z-max', type=float, help='表示するZの上限[m]（指定時はZ対話を省略）。例: 2.0')
    parser.add_argument('--no-z-dialog', action='store_true', help='Z対話を省略。高さ未指定ならカットなし')
    args = parser.parse_args()
    z_dialog = args.z_min is None and args.z_max is None and not args.no_z_dialog
    args.z_min = -1.e9 if args.z_min is None else args.z_min
    args.z_max = 1.e9 if args.z_max is None else args.z_max
    if not all(math.isfinite(v) for v in (args.z_min, args.z_max)) or args.z_min > args.z_max:
        parser.error('--z-min <= --z-max の有限の高さを指定してください')
    try:
        value = args.path
        if not value:
            roots = args.directory or [Path.home() / 'rosbag2_data',
                                      Path(__file__).resolve().parents[2] / 'maps_waypoints' / 'maps3d']
            value = choose_numbered(lambda: map_candidates(roots), '表示する3D地図', roots)
            if value is None:
                return
        path = resolve_ply(value)
        if z_dialog:
            limits = choose_z_limits()
            if limits is None:
                return
            args.z_min, args.z_max = limits
    except (ValueError, OSError, EOFError) as error:
        parser.exit(1, f'{error}\n')
    print(f'表示: {path}\n'
          'Foxglove: foxglove コマンドを起動して ws://<このPCのIP>:8766 に接続\n'
          '3Dパネル: Fixed frame / Display frame=offline_map\n'
          '色付き: /offline_map/colored_points (Color mode=RGB)\n'
          '削除対象: /offline_map/removed_points (Color mode=RGB、赤色。重ねて比較)\n'
          '追加仕上げ: /offline_map/fragment_points (Color mode=RGB、オレンジ)\n'
          '通常: /offline_map/points（表示を切り替えて比較）\n'
          '移動軌跡: /offline_map/trajectory（nav_msgs/Path、地図と同じ座標系）\n'
          f'Zカット: {"なし" if (args.z_min, args.z_max) == (-1.e9, 1.e9) else f"{args.z_min:g} ～ {args.z_max:g} m"}（元地図は変更しません）\n'
          '起動中の高さ変更: ros2 param set /offline_3d_map_publisher z_max 2.0\n'
          '停止: Ctrl+C', flush=True)
    command = ['ros2', 'run', 'sirius_navigation', 'theta_ply_publisher', '--ros-args',
               '-r', '__node:=offline_3d_map_publisher',
               '-p', f'ply_file:={path}', '-p', 'topic:=/offline_map/colored_points',
               '-p', 'geometry_topic:=/offline_map/points', '-p', 'frame_id:=offline_map',
               '-p', 'removed_topic:=/offline_map/removed_points',
               '-p', 'fragment_topic:=/offline_map/fragment_points',
               '-p', 'trajectory_topic:=/offline_map/trajectory',
               '-p', f'z_min:={float(args.z_min)}', '-p', f'z_max:={float(args.z_max)}',
               '-p', 'use_sim_time:=false', '-p', f'period_sec:={max(.5, args.period)}']
    os.execvp(command[0], command)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
