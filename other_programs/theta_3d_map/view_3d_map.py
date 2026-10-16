#!/usr/bin/env python3
"""Select a generated PLY and publish XYZ/RGB maps for Foxglove."""
import argparse
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


def main():
    parser = argparse.ArgumentParser(description='Foxglove用に後処理済み3D地図を配信')
    parser.add_argument('path', nargs='?', help='bag・出力ディレクトリ、またはPLYファイル')
    parser.add_argument('--directory', help='選択候補の検索先（既定: rosbag2_dataとmaps_waypoints/maps3d）')
    parser.add_argument('--period', type=float, default=3., help='再配信間隔（秒、最小0.5）')
    args = parser.parse_args()
    try:
        value = args.path
        if not value:
            roots = args.directory or [Path.home() / 'rosbag2_data',
                                      Path(__file__).resolve().parents[2] / 'maps_waypoints' / 'maps3d']
            value = choose_numbered(lambda: map_candidates(roots), '表示する3D地図', roots)
            if value is None:
                return
        path = resolve_ply(value)
    except (ValueError, OSError, EOFError) as error:
        parser.exit(1, f'{error}\n')
    print(f'表示: {path}\n'
          'Foxglove: foxglove コマンドを起動して ws://<このPCのIP>:8766 に接続\n'
          '3Dパネル: Fixed frame / Display frame=offline_map\n'
          '色付き: /offline_map/colored_points (Color mode=RGB)\n'
          '通常: /offline_map/points（表示を切り替えて比較）\n'
          '停止: Ctrl+C', flush=True)
    command = ['ros2', 'run', 'sirius_navigation', 'theta_ply_publisher', '--ros-args',
               '-r', '__node:=offline_3d_map_publisher',
               '-p', f'ply_file:={path}', '-p', 'topic:=/offline_map/colored_points',
               '-p', 'geometry_topic:=/offline_map/points', '-p', 'frame_id:=offline_map',
               '-p', 'use_sim_time:=false', '-p', f'period_sec:={max(.5, args.period)}']
    os.execvp(command[0], command)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
