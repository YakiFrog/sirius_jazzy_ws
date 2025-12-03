#!/usr/bin/env python3
"""
PGMマップファイルとYAMLファイルを読み込んでSTLを生成するプログラム
ROS2のマップ形式に対応
"""

import numpy as np
from PIL import Image
import trimesh
from trimesh.creation import box
from trimesh.scene import Scene
import yaml
import argparse
import os


def load_map_yaml(yaml_file: str) -> dict:
    """YAMLファイルからマップ設定を読み込む"""
    with open(yaml_file, 'r') as f:
        map_config = yaml.safe_load(f)
    return map_config


def pgm_to_stl(yaml_file: str, output_file: str = None, wall_height: float = 1.0):
    """
    PGMマップをSTLに変換する
    
    Args:
        yaml_file: YAMLファイルのパス
        output_file: 出力STLファイルのパス（省略時は自動生成）
        wall_height: 壁の高さ（メートル）
    """
    # YAMLファイルを読み込み
    map_config = load_map_yaml(yaml_file)
    
    # YAMLファイルのディレクトリを基準にPGMファイルのパスを解決
    yaml_dir = os.path.dirname(yaml_file)
    pgm_file = os.path.join(yaml_dir, map_config['image'])
    
    # パラメータ取得
    resolution = map_config.get('resolution', 0.05)
    origin = map_config.get('origin', [0.0, 0.0, 0.0])
    occupied_thresh = map_config.get('occupied_thresh', 0.65)
    negate = map_config.get('negate', 0)
    
    print(f"マップ設定:")
    print(f"  画像ファイル: {pgm_file}")
    print(f"  解像度: {resolution} m/pixel")
    print(f"  原点: {origin}")
    print(f"  占有閾値: {occupied_thresh}")
    print(f"  壁の高さ: {wall_height} m")
    
    # 画像を読み込み
    image = Image.open(pgm_file).convert("L")
    image = np.array(image, dtype=np.uint8)
    height, width = image.shape
    
    print(f"  画像サイズ: {width} x {height} pixels")
    
    # negateが1の場合は画像を反転
    if negate:
        image = 255 - image
    
    # 占有閾値をピクセル値に変換（0-255）
    # 白(255)が空き、黒(0)が占有
    # occupied_threshは確率なので、255 * (1 - occupied_thresh)以下が壁
    black_threshold = int(255 * (1 - occupied_thresh))
    
    print(f"  壁判定閾値: {black_threshold}")
    
    scene = Scene()
    wall_count = 0
    
    # 原点座標
    origin_x = origin[0]
    origin_y = origin[1]
    
    print("STL生成中...")
    
    for y in range(height):
        for x in range(width):
            if image[y, x] < black_threshold:
                # ピクセル座標をワールド座標に変換
                # ROS2マップの原点は左下、画像の原点は左上
                px = origin_x + (x + 0.5) * resolution
                py = origin_y + (height - y - 0.5) * resolution
                pz = wall_height / 2
                
                wall = box(extents=[resolution, resolution, wall_height])
                wall.apply_translation([px, py, pz])
                scene.add_geometry(wall)
                wall_count += 1
    
    print(f"  壁ブロック数: {wall_count}")
    
    # 出力ファイル名を決定
    if output_file is None:
        base_name = os.path.splitext(map_config['image'])[0]
        output_file = os.path.join(yaml_dir, f"{base_name}.stl")
    
    # STLをエクスポート
    scene.export(output_file)
    print(f"✓ {output_file} を作成しました！")
    
    return output_file


def main():
    parser = argparse.ArgumentParser(
        description='PGMマップファイルをSTLに変換'
    )
    parser.add_argument(
        'yaml_file',
        nargs='?',
        default='1128sim.yaml',
        help='マップのYAMLファイル（デフォルト: 1128sim.yaml）'
    )
    parser.add_argument(
        '-o', '--output',
        default=None,
        help='出力STLファイル名（省略時は自動生成）'
    )
    parser.add_argument(
        '--height',
        type=float,
        default=1.0,
        help='壁の高さ（メートル、デフォルト: 1.0）'
    )
    
    args = parser.parse_args()
    
    # YAMLファイルのパスを解決
    yaml_file = args.yaml_file
    if not os.path.isabs(yaml_file):
        # スクリプトのディレクトリを基準にする
        script_dir = os.path.dirname(os.path.abspath(__file__))
        yaml_file = os.path.join(script_dir, yaml_file)
    
    if not os.path.exists(yaml_file):
        print(f"エラー: YAMLファイルが見つかりません: {yaml_file}")
        return 1
    
    pgm_to_stl(yaml_file, args.output, args.height)
    return 0


if __name__ == "__main__":
    exit(main())
