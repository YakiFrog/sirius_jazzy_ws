#!/usr/bin/env python3
"""
PGMマップからSTL/SDFを生成する対話式プログラム
maps_waypointsフォルダ内のマップを選択して変換
"""

import os
import sys
import glob
import numpy as np
from PIL import Image
import trimesh
from trimesh.creation import box
from trimesh.scene import Scene
import yaml


# パス設定
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
MAPS_DIR = os.path.join(WORKSPACE_DIR, "maps_waypoints", "maps")
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")

# 壁の高さ（固定）
WALL_HEIGHT = 1.5  # メートル


def load_map_yaml(yaml_file: str) -> dict:
    """YAMLファイルからマップ設定を読み込む"""
    with open(yaml_file, 'r') as f:
        map_config = yaml.safe_load(f)
    return map_config


def pgm_to_stl(yaml_file: str, output_file: str, wall_height: float = 1.0) -> str:
    """PGMマップをSTLに変換する"""
    map_config = load_map_yaml(yaml_file)
    yaml_dir = os.path.dirname(yaml_file)
    pgm_file = os.path.join(yaml_dir, map_config['image'])
    
    resolution = map_config.get('resolution', 0.05)
    origin = map_config.get('origin', [0.0, 0.0, 0.0])
    occupied_thresh = map_config.get('occupied_thresh', 0.65)
    negate = map_config.get('negate', 0)
    
    print(f"\n  マップ設定:")
    print(f"    解像度: {resolution} m/pixel")
    print(f"    原点: {origin}")
    print(f"    壁の高さ: {wall_height} m")
    
    image = Image.open(pgm_file).convert("L")
    image = np.array(image, dtype=np.uint8)
    height, width = image.shape
    
    print(f"    画像サイズ: {width} x {height} pixels")
    
    if negate:
        image = 255 - image
    
    black_threshold = int(255 * (1 - occupied_thresh))
    
    scene = Scene()
    wall_count = 0
    origin_x = origin[0]
    origin_y = origin[1]
    
    print("  STL生成中...", end="", flush=True)
    
    for y in range(height):
        for x in range(width):
            if image[y, x] < black_threshold:
                px = origin_x + (x + 0.5) * resolution
                py = origin_y + (height - y - 0.5) * resolution
                pz = wall_height / 2
                
                wall = box(extents=[resolution, resolution, wall_height])
                wall.apply_translation([px, py, pz])
                scene.add_geometry(wall)
                wall_count += 1
    
    scene.export(output_file)
    print(f" 完了！（壁ブロック数: {wall_count}）")
    
    return output_file


def create_sdf_world(stl_file: str, output_file: str, world_name: str = "map_world") -> str:
    """STLファイルを含むSDFワールドファイルを生成する"""
    stl_filename = os.path.basename(stl_file)
    model_name = os.path.splitext(stl_filename)[0] + "_walls"
    stl_uri = stl_filename  # 相対パス（GZ_SIM_RESOURCE_PATH経由）
    
    sdf_content = f'''<?xml version="1.0"?>
<sdf version='1.10'>
  <world name='{world_name}'>
    <physics name='1ms' type='ignored'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/>
    <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
    <plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'/>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>5.5645e-06 2.28758e-05 -4.23884e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- 地面 -->
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 0 0</pose>
        <inertial>
          <pose>0 0 0 0 0 0</pose>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
        <enable_wind>false</enable_wind>
      </link>
      <pose>0 0 0 0 0 0</pose>
      <self_collide>false</self_collide>
    </model>

    <!-- STLマップの壁 -->
    <model name='{model_name}'>
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>{stl_uri}</uri>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>{stl_uri}</uri>
            </mesh>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
            <specular>0.5 0.5 0.5 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 0 0</pose>
        <inertial>
          <pose>0 0 0 0 0 0</pose>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
        <enable_wind>false</enable_wind>
      </link>
      <self_collide>false</self_collide>
    </model>

    <!-- 太陽光 -->
    <light name='sun' type='directional'>
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>-0.5 0.1 -0.9</direction>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <linear>0.01</linear>
        <constant>0.9</constant>
        <quadratic>0.001</quadratic>
      </attenuation>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>

  </world>
</sdf>
'''
    
    with open(output_file, 'w') as f:
        f.write(sdf_content)
    
    print(f"  SDF生成完了！")
    
    return output_file


def get_available_maps() -> list:
    """利用可能なマップファイル（YAML）を取得"""
    yaml_files = glob.glob(os.path.join(MAPS_DIR, "*.yaml"))
    maps = []
    for yaml_file in sorted(yaml_files):
        base_name = os.path.splitext(os.path.basename(yaml_file))[0]
        pgm_file = os.path.join(MAPS_DIR, f"{base_name}.pgm")
        if os.path.exists(pgm_file):
            maps.append({
                'name': base_name,
                'yaml': yaml_file,
                'pgm': pgm_file
            })
    return maps


def display_menu(maps: list):
    """マップ選択メニューを表示"""
    print("\n" + "=" * 50)
    print("  PGM → STL → SDF 変換ツール")
    print("=" * 50)
    print(f"\n📁 マップフォルダ: {MAPS_DIR}")
    print(f"📁 出力フォルダ: {OUTPUT_DIR}")
    print("\n利用可能なマップ:")
    print("-" * 50)
    for i, m in enumerate(maps):
        print(f"  [{i + 1}] {m['name']}")
    print("-" * 50)
    print("  [a] すべてのマップを変換")
    print("  [q] 終了")
    print("-" * 50)


def convert_map(map_info: dict):
    """マップをSTLとSDFに変換"""
    name = map_info['name']
    yaml_file = map_info['yaml']
    
    print(f"\n🔄 変換中: {name}")
    
    # 出力ファイルパス
    stl_file = os.path.join(OUTPUT_DIR, f"{name}.stl")
    sdf_file = os.path.join(OUTPUT_DIR, f"{name}_world.sdf")
    
    # PGM → STL
    pgm_to_stl(yaml_file, stl_file, WALL_HEIGHT)
    print(f"  ✓ STL: {stl_file}")
    
    # STL → SDF
    create_sdf_world(stl_file, sdf_file, f"{name}_world")
    print(f"  ✓ SDF: {sdf_file}")
    
    return stl_file, sdf_file


def main():
    # 出力ディレクトリを作成
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # 利用可能なマップを取得
    maps = get_available_maps()
    
    if not maps:
        print(f"エラー: マップが見つかりません: {MAPS_DIR}")
        return 1
    
    while True:
        display_menu(maps)
        
        # 選択を入力
        choice = input("\n選択してください: ").strip().lower()
        
        if choice == 'q':
            print("\n終了します。")
            break
        elif choice == 'a':
            # すべてのマップを変換
            print(f"\n{'=' * 50}")
            print(f"すべてのマップを変換します（壁の高さ: {WALL_HEIGHT}m）")
            print(f"{'=' * 50}")
            
            for m in maps:
                convert_map(m)
            
            print(f"\n{'=' * 50}")
            print(f"✅ すべての変換が完了しました！")
            print(f"   出力先: {OUTPUT_DIR}")
            print(f"{'=' * 50}")
            
        else:
            try:
                idx = int(choice) - 1
                if 0 <= idx < len(maps):
                    convert_map(maps[idx])
                    print(f"\n✅ 変換完了！")
                else:
                    print("無効な選択です。")
            except ValueError:
                print("無効な入力です。")
        
        # 続けるか確認
        cont = input("\n続けますか？ [Y/n]: ").strip().lower()
        if cont == 'n':
            print("\n終了します。")
            break
    
    # Gazeboでの実行方法を表示
    print("\n" + "=" * 50)
    print("📌 Gazeboでの実行方法:")
    print("=" * 50)
    print(f"export GZ_SIM_RESOURCE_PATH={OUTPUT_DIR}:$GZ_SIM_RESOURCE_PATH")
    print(f"gz sim {OUTPUT_DIR}/<マップ名>_world.sdf")
    print("=" * 50)
    
    return 0


if __name__ == "__main__":
    exit(main())
