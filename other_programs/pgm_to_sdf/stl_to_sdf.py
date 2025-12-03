#!/usr/bin/env python3
"""
STLファイルをGazebo用のSDFワールドファイルに変換するプログラム
"""

import argparse
import os


def create_sdf_world(stl_file: str, output_file: str = None, world_name: str = "map_world", use_relative_path: bool = True):
    """
    STLファイルを含むSDFワールドファイルを生成する
    
    Args:
        stl_file: STLファイルのパス
        output_file: 出力SDFファイルのパス（省略時は自動生成）
        world_name: ワールド名
        use_relative_path: Trueの場合、STLファイル名のみを使用（GZ_SIM_RESOURCE_PATH経由で解決）
    """
    
    # STLファイルのパスを取得
    stl_abs_path = os.path.abspath(stl_file)
    stl_filename = os.path.basename(stl_file)
    model_name = os.path.splitext(stl_filename)[0] + "_walls"
    
    # SDFで使用するURIを決定
    if use_relative_path:
        stl_uri = stl_filename  # ファイル名のみ（GZ_SIM_RESOURCE_PATH経由）
    else:
        stl_uri = stl_abs_path  # 絶対パス
    
    # 出力ファイル名を決定
    if output_file is None:
        base_name = os.path.splitext(stl_file)[0]
        output_file = f"{base_name}_world.sdf"
    
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
    
    print(f"✓ {output_file} を作成しました！")
    print(f"  STLファイル: {stl_filename}")
    print(f"  SDF内のURI: {stl_uri}")
    print(f"  ワールド名: {world_name}")
    if use_relative_path:
        print(f"\n注意: GZ_SIM_RESOURCE_PATHにSTLファイルのディレクトリを追加してください")
    print(f"\n実行コマンド:")
    print(f"  gz sim {output_file}")
    
    return output_file


def main():
    parser = argparse.ArgumentParser(
        description='STLファイルをGazebo用のSDFワールドファイルに変換'
    )
    parser.add_argument(
        'stl_file',
        nargs='?',
        default='1128sim.stl',
        help='入力STLファイル（デフォルト: 1128sim.stl）'
    )
    parser.add_argument(
        '-o', '--output',
        default=None,
        help='出力SDFファイル名（省略時は自動生成）'
    )
    parser.add_argument(
        '-n', '--name',
        default='map_world',
        help='ワールド名（デフォルト: map_world）'
    )
    parser.add_argument(
        '--absolute',
        action='store_true',
        help='STLファイルの絶対パスを使用（デフォルトは相対パス）'
    )
    
    args = parser.parse_args()
    
    # STLファイルのパスを解決
    stl_file = args.stl_file
    if not os.path.isabs(stl_file):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        stl_file = os.path.join(script_dir, stl_file)
    
    if not os.path.exists(stl_file):
        print(f"エラー: STLファイルが見つかりません: {stl_file}")
        return 1
    
    create_sdf_world(stl_file, args.output, args.name, use_relative_path=not args.absolute)
    return 0


if __name__ == "__main__":
    exit(main())
