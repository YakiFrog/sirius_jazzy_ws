#!/usr/bin/env python3
"""
Witmotion HWT905 IMUセンサー起動ファイル
sirius_navigationパッケージ用

使用例:
  # デフォルト設定で起動
  ros2 launch sirius_navigation witmotion_hwt905.launch.py

  # カスタム設定ファイルを指定
  ros2 launch sirius_navigation witmotion_hwt905.launch.py config_file:=/path/to/custom_config.yml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ========================================
    # Launch引数の定義
    # ========================================
    
    # デフォルトのconfig パスを取得
    # ワークスペースのparamsディレクトリを優先的に使用
    default_config = '/home/kotantu-desktop/sirius_jazzy_ws/params/wt905.yaml'
    
    # もしワークスペースのファイルがなければ、パッケージのデフォルトを試す
    if not os.path.exists(default_config):
        try:
            default_config = os.path.join(
                get_package_share_directory('witmotion_ros'),
                'config',
                'wt905.yml'
            )
        except:
            default_config = ''

    # 設定ファイルパスの引数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Witmotion ROSノードの設定ファイルへのパス'
    )

    # ========================================
    # Witmotion ROSノード
    # ========================================
    
    witmotion_node = Node(
        package='witmotion_ros',
        executable='witmotion_ros_node',
        name='witmotion',  # yamlファイルの名前空間と一致させる
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        # remappings=[
        #     # 必要に応じてトピックをリマップ
        #     # ('/imu', '/sensors/imu'),
        # ]
    )

    # ========================================
    # LaunchDescriptionの構築
    # ========================================
    
    return LaunchDescription([
        config_file_arg,
        witmotion_node,
    ])
