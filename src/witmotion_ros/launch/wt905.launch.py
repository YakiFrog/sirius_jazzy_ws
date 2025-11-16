
#!/usr/bin/env python3
"""
Witmotion WT905 IMUセンサー起動ファイル

使用例:
  # デフォルト設定で起動
  ros2 launch witmotion_ros wt905.launch.py

  # カスタム設定ファイルを指定
  ros2 launch witmotion_ros wt905.launch.py config_file:=/path/to/custom_config.yml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # デフォルトのconfig パスを取得
    default_config = os.path.join(
        get_package_share_directory('witmotion_ros'),
        'config',
        'wt905.yml'
    )

    # 設定ファイルパスの引数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Witmotion WT905の設定ファイルへのパス'
    )

    # Witmotion ROSノード
    witmotion_node = Node(
        package='witmotion_ros',
        executable='witmotion_ros_node',
        name='witmotion',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_file_arg,
        witmotion_node,
    ])