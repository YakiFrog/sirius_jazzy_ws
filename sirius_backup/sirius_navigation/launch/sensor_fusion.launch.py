#!/usr/bin/env python3
"""
IMUとオドメトリのセンサフュージョン起動ファイル
robot_localizationパッケージを使用してEKF(拡張カルマンフィルタ)で融合
RViz2の/initialposeをEKFに転送するノードも起動

オプション:
- start_hwt905: HWT905 IMUを自動起動（デフォルト: false）
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ========================================
    # Launch引数の定義
    # ========================================
    
    # HWT905 IMU起動フラグ
    start_hwt905_arg = DeclareLaunchArgument(
        'start_hwt905',
        default_value='false',
        description='HWT905 IMUを自動起動するかどうか（true/false）'
    )
    
    # HWT905のシリアルポート
    hwt905_port_arg = DeclareLaunchArgument(
        'hwt905_port',
        default_value='/dev/ttyUSB0',
        description='HWT905が接続されているシリアルポート'
    )
    
    # HWT905のボーレート
    hwt905_baud_arg = DeclareLaunchArgument(
        'hwt905_baud',
        default_value='115200',
        description='HWT905のボーレート'
    )
    
    # Launch設定の取得
    start_hwt905 = LaunchConfiguration('start_hwt905')
    hwt905_port = LaunchConfiguration('hwt905_port')
    hwt905_baud = LaunchConfiguration('hwt905_baud')
    
    # ========================================
    # パラメータファイルのパス
    # ========================================
    
    # パラメータファイルのパス（絶対パス）
    params_file = os.path.join(
        os.path.expanduser('~'),
        'sirius_jazzy_ws',
        'params',
        'ekf_fusion.yaml'
    )
    
    # sirius_navigationパッケージのパス
    sirius_nav_share = get_package_share_directory('sirius_navigation')
    
    # EKFノード（詳細ログとQoS互換性の設定）
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            params_file,
            {
                # QoS設定の追加パラメータ
                'debug': False,
                'debug_out_file': '/tmp/ekf_debug.txt'
            }
        ],
        remappings=[
            ('/odometry/filtered', '/odom/filtered')
        ],
        # ログレベルを詳細に設定
        arguments=['--ros-args', '--log-level', 'warn']  # info→warnでログ削減
    )
    
    # ========================================
    # HWT905 IMUノード（オプション）
    # ========================================
    
    hwt905_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                sirius_nav_share,
                'launch',
                'witmotion_hwt905.launch.py'
            ])
        ]),
        launch_arguments={
            'port': hwt905_port,
            'baud_rate': hwt905_baud,
            'frequency': '100.0',
            'log_level': 'info',
        }.items(),
        condition=IfCondition(start_hwt905)
    )
    
    # ========================================
    # EKF Pose Initializer
    # ========================================
    
    # EKF Pose Initializer（RViz2の/initialposeをEKFに転送）
    # EKFの起動を待つために2秒遅延
    ekf_pose_initializer = TimerAction(
        period=2.0,  # 2秒待機
        actions=[
            Node(
                package='sirius_navigation',
                executable='ekf_pose_initializer',
                name='ekf_pose_initializer',
                output='screen',
                arguments=['--ros-args', '--log-level', 'info']
            )
        ]
    )
    
    # ========================================
    # LaunchDescriptionの構築
    # ========================================
    
    return LaunchDescription([
        # Launch引数
        start_hwt905_arg,
        hwt905_port_arg,
        hwt905_baud_arg,
        
        # ノード（HWT905はオプション）
        hwt905_launch,
        ekf_node,
        ekf_pose_initializer
    ])