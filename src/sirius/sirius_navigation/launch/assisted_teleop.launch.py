#!/usr/bin/env python3
"""
Sirius Assisted Teleop Launchファイル

このlaunchファイルは以下を起動します:
1. Assisted Teleopノード - 障害物回避をサポートする操縦支援
2. Teleop Twist Keyboard - 別ウィンドウでキーボード操縦

使用方法:
    ros2 launch sirius_navigation assisted_teleop.launch.py

注意:
    - Nav2が既に起動していることが前提です
    - シミュレーションまたは実機のロボットが起動していることを確認してください
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    """Launch Descriptionを生成"""
    
    # Assisted Teleopノード
    assisted_teleop_node = Node(
        package='sirius_navigation',
        executable='assisted_teleop',
        name='sirius_assisted_teleop',
        output='screen',
        parameters=[
            {'time_allowance': 600}  # 10分間の制限時間
        ]
    )
    
    # Teleop Twist Keyboardノード（別ウィンドウで起動）
    # xterm -e を使って別ターミナルで起動
    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[
            # cmd_velトピックはそのまま使用
            # Assisted Teleopがこれを監視して、安全なコマンドに変換します
        ]
    )
    
    return LaunchDescription([
        # まずAssisted Teleopを起動
        assisted_teleop_node,
        
        # 2秒後にTeleopキーボードを起動
        TimerAction(
            period=2.0,
            actions=[teleop_keyboard_node]
        )
    ])
