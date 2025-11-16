#!/usr/bin/env python3
"""
Sirius シミュレーション起動ファイル（UI付き）
UIで設定を選択してから起動するバージョン
"""

import os
import sys
import subprocess
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

# launchディレクトリをPythonパスに追加
launch_dir = Path(__file__).parent
sys.path.insert(0, str(launch_dir))

# UIモジュールをインポート
from launch_config_ui import show_launch_config_ui


def convert_sdf_to_urdf(sdf_file_path, pkg_path):
    """SDFファイルをURDFに変換し、メッシュパスを修正"""
    result = subprocess.run(
        ['gz', 'sdf', '-p', sdf_file_path],
        capture_output=True,
        text=True,
        check=True
    )
    
    urdf_content = result.stdout
    
    # STLメッシュのパス変換
    urdf_content = urdf_content.replace(
        '../urdf/meshes/',
        f'file://{pkg_path}/urdf/meshes/'
    )
    
    urdf_content = urdf_content.replace(
        f'file://{pkg_path}/urdf/meshes/',
        'package://sirius_description/urdf/meshes/'
    )
    
    # DAEメッシュのパス変換
    urdf_content = urdf_content.replace(
        '../dae/',
        f'file://{pkg_path}/dae/'
    )
    
    urdf_content = urdf_content.replace(
        f'file://{pkg_path}/dae/',
        'package://sirius_description/dae/'
    )
    
    return urdf_content


def generate_launch_description_with_config(context, *args, **kwargs):
    """設定に基づいてLaunchDescriptionを生成"""
    # UIから設定を取得
    config = show_launch_config_ui()
    
    if config is None:
        print("起動がキャンセルされました")
        return []
    
    # パッケージのパスを取得
    pkg_share = FindPackageShare('sirius_description')
    pkg_path = get_package_share_directory('sirius_description')
    
    # 選択されたワールドファイルのパスを設定
    world_sdf_file = PathJoinSubstitution([
        pkg_share,
        'worlds',
        config['world_file']
    ])
    
    # ロボットSDFファイルのパスを設定
    robot_sdf_file = PathJoinSubstitution([
        pkg_share,
        'sdf',
        'sirius3.sdf'
    ])
    
    robot_sdf_file_abs = os.path.join(pkg_path, 'sdf', 'sirius3.sdf')
    
    # Rvizファイルのパスを設定
    rviz_config = PathJoinSubstitution([
        pkg_share,
        'rviz',
        'sirius_robot.rviz'
    ])
    
    actions = []
    
    # 1. 環境（world）のGazeboシミュレーション起動（常に起動）
    gazebo_world = ExecuteProcess(
        cmd=['gz', 'sim', world_sdf_file],
        output='screen',
        shell=False
    )
    actions.append(gazebo_world)
    
    # 2. Clock Bridge（選択された場合、最優先）
    if config['clock_bridge']:
        clock_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        )
        actions.append(TimerAction(period=1.0, actions=[clock_bridge]))
    
    # 3. ロボットをスポーン
    timer_actions_3sec = []
    if config['spawn_robot']:
        spawn_robot = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', robot_sdf_file,
                '-name', 'sirius3',
                '-x', '0.0',
                '-y', '0.0', 
                '-z', '0.0825',
                '-Y', '1.5708'
            ],
            output='screen'
        )
        timer_actions_3sec.append(spawn_robot)
    
    if timer_actions_3sec:
        actions.append(TimerAction(period=3.0, actions=timer_actions_3sec))
    
    # 4. ブリッジ類とrobot_state_publisher（5秒後）
    timer_actions_5sec = []
    
    # TF Bridge（EKF使用時は無効化推奨）
    # Note: EKFがIMU+Odomを融合してTF (odom→base_footprint) を配信する場合、
    # このブリッジは競合するため無効化する必要があります
    if config['tf_bridge']:
        tf_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/sirius3/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
            remappings=[('/model/sirius3/tf', '/tf')],
            output='screen'
        )
        timer_actions_5sec.append(tf_bridge)
    
    if config['odom_bridge']:
        odom_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/sirius3/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
            remappings=[('/model/sirius3/odom', '/odom')],
            output='screen'
        )
        timer_actions_5sec.append(odom_bridge)
    
    if config['twist_bridge']:
        twist_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
            output='screen'
        )
        timer_actions_5sec.append(twist_bridge)
    
    if config['robot_state_publisher']:
        robot_description_content = convert_sdf_to_urdf(robot_sdf_file_abs, pkg_path)
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {
                    'use_sim_time': True,
                    'robot_description': robot_description_content,
                    'frame_prefix': 'sirius3/'
                }
            ],
            output='screen'
        )
        timer_actions_5sec.append(robot_state_publisher)
    
    if config['joint_state_bridge']:
        joint_state_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'],
            output='screen'
        )
        timer_actions_5sec.append(joint_state_bridge)
    
    if config['lidar_bridge']:
        lidar_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
            output='screen'
        )
        timer_actions_5sec.append(lidar_bridge)
    
    if config['lidar2_bridge']:
        lidar2_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/hokuyo_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
            output='screen'
        )
        timer_actions_5sec.append(lidar2_bridge)
    
    if config['imu_bridge']:
        imu_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
            output='screen'
        )
        timer_actions_5sec.append(imu_bridge)
    
    if config['velodyne_bridge']:
        velodyne_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/velodyne_points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
            remappings=[('/velodyne_points/points', '/velodyne_points')],
            output='screen'
        )
        timer_actions_5sec.append(velodyne_bridge)
    
    if config.get('realsense_bridge', True):  # デフォルトで有効
        # RealSense RGB画像ブリッジ（rgbd_cameraは/topic_name/imageを生成）
        realsense_image_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/realsense/image@sensor_msgs/msg/Image[gz.msgs.Image'],
            output='screen'
        )
        timer_actions_5sec.append(realsense_image_bridge)
        
        # RealSense深度画像ブリッジ
        realsense_depth_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/realsense/depth_image@sensor_msgs/msg/Image[gz.msgs.Image'],
            output='screen'
        )
        timer_actions_5sec.append(realsense_depth_bridge)
        
        # RealSenseポイントクラウドブリッジ
        realsense_points_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
            output='screen'
        )
        timer_actions_5sec.append(realsense_points_bridge)
        
        # RealSenseカメラ情報ブリッジ
        realsense_camera_info_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/realsense/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
            output='screen'
        )
        timer_actions_5sec.append(realsense_camera_info_bridge)
    
    if timer_actions_5sec:
        actions.append(TimerAction(period=5.0, actions=timer_actions_5sec))
    
    # 5. Teleop（7秒後）
    if config['teleop']:
        teleop_keyboard = Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e'
        )
        actions.append(TimerAction(period=7.0, actions=[teleop_keyboard]))
    
    # 6. RViz（8秒後）
    if config['rviz']:
        rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
        actions.append(TimerAction(period=8.0, actions=[rviz2]))
    
    return actions


def generate_launch_description():
    """Launch Descriptionを生成"""
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        OpaqueFunction(function=generate_launch_description_with_config)
    ])
