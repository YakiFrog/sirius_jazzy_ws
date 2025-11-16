#!/usr/bin/env python3

import os
import subprocess
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def convert_sdf_to_urdf(sdf_file_path, pkg_path):
    """SDFファイルをURDFに変換し、メッシュパスを修正"""
    # gz sdfコマンドでSDFを処理（モデル情報を抽出）
    result = subprocess.run(
        ['gz', 'sdf', '-p', sdf_file_path],
        capture_output=True,
        text=True,
        check=True
    )
    
    # 変換されたURDFを取得
    urdf_content = result.stdout
    
    # 相対パス（../urdf/meshes/）を絶対パスに置換
    urdf_content = urdf_content.replace(
        '../urdf/meshes/',
        f'file://{pkg_path}/urdf/meshes/'
    )
    
    # package:// 形式に変換（推奨）
    urdf_content = urdf_content.replace(
        f'file://{pkg_path}/urdf/meshes/',
        'package://sirius_description/urdf/meshes/'
    )
    
    return urdf_content

def generate_launch_description():
    # パッケージのパスを取得
    pkg_share = FindPackageShare('sirius_description')
    pkg_path = get_package_share_directory('sirius_description')
    
    # 環境SDFファイルのパスを設定
    world_sdf_file = PathJoinSubstitution([
        pkg_share,
        'worlds',
        'sirius_world.sdf'
    ])
    
    # ロボットSDFファイルのパスを設定
    robot_sdf_file = PathJoinSubstitution([
        pkg_share,
        'sdf',
        'sirius3.sdf'
    ])
    
    # ロボットSDFファイルの絶対パス（変換用）
    robot_sdf_file_abs = os.path.join(pkg_path, 'sdf', 'sirius3.sdf')
    
    # Rvizファイルのパスを設定
    rviz_config = PathJoinSubstitution([
        pkg_share,
        'rviz',
        'sirius_robot.rviz'
    ])
    
    # Launch引数の定義
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # 1. 環境（world）のGazeboシミュレーション起動
    gazebo_world = ExecuteProcess(
        cmd=['gz', 'sim', world_sdf_file],
        output='screen',
        shell=False
    )
    
    # 2. ロボットをスポーン（ros_gz_simのcreateノード使用）
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', robot_sdf_file,
            '-name', 'sirius3',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.0825',
            '-Y', '1.5708'  # 左に90度回転（ラジアンで指定、1.5708 ≒ π/2）
        ],
        output='screen'
    )
    
    # SDFファイルをURDFに変換
    robot_description_content = convert_sdf_to_urdf(robot_sdf_file_abs, pkg_path)
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description_content,
                'frame_prefix': 'sirius3/'
            }
        ],
        output='screen'
    )
    
    # TFのros_gz_bridge（EKF使用時はコメントアウト）
    # Note: EKFがIMU融合後のTF (odom → base_footprint) を配信するため、
    # GazeboのTFブリッジは不要（競合を避ける）
    # ロボット固定のTF (base_link → sensors) はrobot_state_publisherが配信
    # tf_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         '/model/sirius3/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V' # Gazebo <-> ROS2
    #     ],
    #     remappings=[
    #         ('/model/sirius3/tf', '/tf')
    #     ],
    #     output='screen'
    # )
    
    # Odometryのros_gz_bridge
    # Note: Odometryメッセージのframe_idには既に sirius3/ プレフィックスが付いています
    # 単一ロボットの場合は /odom でOK、複数ロボットの場合は /sirius3/odom 推奨
    # このトピックはEKFの入力として使用される
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/sirius3/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry' # Gazebo -> ROS2
        ],
        remappings=[
            ('/model/sirius3/odom', '/odom')
        ],
        output='screen'
    )
    
    # Twistのros_gz_bridge
    twist_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'], # ROS2 -> Gazebo
        output='screen'
    )
    
    # Joint Stateのros_gz_bridge
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model' # Gazebo <-> ROS2
        ],
        output='screen'
    )
    
    # LiDARのros_gz_bridge
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'], # Gazebo -> ROS2
        output='screen'
    )

    # LiDAR2のros_gz_bridge
    lidar2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/hokuyo_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'], # Gazebo -> ROS2
        output='screen'
    )
    
    # IMUのros_gz_bridge
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'], # Gazebo -> ROS2
        output='screen'
    )
    
    # Velodyneのros_gz_bridge
    velodyne_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/velodyne_points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'], # Gazebo -> ROS2
        remappings=[
            ('/velodyne_points/points', '/velodyne_points')
        ],
        output='screen'
    )

    # RealSenseカメラのros_gz_bridge
    # rgbd_cameraタイプは自動的にサブトピックを生成: /topic_name/image, /topic_name/depth_image, など
    realsense_image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/realsense/image@sensor_msgs/msg/Image[gz.msgs.Image'], # Gazebo -> ROS2
        output='screen'
    )
    
    realsense_depth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/realsense/depth_image@sensor_msgs/msg/Image[gz.msgs.Image'], # Gazebo -> ROS2
        output='screen'
    )
    
    realsense_points_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'], # Gazebo -> ROS2
        output='screen'
    )
    
    realsense_camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/realsense/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'], # Gazebo -> ROS2
        output='screen'
    )

    # Clock bridgeを追加
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], # Gazebo -> ROS2
        output='screen'
    )

    # Teleopキーボードコントロール
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'
    )
    
    # Rviz2の起動
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch引数
        use_sim_time_arg,
        
        # 1. 環境（ワールド）を最初に起動
        gazebo_world,

        # 2. 1秒後にclockブリッジを起動（最優先）
        TimerAction(
            period=1.0,
            actions=[clock_bridge]
        ),

        # 3. 3秒後にロボットをスポーン
        TimerAction(
            period=3.0,
            actions=[spawn_robot]
        ),

        # 4. 5秒後にブリッジ類とrobot_state_publisherを開始
        # Note: tf_bridgeはEKFがTFを配信するためコメントアウト
        TimerAction(
            period=5.0,
            actions=[
                # tf_bridge,  # EKF使用時は不要
                odom_bridge,
                twist_bridge,
                robot_state_publisher,
                joint_state_bridge,
                lidar_bridge,
                lidar2_bridge,
                imu_bridge,
                velodyne_bridge,
                realsense_image_bridge,
                realsense_depth_bridge,
                realsense_points_bridge,
                realsense_camera_info_bridge
            ]
        ),
        
        # 5. 7秒後にteleopを起動
        TimerAction(
            period=7.0,
            actions=[
                teleop_keyboard
            ]
        ),

        # 6. 8秒後にRVizを起動
        TimerAction(
            period=8.0,
            actions=[
                rviz2
            ]
        )
    ])
