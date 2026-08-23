import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('sirius_zed_recorder')
    default_params = os.path.join(
        package_share, 'config', 'zed_offline_recorder.yaml'
    )
    params_file = LaunchConfiguration('params_file')
    record_fps = LaunchConfiguration('record_fps')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='CUDA-free ZED recorder parameter file',
        ),
        DeclareLaunchArgument(
            'record_fps',
            default_value='8.0',
            description='ROS/rosbag output FPS (camera capture remains at configured fps)',
        ),
        Node(
            package='sirius_zed_recorder',
            executable='zed_stereo_publisher',
            name='zed_stereo_publisher',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': False,
                    'record_fps': ParameterValue(record_fps, value_type=float),
                },
            ],
        ),
    ])
