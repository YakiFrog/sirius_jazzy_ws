"""FAST-LIO2 + フレームブリッジ（camera_init/body -> sirius3/odom/base_footprint）。

FAST-LIO2は camera_init->body のTFと /Odometry を配信する。
アプリ/Nav2 が使う sirius3/odom -> sirius3/base_footprint に接続するため、
静的TFを2つ追加する:
  sirius3/odom -> camera_init   (恒等)
  body         -> sirius3/base_footprint (恒等)
※ LiDAR-IMU 外部校正(extrinsic_T/R)と robot TF の整合は Phase 0 で調整する。
"""
import os
from pathlib import Path
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, LogInfo
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from sirius_description.sensor_mounts import load_mount, fastlio_rotation


def generate_launch_description():
    sirius = get_package_share_directory('sirius_navigation')
    fast_lio_share = get_package_share_directory('fast_lio')

    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_height = LaunchConfiguration('imu_height')

    args = [
        DeclareLaunchArgument(
            'config_path',
            default_value=os.path.join(sirius, 'config')),
        DeclareLaunchArgument('config_file', default_value='fast_lio_sirius.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization_mode', default_value='false',
                              description='Disable accumulated map/path publication for localization'),
        DeclareLaunchArgument('mount_config', default_value='',
                              description='Shared real rig calibration; blank or none preserves the selected YAML'),
        DeclareLaunchArgument('imu_height', default_value='0.65',
                              description='base_footprintからIMUまでの高さ[m]'),
    ]

    bridge_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='lio_bridge_odom',
        arguments=['--frame-id', 'sirius3/odom', '--child-frame-id', 'camera_init'],
    )
    bridge_base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='lio_bridge_base',
        arguments=['--frame-id', 'body', '--child-frame-id', 'sirius3/base_footprint',
                   '--z', PythonExpression(['-', imu_height])],
    )

    def calibrated_lio(context):
        path, filename = config_path.perform(context), config_file.perform(context)
        mount_path = LaunchConfiguration('mount_config').perform(context)
        localization = LaunchConfiguration('localization_mode').perform(context).lower() in ('true', '1')
        calibrated = bool(mount_path and mount_path.lower() != 'none')
        actions = []
        if calibrated or localization:
            config = yaml.safe_load((Path(path)/filename).read_text())
            params = config['/**']['ros__parameters']
            if calibrated:
                if params['common']['lid_topic'] == '/velodyne_points_leveled':
                    raise ValueError('Calibrated FAST-LIO requires raw LiDAR points, not leveled points')
                mount = load_mount(mount_path)
                params['mapping']['extrinsic_R'] = fastlio_rotation(params, mount)
                params['mapping']['extrinsic_est_en'] = False
                actions.append(LogInfo(msg=f'FAST-LIO real rig calibration: {mount_path}; rpy={mount["rig_rpy_degrees"]} deg'))
            if localization:
                params.setdefault('publish', {}).update(map_en=False, path_en=False)
                params['performance_diagnostics_enabled'] = True
                actions.append(LogInfo(msg='FAST-LIO localization mode: accumulated map/path publication disabled'))
            with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='sirius_mount_', delete=False) as f:
                yaml.safe_dump(config, f, sort_keys=False)
                resolved = Path(f.name)
            actions += [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
                            function=lambda _: resolved.unlink(missing_ok=True))]))]
            path, filename = str(resolved.parent), resolved.name
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')),
            launch_arguments={'config_path': path, 'config_file': filename,
                              'use_sim_time': use_sim_time, 'rviz': 'false'}.items()))
        return actions

    lio = OpaqueFunction(function=calibrated_lio)

    return LaunchDescription(args + [bridge_odom, bridge_base, lio])
