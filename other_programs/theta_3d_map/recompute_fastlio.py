#!/usr/bin/env python3
"""Recompute FastLIO in an isolated local ROS domain, preserving raw sensor stamps."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import fcntl

import numpy as np
import yaml

from mapping_fusion import read_static_calibration, invert
from offline_3d_mapping import reader


def select_extrinsics(params, imu_lidar, source):
    """Honor explicit calibration candidates instead of silently replacing them."""
    if source == 'recorded-tf':
        params['mapping'].update(extrinsic_R=imu_lidar[:3, :3].reshape(-1).tolist(),
                                 extrinsic_T=imu_lidar[:3, 3].tolist())
    rotation = np.asarray(params['mapping']['extrinsic_R'], dtype=float).reshape(3, 3)
    translation = np.asarray(params['mapping']['extrinsic_T'], dtype=float)
    if (translation.shape != (3,) or not np.isfinite(translation).all()
            or not np.isfinite(rotation).all()
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1., atol=1e-6)):
        raise ValueError('Extrinsics must contain a proper rotation and finite XYZ translation')
    params['mapping']['extrinsic_est_en'] = False


def run(args):
    # Set before rclpy.init; never publish replayed robot inputs into the live domain.
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['OMP_NUM_THREADS'] = '2'
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import PointCloud2, Imu
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    import rosbag2_py
    import rclpy
    bag, output = Path(args.bag).expanduser().resolve(), Path(args.output).expanduser().resolve()
    if output.exists():
        raise ValueError(f'Output already exists: {output}')
    config = yaml.safe_load((Path(args.config).expanduser() if args.config else bag/'capture/calibration.yaml').read_text())
    params = config['/**']['ros__parameters']
    lidar, imu, _, _ = read_static_calibration(bag, reader, 'sirius3/base_footprint', '/velodyne_points', '/imu')
    if lidar is None or imu is None:
        raise ValueError('Recorded static LiDAR and IMU mounts are required')
    imu_lidar = invert(imu) @ lidar
    select_extrinsics(params, imu_lidar, args.extrinsics)
    wheel_defaults = {'enabled': args.wheel == 'on', 'topic': '/odom',
                       'child_frame': 'sirius3/base_footprint', 'max_age': .2,
                       'velocity_std': .05, 'yaw_rate_std': .05,
                       'imu_to_base_translation': invert(imu)[:3, 3].tolist(),
                       'imu_from_base_rotation': invert(imu)[:3, :3].reshape(-1).tolist()}
    if args.extrinsics == 'config':
        wheel_defaults.update(params.get('wheel', {}))
    wheel_defaults['enabled'] = args.wheel == 'on'
    params['wheel'] = wheel_defaults
    params['publish'] = {'path_en': False, 'scan_publish_en': args.capture_cloud_period > 0,
                         'scan_bodyframe_pub_en': args.capture_cloud_period > 0,
                         'map_en': False, 'dense_publish_en': False}
    params['pcd_save'] = {'pcd_save_en': False, 'interval': -1}
    params['runtime_pos_log_enable'] = False
    params['preprocess']['normalize_point_time'] = args.point_time == 'normalized'
    params['use_sim_time'] = False
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    writer.create_topic(rosbag2_py.TopicMetadata(id=0, name='/Odometry', type='nav_msgs/msg/Odometry', serialization_format='cdr'))
    if args.capture_cloud_period > 0:
        writer.create_topic(rosbag2_py.TopicMetadata(id=1, name='/cloud_registered_body', type='sensor_msgs/msg/PointCloud2', serialization_format='cdr'))
    capture = output/'capture'
    capture.mkdir()
    (output/'.mapping-diagnostic').write_text('FastLIO pose sidecar; pass with --lio-bag, not as the raw sensor bag.\n')
    (output/'Log').mkdir()
    params['log_directory'] = str(output/'Log')
    (capture/'calibration.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    (capture/'fastlio_runtime.json').write_text(json.dumps({'wheel_enabled': args.wheel == 'on',
        'source': 'isolated offline recomputation', 'source_bag': str(bag), 'domain': args.domain,
        'point_time': args.point_time, 'extrinsics_source': args.extrinsics,
        'imu_accel_bias_subtracted_m_s2': args.imu_accel_bias,
        'capture_cloud_period_s': args.capture_cloud_period}, indent=2)+'\n')
    rclpy.init()
    node = rclpy.create_node('offline_fastlio_replay')
    types = {'/velodyne_points': PointCloud2, '/imu': Imu, '/odom': Odometry}
    publishers = {name: node.create_publisher(typ, name, 1000)
                  for name, typ in types.items()}
    written = 0
    def save(msg):
        nonlocal written
        ns = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        writer.write('/Odometry', serialize_message(msg), ns)
        written += 1
    subscription = node.create_subscription(Odometry, '/Odometry', save, 1000)
    last_cloud_ns = -10**18
    def save_cloud(msg):
        nonlocal last_cloud_ns
        ns = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if ns-last_cloud_ns >= args.capture_cloud_period*1e9:
            writer.write('/cloud_registered_body', serialize_message(msg), ns)
            last_cloud_ns = ns
    cloud_subscription = node.create_subscription(PointCloud2, '/cloud_registered_body', save_cloud, 10) if args.capture_cloud_period > 0 else None
    log = (output/'fastlio.log').open('w')
    child = subprocess.Popen(['ros2', 'run', 'fast_lio', 'fastlio_mapping', '--ros-args',
                              '--params-file', str(capture/'calibration.yaml'),
                              '--log-level', 'warn'], stdout=log, stderr=subprocess.STDOUT,
                             start_new_session=True)
    published = 0
    try:
        deadline = time.monotonic()+20.
        required = ['/velodyne_points', '/imu']+(['/odom'] if args.wheel == 'on' else [])
        while not all(publishers[t].get_subscription_count() for t in required):
            rclpy.spin_once(node, timeout_sec=.1)
            if child.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError(f'FastLIO did not connect; see {output}/fastlio.log')
        r = reader(bag, list(types))
        first_ns, first_wall, last_print = None, time.monotonic(), 0.
        while r.has_next():
            topic, data, ns = r.read_next()
            if first_ns is None:
                first_ns = ns
                first_wall = time.monotonic()
            if args.duration and (ns-first_ns)*1e-9 > args.duration:
                break
            due = first_wall+(ns-first_ns)*1e-9/args.rate
            while time.monotonic() < due:
                rclpy.spin_once(node, timeout_sec=max(0., min(.01, due-time.monotonic())))
            msg = deserialize_message(data, types[topic])
            if topic == '/imu':
                for axis, bias in zip(('x', 'y', 'z'), args.imu_accel_bias):
                    setattr(msg.linear_acceleration, axis, getattr(msg.linear_acceleration, axis)-bias)
            publishers[topic].publish(msg)
            published += 1
            rclpy.spin_once(node, timeout_sec=0.)
            if child.poll() is not None:
                raise RuntimeError(f'FastLIO stopped; see {output}/fastlio.log')
            if time.monotonic()-last_print > 5.:
                print(f'replayed {(ns-first_ns)*1e-9:.1f}s; sensor messages={published}; LIO poses={written}', flush=True)
                last_print = time.monotonic()
        deadline = time.monotonic()+5.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=10.)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=5.)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        del writer
    if written < 2:
        raise RuntimeError(f'Not enough recomputed poses: {written}')
    print(f'Saved {written} LIO poses: {output}', flush=True)
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', nargs='?', help='Raw bag; omit to choose by number')
    parser.add_argument('--output', help='New pose bag directory; default: unused recomputed_lio[_N] under raw bag')
    parser.add_argument('--directory', default=str(Path.home()/'rosbag2_data'))
    parser.add_argument('--config', help='Frontend YAML; default: recorded capture/calibration.yaml')
    parser.add_argument('--wheel', choices=['on', 'off'], default='on')
    parser.add_argument('--point-time', choices=['normalized', 'legacy'], default='normalized',
                        help='legacy disables signed-time normalization for controlled diagnosis only')
    parser.add_argument('--extrinsics', choices=['recorded-tf', 'config'], default='recorded-tf',
                        help='Use recorded TF mounts or preserve the extrinsics in the selected YAML')
    parser.add_argument('--imu-accel-bias', type=float, nargs=3, default=[0., 0., 0.], metavar=('X', 'Y', 'Z'),
                        help='Subtract this bias [m/s^2] from replayed IMU only; never edits the input bag')
    parser.add_argument('--capture-cloud-period', type=float, default=0.,
                        help='Save deskewed body scans at this sensor-time interval [s]; 0 disables')
    parser.add_argument('--domain', type=int, help='Private replay domain; omit to choose a free tool domain')
    parser.add_argument('--rate', type=float, default=2.)
    parser.add_argument('--duration', type=float, help='Optional sensor-time duration [s]')
    options = parser.parse_args()
    if options.bag is None:
        from map_selection import bag_candidates, choose_numbered
        options.bag = choose_numbered(lambda: bag_candidates(options.directory), 'FastLIOを再計算するBAG', options.directory)
        if options.bag is None:
            raise SystemExit(0)
    if options.output is None:
        base = Path(options.bag).expanduser().resolve()
        options.output = base/'recomputed_lio'
        number = 2
        while options.output.exists():
            options.output = base/f'recomputed_lio_{number}'
            number += 1
    if ((options.domain is not None and not 1 <= options.domain <= 232) or options.rate <= 0
            or (options.duration is not None and options.duration <= 0)
            or options.capture_cloud_period < 0 or not np.isfinite(options.imu_accel_bias).all()):
        parser.error('Invalid replay domain, rate or duration')
    live_domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    if options.domain == live_domain:
        parser.error('Replay domain must differ from the current live ROS domain')
    domain_lock = None
    for domain in ([options.domain] if options.domain is not None else range(231, 219, -1)):
        if domain == live_domain:
            continue
        handle = open(f'/tmp/sirius_offline_fastlio_domain_{domain}.lock', 'a')
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            handle.close()
            continue
        options.domain, domain_lock = domain, handle
        break
    if domain_lock is None:
        parser.error('Private replay domain is busy; wait or choose another --domain')
    raise SystemExit(run(options))
