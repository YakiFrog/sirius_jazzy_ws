#!/usr/bin/env python3
"""Restart localization and IMU through the launcher after a measured stationary handover."""
import argparse
import json
import math
import os
from pathlib import Path
import shlex
import sys
import time
import urllib.request


def api(route, payload=None):
    request = urllib.request.Request('http://127.0.0.1:8090'+route,
        data=None if payload is None else json.dumps(payload).encode(),
        headers={'Content-Type': 'application/json'})
    result = json.load(urllib.request.urlopen(request, timeout=30))
    if result.get('error'):
        raise RuntimeError(result['error'])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True, type=Path)
    parser.add_argument('--restart-imu', action='store_true')
    parser.add_argument('--keep-lio', action='store_true', help='Restart map matching and TF only')
    args = parser.parse_args()
    if args.keep_lio and args.restart_imu:
        parser.error("--keep-lio and --restart-imu are mutually exclusive")
    map_path = args.map.expanduser().resolve(strict=True)
    import rclpy
    import yaml
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from std_msgs.msg import Bool
    from tf2_ros import Buffer, TransformListener, TransformException
    from sirius_navigation.map_localizer_node import _matrix_from_transform
    rclpy.init()
    node = rclpy.create_node('localization_stationary_handover')
    buffer = Buffer(); listener = TransformListener(buffer, node)
    wheel, ready = [], []
    node.create_subscription(Odometry, '/odom', lambda m: wheel.append((time.monotonic(), m)), qos_profile_sensor_data)
    node.create_subscription(Bool, '/localization_ready', lambda m: ready.append(m.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def spin_until(predicate, timeout):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return
        raise RuntimeError('Timed out waiting for localization handover condition')
    def stationary():
        recent = [(t, m) for t, m in wheel if t > time.monotonic()-1.5]
        if len(recent) < 10 or recent[-1][0]-recent[0][0] < .8:
            return False
        return all(math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y) < .03
                   and abs(m.twist.twist.angular.z) < .05 for _, m in recent)
    try:
        spin_until(stationary, 15)
        seed = None
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            try:
                odom = buffer.lookup_transform('sirius3/odom', 'sirius3/base_footprint', Time())
                age = (node.get_clock().now().nanoseconds-
                       (odom.header.stamp.sec*10**9+odom.header.stamp.nanosec))/1e9
                if not 0 <= age < .3:
                    continue
                correction = buffer.lookup_transform('map', 'sirius3/odom', Time())
                T = _matrix_from_transform(correction) @ _matrix_from_transform(odom)
                seed = dict(x=float(T[0, 3]), y=float(T[1, 3]), z=float(T[2, 3]),
                            yaw=math.atan2(T[1, 0], T[0, 0]), source_tf_age_s=age)
                break
            except TransformException:
                pass
        if seed is None:
            raise RuntimeError('No fresh map-frame pose; refusing to invent a restart seed')
        state = Path.home()/'.local/state/sirius_localization_v15'
        state.mkdir(parents=True, exist_ok=True)
        tag = time.strftime('%Y%m%d_%H%M%S')
        seed_path = state/f'initial_pose_handover_{tag}.yaml'
        seed_path.write_text(yaml.safe_dump(seed))
        print('Stationary handover:', json.dumps(seed), flush=True)
        for name in ['loc3d']+([] if args.keep_lio else ['lio3d_loc'])+(['imu'] if args.restart_imu else []):
            print('Stop', name, api('/api/stop', {'name': name}), flush=True)
        if args.restart_imu:
            print('Start imu', api('/api/start', {'name': 'imu', 'domain': os.environ['ROS_DOMAIN_ID']}), flush=True)
        if not args.keep_lio:
            print('Start lio3d_loc', api('/api/start', {'name': 'lio3d_loc', 'domain': os.environ['ROS_DOMAIN_ID']}), flush=True)
        # Require an odometry sample generated after the restart, not a cached old TF.
        restarted = node.get_clock().now().nanoseconds
        def fresh_lio():
            try:
                t = buffer.lookup_transform('sirius3/odom', 'sirius3/base_footprint', Time())
                ns = t.header.stamp.sec*10**9+t.header.stamp.nanosec
                return ns > restarted and 0 <= (node.get_clock().now().nanoseconds-ns)/1e9 < .15
            except TransformException:
                return False
        spin_until(fresh_lio, 30)
        # API shutdown can take seconds; wait for a fresh stationary window.
        spin_until(stationary, 10)
        ws = Path.home()/'sirius_jazzy_ws'
        sys.path.insert(0, str(ws/'other_programs/sirius_launcher'))
        from process_manager import ProcessManager
        command = ('SIRIUS_3D_MAP='+shlex.quote(str(map_path))+
                   ' SIRIUS_LOCALIZATION_INITIAL_POSE_FILE='+shlex.quote(str(seed_path))+
                   ' bash '+shlex.quote(str(ws/'bash/startup_bash/start_3d_localization.sh'))+' real')
        ready.clear()
        manager = ProcessManager('loc3d', command)
        if not manager.launch_web(os.environ['ROS_DOMAIN_ID']):
            raise RuntimeError('Failed to start loc3d')
        spin_until(lambda: bool(ready) and ready[-1], 30)
        (state/f'handover_{tag}.json').write_text(json.dumps(
            {'seed': seed, 'map': str(map_path), 'ready': True}, indent=2)+'\n')
        print('Localization ready: independent TF and accepted registration confirmed', flush=True)
    finally:
        node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
