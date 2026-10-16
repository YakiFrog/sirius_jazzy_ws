#!/usr/bin/env python3
"""Resolve simulator calibration from the published sensor contract."""
import argparse
import copy
import json
from pathlib import Path
import time
import numpy as np
import yaml


def apply_profile(template, profile):
    result = copy.deepcopy(template)
    if profile.get('profile') not in ('sirius-real', 'ideal'):
        raise ValueError('Unknown Web sensor profile')
    mounts = profile['mounts']
    def xyz(name):
        value = np.asarray(mounts[name], dtype=float)
        if value.shape != (3,) or not np.isfinite(value).all():
            raise ValueError(f'Invalid {name} mount')
        return value
    lidar, imu = xyz('lidar'), xyz('imu')
    p = result['/**']['ros__parameters']
    p['mapping']['extrinsic_T'] = (lidar-imu).tolist()
    p['mapping']['extrinsic_R'] = np.eye(3).reshape(-1).tolist()
    p.setdefault('wheel', {})['imu_to_base_translation'] = (-imu).tolist()
    p['preprocess']['lidar_type'] = 2 if profile['lidar']['rolling'] else 0
    p['preprocess']['timestamp_unit'] = 0
    return result


def receive_profile(timeout):
    import rclpy
    from std_msgs.msg import String
    rclpy.init(args=[])
    node = rclpy.create_node('sirius_web_sensor_profile')
    found = []
    sub = node.create_subscription(String, '/sirius_web/sensor_config', lambda m: found.append(m.data), 10)
    end = time.monotonic()+timeout
    try:
        while not found and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        return json.loads(found[-1]) if found else None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--template', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--wait', type=float, default=4.)
    args = parser.parse_args()
    profile = receive_profile(args.wait)
    if profile is None:
        parser.exit(1, 'Webセンサー設定を受信できません。WebでROS接続後に起動してください。\n')
    result = apply_profile(yaml.safe_load(Path(args.template).read_text()), profile)
    Path(args.output).write_text(yaml.safe_dump(result, sort_keys=False))
    print(float(profile['mounts']['imu'][2]))


if __name__ == '__main__':
    main()
