#!/usr/bin/env python3
"""Sample a PointCloud2 and report whether its RGB is ring colors or THETA colors.

使い方: cloud_color_check.py [topic] [timeout_s]
出力:  'theta:N'（自然色の可能性, N>32色） / 'ring:N'（リング色） / 'no_rgb' / 'timeout'
終了コード: 0=THETA色, 1=リング色, 2=判定不可
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/velodyne_points'
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    rclpy.init()
    node = Node('cloud_color_check')
    got = []
    node.create_subscription(PointCloud2, topic, lambda m: got.append(m),
                             qos_profile_sensor_data)
    end = time.monotonic() + timeout
    while not got and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    code = 2
    if got:
        p = read_points(got[0], skip_nans=True)
        field = ('rgb' if 'rgb' in p.dtype.names else
                 'rgba' if 'rgba' in p.dtype.names else None)
        if field is None:
            print('no_rgb')
        else:
            packed = np.ascontiguousarray(p[field]).reshape(-1)
            packed = (packed.view(np.uint32) if packed.dtype.kind == 'f'
                      else packed.astype(np.uint32))
            cols = np.stack([(packed >> 16) & 255, (packed >> 8) & 255, packed & 255], 1)
            unique = len(np.unique(cols, axis=0))
            if unique <= 32:
                print(f'ring:{unique}')
                code = 1
            else:
                print(f'theta:{unique}')
                code = 0
    else:
        print('timeout')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
