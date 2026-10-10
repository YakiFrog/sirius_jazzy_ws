#!/usr/bin/env python3
"""画像(PNG)を sensor_msgs/Image として配信（Foxglove/Image表示用・実験的）。

使い方:
  theta_image_publisher.py --ros-args -p image_file:=/path/ground_map.png \
      -p topic:=/ground_map_image -p frame_id:=ground_map
"""
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('theta_image_publisher')
        self.declare_parameter('image_file', '')
        self.declare_parameter('topic', '/ground_map_image')
        self.declare_parameter('frame_id', 'ground_map')
        self.declare_parameter('period_sec', 2.0)
        path = self.get_parameter('image_file').value
        if not path or not os.path.exists(path):
            self.get_logger().error(f'画像が見つかりません: "{path}"')
            raise SystemExit(1)
        import cv2
        img = cv2.imread(path, cv2.IMREAD_COLOR)  # BGR
        if img is None:
            self.get_logger().error(f'画像読込失敗: {path}')
            raise SystemExit(1)
        self.img = img
        self.frame_id = self.get_parameter('frame_id').value
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Image, self.get_parameter('topic').value, qos)
        period = max(0.5, float(self.get_parameter('period_sec').value))
        self.create_timer(period, self.publish)
        self.publish()
        self.get_logger().info(
            f'配信中: {self.get_parameter("topic").value} {img.shape[1]}x{img.shape[0]} ({path})')

    def publish(self):
        h, w = self.img.shape[:2]
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = h, w
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = w * 3
        msg.data = self.img.tobytes()
        self.pub.publish(msg)


def main():
    rclpy.init()
    try:
        node = ImagePublisher()
    except SystemExit:
        if rclpy.ok():
            rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
