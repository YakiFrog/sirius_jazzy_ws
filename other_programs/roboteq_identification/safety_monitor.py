#!/usr/bin/env python3
"""
自律走行中の安全監視。/roboteq/status を監視し、
  - FF に 短絡(8) / E-stop(16) が立った
  - バス電圧 < 6.0V（深刻な崩壊）
なら /stop true を publish して強制停止する。
FF=2(過電圧) / FF=4(低電圧) は記録のみ（観測対象のため停止しない）。
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    from roboteq_ros2_driver.msg import RoboteqStatus
except Exception:
    print("RoboteqStatus not available")
    sys.exit(1)


class Safety(Node):
    def __init__(self):
        super().__init__("roboteq_safety_monitor")
        self.pub = self.create_publisher(Bool, "/stop", 10)
        self.create_subscription(RoboteqStatus, "/roboteq/status", self.on_status, 50)
        self.events = 0

    def on_status(self, m):
        ff = m.fault_flags
        if ff & 2:
            print("FF=2 overvoltage V=%.1f" % m.voltage, flush=True)
        if ff & 4:
            print("FF=4 undervoltage V=%.1f" % m.voltage, flush=True)
        fatal = (ff & 8) or (ff & 16) or (m.voltage < 6.0)
        if fatal:
            self.events += 1
            print("!! FATAL ff=%d V=%.1f -> /stop" % (ff, m.voltage), flush=True)
            msg = Bool()
            msg.data = True
            for _ in range(5):
                self.pub.publish(msg)
            raise SystemExit


def main():
    rclpy.init()
    node = Safety()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
