#!/usr/bin/env python3
"""
最低可動速度(デッドバンド)測定。低速度ランプを前進→後退で与え、
各速度の定常実速度を測定する。安全のため /scan3 監視・変位制限つき。

使い方:
  source install/setup.bash
  python3 min_speed_test.py --label open     # closed_loop=false で実行
  python3 min_speed_test.py --label closed   # closed_loop=true で実行
"""

import argparse
import csv
import math
import os
import signal
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

try:
    from roboteq_ros2_driver.msg import RoboteqStatus
    HAVE_STATUS = True
except Exception:
    HAVE_STATUS = False

STOP_FWD = 0.9
STOP_REAR = 0.9
MAX_DISP = 2.0
ABORT_FF = 4 | 8 | 16


class MinSpeed(Node):
    def __init__(self):
        super().__init__("min_speed_test")
        self.pub = self.create_publisher(Twist, "/cmd_vel_direct", 10)
        self.odom = None
        self.status = None
        self.scan = None
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.create_subscription(LaserScan, "/scan3", self.on_scan, 20)
        if HAVE_STATUS:
            self.create_subscription(RoboteqStatus, "/roboteq/status", self.on_status, 50)
        self.cmd = Twist()
        self.start_xy = None
        self.aborted = False
        self.reason = ""

    def on_odom(self, m):
        self.odom = m

    def on_status(self, m):
        self.status = m

    def on_scan(self, m):
        self.scan = m

    def pub_cmd(self, lin):
        self.cmd.linear.x = lin
        self.pub.publish(self.cmd)

    def min_range(self, center, half):
        if self.scan is None:
            return float("inf")
        best = float("inf")
        for i, r in enumerate(self.scan.ranges):
            if not math.isfinite(r) or r < 0.05 or r > self.scan.range_max:
                continue
            a = self.scan.angle_min + i * self.scan.angle_increment
            d = math.atan2(math.sin(a - center), math.cos(a - center))
            if abs(d) <= half and r < best:
                best = r
        return best

    def disp(self):
        if self.odom is None:
            return 0.0
        p = self.odom.pose.pose.position
        if self.start_xy is None:
            self.start_xy = (p.x, p.y)
        return math.hypot(p.x - self.start_xy[0], p.y - self.start_xy[1])

    def safety(self, lin):
        if self.status:
            if self.status.fault_flags & ABORT_FF:
                self.reason = "FF=%d" % self.status.fault_flags
                return False
            if self.status.voltage < 9.0:
                self.reason = "V=%.1f" % self.status.voltage
                return False
        if self.disp() > MAX_DISP:
            self.reason = "disp>%.1f" % MAX_DISP
            return False
        if lin > 0.02 and self.min_range(0.0, math.radians(30)) < STOP_FWD:
            self.reason = "fwd obstacle"
            return False
        if lin < -0.02 and self.min_range(math.pi, math.radians(30)) < STOP_REAR:
            self.reason = "rear obstacle"
            return False
        return True

    def seg(self, phase, lin, hold):
        end = time.monotonic() + hold
        vals = []
        while time.monotonic() < end:
            if self.aborted:
                break
            if not self.safety(lin):
                self.aborted = True
                print("!! 中断:", self.reason)
                break
            self.pub_cmd(lin)
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.odom:
                vals.append(self.odom.twist.twist.linear.x)
            time.sleep(0.05)
        self.pub_cmd(0.0)
        rclpy.spin_once(self, timeout_sec=0.0)
        if vals:
            tail = vals[len(vals) // 2:]
            return sum(tail) / len(tail)
        return 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="run")
    ap.add_argument("--hold", type=float, default=2.0)
    ap.add_argument("--speeds", default="0.05,0.1,0.15,0.2,0.25,0.3,0.4")
    ap.add_argument("--outdir", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "results"))
    args = ap.parse_args()

    rclpy.init()
    node = MinSpeed()

    def on_sig(*_):
        node.aborted = True
        node.pub_cmd(0.0)
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    # 状態待ち
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0 and (node.status is None or node.scan is None):
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.status and (node.status.voltage < 11.5 or node.status.fault_flags != 0):
        print("!! 電源/フォルト異常:", node.status.voltage, node.status.fault_flags)
        node.destroy_node(); rclpy.shutdown(); return
    print("クリアランス 前=%.2f 後=%.2f" % (
        node.min_range(0.0, math.radians(30)), node.min_range(math.pi, math.radians(30))))

    speeds = [float(x) for x in args.speeds.split(",")]
    print("== 最低可動速度測定 (%s) ==" % args.label)
    results = []
    for v in speeds:
        f = node.seg("fwd_%.2f" % v, v, args.hold)
        if node.aborted:
            break
        node.seg("pause", 0.0, 0.4)
        r = node.seg("rev_%.2f" % v, -v, args.hold)
        if node.aborted:
            break
        node.seg("pause", 0.0, 0.4)
        results.append((v, f, r))
        print("  cmd=%5.2f  fwd実速度=%+.3f  rev実速度=%+.3f  動いた=%s" % (
            v, f, r, "YES" if (abs(f) > 0.05 or abs(r) > 0.05) else "no"))

    print("== 結果 ==")
    thresh = None
    for v, f, r in results:
        moved = abs(f) > 0.05 or abs(r) > 0.05
        if moved and thresh is None:
            thresh = v
        print("  cmd=%.2f fwd=%+.3f rev=%+.3f" % (v, f, r))
    print("最低可動指令速度 ≈ %s m/s" % (thresh if thresh is not None else ">max"))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
