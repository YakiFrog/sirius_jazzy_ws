#!/usr/bin/env python3
"""
オープンループ vs ソフト閉ループの追従比較。

/cmd_vel に直進の速度ステップを与え、/odom の twist.linear.x(実速度)を記録する。
定常区間の「指令 - 実速度」誤差を集計する。

使い方:
  source install/setup.bash
  python3 cl_compare.py --label open
  ros2 param set /roboteq_ros2_driver closed_loop true
  python3 cl_compare.py --label closed
"""

import argparse
import csv
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Compare(Node):
    def __init__(self):
        super().__init__("cl_compare")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.cmd = Twist()
        self.actual = 0.0
        self.rows = []
        self.t0 = time.monotonic()
        self.seg = -1

    def on_odom(self, msg):
        self.actual = msg.twist.twist.linear.x
        self.rows.append((time.monotonic() - self.t0, self.seg,
                          self.cmd.linear.x, self.actual))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="run")
    ap.add_argument("--targets", default="0.3,0.6,0.9,0.6,0.3,0.0")
    ap.add_argument("--hold", type=float, default=2.5)
    ap.add_argument("--outdir", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "results"))
    args = ap.parse_args()

    rclpy.init()
    node = Compare()
    targets = [float(x) for x in args.targets.split(",")]
    os.makedirs(args.outdir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(args.outdir, "cl_%s_%s.csv" % (args.label, stamp))

    results = []
    for i, tgt in enumerate(targets):
        node.seg = i
        node.cmd.linear.x = tgt
        node.cmd.angular.z = 0.0
        end = time.monotonic() + args.hold
        seg_rows = []
        while time.monotonic() < end:
            node.pub.publish(node.cmd)
            rclpy.spin_once(node, timeout_sec=0.02)
            seg_rows.append((node.actual, tgt))
            time.sleep(0.03)
        # steady state = last 50%
        tail = seg_rows[len(seg_rows) // 2:] if seg_rows else [(0, tgt)]
        act = sum(r[0] for r in tail) / len(tail)
        results.append((tgt, act, tgt - act))
        print("  tgt=%.2f actual=%.3f err=%+.3f" % (tgt, act, tgt - act))

    # stop
    node.cmd.linear.x = 0.0
    for _ in range(20):
        node.pub.publish(node.cmd)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.03)

    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "seg", "target", "actual"])
        w.writerows(node.rows)
    rms = (sum(e * e for _, _, e in results) / len(results)) ** 0.5
    print("RMS err = %.4f m/s   CSV=%s" % (rms, path))
    node.destroy_node()
    rclpy.shutdown()
    return results


if __name__ == "__main__":
    main()
