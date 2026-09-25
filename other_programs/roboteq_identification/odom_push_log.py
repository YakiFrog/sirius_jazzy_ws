#!/usr/bin/env python3
"""
手押し校正用ロガー。/odom と /roboteq/status を記録し、終了時に
経路長・正味変位・エンコーダ回転数を出力する。

使い方（バックグラウンドで起動→手押し→SIGINTで停止）:
  source install/setup.bash
  python3 odom_push_log.py --out results/push_YYYYmmdd.csv &
  ... 5m 手押し ...
  kill -INT <pid>   # もしくは timeout で自動終了
"""

import argparse
import csv
import math
import os
import signal
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

try:
    from roboteq_ros2_driver.msg import RoboteqStatus
    HAVE_STATUS = True
except Exception:
    HAVE_STATUS = False

WHEEL_CIRC = 0.825  # roboteq.yaml と同じ
PULSE = 475


class PushLog(Node):
    def __init__(self):
        super().__init__("odom_push_log")
        self.rows = []
        self.odom = None
        self.status = None
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        if HAVE_STATUS:
            self.create_subscription(RoboteqStatus, "/roboteq/status", self.on_status, 50)
        self.t0 = time.monotonic()
        self.stop = False

    def on_odom(self, m):
        self.odom = m

    def on_status(self, m):
        self.status = m

    def tick(self):
        if self.odom is None:
            return
        p = self.odom.pose.pose.position
        t = self.odom.twist.twist
        r = self.rows[-1] if self.rows else None
        self.rows.append({
            "t": round(time.monotonic() - self.t0, 3),
            "x": round(p.x, 4), "y": round(p.y, 4),
            "v": round(t.linear.x, 4), "w": round(t.angular.z, 4),
            "rpm1": round(self.status.rpm_ch1, 0) if self.status else "",
            "rpm2": round(self.status.rpm_ch2, 0) if self.status else "",
            "ff": self.status.fault_flags if self.status else "",
            "V": round(self.status.voltage, 1) if self.status else "",
        })

    def summary(self):
        if len(self.rows) < 2:
            return "no data"
        xs = [r["x"] for r in self.rows]
        ys = [r["y"] for r in self.rows]
        path = 0.0
        for i in range(1, len(self.rows)):
            path += math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
        net = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
        ff = [r["ff"] for r in self.rows if r["ff"] != ""]
        vmin = min([r["V"] for r in self.rows if r["V"] != ""], default=0)
        return ("path_len=%.3f m  net_disp=%.3f m  start=(%.3f,%.3f) end=(%.3f,%.3f)  "
                "ff_seen=%s Vmin=%.1f" % (path, net, xs[0], ys[0], xs[-1], ys[-1],
                                          sorted(set(ff)), vmin))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None)
    ap.add_argument("--duration", type=float, default=0.0, help=">0なら自動終了[s]")
    args = ap.parse_args()

    rclpy.init()
    node = PushLog()
    out = args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "results", "push_%s.csv" % datetime.now().strftime("%Y%m%d_%H%M%S"))

    def on_sig(*_):
        node.stop = True
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    print("logging -> %s" % out, flush=True)
    t_end = time.monotonic() + args.duration if args.duration > 0 else None
    while not node.stop and (t_end is None or time.monotonic() < t_end):
        rclpy.spin_once(node, timeout_sec=0.05)
        node.tick()

    with open(out, "w", newline="") as fh:
        if node.rows:
            w = csv.DictWriter(fh, fieldnames=list(node.rows[0].keys()))
            w.writeheader()
            w.writerows(node.rows)
    s = node.summary()
    print("SUMMARY:", s, flush=True)
    with open(out + ".summary.txt", "w") as fh:
        fh.write(s + "\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
