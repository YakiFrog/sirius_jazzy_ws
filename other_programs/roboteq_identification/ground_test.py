#!/usr/bin/env python3
"""
接地テスト(安全版): 直進(往復)・加減速・急停止・その場旋回。

安全機能:
  - /scan3 を監視し、進行方向(前進/後退)や旋回の障害物が閾値以内なら即中断。
  - /roboteq/status の fault_flags / voltage を監視。低電圧(4)/短絡(8)/E-stop(16)で即中断。
    FF=2(過電圧)は記録するが継続。
  - 変位 > 1.5m で即中断。
  - 中断時はテスト全体を停止(以降の区間を実行しない)。
  - 開始ゲート: V>11.5V かつ FF==0 のときのみ走行。前進/後退の方向確認を最初に実施。

使い方:
  source install/setup.bash
  python3 ground_test.py --label open --max-speed 0.6
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
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

try:
    from roboteq_ros2_driver.msg import RoboteqStatus
    HAVE_STATUS = True
except Exception:
    HAVE_STATUS = False

# 障害物停止距離 [m]
STOP_FWD = 0.9
STOP_REAR = 0.9
STOP_SPIN = 0.5
# 中断条件
MAX_DISPLACEMENT = 1.5
MIN_VOLTAGE = 9.0
ABORT_FF_MASK = 4 | 8 | 16  # undervoltage / short / estop


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GroundTest(Node):
    def __init__(self):
        super().__init__("ground_test")
        self.pub = self.create_publisher(Twist, "/cmd_vel_direct", 10)
        self.odom = None
        self.filt = None
        self.imu_yaw = 0.0
        self.amcl = None
        self.status = None
        self.scan = None
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.create_subscription(Odometry, "/odom/filtered", self.on_filt, 50)
        self.create_subscription(Imu, "/imu", self.on_imu, 100)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 10)
        self.create_subscription(LaserScan, "/scan3", self.on_scan, 20)
        if HAVE_STATUS:
            self.create_subscription(RoboteqStatus, "/roboteq/status", self.on_status, 50)
        self.t0 = time.monotonic()
        self.rows = []
        self.cmd = Twist()
        self.phase = "init"
        self.start_xy = None
        self.aborted = False
        self.abort_reason = ""

    def on_odom(self, m):
        self.odom = m

    def on_filt(self, m):
        self.filt = m

    def on_imu(self, m):
        self.imu_yaw = yaw_from_quat(m.orientation)

    def on_amcl(self, m):
        self.amcl = m

    def on_status(self, m):
        self.status = m

    def on_scan(self, m):
        self.scan = m

    def publish(self, lin, ang):
        self.cmd.linear.x = lin
        self.cmd.angular.z = ang
        self.pub.publish(self.cmd)

    def min_range_sector(self, center_rad, half_rad):
        """scan3の中心角±半角内の最小距離[m]。無効値は無視。"""
        s = self.scan
        if s is None:
            return float("inf")
        best = float("inf")
        n = len(s.ranges)
        for i, r in enumerate(s.ranges):
            if not math.isfinite(r) or r < max(s.range_min, 0.05) or r > s.range_max:
                continue
            a = s.angle_min + i * s.angle_increment
            d = math.atan2(math.sin(a - center_rad), math.cos(a - center_rad))
            if abs(d) <= half_rad and r < best:
                best = r
        return best

    def obstacle_clearance(self, lin, ang):
        if self.scan is None:
            return float("inf")
        if abs(ang) > 0.3 and abs(lin) < 0.05:
            return self.min_range_sector(0.0, math.pi)  # 旋回: 全周
        if lin > 0.05:
            return self.min_range_sector(0.0, math.radians(30))
        if lin < -0.05:
            return self.min_range_sector(math.pi, math.radians(30))
        return float("inf")

    def displacement(self):
        src = self.odom.pose.pose.position if self.odom else (
            self.amcl.pose.pose.position if self.amcl else None)
        if src is None:
            return 0.0
        if self.start_xy is None:
            self.start_xy = (src.x, src.y)
        return math.hypot(src.x - self.start_xy[0], src.y - self.start_xy[1])

    def record(self):
        od = self.odom.twist.twist if self.odom else None
        ft = self.filt.twist.twist if self.filt else None
        st = self.status
        ax = self.amcl.pose.pose.position if self.amcl else None
        ay = yaw_from_quat(self.amcl.pose.pose.orientation) if self.amcl else 0.0
        self.rows.append({
            "t": round(time.monotonic() - self.t0, 4), "phase": self.phase,
            "cmd_lin": round(self.cmd.linear.x, 4), "cmd_ang": round(self.cmd.angular.z, 4),
            "odom_v": round(od.linear.x, 4) if od else "",
            "odom_w": round(od.angular.z, 4) if od else "",
            "filt_v": round(ft.linear.x, 4) if ft else "",
            "filt_w": round(ft.angular.z, 4) if ft else "",
            "imu_yaw": round(self.imu_yaw, 4),
            "amcl_x": round(ax.x, 4) if ax else "",
            "amcl_y": round(ax.y, 4) if ay else "",
            "amcl_yaw": round(ay, 4),
            "clearance": round(self.obstacle_clearance(self.cmd.linear.x, self.cmd.angular.z), 3),
            "v_batt": round(st.voltage, 2) if st else "",
            "a1": round(st.current_ch1, 2) if st else "",
            "a2": round(st.current_ch2, 2) if st else "",
            "ba1": round(st.battery_current_ch1, 2) if st else "",
            "ba2": round(st.battery_current_ch2, 2) if st else "",
            "rpm1": round(st.rpm_ch1, 0) if st else "",
            "rpm2": round(st.rpm_ch2, 0) if st else "",
            "ff": st.fault_flags if st else "",
        })

    def safety_check(self, lin, ang):
        """安全ならTrue。中断条件に当たればself.abortedを立てる。"""
        if self.status:
            ff = self.status.fault_flags
            if ff & ABORT_FF_MASK:
                self.abort_reason = "FF=%d (低電圧/短絡/E-stop)" % ff
                return False
            if self.status.voltage < MIN_VOLTAGE:
                self.abort_reason = "V=%.1fV 低下" % self.status.voltage
                return False
        if self.displacement() > MAX_DISPLACEMENT:
            self.abort_reason = "変位>%.1fm" % MAX_DISPLACEMENT
            return False
        clr = self.obstacle_clearance(lin, ang)
        if abs(ang) > 0.3 and abs(lin) < 0.05:
            if clr < STOP_SPIN:
                self.abort_reason = "旋回障害物 %.2fm" % clr
                return False
        elif lin > 0.05 and clr < STOP_FWD:
            self.abort_reason = "前方障害物 %.2fm" % clr
            return False
        elif lin < -0.05 and clr < STOP_REAR:
            self.abort_reason = "後方障害物 %.2fm" % clr
            return False
        return True

    def run_segment(self, phase, lin, ang, hold):
        self.phase = phase
        end = time.monotonic() + hold
        while time.monotonic() < end:
            if self.aborted:
                break
            if not self.safety_check(lin, ang):
                self.aborted = True
                print("!! 中断: %s" % self.abort_reason)
                break
            self.publish(lin, ang)
            rclpy.spin_once(self, timeout_sec=0.0)
            self.record()
            time.sleep(0.05)
        self.publish(0.0, 0.0)
        rclpy.spin_once(self, timeout_sec=0.0)
        self.record()

    def stop_hard(self, sec=1.0):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            self.publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

    def wait_status(self, sec=3.0):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.status and self.scan:
                return
        return


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="run")
    ap.add_argument("--outdir", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "results"))
    ap.add_argument("--max-speed", type=float, default=0.6)
    ap.add_argument("--idle", type=float, default=20.0)
    ap.add_argument("--hold", type=float, default=1.5)
    ap.add_argument("--check-only", action="store_true",
                    help="走行せずに電源/フォルト/クリアランスだけ表示")
    args = ap.parse_args()

    rclpy.init()
    node = GroundTest()
    os.makedirs(args.outdir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(args.outdir, "ground_%s_%s.csv" % (args.label, stamp))

    def on_signal(*_):
        node.aborted = True
        node.abort_reason = "signal"
        node.publish(0.0, 0.0)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    print("ground test label=%s max=%.2f" % (args.label, args.max_speed))
    try:
        node.wait_status()
        st = node.status
        if st is None:
            print("!! /roboteq/status 未受信。中止。")
            return
        print("開始ゲート: V=%.1fV FF=%d" % (st.voltage, st.fault_flags))
        if st.voltage < 11.5 or st.fault_flags != 0:
            print("!! 電源/フォルト異常のため中止（e-stop復帰・電源ONを確認）")
            return
        if node.scan is None:
            print("!! /scan3 未受信。障害物監視できないため中止。")
            return

        fclr = node.min_range_sector(0.0, math.radians(30))
        rclr = node.min_range_sector(math.pi, math.radians(30))
        sclr = node.min_range_sector(0.0, math.pi)
        print("クリアランス: 前方=%.2fm 後方=%.2fm 全周最小=%.2fm" % (fclr, rclr, sclr))
        if args.check_only:
            print("--check-only: 走行しません。")
            return

        # 方向・クリアランス確認（微小前進→後退）
        fclr = node.min_range_sector(0.0, math.radians(30))
        rclr = node.min_range_sector(math.pi, math.radians(30))
        print("前方クリアランス=%.2fm 後方=%.2fm" % (fclr, rclr))
        # 接地は静摩擦が大きく0.15m/sでは動かないため0.3m/sで確認
        node.run_segment("dir_fwd", 0.3, 0.0, 0.8)
        node.run_segment("dir_rev", -0.3, 0.0, 0.8)
        if node.aborted:
            raise SystemExit
        d = node.displacement()
        print("方向確認後の変位=%.2fm（>0.05で動作確認）" % d)

        node.run_segment("idle", 0.0, 0.0, args.idle)
        vmax = min(0.9, args.max_speed)
        for v in [0.2, 0.4, vmax]:
            node.run_segment("fwd_%.1f" % v, v, 0.0, args.hold)
            if node.aborted:
                break
            node.run_segment("pause", 0.0, 0.0, 0.5)
            node.run_segment("rev_%.1f" % v, -v, 0.0, args.hold)
            if node.aborted:
                break
            node.run_segment("pause", 0.0, 0.0, 0.5)
        if not node.aborted:
            node.run_segment("accel_%.1f" % vmax, vmax, 0.0, 1.5)
            node.run_segment("decel_0", 0.0, 0.0, 1.5)
        if not node.aborted:
            node.run_segment("estop_run", vmax, 0.0, 1.2)
            node.run_segment("estop_stop", 0.0, 0.0, 1.5)
        if not node.aborted:
            for w in [0.5, 1.0, -1.0]:
                node.run_segment("spin_%.1f" % w, 0.0, w, 2.0)
                if node.aborted:
                    break
        if not node.aborted:
            node.run_segment("spin_+360", 0.0, 0.8, 7.85)
        if not node.aborted:
            node.run_segment("spin_-360", 0.0, -0.8, 7.85)
        if not node.aborted:
            node.run_segment("idle_end", 0.0, 0.0, 3.0)
    finally:
        node.stop_hard(1.0)
        if node.rows:
            with open(path, "w", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=list(node.rows[0].keys()))
                w.writeheader()
                w.writerows(node.rows)
            print("CSV:", path, "rows:", len(node.rows))
        print("aborted:", node.aborted, "reason:", node.abort_reason)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
