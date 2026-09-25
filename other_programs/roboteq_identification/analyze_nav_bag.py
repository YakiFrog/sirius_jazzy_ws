#!/usr/bin/env python3
"""Nav2自律走行バッグを解析: 追従・サグ・FF=2・蛇行。"""

import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    tmap = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = {"/odom/filtered", "/cmd_vel_smoothed", "/cmd_vel_nav", "/imu",
            "/roboteq/status", "/amcl_pose"}
    data = {k: [] for k in want}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic not in want:
            continue
        t = t_ns * 1e-9
        msg = deserialize_message(raw, get_message(tmap[topic]))
        if topic == "/odom/filtered":
            tw = msg.twist.twist
            data[topic].append((t, tw.linear.x, tw.angular.z))
        elif topic in ("/cmd_vel_smoothed", "/cmd_vel_nav"):
            data[topic].append((t, msg.linear.x, msg.angular.z))
        elif topic == "/imu":
            data[topic].append((t, msg.angular_velocity.z))
        elif topic == "/roboteq/status":
            data[topic].append((t, msg.voltage, msg.current_ch1, msg.current_ch2,
                                msg.battery_current_ch1, msg.battery_current_ch2,
                                msg.rpm_ch1, msg.rpm_ch2, msg.fault_flags))
        elif topic == "/amcl_pose":
            p = msg.pose.pose
            data[topic].append((t, p.position.x, p.position.y, yaw_from_quat(p.orientation)))

    t0 = min(v[0][0] for v in data.values() if v)

    def nearest(series, t, idx):
        if not series:
            return None
        # 線形に近傍探索
        best = min(series, key=lambda r: abs(r[0] - t))
        return best

    st = data["/roboteq/status"]
    vs = [r[1] for r in st]
    print("== 電圧 ==")
    print("  V min=%.2f max=%.2f" % (min(vs), max(vs)))
    a_max = max(max(abs(r[2]), abs(r[3])) for r in st)
    print("  モータ電流 max|A|=%.1f  (ALIM=12)" % a_max)
    n_ov = sum(1 for r in st if r[8] & 2)
    n_uv = sum(1 for r in st if r[8] & 4)
    print("  FF=2サンプル=%d  FF=4サンプル=%d  (status %d件)" % (n_ov, n_uv, len(st)))
    cvs = [r[1] for r in data["/cmd_vel_smoothed"]]
    print("  指令v max=%.2f  mean=%.2f" % (max(cvs), sum(cvs) / len(cvs)))

    print("== FFイベント ==")
    prev = 0
    events = []
    for r in st:
        t, V, a1, a2, ba1, ba2, rpm1, rpm2, ff = r
        if ff != 0 and prev == 0:
            cmd = nearest(data["/cmd_vel_smoothed"], t, 1)
            od = nearest(data["/odom/filtered"], t, 1)
            events.append((t - t0, ff, V, cmd, od))
        prev = ff
    for (dt, ff, V, cmd, od) in events:
        cl = cmd[1] if cmd else float("nan")
        ca = cmd[2] if cmd else float("nan")
        ov = od[1] if od else float("nan")
        print("  t=%6.2f FF=%-2d V=%5.1f cmd=(v%.2f,w%.2f) odom_v=%.2f" %
              (dt, ff, V, cl, ca, ov))
    print("  総イベント数=%d" % len(events))

    print("== 追従 (|cmd_v|>0.2) ==")
    ods = data["/odom/filtered"]
    pairs = []
    for (t, cv, cw) in data["/cmd_vel_smoothed"]:
        if abs(cv) > 0.2:
            o = nearest(ods, t, 1)
            if o and abs(o[0] - t) < 0.06:
                pairs.append((cv, o[1]))
    if pairs:
        ratio = [o / c for c, o in pairs if abs(c) > 1e-6]
        err = [o - c for c, o in pairs]
        print("  n=%d  mean(actual/cmd)=%.2f  mean err=%.3f  rms err=%.3f" %
              (len(pairs), sum(ratio) / len(ratio), sum(err) / len(err),
               (sum(e * e for e in err) / len(err)) ** 0.5))
        print("  cmd mean=%.2f  actual mean=%.2f" %
              (sum(c for c, o in pairs) / len(pairs), sum(o for c, o in pairs) / len(pairs)))

    print("== 蛇行 (直進時のヨーレート) ==")
    # |cmd_w|<0.1 の直進区間で odom_w のばらつき
    wz = []
    for (t, cv, cw) in data["/cmd_vel_smoothed"]:
        if abs(cw) < 0.1 and abs(cv) > 0.2:
            o = nearest(ods, t, 1)
            if o and abs(o[0] - t) < 0.06:
                wz.append(o[2])
    if wz:
        mean = sum(wz) / len(wz)
        rms = (sum((w - mean) ** 2 for w in wz) / len(wz)) ** 0.5
        print("  直進サンプル n=%d  yaw_rate mean=%.3f rms=%.3f rad/s  max|w|=%.3f" %
              (len(wz), mean, rms, max(abs(w) for w in wz)))
    # IMUヨーレートのばらつき（全走行）
    imu = [r[1] for r in data["/imu"]]
    if imu:
        m = sum(imu) / len(imu)
        print("  IMU yaw_rate rms=%.3f rad/s (全走行)" %
              ((sum((w - m) ** 2 for w in imu) / len(imu)) ** 0.5))

    print("== 速度・経路 ==")
    amcl = data["/amcl_pose"]
    if amcl:
        xs = [r[1] for r in amcl]
        ys = [r[2] for r in amcl]
        d = 0.0
        for i in range(1, len(amcl)):
            d += math.hypot(amcl[i][1] - amcl[i - 1][1], amcl[i][2] - amcl[i - 1][2])
        print("  AMCL走行距離≈%.2f m  範囲 x[%.1f,%.1f] y[%.1f,%.1f]" %
              (d, min(xs), max(xs), min(ys), max(ys)))


if __name__ == "__main__":
    main(sys.argv[1])
