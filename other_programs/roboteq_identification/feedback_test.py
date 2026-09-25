#!/usr/bin/env python3
"""
空転(宙吊り)でのフィードバック制御テスト。

2方式を比較する:
  software : ROS/Python側でソフトPI制御。?BS(モータRPM)をフィードバックし、
             同定したFF(duty->rpm)にPI補正を足して !G でduty指令。
  roboteq  : Roboteq内蔵の閉ループ速度制御。^MMOD x 1 にし、^KP/^KI/^KD を設定、
             !S <speed> で目標速度を与える。

使い方:
  python3 feedback_test.py --mode software --targets 500,1000,2000,3000,1000,0
  python3 feedback_test.py --mode roboteq --targets 1000,2000,3000 --kp 2 --ki 1
  python3 feedback_test.py --mode probe      # Roboteq閉ループのスケーリング確認
"""

import argparse
import csv
import json
import os
import signal
import sys
import time
from datetime import datetime

from roboteq_id import RoboteqSerial


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def parse_two(resp):
    if "=" in resp:
        resp = resp.split("=", 1)[1]
    out = []
    for p in resp.replace(" ", "").split(":"):
        try:
            out.append(float(p))
        except ValueError:
            pass
    while len(out) < 2:
        out.append(0.0)
    return out[0], out[1]


def parse_v_batt(resp):
    if "=" in resp:
        resp = resp.split("=", 1)[1]
    parts = resp.split(":")
    try:
        return float(parts[1]) * 0.1
    except (IndexError, ValueError):
        return 0.0


def parse_ff(resp):
    try:
        return int(float(resp.split("=", 1)[-1].split(":")[0]))
    except (ValueError, IndexError):
        return 0


class Tester:
    def __init__(self, port, baud, outdir):
        self.rob = RoboteqSerial(port, baud)
        self.rob.open()
        # 他プロセスが残したストリームやE-stopを解除してから設定を読む。
        # (ストリームが残っているとquery応答に混入し?BS等が誤読される)
        self.rob.send("#")
        self.rob.send("!MG")
        self.rob.send("^MMOD 1 0")
        self.rob.send("^MMOD 2 0")
        time.sleep(0.2)
        try:
            self.rob.ser.reset_input_buffer()
        except Exception:
            pass
        self.cfg = self.rob.read_config()
        self.outdir = outdir
        os.makedirs(outdir, exist_ok=True)
        self.stopped = {"v": False}

    def stop_motors(self):
        try:
            self.rob.send("!G 1 0")
            self.rob.send("!G 2 0")
            self.rob.send("!S 1 0")
            self.rob.send("!S 2 0")
        except Exception:
            pass

    def restore(self):
        try:
            self.rob.send("^MMOD 1 %s" % self.cfg.get("MMOD1", "0"))
            self.rob.send("^MMOD 2 %s" % self.cfg.get("MMOD2", "0"))
            self.rob.send("^MXRPM 1 %s" % self.cfg.get("MXRPM1", "100"))
            self.rob.send("^MXRPM 2 %s" % self.cfg.get("MXRPM2", "100"))
            for k in ("KP", "KI", "KD"):
                self.rob.send("^%s 1 %s" % (k, self.cfg.get("%s1" % k, "0")))
                self.rob.send("^%s 2 %s" % (k, self.cfg.get("%s2" % k, "0")))
        except Exception:
            pass

    def close(self):
        self.stop_motors()
        self.rob.close()

    def raw_query(self, cmd, key):
        """!G等のACK(+)が先行しても、期待するKEY=行まで読み飛ばして返す。"""
        with self.rob.lock:
            ser = self.rob.ser
            ser.reset_input_buffer()
            ser.write((cmd + "\r").encode("ascii"))
            ser.flush()
            t0 = time.monotonic()
            buf = b""
            while time.monotonic() - t0 < 0.3:
                buf += ser.read(256)
                while b"\r" in buf:
                    line, buf = buf.split(b"\r", 1)
                    s = line.decode("ascii", "replace").strip()
                    if s.startswith(key + "="):
                        return s
            return ""

    def query_bs(self):
        return parse_two(self.raw_query("?BS", "BS"))

    def query_v(self):
        return parse_v_batt(self.raw_query("?V", "V"))

    def query_a(self):
        return parse_two(self.raw_query("?A", "A"))

    def query_ff(self):
        return parse_ff(self.raw_query("?FF", "FF"))

    def log_header(self):
        return ["t", "mode", "phase", "target1", "target2", "actual1", "actual2",
                "duty1", "duty2", "v_batt", "a1", "a2", "ff"]

    def run_software(self, targets, hold, rate, kp, ki, kff, max_duty, vnom):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.outdir, "feedback_software_%s.csv" % stamp)
        fh = open(path, "w", newline="")
        w = csv.writer(fh)
        w.writerow(self.log_header())
        print("software PI: targets=%s hold=%.1fs rate=%.0fHz kp=%.3f ki=%.3f kff=%.2f" %
              (targets, hold, rate, kp, ki, kff))
        duty = [0.0, 0.0]
        integ = [0.0, 0.0]
        start = time.monotonic()
        last = start
        it = 0
        v_batt = vnom
        results = []
        for tgt in targets:
            seg_end = time.monotonic() + hold
            t1, t2 = tgt, -tgt  # both wheels forward (ch2 is negative)
            seg_rows = []
            while time.monotonic() < seg_end and not self.stopped["v"]:
                now = time.monotonic()
                dt = max(1e-3, now - last)
                last = now
                a1, a2 = self.query_bs()
                if it % 10 == 0:
                    v_batt = self.query_v()
                for i, (target, act) in enumerate(((t1, a1), (t2, a2))):
                    err = target - act
                    ff = target / kff * (vnom / max(v_batt, 1e-6))
                    integ[i] = clamp(integ[i] + err * dt, -800.0, 800.0)
                    duty[i] = clamp(ff + kp * err + ki * integ[i], -max_duty, max_duty)
                self.rob.send("!G 1 %d" % int(round(duty[0])))
                self.rob.send("!G 2 %d" % int(round(duty[1])))
                row = [round(now - start, 4), "software", "tgt=%d" % tgt,
                       t1, t2, round(a1, 1), round(a2, 1),
                       int(round(duty[0])), int(round(duty[1])),
                       round(v_batt, 2), "", "", ""]
                w.writerow(row)
                seg_rows.append((now, a1, a2, duty[0], duty[1]))
                it += 1
                time.sleep(max(0.0, 1.0 / rate - (time.monotonic() - now)))
            # steady-state stats (last 40%)
            if seg_rows:
                tail = seg_rows[int(len(seg_rows) * 0.6):]
                m1 = sum(r[1] for r in tail) / len(tail)
                m2 = sum(r[2] for r in tail) / len(tail)
                results.append((tgt, m1, m2))
                print("  tgt=%-6d actual1=%8.1f actual2=%8.1f  err1=%7.1f err2=%7.1f" %
                      (tgt, m1, m2, tgt - m1, -tgt - m2))
        # stop
        self.rob.send("!G 1 0")
        self.rob.send("!G 2 0")
        fh.close()
        print("CSV:", path)
        return results

    def run_roboteq(self, targets, hold, rate, kp, ki, kd, mxrpm, max_duty):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.outdir, "feedback_roboteq_%s.csv" % stamp)
        fh = open(path, "w", newline="")
        w = csv.writer(fh)
        w.writerow(self.log_header())
        print("roboteq closed-loop: targets=%s mxrpm=%d kp=%d ki=%d kd=%d" %
              (targets, mxrpm, kp, ki, kd))
        self.rob.send("^MMOD 1 1")
        self.rob.send("^MMOD 2 1")
        self.rob.send("^MXRPM 1 %d" % mxrpm)
        self.rob.send("^MXRPM 2 %d" % mxrpm)
        for ch in (1, 2):
            self.rob.send("^KP %d %d" % (ch, kp))
            self.rob.send("^KI %d %d" % (ch, ki))
            self.rob.send("^KD %d %d" % (ch, kd))
        time.sleep(0.2)
        start = time.monotonic()
        last = start
        it = 0
        results = []
        for tgt in targets:
            sp1 = clamp(tgt / mxrpm * 1000.0, -1000, 1000)
            sp2 = -sp1
            self.rob.send("!S 1 %d" % int(round(sp1)))
            self.rob.send("!S 2 %d" % int(round(sp2)))
            seg_end = time.monotonic() + hold
            seg_rows = []
            while time.monotonic() < seg_end and not self.stopped["v"]:
                now = time.monotonic()
                a1, a2 = self.query_bs()
                if it % 10 == 0:
                    v = self.query_v()
                    a = self.query_a()
                    ff = self.query_ff()
                row = [round(now - start, 4), "roboteq", "tgt=%d" % tgt,
                       int(round(sp1)), int(round(sp2)), round(a1, 1), round(a2, 1),
                       "", "", round(v, 2), a[0], a[1], ff]
                w.writerow(row)
                seg_rows.append((a1, a2))
                it += 1
                time.sleep(max(0.0, 1.0 / rate - (time.monotonic() - now)))
            if seg_rows:
                tail = seg_rows[int(len(seg_rows) * 0.6):]
                m1 = sum(r[0] for r in tail) / len(tail)
                m2 = sum(r[1] for r in tail) / len(tail)
                results.append((tgt, m1, m2))
                print("  tgt=%-6d actual1=%8.1f actual2=%8.1f" % (tgt, m1, m2))
        self.rob.send("!S 1 0")
        self.rob.send("!S 2 0")
        self.restore()
        fh.close()
        print("CSV:", path)
        return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/roboteq")
    ap.add_argument("--baud", type=int, default=230400)
    ap.add_argument("--mode", choices=["software", "roboteq", "probe"], default="software")
    ap.add_argument("--targets", default="500,1000,2000,3000,1000,0",
                    help="モータRPM目標のカンマ区切り")
    ap.add_argument("--hold", type=float, default=2.5)
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--kp", type=float, default=0.10)
    ap.add_argument("--ki", type=float, default=0.60)
    ap.add_argument("--kd", type=int, default=0)
    ap.add_argument("--kff", type=float, default=3.2, help="rpm/duty(同定値)")
    ap.add_argument("--vnom", type=float, default=13.3)
    ap.add_argument("--max-duty", type=int, default=1000)
    ap.add_argument("--mxrpm", type=int, default=4000)
    ap.add_argument("--outdir", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "results"))
    args = ap.parse_args()

    t = Tester(args.port, args.baud, args.outdir)

    def on_signal(*_):
        if t.stopped["v"]:
            return
        t.stopped["v"] = True
        sys.stderr.write("\n[feedback] 中断: モータ停止\n")
        t.stop_motors()
        t.restore()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    print("FID:", t.cfg.get("FID"))
    print("MMOD:", t.cfg.get("MMOD1"), t.cfg.get("MMOD2"),
          " MXRPM:", t.cfg.get("MXRPM1"), t.cfg.get("MXRPM2"),
          " EPPR:", t.cfg.get("EPPR1"), " KP/KI/KD:",
          t.cfg.get("KP1"), t.cfg.get("KI1"), t.cfg.get("KD1"))
    try:
        if args.mode == "probe":
            t.rob.send("^MMOD 1 1")
            t.rob.send("^MMOD 2 1")
            t.rob.send("^MXRPM 1 %d" % args.mxrpm)
            t.rob.send("^MXRPM 2 %d" % args.mxrpm)
            for ch in (1, 2):
                t.rob.send("^KP %d 1" % ch)
                t.rob.send("^KI %d 0" % ch)
            print("probe: !S 1 250 (25%% of MXRPM=%d)" % args.mxrpm)
            t.rob.send("!S 1 250")
            time.sleep(1.5)
            a1, a2 = t.query_bs()
            print("  ?BS1 =", a1, " ?BS2 =", a2)
            t.rob.send("!S 1 500")
            time.sleep(1.5)
            a1, a2 = t.query_bs()
            print("  !S=500 -> ?BS1 =", a1)
            t.rob.send("!S 1 0")
        elif args.mode == "software":
            targets = [int(x) for x in args.targets.split(",") if x.strip()]
            t.run_software(targets, args.hold, args.rate, args.kp, args.ki,
                           args.kff, args.max_duty, args.vnom)
        else:
            targets = [int(x) for x in args.targets.split(",") if x.strip()]
            t.run_roboteq(targets, args.hold, args.rate, args.kp, args.ki,
                          args.kd, args.mxrpm, args.max_duty)
    finally:
        t.stop_motors()
        t.restore()
        t.close()
        print("done")


if __name__ == "__main__":
    main()
