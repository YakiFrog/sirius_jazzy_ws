#!/usr/bin/env python3
"""
Roboteq 車輪モータ同定ツール (スタンドアロン / ROS不要)

実機の駆動輪を宙に浮かせた状態で、Roboteq コントローラへ直接シリアル接続し、
デューティ指令スケジュールを与えながら 電圧・電流・RPM・エンコーダ・温度・
フォルトフラグを CSV に記録する。

目的:
  - オープンループ指令(duty) -> 実車輪速度 のゲイン / デッドバンド / 時定数
  - 左右輪の差 (蛇行原因の切り分け)
  - バッテリ電圧降下と 12A 電流制限 (^ALIM) の到達状況
  - フォルト (?FF) の発生

安全:
  - 宙吊り専用。接地走行では使わないこと。
  - 終了時 / Ctrl+C 時に必ず !G 0 を送り、ストリームを停止する。
  - Roboteq のウォッチドッグ (^RWD) を切らさないよう定期送信する。

使い方:
    python3 roboteq_id.py --dry-run                 # 無通電でストリーム確認
    python3 roboteq_id.py --tests env               # アイドル記録のみ
    python3 roboteq_id.py --tests sweep,step        # 同定実験
    python3 roboteq_id.py                           # 既定スイート一式
    python3 roboteq_id.py --reverse                 # 後退も含める
"""

import argparse
import csv
import json
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime

try:
    import serial
except ImportError:
    sys.stderr.write("pyserial が必要です: pip install pyserial\n")
    sys.exit(1)

DEFAULT_PORT = "/dev/roboteq"
DEFAULT_BAUD = 230400
DEFAULT_STREAM_MS = 20
STREAM_QUERIES = ["CB", "BS", "A", "BA", "V", "T", "FF"]

CONFIG_KEYS = [
    "MMOD", "MXRPM", "ALIM", "EPPR", "EMOD", "ENCP", "BLFB",
    "KP", "KI", "KD", "MAC", "MDEC", "MXACC", "MXDEC", "RWD",
]


def parse_values(text):
    out = []
    for part in text.split(":"):
        part = part.strip()
        if part == "" or part == "-":
            continue
        try:
            out.append(float(part))
        except ValueError:
            try:
                out.append(int(part, 0))
            except ValueError:
                pass
    return out


class RoboteqSerial:
    def __init__(self, port, baud, stream_ms=DEFAULT_STREAM_MS):
        self.port = port
        self.baud = baud
        self.stream_ms = stream_ms
        self.ser = None
        self.lock = threading.Lock()
        self._latest = {}
        self._running = False
        self._thread = None
        self._buf = b""

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.02)
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.send("^ECHOF 1")
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    def close(self):
        self.stop_stream()
        self.stop_reader()
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send(self, cmd):
        with self.lock:
            self.ser.write((cmd + "\r").encode("ascii"))
            self.ser.flush()

    def query(self, cmd, timeout=0.3):
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write((cmd + "\r").encode("ascii"))
            self.ser.flush()
            t0 = time.monotonic()
            buf = b""
            while time.monotonic() - t0 < timeout:
                buf += self.ser.read(256)
                if b"\r" in buf:
                    line = buf.split(b"\r")[0].decode("ascii", "replace").strip()
                    if line:
                        return line
            return buf.decode("ascii", "replace").strip()

    def read_config(self):
        cfg = {}
        cfg["FID"] = self.query("?FID")
        for key in CONFIG_KEYS:
            for ch in (1, 2):
                resp = self.query("~%s %d" % (key, ch))
                if "=" in resp:
                    cfg["%s%d" % (key, ch)] = resp.split("=", 1)[1].strip()
                else:
                    cfg["%s%d" % (key, ch)] = resp
        return cfg

    def start_stream(self, queries=None, interval_ms=None):
        queries = queries or STREAM_QUERIES
        interval_ms = interval_ms or self.stream_ms
        body = "".join("?%s_" % q for q in queries)
        self.send("# C_%s# %d" % (body, interval_ms))
        self._running = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def stop_stream(self):
        if self.ser and self.ser.is_open:
            try:
                self.send("#")
            except Exception:
                pass

    def stop_reader(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None

    def _reader(self):
        while self._running:
            try:
                chunk = self.ser.read(256)
            except Exception:
                break
            if not chunk:
                continue
            self._buf += chunk
            while b"\r" in self._buf:
                line, self._buf = self._buf.split(b"\r", 1)
                self._handle(line.decode("ascii", "replace").strip())

    def _handle(self, line):
        if "=" not in line:
            return
        key, _, val = line.partition("=")
        key = key.strip()
        if key not in ("CB", "BS", "A", "BA", "V", "T", "FF"):
            return
        self._latest[key] = (time.monotonic(), parse_values(val))

    def snapshot(self):
        now = time.monotonic()
        return {k: (t, list(v)) for k, (t, v) in self._latest.items()}, now

    def wait_for_stream(self, timeout=2.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if "CB" in self._latest or "BS" in self._latest:
                return True
            time.sleep(0.05)
        return False


@dataclass
class Segment:
    phase: str
    c1: int
    c2: int
    duration: float
    label: str = ""


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def build_suite(tests, max_duty, reverse, hold, settle):
    segs = []

    def add(phase, c1, c2, dur, label=""):
        c1 = clamp(int(round(c1)), -max_duty, max_duty)
        c2 = clamp(int(round(c2)), -max_duty, max_duty)
        segs.append(Segment(phase, c1, c2, dur, label))

    if "env" in tests:
        add("idle", 0, 0, settle, "idle")

    if "sweep" in tests:
        duties = [50, 100, 150, 200, 250, 300, 400, 500, 600, 700, 800, 900, 1000]
        for d in duties:
            add("sweep_both_up", d, d, hold, "d=%d" % d)
        for d in reversed(duties):
            add("sweep_both_down", d, d, hold, "d=%d" % d)
        for d in [100, 200, 300, 500, 700, 900, 1000]:
            add("sweep_ch1", d, 0, hold, "d=%d" % d)
        for d in [100, 200, 300, 500, 700, 900, 1000]:
            add("sweep_ch2", 0, d, hold, "d=%d" % d)
        add("sweep_both_up", 0, 0, settle, "d=0")
        if reverse:
            for d in [200, 400, 600, 800, 1000]:
                add("sweep_both_rev", -d, -d, hold, "d=%d" % d)
            add("sweep_both_rev", 0, 0, settle, "d=0")

    if "step" in tests:
        for target in [300, 600, 900]:
            add("step_base", 0, 0, 1.0, "base")
            add("step_up", target, target, 2.5, "d=%d" % target)
            add("step_base", 0, 0, 1.0, "base")
        for target in [300, 600, 900]:
            add("step_ch1", 0, 0, 1.0, "base")
            add("step_ch1", target, 0, 2.5, "d=%d" % target)
            add("step_ch1", 0, 0, 1.0, "base")
        for target in [300, 600, 900]:
            add("step_ch2", 0, 0, 1.0, "base")
            add("step_ch2", 0, target, 2.5, "d=%d" % target)
            add("step_ch2", 0, 0, 1.0, "base")

    if "square" in tests:
        for _ in range(6):
            add("square", 200, 200, 0.5, "lo")
            add("square", 800, 800, 0.5, "hi")
        add("square", 0, 0, settle, "stop")

    if "ramp" in tests:
        n = 80
        for i in range(n + 1):
            add("ramp_up", 1000.0 * i / n, 1000.0 * i / n, 0.1, "i=%d" % i)
        add("ramp_up", max_duty, max_duty, 2.0, "hold_max")
        for i in range(n + 1):
            add("ramp_down", max_duty * (1.0 - i / n),
                max_duty * (1.0 - i / n), 0.1, "i=%d" % i)
        add("ramp_down", 0, 0, settle, "stop")

    if "supply" in tests:
        add("supply_idle", 0, 0, 2.0, "idle")
        add("supply_both_hi", 800, 800, 4.0, "d=800")
        add("supply_ch1_hi", 800, 0, 4.0, "d=800")
        add("supply_ch2_hi", 0, 800, 4.0, "d=800")
        add("supply_both_max", 1000, 1000, 4.0, "d=1000")
        add("supply_idle", 0, 0, settle, "idle")

    return segs


class CsvLogger:
    FIELDS = [
        "t", "wall", "phase", "label", "cmd1", "cmd2",
        "cb1", "cb2", "bs1", "bs2", "a1", "a2",
        "ba1", "ba2", "v_batt", "v_int", "v_5v", "t_mcu", "ff",
    ]

    def __init__(self, path):
        self.path = path
        self.fh = open(path, "w", newline="")
        self.writer = csv.DictWriter(self.fh, fieldnames=self.FIELDS)
        self.writer.writeheader()
        self._last_flush = time.monotonic()

    def write(self, row):
        self.writer.writerow(row)
        if time.monotonic() - self._last_flush > 1.0:
            self.fh.flush()
            self._last_flush = time.monotonic()

    def close(self):
        try:
            self.fh.flush()
            self.fh.close()
        except Exception:
            pass


def pick(vals, idx, scale=1.0):
    if not vals or idx >= len(vals):
        return ""
    return round(vals[idx] * scale, 4)


def run(args):
    rob = RoboteqSerial(args.port, args.baud, args.stream_ms)
    logger = None
    meta = {}
    stopped = {"done": False}

    def cleanup():
        try:
            rob.send("!G 1 0")
            rob.send("!G 2 0")
            rob.send("!S 1 0")
            rob.send("!S 2 0")
        except Exception:
            pass
        try:
            rob.stop_stream()
        except Exception:
            pass

    def on_signal(*_):
        if stopped["done"]:
            return
        stopped["done"] = True
        sys.stderr.write("\n[ID] 中断: モータ停止 (!G 0) を送信します\n")
        cleanup()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        rob.open()
        cfg = rob.read_config()
        print("FID             :", cfg.get("FID"))
        print("MMOD            :", cfg.get("MMOD1"), cfg.get("MMOD2"))
        print("MXRPM           :", cfg.get("MXRPM1"), cfg.get("MXRPM2"))
        print("ALIM (0.1A)     :", cfg.get("ALIM1"), cfg.get("ALIM2"))
        print("EPPR            :", cfg.get("EPPR1"), cfg.get("EPPR2"))
        print("EMOD            :", cfg.get("EMOD1"), cfg.get("EMOD2"))
        print("KP/KI/KD        :", cfg.get("KP1"), cfg.get("KI1"), cfg.get("KD1"))
        print("RWD (ms)        :", cfg.get("RWD1"))

        if cfg.get("MMOD1") not in (None, "0") or cfg.get("MMOD2") not in (None, "0"):
            print("警告: MMOD が 0(オープンループ) ではない。!G は速度指令として解釈されます。")
            if not args.force_open_loop:
                print("      --force-open-loop を付けると ^MMOD x 0 に一時変更します。")
        if args.force_open_loop:
            rob.send("^MMOD 1 0")
            rob.send("^MMOD 2 0")
            print("MMOD をオープンループ(0)に設定しました。")

        alim = None
        try:
            alim = float(cfg.get("ALIM1", "0")) / 10.0
        except ValueError:
            alim = None

        rob.start_stream(interval_ms=args.stream_ms)
        if not rob.wait_for_stream(2.0):
            rob.stop_reader()
            print("エラー: ストリームを受信できません。--poll を試してください。")
            return 1

        if args.dry_run:
            print("\n[元データ確認] 1秒分のストリーム値（モータ指令なし）")
            t0 = time.monotonic()
            while time.monotonic() - t0 < 1.0:
                snap, now = rob.snapshot()
                print({k: v[1] for k, v in snap.items()})
                time.sleep(0.2)
            return 0

        segs = build_suite(set(args.tests), args.max_duty, args.reverse,
                           args.hold, args.settle)
        total = sum(s.duration for s in segs)
        print("\n実験セグメント数: %d  合計時間: 約 %.1f 秒" % (len(segs), total))
        if not args.yes:
            ans = input("宙吊りを確認し、開始しますか? [yes/N]: ").strip().lower()
            if ans not in ("y", "yes"):
                print("中止しました。")
                return 0

        os.makedirs(args.outdir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(args.outdir, "id_%s.csv" % stamp)
        meta_path = os.path.join(args.outdir, "id_%s.json" % stamp)
        logger = CsvLogger(csv_path)
        meta = {
            "timestamp": stamp,
            "port": args.port,
            "baud": args.baud,
            "stream_ms": args.stream_ms,
            "max_duty": args.max_duty,
            "tests": list(set(args.tests)),
            "reverse": args.reverse,
            "config": cfg,
            "alim_amps": alim,
            "segments": [s.__dict__ for s in segs],
        }
        with open(meta_path, "w") as fh:
            json.dump(meta, fh, indent=2, ensure_ascii=False)

        print("CSV : %s" % csv_path)
        print("META: %s" % meta_path)

        start = time.monotonic()
        row = {f: "" for f in CsvLogger.FIELDS}
        for seg in segs:
            if stopped["done"]:
                break
            seg_end = time.monotonic() + seg.duration
            rob.send("!G 1 %d" % seg.c1)
            rob.send("!G 2 %d" % seg.c2)
            last_cmd = time.monotonic()
            print("[%6.1fs] %-16s %-10s cmd=(%4d,%4d) %.1fs" % (
                time.monotonic() - start, seg.phase, seg.label,
                seg.c1, seg.c2, seg.duration))
            while time.monotonic() < seg_end:
                if stopped["done"]:
                    break
                now = time.monotonic()
                if now - last_cmd >= args.keepalive:
                    rob.send("!G 1 %d" % seg.c1)
                    rob.send("!G 2 %d" % seg.c2)
                    last_cmd = now
                snap, _ = rob.snapshot()
                cb = snap.get("CB", (0, []))[1]
                bs = snap.get("BS", (0, []))[1]
                a = snap.get("A", (0, []))[1]
                ba = snap.get("BA", (0, []))[1]
                v = snap.get("V", (0, []))[1]
                t = snap.get("T", (0, []))[1]
                ff = snap.get("FF", (0, []))[1]
                row.update({
                    "t": round(now - start, 4),
                    "wall": datetime.now().isoformat(timespec="milliseconds"),
                    "phase": seg.phase, "label": seg.label,
                    "cmd1": seg.c1, "cmd2": seg.c2,
                    "cb1": pick(cb, 0), "cb2": pick(cb, 1),
                    "bs1": pick(bs, 0), "bs2": pick(bs, 1),
                    "a1": pick(a, 0, 0.1), "a2": pick(a, 1, 0.1),
                    "ba1": pick(ba, 0, 0.1), "ba2": pick(ba, 1, 0.1),
                    "v_batt": pick(v, 1, 0.1), "v_int": pick(v, 0, 0.1),
                    "v_5v": pick(v, 2, 0.001), "t_mcu": pick(t, 0),
                    "ff": pick(ff, 0),
                })
                logger.write(row)
                time.sleep(1.0 / args.rate)

        rob.send("!G 1 0")
        rob.send("!G 2 0")
        print("\n完了。CSV: %s" % csv_path)
        print("解析: python3 analyze_id.py %s" % csv_path)
        return 0

    finally:
        cleanup()
        if logger:
            logger.close()
        rob.close()


def main():
    ap = argparse.ArgumentParser(
        description="Roboteq車輪モータ同定(空転). 詳細はファイル冒頭のdocstring参照.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--stream-ms", type=int, default=DEFAULT_STREAM_MS,
                    help="ストリーム間隔[ms]")
    ap.add_argument("--tests", default="env,sweep,step,square",
                    help="カンマ区切り: env,sweep,step,square,ramp,supply")
    ap.add_argument("--max-duty", type=int, default=1000,
                    help="指令dutyの絶対値上限(0-1000)")
    ap.add_argument("--hold", type=float, default=2.0, help="掃引1点の保持[s]")
    ap.add_argument("--settle", type=float, default=2.0, help="停止区間[s]")
    ap.add_argument("--rate", type=float, default=100.0, help="ログ周波数[Hz]")
    ap.add_argument("--keepalive", type=float, default=0.1,
                    help="ウォッチドッグ用の指令再送間隔[s]")
    ap.add_argument("--reverse", action="store_true", help="後退も同定する")
    ap.add_argument("--force-open-loop", action="store_true",
                    help="^MMOD x 0 に一時変更する")
    ap.add_argument("--dry-run", action="store_true",
                    help="モータを回さずストリーム値だけ表示")
    ap.add_argument("--yes", action="store_true", help="開始確認をスキップ")
    ap.add_argument("--outdir", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "results"))
    args = ap.parse_args()
    args.tests = [t.strip() for t in args.tests.split(",") if t.strip()]
    sys.exit(run(args))


if __name__ == "__main__":
    main()
