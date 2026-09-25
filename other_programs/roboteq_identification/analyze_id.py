#!/usr/bin/env python3
"""
roboteq_id.py が出力した CSV を解析し、モータ/ドライブ/供給の特性を要約する。

使い方:
    python3 analyze_id.py results/id_YYYYmmdd_HHMMSS.csv
    python3 analyze_id.py results/id_*.csv --no-plots
"""

import argparse
import csv
import glob
import json
import os
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_MPL = True
except Exception:
    HAVE_MPL = False

try:
    from scipy.optimize import curve_fit
    HAVE_SCIPY = True
except Exception:
    HAVE_SCIPY = False

NUM_COLS = ["t", "cmd1", "cmd2", "cb1", "cb2", "bs1", "bs2",
            "a1", "a2", "ba1", "ba2", "v_batt", "v_int", "t_mcu", "ff"]

FF_BITS = {1: "Overheat", 2: "Overvoltage", 4: "Undervoltage", 8: "Short",
           16: "E-Stop", 32: "Motor/Sensor", 64: "MOSfail",
           128: "Self-config", 256: "STO"}


def decode_ff(v):
    try:
        v = int(v)
    except (TypeError, ValueError):
        return []
    return [name for bit, name in FF_BITS.items() if v & bit]


def to_f(v):
    if v is None or v == "":
        return np.nan
    try:
        return float(v)
    except ValueError:
        return np.nan


def load(path):
    rows = []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            for c in NUM_COLS:
                r[c] = to_f(r.get(c))
            rows.append(r)
    return rows


def add_cb_rpm(rows, eppr):
    if not eppr:
        return
    prev = None
    for r in rows:
        if prev is None:
            r["rpm1"] = np.nan
            r["rpm2"] = np.nan
        else:
            dt = r["t"] - prev["t"]
            if dt > 1e-6:
                r["rpm1"] = (r["cb1"] - prev["cb1"]) / eppr / dt * 60.0
                r["rpm2"] = (r["cb2"] - prev["cb2"]) / eppr / dt * 60.0
            else:
                r["rpm1"] = np.nan
                r["rpm2"] = np.nan
        prev = r


def contiguous_segments(rows):
    segs = []
    cur = None
    for r in rows:
        key = (r["phase"], r["label"], r["cmd1"], r["cmd2"])
        if cur is None or cur["key"] != key:
            cur = {"key": key, "rows": []}
            segs.append(cur)
        cur["rows"].append(r)
    return segs


def steady(seg, frac=0.4):
    rs = seg["rows"]
    if len(rs) < 5:
        return None
    k = max(1, int(len(rs) * (1.0 - frac)))
    win = rs[k:]
    out = {"n": len(win)}
    for c in ["cmd1", "cmd2", "bs1", "bs2", "a1", "a2", "ba1", "ba2",
              "v_batt", "v_int", "cb1", "cb2", "t_mcu", "ff"]:
        vals = np.array([r[c] for r in win], dtype=float)
        vals = vals[~np.isnan(vals)]
        out[c] = float(np.mean(vals)) if vals.size else np.nan
    out["ff_max"] = float(np.nanmax([r["ff"] for r in win])) if win else np.nan
    out["t0"] = win[0]["t"]
    out["t1"] = win[-1]["t"]
    return out


def cb_motor_rpm(seg, eppr):
    rs = seg["rows"]
    if len(rs) < 5 or not eppr:
        return np.nan, np.nan
    r0, r1 = rs[0], rs[-1]
    dt = r1["t"] - r0["t"]
    if dt <= 0:
        return np.nan, np.nan
    m1 = (r1["cb1"] - r0["cb1"]) / eppr / dt * 60.0
    m2 = (r1["cb2"] - r0["cb2"]) / eppr / dt * 60.0
    return m1, m2


def speed_series(seg, ch):
    bs = np.array([r.get("bs%d" % ch, np.nan) for r in seg["rows"]], float)
    rpm = np.array([r.get("rpm%d" % ch, np.nan) for r in seg["rows"]], float)
    if np.isfinite(bs).any() and np.nanmax(np.abs(bs)) > 0.5:
        return bs
    return rpm


def fit_first_order(t, y):
    y0 = float(np.nanmean(y[:max(1, len(y) // 10)]))
    yss = float(np.nanmean(y[-max(1, len(y) // 4):]))
    K = yss - y0
    if not HAVE_SCIPY or abs(K) < 1e-9:
        return None
    def model(tt, K, tau, off):
        return off + K * (1.0 - np.exp(-np.maximum(tt, 0) / max(tau, 1e-6)))
    try:
        p, _ = curve_fit(model, t, y, p0=[K, 0.15, y0],
                         bounds=([-1e9, 1e-4, -1e9], [1e9, 5.0, 1e9]),
                         maxfev=20000)
        return {"K": float(p[0]), "tau": float(p[1]), "offset": float(p[2])}
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="roboteq_id.py の出力CSV")
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--frac", type=float, default=0.4,
                    help="定常値に使う末尾割合")
    args = ap.parse_args()

    paths = sorted(glob.glob(args.csv)) if any(ch in args.csv for ch in "*?[") else [args.csv]
    for path in paths:
        analyze_one(path, args)
    return 0


def analyze_one(path, args):
    rows = load(path)
    if not rows:
        print("空のCSV:", path)
        return
    meta = None
    mpath = os.path.splitext(path)[0] + ".json"
    if os.path.exists(mpath):
        with open(mpath) as fh:
            meta = json.load(fh)
    eppr = 0.0
    alim = None
    if meta:
        try:
            eppr = float(meta["config"].get("EPPR1", "0") or 0)
        except Exception:
            eppr = 0.0
        alim = meta.get("alim_amps")

    add_cb_rpm(rows, eppr)
    segs = contiguous_segments(rows)
    print("=" * 78)
    print("file:", os.path.basename(path))
    print("rows:", len(rows), " segments:", len(segs))
    if alim:
        print("ALIM: %.1f A" % alim)

    print("\n--- 定常特性 (末尾%.0f%%平均) ---" % (args.frac * 100))
    print("%-16s %-10s %5s %5s | %7s %7s | %5s %5s | %6s | %s" % (
        "phase", "label", "cmd1", "cmd2", "bs1", "bs2",
        "a1", "a2", "v_batt", "ff"))
    recs = []
    for seg in segs:
        s = steady(seg, args.frac)
        if s is None:
            continue
        m1, m2 = cb_motor_rpm(seg, eppr)
        spd1 = s["bs1"] if (np.isfinite(s["bs1"]) and abs(s["bs1"]) > 0.5) else m1
        spd2 = s["bs2"] if (np.isfinite(s["bs2"]) and abs(s["bs2"]) > 0.5) else m2
        rec = {"phase": seg["key"][0], "label": seg["key"][1],
               "cmd1": s["cmd1"], "cmd2": s["cmd2"],
               "bs1": s["bs1"], "bs2": s["bs2"], "rpm1": m1, "rpm2": m2,
               "spd1": spd1, "spd2": spd2,
               "a1": s["a1"], "a2": s["a2"], "ba1": s["ba1"], "ba2": s["ba2"],
               "v_batt": s["v_batt"], "ff_max": s["ff_max"], "n": s["n"],
               "t0": s["t0"], "t1": s["t1"]}
        recs.append(rec)
        print("%-16s %-10s %5.0f %5.0f | %7.1f %7.1f | %5.2f %5.2f | %6.2f | %.0f" % (
            rec["phase"], rec["label"], rec["cmd1"], rec["cmd2"],
            rec["spd1"] or np.nan, rec["spd2"] or np.nan,
            rec["a1"] or np.nan, rec["a2"] or np.nan,
            rec["v_batt"] or np.nan, rec["ff_max"] or 0))

    summary = {"file": os.path.basename(path), "alim_amps": alim, "records": recs}

    def by_phase(prefix):
        return [r for r in recs if r["phase"].startswith(prefix)]

    def linear_fit(x, y):
        x = np.asarray(x, float)
        y = np.asarray(y, float)
        m = np.isfinite(x) & np.isfinite(y)
        x, y = x[m], y[m]
        if len(x) < 2:
            return np.nan, np.nan
        slope, inter = np.polyfit(x, y, 1)
        return float(slope), float(inter)

    print("\n--- デッドバンド / ゲイン ---")
    gains = {}
    for phase, speed_col in [("sweep_ch1", "spd1"), ("sweep_ch2", "spd2")]:
        pts = [(abs(r["cmd1"] if phase == "sweep_ch1" else r["cmd2"]), r[speed_col])
               for r in by_phase(phase)]
        pts = [(d, s) for d, s in pts if np.isfinite(s)]
        if not pts:
            continue
        maxs = max(s for _, s in pts)
        live = [d for d, s in pts if s > 0.05 * maxs]
        dead = min(live) if live else np.nan
        hi = [(d, s) for d, s in pts if d >= dead]
        slope, inter = linear_fit([p[0] for p in hi], [p[1] for p in hi])
        gains[phase] = {"deadband_duty": dead, "gain_rpm_per_duty": slope,
                        "intercept_rpm": inter, "max_rpm": maxs}
        print("  %-10s deadband~%3.0f  gain=%7.3f rpm/duty  max=%7.1f rpm" % (
            phase, dead, slope, maxs))
    summary["gains"] = gains

    print("\n--- 左右差 (sweep_both) ---")
    both = [(r["cmd1"], r["spd1"], r["spd2"]) for r in by_phase("sweep_both_up")
            if np.isfinite(r["spd1"]) and np.isfinite(r["spd2"])]
    ratios = []
    for d, s1, s2 in both:
        if abs(d) > 100 and abs(s2) > 1e-6:
            ratios.append(s1 / s2)
    if ratios:
        print("  spd1/spd2 (duty>100): mean=%.3f  min=%.3f  max=%.3f  n=%d" % (
            float(np.mean(ratios)), float(np.min(ratios)),
            float(np.max(ratios)), len(ratios)))
        summary["lr_ratio"] = {"mean": float(np.mean(ratios)),
                              "min": float(np.min(ratios)),
                              "max": float(np.max(ratios))}

    print("\n--- ステップ応答 (一次系 K, tau) ---")
    steps = {}
    for phase, ch in [("step_up", 1), ("step_ch1", 1), ("step_ch2", 2)]:
        for seg in segs:
            if seg["key"][0] != phase:
                continue
            rs = seg["rows"]
            t = np.array([r["t"] - rs[0]["t"] for r in rs], float)
            y = speed_series(seg, ch)
            good = np.isfinite(y)
            if good.sum() < 5:
                continue
            fit = fit_first_order(t[good], y[good])
            if fit:
                steps.setdefault(phase, []).append(fit)
                print("  %-9s %-8s K=%8.1f rpm  tau=%.3f s" % (
                    phase, seg["key"][1], fit["K"], fit["tau"]))
    summary["steps"] = steps

    print("\n--- 供給 / 電流制限 ---")
    max_a1 = np.nanmax(np.abs([r["a1"] for r in rows])) if rows else np.nan
    max_a2 = np.nanmax(np.abs([r["a2"] for r in rows])) if rows else np.nan
    max_ba1 = np.nanmax(np.abs([r["ba1"] for r in rows])) if rows else np.nan
    max_ba2 = np.nanmax(np.abs([r["ba2"] for r in rows])) if rows else np.nan
    max_v = np.nanmax([r["v_batt"] for r in rows]) if rows else np.nan
    min_v = np.nanmin([r["v_batt"] for r in rows if np.isfinite(r["v_batt"])]) if rows else np.nan
    idle_v = [r["v_batt"] for r in recs if "idle" in r["phase"]]
    idle_v = float(np.mean(idle_v)) if idle_v else float("nan")
    print("  max |motor A|   : ch1=%.2f  ch2=%.2f (ALIM=%s)" % (
        max_a1 if np.isfinite(max_a1) else -1,
        max_a2 if np.isfinite(max_a2) else -1, alim))
    print("  max |batt A|    : ch1=%.2f  ch2=%.2f" % (
        max_ba1 if np.isfinite(max_ba1) else -1,
        max_ba2 if np.isfinite(max_ba2) else -1))
    print("  v_batt max/min  : %.2f / %.2f V   (idle mean %.2f V, sag=%.2f V)" % (
        max_v if np.isfinite(max_v) else -1,
        min_v if np.isfinite(min_v) else -1, idle_v,
        idle_v - min_v if np.isfinite(min_v) else float("nan")))
    n_clip = sum(1 for r in rows if alim and
                 (abs(r["a1"]) >= 0.95 * alim or abs(r["a2"]) >= 0.95 * alim))
    print("  ALIM到達サンプル数: %d" % n_clip)
    ff_bad = [r for r in recs if r["ff_max"] and r["ff_max"] > 0]
    ff_seen = sorted({int(r["ff_max"]) for r in ff_bad})
    print("  フォルト発生セグメント: %d  値=%s" % (len(ff_bad), ff_seen))
    for v in ff_seen:
        print("    FF=%d -> %s" % (v, ",".join(decode_ff(v)) or "?"))
    summary["supply"] = {"max_a1": float(max_a1), "max_a2": float(max_a2),
                         "max_ba1": float(max_ba1), "max_ba2": float(max_ba2),
                         "v_max": float(max_v), "v_min": float(min_v),
                         "idle_v": idle_v, "sag_v": idle_v - min_v,
                         "alim_clip_samples": n_clip,
                         "ff_values": ff_seen}

    max_speed_duty = None
    best = None
    for r in by_phase("sweep_both_up") + by_phase("supply_both_max"):
        sp = max(r["spd1"] if np.isfinite(r["spd1"]) else -1,
                 r["spd2"] if np.isfinite(r["spd2"]) else -1)
        if best is None or sp > best[0]:
            best = (sp, r["cmd1"])
    if best:
        print("  最高速度        : %.1f rpm @ duty=%.0f" % best)
        summary["max_speed"] = {"rpm": best[0], "duty": best[1]}

    outdir = os.path.dirname(path)
    base = os.path.splitext(os.path.basename(path))[0]
    spath = os.path.join(outdir, base + "_summary.json")
    with open(spath, "w") as fh:
        json.dump(summary, fh, indent=2, ensure_ascii=False)
    print("\nsummary:", spath)

    if HAVE_MPL and not args.no_plots:
        make_plots(rows, segs, recs, eppr, outdir, base)
        print("plots  : %s/%s_*.png" % (outdir, base))
    return summary


def make_plots(rows, segs, recs, eppr, outdir, base):
    t = np.array([r["t"] for r in rows], float)

    def col(c):
        return np.array([r.get(c, np.nan) for r in rows], float)

    def legend(ax):
        h, l = ax.get_legend_handles_labels()
        if h:
            ax.legend()

    fig, ax = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    ax[0].plot(t, col("cmd1"), label="cmd1")
    ax[0].plot(t, col("cmd2"), label="cmd2")
    ax[0].set_ylabel("duty (cmd)")
    legend(ax[0]); ax[0].grid(True)
    ax[1].plot(t, col("bs1"), label="rpm ch1(?BS1)")
    ax[1].plot(t, col("bs2"), label="rpm ch2(?BS2)")
    ax[1].plot(t, col("rpm1"), "--", alpha=0.6, label="rpm ch1(CB)")
    ax[1].plot(t, col("rpm2"), "--", alpha=0.6, label="rpm ch2(CB)")
    ax[1].set_ylabel("motor rpm")
    legend(ax[1]); ax[1].grid(True)
    ax[2].plot(t, col("a1"), label="A ch1")
    ax[2].plot(t, col("a2"), label="A ch2")
    ax[2].set_ylabel("motor current [A]")
    legend(ax[2]); ax[2].grid(True)
    ax[3].plot(t, col("v_batt"), label="Vbatt")
    ax[3].plot(t, col("v_int"), label="Vint")
    ax[3].set_ylabel("voltage [V]"); ax[3].set_xlabel("t [s]")
    legend(ax[3]); ax[3].grid(True)
    fig.suptitle("Roboteq ID time series: " + base)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, base + "_timeseries.png"), dpi=110)
    plt.close(fig)

    fig, ax = plt.subplots(1, 2, figsize=(12, 5))
    for phase, xcol, ycol, lbl in [
            ("sweep_ch1", "cmd1", "spd1", "ch1"),
            ("sweep_ch2", "cmd2", "spd2", "ch2")]:
        pts = [(abs(r[xcol]), r[ycol]) for r in recs if r["phase"] == phase]
        pts = [(x, y) for x, y in pts if np.isfinite(y)]
        pts.sort()
        if pts:
            ax[0].plot([p[0] for p in pts], [p[1] for p in pts], "o-", label=lbl)
    ax[0].set_xlabel("duty"); ax[0].set_ylabel("rpm")
    ax[0].set_title("duty vs speed"); legend(ax[0]); ax[0].grid(True)

    for phase, ch in [("sweep_ch1", "a1"), ("sweep_ch2", "a2")]:
        pts = [(abs(r["cmd1"] if phase == "sweep_ch1" else r["cmd2"]), r[ch])
               for r in recs if r["phase"] == phase]
        pts = [(x, y) for x, y in pts if np.isfinite(y)]
        pts.sort()
        if pts:
            ax[1].plot([p[0] for p in pts], [p[1] for p in pts], "o-",
                       label=ch)
    ax[1].set_xlabel("duty"); ax[1].set_ylabel("current [A]")
    ax[1].set_title("duty vs current"); legend(ax[1]); ax[1].grid(True)
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, base + "_static.png"), dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
