#!/usr/bin/env python3
"""
HWT905(WT901系)の融合軸モードを設定する。
  axis=6 -> 6軸(磁気なし, ジャイロ)  : FF AA 24 01 00
  axis=9 -> 9軸(磁気あり, 絶対ヨー)  : FF AA 24 00 00
事前に witmotion ノードを停止してポートを解放すること。
EEPROM保存はしない(電源再投入で戻る)。

使い方: python3 imu_set_axis.py 6|9 [--port /dev/wt905] [--baud 115200]
"""

import argparse
import time

import serial


def send(ser, data):
    ser.write(bytes(data))
    ser.flush()
    time.sleep(0.15)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("axis", choices=["6", "9"])
    ap.add_argument("--port", default="/dev/wt905")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    raw0 = 0x01 if args.axis == "6" else 0x00
    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        send(ser, [0xFF, 0xAA, 0x69, 0x88, 0xB5])      # unlock configuration
        send(ser, [0xFF, 0xAA, 0x24, raw0, 0x00])      # axis transition algorithm
        print("sent axis=%s (reg 0x24 raw0=0x%02X) on %s" % (args.axis, raw0, args.port))


if __name__ == "__main__":
    main()
