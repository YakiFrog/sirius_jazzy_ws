#!/usr/bin/env python3
"""
Pyroboteq サンプルプログラム

Roboteq モーターコントローラとの通信サンプルです。
基本的な接続、モーター制御、センサー読み取りの方法を示します。

使用方法:
    python3 sample_program.py [--port /dev/roboteq] [--debug]
"""

import sys
import time
import argparse

# pyroboteq モジュールのインポート
from roboteq_handler import RoboteqHandler
from roboteq_commands import (
    GO,                # モーターパワー指令 (!G)
    SET_SPEED,         # モーター速度指令 (!S)
    DUAL_DRIVE,        # デュアルドライブ (!M)
    READ_MOTOR_AMPS,   # モーター電流読み取り (?A)
    READ_VOLTS,        # 電圧読み取り (?V)
    READ_TEMP,         # 温度読み取り (?T)
    READ_BLCNTR,       # ブラシレスエンコーダカウンタ (?CB)
    READ_BL_MOTOR_RPM, # ブラシレスモーターRPM (?BS)
    READ_FLTFLAG,      # フォルトフラグ (?FF)
    EM_STOP,           # 緊急停止 (!EX)
    REL_EM_STOP,       # 緊急停止解除 (!MG)
    STOP_ALL,          # 全停止 (!MS)
)


def demo_basic_connection(handler: RoboteqHandler):
    """基本的な接続とステータス確認のデモ"""
    print("\n" + "=" * 50)
    print("【基本接続デモ】")
    print("=" * 50)
    
    # ファームウェアID読み取り
    response = handler.request_handler("?FID")
    print(f"ファームウェアID: {response}")
    
    # フォルトフラグ確認
    response = handler.read_value("?FF")
    print(f"フォルトフラグ: {response}")


def demo_read_sensors(handler: RoboteqHandler):
    """センサー値読み取りのデモ"""
    print("\n" + "=" * 50)
    print("【センサー読み取りデモ】")
    print("=" * 50)
    
    # 電圧読み取り (V * 10 で返ってくる)
    # パラメータ: 1=内部電圧, 2=バッテリー電圧, 3=5V出力電圧
    response = handler.read_value("?V", "2")
    print(f"バッテリー電圧: {response} (値/10 = 実際のV)")
    
    # 温度読み取り
    # パラメータ: 1=MCU温度, 2=チャンネル1ヒートシンク, 3=チャンネル2ヒートシンク
    response = handler.read_value("?T", "1")
    print(f"MCU温度: {response}°C")
    
    # モーター電流読み取り
    # パラメータ: 1=チャンネル1, 2=チャンネル2
    response = handler.read_value("?A", "1")
    print(f"モーター1電流: {response} (値/10 = 実際のA)")
    
    response = handler.read_value("?A", "2")
    print(f"モーター2電流: {response} (値/10 = 実際のA)")
    
    # エンコーダカウンタ読み取り (ブラシレス)
    response = handler.read_value("?CB", "1")
    print(f"エンコーダ1カウント: {response}")
    
    response = handler.read_value("?CB", "2")
    print(f"エンコーダ2カウント: {response}")


def demo_motor_control(handler: RoboteqHandler, power: int = 100):
    """モーター制御のデモ (注意: モーターが動きます！)"""
    print("\n" + "=" * 50)
    print("【モーター制御デモ】")
    print(f"パワー: {power} (-1000〜1000)")
    print("=" * 50)
    
    print("3秒後にモーターが動きます。安全を確認してください...")
    for i in range(3, 0, -1):
        print(f"  {i}...")
        time.sleep(1)
    
    # 方法1: GO コマンド (オープンループパワー)
    # !G <channel> <power>  power: -1000 〜 +1000
    print(f"\n[GO コマンド] モーター1にパワー {power} を送信")
    handler.send_command("!G", "1", str(power))
    handler.send_command("!G", "2", str(power))
    time.sleep(2)
    
    # 停止
    print("[停止] パワー 0 を送信")
    handler.send_command("!G", "1", "0")
    handler.send_command("!G", "2", "0")
    time.sleep(1)
    
    # 方法2: デュアルドライブモード
    # !M <left_power> <right_power>
    print(f"\n[DUAL_DRIVE] 左右に {power} を送信")
    handler.dual_motor_control(left_motor=power, right_motor=power)
    time.sleep(2)
    
    # 停止
    print("[停止]")
    handler.dual_motor_control(left_motor=0, right_motor=0)


def demo_encoder_monitoring(handler: RoboteqHandler, duration: float = 5.0):
    """エンコーダ値のモニタリングデモ"""
    print("\n" + "=" * 50)
    print(f"【エンコーダモニタリング】({duration}秒間)")
    print("=" * 50)
    
    start_time = time.time()
    while time.time() - start_time < duration:
        # 両方のエンコーダを読み取り
        enc1 = handler.read_value("?CB", "1")
        enc2 = handler.read_value("?CB", "2")
        
        print(f"\rエンコーダ: 右={enc1:>8s}  左={enc2:>8s}", end="", flush=True)
        time.sleep(0.1)
    
    print("\n完了")


def demo_emergency_stop(handler: RoboteqHandler):
    """緊急停止のデモ"""
    print("\n" + "=" * 50)
    print("【緊急停止デモ】")
    print("=" * 50)
    
    # 緊急停止発動
    print("緊急停止を発動: !EX")
    handler.send_command("!EX")
    time.sleep(1)
    
    # フォルトフラグ確認
    response = handler.read_value("?FF")
    print(f"フォルトフラグ: {response}")
    
    # 緊急停止解除
    print("緊急停止を解除: !MG")
    handler.send_command("!MG")
    time.sleep(0.5)
    
    # フォルトフラグ再確認
    response = handler.read_value("?FF")
    print(f"フォルトフラグ(解除後): {response}")


def interactive_mode(handler: RoboteqHandler):
    """対話モード - コマンドを直接入力できる"""
    print("\n" + "=" * 50)
    print("【対話モード】")
    print("コマンドを直接入力できます。")
    print("例: !G 1 100  (モーター1にパワー100)")
    print("例: ?V 2      (バッテリー電圧読み取り)")
    print("例: ?CB 1     (エンコーダ1読み取り)")
    print("'quit' または 'exit' で終了")
    print("=" * 50)
    
    while True:
        try:
            cmd = input("\n> ").strip()
            if cmd.lower() in ['quit', 'exit', 'q']:
                break
            if not cmd:
                continue
            
            response = handler.request_handler(cmd)
            print(f"応答: {response}")
            
        except KeyboardInterrupt:
            print("\n中断されました")
            break


def main():
    parser = argparse.ArgumentParser(description="Pyroboteq サンプルプログラム")
    parser.add_argument("--port", "-p", default="/dev/roboteq",
                        help="シリアルポート (デフォルト: /dev/roboteq)")
    parser.add_argument("--baud", "-b", type=int, default=115200,
                        help="ボーレート (デフォルト: 115200)")
    parser.add_argument("--debug", "-d", action="store_true",
                        help="デバッグモードを有効化")
    parser.add_argument("--mode", "-m", 
                        choices=["all", "sensors", "motor", "encoder", "interactive"],
                        default="sensors",
                        help="実行モード (デフォルト: sensors)")
    parser.add_argument("--power", type=int, default=100,
                        help="モーターテスト時のパワー値 (デフォルト: 100)")
    
    args = parser.parse_args()
    
    print("=" * 50)
    print("Pyroboteq サンプルプログラム")
    print("=" * 50)
    print(f"ポート: {args.port}")
    print(f"ボーレート: {args.baud}")
    print(f"デバッグモード: {args.debug}")
    print(f"実行モード: {args.mode}")
    
    # ハンドラー作成と接続
    handler = RoboteqHandler(exit_on_interrupt=False, debug_mode=args.debug)
    
    print(f"\n{args.port} に接続中...")
    if not handler.connect(args.port, args.baud):
        print("エラー: 接続に失敗しました")
        print("ポート名とデバイスの接続を確認してください")
        sys.exit(1)
    
    print("接続成功！")
    
    try:
        if args.mode == "all":
            demo_basic_connection(handler)
            demo_read_sensors(handler)
            demo_encoder_monitoring(handler, duration=3.0)
            # モーター制御は危険なのでコメントアウト
            # demo_motor_control(handler, args.power)
            
        elif args.mode == "sensors":
            demo_basic_connection(handler)
            demo_read_sensors(handler)
            
        elif args.mode == "motor":
            print("\n⚠️  警告: モーターが動きます！")
            confirm = input("続行しますか？ (yes/no): ")
            if confirm.lower() == "yes":
                demo_motor_control(handler, args.power)
            else:
                print("キャンセルしました")
                
        elif args.mode == "encoder":
            demo_encoder_monitoring(handler, duration=10.0)
            
        elif args.mode == "interactive":
            interactive_mode(handler)
    
    except KeyboardInterrupt:
        print("\n\n中断されました")
    
    finally:
        # 安全のため停止コマンドを送信
        print("\n停止コマンドを送信...")
        handler.send_command("!G", "1", "0")
        handler.send_command("!G", "2", "0")
        print("終了")


if __name__ == "__main__":
    main()
