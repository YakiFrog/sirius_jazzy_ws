#!/usr/bin/env python3
import asyncio  # 非同期処理用
import sys  # システム関連
import argparse  # コマンドライン引数解析
import logging  # ロギング
from bleak import BleakScanner  # BLEスキャナ
from SolixBLE import C300, discover_devices  # Anker SOLIX C300用BLEライブラリ

async def scan_solix_devices(timeout=10):
    """
    Anker SOLIXデバイスをスキャンする関数
    :param timeout: スキャンのタイムアウト秒数
    :return: 見つかったデバイスのリスト
    """
    print(f"Scanning for Anker Solix devices (timeout: {timeout}s)...")
    try:
        devices = await discover_devices(timeout=timeout)  # デバイスをスキャン
        if not devices:
            print("No Solix devices found. Please ensure:")
            print("  1. The Anker SOLIX C300 is nearby.")
            print("  2. Bluetooth/IoT button on the power station is turned on.")
            print("  3. No other device (like your phone app) is currently connected to it via Bluetooth.")
            return []
        # 見つかったデバイスを表示
        print(f"\nFound {len(devices)} Solix device(s):")
        for i, dev in enumerate(devices):
            print(f"  [{i}] Name: {dev.name}  |  MAC Address: {dev.address}")
        return devices
    except Exception as e:
        print(f"Error during BLE scanning: {e}", file=sys.stderr)
        return []

async def monitor_device(mac_address, interval=5, single_shot=False):
    """
    指定したMACアドレスのSOLIX C300に接続し、状態を監視する関数
    :param mac_address: 接続するデバイスのMACアドレス
    :param interval: 更新間隔（秒）
    :param single_shot: 1回だけ取得して終了する場合はTrue
    """
    print(f"Scanning to resolve BLE Device with address {mac_address}...")
    ble_device = await BleakScanner.find_device_by_address(mac_address, timeout=5.0)  # BLEデバイスをスキャン
    if not ble_device:
        print("Warning: Could not find BLE device via scan. Attempting direct connection bypass...")
        from bleak.backends.device import BLEDevice
        # スキャンで見つからない場合は直接BLEデバイス情報を生成
        details = {
            'path': f'/org/bluez/hci0/dev_{mac_address.replace(":", "_")}',
            'props': {
                'Address': mac_address,
                'AddressType': 'public',
                'Name': 'Anker SOLIX C300',
                'Alias': 'Anker SOLIX C300',
                'Paired': False,
                'Bonded': False,
                'Trusted': False,
                'Blocked': False,
                'LegacyPairing': False,
                'RSSI': -99,
                'Connected': False,
                'UUIDs': [],
                'Adapter': '/org/bluez/hci0',
                'ServicesResolved': False
            }
        }
        ble_device = BLEDevice(address=mac_address, name="Anker SOLIX C300", details=details, rssi=-99)

    print(f"Initializing connection to Anker SOLIX C300 ({mac_address})...")
    power_station = C300(ble_device)  # C300クラスのインスタンス生成

    try:
        print("Connecting via Bluetooth...")
        print("Resolving BLE services and negotiating keys. Please wait, this can take up to 30 seconds...")
        if not await power_station.connect():  # BLE接続と鍵交換
            print("Failed to connect/negotiate with the device. Exiting.")
            return
        print("Connected successfully! Waiting for initial telemetry data...")
        
        # 監視ループ
        while True:
            # バッテリーの状態更新を要求
            try:
                await power_station.get_status_update()
            except Exception as e:
                print(f"Error requesting update: {e}", file=sys.stderr)

            # ステータス情報を表示
            print("\n==========================================")
            print("   Anker SOLIX C300 Status")
            print("==========================================")
            print(f"  Device Name:      {power_station.name}")
            print(f"  Serial Number:    {power_station.serial_number}")
            print(f"  Software Version: {power_station.software_version}")
            print(f"  Battery Level:    {power_station.battery_percentage}%")
            print(f"  Temperature:      {power_station.temperature}°C")
            print(f"  Total Input:      {power_station.power_in}W")
            print(f"    - AC Input:     {power_station.ac_power_in}W")
            print(f"    - Solar Input:  {power_station.solar_power_in}W")
            print(f"  Total Output:     {power_station.power_out}W")
            print(f"    - AC Output:    {power_station.ac_power_out}W")
            print(f"    - DC Output:    {power_station.dc_power_out}W")
            print(f"  Charging Status:  {power_station.charging_status}")
            print(f"  USB-C1 Port:      {power_station.usb_c1_power}W ({'ON' if power_station.usb_port_c1 else 'OFF'})")
            print(f"  USB-C2 Port:      {power_station.usb_c2_power}W ({'ON' if power_station.usb_port_c2 else 'OFF'})")
            print(f"  USB-C3 Port:      {power_station.usb_c3_power}W ({'ON' if power_station.usb_port_c3 else 'OFF'})")
            print(f"  USB-A1 Port:      {power_station.usb_a1_power}W ({'ON' if power_station.usb_port_a1 else 'OFF'})")
            print("==========================================\n")

            if single_shot:
                break  # 1回だけ取得して終了

            print(f"Waiting {interval} seconds before next update... (Press Ctrl+C to exit)")
            await asyncio.sleep(interval)  # 指定秒数待機

    except asyncio.CancelledError:
        print("\nMonitoring stopped by user.")
    except Exception as e:
        print(f"\nAn error occurred: {e}", file=sys.stderr)
    finally:
        if power_station.connected:
            print("Disconnecting from device...")
            await power_station.disconnect()
            print("Disconnected.")

async def main():
    """
    メイン関数。コマンドライン引数を解析し、スキャンまたは監視を実行
    """
    parser = argparse.ArgumentParser(description="Anker SOLIX C300 BLE Battery Monitor")
    parser.add_argument("--scan", action="store_true", help="近くのAnker Solixデバイスをスキャンして終了")
    parser.add_argument("--mac", type=str, help="接続するAnker SOLIX C300のMACアドレス")
    parser.add_argument("--interval", type=int, default=5, help="更新間隔（秒、デフォルト: 5）")
    parser.add_argument("--once", action="store_true", help="1回だけ状態を表示して終了")
    parser.add_argument("--debug", action="store_true", help="デバッグログを有効化")
    args = parser.parse_args()

    if args.debug:
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
        )
        # bleakのD-Busログを抑制し、SolixBLEのみデバッグ表示
        logging.getLogger("SolixBLE").setLevel(logging.DEBUG)
    else:
        # デバッグでなければbleakの警告を抑制し、重大な問題のみ表示
        logging.basicConfig(
            level=logging.WARNING,
            format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
        )

    # 引数がなければスキャンを実行
    if args.scan or not args.mac:
        devices = await scan_solix_devices()
        if not args.scan and devices:
            print("\nヒント: '--mac <address>' オプションで特定のデバイスに接続できます。")
            print("例: ./solix_monitor.py --mac " + devices[0].address)
    else:
        await monitor_device(args.mac, interval=args.interval, single_shot=args.once)

if __name__ == "__main__":
    # スクリプトが直接実行された場合のエントリーポイント
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram terminated.")
