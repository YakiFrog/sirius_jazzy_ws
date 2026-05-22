#!/usr/bin/env python3
import asyncio
import sys
import argparse
import logging
from bleak import BleakScanner
from SolixBLE import C300, discover_devices

async def scan_solix_devices(timeout=10):
    print(f"Scanning for Anker Solix devices (timeout: {timeout}s)...")
    try:
        devices = await discover_devices(timeout=timeout)
        if not devices:
            print("No Solix devices found. Please ensure:")
            print("  1. The Anker SOLIX C300 is nearby.")
            print("  2. Bluetooth/IoT button on the power station is turned on.")
            print("  3. No other device (like your phone app) is currently connected to it via Bluetooth.")
            return []
        
        print(f"\nFound {len(devices)} Solix device(s):")
        for i, dev in enumerate(devices):
            print(f"  [{i}] Name: {dev.name}  |  MAC Address: {dev.address}")
        return devices
    except Exception as e:
        print(f"Error during BLE scanning: {e}", file=sys.stderr)
        return []

async def monitor_device(mac_address, interval=5, single_shot=False):
    print(f"Scanning to resolve BLE Device with address {mac_address}...")
    ble_device = await BleakScanner.find_device_by_address(mac_address, timeout=5.0)
    if not ble_device:
        print("Warning: Could not find BLE device via scan. Attempting direct connection bypass...")
        from bleak.backends.device import BLEDevice
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
    power_station = C300(ble_device)

    try:
        print("Connecting via Bluetooth...")
        print("Resolving BLE services and negotiating keys. Please wait, this can take up to 30 seconds...")
        if not await power_station.connect():
            print("Failed to connect/negotiate with the device. Exiting.")
            return
        print("Connected successfully! Waiting for initial telemetry data...")
        
        # Immediately start active polling and status reporting

        while True:
            # Send status update request to the battery
            try:
                await power_station.get_status_update()
            except Exception as e:
                print(f"Error requesting update: {e}", file=sys.stderr)

            # Output the status
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
                break

            print(f"Waiting {interval} seconds before next update... (Press Ctrl+C to exit)")
            await asyncio.sleep(interval)

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
    parser = argparse.ArgumentParser(description="Anker SOLIX C300 BLE Battery Monitor")
    parser.add_argument("--scan", action="store_true", help="Scan for nearby Anker Solix devices and exit")
    parser.add_argument("--mac", type=str, help="MAC address of the Anker SOLIX C300 to connect to")
    parser.add_argument("--interval", type=int, default=5, help="Update interval in seconds (default: 5)")
    parser.add_argument("--once", action="store_true", help="Print status once and exit")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    args = parser.parse_args()

    if args.debug:
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
        )
        # Only show debug logs for our library to suppress bleak D-Bus spam
        logging.getLogger("SolixBLE").setLevel(logging.DEBUG)
    else:
        # Suppress warnings from bleak if not in debug mode, but show critical issues
        logging.basicConfig(
            level=logging.WARNING,
            format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
        )

    # Default to scanning if no arguments provided
    if args.scan or not args.mac:
        devices = await scan_solix_devices()
        if not args.scan and devices:
            print("\nHint: Run this script with '--mac <address>' to connect to a specific device.")
            print("Example: ./solix_monitor.py --mac " + devices[0].address)
    else:
        await monitor_device(args.mac, interval=args.interval, single_shot=args.once)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram terminated.")
