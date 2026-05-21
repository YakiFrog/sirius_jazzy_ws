#!/usr/bin/env python3
import asyncio
import sys
import logging
from bleak import BleakScanner
from SolixBLE import C300

# Set up logging to show debug statements from SolixBLE.
# This will expose "C300 plaintext telemetry detected. Bypassing decryption. Raw payload: <hex>"
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logging.getLogger("SolixBLE").setLevel(logging.DEBUG)
_LOGGER = logging.getLogger("TelemetryAnalyzer")

async def main():
    if len(sys.argv) < 2:
        print("Usage: ./analyze_telemetry.py <MAC_ADDRESS>")
        sys.exit(1)
        
    mac_address = sys.argv[1]
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

    print(f"Initializing C300 connection to {mac_address}...")
    power_station = C300(ble_device)

    # Define a callback to trigger when _data changes
    def on_state_changed():
        print("\n" + "="*50)
        print("   RAW TELEMETRY DATA RECEIVED")
        print("="*50)
        if power_station._data:
            print("Parsed Key-Value Pairs:")
            for key, val in sorted(power_station._data.items()):
                print(f"  Key: {key:<4} | Len: {len(val):<2} | Hex: {val.hex():<30} | Int (LE): {int.from_bytes(val, 'little'):<10} | Int (BE): {int.from_bytes(val, 'big'):<10}")
        else:
            print("  No parameters in power_station._data yet.")
        print("="*50 + "\n")

    power_station.add_callback(on_state_changed)

    try:
        print("Connecting via Bluetooth and initiating negotiation trigger...")
        if not await power_station.connect():
            print("Failed to connect/negotiate with the device. Exiting.")
            return
            
        print("Connected successfully! Listening passively for telemetry data. Press Ctrl+C to exit.")
        
        # Keep running to listen for notifications
        while True:
            await asyncio.sleep(1)

    except asyncio.CancelledError:
        print("\nMonitoring stopped by user.")
    except Exception as e:
        print(f"\nAn error occurred: {e}", file=sys.stderr)
    finally:
        if power_station.connected:
            print("Disconnecting from device...")
            await power_station.disconnect()
            print("Disconnected.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram terminated.")
