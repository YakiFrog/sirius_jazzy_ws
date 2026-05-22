import asyncio
import logging
import sys
from bleak import BleakScanner
from SolixBLE import C300

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    stream=sys.stdout
)
_LOGGER = logging.getLogger("test_passive")

async def main():
    mac_address = "F4:9D:8A:7E:97:24"
    _LOGGER.info("Scanning for device...")
    ble_device = await BleakScanner.find_device_by_address(mac_address, timeout=5.0)
    if not ble_device:
        _LOGGER.error("Device not found.")
        return

    device = C300(ble_device)

    # Patch _process_notification to print everything
    orig_process = device._process_notification
    async def patched_process(client, handle, data):
        _LOGGER.info(f"NOTIFICATION: handle={handle}, hex={data.hex()}")
        try:
            pattern, cmd, payload = device._split_packet(data)
            _LOGGER.info(f"Parsed: pattern={pattern.hex()}, cmd={cmd.hex()}, payload={payload.hex()}")
        except Exception as e:
            _LOGGER.info(f"Split error: {e}")
        await orig_process(client, handle, data)
    device._process_notification = patched_process

    # Connection
    _LOGGER.info("Connecting...")
    if not await device.connect():
        _LOGGER.error("Connection failed.")
        return

    _LOGGER.info("Connected successfully! Listening passively for 25 seconds...")
    await asyncio.sleep(25.0)

    _LOGGER.info("Disconnecting...")
    await device.disconnect()
    _LOGGER.info("Done.")

if __name__ == "__main__":
    asyncio.run(main())
