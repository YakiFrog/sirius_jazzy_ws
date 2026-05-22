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
_LOGGER = logging.getLogger("test_plaintext")

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

    _LOGGER.info("Connected. Sending PLAINTEXT query...")
    # build a plaintext packet using device._build_packet
    # payload is "a10121" + timestamp
    # We can use the device's own timestamp builder
    # Since C300 overrides nothing in _send_command except the bypass, let's manually send a plaintext packet
    from SolixBLE.const import BASE_TIMESTAMP
    import time
    negotiation_time = device._negotiation_timestamp if device._negotiation_timestamp is not None else time.time()
    time_passed = int(time.time() - negotiation_time)
    base_timestamp = int.from_bytes(bytes.fromhex(BASE_TIMESTAMP), byteorder="little")
    new_timestamp = (base_timestamp + time_passed).to_bytes(length=4, byteorder="little")
    raw_payload = bytes.fromhex("a10121") + bytes.fromhex("fe0503") + new_timestamp

    packet = device._build_packet(bytes.fromhex("03000f"), bytes.fromhex("4040"), raw_payload)
    _LOGGER.info(f"Sending plaintext query packet: {packet.hex()}")
    await device._client.write_gatt_char(UUID_COMMAND:="8c850002-0302-41c5-b46e-cf057c562025", packet)

    _LOGGER.info("Query sent. Listening for 20 seconds...")
    await asyncio.sleep(20.0)

    _LOGGER.info("Disconnecting...")
    await device.disconnect()
    _LOGGER.info("Done.")

if __name__ == "__main__":
    asyncio.run(main())
