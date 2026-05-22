import asyncio
import logging
import sys
from SolixBLE import C300
import time
import hashlib
from Crypto.Cipher import AES
from cryptography.hazmat.primitives.padding import PKCS7
from bleak import BleakScanner, BleakClient

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(message)s",
    stream=sys.stdout
)
_LOGGER = logging.getLogger("diagnose_c300")

UUID_COMMAND = "8c850002-0302-41c5-b46e-cf057c562025"
UUID_NOTIFICATION = "8c850003-0302-41c5-b46e-cf057c562025"

# Fallback shared secret
SHARED_SECRET = hashlib.sha256(b"Solix").digest()
AES_KEY = SHARED_SECRET[:16]
AES_IV = SHARED_SECRET[16:]

def decrypt_payload(payload: bytes) -> bytes:
    try:
        cipher = AES.new(AES_KEY, AES.MODE_CBC, iv=AES_IV)
        decrypted = cipher.decrypt(payload)
        unpadder = PKCS7(128).unpadder()
        unpadded = unpadder.update(decrypted) + unpadder.finalize()
        return unpadded
    except Exception as e:
        return f"DECRYPTION_ERROR: {e}".encode()

def encrypt_payload(payload: bytes) -> bytes:
    padder = PKCS7(128).padder()
    padded = padder.update(payload) + padder.finalize()
    cipher = AES.new(AES_KEY, AES.MODE_CBC, iv=AES_IV)
    return cipher.encrypt(padded)

def checksum(packet: bytes) -> bytes:
    val = 0
    for b in packet:
        val ^= b
    return val.to_bytes(1, byteorder="little")

def build_packet(pattern: bytes, cmd: bytes, payload: bytes) -> bytes:
    length = 2 + 2 + 3 + 2 + len(payload) + 1
    length_bytes = length.to_bytes(length=2, byteorder="little")
    packet = bytes.fromhex("ff09") + length_bytes + pattern + cmd + payload
    return packet + checksum(packet)

def split_packet(packet: bytes) -> tuple[bytes, bytes, bytes]:
    # header(2) + len(2) + pattern(3) + cmd(2) + payload(n) + checksum(1)
    pattern = packet[4:7]
    cmd = packet[7:9]
    payload = packet[9:-1]
    return pattern, cmd, payload

negotiation_timestamp = None

def build_command_payload(payload: bytes) -> bytes:
    global negotiation_timestamp
    if negotiation_timestamp is None:
        negotiation_timestamp = time.time()
    time_passed = int(time.time() - negotiation_timestamp)
    base_timestamp = int.from_bytes(bytes.fromhex("5b7dd41a"), byteorder="little")
    new_timestamp = (base_timestamp + time_passed).to_bytes(length=4, byteorder="little")
    return payload + bytes.fromhex("fe0503") + new_timestamp

async def run_diagnostics(mac_address):
    _LOGGER.info(f"Scanning for {mac_address}...")
    ble_device = await BleakScanner.find_device_by_address(mac_address, timeout=5.0)
    if not ble_device:
        _LOGGER.error("Device not found.")
        return

    _LOGGER.info("Connecting...")
    async with BleakClient(ble_device) as client:
        _LOGGER.info("Connected!")
        
        # Setup notification handler
        def notification_handler(sender, data):
            data_bytes = bytes(data)
            _LOGGER.info(f"--- NOTIFICATION: {data_bytes.hex()}")
            try:
                pattern, cmd, payload = split_packet(data_bytes)
                _LOGGER.info(f"Split: pattern={pattern.hex()} cmd={cmd.hex()} payload={payload.hex()}")
                # Try decrypting
                decrypted = decrypt_payload(payload)
                _LOGGER.info(f"Attempt Decrypt: {decrypted.hex()} (ASCII: {decrypted})")
            except Exception as e:
                _LOGGER.error(f"Error handling notification: {e}")

        await client.start_notify(UUID_NOTIFICATION, notification_handler)

        # Let's perform negotiation stage 1 to 5
        # We write Stage 0 (auto request) or wait for device
        # Usually we just write NEGOTIATION_COMMAND_1 to initiate
        # Wait, the library does this on connect:
        # Let's write the standard negotiation sequence:
        _LOGGER.info("Sending negotiation request (Stage 0)...")
        # In SolixBLE, Stage 0 is:
        # self._client.write_gatt_char(UUID_COMMAND, bytes.fromhex("ff090d000300010001a102000000"))
        # which is pattern=030001 cmd=0001 payload=a1020000
        await client.write_gatt_char(UUID_COMMAND, bytes.fromhex("ff090d000300010001a102000000"))

        # Wait for negotiation to complete (5 seconds)
        # We will see notifications logged by our handler and handle them if needed.
        # But wait! If we do it manually, we have to respond to notifications.
        # Since we just want to see how the device behaves, let's let the actual SolixBLE class do the connection and negotiation,
        # and we just inspect/use its connection.
        # So we can just use the C300 class, but override _send_encrypted_packet and _process_notification.

async def main():
    # We will use the C300 class to do negotiation automatically, and then perform our diagnostic commands.
    mac_address = "F4:9D:8A:7E:97:24"
    _LOGGER.info("Scanning via BleakScanner...")
    ble_device = await BleakScanner.find_device_by_address(mac_address, timeout=5.0)
    if not ble_device:
        _LOGGER.error("Device not found.")
        return

    device = C300(ble_device)

    # Patch notification processing to show raw and decrypted
    async def custom_process_notification(client_obj, handle, data):
        _LOGGER.info(f"\n========================================\nRAW NOTIFICATION: handle={handle}, hex={data.hex()}")
        try:
            pattern, cmd, payload = device._split_packet(data)
            _LOGGER.info(f"Parsed: pattern={pattern.hex()}, cmd={cmd.hex()}, payload={payload.hex()}")
            
            # Show decrypted payload if decrypted
            dec = decrypt_payload(payload)
            _LOGGER.info(f"Decrypt using 'Solix' key: {dec.hex()}")
            try:
                _LOGGER.info(f"Parsed parameters from decrypted: {device._parameters_to_str(device._parse_payload(dec))}")
            except Exception as e:
                _LOGGER.info(f"Failed to parse decrypted payload: {e}")

            # Try plaintext parsing
            try:
                _LOGGER.info(f"Parsed parameters from plaintext: {device._parameters_to_str(device._parse_payload(payload))}")
            except Exception as e:
                _LOGGER.info(f"Failed to parse plaintext payload: {e}")
        except Exception as e:
            _LOGGER.error(f"Error splitting/parsing notification: {e}")
        
        # Forward to original notification handler
        await orig_process_notification(client_obj, handle, data)

    orig_process_notification = device._process_notification
    device._process_notification = custom_process_notification

    _LOGGER.info("Connecting & negotiating using C300 class...")
    if not await device.connect():
        _LOGGER.error("Failed to connect/negotiate.")
        return

    _LOGGER.info("Connected & negotiated!")
    # Wait a bit
    await asyncio.sleep(2)

    # 1. Test PLAINTEXT query command
    # cmd=4040, pattern=03000f, payload=a10121 + timestamp
    # Let's send it in plaintext (unencrypted)
    raw_payload = build_command_payload(bytes.fromhex("a10121"))
    plaintext_packet = build_packet(bytes.fromhex("03000f"), bytes.fromhex("4040"), raw_payload)
    _LOGGER.info(f"Sending PLAINTEXT query: packet={plaintext_packet.hex()}")
    await device._client.write_gatt_char(UUID_COMMAND, plaintext_packet)
    _LOGGER.info("PLAINTEXT query sent. Waiting 4 seconds for response...")
    await asyncio.sleep(4)

    # 2. Test ENCRYPTED query command
    # Same payload, but AES-CBC encrypted using SHA256('Solix') key
    encrypted_payload = encrypt_payload(raw_payload)
    encrypted_packet = build_packet(bytes.fromhex("03000f"), bytes.fromhex("4040"), encrypted_payload)
    _LOGGER.info(f"Sending ENCRYPTED query: packet={encrypted_packet.hex()}")
    await device._client.write_gatt_char(UUID_COMMAND, encrypted_packet)
    _LOGGER.info("ENCRYPTED query sent. Waiting 4 seconds for response...")
    await asyncio.sleep(4)

    _LOGGER.info("Disconnecting...")
    await device.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
