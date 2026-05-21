# XOR brute-force helper
encrypted_payload = bytes.fromhex("46d543e87674195355ebeb1bfb5a0cdb")
serial = b"AZVSB70F03400100"
mac = bytes.fromhex("f49d8a958b18")

keys = {
    "Serial": serial,
    "MAC": mac,
    "MAC reversed": mac[::-1],
    "Fixed Key (0x5A)": b"\x5A" * 16,
    "Fixed Key (0xA5)": b"\xA5" * 16,
}

for name, key in keys.items():
    decrypted = bytes([encrypted_payload[i] ^ key[i % len(key)] for i in range(len(encrypted_payload))])
    first_byte = decrypted[0]
    is_plausible = (first_byte == 0x00 or (first_byte & 0xF0 == 0xA0))
    
    print(f"Key: {name}")
    print(f"  Result Hex: {decrypted.hex()}")
    print(f"  Result ASCII: {repr(decrypted)}")
    if is_plausible:
        print("  >>> Plausible XOR structure detected! <<<")
    print("-" * 50)
