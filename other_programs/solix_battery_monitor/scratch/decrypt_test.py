import hashlib
from Crypto.Cipher import AES

# Raw encrypted payload from 4840 notification
encrypted_payload = bytes.fromhex("46d543e87674195355ebeb1bfb5a0cdb")
serial_str = "AZVSB70F03400100"
serial = serial_str.encode()

md5_hash = hashlib.md5(serial).digest()
sha256_hash = hashlib.sha256(serial).digest()

keys = {
    "MD5 of Serial": md5_hash,
    "Serial itself": serial,
    "SHA256 (first 16B)": sha256_hash[:16],
    "SHA256 (last 16B)": sha256_hash[16:],
}

# Also try MD5 in Hex format (32B string -> first 16B / last 16B)
md5_hex = hashlib.md5(serial).hexdigest().encode()
keys["MD5 Hex (first 16B)"] = md5_hex[:16]
keys["MD5 Hex (last 16B)"] = md5_hex[16:]

# Add common test cases for IVs
for key_name, key_bytes in list(keys.items()):
    iv_candidates = {
        "All Zeros": b"\x00" * 16,
        "Same as Key": key_bytes,
        "Bitwise NOT of Key": bytes([b ^ 0xFF for b in key_bytes])
    }
    
    for iv_name, iv_bytes in iv_candidates.items():
        try:
            cipher = AES.new(key_bytes, AES.MODE_CBC, iv=iv_bytes)
            decrypted = cipher.decrypt(encrypted_payload)
            
            # Check if the decrypted payload looks like valid TLV / parameter format
            # Typical starting byte is 0x00 (header strip target) or 0xa1, 0xa2 etc.
            first_byte = decrypted[0]
            # Simple heuristic: parameters starts with aX (like a1, a2...) or 00
            is_plausible = (first_byte == 0x00 or (first_byte & 0xF0 == 0xA0))
            
            print(f"Key: {key_name} | IV: {iv_name}")
            print(f"  Decrypted Hex: {decrypted.hex()}")
            print(f"  Decrypted ASCII (rough): {repr(decrypted)}")
            if is_plausible:
                print("  >>> Plausible decrypted structure detected! <<<")
            print("-" * 50)
        except Exception as e:
            print(f"Failed Key: {key_name} | IV: {iv_name} -> {e}")
