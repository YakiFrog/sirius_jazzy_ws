import hashlib
from Crypto.Cipher import AES

payload = bytes.fromhex("ec4950c89bc1d23561764a77ac282891")
serial_str = "AZVSB70E49500688"
serial = serial_str.encode()
mac = bytes.fromhex("f49d8a7e9724")

candidates = {
    "All Zeros 32B": b"\x00" * 32,
    "All Ones 32B": b"\x01" * 32,
    "SHA256(Serial)": hashlib.sha256(serial).digest(),
    "SHA256(Serial + MAC)": hashlib.sha256(serial + mac).digest(),
    "SHA256(MAC + Serial)": hashlib.sha256(mac + serial).digest(),
    "MD5(Serial) repeated": hashlib.md5(serial).digest() * 2,
    "MD5(Serial)": hashlib.md5(serial).digest() + b"\x00"*16,
    "SHA256(Anker)": hashlib.sha256(b"Anker").digest(),
    "SHA256(Solix)": hashlib.sha256(b"Solix").digest(),
    "Serial itself (padded)": serial + b"\x00"*16,
}

for name, secret in candidates.items():
    key = secret[:16]
    iv = secret[16:32]
    
    try:
        cipher = AES.new(key, AES.MODE_CBC, iv=iv)
        decrypted = cipher.decrypt(payload)
        
        # Check if the decrypted payload looks like valid TLV / parameter format
        # Typical starting byte is 0x00 (header strip target) or 0xa1, 0xa2 etc.
        first_byte = decrypted[0]
        # Simple heuristic: parameters starts with aX (like a1, a2...) or 00 or 01
        is_plausible = (first_byte == 0x00 or first_byte == 0x01 or (first_byte & 0xF0 == 0xA0))
        
        print(f"Secret: {name}")
        print(f"  Key: {key.hex()} | IV: {iv.hex()}")
        print(f"  Decrypted Hex: {decrypted.hex()}")
        print(f"  Decrypted ASCII: {repr(decrypted)}")
        if is_plausible:
            print("  >>> Plausible decrypted structure detected! <<<")
        print("-" * 50)
    except Exception as e:
        print(f"Failed with {name}: {e}")
