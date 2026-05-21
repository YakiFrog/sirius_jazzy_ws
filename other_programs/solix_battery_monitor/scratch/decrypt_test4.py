import hashlib
from Crypto.Cipher import AES

encrypted_payload = bytes.fromhex("46d543e87674195355ebeb1bfb5a0cdb")
serial = "AZVSB70F03400100".encode()
mac = bytes.fromhex("f49d8a958b18")

candidates = {
    "All Zeros 32B": b"\x00" * 32,
    "All Ones 32B": b"\x01" * 32,
    "SHA256(Serial + MAC)": hashlib.sha256(serial + mac).digest(),
    "SHA256(MAC + Serial)": hashlib.sha256(mac + serial).digest(),
    "SHA256(Serial)": hashlib.sha256(serial).digest(),
    "MD5(Serial) repeated": hashlib.md5(serial).digest() * 2,
    "SHA256(Anker)": hashlib.sha256(b"Anker").digest(),
    "SHA256(Solix)": hashlib.sha256(b"Solix").digest(),
    "PC Public Key SHA256": hashlib.sha256(bytes.fromhex("060ea168f232aedb37fb2d120c49180329ac72ab5ec3eb8fd30a2f252dc5e151dabccd9b1dc1e288704ca760a0d8c918e5c94823a1f609a4bf07fb4c33ee2190")).digest()
}

for name, secret in candidates.items():
    key = secret[:16]
    iv = secret[16:32]
    
    try:
        cipher = AES.new(key, AES.MODE_CBC, iv=iv)
        decrypted = cipher.decrypt(encrypted_payload)
        
        first_byte = decrypted[0]
        is_plausible = (first_byte == 0x00 or (first_byte & 0xF0 == 0xA0))
        
        print(f"Secret: {name}")
        print(f"  Key: {key.hex()} | IV: {iv.hex()}")
        print(f"  Decrypted Hex: {decrypted.hex()}")
        print(f"  Decrypted ASCII: {repr(decrypted)}")
        if is_plausible:
            print("  >>> Plausible decrypted structure detected! <<<")
        print("-" * 50)
    except Exception as e:
        print(f"Failed with {name}: {e}")
