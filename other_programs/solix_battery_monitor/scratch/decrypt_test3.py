from Crypto.Cipher import AES

# Raw encrypted payload from 4840 notification
encrypted_payload = bytes.fromhex("46d543e87674195355ebeb1bfb5a0cdb")

# The private key constant from device.py
private_key_hex = "7dfbea61cd95cee49c458ad7419e817f1ade9a66136de3c7d5787af1458e39f4"
private_key_bytes = bytes.fromhex(private_key_hex)

key = private_key_bytes[:16]
iv = private_key_bytes[16:]

try:
    cipher = AES.new(key, AES.MODE_CBC, iv=iv)
    decrypted = cipher.decrypt(encrypted_payload)
    
    print("Decryption Test with PRIVATE_KEY constant itself:")
    print(f"  Decrypted Hex: {decrypted.hex()}")
    print(f"  Decrypted ASCII: {repr(decrypted)}")
except Exception as e:
    print(f"Decryption failed: {e}")
