from Crypto.Cipher import AES

# Raw encrypted payload from 4840 notification
encrypted_payload = bytes.fromhex("46d543e87674195355ebeb1bfb5a0cdb")

# The computed shared secret using PC's own public key coordinate as fallback
shared_secret = bytes.fromhex("dfdbcd82f5a37a939cff04a2694ee02e71ac047415c7743bf5d08caa5327bb35")

key = shared_secret[:16]
iv = shared_secret[16:]

try:
    cipher = AES.new(key, AES.MODE_CBC, iv=iv)
    decrypted = cipher.decrypt(encrypted_payload)
    
    print("Decryption Test with fallback ECDH shared secret:")
    print(f"  Decrypted Hex: {decrypted.hex()}")
    print(f"  Decrypted ASCII: {repr(decrypted)}")
except Exception as e:
    print(f"Decryption failed: {e}")
