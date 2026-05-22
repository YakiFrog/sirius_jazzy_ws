import hashlib
from Crypto.Cipher import AES

payload = bytes.fromhex("ec4950c89bc1d23561764a77ac282891")
secret = hashlib.sha256(b"Solix").digest()
key = secret[:16]
iv = secret[16:]

print(f"Secret: {secret.hex()}")
print(f"Key:    {key.hex()}")
print(f"IV:     {iv.hex()}")

cipher = AES.new(key, AES.MODE_CBC, iv=iv)
decrypted = cipher.decrypt(payload)
print(f"Decrypted Raw Hex: {decrypted.hex()}")
