import hashlib
from cryptography.hazmat.primitives.asymmetric.ec import SECP256R1, EllipticCurvePrivateKey, EllipticCurvePublicKey, ECDH
from cryptography.hazmat.primitives.asymmetric import ec
from Crypto.Cipher import AES

# Raw encrypted payload from 4840 notification
encrypted_payload = bytes.fromhex("46d543e87674195355ebeb1bfb5a0cdb")
serial_str = "AZVSB70F03400100"
serial = serial_str.encode()

# Fixed PC private key constant
pc_private_key_hex = "7dfbea61cd95cee49c458ad7419e817f1ade9a66136de3c7d5787af1458e39f4"
pc_private_value = int.from_bytes(bytes.fromhex(pc_private_key_hex), byteorder="big")
pc_private_key = ec.derive_private_key(pc_private_value, SECP256R1())

# Generate potential device private values from serial hashes
seeds = {
    "SHA256 of Serial": hashlib.sha256(serial).digest(),
    "MD5 of Serial (padded to 32B with zeros)": hashlib.md5(serial).digest() + b"\x00"*16,
    "SHA256 of Serial lower": hashlib.sha256(serial_str.lower().encode()).digest(),
    "SHA1 of Serial (padded)": hashlib.sha1(serial).digest() + b"\x00"*12,
}

for name, seed in seeds.items():
    try:
        # Interpret seed as a private scalar value
        dev_private_value = int.from_bytes(seed, byteorder="big")
        
        # Check if scalar is within the valid range for secp256r1
        # Order n of secp256r1:
        n = 0xffffffff00000000ffffffffffffffffbce6faada7179e84f3b9cac2fc632551
        dev_private_value = dev_private_value % n
        if dev_private_value == 0:
            continue
            
        dev_private_key = ec.derive_private_key(dev_private_value, SECP256R1())
        dev_public_key = dev_private_key.public_key()
        
        # ECDH exchange: PC private key * Device public key
        shared_secret = pc_private_key.exchange(ec.ECDH(), dev_public_key)
        
        key = shared_secret[:16]
        iv = shared_secret[16:]
        
        cipher = AES.new(key, AES.MODE_CBC, iv=iv)
        decrypted = cipher.decrypt(encrypted_payload)
        
        first_byte = decrypted[0]
        is_plausible = (first_byte == 0x00 or (first_byte & 0xF0 == 0xA0))
        
        print(f"Seed: {name}")
        print(f"  Shared Secret: {shared_secret.hex()}")
        print(f"  Decrypted Hex: {decrypted.hex()}")
        print(f"  Decrypted ASCII: {repr(decrypted)}")
        if is_plausible:
            print("  >>> Plausible ECDH decrypted structure detected! <<<")
        print("-" * 50)
    except Exception as e:
        print(f"Failed with seed {name}: {e}")
