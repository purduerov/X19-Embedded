import binascii
import time

def crc_xmodem(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc

data = b"a" * 1024

start = time.perf_counter()
for _ in range(1000):
    crc_xmodem(data)
t1 = time.perf_counter() - start

start = time.perf_counter()
for _ in range(1000):
    binascii.crc_hqx(data, 0)
t2 = time.perf_counter() - start

print(f"python crc: {t1:.4f} s")
print(f"c hqx crc: {t2:.4f} s")
print(f"speedup: {t1/t2:.1f}x")
