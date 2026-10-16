import binascii

def crc_xmodem(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc

data = b"hello world 12345"
print("crc_xmodem:", hex(crc_xmodem(data)))
print("crc_hqx:", hex(binascii.crc_hqx(data, 0)))
