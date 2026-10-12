from bootloader.tools.can_flash import crc16_ccitt
import binascii
data = b"123456789"
print(hex(crc16_ccitt(data)))
print(hex(binascii.crc_hqx(data, 0xFFFF)))
