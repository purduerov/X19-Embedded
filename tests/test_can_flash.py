import sys
import os

# Add bootloader/tools directory to sys.path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "bootloader", "tools"))

from can_flash import crc16_ccitt


def test_crc16_ccitt_standard_vector():
    # Standard CCITT test vector b"123456789" with init 0xFFFF and polynomial 0x1021
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_crc16_ccitt_empty():
    # Empty input should return initial value 0xFFFF
    assert crc16_ccitt(b"") == 0xFFFF


def test_crc16_ccitt_single_byte():
    assert crc16_ccitt(b"A") == 0xB915


def test_crc16_ccitt_zeros():
    assert crc16_ccitt(b"\x00" * 10) == 0xE139


def test_crc16_ccitt_ones():
    assert crc16_ccitt(b"\xFF" * 10) == 0xA6E1
