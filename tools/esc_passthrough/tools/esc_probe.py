#!/usr/bin/env python3
"""Check the Nucleo ESC passthrough: enter 4-way mode, connect to the ESC
bootloader and print the BLHeli_S settings header. Read-only.

usage: python3 esc_probe.py /dev/cu.usbmodemXXXX     (pip install pyserial)
"""
import struct
import sys

import serial

BLHELI_S_EEPROM = 0x1A00


def msp(port, cmd, data=b""):
    ck = len(data) ^ cmd
    for b in data:
        ck ^= b
    port.write(b"$M<" + bytes([len(data), cmd]) + data + bytes([ck]))
    hdr = port.read(5)
    if len(hdr) < 5 or hdr[:2] != b"$M":
        raise RuntimeError(f"no MSP reply to {cmd}: {hdr!r}")
    payload = port.read(hdr[3] + 1)[:-1]
    if hdr[2:3] != b">":
        raise RuntimeError(f"MSP {cmd} rejected")
    return payload


def crc_xmodem(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def fourway(port, cmd, addr=0, params=b"\x00"):
    frame = bytes([0x2F, cmd, addr >> 8, addr & 0xFF, len(params) & 0xFF]) + params
    port.write(frame + struct.pack(">H", crc_xmodem(frame)))
    hdr = port.read(5)
    if len(hdr) < 5 or hdr[0] != 0x2E:
        raise RuntimeError(f"no 4-way reply to 0x{cmd:02X}: {hdr!r}")
    n = hdr[4] or 256
    rest = port.read(n + 3)
    body, ack, crc = rest[:n], rest[n], struct.unpack(">H", rest[n + 1:])[0]
    if crc != crc_xmodem(hdr + body + bytes([ack])):
        raise RuntimeError("4-way CRC mismatch")
    return ack, body


def main():
    port = serial.Serial(sys.argv[1], 115200, timeout=3)
    # Same handshake esc-configurator.com runs before reading ESCs
    for cmd in (1, 3, 5, 4, 160, 104, 36, 101):
        msp(port, cmd)
    print("FC variant:", msp(port, 2).decode())
    print("ESC count :", msp(port, 245)[0])

    ack, info = fourway(port, 0x37)  # DeviceInitFlash, ESC 0
    if ack:
        sys.exit(f"ESC bootloader not responding (ack 0x{ack:02X}). "
                 "Power-cycle the ESC and retry.")
    print(f"Signature : 0x{info[1]:02X}{info[0]:02X}  mode {info[3]}")

    ack, ee = fourway(port, 0x3A, BLHELI_S_EEPROM, bytes([0x70]))  # DeviceRead
    if ack:
        sys.exit(f"read failed (ack 0x{ack:02X})")
    text = lambda b: b.split(b"\x00")[0].decode(errors="replace").strip()
    print(f"Firmware  : {ee[0]}.{ee[1]} (layout rev {ee[2]})")
    print("Layout    :", text(ee[0x40:0x50]))
    print("MCU       :", text(ee[0x50:0x60]))
    print("Name      :", text(ee[0x60:0x70]))

    fourway(port, 0x35)  # DeviceReset
    fourway(port, 0x34)  # InterfaceExit


if __name__ == "__main__":
    main()
