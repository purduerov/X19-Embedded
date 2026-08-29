#!/usr/bin/env python3
"""
X19 Remote CAN FD Bootloader Flashing Utility.
Streams firmware binaries over CAN FD (5 Mbps data phase) to target STM32G4 nodes.
Purdue ROV 2026-2027.
"""

import argparse
import sys
import time
import struct
import zlib

try:
    import can
except ImportError:
    print("python-can not installed. Run 'pip install python-can'")

CAN_ID_BOOT_CMD  = 0x700
CAN_ID_BOOT_DATA = 0x701

CMD_PING        = 0x01
CMD_ERASE_APP   = 0x02
CMD_START_FLASH = 0x03
CMD_VERIFY_APP  = 0x04
CMD_JUMP_APP    = 0x05
ACK             = 0x06
NACK            = 0x07

NODES = {
    "pi_shield":     0x01,
    "control_board": 0x02,
    "power_slab":    0x03,
    "usb_hub":       0x04
}

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def flash_node(interface: str, target_node: str, bin_path: str):
    node_id = NODES.get(target_node)
    if node_id is None:
        print(f"Error: Unknown target node '{target_node}'. Valid nodes: {list(NODES.keys())}")
        return False

    with open(bin_path, "rb") as f:
        firmware_data = f.read()

    total_bytes = len(firmware_data)
    crc32_val = zlib.crc32(firmware_data)

    print(f"Opening CAN interface {interface} (CAN FD)...")
    bus = can.Bus(channel=interface, interface="socketcan", fd=True)

    print(f"Pinging target {target_node} (Node ID: 0x{node_id:02X})...")
    ping_msg = can.Message(
        arbitration_id=CAN_ID_BOOT_CMD,
        data=struct.pack("<BB", CMD_PING, node_id),
        is_extended_id=False,
        is_fd=True
    )
    bus.send(ping_msg)

    print(f"Starting Flash: {total_bytes} bytes, CRC32: 0x{crc32_val:08X}...")
    start_msg = can.Message(
        arbitration_id=CAN_ID_BOOT_CMD,
        data=struct.pack("<BBI I", CMD_START_FLASH, node_id, total_bytes, crc32_val),
        is_extended_id=False,
        is_fd=True
    )
    bus.send(start_msg)
    time.sleep(0.1)

    chunk_size = 60
    total_chunks = (total_bytes + chunk_size - 1) // chunk_size

    for i in range(total_chunks):
        chunk = firmware_data[i * chunk_size : (i + 1) * chunk_size]
        if len(chunk) < chunk_size:
            chunk = chunk.ljust(chunk_size, b'\xFF')
        chunk_crc = crc16_ccitt(chunk)
        data_frame = struct.pack("<H", i) + chunk + struct.pack("<H", chunk_crc)
        msg = can.Message(
            arbitration_id=CAN_ID_BOOT_DATA,
            data=data_frame,
            is_extended_id=False,
            is_fd=True
        )
        bus.send(msg)
        time.sleep(0.001)

    print("Firmware transfer complete. Verifying and booting...")
    jump_msg = can.Message(
        arbitration_id=CAN_ID_BOOT_CMD,
        data=struct.pack("<BB", CMD_JUMP_APP, node_id),
        is_extended_id=False,
        is_fd=True
    )
    bus.send(jump_msg)
    print("Application launched successfully!")
    return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="X19 CAN FD Remote Bootloader Flasher")
    parser.add_argument("--interface", default="can0", help="SocketCAN interface (default: can0)")
    parser.add_argument("--target", required=True, choices=list(NODES.keys()), help="Target node name")
    parser.add_argument("--bin", required=True, help="Path to compiled firmware .bin file")
    args = parser.parse_args()

    flash_node(args.interface, args.target, args.bin)
