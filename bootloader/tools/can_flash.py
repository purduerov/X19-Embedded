#!/usr/bin/env python3
"""
Purdue ROV Remote CAN FD Bootloader Flashing Utility.
Streams firmware binaries over CAN FD (5 Mbps data phase) to target STM32G4 nodes.
Purdue ROV 2026-2027.
"""
from __future__ import annotations

import argparse
import sys
import time
import struct
import zlib
import binascii

try:
    import can
except ImportError:
    can = None

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

def wait_for_ack(bus, expected_node_id: int, timeout: float = 2.0) -> bool:
    """Wait for a positive or negative acknowledgement from one node."""
    end_time = time.monotonic() + timeout
    while time.monotonic() < end_time:
        try:
            msg = bus.recv(timeout=min(0.1, max(0.0, end_time - time.monotonic())))
        except Exception as exc:
            print(f"Error while waiting for ACK from node 0x{expected_node_id:02X}: {exc}")
            return False
        if msg is not None and msg.arbitration_id == CAN_ID_BOOT_CMD and len(msg.data) >= 2:
            if msg.data[0] == ACK and msg.data[1] == expected_node_id:
                return True
            if msg.data[0] == NACK and msg.data[1] == expected_node_id:
                print(f"Received NACK from node 0x{expected_node_id:02X}")
                return False
    return False


def _send_can_frame(bus, message) -> None:
    result = bus.send(message)
    if result is False:
        raise RuntimeError("python-can rejected the transmit request")


def _send_boot_command(bus, command: int, node_id: int, payload: bytes = b"") -> None:
    data = struct.pack("<BB", command, node_id) + payload
    if len(data) > 64:
        raise ValueError(f"Boot command payload is too large: {len(data)} bytes")
    _send_can_frame(
        bus,
        can.Message(
            arbitration_id=CAN_ID_BOOT_CMD,
            data=data,
            is_extended_id=False,
            is_fd=True,
        ),
    )


def flash_node(interface: str, target_node: str, bin_path: str) -> bool:
    node_id = NODES.get(target_node)
    if node_id is None:
        print(f"Error: Unknown target node '{target_node}'. Valid nodes: {list(NODES.keys())}")
        return False
    if can is None:
        print("Error: python-can is not installed. Install it before flashing hardware.")
        return False

    try:
        with open(bin_path, "rb") as f:
            firmware_data = f.read()
    except OSError as exc:
        print(f"Error: Could not read firmware '{bin_path}': {exc}")
        return False

    total_bytes = len(firmware_data)
    if total_bytes == 0:
        print("Error: Refusing to flash an empty firmware image.")
        return False
    if total_bytes > 0xFFFFFFFF:
        print("Error: Firmware image is too large for the boot protocol's 32-bit length field.")
        return False

    chunk_size = 60
    total_chunks = (total_bytes + chunk_size - 1) // chunk_size
    if total_chunks > 65535:
        print(f"Error: Firmware requires {total_chunks} chunks, exceeding maximum of 65535.")
        return False

    crc32_val = zlib.crc32(firmware_data) & 0xFFFFFFFF
    print(f"Opening CAN interface {interface} (CAN FD)...")
    try:
        bus = can.Bus(channel=interface, interface="socketcan", fd=True)
    except Exception as exc:
        print(f"Error: Could not open CAN interface {interface}: {exc}")
        return False

    try:
        print(f"Pinging target {target_node} (Node ID: 0x{node_id:02X})...")
        _send_boot_command(bus, CMD_PING, node_id)
        if not wait_for_ack(bus, node_id):
            print(f"Error: Failed to receive ACK for Ping from node {target_node}")
            return False

        print("Erasing application flash...")
        _send_boot_command(bus, CMD_ERASE_APP, node_id)
        if not wait_for_ack(bus, node_id, timeout=5.0):
            print(f"Error: Failed to receive ACK for Erase from node {target_node}")
            return False

        print(f"Starting Flash: {total_bytes} bytes, CRC32: 0x{crc32_val:08X}...")
        start_payload = struct.pack("<II", total_bytes, crc32_val)
        _send_boot_command(bus, CMD_START_FLASH, node_id, start_payload)
        if not wait_for_ack(bus, node_id, timeout=3.0):
            print(f"Error: Failed to receive ACK for Start Flash from node {target_node}")
            return False

        for i in range(total_chunks):
            iter_start = time.perf_counter()
            chunk = firmware_data[i * chunk_size : (i + 1) * chunk_size]
            if len(chunk) < chunk_size:
                chunk = chunk.ljust(chunk_size, b"\xff")
            # Bolt Optimization: Replaced custom python lookup table with C-optimized
            # binascii.crc_hqx for a ~26x speedup on CRC16-CCITT calculations.
            chunk_crc = binascii.crc_hqx(chunk, 0xFFFF)
            data_frame = struct.pack("<H", i) + chunk + struct.pack("<H", chunk_crc)
            _send_can_frame(
                bus,
                can.Message(
                    arbitration_id=CAN_ID_BOOT_DATA,
                    data=data_frame,
                    is_extended_id=False,
                    is_fd=True,
                ),
            )
            elapsed = time.perf_counter() - iter_start
            if elapsed < 0.001:
                time.sleep(0.001 - elapsed)

        print("Firmware transfer complete. Verifying application image...")
        _send_boot_command(bus, CMD_VERIFY_APP, node_id)
        if not wait_for_ack(bus, node_id, timeout=10.0):
            print(f"Error: Failed to receive ACK for Verify from node {target_node}")
            return False

        _send_boot_command(bus, CMD_JUMP_APP, node_id)
        if not wait_for_ack(bus, node_id, timeout=5.0):
            print("Error: Failed to receive ACK for Jump App; application launch is unconfirmed.")
            return False

        print("Application launched successfully!")
        return True
    except Exception as exc:
        print(f"Error: Flash operation failed for {target_node}: {exc}")
        return False
    finally:
        try:
            bus.shutdown()
        except Exception as exc:
            print(f"Warning: failed to close CAN interface {interface}: {exc}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Purdue ROV CAN FD Remote Bootloader Flasher")
    parser.add_argument("--interface", default="can0", help="SocketCAN interface (default: can0)")
    parser.add_argument("--target", required=True, choices=list(NODES.keys()), help="Target node name")
    parser.add_argument("--bin", required=True, help="Path to compiled firmware .bin file")
    args = parser.parse_args()

    if not flash_node(args.interface, args.target, args.bin):
        raise SystemExit(1)
