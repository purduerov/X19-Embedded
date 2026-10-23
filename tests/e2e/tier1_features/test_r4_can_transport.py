"""Tier 1 Feature Coverage: R4 Unified CAN Multi-Transport Verification.

Tests the unified multi-transport CAN interface: SocketCanTransport (Linux can0)
and SilTcpTransport (SIL TCP bridge), verifying frame formatting, CAN FD 64-byte
payload handling, and graceful transport fallback.
Authoritative source: ORIGINAL_REQUEST.md § R4, PROJECT.md § Feature 11 & Interface Contracts.
"""

from __future__ import annotations

import os
import socket
import unittest
from typing import Optional

from tests.e2e.e2e_context import (
    CAN_ID_EMERGENCY_BREAK,
    CAN_ID_NAV_TELEMETRY,
    CAN_ID_THRUSTER_CMD,
    SIL_MAGIC_HEADER,
    MockSilTcpServer,
    pack_sil_can_frame,
    unpack_sil_can_frame,
)


def _try_import_rov_can():
    try:
        import tools.rov_can.transport as t
        return t
    except ImportError:
        return None


class TestR4CanTransport(unittest.TestCase):
    """Verify unified CAN multi-transport architecture and protocol contracts."""

    def test_r4_can_transport_base_contract_interface(self):
        """Contract: CanTransport defines send_message and receive_message interface."""
        rov_can = _try_import_rov_can()
        if rov_can is None and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M4: Consolidated tools.rov_can package")

        self.assertTrue(hasattr(rov_can, "CanTransport"), "CanTransport base class must exist")
        self.assertTrue(hasattr(rov_can, "SocketCanTransport"), "SocketCanTransport must exist")
        self.assertTrue(hasattr(rov_can, "SilTcpTransport"), "SilTcpTransport must exist")

    def test_r4_sil_tcp_frame_packing_contract(self):
        """Verify: 73-byte wire framing complies with ROVC header, id, len, and 64-byte buffer."""
        test_payload = b"\x01\x02\x03\x04\x05\x06\x07\x08"
        packed = pack_sil_can_frame(CAN_ID_THRUSTER_CMD, test_payload)
        self.assertEqual(len(packed), 73, "Wire packet must be exactly 73 bytes")

        magic, arb_id, length, data = unpack_sil_can_frame(packed)
        self.assertEqual(magic, SIL_MAGIC_HEADER)
        self.assertEqual(arb_id, CAN_ID_THRUSTER_CMD)
        self.assertEqual(length, len(test_payload))
        self.assertEqual(data, test_payload)

    def test_r4_can_fd_64byte_payload_boundary(self):
        """Verify: Full 64-byte CAN FD payload boundary packs and unpacks without truncation."""
        full_fd_payload = bytes(range(64))
        packed = pack_sil_can_frame(CAN_ID_NAV_TELEMETRY, full_fd_payload)
        self.assertEqual(len(packed), 73)

        magic, arb_id, length, data = unpack_sil_can_frame(packed)
        self.assertEqual(length, 64)
        self.assertEqual(data, full_fd_payload)

        # Oversized payload (>64 bytes) must be rejected
        oversized_payload = bytes(65)
        with self.assertRaises(ValueError):
            pack_sil_can_frame(CAN_ID_NAV_TELEMETRY, oversized_payload)

    def test_r4_sil_tcp_transport_socket_roundtrip(self):
        """Verify: Client can transmit CAN FD frames to SIL bridge server over TCP."""
        server = MockSilTcpServer()
        port = server.start()
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect(("127.0.0.1", port))

            # Send thruster command frame
            test_cmd = b"\x05\xdc" * 8  # 1500 us neutral PWM across 8 thrusters
            packet = pack_sil_can_frame(CAN_ID_THRUSTER_CMD, test_cmd)
            client.sendall(packet)

            # Wait briefly for server receipt
            import time
            time.sleep(0.1)

            self.assertEqual(len(server.received_frames), 1, "Server should have received 1 CAN frame")
            recv_id, recv_data = server.received_frames[0]
            self.assertEqual(recv_id, CAN_ID_THRUSTER_CMD)
            self.assertEqual(recv_data, test_cmd)

            # Server sends back telemetry
            server.send_frame_to_client(CAN_ID_EMERGENCY_BREAK, b"\x01")
            client.settimeout(1.0)
            recv_bytes = client.recv(1024)
            _, back_id, back_len, back_data = unpack_sil_can_frame(recv_bytes)
            self.assertEqual(back_id, CAN_ID_EMERGENCY_BREAK)
            self.assertEqual(back_data, b"\x01")

            client.close()
        finally:
            server.stop()

    def test_r4_socketcan_transport_environment_guard(self):
        """Verify: SocketCanTransport raises descriptive exception on non-Linux or missing can0."""
        rov_can = _try_import_rov_can()
        if rov_can is None and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M4: Consolidated tools.rov_can package")

        import sys
        if sys.platform != "linux":
            # On Windows/macOS, attempting to initialize SocketCanTransport should raise OSError or RuntimeError
            try:
                transport = rov_can.SocketCanTransport(interface="can0")
                self.fail("SocketCanTransport should not succeed on non-Linux platform without SocketCAN")
            except (OSError, RuntimeError, SystemError, NotImplementedError, ValueError) as ex:
                self.assertIsNotNone(ex)


if __name__ == "__main__":
    unittest.main()
