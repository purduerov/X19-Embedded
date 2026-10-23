"""Tier 3 Cross-Feature Combinations: CAN Communication over SIL TCP Bridge.

Tests bidirectional CAN FD messaging across client and SIL TCP bridge server,
including telemetry streaming, thruster commands, rapid bursts, and client reconnection.
Authoritative source: ORIGINAL_REQUEST.md § R4, PROJECT.md § Tier 3.
"""

from __future__ import annotations

import socket
import time
import unittest

from tests.e2e.e2e_context import (
    CAN_ID_ENV_TELEMETRY,
    CAN_ID_NAV_TELEMETRY,
    CAN_ID_POWER_TELEMETRY,
    CAN_ID_THRUSTER_CMD,
    MockSilTcpServer,
    pack_sil_can_frame,
    unpack_sil_can_frame,
)


class TestCanSilBridge(unittest.TestCase):
    """Verify bidirectional CAN communication and framing over SIL TCP bridge."""

    def test_can_client_to_server_thruster_command_flow(self):
        """Verify: Thruster PWM command frame (0x100) is delivered over bridge."""
        server = MockSilTcpServer()
        port = server.start()
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect(("127.0.0.1", port))
            self.assertTrue(server.wait_for_client(2.0))

            # 8 x 16-bit PWM channels = 16 bytes
            pwm_payload = b"\xdc\x05" * 8  # 1500 us neutral
            packet = pack_sil_can_frame(CAN_ID_THRUSTER_CMD, pwm_payload)
            client.sendall(packet)

            time.sleep(0.1)
            self.assertEqual(len(server.received_frames), 1)
            arb_id, data = server.received_frames[0]
            self.assertEqual(arb_id, CAN_ID_THRUSTER_CMD)
            self.assertEqual(data, pwm_payload)
            client.close()
        finally:
            server.stop()

    def test_can_server_to_client_telemetry_flow(self):
        """Verify: Telemetry frames (Nav 0x200, Env 0x210, Power 0x300) are streamed to client."""
        server = MockSilTcpServer()
        port = server.start()
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect(("127.0.0.1", port))
            self.assertTrue(server.wait_for_client(2.0))

            telemetry_stream = [
                (CAN_ID_NAV_TELEMETRY, b"\x10" * 32),
                (CAN_ID_ENV_TELEMETRY, b"\x20" * 16),
                (CAN_ID_POWER_TELEMETRY, b"\x30" * 24),
            ]

            for arb_id, payload in telemetry_stream:
                server.send_frame_to_client(arb_id, payload)

            client.settimeout(1.0)
            received = []
            buf = bytearray()
            while len(received) < 3:
                chunk = client.recv(1024)
                if not chunk:
                    break
                buf.extend(chunk)
                while len(buf) >= 73:
                    _, r_id, _, r_data = unpack_sil_can_frame(bytes(buf[:73]))
                    received.append((r_id, r_data))
                    del buf[:73]

            self.assertEqual(len(received), 3)
            self.assertEqual(received[0], telemetry_stream[0])
            self.assertEqual(received[1], telemetry_stream[1])
            self.assertEqual(received[2], telemetry_stream[2])
            client.close()
        finally:
            server.stop()

    def test_can_fd_64byte_payload_roundtrip(self):
        """Verify: 64-byte maximum CAN FD payload preserves full integrity through bridge."""
        server = MockSilTcpServer()
        port = server.start()
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect(("127.0.0.1", port))
            self.assertTrue(server.wait_for_client(2.0))

            test_payload = bytes(range(64))
            packet = pack_sil_can_frame(0x555, test_payload)
            client.sendall(packet)

            time.sleep(0.1)
            self.assertEqual(len(server.received_frames), 1)
            self.assertEqual(server.received_frames[0][1], test_payload)
            client.close()
        finally:
            server.stop()

    def test_can_rapid_burst_under_load(self):
        """Verify: 25 consecutive CAN frames sent in rapid succession are received in order."""
        server = MockSilTcpServer()
        port = server.start()
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect(("127.0.0.1", port))
            self.assertTrue(server.wait_for_client(2.0))

            burst_count = 25
            for i in range(burst_count):
                packet = pack_sil_can_frame(0x100 + i, bytes([i] * 8))
                client.sendall(packet)

            time.sleep(0.2)
            self.assertEqual(len(server.received_frames), burst_count)
            for i in range(burst_count):
                self.assertEqual(server.received_frames[i][0], 0x100 + i)
                self.assertEqual(server.received_frames[i][1], bytes([i] * 8))
            client.close()
        finally:
            server.stop()

    def test_can_reconnect_after_transient_socket_close(self):
        """Verify: Bridge server accepts reconnection after client disconnects."""
        server = MockSilTcpServer()
        port = server.start()
        try:
            # Client 1 connects and sends frame
            client1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client1.connect(("127.0.0.1", port))
            self.assertTrue(server.wait_for_client(2.0))
            client1.sendall(pack_sil_can_frame(0x101, b"hello"))
            time.sleep(0.05)
            client1.close()

            # Client 2 connects to the same server and sends frame
            client2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client2.connect(("127.0.0.1", port))
            client2.sendall(pack_sil_can_frame(0x102, b"world"))
            time.sleep(0.05)
            client2.close()

            self.assertEqual(len(server.received_frames), 2)
            self.assertEqual(server.received_frames[0][0], 0x101)
            self.assertEqual(server.received_frames[1][0], 0x102)
        finally:
            server.stop()


if __name__ == "__main__":
    unittest.main()
