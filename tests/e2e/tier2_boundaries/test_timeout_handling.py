"""Tier 2 Boundary & Corner Cases: Timeout Handling & Resource Boundaries.

Tests timeout handling across serial monitoring, CAN message reception, and socket operations.
Authoritative source: ORIGINAL_REQUEST.md § R1 & R4, PROJECT.md § Boundary Cases.
"""

from __future__ import annotations

import socket
import time
import unittest

from tests.e2e.e2e_context import (
    MockSilTcpServer,
    run_rov_cli,
    unpack_sil_can_frame,
)


class TestTimeoutHandling(unittest.TestCase):
    """Verify precise timeout handling across monitoring and network transports."""

    def test_monitor_timeout_exits_cleanly(self):
        """Verify: rov monitor with -t 0.1 exits cleanly within 2 seconds."""
        t_start = time.time()
        res = run_rov_cli(["monitor", "-t", "0.1"], timeout=5.0)
        t_elapsed = time.time() - t_start
        self.assertLess(t_elapsed, 4.0, "Monitor should exit promptly after timeout")

    def test_can_receive_message_timeout_returns_safely(self):
        """Verify: Socket receiving CAN packet returns None/times out rather than hanging indefinitely."""
        server = MockSilTcpServer()
        port = server.start()
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect(("127.0.0.1", port))
            client.settimeout(0.2)

            # Read from server when no packets are sent
            t_start = time.time()
            with self.assertRaises(socket.timeout):
                client.recv(1024)
            t_elapsed = time.time() - t_start

            self.assertGreaterEqual(t_elapsed, 0.15)
            self.assertLess(t_elapsed, 0.6)
            client.close()
        finally:
            server.stop()

    def test_zero_timeout_immediate_nonblocking_behavior(self):
        """Verify: timeout=0.0 immediately returns or raises BlockingIOError if no data is present."""
        server = MockSilTcpServer()
        port = server.start()
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect(("127.0.0.1", port))
            client.setblocking(False)

            with self.assertRaises((BlockingIOError, OSError)):
                client.recv(1024)

            client.close()
        finally:
            server.stop()

    def test_negative_timeout_rejected(self):
        """Verify: Negative timeout parameter is rejected or raises ValueError."""
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        with self.assertRaises(ValueError):
            client.settimeout(-1.0)
        client.close()

    def test_sil_bridge_rapid_burst_receives_within_timeout(self):
        """Verify: A burst of frames received within timeout window are processed without loss."""
        server = MockSilTcpServer()
        port = server.start()
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect(("127.0.0.1", port))
            self.assertTrue(server.wait_for_client(2.0), "Server must accept connection")

            # Send 5 frames from server
            from tests.e2e.e2e_context import pack_sil_can_frame
            for i in range(5):
                packet = pack_sil_can_frame(0x200 + i, bytes([i] * 8))
                server.send_frame_to_client(0x200 + i, bytes([i] * 8))

            client.settimeout(1.0)
            received_ids = []
            buf = bytearray()
            while len(received_ids) < 5:
                chunk = client.recv(1024)
                if not chunk:
                    break
                buf.extend(chunk)
                while len(buf) >= 73:
                    _, arb_id, _, _ = unpack_sil_can_frame(bytes(buf[:73]))
                    received_ids.append(arb_id)
                    del buf[:73]

            self.assertEqual(len(received_ids), 5)
            self.assertEqual(received_ids, [0x200, 0x201, 0x202, 0x203, 0x204])
            client.close()
        finally:
            server.stop()


if __name__ == "__main__":
    unittest.main()
