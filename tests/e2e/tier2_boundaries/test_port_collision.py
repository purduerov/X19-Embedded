"""Tier 2 Boundary & Corner Cases: Socket Port Collision & Network Errors.

Tests network error recovery, TCP port collisions, refused connections, and socket timeouts
for the SIL TCP CAN bridge transport.
Authoritative source: ORIGINAL_REQUEST.md § R4, PROJECT.md § Boundary Cases.
"""

from __future__ import annotations

import socket
import unittest

from tests.e2e.e2e_context import (
    MockSilTcpServer,
    pack_sil_can_frame,
)


class TestPortCollision(unittest.TestCase):
    """Verify robust network error handling for SIL TCP transports."""

    def test_tcp_bridge_port_collision_detected(self):
        """Verify: Attempting to bind a second server on the same TCP port raises OSError."""
        server1 = MockSilTcpServer()
        port = server1.start()
        try:
            sock2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Without SO_REUSEADDR, binding to the exact same port must fail
            with self.assertRaises(OSError):
                sock2.bind(("127.0.0.1", port))
            sock2.close()
        finally:
            server1.stop()

    def test_client_connect_to_unopened_port_refused(self):
        """Verify: Connecting to a port with no listening bridge server raises ConnectionRefusedError."""
        # Find an unused free port then immediately close it
        temp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        temp_sock.bind(("127.0.0.1", 0))
        free_port = temp_sock.getsockname()[1]
        temp_sock.close()

        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(0.5)
        with self.assertRaises((ConnectionRefusedError, OSError, TimeoutError)):
            client.connect(("127.0.0.1", free_port))
        client.close()

    def test_send_to_closed_socket_raises_error(self):
        """Verify: Transmitting CAN frames on a closed/terminated connection raises OSError."""
        server = MockSilTcpServer()
        port = server.start()
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect(("127.0.0.1", port))
            client.close()

            frame = pack_sil_can_frame(0x100, b"\x00" * 8)
            with self.assertRaises(OSError):
                client.sendall(frame)
        finally:
            server.stop()

    def test_socket_invalid_port_number_boundary(self):
        """Verify: Out-of-bounds port numbers (< 0 or > 65535) raise ValueError/OverflowError."""
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        with self.assertRaises((OverflowError, ValueError, OSError)):
            client.connect(("127.0.0.1", 70000))
        with self.assertRaises((OverflowError, ValueError, OSError)):
            client.connect(("127.0.0.1", -1))
        client.close()

    def test_socket_connect_timeout_behavior(self):
        """Verify: Short connection timeout to non-responsive endpoint triggers TimeoutError."""
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(0.1)
        # 192.0.2.1 is TEST-NET-1 (non-routable documentation network)
        try:
            with self.assertRaises((TimeoutError, OSError)):
                client.connect(("192.0.2.1", 8765))
        finally:
            client.close()


if __name__ == "__main__":
    unittest.main()
