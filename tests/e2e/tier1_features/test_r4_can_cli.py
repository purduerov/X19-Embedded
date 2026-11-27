"""`rov can` end to end against an in-process SIL bridge, plus the transport contract."""

from __future__ import annotations

import socket
import struct
import subprocess
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from tests.e2e.e2e_context import MockSilTcpServer

REPO = Path(__file__).resolve().parents[3]
ESTOP = b"\xAA\x55\x01"
NEUTRAL = struct.pack("<8H", *([1500] * 8))


def rov_can(port: int, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, "rov.py", "can", "--sil", f"127.0.0.1:{port}", *args],
        cwd=REPO, capture_output=True, text=True, timeout=30,
    )


class TestPwmTimingValidation(unittest.TestCase):
    def test_invalid_timing_returns_usage_error_without_sending(self):
        from tools.rov_can.cli import cmd_pwm

        cases = [(hz, 0.0) for hz in (0.0, -1.0, float("nan"), float("inf"), -float("inf"))]
        cases += [(50.0, seconds) for seconds in (-1.0, float("nan"), float("inf"), -float("inf"))]
        for hz, seconds in cases:
            with self.subTest(hz=hz, seconds=seconds):
                transport = Mock()
                # Bound the original buggy loop too, including an infinite duration.
                with patch("tools.rov_can.cli.time.monotonic", side_effect=[0.0, float("inf")]):
                    with patch("sys.stderr") as stderr:
                        self.assertEqual(cmd_pwm(transport, [1500], hz, seconds), 2)
                transport.send_message.assert_not_called()
                self.assertTrue(stderr.write.called)

    def test_zero_duration_still_returns_to_neutral(self):
        from tools.rov_can.cli import cmd_pwm

        transport = Mock()
        self.assertEqual(cmd_pwm(transport, [1550], 50.0, 0.0), 0)
        transport.send_message.assert_called_once_with(0x100, NEUTRAL)


class TestRovCan(unittest.TestCase):
    def setUp(self):
        self.server = MockSilTcpServer()
        self.port = self.server.start()

    def tearDown(self):
        self.server.stop()

    def _wait_frames(self, n: int, timeout: float = 3.0):
        end = time.monotonic() + timeout
        while len(self.server.received_frames) < n and time.monotonic() < end:
            time.sleep(0.02)
        return self.server.received_frames

    def test_estop_sends_the_real_signature_frame(self):
        r = rov_can(self.port, "estop")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertEqual(self._wait_frames(1), [(0x001, ESTOP)])

    def test_pwm_out_of_range_is_refused_and_sends_nothing(self):
        r = rov_can(self.port, "pwm", "2500")
        self.assertEqual(r.returncode, 2, r.stdout + r.stderr)
        time.sleep(0.2)
        self.assertEqual(self.server.received_frames, [])

    def test_zero_pwm_rate_is_a_usage_error_and_sends_nothing(self):
        r = rov_can(self.port, "pwm", "1500", "--hz", "0")
        self.assertEqual(r.returncode, 2, r.stdout + r.stderr)
        self.assertIn("--hz", r.stderr)
        self.assertNotIn("Traceback", r.stderr)
        self.assertEqual(self.server.received_frames, [])

    def test_pwm_ends_on_neutral(self):
        r = rov_can(self.port, "pwm", "1550", "--hz", "50", "-t", "0.2")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        frames = self._wait_frames(2)
        self.assertGreaterEqual(len(frames), 2)
        self.assertTrue(all(i == 0x100 for i, _ in frames))
        self.assertEqual(frames[0][1], struct.pack("<8H", *([1550] * 8)))
        self.assertEqual(frames[-1][1], NEUTRAL, "must always return thrusters to neutral")

    def test_raw_send(self):
        r = rov_can(self.port, "send", "0x200", "01 02 03")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertEqual(self._wait_frames(1), [(0x200, b"\x01\x02\x03")])

    def test_transport_receives_what_the_bridge_sends(self):
        from tools.rov_can import SilTcpTransport

        with SilTcpTransport("127.0.0.1", self.port) as t:
            self.assertIsNone(t.receive_message(0.05), "timeout must return None, not block")
            deadline = time.monotonic() + 3
            while self.server._client_socket is None and time.monotonic() < deadline:
                time.sleep(0.02)
            self.server.send_frame_to_client(0x210, b"\x07" * 17)
            self.assertEqual(t.receive_message(2.0), (0x210, b"\x07" * 17))

    def test_transport_rejects_oversized_and_bad_ids(self):
        from tools.rov_can import SilTcpTransport

        with SilTcpTransport("127.0.0.1", self.port) as t:
            with self.assertRaises(ValueError):
                t.send_message(0x100, bytes(65))
            with self.assertRaises(ValueError):
                t.send_message(0x800, b"\x00")

    def test_no_bridge_gives_clean_error_not_traceback(self):
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        free = s.getsockname()[1]
        s.close()
        r = rov_can(free, "estop")
        self.assertEqual(r.returncode, 2)
        self.assertNotIn("Traceback", r.stderr)

    @unittest.skipIf(sys.platform == "linux", "non-Linux guard only")
    def test_socketcan_off_linux_says_why(self):
        r = subprocess.run([sys.executable, "rov.py", "can", "estop"], cwd=REPO, capture_output=True, text=True, timeout=30)
        self.assertEqual(r.returncode, 2)
        self.assertIn("Linux", r.stderr)
        self.assertNotIn("Traceback", r.stderr)


if __name__ == "__main__":
    unittest.main()
