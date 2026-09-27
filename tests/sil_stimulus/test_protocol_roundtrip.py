"""
Framing round-trip tests for the SIL stimulus harness.

These tests pin the Python view of the SIL wire format that the native
``sil_bridge_server`` speaks.  Framing is defined once in
``tests/sil_companion_bridge/sil_protocol.py``; nothing here redefines it.
"""

import os
import socket
import struct
import time
import unittest
from pathlib import Path

try:
    from .sil_test_support import (
        CAN_ID_SOLENOID_CMD,
        CAN_ID_SIL_OUTPUT_STATUS,
        CAN_ID_THRUSTER_CMD,
        SERVER_EXE,
        SilServerProcess,
        free_tcp_port,
        recv_frames,
        require_server_executable,
        send_raw_frame,
        server_executable,
    )
    from . import sil_test_support
except ImportError:  # pragma: no cover - direct execution fallback
    from sil_test_support import (  # type: ignore[no-redef]
        CAN_ID_SOLENOID_CMD,
        CAN_ID_SIL_OUTPUT_STATUS,
        CAN_ID_THRUSTER_CMD,
        SERVER_EXE,
        SilServerProcess,
        free_tcp_port,
        recv_frames,
        require_server_executable,
        send_raw_frame,
        server_executable,
    )
    import sil_test_support  # type: ignore[no-redef]

# Assert the default location the brief documents stays in sync with discovery.
REPO_ROOT = Path(__file__).resolve().parents[2]
NEUTRAL_PWMS = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]


def _port_refuses_connections(port: int, timeout: float = 1.0) -> bool:
    """Return True when nothing is listening on ``port`` (connect is refused)."""
    try:
        probe = socket.create_connection(("127.0.0.1", port), timeout=timeout)
    except OSError:
        return True
    probe.close()
    return False


class TestSilProtocolRoundtrip(unittest.TestCase):
    """The pure framing layer; no native binary required."""

    def test_command_frame_roundtrip(self):
        payload = sil_test_support.ThrusterCommand(
            pwm_us=[1500, 1600, 1400, 1500, 1500, 1500, 1500, 1500]
        ).pack()
        can_id, decoded = sil_test_support.unpack_sil_can_frame(
            sil_test_support.pack_sil_can_frame(CAN_ID_THRUSTER_CMD, payload)
        )
        self.assertEqual(can_id, CAN_ID_THRUSTER_CMD)
        self.assertEqual(
            sil_test_support.ThrusterCommand.unpack(decoded).pwm_us,
            [1500, 1600, 1400, 1500, 1500, 1500, 1500, 1500],
        )

    def test_output_status_roundtrip(self):
        payload = struct.pack("<8H", 1500, 1600, 1400, 1500, 1500, 1500, 1500, 1500) + struct.pack(
            "<BHI", 1, 0x0123, 4321
        )
        pwms, brake, solenoids, sim_ms = sil_test_support.unpack_sil_output_status(payload)
        self.assertEqual(pwms, [1500, 1600, 1400, 1500, 1500, 1500, 1500, 1500])
        self.assertTrue(brake)
        self.assertEqual(solenoids, 0x0123)
        self.assertEqual(sim_ms, 4321)

    def test_solenoid_command_frame_roundtrip(self):
        payload = sil_test_support.SolenoidCommand(solenoid_mask=0x0155).pack()
        can_id, decoded = sil_test_support.unpack_sil_can_frame(
            sil_test_support.pack_sil_can_frame(CAN_ID_SOLENOID_CMD, payload)
        )
        self.assertEqual(can_id, CAN_ID_SOLENOID_CMD)
        self.assertEqual(sil_test_support.SolenoidCommand.unpack(decoded).solenoid_mask, 0x0155)

    def test_output_status_survives_sil_frame_wrapping(self):
        """The native server sends 23 status bytes inside a 73-byte SIL packet."""
        payload = struct.pack("<8H", *NEUTRAL_PWMS) + struct.pack("<BHI", 0, 0x0000, 100)
        packet = sil_test_support.pack_sil_can_frame(CAN_ID_SIL_OUTPUT_STATUS, payload)
        self.assertEqual(len(packet), sil_test_support.SIL_PACKET_SIZE)

        can_id, decoded = sil_test_support.unpack_sil_can_frame(packet)
        self.assertEqual(can_id, CAN_ID_SIL_OUTPUT_STATUS)
        pwms, brake, solenoids, sim_ms = sil_test_support.unpack_sil_output_status(decoded)
        self.assertEqual(pwms, NEUTRAL_PWMS)
        self.assertFalse(brake)
        self.assertEqual(solenoids, 0)
        self.assertEqual(sim_ms, 100)

    def test_unpack_rejects_invalid_magic(self):
        corrupted = bytearray(
            sil_test_support.pack_sil_can_frame(CAN_ID_THRUSTER_CMD, b"\x00" * 16)
        )
        corrupted[0:4] = b"\xDE\xAD\xBE\xEF"
        with self.assertRaises(ValueError):
            sil_test_support.unpack_sil_can_frame(bytes(corrupted))


class TestFrameIoHelpers(unittest.TestCase):
    """send_raw_frame / recv_frames must survive split TCP reads."""

    def test_free_tcp_port_is_bindable(self):
        port = free_tcp_port()
        self.assertGreater(port, 0)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.bind(("127.0.0.1", port))

    def test_recv_frames_reassembles_split_packets(self):
        """A frame straddling two recv_frames calls must survive the call boundary.

        The first write is deliberately cut mid-frame (100 bytes = one whole
        73-byte packet plus 27 bytes of the next).  A function-local buffer
        would drop those 27 bytes, so the second call could never reconstruct
        the trailing packet.
        """
        expected = [
            (CAN_ID_THRUSTER_CMD, b"\x01\x02\x03\x04"),
            (CAN_ID_SIL_OUTPUT_STATUS, b"\x05\x06\x07\x08"),
            (CAN_ID_SOLENOID_CMD, b"\x09\x0A"),
        ]
        stream = b"".join(
            sil_test_support.pack_sil_can_frame(can_id, payload) for can_id, payload in expected
        )
        packet_size = sil_test_support.SIL_PACKET_SIZE
        self.assertEqual(len(stream), 3 * packet_size)

        # Cut point sits 27 bytes into the second frame.
        cut = packet_size + 27
        self.assertEqual(len(stream) - cut, 119)

        left, right = socket.socketpair()
        self.addCleanup(left.close)
        self.addCleanup(right.close)

        left.sendall(stream[:cut])
        first_batch = recv_frames(right, 0.5)
        self.assertEqual(
            first_batch,
            expected[:1],
            "The first call must decode exactly the one complete frame and keep the "
            "27 trailing bytes buffered",
        )

        left.sendall(stream[cut:])
        second_batch = recv_frames(right, 0.5)
        self.assertEqual(
            second_batch,
            expected[1:],
            "The second call must reuse the 27 bytes held over from the first call to "
            "reconstruct the remaining frames; a per-call buffer loses them",
        )

    def test_recv_frames_returns_empty_list_on_timeout(self):
        left, right = socket.socketpair()
        self.addCleanup(left.close)
        self.addCleanup(right.close)
        self.assertEqual(recv_frames(right, 0.2), [])

    def test_recv_frames_rejects_invalid_magic(self):
        left, right = socket.socketpair()
        self.addCleanup(left.close)
        self.addCleanup(right.close)
        left.sendall(b"\xDE\xAD\xBE\xEF" + b"\x00" * (sil_test_support.SIL_PACKET_SIZE - 4))
        with self.assertRaises(ValueError):
            recv_frames(right, 2.0)


class TestSilServerProcess(unittest.TestCase):
    """Exercises the real native server; skipped when the binary is unbuilt."""

    @staticmethod
    def _pump_until(sock, predicate, timeout_sec=8.0):
        deadline = time.monotonic() + timeout_sec
        collected = []
        while time.monotonic() < deadline:
            collected.extend(recv_frames(sock, 0.2))
            if predicate(collected):
                return True, collected
        return False, collected

    def test_server_binary_discovery_matches_documented_default(self):
        resolved = server_executable()
        if resolved is None:
            self.assertFalse(SERVER_EXE.is_file(), "Discovery said no binary but the default path exists")
            self.skipTest(
                "Native sil_bridge_server is not built. Run: cmake -B build-native -G Ninja; "
                "cmake --build build-native --target sil_bridge_server"
            )
        self.assertTrue(resolved.is_file())
        self.assertEqual(Path(os.path.realpath(resolved)).stem, sil_test_support.SERVER_STEM)
        self.assertEqual(
            resolved,
            SERVER_EXE,
            "server_executable() must agree with the documented default SERVER_EXE; "
            f"discovery returned {resolved} but the default is {SERVER_EXE}",
        )

    def test_server_starts_streams_and_stops(self):
        executable = require_server_executable()

        with SilServerProcess(executable) as server:
            self.assertIsNotNone(server.process)
            self.assertIsNone(server.process.poll(), "SIL server must stay alive while in use")

            with server.connect_socket() as sock:
                ready, frames = self._pump_until(
                    sock,
                    lambda got: any(can_id == CAN_ID_SIL_OUTPUT_STATUS for can_id, _ in got),
                    timeout_sec=10.0,
                )
                self.assertTrue(
                    ready,
                    f"SIL server must stream CAN ID 0x{CAN_ID_SIL_OUTPUT_STATUS:03X}; saw {sorted({hex(c) for c, _ in frames})}",
                )

                statuses = [
                    payload
                    for can_id, payload in frames
                    if can_id == CAN_ID_SIL_OUTPUT_STATUS
                ]
                first = sil_test_support.unpack_sil_output_status(statuses[0])
                self.assertEqual(len(first[0]), 8)
                self.assertTrue(all(1000 <= pwm <= 2000 for pwm in first[0]))
                self.assertGreaterEqual(first[3], 0)

                # Neutral command traffic must not disturb the stream.
                send_raw_frame(
                    sock,
                    CAN_ID_THRUSTER_CMD,
                    sil_test_support.ThrusterCommand(pwm_us=NEUTRAL_PWMS).pack(),
                )
                send_raw_frame(
                    sock,
                    CAN_ID_SOLENOID_CMD,
                    sil_test_support.SolenoidCommand(solenoid_mask=0x0000).pack(),
                )
                still_alive, more = self._pump_until(
                    sock,
                    lambda got: any(can_id == CAN_ID_SIL_OUTPUT_STATUS for can_id, _ in got),
                    timeout_sec=5.0,
                )
                self.assertTrue(still_alive, "Status streaming must continue after command frames")

                later = [
                    payload
                    for can_id, payload in more
                    if can_id == CAN_ID_SIL_OUTPUT_STATUS
                ]
                self.assertTrue(later)
                self.assertGreaterEqual(
                    sil_test_support.unpack_sil_output_status(later[-1])[3],
                    first[3],
                    "Virtual simulation time must be monotonic",
                )

        self.assertIsNone(server.process, "stop() must clear the process handle")

    def test_context_manager_terminates_server_on_failure(self):
        """A raising test body must still leave no live child and no live listener.

        ``stop()`` unconditionally sets ``self.process = None``, so the handle
        being cleared proves nothing.  These assertions check the OS-level facts
        instead: the child really exited, and nothing is still serving the port.
        The post-teardown connect check is preferred over a rebind probe because
        the C server sets ``SO_REUSEADDR`` (``sil_bridge_server.c:184``), which on
        Windows lets a second ``bind()`` succeed against a still-listening stale
        server and hand ``accept()`` traffic to the first listener.
        """
        executable = require_server_executable()
        captured = SilServerProcess(executable)

        with self.assertRaises(ZeroDivisionError):
            with captured:
                proc = captured.process
                self.assertIsNotNone(proc, "start() must publish a process handle")
                # Prove the port is genuinely served while the block is alive, so
                # the refusal asserted below is a real signal, not a vacuous pass.
                self.assertFalse(
                    _port_refuses_connections(captured.port),
                    "The SIL server must be accepting connections while the with block runs",
                )
                1 / 0

        self.assertIsNone(captured.process, "stop() must clear the process handle")
        self.assertIsNotNone(
            proc.poll(),
            f"SIL server child must have exited after teardown; returncode={proc.poll()}",
        )
        self.assertTrue(
            _port_refuses_connections(captured.port),
            f"Port {captured.port} must no longer be served: a stale listener survived teardown",
        )


if __name__ == "__main__":
    unittest.main()
