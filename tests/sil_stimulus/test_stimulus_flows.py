"""
Tests for the canonical fake-CAN stimulus tool (``tools/can_stimulus.py``).

The tool replaces two divergent prototype scripts whose ``--auto`` runs reported
success while their own checks were raising.  The defect class that matters is
therefore not "does a check pass" but "can a check that crashed, timed out, or
never ran be reported as a pass".  Every test below is chosen to pin that:

* an exception raised inside a check becomes a FAILED ``CheckResult``;
* a report with any failure is not ok, and an empty report is not ok;
* ``main()`` returns nonzero for a failing report, for a raising check, and for
  an unreachable server, and zero only when every check passed;
* the solenoid check explains the firmware interlock instead of mistaking the
  all-zero mask the firmware leaves behind for a successful actuation.

These tests need no native binary.  The single server-backed case at the bottom
skips cleanly through ``require_server_executable()``.

Run from the repository root::

    python -m unittest tests.sil_stimulus.test_stimulus_flows -v
"""

import contextlib
import io
import socket
import struct
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

# tests/sil_stimulus/test_stimulus_flows.py -> tests/sil_stimulus -> tests -> repo root
REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools import can_stimulus  # noqa: E402  (path setup must precede the import)
from tools.can_stimulus import (  # noqa: E402
    CAN_ID_EFUSE_FAULT_ALERT,
    CAN_ID_EMERGENCY_BREAK,
    CAN_ID_ENV_TELEMETRY,
    CAN_ID_NAV_TELEMETRY,
    CAN_ID_SIL_OUTPUT_STATUS,
    CAN_ID_THRUSTER_CMD,
    ESC_ARMING_SIM_MS,
    NEUTRAL_PWMS,
    NEUTRAL_US,
    PWM_MAX_US,
    PWM_MIN_US,
    SIL_PACKET_SIZE,
    CheckResult,
    OutputStatus,
    SilSocketBackend,
    StimulusReport,
    VehicleStimulusTester,
    main,
    pack_sil_can_frame,
    run_selected_action,
)
from sil_protocol import SIL_OUTPUT_STATUS_STRUCT  # noqa: E402  (via can_stimulus's sys.path setup)

try:
    from .sil_test_support import SilServerProcess, free_tcp_port, require_server_executable
except ImportError:  # pragma: no cover - direct execution fallback
    from sil_test_support import (  # type: ignore[no-redef]
        SilServerProcess,
        free_tcp_port,
        require_server_executable,
    )


def status(pwms=None, brake_active=False, solenoids=0, sim_time_ms=0):
    """Build a decoded 0x7FE output snapshot the way the real backend does."""
    return OutputStatus(
        pwms=tuple(pwms) if pwms is not None else tuple(NEUTRAL_PWMS),
        brake_active=brake_active,
        solenoids=solenoids,
        sim_time_ms=sim_time_ms,
    )


class FakeBackend:
    """
    Transport double implementing the same contract as ``SilSocketBackend``.

    Output snapshots and ordinary frames are scripted separately because the real
    backend consumes 0x7FE into ``latest_outputs()`` and never returns those
    frames to the listener (0x7FE is a SIL-only mock-BSP channel).  A 0x7FE id
    placed in ``frames`` is routed to the readback here too, so the double cannot
    quietly disagree with the real routing.
    """

    def __init__(self, statuses=(), frames=(), connect_error=None, send_error=None):
        self.statuses = list(statuses)
        self.frames = list(frames)
        self.sent = []
        self.listener = None
        self.closed = False
        self.connect_error = connect_error
        self.send_error = send_error
        self._latest = None

    def connect(self, timeout_s=None):
        if self.connect_error is not None:
            raise self.connect_error

    def set_frame_listener(self, listener):
        previous, self.listener = self.listener, listener
        return previous

    def send_frame(self, can_id, payload):
        if self.send_error is not None:
            raise self.send_error
        self.sent.append((can_id, bytes(payload)))

    def recv_frame(self, timeout_s=0.0):
        if self.statuses:
            self._latest = self.statuses.pop(0)
        if not self.frames:
            return None
        can_id, payload = self.frames.pop(0)
        if can_id == CAN_ID_SIL_OUTPUT_STATUS:
            self._latest = OutputStatus.decode(payload)
            return None
        if self.listener is not None:
            self.listener(can_id, payload)
        return can_id, payload

    def drain_frames(self, timeout_s=0.0):
        # One read window yields a fresh 0x7FE snapshot plus whatever telemetry
        # arrived with it, so the readback advances even when no telemetry is
        # queued - exactly how the real backend behaves.
        collected = []
        if self.statuses:
            self._latest = self.statuses.pop(0)
        while self.frames:
            frame = self.recv_frame(0.0)
            if frame is not None:
                collected.append(frame)
        return collected

    def latest_outputs(self):
        return self._latest

    def close(self):
        self.closed = True


class RecordingTester:
    """Records which tester methods ``run_selected_action`` dispatches to."""

    def __init__(self):
        self.calls = []

    def guard(self, name, action):
        """Same contract as VehicleStimulusTester.guard, for the dispatch test."""
        try:
            return action()
        except Exception as exc:  # noqa: BLE001 - deliberately total
            return CheckResult(name, False, f"check raised {type(exc).__name__}: {exc}")

    def _record(self, name, *args):
        self.calls.append((name, args))
        return CheckResult(name, True, f"{name} recorded")

    def monitor_node1_env(self, duration_s=3.0):
        return self._record("monitor_node1_env", duration_s)

    def monitor_node2_depth(self, duration_s=2.0):
        return self._record("monitor_node2_depth", duration_s)

    def monitor_node3_power(self, duration_s=3.0):
        return self._record("monitor_node3_power", duration_s)

    def test_node2_arming(self):
        return self._record("test_node2_arming")

    def test_node2_pwm(self, channel, pulse_us, duration_s=1.0):
        return self._record("test_node2_pwm", channel, pulse_us, duration_s)

    def test_node2_all(self, pulse_us, duration_s=1.0):
        return self._record("test_node2_all", pulse_us, duration_s)

    def test_node2_solenoid(self, mask, timeout_s=1.0):
        return self._record("test_node2_solenoid", mask, timeout_s)

    def test_emergency_break(self):
        return self._record("test_emergency_break")

    def raw_send(self, can_id, payload):
        return self._record("raw_send", can_id, payload)

    def sniff(self, duration_s=1.0):
        return self._record("sniff", duration_s)


def run_cli(argv, backend_factory=None, tester_class=None):
    """Run ``main()`` with stdout captured; return (status, printed text)."""
    buffer = io.StringIO()
    patches = []
    if backend_factory is not None:
        patches.append(mock.patch.object(can_stimulus, "build_backend", backend_factory))
    if tester_class is not None:
        patches.append(mock.patch.object(can_stimulus, "VehicleStimulusTester", tester_class))
    for patch in patches:
        patch.start()
    try:
        with contextlib.redirect_stdout(buffer):
            status_code = main(argv)
    finally:
        for patch in reversed(patches):
            patch.stop()
    return status_code, buffer.getvalue()


class TestStimulusBackend(unittest.TestCase):
    """Transport-level contracts that need no server and no socket."""

    def test_backend_rejects_oversized_payload(self):
        backend = SilSocketBackend("127.0.0.1", 1, auto_start=False)
        with self.assertRaises(ValueError):
            backend.send_frame(CAN_ID_THRUSTER_CMD, b"x" * 65)

    def test_backend_accepts_a_maximum_length_payload(self):
        """Invariant: exactly 64 bytes is legal; only 65 is oversized."""
        backend = SilSocketBackend("127.0.0.1", 1, auto_start=False)
        with self.assertRaises(Exception) as caught:
            backend.send_frame(CAN_ID_THRUSTER_CMD, b"x" * 64)
        # A 64-byte payload passes the length gate and then fails on the missing
        # connection, which proves the length check is not the thing that fired.
        self.assertNotIsInstance(caught.exception, ValueError)

    def test_send_frame_reports_a_missing_connection(self):
        backend = SilSocketBackend("127.0.0.1", 1, auto_start=False)
        with self.assertRaises(ConnectionError):
            backend.send_frame(CAN_ID_THRUSTER_CMD, b"\x00" * 16)

    def test_latest_outputs_is_none_before_any_readback(self):
        backend = SilSocketBackend("127.0.0.1", 1, auto_start=False)
        self.assertIsNone(backend.latest_outputs())

    def test_sil_backend_refuses_to_run_outside_sil_mode(self):
        """Invariant: a child process may only be launched in SIL mode."""
        backend = SilSocketBackend("127.0.0.1", free_tcp_port(), auto_start=True, mode="can")
        with self.assertRaises(RuntimeError) as caught:
            backend.connect()
        self.assertIn("SIL", str(caught.exception))

    def test_close_is_idempotent_without_a_connection(self):
        backend = SilSocketBackend("127.0.0.1", 1, auto_start=False)
        backend.close()
        backend.close()

    def test_unreachable_server_fails_instead_of_hanging(self):
        """
        Invariant: connecting to a dead port raises promptly and says what failed.

        The broken prototype's real failure mode was a check that blocked or
        crashed; a connect that hangs forever is the same defect in a slower
        disguise, so the elapsed time is asserted, not just the exception.
        """
        backend = SilSocketBackend("127.0.0.1", free_tcp_port(), auto_start=False)
        started = time.monotonic()
        with self.assertRaises(ConnectionError) as caught:
            backend.connect(timeout_s=2.0)
        self.assertLess(time.monotonic() - started, 10.0)
        self.assertIn("127.0.0.1", str(caught.exception))


class TestStimulusReport(unittest.TestCase):
    """Report aggregation: the thing the broken prototype got wrong."""

    def test_report_fails_when_a_check_fails(self):
        report = StimulusReport([CheckResult("node3", False, "missing frame")])
        self.assertFalse(report.ok)

    def test_report_passes_only_when_every_check_passes(self):
        report = StimulusReport([CheckResult("a", True, "ok"), CheckResult("b", True, "ok")])
        self.assertTrue(report.ok)

    def test_one_failure_among_successes_still_fails_the_report(self):
        report = StimulusReport(
            [CheckResult("a", True, "ok"), CheckResult("b", False, "timeout"), CheckResult("c", True, "ok")]
        )
        self.assertFalse(report.ok)

    def test_an_empty_report_is_not_a_pass(self):
        """
        Invariant: "nothing was checked" is not evidence of anything.

        This is the silent-success hole: a flag combination that selected no
        action would otherwise produce a vacuously-true report and exit 0.
        """
        self.assertFalse(StimulusReport([]).ok)

    def test_failures_are_listed_with_their_detail(self):
        report = StimulusReport(
            [CheckResult("a", True, "ok"), CheckResult("b", False, "no 0x7FE readback")]
        )
        failures = report.failures
        self.assertEqual([f.name for f in failures], ["b"])
        self.assertIn("no 0x7FE readback", report.format())


class TestStimulusCheckGuard(unittest.TestCase):
    """
    The direct test for the prototype defect: a check that raises.

    ``tools/can_stimulus.py --auto`` used to exit 0 while its own checks raised
    ``PowerTelemetry object has no attribute 'tether_v'``.  A raised exception
    must become a FAILED ``CheckResult`` carrying the text, never a traceback and
    never a pass.
    """

    def test_guard_turns_a_raised_exception_into_a_failed_check(self):
        tester = VehicleStimulusTester(FakeBackend())

        def boom():
            raise AttributeError("PowerTelemetry object has no attribute 'tether_v'")

        result = tester.guard("node3_power", boom)
        self.assertFalse(result.passed)
        self.assertIn("AttributeError", result.detail)
        self.assertIn("tether_v", result.detail)

    def test_guard_preserves_the_exception_type_name(self):
        tester = VehicleStimulusTester(FakeBackend())
        for exc_type in (ValueError, KeyError, OSError, RuntimeError):
            with self.subTest(exc=exc_type.__name__):
                def boom(exc_type=exc_type):
                    raise exc_type("simulated failure")

                result = tester.guard("check", boom)
                self.assertFalse(result.passed)
                self.assertIn(exc_type.__name__, result.detail)

    def test_guard_passes_a_successful_check_through_unchanged(self):
        tester = VehicleStimulusTester(FakeBackend())
        result = tester.guard("node1_env", lambda: CheckResult("node1_env", True, "12 frames"))
        self.assertTrue(result.passed)
        self.assertEqual(result.detail, "12 frames")

    def test_a_raising_check_fails_the_whole_report(self):
        """
        Invariant: aggregation keeps the failure instead of losing the check.

        A raising check must still appear in the report as a named failure, so
        the summary counts it and the process exits nonzero. Silently dropping it
        would leave a report that looks complete.
        """
        backend = FakeBackend(statuses=[status() for _ in range(8)], send_error=OSError("transport died"))
        tester = VehicleStimulusTester(backend)

        class Args:
            node1_env = False
            node2_arm = False
            node2_thruster = None
            node2_all = None
            node2_depth = None
            node2_solenoid = None
            node3_power = False
            emergency_break = True
            raw_send = None
            sniff = None

        report = run_selected_action(tester, Args())
        self.assertFalse(report.ok)
        self.assertEqual([f.name for f in report.failures], ["emergency_break"])
        self.assertIn("transport died", report.failures[0].detail)
        self.assertIn("OSError", report.failures[0].detail)
        self.assertIn("emergency_break", report.format())


class TestStimulusSelectedActions(unittest.TestCase):
    """Every requested flag maps to exactly one tester method."""

    def _args(self, **overrides):
        defaults = dict(
            node1_env=False,
            node2_arm=False,
            node2_thruster=None,
            node2_all=None,
            node2_depth=None,
            node2_solenoid=None,
            node3_power=False,
            emergency_break=False,
            raw_send=None,
            sniff=None,
        )
        defaults.update(overrides)
        return type("Args", (), defaults)()

    def test_no_flags_selects_nothing_and_does_not_pass(self):
        report = run_selected_action(RecordingTester(), self._args())
        self.assertEqual(report.results, [])
        self.assertFalse(report.ok)

    def test_every_flag_maps_to_its_own_check(self):
        tester = RecordingTester()
        report = run_selected_action(
            tester,
            self._args(
                node1_env=True,
                node2_arm=True,
                node2_thruster=(3, 1700),
                node2_all=1650,
                node2_depth=1.5,
                node2_solenoid=0x0001,
                node3_power=True,
                emergency_break=True,
                raw_send=[(CAN_ID_THRUSTER_CMD, b"\xdc\x05" * 8)],
                sniff=0.5,
            ),
        )
        self.assertEqual(
            [name for name, _ in tester.calls],
            [
                "monitor_node1_env",
                "test_node2_arming",
                "test_node2_pwm",
                "test_node2_all",
                "monitor_node2_depth",
                "test_node2_solenoid",
                "monitor_node3_power",
                "test_emergency_break",
                "raw_send",
                "sniff",
            ],
        )
        self.assertEqual(len(report.results), len(tester.calls))
        self.assertTrue(report.ok)

    def test_thruster_flag_carries_channel_and_pulse_through(self):
        tester = RecordingTester()
        run_selected_action(tester, self._args(node2_thruster=(5, 1900)))
        self.assertIn(("test_node2_pwm", (5, 1900, 1.0)), tester.calls)


class TestStimulusPwmAndSolenoidValidation(unittest.TestCase):
    """Arguments the firmware would reject must be rejected before the wire."""

    def test_pwm_outside_the_firmware_envelope_is_refused(self):
        tester = VehicleStimulusTester(FakeBackend())
        for bad in (PWM_MIN_US - 1, PWM_MAX_US + 1, 0, -1500, 2500):
            with self.subTest(pulse_us=bad):
                with self.assertRaises(ValueError):
                    tester.test_node2_pwm(0, bad)

    def test_thruster_channel_must_exist(self):
        tester = VehicleStimulusTester(FakeBackend())
        for bad in (-1, 8, 99):
            with self.subTest(channel=bad):
                with self.assertRaises(ValueError):
                    tester.test_node2_pwm(bad, 1700)

    def test_all_channels_pwm_is_range_checked(self):
        tester = VehicleStimulusTester(FakeBackend())
        with self.assertRaises(ValueError):
            tester.test_node2_all(PWM_MAX_US + 1)

    def test_solenoid_mask_within_range_is_accepted(self):
        # 0x0000 plus one coil per valve, taken from either side of each pair.
        for mask in (0x0000, 0x0001, 0x0002, 0x0004, 0x0005, 0x0015, 0x02AA, 0x0155):
            with self.subTest(mask=hex(mask)):
                self.assertIsNone(can_stimulus.conflicting_solenoid_valve(mask))

    def test_solenoid_mask_with_both_coils_of_a_pair_is_rejected(self):
        # Valve v owns bits 2v and 2v+1; energising both shorts the valve.
        for valve in range(5):
            pair = 0x3 << (2 * valve)
            with self.subTest(valve=valve):
                self.assertEqual(can_stimulus.conflicting_solenoid_valve(pair), valve)
                self.assertEqual(can_stimulus.conflicting_solenoid_valve(pair | 0x0001), valve)

    def test_all_ten_coils_is_rejected_as_a_whole_mask(self):
        """
        Invariant: 0x03FF is inside the 10-bit field but is not a valid mask.

        It is the case most likely to be sent by mistake, and the one where the
        firmware zeroes the *entire* mask rather than trimming one pair.
        """
        self.assertEqual(can_stimulus.conflicting_solenoid_valve(0x03FF), 0)

    def test_solenoid_check_explains_the_interlock_instead_of_reporting_a_zero_mask(self):
        """
        Invariant: an invalid mask fails *before* the wire and names the interlock.

        ``rov_can_unpack_solenoid_cmd`` zeroes the whole mask and returns
        ``ROV_ERR_INVALID_ARG`` when both coils of a valve are energised, and
        ``node2_app_step`` then skips ``bsp_solenoid_set`` entirely.  The readback
        would therefore still show whatever mask was there before, which says
        nothing about the frame just sent.  Reporting that stale (or zero) mask
        as a pass or as an actuation failure would be the wrong reason.
        """
        backend = FakeBackend(statuses=[status(solenoids=0)] * 8)
        tester = VehicleStimulusTester(backend)
        result = tester.test_node2_solenoid(0x0003)
        self.assertFalse(result.passed)
        self.assertIn("valve 0", result.detail)
        self.assertIn("rov_can_unpack_solenoid_cmd", result.detail)
        self.assertEqual(backend.sent, [], "An invalid mask must never reach the transport")

    def test_solenoid_check_fails_when_no_readback_arrives(self):
        tester = VehicleStimulusTester(FakeBackend())
        result = tester.test_node2_solenoid(0x0001, timeout_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("0x7FE", result.detail)

    def test_emergency_check_fails_when_no_readback_arrives(self):
        tester = VehicleStimulusTester(FakeBackend())
        result = tester.test_emergency_break(timeout_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("0x7FE", result.detail)

    def test_emergency_check_refuses_to_claim_a_brake_that_was_already_tripped(self):
        """
        Invariant: a trip that was already latched before the frame cannot be
        attributed to the authorized signature, so the check must not pass.
        """
        backend = FakeBackend(statuses=[status(brake_active=True)] * 8)
        tester = VehicleStimulusTester(backend)
        result = tester.test_emergency_break(timeout_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("already", result.detail.lower())

    def test_raw_send_refuses_an_unsigned_safety_frame(self):
        """
        Invariant: an unsigned 0x001 or 0x005 is never written.

        The firmware ignores both without their signature (node2 app.c:112 and
        :121), so a tool that wrote one and reported success would be reporting
        something the vehicle never saw.
        """
        for can_id in (CAN_ID_EMERGENCY_BREAK, CAN_ID_EFUSE_FAULT_ALERT):
            with self.subTest(can_id=hex(can_id)):
                backend = FakeBackend()
                result = VehicleStimulusTester(backend).raw_send(can_id, b"\x00\x00")
                self.assertFalse(result.passed)
                self.assertIn("authorization signature", result.detail)
                self.assertEqual(backend.sent, [])

    def test_raw_send_writes_a_signed_safety_frame(self):
        backend = FakeBackend()
        result = VehicleStimulusTester(backend).raw_send(
            CAN_ID_EMERGENCY_BREAK, can_stimulus.EMERGENCY_FRAME
        )
        self.assertTrue(result.passed)
        self.assertEqual(backend.sent, [(CAN_ID_EMERGENCY_BREAK, can_stimulus.EMERGENCY_FRAME)])

    def test_raw_send_asserts_the_write_only(self):
        """
        Invariant: the pass text says what it does and does not claim, so an
        operator cannot read a transport write as a vehicle response.
        """
        result = VehicleStimulusTester(FakeBackend()).raw_send(CAN_ID_THRUSTER_CMD, b"\xdc\x05" * 8)
        self.assertTrue(result.passed)
        self.assertIn("not any vehicle response", result.detail)

    def test_full_smoke_order_arms_first_and_trips_the_brake_last(self):
        """
        Invariant: the ESC arming gate is observed before anything sends thrust,
        and the latching emergency break runs after every other check.

        The gate is 3000 ms of *simulated* time (app.c:24), so it closes whether
        or not anyone is watching; a command sent before the arming check would
        leave it with nothing to observe. The emergency break latches for the life
        of the engine (app.c:116 sets ESC_STATE_DISARMED and rov_safety.c has no
        clear path), so a check after it would only see a permanently disarmed
        board.
        """
        tester = VehicleStimulusTester(FakeBackend())
        order = []
        original = tester.guard

        def spy(name, action):
            order.append(name)
            return original(name, action)

        tester.guard = spy
        report = tester.run_full_smoke()
        self.assertEqual(len(order), len(report.results))
        self.assertEqual(order[0], "node2_arming")
        self.assertEqual(order[-1], "emergency_break")
        self.assertEqual(
            set(order),
            {
                "node2_arming",
                "node1_env",
                "node3_power",
                "node2_depth",
                "node2_pwm",
                "node2_all",
                "node2_solenoid",
                "emergency_break",
            },
        )


class TestStimulusCliStatus(unittest.TestCase):
    """``main()`` status is the contract the two prototypes broke."""

    def test_main_returns_zero_only_for_a_passing_report(self):
        passing = StimulusReport([CheckResult("node1_env", True, "3 frames decoded")])

        class PassingTester:
            def __init__(self, backend):
                pass

            def run_full_smoke(self):
                return passing

        status_code, printed = run_cli(["--mode", "sil", "--auto"], tester_class=PassingTester)
        self.assertEqual(status_code, 0)
        self.assertIn("node1_env", printed)

    def test_main_returns_one_and_names_every_failure(self):
        failing = StimulusReport(
            [
                CheckResult("node1_env", True, "3 frames decoded"),
                CheckResult("node2_solenoid", False, "no 0x7FE readback within 1.0 s"),
                CheckResult("node3_power", False, "check raised KeyError: 'tether_current_ma'"),
            ]
        )

        class FailingTester:
            def __init__(self, backend):
                pass

            def run_full_smoke(self):
                return failing

        status_code, printed = run_cli(["--mode", "sil", "--auto"], tester_class=FailingTester)
        self.assertEqual(status_code, 1)
        self.assertIn("node2_solenoid", printed)
        self.assertIn("no 0x7FE readback", printed)
        self.assertIn("node3_power", printed)
        self.assertIn("tether_current_ma", printed)
        self.assertIn("2 check(s) failed", printed)

    def test_main_returns_one_when_a_check_raises(self):
        class ExplodingTester:
            def __init__(self, backend):
                pass

            def run_full_smoke(self):
                raise RuntimeError("stimulus run blew up")

        status_code, printed = run_cli(["--mode", "sil", "--auto"], tester_class=ExplodingTester)
        self.assertEqual(status_code, 1)
        self.assertIn("stimulus run blew up", printed)

    def test_main_returns_one_for_a_real_check_that_raises_on_the_wire(self):
        """
        Invariant: a transport fault during a real check is a failure, not a pass.

        This is the end-to-end version of the prototype bug, driven through the
        real ``VehicleStimulusTester``: the emergency check reads a clean
        untripped snapshot, then the transport raises while sending.
        """
        backend = FakeBackend(statuses=[status(brake_active=False)] * 8)
        backend.send_error = OSError("broken pipe")

        def factory(args):
            return backend

        status_code, printed = run_cli(["--mode", "sil", "--emergency-break"], backend_factory=factory)
        self.assertEqual(status_code, 1)
        self.assertIn("broken pipe", printed)

    def test_main_returns_nonzero_when_no_action_is_selected(self):
        status_code, printed = run_cli(["--mode", "sil"], backend_factory=lambda args: FakeBackend())
        self.assertNotEqual(status_code, 0)
        self.assertIn("--auto", printed)

    def test_main_reports_an_unreachable_server_without_hanging(self):
        dead_port = free_tcp_port()

        def factory(args):
            # auto_start=False: nothing must be launched for a dead port, so the
            # run exercises the real socket failure path.
            return SilSocketBackend(args.host, args.port, auto_start=False)

        started = time.monotonic()
        status_code, printed = run_cli(
            ["--mode", "sil", "--port", str(dead_port), "--node1-env"], backend_factory=factory
        )
        elapsed = time.monotonic() - started
        self.assertNotEqual(status_code, 0)
        self.assertLess(elapsed, 30.0, "An unreachable server must fail fast, not hang")
        self.assertIn(str(dead_port), printed)
        self.assertIn("127.0.0.1", printed)

    def test_main_closes_the_backend_even_when_a_check_raises(self):
        backend = FakeBackend(statuses=[status(brake_active=False)] * 8)
        backend.send_error = OSError("broken pipe")
        run_cli(["--mode", "sil", "--emergency-break"], backend_factory=lambda args: backend)
        self.assertTrue(backend.closed, "main() must close the transport on every path")


class TestOptionalHardwareBackends(unittest.TestCase):
    """The hardware adapters must never fail at import time."""

    def test_python_can_backend_reports_the_missing_dependency_clearly(self):
        backend = can_stimulus.PythonCanBackend(interface="vcan0")
        if can_stimulus._optional_module_available("can"):
            self.skipTest("python-can is installed, so the missing-dependency path cannot be tested")
        with self.assertRaises(RuntimeError) as caught:
            backend.connect()
        message = str(caught.exception)
        self.assertIn("python-can", message)
        self.assertNotIsInstance(caught.exception, ImportError)

    def test_uart_backend_reports_a_missing_dependency_clearly(self):
        backend = can_stimulus.UartBackend(port="/dev/definitely-not-a-tty")
        if not can_stimulus._optional_module_available("serial"):
            with self.assertRaises(RuntimeError) as caught:
                backend.connect()
            self.assertIn("pyserial", str(caught.exception))
        else:
            with self.assertRaises(RuntimeError):
                backend.connect()

    def test_hardware_backends_have_no_sil_output_readback(self):
        """
        Invariant: 0x7FE is a SIL-only mock-BSP channel, so a real bus can never
        satisfy the readback the checks demand; the adapter must say so rather
        than fabricate a snapshot.
        """
        for backend in (can_stimulus.PythonCanBackend(), can_stimulus.UartBackend()):
            with self.subTest(backend=type(backend).__name__):
                self.assertIsNone(backend.latest_outputs())

    def test_hardware_backends_reject_oversized_payloads_too(self):
        for backend in (can_stimulus.PythonCanBackend(), can_stimulus.UartBackend()):
            with self.subTest(backend=type(backend).__name__):
                with self.assertRaises(ValueError):
                    backend.send_frame(CAN_ID_THRUSTER_CMD, b"x" * 65)


class TestSilSocketFraming(unittest.TestCase):
    """
    Real ``SilSocketBackend`` against a loopback peer, no engine needed.

    Pins the three framing promises the tool makes to the tests: a packet split
    across TCP writes is reassembled across ``recv_frame`` calls, decoding yields
    ``(can_id, payload)``, and a 0x7FE snapshot lands in the readback instead of
    being handed to the caller.
    """

    STATUS_PWMS = (1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500)
    STATUS_BRAKE = True
    STATUS_SOLENOIDS = 0x0001
    STATUS_SIM_MS = 123456

    def setUp(self):
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(1)
        self.port = self.listener.getsockname()[1]
        self.addCleanup(self.listener.close)
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()
        self.addCleanup(self.thread.join, 2.0)

    def _status_payload(self):
        return SIL_OUTPUT_STATUS_STRUCT.pack(
            *self.STATUS_PWMS,
            self.STATUS_BRAKE,
            self.STATUS_SOLENOIDS,
            self.STATUS_SIM_MS,
        )

    def _serve(self):
        try:
            conn, _ = self.listener.accept()
        except OSError:  # pragma: no cover - only if the test tears down first
            return
        with conn:
            packet = pack_sil_can_frame(CAN_ID_SIL_OUTPUT_STATUS, self._status_payload())
            # Split the packet across two writes with a gap longer than one read
            # window, so a reader without a cross-call buffer sees a short frame
            # and desynchronises every later boundary.
            conn.sendall(packet[: SIL_PACKET_SIZE // 2])
            time.sleep(0.20)
            conn.sendall(packet[SIL_PACKET_SIZE // 2 :])
            conn.sendall(pack_sil_can_frame(CAN_ID_NAV_TELEMETRY, b"\x01" * 41))
            time.sleep(1.0)

    def test_split_packet_is_reassembled_across_calls(self):
        backend = SilSocketBackend("127.0.0.1", self.port, auto_start=False)
        backend.connect(timeout_s=2.0)
        self.addCleanup(backend.close)

        # Only half a packet is available, and the read window closes well before
        # the rest arrives: the correct answer is "nothing yet", not a short
        # frame and not a desynchronised one.
        self.assertIsNone(backend.recv_frame(timeout_s=0.02))

        frame = backend.recv_frame(timeout_s=2.0)
        self.assertIsNotNone(frame, "the second half of the packet must complete the first")
        can_id, payload = frame
        self.assertEqual(can_id, CAN_ID_NAV_TELEMETRY)
        self.assertEqual(len(payload), 41)

        snapshot = backend.latest_outputs()
        self.assertIsNotNone(snapshot, "the 0x7FE snapshot must be recorded, not returned")
        self.assertEqual(snapshot.pwms, self.STATUS_PWMS)
        self.assertTrue(snapshot.brake_active)
        self.assertEqual(snapshot.solenoids, self.STATUS_SOLENOIDS)
        self.assertEqual(snapshot.sim_time_ms, self.STATUS_SIM_MS)

    def test_frame_listener_sees_telemetry_but_not_the_output_channel(self):
        backend = SilSocketBackend("127.0.0.1", self.port, auto_start=False)
        backend.connect(timeout_s=2.0)
        self.addCleanup(backend.close)
        seen = []
        backend.set_frame_listener(lambda can_id, payload: seen.append(can_id))
        backend.drain_frames(2.0)
        self.assertIn(CAN_ID_NAV_TELEMETRY, seen)
        self.assertNotIn(CAN_ID_SIL_OUTPUT_STATUS, seen)


class TestStimulusAgainstServer(unittest.TestCase):
    """
    End-to-end CLI coverage against the real native server.

    Deliberately one CLI-level test only: the per-node assertions belong to the
    integration task, and this file's job is the tool's status contract.
    """

    @classmethod
    def setUpClass(cls):
        cls.server_exe = require_server_executable()

    def setUp(self):
        self.server = SilServerProcess(self.server_exe)
        self.server.start()
        self.addCleanup(self.server.stop)

    def test_auto_cli_succeeds_against_the_real_server(self):
        status_code, printed = run_cli(["--mode", "sil", "--port", str(self.server.port), "--auto"])
        self.assertEqual(
            status_code,
            0,
            f"--auto must pass against the real server. Output:\n{printed}",
        )
        self.assertNotIn("[FAIL]", printed)


if __name__ == "__main__":
    unittest.main()
