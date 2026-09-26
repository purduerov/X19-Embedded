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

import ast
import contextlib
import inspect
import io
import re
import socket
import struct
import subprocess
import sys
import threading
import time
import unittest
from pathlib import Path
from typing import NamedTuple
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
    CAN_ID_POWER_TELEMETRY,
    CAN_ID_SIL_OUTPUT_STATUS,
    CAN_ID_SOLENOID_CMD,
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
    unpack_sil_can_frame,
)
from sil_protocol import (  # noqa: E402  (via can_stimulus's sys.path setup)
    SIL_OUTPUT_STATUS_STRUCT,
    EnvTelemetry,
    PowerTelemetry,
    ThrusterCommand,
)

try:
    from .sil_test_support import (
        ServerBackedTestCase,
        SilServerProcess,
        free_tcp_port,
        require_server_executable,
    )
except ImportError:  # pragma: no cover - direct execution fallback
    from sil_test_support import (  # type: ignore[no-redef]
        ServerBackedTestCase,
        SilServerProcess,
        free_tcp_port,
        require_server_executable,
    )


# The pulse the post-ESTOP probe uses; mirrored so a scripted board can reproduce
# the channels coming back off neutral.
POST_ESTOP_PULSE_US = can_stimulus.POST_ESTOP_PROBE_PWM_US


def status(pwms=None, brake_active=False, solenoids=0, sim_time_ms=0):
    """Build a decoded 0x7FE output snapshot the way the real backend does."""
    return OutputStatus(
        pwms=tuple(pwms) if pwms is not None else tuple(NEUTRAL_PWMS),
        brake_active=brake_active,
        solenoids=solenoids,
        sim_time_ms=sim_time_ms,
    )


# Telemetry payload builders for the injected stream.
#
# These are test *fixtures*, not a second definition of the wire: the checks
# decode with the public sil_protocol unpackers, so if either side changes the
# fixture and the check stop agreeing and the failure-path tests fail loudly.
# Each format is the one sil_protocol uses (its module-level struct objects) and
# mirrors the matching field order in shared/include/rov_can_protocol.h:
#   env   "<3fB"  pressure_hpa, humidity_pct, temperature_c, leak_flags  (13 B)
#   nav   "<Q8fB" timestamp_us, 4 quat, 3 gyro, depth_meters, imu_status  (41 B)
#   power "<HHHH4HhH" tether V/mA, 5 V V/mA, 4x 12 V mA, pcb temp tenths, flags (20 B)
# The checks' own minimum lengths come from sil_protocol's unpack guards.
def env_payload(pressure_hpa=1013.25, humidity_pct=35.0, temperature_c=24.0, leak_flags=0):
    return struct.pack("<3fB", pressure_hpa, humidity_pct, temperature_c, leak_flags)


def nav_payload(depth_meters=0.0, imu_status=3, timestamp_us=1000):
    return struct.pack(
        "<Q8fB",
        timestamp_us,
        1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        depth_meters,
        imu_status,
    )


def power_payload(
    tether_voltage_mv=48000,
    tether_current_ma=670,
    v5_voltage_mv=5200,
    v5_current_ma=2500,
    v12_current_ma=(400, 400, 400, 400),
    pcb_temp_c_tenths=300,
    status_flags=0,
):
    return struct.pack(
        "<HHHH4HhH",
        tether_voltage_mv,
        tether_current_ma,
        v5_voltage_mv,
        v5_current_ma,
        *v12_current_ma,
        pcb_temp_c_tenths,
        status_flags,
    )



class _ScriptedTransport:
    """
    Shared scripted-frame behaviour for the transport doubles.

    ``statuses`` are 0x7FE snapshots handed to the readback one per read window;
    ``frames`` are ``(can_id, payload)`` pairs handed to the frame listener.  A
    0x7FE id placed in ``frames`` is routed to the readback instead, so a double
    cannot quietly disagree with the real backend's routing.
    """

    def __init__(self, statuses=(), frames=(), connect_error=None, send_error=None):
        # ``statuses`` may be a list or a callable stream; see _next_status.
        self.statuses = statuses if callable(statuses) else list(statuses)
        # True by default because every SIL-shaped double supplies the readback;
        # NonSilBackend overrides it to False.
        self.provides_output_readback = True
        self.frames = list(frames)
        self.sent = []
        self.listener = None
        self.closed = False
        self.connect_error = connect_error
        self.send_error = send_error
        self._latest = None
        self._last_status = None
        self._read_index = 0

    def _next_status(self):
        """
        Advance the scripted snapshot stream by one read.

        Three properties make the scripts in this file deterministic:

        * ``statuses`` may be a plain list (consumed in order, last one then held)
          or a callable ``index -> OutputStatus | None``, which is how the phase
          scripts below express "neutral for a while, then armed, then braked"
          without depending on exactly how many reads a pump performs;
        * the last value is held once the list is exhausted, because real firmware
          keeps reporting its state between commands. An empty stream means "the
          engine never published a snapshot" and stays silent;
        * each read costs half a poll slice, so a pump advances roughly one
          snapshot per slice the way a blocking socket read does. Without that a
          pump would spin through the whole script inside a single window.
        """
        if callable(self.statuses):
            self._read_index += 1
            self._last_status = self.statuses(self._read_index)
        elif self.statuses:
            self._last_status = self.statuses.pop(0)
        time.sleep(can_stimulus.READ_POLL_S / 2.0)
        return self._last_status

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
        if not self.frames:
            self._latest = self._next_status()
            return None
        can_id, payload = self.frames.pop(0)
        if can_id == CAN_ID_SIL_OUTPUT_STATUS:
            self._latest = OutputStatus.decode(payload)
            return None
        if self.listener is not None:
            self.listener(can_id, payload)
        return can_id, payload

    def describe_destination(self, can_id):
        return f"CAN 0x{can_id:03X}"

    def close(self):
        self.closed = True


class FakeBackend(_ScriptedTransport):
    """
    SIL-shaped transport: same surface as ``SilSocketBackend``, including the
    0x7FE readback and ``drain_frames``.
    """

    def drain_frames(self, timeout_s=0.0):
        # One read window yields a fresh 0x7FE snapshot plus whatever telemetry
        # arrived with it, so the readback advances even when no telemetry is
        # queued - exactly how the real backend behaves.
        collected = []
        self._latest = self._next_status()
        while self.frames:
            frame = self.recv_frame(0.0)
            if frame is not None:
                collected.append(frame)
        return collected

    def latest_outputs(self):
        return self._latest


class NonSilBackend(_ScriptedTransport):
    """
    Hardware-shaped transport, mirroring ``PythonCanBackend``/``UartBackend``.

    No 0x7FE readback and, deliberately, no ``drain_frames`` - so this double also
    exercises the ``_pump`` fallback branch that only the non-SIL adapters reach.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.provides_output_readback = False

    def latest_outputs(self):
        return None



class InstantTester(VehicleStimulusTester):
    """
    Every check answers immediately, so an ordering or streaming test costs no
    wall time. The real checks wait out real timeouts; these tests are about
    structure, not about any check's behaviour.
    """

    def _instant(self, name):
        return CheckResult(name, True, "stub")

    monitor_node1_env = lambda self, duration_s=3.0: self._instant("node1_env")  # noqa: E731
    monitor_node2_depth = lambda self, duration_s=2.0: self._instant("node2_depth")  # noqa: E731
    monitor_node3_power = lambda self, duration_s=3.0: self._instant("node3_power")  # noqa: E731
    test_node2_arming = lambda self, **kw: self._instant("node2_arming")  # noqa: E731
    test_node2_pwm = lambda self, channel, pulse_us, duration_s=1.0, **kw: self._instant("node2_pwm")  # noqa: E731
    test_node2_all = lambda self, pulse_us, duration_s=1.0, **kw: self._instant("node2_all")  # noqa: E731
    test_node2_solenoid = lambda self, mask, timeout_s=1.5: self._instant("node2_solenoid")  # noqa: E731
    test_emergency_break = lambda self, **kw: self._instant("emergency_break")  # noqa: E731


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

    def run_plan(self, plan):
        return StimulusReport([self.guard(name, action) for name, action in plan])

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
    """
    Run ``main()`` with stdout captured; return (status, printed text).

    ``backend_factory`` defaults to a throwaway ``FakeBackend`` on purpose.  The
    CLI status contract must be testable with no C build, no free port, and no
    engine process: an earlier version of this helper let ``main()`` fall through
    to the real ``build_backend``, which spawned a real 100 Hz engine on the
    fixed default port 8765 and raced any developer's running dashboard - and
    made three tests fail outright when the native binary was absent.  A test that
    genuinely needs a real transport must pass its own factory.
    """
    if backend_factory is None:
        def backend_factory(args, _fake=FakeBackend()):
            return _fake

    buffer = io.StringIO()
    patches = []
    if tester_class is not None:
        patches.append(mock.patch.object(can_stimulus, "VehicleStimulusTester", tester_class))
    for patch in patches:
        patch.start()
    try:
        with contextlib.redirect_stdout(buffer):
            status_code = main(argv, backend_factory=backend_factory)
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

    def test_guard_rejects_a_non_checkresult_return(self):
        """
        Invariant: a check that returns a bare bool cannot be a pass.

        This is the second half of the central defence: a stub, a refactor, or a
        check that forgets to return a ``CheckResult`` must fail closed rather
        than have its truthiness mistaken for a verdict.
        """
        tester = VehicleStimulusTester(FakeBackend())
        for returned in (True, False, None, [], {"passed": True}, "ok"):
            with self.subTest(returned=repr(returned)):
                result = tester.guard("check", lambda value=returned: value)
                self.assertFalse(result.passed)
                self.assertIn("instead of a CheckResult", result.detail)

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

        Driven with a stub tester: the real checks wait out real timeouts, and this
        is a statement about ordering, not about any check's behaviour.
        """
        tester = InstantTester(FakeBackend())
        report = tester.run_full_smoke()
        order = [result.name for result in report.results]
        self.assertEqual(len(order), 8)
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
        self.assertTrue(report.ok)


class TestStimulusCliStatus(unittest.TestCase):
    """
    ``main()`` status is the contract the two prototypes broke.

    Hermetic by construction: ``run_cli`` supplies a ``FakeBackend`` unless a test
    passes its own factory, so no test here can reach a socket, a build, or a
    process. The three tests that used to fall through to the real
    ``build_backend`` spawned a real 100 Hz engine on the fixed default port 8765
    and failed outright when the native binary was absent.
    """

    def test_main_returns_zero_only_for_a_passing_report(self):
        passing = StimulusReport([CheckResult("node1_env", True, "3 frames decoded")])

        class PassingTester:
            def __init__(self, backend, on_result=None):
                self.on_result = on_result

            def run_full_smoke(self):
                if self.on_result is not None:
                    for result in passing.results:
                        self.on_result(result)
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
            def __init__(self, backend, on_result=None):
                self.on_result = on_result

            def run_full_smoke(self):
                if self.on_result is not None:
                    for result in failing.results:
                        self.on_result(result)
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
            def __init__(self, backend, on_result=None):
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

        status_code, printed = run_cli(
            ["--mode", "sil", "--emergency-break"], backend_factory=lambda args: backend
        )
        self.assertEqual(status_code, 1)
        self.assertIn("broken pipe", printed)

    def test_main_delegates_its_summary_to_the_shared_formatter(self):
        """
        Invariant: the operator-facing summary has exactly one implementation.

        ``main`` streams every per-check line itself, so its closing block is
        ``StimulusReport.format(include_checks=False)`` and nothing else. It used
        to be a second hand-written copy of that text: a maintainer could change
        ``format()`` and the operator would never see it, or change ``main``'s copy
        and the format tests would stay green. Two copies of safety-relevant text
        that can drift is the defect, not the duplication being tidy.
        """
        source = inspect.getsource(can_stimulus.main)
        self.assertNotIn(
            "Summary:",
            source,
            "main() must not spell the summary out itself; StimulusReport.format "
            "owns that text",
        )
        self.assertIn("print_report(", source, "main() must delegate to print_report()")

    def test_the_operator_reads_exactly_what_the_shared_format_produces(self):
        """
        The shipped output and ``format()`` must be the same bytes, not two truths.

        This is the coupling the duplication defeated: it walks the real ``main``
        with a failing report and requires the tail of its stdout to equal
        ``StimulusReport.format(include_checks=False)`` exactly, so the
        ``Summary:`` line and every ``[FAIL]`` line the operator files are pinned
        to the one function the format tests already cover.
        """
        failing = StimulusReport(
            [
                CheckResult("node1_env", True, "3 frames decoded"),
                CheckResult("node2_solenoid", False, "no 0x7FE readback within 1.0 s"),
                CheckResult("node3_power", False, "check raised KeyError: 'tether_current_ma'"),
            ]
        )

        class FailingTester:
            def __init__(self, backend, on_result=None):
                self.on_result = on_result

            def run_full_smoke(self):
                if self.on_result is not None:
                    for result in failing.results:
                        self.on_result(result)
                return failing

        status_code, printed = run_cli(["--mode", "sil", "--auto"], tester_class=FailingTester)
        self.assertEqual(status_code, 1)
        expected = failing.format(include_checks=False).splitlines()
        self.assertEqual(
            printed.splitlines()[-len(expected):],
            expected,
            "main()'s closing block must be exactly format(include_checks=False)",
        )
        self.assertIn("Summary: 3 check(s), 1 passed, 2 failed", printed)
        self.assertIn("[FAIL] 2 check(s) failed: node2_solenoid, node3_power", printed)

    def test_main_returns_nonzero_when_no_action_is_selected(self):
        status_code, printed = run_cli(["--mode", "sil"])
        self.assertEqual(status_code, 2)
        self.assertIn("--auto", printed)

    def test_zero_valued_flags_are_actions_not_usage_errors(self):
        """
        Invariant: ``is not None``, not truthiness.

        ``--sniff 0``, ``--node2-depth 0`` and ``--node2-all 0`` are real
        requests. Reporting them as "no action was selected" hid the operator's
        intent behind a usage error and disagreed with ``run_selected_action``.
        """
        for argv, expected in (
            (["--sniff", "0"], "no frames decoded"),
            (["--node2-depth", "0"], "no 0x200 navigation telemetry"),
        ):
            with self.subTest(argv=argv):
                status_code, printed = run_cli(argv)
                self.assertNotEqual(status_code, 2, f"{argv} was treated as no action at all")
                self.assertIn(expected, printed)

    def test_auto_with_other_flags_says_they_are_ignored(self):
        # InstantTester, because this is about the warning and not about any
        # check: the real run_full_smoke over an empty FakeBackend waits out its
        # timeouts and cost about 27 s of the suite for a one-line message.
        status_code, printed = run_cli(
            ["--mode", "sil", "--auto", "--node2-solenoid", "3"], tester_class=InstantTester
        )
        self.assertIn("[WARN]", printed)
        self.assertIn("--node2-solenoid", printed)
        self.assertIn("ignored", printed)

    def test_results_are_streamed_as_they_complete(self):
        """
        Invariant: a long run leaves per-check evidence even if it is killed.

        The whole point of the tool is the evidence, and a report that only
        appears at the end of a 30 s run leaves nothing behind when the run dies.
        Driven through a stub tester so it costs no engine time.
        """
        seen = []
        tester = InstantTester(
            FakeBackend(), on_result=lambda result: seen.append(can_stimulus.format_result(result))
        )
        report = tester.run_full_smoke()
        self.assertEqual(len(seen), len(report.results))
        self.assertTrue(all(line.startswith("[PASS]") for line in seen), seen)
        self.assertIn("node2_arming", seen[0])

    def test_main_reports_an_unreachable_server_without_hanging(self):
        """
        The one CLI test that does use a real transport, and it is a dead one.

        ``auto_start=False`` guarantees nothing is launched, so this exercises the
        real socket failure path without a build, a free port, or a process.
        """
        dead_port = free_tcp_port()

        def factory(args):
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
        """
        Invariant: the transport is closed on the failing path too.

        ``--raw-send`` is used because it writes a frame unconditionally, so the
        transport raises on the first send rather than on some later poll. A leak
        here would leave a real engine or socket open for the rest of the session.
        """
        backend = FakeBackend()
        backend.send_error = OSError("broken pipe")
        status_code, printed = run_cli(
            ["--mode", "sil", "--raw-send", f"0x{CAN_ID_THRUSTER_CMD:03X}:DC05DC05DC05DC05DC05DC05DC05DC05"],
            backend_factory=lambda args: backend,
        )
        self.assertTrue(backend.closed, "main() must close the transport on every path")
        self.assertIn("broken pipe", printed)
        self.assertNotEqual(status_code, 0)

    def test_the_status_suite_never_reaches_the_real_backend_builder(self):
        """
        Structural guard on the defect the review found.

        A CLI test that reaches ``build_backend`` spawns a real 100 Hz engine on
        the fixed default port 8765 and fails outright when the native binary is
        absent. ``run_cli`` is the only door to ``main()`` here and always supplies
        a transport, so the real builder must never be consulted.
        """

        class Stub:
            def __init__(self, backend, on_result=None):
                pass

            def run_full_smoke(self):
                return StimulusReport([CheckResult("x", True, "ok")])

        reached = []
        original = can_stimulus.build_backend

        def spy(args):
            reached.append(args)
            return original(args)

        with mock.patch.object(can_stimulus, "build_backend", spy):
            run_cli(["--mode", "sil", "--auto"], tester_class=Stub)
        self.assertEqual(reached, [], "run_cli must not reach the real backend builder")


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
        # attach_existing=True: the loopback peer is this test's own engine.
        backend = SilSocketBackend("127.0.0.1", self.port, auto_start=False, attach_existing=True)
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
        backend = SilSocketBackend("127.0.0.1", self.port, auto_start=False, attach_existing=True)
        backend.connect(timeout_s=2.0)
        self.addCleanup(backend.close)
        seen = []
        backend.set_frame_listener(lambda can_id, payload: seen.append(can_id))
        backend.drain_frames(2.0)
        self.assertIn(CAN_ID_NAV_TELEMETRY, seen)
        self.assertNotIn(CAN_ID_SIL_OUTPUT_STATUS, seen)

    # -- the output-status rule, single-sourced ----------------------------
    #
    # The loopback peer above sends exactly one 0x7FE and one 0x200, so each of
    # these tests sees the whole script and the two-frame result is exact rather
    # than approximate.
    def _connected(self):
        # attach_existing=True: this loopback peer is a stand-in for an engine the
        # test started, so the backend legitimately attaches to it.
        backend = SilSocketBackend("127.0.0.1", self.port, auto_start=False, attach_existing=True)
        backend.connect(timeout_s=2.0)
        self.addCleanup(backend.close)
        return backend

    def _assert_readback_decoded(self, backend):
        """The withheld 0x7FE must still have been decoded into the readback."""
        snapshot = backend.latest_outputs()
        self.assertIsNotNone(
            snapshot, "withholding 0x7FE from the returned list must not skip decoding it"
        )
        self.assertEqual(snapshot.pwms, self.STATUS_PWMS)
        self.assertTrue(snapshot.brake_active)
        self.assertEqual(snapshot.solenoids, self.STATUS_SOLENOIDS)
        self.assertEqual(snapshot.sim_time_ms, self.STATUS_SIM_MS)

    def test_drain_frames_withholds_the_output_status_channel_by_default(self):
        """
        0x7FE is a mock-BSP readout, not vehicle traffic, so it is not returned.

        It must still reach ``latest_outputs()``: withholding a frame and skipping
        it would be different defects, and only one of them is wanted.
        """
        backend = self._connected()
        ids = [can_id for can_id, _ in backend.drain_frames(2.0)]
        self.assertNotIn(
            CAN_ID_SIL_OUTPUT_STATUS,
            ids,
            "drain_frames must withhold 0x7FE by default; the engine emits it every 10 ms tick, "
            "so a caller counting this list would be counting a mock-BSP channel",
        )
        self.assertIn(CAN_ID_NAV_TELEMETRY, ids, "real telemetry must still be returned")
        self._assert_readback_decoded(backend)

    def test_drain_frames_returns_the_output_status_channel_on_request(self):
        """The opt-in is the documented escape hatch for the raw stream."""
        backend = self._connected()
        frames = backend.drain_frames(2.0, include_output_status=True)
        ids = [can_id for can_id, _ in frames]
        self.assertIn(CAN_ID_SIL_OUTPUT_STATUS, ids, "include_output_status=True must return 0x7FE")
        self.assertIn(CAN_ID_NAV_TELEMETRY, ids, "the opt-in must not cost the telemetry frames")
        self._assert_readback_decoded(backend)
        status_payloads = [payload for can_id, payload in frames if can_id == CAN_ID_SIL_OUTPUT_STATUS]
        self.assertEqual(
            status_payloads[0],
            self._status_payload(),
            "the opt-in must return the frame verbatim, not a re-encoding",
        )

    def test_drain_frames_and_recv_frame_agree_about_the_output_status_rule(self):
        """
        The two read paths must not drift apart on the same channel.

        This is the regression the shared ``_wanted`` predicate exists to prevent:
        ``drain_frames`` used to return 0x7FE while ``recv_frame`` documented that
        it never would, and a ``sniff``-style caller fed from ``drain_frames``
        would have reported counts dominated by the mock channel.
        """
        backend = self._connected()
        frame = backend.recv_frame(timeout_s=2.0)
        self.assertIsNotNone(frame, "the peer's 0x200 must be reachable")
        self.assertEqual(
            frame[0],
            CAN_ID_NAV_TELEMETRY,
            "recv_frame must skip the 0x7FE and return the first real frame, not None",
        )
        self._assert_readback_decoded(backend)
        self.assertEqual(
            [can_id for can_id, _ in backend.drain_frames(0.2)],
            [],
            "drain_frames must apply the same exclusion, so the already-consumed queue yields "
            "nothing rather than the 0x7FE that recv_frame swallowed",
        )


class _RecordingEngine:
    """
    A bare TCP peer standing in for an engine somebody ELSE started.

    Accepts one connection and records every SIL packet it is sent, so a test can
    assert on what the tool actually put on the wire rather than on what the tool
    says it did.  It sends nothing back: this test is about the refusal arriving
    before any write, not about a readback.

    ``RECV_POLL_S`` is the peer's own read window, and the observe/settle helpers
    below are expressed in multiples of it, so "has the recorder caught up?" is
    answered against the recorder's real cadence rather than a guessed sleep.
    """

    RECV_POLL_S = 0.2

    def __init__(self, listening: bool = True):
        # Bind first and listen second, so a peer can be created on a known port
        # that is not yet reachable: a connect() to a bound-but-not-listening port
        # is refused, which is what makes the auto-start branch testable without
        # launching anything.
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind(("127.0.0.1", 0))
        self.port = self.listener.getsockname()[1]
        self.frames = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        if listening:
            self.open()

    def open(self):
        if self._thread is not None:
            return
        self.listener.listen(1)
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _serve(self):
        try:
            conn, _ = self.listener.accept()
        except OSError:  # pragma: no cover - only if the test tears down first
            return
        with conn:
            buf = b""
            conn.settimeout(self.RECV_POLL_S)
            while not self._stop.is_set():
                try:
                    chunk = conn.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    return
                if not chunk:
                    return
                buf += chunk
                while len(buf) >= SIL_PACKET_SIZE:
                    can_id, payload = unpack_sil_can_frame(buf[:SIL_PACKET_SIZE])
                    buf = buf[SIL_PACKET_SIZE:]
                    with self._lock:
                        self.frames.append((can_id, payload))

    def ids(self):
        with self._lock:
            return [can_id for can_id, _ in self.frames]

    def count(self):
        with self._lock:
            return len(self.frames)

    def wait_for_count(self, minimum: int, timeout_s: float = 2.0) -> bool:
        """
        Bounded wait for at least ``minimum`` frames to be recorded.

        ``_serve`` parses on its own thread, so ``ids()`` read straight after a
        synchronous write is a *lower bound* and can lag by a frame.  This peer
        had the same defect as the dashboard's wire recorder -- a test asserted
        ``[CAN_ID_THRUSTER_CMD]`` immediately after the CLI returned, and on a
        loaded host the reader thread had not got there yet and the list was
        empty.  Polling to convergence is the honest form of "the frame arrived".
        """
        deadline = time.monotonic() + timeout_s
        while True:
            if self.count() >= minimum:
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.005)

    def settle(self, quiet_s: float = 2 * RECV_POLL_S, timeout_s: float = 2.0) -> int:
        """
        Return the frame count once the recorder has stopped seeing new frames.

        The counterpart to :meth:`wait_for_count`, for the "nothing else was
        written" claim, which cannot be expressed as a wait for something to
        appear.  Settling means "no new frame for two full read windows", the
        smallest observation that distinguishes drained from still-arriving, and
        it is bounded so a genuinely live stream fails the caller's assertion
        instead of hanging the suite.
        """
        deadline = time.monotonic() + timeout_s
        last, stable_since = -1, time.monotonic()
        while True:
            current = self.count()
            if current != last:
                last, stable_since = current, time.monotonic()
            elif time.monotonic() - stable_since >= quiet_s:
                return current
            if time.monotonic() >= deadline:
                return current
            time.sleep(0.01)

    def close(self):
        self._stop.set()
        with contextlib.suppress(OSError):
            self.listener.close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)


class TestEngineOwnership(unittest.TestCase):
    """
    Invariant: this tool drives an engine it started, or it refuses.

    The reproduced defect: with an engine already listening on the port, the tool
    connected to it, drove all eight thrusters to 1800 us, latched *that* engine's
    emergency brake for the life of the engine, and printed ``[PASS]`` with exit 0.
    The check's verdict was earned about the board it reached; the defect is the
    shape the whole branch exists to kill - a success reported without the
    precondition ("this is my own throwaway engine") ever being established,
    checked, or disclosed.

    Ownership is the same class of precondition as the output-status readback, so
    it is enforced the same way: before anything is written.  These tests drive the
    real ``SilSocketBackend`` and the real ``main()`` against a peer that records
    every frame, so "nothing was written" is observed rather than assumed.
    """

    def setUp(self):
        self.engine = _RecordingEngine()
        self.addCleanup(self.engine.close)

    def _cli(self, extra=(), backend_factory=None, action=("--emergency-break",)):
        argv = [
            "--mode",
            "sil",
            "--port",
            str(self.engine.port),
            *action,
            *extra,
        ]
        return run_cli(argv, backend_factory=backend_factory or build_real_backend)

    def test_connect_refuses_an_engine_this_backend_did_not_start(self):
        backend = SilSocketBackend("127.0.0.1", self.engine.port, auto_start=False)
        self.addCleanup(backend.close)
        with self.assertRaises(can_stimulus.ForeignEngineError) as caught:
            backend.connect(timeout_s=2.0)
        message = str(caught.exception)
        self.assertIn(str(self.engine.port), message, "the refusal must name the port it found")
        self.assertIn("--attach-existing", message, "the refusal must name the opt-in")
        self.assertFalse(backend.connected)

    def test_the_refusal_arrives_before_any_frame_is_written(self):
        """
        The load-bearing assertion: no 0x100 and no 0x001 on the wire.

        A refusal that arrives after the actuation is a refusal that is too late,
        which is why ``_require_output_readback`` runs before the first send. This
        is the same rule applied to ownership, so it is asserted the same way - on
        the bytes the foreign engine received, not on the tool's return value.
        """
        status_code, printed = self._cli()
        self.assertEqual(status_code, 2, printed)
        self.assertEqual(
            self.engine.ids(),
            [],
            "the tool wrote frames to an engine it does not own; the refusal must "
            "come before the first write, not after it",
        )

    def test_the_cli_reports_the_refusal_rather_than_a_check_verdict(self):
        status_code, printed = self._cli()
        self.assertEqual(status_code, 2, printed)
        self.assertIn("[FAIL] refused:", printed)
        self.assertIn(str(self.engine.port), printed)
        self.assertNotIn("[PASS]", printed)

    def test_the_explicit_opt_in_does_attach_and_is_the_only_way_to(self):
        """
        The opt-in path has to work, or the refusal is a dead end.

        Driven through the real ``build_backend`` so the flag is mapped the way it
        is for an operator. ``--raw-send`` is used because it is the one check
        that writes a frame the operator named and asserts nothing about the
        vehicle, so this stays a transport-attachment test rather than an
        actuation test.
        """
        status_code, printed = self._cli(
            ["--attach-existing", "--raw-send", f"0x{CAN_ID_THRUSTER_CMD:03X}:DC05DC05DC05DC05DC05DC05DC05DC05"],
            backend_factory=can_stimulus.build_backend,
            action=(),
        )
        self.assertEqual(status_code, 0, printed)
        # Bounded wait, not an immediate read.  The CLI returns once the write
        # has left the socket; _serve parses on its own thread, so ids() is a
        # lower bound that can lag by a frame.  Reading it directly here made the
        # test depend on which thread the scheduler ran.
        self.assertTrue(
            self.engine.wait_for_count(1),
            "the opt-in never let any frame reach the engine",
        )
        self.assertEqual(
            self.engine.ids(),
            [CAN_ID_THRUSTER_CMD],
            "the opt-in must let the named frame through, and only that frame",
        )

    def test_auto_is_refused_with_the_opt_in_because_it_is_a_whole_vehicle_run(self):
        """
        ``--auto`` drives every thruster, actuates a valve, and latches the brake.

        There is no such thing as a harmless ``--auto`` against somebody else's
        engine, so the two flags cannot be combined: the strict gate is the whole
        point of the opt-in.
        """
        status_code, printed = run_cli(
            ["--mode", "sil", "--port", str(self.engine.port), "--auto", "--attach-existing"],
            backend_factory=build_real_backend,
        )
        self.assertEqual(status_code, 2, printed)
        self.assertIn("--auto", printed)
        self.assertIn("--attach-existing", printed)
        # Settle before claiming nothing arrived: an unsettled read of an
        # asynchronous recorder proves nothing in the "nothing" direction, and
        # this is the assertion the whole ownership gate rests on.
        self.engine.settle()
        self.assertEqual(self.engine.ids(), [])

    def test_a_backend_that_owns_its_engine_is_not_refused(self):
        """
        The opt-in is not a blanket: a backend with a child of its own connects.

        The condition is ownership, not the presence of a listener, so the
        auto-start path must be unaffected. The peer is bound but not listening,
        so the first probe fails exactly as it does against an unused port, and
        the stand-in ``_start_server`` is what makes the engine reachable.
        """
        peer = _RecordingEngine(listening=False)
        self.addCleanup(peer.close)
        backend = SilSocketBackend("127.0.0.1", peer.port, auto_start=True)
        self.addCleanup(backend.close)

        class _Child:
            """A stand-in Popen: alive, and harmless to close."""

            pid = -1

            @staticmethod
            def poll():
                return None

            @staticmethod
            def terminate():
                pass

            @staticmethod
            def kill():
                pass

            @staticmethod
            def wait(timeout=None):
                return 0

        def fake_start(_self):
            peer.open()
            _self._process = _Child()  # ownership, without a real child

        with mock.patch.object(SilSocketBackend, "_start_server", fake_start):
            backend.connect(timeout_s=2.0)
        self.assertTrue(backend.connected)
        self.assertTrue(backend.started_server)

    def test_the_ephemeral_port_integration_backend_declares_its_own_attachment(self):
        """
        Every in-tree backend that talks to an engine it did not launch must say so.

        ``SilServerProcess`` starts the engine, so the backend behind it legitimately
        attaches. Pinning that here is what stops the refusal from being "fixed" by
        relaxing it: a new fixture has to make the same declaration.
        """
        source = Path(REPO_ROOT / "tests" / "sil_stimulus" / "test_stimulus_flows.py").read_text(
            encoding="utf-8"
        )
        # Only call sites: a string host argument means it is a construction, so the
        # ``class ObservingSilBackend(SilSocketBackend)`` definition is not matched.
        constructions = re.findall(r"ObservingSilBackend\(\"[^\"]*\",[^\n]*\)", source)
        self.assertTrue(
            constructions,
            "no ObservingSilBackend construction was found, so this test would "
            "silently pass while asserting nothing",
        )
        for construction in constructions:
            with self.subTest(construction=construction):
                self.assertIn(
                    "attach_existing=True",
                    construction,
                    "a backend that attaches to an externally started engine must say so",
                )


def build_real_backend(args):
    """
    The real SIL transport, for the tests that need a real socket and no engine.

    ``auto_start=False`` guarantees nothing is launched, so the only thing these
    tests can ever reach is whatever is already listening on the port.
    """
    return SilSocketBackend(args.host, args.port, auto_start=False)


class TestCanSnifferAgreesWithSilProtocol(unittest.TestCase):
    """
    Invariant: the third copy of the CAN id table cannot drift silently.

    ``tools/can_sniffer.py`` keeps its own ``CAN_ID_*`` constants and its own
    ``struct.Struct`` formats. That duplication is where both broken prototypes
    came from, and it is invisible precisely because the values agree today - so
    the drift this test exists to catch would be silent, too.

    It parses the source rather than importing it: ``can_sniffer`` does
    ``import can`` and ``sys.exit(1)`` at module scope, so importing it would make
    this test a python-can test. ``ast`` reads the literals that matter without
    executing the module or needing the dependency.
    """

    SNIFFER = REPO_ROOT / "tools" / "can_sniffer.py"
    # can_sniffer names this one _STRUCT_ENV; sil_protocol names it _STRUCT_3FB.
    # The formats differ as text ("<fffB" vs "<3fB") and are the same 13-byte
    # layout, which is exactly the kind of difference a text comparison would
    # either reject or paper over.
    STRUCT_NAMES = {
        "_STRUCT_TIME_SYNC_MASTER": "_STRUCT_TIME_SYNC_MASTER",
        "_STRUCT_TIME_SYNC_REQ": "_STRUCT_TIME_SYNC_REQ",
        "_STRUCT_TIME_SYNC_RESP": "_STRUCT_TIME_SYNC_RESP",
        "_STRUCT_8H": "_STRUCT_8H",
        "_STRUCT_H": "_STRUCT_H",
        "_STRUCT_NAV_TS": "_STRUCT_NAV_TS",
        "_STRUCT_NAV_LEGACY": "_STRUCT_NAV_LEGACY",
        "_STRUCT_ENV": "_STRUCT_3FB",
        "_STRUCT_POWER": "_STRUCT_POWER",
    }

    @staticmethod
    def _module_constants(path):
        """Every module-level literal assignment in ``path``, as {name: value}."""
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        constants = {}
        for node in tree.body:
            if not isinstance(node, ast.Assign) or len(node.targets) != 1:
                continue
            target = node.targets[0]
            if not isinstance(target, ast.Name):
                continue
            try:
                constants[target.id] = ast.literal_eval(node.value)
            except ValueError:
                # struct.Struct("...") and friends: handled separately.
                continue
        return constants

    @classmethod
    def _struct_formats(cls, path):
        """{name: format string} for every module-level ``struct.Struct(...)``."""
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        formats = {}
        for node in tree.body:
            if not isinstance(node, ast.Assign) or len(node.targets) != 1:
                continue
            target = node.targets[0]
            call = node.value
            if not isinstance(target, ast.Name) or not isinstance(call, ast.Call):
                continue
            if not isinstance(call.func, ast.Attribute) or call.func.attr != "Struct":
                continue
            if len(call.args) == 1 and isinstance(call.args[0], ast.Constant):
                formats[target.id] = call.args[0].value
        return formats

    @staticmethod
    def _layout(format_string):
        """
        (size, one code per field), so '<3fB' and '<fffB' compare equal.

        Comparing format TEXT would report a difference where there is none, and
        comparing only the size would miss a signedness flip (h vs H) that packs
        identically. This is the layout, exactly.
        """
        codes = []
        index = 0
        while index < len(format_string):
            char = format_string[index]
            if char.isdigit():
                end = index
                while format_string[end].isdigit():
                    end += 1
                codes.extend([format_string[end]] * int(format_string[index:end]))
                index = end + 1
            elif char in "<>=!@":
                index += 1
            else:
                codes.append(char)
                index += 1
        return (struct.calcsize(format_string), tuple(codes))

    def test_every_can_id_agrees_with_sil_protocol(self):
        sniffer = self._module_constants(self.SNIFFER)
        protocol = self._module_constants(REPO_ROOT / "tests" / "sil_companion_bridge" / "sil_protocol.py")
        declared = {
            name: value for name, value in sniffer.items() if name.startswith("CAN_ID_")
        }
        self.assertTrue(declared, "the sniffer declares no CAN ids, so this asserts nothing")
        for name, value in sorted(declared.items()):
            with self.subTest(constant=name):
                self.assertIn(
                    name, protocol, f"{self.SNIFFER.name} declares {name}, which sil_protocol does not"
                )
                self.assertEqual(
                    value,
                    protocol[name],
                    f"{self.SNIFFER.name}:{name} disagrees with sil_protocol; one of them "
                    "is decoding the wrong frames",
                )

    def test_every_struct_layout_agrees_with_sil_protocol(self):
        sniffer = self._struct_formats(self.SNIFFER)
        protocol = self._struct_formats(
            REPO_ROOT / "tests" / "sil_companion_bridge" / "sil_protocol.py"
        )
        self.assertTrue(sniffer, "no struct formats were parsed, so this asserts nothing")
        for name, canonical in sorted(self.STRUCT_NAMES.items()):
            with self.subTest(struct=name):
                self.assertIn(name, sniffer, f"{self.SNIFFER.name} no longer declares {name}")
                self.assertIn(
                    canonical,
                    protocol,
                    f"sil_protocol no longer declares {canonical}, which {name} mirrors",
                )
                self.assertEqual(
                    self._layout(sniffer[name]),
                    self._layout(protocol[canonical]),
                    f"{name} ({sniffer[name]}) and sil_protocol's {canonical} "
                    f"({protocol[canonical]}) are not the same layout",
                )

    def test_a_shared_struct_is_not_silently_renamed_out_of_the_mapping(self):
        """
        The mapping is the part that can rot, so every sniffer struct must be in it.

        A new struct in the sniffer that nobody mapped would otherwise be compared
        against nothing - the same silent-drift hole the two mappings close.
        """
        sniffer = self._struct_formats(self.SNIFFER)
        unmapped = sorted(set(sniffer) - set(self.STRUCT_NAMES))
        self.assertEqual(
            unmapped,
            [],
            f"{self.SNIFFER.name} declares struct(s) this test does not compare with "
            f"sil_protocol: {unmapped}. Map each one or delete the copy.",
        )


class TestStimulusNonSilTransport(unittest.TestCase):
    """
    A transport that cannot supply the 0x7FE readback must not be actuated.

    The reproduced defect: ``run_full_smoke()`` through a CAN-shaped transport
    emitted 285 frames including 0x100 thruster commands and a 0x110 valve
    actuation, and only reported failure afterwards.  Refusing to fabricate a
    hardware verdict was right; stopping only at the refusal was not.
    """

    def test_full_smoke_over_a_non_sil_backend_emits_no_frame_at_all(self):
        backend = NonSilBackend()
        report = VehicleStimulusTester(backend).run_full_smoke()
        self.assertFalse(report.ok)
        self.assertEqual(len(report.failures), len(report.results))
        self.assertEqual(
            backend.sent,
            [],
            "no frame may reach a transport that cannot verify its effect",
        )

    def test_every_actuating_check_refuses_and_names_the_reason(self):
        for name, call in (
            ("node2_arming", lambda t: t.test_node2_arming()),
            ("node2_pwm", lambda t: t.test_node2_pwm(0, 1700, timeout_s=0.01)),
            ("node2_all", lambda t: t.test_node2_all(1700, timeout_s=0.01)),
            ("node2_solenoid", lambda t: t.test_node2_solenoid(0x0001, timeout_s=0.01)),
            ("emergency_break", lambda t: t.test_emergency_break(timeout_s=0.01)),
        ):
            with self.subTest(check=name):
                backend = NonSilBackend()
                result = call(VehicleStimulusTester(backend))
                self.assertFalse(result.passed)
                self.assertIn("refused before sending any frame", result.detail)
                self.assertIn("SIL-only", result.detail)
                self.assertEqual(backend.sent, [], f"{name} must not write on a non-SIL transport")

    def test_the_emergency_frame_is_not_sent_by_a_refused_check(self):
        """
        Invariant: the refusal happens before the 0x001 write, not after.

        An E-stop frame is the most consequential thing this tool can emit, so
        the ordering is asserted directly rather than inferred.
        """
        backend = NonSilBackend()
        VehicleStimulusTester(backend).test_emergency_break(timeout_s=0.01)
        self.assertNotIn(
            CAN_ID_EMERGENCY_BREAK,
            [can_id for can_id, _ in backend.sent],
            "the authorized emergency frame must never leave a non-SIL transport",
        )

    def test_receive_only_checks_still_run_on_a_non_sil_backend(self):
        """
        Invariant: observing a real bus is the supported use of --mode can/uart.

        ``sniff`` needs no readback, so it must work on a hardware transport - and
        running it exercises ``_pump``'s ``recv_frame`` fallback, the branch only
        the non-SIL adapters reach.
        """
        backend = NonSilBackend(frames=[(CAN_ID_ENV_TELEMETRY, env_payload())])
        result = VehicleStimulusTester(backend).sniff(duration_s=0.05)
        self.assertTrue(result.passed, result.detail)
        self.assertIn(f"0x{CAN_ID_ENV_TELEMETRY:03X}", result.detail)
        self.assertEqual(backend.sent, [], "observing must not write")

    def test_pump_falls_back_to_recv_frame_when_the_transport_cannot_drain(self):
        """Covers the ``else`` branch in ``_pump`` that only hardware adapters hit."""
        backend = NonSilBackend(frames=[(CAN_ID_ENV_TELEMETRY, env_payload())])
        self.assertFalse(hasattr(backend, "drain_frames"), "this double must mirror the adapters")
        received = []
        tester = VehicleStimulusTester(backend)
        tester._pump(
            time.monotonic() + 0.05,
            on_frame=lambda can_id, payload: received.append(can_id),
        )
        self.assertIn(CAN_ID_ENV_TELEMETRY, received)

    def test_auto_is_refused_in_non_sil_mode(self):
        for mode in ("can", "uart"):
            with self.subTest(mode=mode):
                status_code, printed = run_cli(["--mode", mode, "--auto"])
                self.assertEqual(status_code, 2)
                self.assertIn("blind actuation", printed)
                self.assertIn(mode, printed)

    def test_raw_send_is_the_one_documented_exception(self):
        """
        Invariant: an operator-supplied frame still goes out, described truthfully.

        The operator typed the exact id and bytes, and the tool is a pipe for
        them; refusing would second-guess them.  What must not happen is claiming
        the frame was verified, or calling an unaddressed UART write "CAN 0xNNN".
        """
        backend = NonSilBackend()
        tester = VehicleStimulusTester(backend)
        signed = tester.raw_send(CAN_ID_EMERGENCY_BREAK, can_stimulus.EMERGENCY_FRAME)
        self.assertTrue(signed.passed)
        self.assertIn("not any vehicle response", signed.detail)
        self.assertEqual(backend.sent, [(CAN_ID_EMERGENCY_BREAK, can_stimulus.EMERGENCY_FRAME)])

    def test_uart_destination_text_does_not_claim_an_arbitration_id(self):
        """
        Invariant: the pass text matches what the transport actually did.

        ``UartBackend.send_frame`` discards the CAN id and writes bare bytes, so
        "wrote ... to CAN 0x001" would be a false statement about three
        unaddressed bytes.
        """
        backend = can_stimulus.UartBackend(port="/dev/ttyFAKE", baudrate=9600)
        description = backend.describe_destination(CAN_ID_EMERGENCY_BREAK)
        self.assertIn("/dev/ttyFAKE", description)
        self.assertIn("9600 baud", description)
        self.assertIn("carries no arbitration id", description)
        self.assertIn("was NOT addressed", description)

    def test_can_destination_text_names_the_interface(self):
        backend = can_stimulus.PythonCanBackend(interface="vcan0")
        self.assertEqual(
            backend.describe_destination(CAN_ID_THRUSTER_CMD), "CAN 0x100 on vcan0"
        )

    def test_raw_send_rejects_an_impossible_can_id(self):
        """
        Invariant: the id is validated before the check names itself.

        The name is built from a validated id, so ``--raw-send -1:FF`` can never
        report itself as ``raw_send_0x-1`` on the way to raising. The old code
        formatted the name first, which put a nonsense report name in the text and
        still raised - cosmetic, but it is the tool's own report.
        """
        backend = FakeBackend()
        tester = VehicleStimulusTester(backend)
        for bad in (-1, 0x20000000, "0x100"):
            with self.subTest(can_id=bad):
                with self.assertRaises((ValueError, TypeError)):
                    tester.raw_send(bad, b"\x01")
        self.assertEqual(backend.sent, [])
        self.assertNotIn(
            "raw_send_0x-1",
            inspect.getsource(tester.raw_send),
            "the name must be derived after validation, not before it",
        )

    def test_a_transport_that_forgets_to_declare_the_capability_fails_safe(self):
        class UndeclaredTransport(NonSilBackend):
            provides_output_readback = None

        backend = UndeclaredTransport()
        result = VehicleStimulusTester(backend).test_node2_arming()
        self.assertFalse(result.passed)
        self.assertEqual(backend.sent, [])


class ScriptedBoard(_ScriptedTransport):
    """
    A board that reacts to the frames it receives, like the real firmware does.

    Index-guessing a read stream is brittle: how many reads a check performs
    depends on the path it takes, so a script written as "hot on read 5" lands in
    a different phase after any change. This double instead keeps state and
    updates it from ``send_frame``, mirroring the four firmware rules the checks
    depend on:

    * ``app.c:127-129`` - thruster commands are accepted only while ACTIVE and
      never while the emergency break is active;
    * ``app.c:112`` - 0x001 acts only on the 0xAA 0x55 signature;
    * ``app.c:116-118`` - an authorized trip latches, disarms, and forces neutral;
    * ``app.c:192-193`` - everything stays neutral while arming.

    A phase script then just says what the board is *capable* of, and the test
    states what the check should conclude.
    """

    def __init__(self, armed=True, trips_on_authorized=True, resumes_after_estop=False):
        super().__init__()
        self.armed = armed          # may the board accept thrust at all?
        self.trips_on_authorized = trips_on_authorized
        self.resumes_after_estop = resumes_after_estop
        self.brake = False
        self.pwms = [NEUTRAL_US] * 8
        self.sim_ms = 0
        self.saw_authorized_trip = False

    def send_frame(self, can_id, payload):
        self.sent.append((can_id, bytes(payload)))
        if can_id == CAN_ID_EMERGENCY_BREAK:
            if bytes(payload).startswith(can_stimulus.EMERGENCY_SIGNATURE):
                self.saw_authorized_trip = True
                if self.trips_on_authorized:
                    # app.c:113-118: trip, disarm, force neutral. Latched: nothing
                    # below ever clears it.
                    self.brake = True
                    self.armed = False
                    self.pwms = [NEUTRAL_US] * 8
        elif can_id == CAN_ID_THRUSTER_CMD:
            if self.brake:
                # The only window in which a post-ESTOP command could take effect.
                # Resuming from here is the specific defect under test: the latch
                # is not holding, so the command the check sends to *prove* the
                # latch moves the thrusters.
                if self.resumes_after_estop and self.saw_authorized_trip:
                    self.pwms = [POST_ESTOP_PULSE_US] * 8
            elif self.armed:
                # app.c:135-145: accepted, clamped into the envelope.
                self.pwms = [
                    max(
                        PWM_MIN_US,
                        min(PWM_MAX_US, int.from_bytes(payload[i * 2 : i * 2 + 2], "little")),
                    )
                    for i in range(8)
                ]
        elif can_id == CAN_ID_SOLENOID_CMD and not self.brake:
            self.solenoids = int.from_bytes(payload[:2], "little")

    def _next_status(self):
        self.sim_ms += 10
        return OutputStatus(
            tuple(self.pwms), self.brake, getattr(self, "solenoids", 0), self.sim_ms
        )

    def latest_outputs(self):
        return self._latest


class ScriptedArmingBoard(_ScriptedTransport):
    """
    A board whose ESC gate behaviour and virtual clock are under test control.

    The arming check judges two things against the *simulated* clock: that the
    outputs stay neutral for ``ESC_ARMING_SIM_MS`` and that the command takes
    effect afterwards. So the double advances ``sim_step_ms`` per read (1000 ms
    by default, so a few reads cross the 3000 ms boundary) and applies a
    ``gate_opens_early`` / ``stuck_after_arming`` switch, mirroring
    ``app.c:99-102`` and ``app.c:127-129``.
    """

    def __init__(self, gate_opens_early=False, stuck_after_arming=False, sim_step_ms=500):
        super().__init__()
        self.gate_opens_early = gate_opens_early
        self.stuck_after_arming = stuck_after_arming
        self.sim_step_ms = sim_step_ms
        self.sim_ms = 0
        self.armed = False
        self.armed_at_ms = None
        self.pwms = [NEUTRAL_US] * 8

    def _gate_should_be_open(self):
        # app.c:99-102: the gate closes for ESC_ARMING_TIME_MS of simulated time.
        return self.gate_opens_early or self.sim_ms >= ESC_ARMING_SIM_MS

    def send_frame(self, can_id, payload):
        self.sent.append((can_id, bytes(payload)))
        if can_id != CAN_ID_THRUSTER_CMD:
            return
        pwms = [
            max(PWM_MIN_US, min(PWM_MAX_US, int.from_bytes(payload[i * 2 : i * 2 + 2], "little")))
            for i in range(8)
        ]
        if self._gate_should_be_open() and not self.stuck_after_arming:
            # app.c:135-145: accepted and applied (no ramp in this double; the
            # slew rate is not what these branches are about).
            self.pwms = pwms
            self.armed = True
            if self.armed_at_ms is None:
                self.armed_at_ms = self.sim_ms

    def _next_status(self):
        self.sim_ms += self.sim_step_ms
        return OutputStatus(tuple(self.pwms), False, 0, self.sim_ms)

    def latest_outputs(self):
        return self._latest


class TestStimulusEmergencyPrecondition(unittest.TestCase):
    """
    The post-ESTOP latch claim is only earned if the board was ACTIVE first.

    Reproduced defect: run against a fresh engine, the check latched the brake at
    sim 390 ms - 2.6 s before the 3000 ms arming window closes - and reported
    "[PASS] ... a 1800 us command afterwards did not resume thrust" with exit 0.
    ``app.c:127-129`` refuses thruster commands whenever the ESC state is not
    ACTIVE and ``app.c:192-193`` holds neutral throughout arming, so on a
    disarmed board that observation is guaranteed and means nothing.
    """

    # Both observation windows are shrunk as well as the sim rate: a pump
    # iteration consumes one scripted snapshot, so a 0.4 s window would eat 20
    # of them and leave nothing for the later phases.
    FAST = dict(
        timeout_s=0.05,
        sim_rate=1e6,
        slack_s=0.05,
        unauthorized_window_s=0.02,
        estop_window_s=0.02,
        first_output_timeout_s=0.05,
    )
    NEUTRAL = [NEUTRAL_US] * 8
    HOT = [1800] * 8

    def _tester(self, board):
        return VehicleStimulusTester(board)

    def test_emergency_check_fails_when_the_board_never_accepted_thrust(self):
        """
        The exact reproduced failure, as a unit test.

        A board still inside its 3000 ms arming window never accepts thrust, so
        the post-ESTOP claim is unobservable. The check must FAIL and say so,
        rather than dropping the claim and passing.
        """
        result = self._tester(ScriptedBoard(armed=False)).test_emergency_break(**self.FAST)
        self.assertFalse(result.passed, "a never-armed board must not report a proven latch")
        self.assertIn("UNOBSERVABLE", result.detail)
        self.assertIn("never reached ESC_STATE_ACTIVE", result.detail)
        self.assertIn("FAILED rather than as a partial pass", result.detail)

    def test_emergency_check_passes_on_a_properly_armed_board(self):
        """The green counterpart: an ACTIVE board that accepted thrust first."""
        board = ScriptedBoard(armed=True)
        result = self._tester(board).test_emergency_break(**self.FAST)
        self.assertTrue(result.passed, result.detail)
        self.assertIn("The latch claim is earned", result.detail)
        self.assertIn("ESC ACTIVE, accepting thrust", result.detail)
        self.assertTrue(board.saw_authorized_trip, "the authorized frame must have been sent")

    def test_emergency_check_fails_when_a_post_estop_command_resumes_thrust(self):
        """The armed board latches, but a later command still moves: the latch is not holding."""
        board = ScriptedBoard(armed=True, resumes_after_estop=True)
        result = self._tester(board).test_emergency_break(**self.FAST)
        self.assertFalse(result.passed)
        self.assertIn("resumed thrust", result.detail)
        self.assertIn("latch is not holding", result.detail)

    def test_emergency_check_fails_when_an_unauthorized_frame_trips_the_brake(self):
        """
        The negative control is a real discriminator, and it is checked.

        This board ignores the signature and trips on anything, which is the one
        behaviour that would make the authorized trip meaningless.
        """

        class SloppyBoard(ScriptedBoard):
            def send_frame(self, can_id, payload):
                super().send_frame(can_id, payload)
                if can_id == CAN_ID_EMERGENCY_BREAK:
                    self.brake = True
                    self.armed = False
                    self.pwms = [NEUTRAL_US] * 8

        result = self._tester(SloppyBoard(armed=True)).test_emergency_break(**self.FAST)
        self.assertFalse(result.passed)
        self.assertIn("unauthorized 0xAA 0x56 frame tripped the brake", result.detail)

    def test_emergency_check_fails_when_the_authorized_frame_does_not_latch(self):
        """
        The board arms and accepts thrust, then ignores the authorized 0x001.

        It stays neutral and untripped for the rest of the run, so the latch wait
        expires with the board visibly healthy - exactly the case where a check
        must not report success.
        """
        board = ScriptedBoard(armed=True, trips_on_authorized=False)
        result = self._tester(board).test_emergency_break(**self.FAST)
        self.assertFalse(result.passed)
        self.assertIn("did not latch the", result.detail)

    def test_emergency_check_fails_when_no_readback_arrives(self):
        result = self._tester(FakeBackend()).test_emergency_break(timeout_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("0x7FE", result.detail)

    def test_emergency_check_fails_when_the_board_is_already_braked(self):
        """
        A board whose brake was tripped before this run cannot be armed, and the
        check must attribute nothing to its own frame.
        """
        board = ScriptedBoard(armed=True)
        board.brake = True
        result = self._tester(board).test_emergency_break(**self.FAST)
        self.assertFalse(result.passed)
        self.assertIn("already tripped", result.detail)
        self.assertIn("Restart the engine", result.detail)


class TestOperatorIntegerLiterals(unittest.TestCase):
    """
    Invariant: a mask or a CAN id is spelled the way the operator already spells it.

    The file used to carry both wrong forms of the same idea. ``--node2-solenoid``
    used ``type=int``, so ``0x0001`` was a usage error and ``010`` silently meant
    ten. ``--raw-send``'s id used ``int(id_text, 0)``, so ``0x001`` worked and
    ``010`` raised - Python rejects a leading zero at base 0. The obvious
    unification, ``type=lambda x: int(x, 0)``, would have fixed the hex case and
    newly broken the zero-padded decimal, trading one break for another.

    So the rule is explicit and small: an explicit ``0x``/``0o``/``0b`` prefix is
    honoured, and everything else is plain base-10 decimal.
    """

    # 010 must be ten: that is what the deleted Node 2 prototype's int(x, 0)
    # caller believed it was doing for plain decimal input, and what an operator
    # writing a four-digit mask in a script means.
    SPELLINGS = (
        ("0x0001", 1),
        ("0X001", 1),
        ("0o10", 8),
        ("0b1", 1),
        ("010", 10),
        ("1", 1),
        ("1023", 1023),
        ("0x3FF", 1023),
    )

    def test_the_solenoid_mask_flag_accepts_every_common_spelling(self):
        parser = can_stimulus.build_parser()
        for text, expected in self.SPELLINGS:
            with self.subTest(text=text):
                self.assertEqual(parser.parse_args(["--node2-solenoid", text]).node2_solenoid, expected)

    def test_a_raw_send_can_id_accepts_every_common_spelling(self):
        parser = can_stimulus.build_parser()
        for text, expected in self.SPELLINGS:
            with self.subTest(text=text):
                args = parser.parse_args(["--raw-send", f"{text}:AA5501"])
                self.assertEqual(args.raw_send, [(expected, b"\xaa\x55\x01")])

    def test_a_zero_padded_decimal_can_id_reaches_the_wire_as_ten(self):
        """
        End to end, because the point is what the operator gets: no usage error.

        0x001 is the emergency break, and ``AA 55 01`` is its authorized
        signature, so the write is accepted and the check passes - the frame
        itself is the evidence that the literal was parsed as 10, not refused.
        """
        backend = FakeBackend()
        status_code, printed = run_cli(
            ["--mode", "sil", "--raw-send", "010:AA5501"], backend_factory=lambda args: backend
        )
        self.assertEqual(status_code, 0, printed)
        self.assertEqual([can_id for can_id, _ in backend.sent], [10])

    def test_an_unparseable_value_is_still_a_usage_error(self):
        parser = can_stimulus.build_parser()
        for text in ("0x", "twelve", "", "1.5", "0xZZ"):
            with self.subTest(text=text), self.assertRaises(SystemExit) as caught:
                parser.parse_args(["--node2-solenoid", text])
            self.assertEqual(caught.exception.code, 2)

    def test_the_two_flags_share_one_parser_rather_than_two_forms(self):
        """
        A second spelling is a second set of rules, which is how the file ended up
        with both wrong forms in it.
        """
        parser = can_stimulus.build_parser()
        solenoid = next(
            action
            for action in parser._actions
            if "--node2-solenoid" in action.option_strings
        )
        self.assertIs(
            solenoid.type,
            can_stimulus.int_literal,
            "--node2-solenoid must use the shared prefix-aware parser",
        )
        self.assertNotEqual(solenoid.type, int)
        self.assertNotEqual(solenoid.type, lambda text: int(text, 0))


class TestStimulusSolenoidBehaviour(unittest.TestCase):
    """A mask that cannot be evidenced must not be reported as actuated."""

    def test_solenoid_mask_zero_is_refused_at_the_argument_boundary(self):
        """
        Invariant: 0x0000 cannot evidence a command.

        Reproduced defect: mask 0 passed on the first snapshot of a board that
        never received the frame, because both the actuate and the release
        predicates were already true, and the pass text claimed the mask
        "energises one coil per valve".
        """
        backend = FakeBackend()
        with self.assertRaises(ValueError) as caught:
            VehicleStimulusTester(backend).test_node2_solenoid(0, timeout_s=0.05)
        message = str(caught.exception)
        self.assertIn("released state", message)
        self.assertIn("cannot be evidenced", message)
        self.assertEqual(backend.sent, [])

    def test_solenoid_check_fails_on_a_board_that_ignores_the_frame(self):
        """
        The masked form of the same hole: a valid mask the board never applies.

        The readback stays at 0x0000, so the actuate predicate never matches and
        the check must fail rather than pass on the release predicate.
        """
        statuses = [status(solenoids=0, sim_time_ms=i * 10) for i in range(40)]
        result = VehicleStimulusTester(FakeBackend(statuses=statuses)).test_node2_solenoid(
            0x0001, timeout_s=0.05
        )
        self.assertFalse(result.passed)
        self.assertIn("never reported it back", result.detail)
        self.assertIn("0x0000", result.detail)

    def test_solenoid_check_fails_when_the_readback_never_changes(self):
        """
        Invariant: the board must be observed to *change* state.

        Commanding a mask the board already holds produces an identical readback
        before and after, which is indistinguishable from an ignored frame.
        """
        statuses = [status(solenoids=0x0001, sim_time_ms=i * 10) for i in range(40)]
        result = VehicleStimulusTester(FakeBackend(statuses=statuses)).test_node2_solenoid(
            0x0001, timeout_s=0.05
        )
        self.assertFalse(result.passed)
        self.assertIn("already reports solenoid mask 0x0001", result.detail)
        self.assertIn("cannot be distinguished from one that was ignored", result.detail)

    def test_solenoid_check_requires_a_fresh_readback_before_acting(self):
        """With no readback at all there is no pre-command state to compare against."""
        result = VehicleStimulusTester(FakeBackend()).test_node2_solenoid(0x0001, timeout_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("0x7FE", result.detail)

    def test_solenoid_pass_text_names_the_coils_it_energised(self):
        """
        Invariant: the pass text is true for whatever mask was accepted.

        The previous text asserted "energises one coil per valve" for every mask,
        which is not a description of anything the tool actually did.
        """
        statuses = [
            status(solenoids=0x0000, sim_time_ms=0),
            status(solenoids=0x0002, sim_time_ms=10),
            status(solenoids=0, sim_time_ms=20),
            status(solenoids=0, sim_time_ms=30),
        ]
        result = VehicleStimulusTester(FakeBackend(statuses=statuses)).test_node2_solenoid(
            0x0002, timeout_s=0.05
        )
        self.assertTrue(result.passed, result.detail)
        self.assertIn("energised valve 0 coil B", result.detail)
        self.assertIn("a change from the pre-command 0x0000", result.detail)

    def test_energised_coils_names_every_coil(self):
        self.assertEqual(can_stimulus.energised_coils(0x0001), ["valve 0 coil A"])
        self.assertEqual(can_stimulus.energised_coils(0x0004), ["valve 1 coil A"])
        self.assertEqual(
            can_stimulus.energised_coils(0x0155),
            [f"valve {v} coil A" for v in range(5)],
        )
        self.assertEqual(can_stimulus.energised_coils(0x0000), [])


class _PairedTelemetryStream:
    """
    One telemetry frame and one fresh 0x7FE per read window, stepping together.

    The stock ``FakeBackend`` hands over every queued frame in a single window, so
    a burst of telemetry always lands on one simulated timestamp. Anything that
    measures a *rate* has to see the frames spread across the engine's clock the
    way a real engine spreads them, one tick at a time, so this double does that.
    """

    provides_output_readback = True

    def __init__(self, frames, step_ms=10, readback=True):
        self.frames = list(frames)
        self.step_ms = step_ms
        self.readback = readback
        if not readback:
            # A transport that supplies no 0x7FE at all, e.g. a real CAN adapter.
            self.provides_output_readback = False
        self.sent = []
        self.served = 0
        self._latest = None
        self._listener = None

    def connect(self, timeout_s=None):
        pass

    def close(self):
        pass

    def set_frame_listener(self, listener):
        previous, self._listener = self._listener, listener
        return previous

    def send_frame(self, can_id, payload):
        self.sent.append((can_id, bytes(payload)))

    def latest_outputs(self):
        return self._latest

    def describe_destination(self, can_id):
        return f"CAN 0x{can_id:03X}"

    def drain_frames(self, timeout_s=0.0):
        if not self.frames:
            return []
        can_id, payload = self.frames.pop(0)
        if self.readback:
            self._latest = status(sim_time_ms=self.served * self.step_ms)
            self.served += 1
        if self.listener is not None:
            self.listener(can_id, payload)
        return [(can_id, payload)]

    @property
    def listener(self):
        return self._listener


class TestSimulatedRateClaims(unittest.TestCase):
    """
    Invariant: a pass message states a rate the tool MEASURED.

    ``monitor_node2_depth`` used to divide the observed wall rate by
    ``SIM_MS_PER_WALL_S`` (0.8) and print the quotient as a simulated rate. 0.8 is
    a deliberately generous LOWER BOUND on the engine's tick rate, not a
    measurement of it, so the number was an inference wearing a unit: on a host
    running the engine at 1.0 it under-reports, and the ">= 100 Hz simulated,
    against the 100 Hz design rate" line the operator files is a claim about a
    quantity nobody looked at. The engine's own clock is on every 0x7FE snapshot,
    so the host-independent rate is the one to report.
    """

    # 21 frames one 10 ms tick apart is a 200 ms span: 21 / 0.2 = 105 Hz, exactly,
    # and comfortably above the 20 Hz floor. The old inference would have printed
    # 42 / 0.8 = 52 Hz for the same stream, which is why this is a real assertion
    # and not a search for a substring that happens to be there.
    FRAMES = 21
    STEP_MS = 10
    EXPECTED_SIM_HZ = FRAMES / ((FRAMES - 1) * STEP_MS / 1000.0)

    def _stream(self, frames=FRAMES, with_readback=True):
        return _PairedTelemetryStream(
            [(CAN_ID_NAV_TELEMETRY, nav_payload()) for _ in range(frames)],
            readback=with_readback,
        )

    def test_the_pass_text_reports_the_simulated_rate_it_measured(self):
        result = VehicleStimulusTester(self._stream()).monitor_node2_depth(duration_s=0.5)
        self.assertTrue(result.passed, result.detail)
        self.assertIn(
            f"{self.EXPECTED_SIM_HZ:.0f} Hz simulated",
            result.detail,
            "the simulated rate must be the one the engine's clock shows, not the "
            f"wall rate divided by a lower bound: {result.detail!r}",
        )
        self.assertIn("Hz wall", result.detail, "the wall figure must stay, labelled as wall")

    def test_a_stream_with_no_sim_clock_says_so_instead_of_inventing_a_rate(self):
        """
        A transport with no 0x7FE cannot evidence a simulated rate at all.

        It must still pass or fail on what it can see, and must say the simulated
        rate was not measured rather than deriving one from a constant.
        """
        result = VehicleStimulusTester(self._stream(with_readback=False)).monitor_node2_depth(
            duration_s=0.05
        )
        self.assertIn("Hz wall", result.detail)
        self.assertNotIn("Hz simulated", result.detail)
        self.assertIn("unmeasured", result.detail.lower())


class TestStimulusVehicleCheckFailures(unittest.TestCase):
    """
    Failure-path coverage for the six vehicle checks.

    Every test here injects a scripted stream and asserts a FAILED ``CheckResult``
    with the specific evidence, so emptying any check body to
    ``CheckResult(name, True, "")`` cannot leave the suite green.
    """

    # test_node2_arming takes no timeout_s; it needs the sim-rate injection and a
    # bounded first-output wait so a scripted stream is not drained in 3 s.
    ARMING_FAST = dict(sim_rate=1e6, slack_s=0.05, first_output_timeout_s=0.05)
    FAST = dict(timeout_s=0.05, sim_rate=1e6, slack_s=0.05)
    NEUTRAL = [NEUTRAL_US] * 8

    def _telemetry_tester(self, can_id, payload, repeats=3, statuses=()):
        frames = [(can_id, payload)] * repeats
        return VehicleStimulusTester(FakeBackend(statuses=list(statuses), frames=frames))

    # -- node1_env --------------------------------------------------------
    def test_node1_env_fails_when_nothing_arrives(self):
        result = VehicleStimulusTester(FakeBackend()).monitor_node1_env(duration_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("no 0x210 environment telemetry", result.detail)

    def test_node1_env_fails_on_a_single_frame(self):
        result = self._telemetry_tester(CAN_ID_ENV_TELEMETRY, env_payload(), repeats=1).monitor_node1_env(
            duration_s=0.05
        )
        self.assertFalse(result.passed)
        self.assertIn("at least 2 are required", result.detail)

    def test_node1_env_fails_on_an_undecodable_frame(self):
        result = self._telemetry_tester(CAN_ID_ENV_TELEMETRY, b"\x01\x02\x03\x04").monitor_node1_env(
            duration_s=0.05
        )
        self.assertFalse(result.passed)
        self.assertIn("could not be decoded as EnvTelemetry", result.detail)

    def test_node1_env_fails_on_an_implausible_pressure(self):
        payload = env_payload(pressure_hpa=2000.0)
        result = self._telemetry_tester(CAN_ID_ENV_TELEMETRY, payload).monitor_node1_env(duration_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("outside the", result.detail)

    def test_node1_env_fails_on_a_non_finite_reading(self):
        payload = env_payload(humidity_pct=float("nan"))
        result = self._telemetry_tester(CAN_ID_ENV_TELEMETRY, payload).monitor_node1_env(duration_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("non-finite humidity_pct", result.detail)

    def test_node1_env_fails_when_a_leak_is_flagged(self):
        payload = env_payload(leak_flags=0x06)
        result = self._telemetry_tester(CAN_ID_ENV_TELEMETRY, payload).monitor_node1_env(duration_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("leak_flags=0x06", result.detail)
        self.assertIn("must be dry", result.detail)

    # -- node3_power ------------------------------------------------------
    def test_node3_power_fails_when_nothing_arrives(self):
        result = VehicleStimulusTester(FakeBackend()).monitor_node3_power(duration_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("no 0x300 power telemetry", result.detail)

    def test_node3_power_fails_on_a_single_frame(self):
        result = self._telemetry_tester(
            CAN_ID_POWER_TELEMETRY, power_payload(), repeats=1
        ).monitor_node3_power(duration_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("at least 2 are required", result.detail)

    def test_node3_power_fails_on_an_undecodable_frame(self):
        result = self._telemetry_tester(
            CAN_ID_POWER_TELEMETRY, b"\x01\x02\x03"
        ).monitor_node3_power(duration_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("could not be decoded as PowerTelemetry", result.detail)

    def test_node3_power_fails_on_the_fault_bit(self):
        """
        The fault bit is the thing that matters on 0x300: node 3 latched a fault
        and broadcast 0x005, which trips the board brake.
        """
        payload = power_payload(status_flags=0x0001)
        result = self._telemetry_tester(CAN_ID_POWER_TELEMETRY, payload).monitor_node3_power(
            duration_s=0.05
        )
        self.assertFalse(result.passed)
        self.assertIn("fault bit set", result.detail)
        self.assertIn("0x005", result.detail)

    def test_node3_power_fails_on_an_out_of_envelope_rail(self):
        result = self._telemetry_tester(
            CAN_ID_POWER_TELEMETRY, power_payload(v5_voltage_mv=9000)
        ).monitor_node3_power(duration_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("5 V rail", result.detail)

    def test_node3_power_fails_on_an_overcurrent_tether(self):
        result = self._telemetry_tester(
            CAN_ID_POWER_TELEMETRY, power_payload(tether_current_ma=30000)
        ).monitor_node3_power(duration_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("above the 25 A limit", result.detail)

    def test_node3_power_fails_on_an_implausible_pcb_temperature(self):
        result = self._telemetry_tester(
            CAN_ID_POWER_TELEMETRY, power_payload(pcb_temp_c_tenths=2000)
        ).monitor_node3_power(duration_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("PCB temperature", result.detail)

    # -- node2_depth ------------------------------------------------------
    def test_node2_depth_fails_when_nothing_arrives(self):
        result = VehicleStimulusTester(FakeBackend()).monitor_node2_depth(duration_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("no 0x200 navigation telemetry", result.detail)

    def test_node2_depth_fails_on_a_starved_stream(self):
        """Two frames in half a second is not a 100 Hz stream."""
        result = self._telemetry_tester(
            CAN_ID_NAV_TELEMETRY, nav_payload(), repeats=2
        ).monitor_node2_depth(duration_s=0.5)
        self.assertFalse(result.passed)
        self.assertIn("only 4.0 Hz", result.detail)
        self.assertIn("broken stream", result.detail)

    def test_node2_depth_fails_on_an_undecodable_frame(self):
        result = self._telemetry_tester(
            CAN_ID_NAV_TELEMETRY, b"\x01\x02\x03"
        ).monitor_node2_depth(duration_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("could not be decoded as NavTelemetry", result.detail)

    def test_node2_depth_fails_on_a_negative_depth(self):
        result = self._telemetry_tester(
            CAN_ID_NAV_TELEMETRY, nav_payload(depth_meters=-1.0)
        ).monitor_node2_depth(duration_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("clamps a small negative reading to 0.0", result.detail)

    def test_node2_depth_fails_on_a_non_finite_reading(self):
        result = self._telemetry_tester(
            CAN_ID_NAV_TELEMETRY, nav_payload(depth_meters=float("inf"))
        ).monitor_node2_depth(duration_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("non-finite depth_meters", result.detail)

    # -- node2_pwm / node2_all -------------------------------------------
    def test_node2_pwm_fails_when_the_target_is_never_reached(self):
        statuses = [status(self.NEUTRAL, sim_time_ms=i * 10) for i in range(40)]
        result = VehicleStimulusTester(FakeBackend(statuses=statuses)).test_node2_pwm(
            0, 1700, **self.FAST
        )
        self.assertFalse(result.passed)
        self.assertIn("never reached 1700 us", result.detail)
        self.assertIn("ESC gate is open", result.detail)

    def test_node2_pwm_blames_a_closed_gate_rather_than_the_pwm_path(self):
        """
        A budget that expires while the board is still arming has not tested the
        PWM path at all, and must not be reported as though it had.

        ``RAMP_TIMEOUT_S`` is a fixed cushion on top of the slew ramp, while the
        arming gate is 3000 ms of *simulated* time, so a slow host can spend the
        entire default budget arming. Saying only "never reached" sends the
        operator after a PWM path that was never exercised.
        """
        board = FakeBackend(statuses=[status(self.NEUTRAL, sim_time_ms=i * 10) for i in range(40)])
        result = VehicleStimulusTester(board).test_node2_pwm(0, 1700, **self.FAST)
        self.assertFalse(result.passed)
        last_sim_ms = board.latest_outputs().sim_time_ms
        self.assertLess(
            last_sim_ms,
            ESC_ARMING_SIM_MS,
            "this test is only meaningful while the gate is still shut",
        )
        self.assertIn(
            f"the ESC gate was still closed at sim_time_ms={last_sim_ms}", result.detail
        )
        self.assertIn("has NOT examined the PWM path", result.detail)
        self.assertIn("too-short timeout_s or a slow host", result.detail)
        # The mechanical evidence sentences are still there, so the new branch
        # adds a diagnosis rather than replacing one.
        self.assertIn("never reached 1700 us", result.detail)
        self.assertIn("ESC gate is open", result.detail)

    def test_node2_pwm_does_not_blame_the_gate_once_it_has_opened(self):
        """
        The discriminator for the branch above: past the boundary, the PWM path is
        what is on trial and the message must say so.
        """
        statuses = [
            status(self.NEUTRAL, sim_time_ms=ESC_ARMING_SIM_MS + i * 10) for i in range(40)
        ]
        board = FakeBackend(statuses=statuses)
        result = VehicleStimulusTester(board).test_node2_pwm(0, 1700, **self.FAST)
        self.assertFalse(result.passed)
        self.assertGreaterEqual(
            board.latest_outputs().sim_time_ms,
            ESC_ARMING_SIM_MS,
            "this test is only meaningful once the gate has opened",
        )
        self.assertIn("never reached 1700 us", result.detail)
        self.assertNotIn(
            "gate was still closed",
            result.detail,
            "an open gate must not be blamed for a PWM path that did not move",
        )

    def test_node2_pwm_fails_when_the_hold_decays(self):
        """
        A 100 ms watchdog lapse looks exactly like this, so the check must catch
        a command that was accepted once and then decayed.
        """
        held = [NEUTRAL_US] * 8
        held[0] = 1700
        statuses = [status(held, sim_time_ms=0)] + [
            status(self.NEUTRAL, sim_time_ms=10 + i * 10) for i in range(20)
        ]
        result = VehicleStimulusTester(FakeBackend(statuses=statuses)).test_node2_pwm(
            0, 1700, duration_s=0.05, **self.FAST
        )
        self.assertFalse(result.passed)
        self.assertIn("did not hold 1700 us", result.detail)
        self.assertIn("watchdog", result.detail)

    def test_node2_pwm_fails_when_an_unchannel_is_disturbed(self):
        """
        Covers the ``polluted`` branch: one channel commanded, another moves.

        Nothing else in the suite reaches this, and a check that ignored the other
        seven channels would still pass the two tests above.
        """
        held = [NEUTRAL_US] * 8
        held[0] = 1700
        disturbed = list(held)
        disturbed[1] = 1600
        statuses = [status(held, sim_time_ms=0)] + [
            status(disturbed, sim_time_ms=10 + i * 10) for i in range(20)
        ]
        result = VehicleStimulusTester(FakeBackend(statuses=statuses)).test_node2_pwm(
            0, 1700, duration_s=0.05, **self.FAST
        )
        self.assertFalse(result.passed)
        self.assertIn("moved off 1500 us", result.detail)
        self.assertIn("[1]", result.detail)

    def test_node2_pwm_fails_when_the_sustain_window_is_unobserved(self):
        """Reached the target, then the readback stopped: the hold is unverified."""
        held = [NEUTRAL_US] * 8
        held[0] = 1700
        result = VehicleStimulusTester(
            FakeBackend(statuses=[status(held, sim_time_ms=0)])
        ).test_node2_pwm(0, 1700, duration_s=0.05, **self.FAST)
        self.assertFalse(result.passed)
        self.assertIn("no fresh 0x7FE snapshot", result.detail)

    def test_node2_pwm_fails_when_the_outputs_do_not_release(self):
        held = [NEUTRAL_US] * 8
        held[0] = 1700
        statuses = [status(held, sim_time_ms=i * 10) for i in range(20)]
        result = VehicleStimulusTester(FakeBackend(statuses=statuses)).test_node2_pwm(
            0, 1700, duration_s=0.02, **self.FAST
        )
        self.assertFalse(result.passed)
        self.assertIn("did not return to 1500 us", result.detail)

    def test_node2_all_fails_on_a_partial_allocation(self):
        target = [1650] * 8
        partial = list(target)
        partial[3] = 1500
        statuses = [status(target, sim_time_ms=0)] + [
            status(partial, sim_time_ms=10 + i * 10) for i in range(20)
        ]
        result = VehicleStimulusTester(FakeBackend(statuses=statuses)).test_node2_all(
            1650, duration_s=0.05, **self.FAST
        )
        self.assertFalse(result.passed)
        self.assertIn("did not hold", result.detail)
        self.assertIn("[3]", result.detail)

    def test_node2_all_fails_when_the_vector_is_never_reached(self):
        statuses = [status(self.NEUTRAL, sim_time_ms=i * 10) for i in range(20)]
        result = VehicleStimulusTester(FakeBackend(statuses=statuses)).test_node2_all(
            1650, **self.FAST
        )
        self.assertFalse(result.passed)
        self.assertIn("never reached 1650 us on all 8 channels", result.detail)

    # -- node2_arming -----------------------------------------------------
    def test_arming_fails_when_the_gate_is_already_open(self):
        """A warm engine cannot show the gate; the check says so instead of guessing."""
        result = VehicleStimulusTester(
            FakeBackend(statuses=[status(self.NEUTRAL, sim_time_ms=4000)])
        ).test_node2_arming(**self.ARMING_FAST)
        self.assertFalse(result.passed)
        self.assertIn("already open", result.detail)
        self.assertIn("Restart the engine", result.detail)

    def test_arming_fails_when_outputs_are_not_neutral_before_arming(self):
        hot = [NEUTRAL_US] * 8
        hot[2] = 1600
        result = VehicleStimulusTester(
            FakeBackend(statuses=[status(hot, sim_time_ms=100)])
        ).test_node2_arming(**self.ARMING_FAST)
        self.assertFalse(result.passed)
        self.assertIn("not neutral before arming", result.detail)

    def test_arming_fails_when_the_window_never_elapses(self):
        """A frozen virtual clock admits no arming verdict at all."""
        board = ScriptedArmingBoard(sim_step_ms=0)
        result = VehicleStimulusTester(board).test_node2_arming(**self.ARMING_FAST)
        self.assertFalse(result.passed)
        self.assertIn("never elapsed", result.detail)
        self.assertIn("virtual clock is not advancing", result.detail)

    def test_arming_fails_when_the_gate_lets_through_early(self):
        """
        The discriminating case: the gate opens early and thrust appears while the
        window is still open. This is the assertion that actually distinguishes a
        real arming gate from "outputs happened to be neutral".
        """
        board = ScriptedArmingBoard(gate_opens_early=True)
        result = VehicleStimulusTester(board).test_node2_arming(**self.ARMING_FAST)
        self.assertFalse(result.passed)
        self.assertIn("left neutral at sim_time_ms=", result.detail)
        self.assertIn("must hold neutral for the whole arming window", result.detail)
        self.assertIsNotNone(board.armed_at_ms, "the board must have taken thrust at all")
        self.assertLess(
            board.armed_at_ms,
            ESC_ARMING_SIM_MS,
            "this test is only meaningful if the board armed *before* the boundary",
        )

    def test_arming_fails_when_too_few_samples_land_inside_the_window(self):
        """
        A single post-boundary snapshot used to make the intruder list empty and
        the gate look verified. The sample floor closes that.
        """
        # 1200 ms per read puts snapshots at 1200 and 2400 inside the window and
        # 3600 past it: two observations, below the floor of three.
        board = ScriptedArmingBoard(sim_step_ms=1200)
        result = VehicleStimulusTester(board).test_node2_arming(**self.ARMING_FAST)
        self.assertFalse(result.passed)
        self.assertIn("observation(s) landed strictly inside", result.detail)
        self.assertIn("unverified", result.detail)

    def test_arming_fails_when_the_gate_never_opens(self):
        """The window closes but the command never takes effect: the board is stuck."""
        board = ScriptedArmingBoard(stuck_after_arming=True)
        result = VehicleStimulusTester(board).test_node2_arming(**self.ARMING_FAST)
        self.assertFalse(result.passed)
        self.assertIn("never left", result.detail)
        self.assertIn("gate never opened", result.detail)

    def test_arming_fails_when_outputs_do_not_release_after_the_gate_opens(self):
        """A board that takes thrust and never gives it back is a fail-safe failure."""
        board = ScriptedArmingBoard()

        class StickyBoard(ScriptedArmingBoard):
            def _next_status(self):
                snapshot = super()._next_status()
                # Once armed, refuse to come back to neutral: the heartbeat lapse
                # the release is waiting for never arrives.
                if self.armed:
                    return OutputStatus(
                        (1700,) + (NEUTRAL_US,) * 7, False, 0, snapshot.sim_time_ms
                    )
                return snapshot

        board = StickyBoard()
        result = VehicleStimulusTester(board).test_node2_arming(**self.ARMING_FAST)
        self.assertFalse(result.passed)
        self.assertIn("did not return to", result.detail)

    def test_arming_passes_when_the_gate_holds_then_opens(self):
        """The green counterpart, so the failure tests above are not vacuous."""
        board = ScriptedArmingBoard()
        result = VehicleStimulusTester(board).test_node2_arming(**self.ARMING_FAST)
        self.assertTrue(result.passed, result.detail)
        self.assertIn("gate held neutral across", result.detail)

    # -- sniff ------------------------------------------------------------
    def test_sniff_fails_when_the_link_is_silent(self):
        result = VehicleStimulusTester(FakeBackend()).sniff(duration_s=0.05)
        self.assertFalse(result.passed)
        self.assertIn("no frames decoded", result.detail)


class TestStimulusAgainstServer(ServerBackedTestCase):
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

    def _run(self, argv):
        """
        Run the CLI against this test's own engine.

        ``run_cli`` deliberately defaults to a fake transport so no status test
        can spawn a process; a server-backed test must ask for a real one, pointed
        at the ``SilServerProcess`` this class owns rather than the fixed default
        port.
        """
        port = self.server.port
        return run_cli(
            argv,
            # attach_existing=True: this class started the engine, through
            # SilServerProcess, so the backend is attaching to its own.
            backend_factory=lambda args: SilSocketBackend(
                args.host, port, auto_start=False, attach_existing=True
            ),
        )

    def test_auto_cli_succeeds_against_the_real_server(self):
        status_code, printed = self._run(["--mode", "sil", "--port", str(self.server.port), "--auto"])
        self.assertEqual(
            status_code,
            0,
            f"--auto must pass against the real server. Output:\n{printed}",
        )
        self.assertNotIn("[FAIL]", printed)

    def test_results_are_streamed_before_the_run_finishes(self):
        """
        Invariant: evidence appears as checks complete, not only at the end.

        Asserted against the real engine: ``emergency_break`` is the last check in
        ``--auto`` and takes about a second, so if nothing were streamed the
        earlier results would still be missing when it starts.
        """
        streamed = []

        class Spy:
            """Replaces stdout so main()'s prints are captured, not leaked."""

            def write(self, text):
                if text.startswith("[PASS]") or text.startswith("[FAIL]"):
                    streamed.append(text)
                return len(text)

            def flush(self):
                pass

        port = self.server.port
        with mock.patch.object(sys, "stdout", Spy()):
            status_code = main(
                ["--mode", "sil", "--port", str(port), "--auto"],
                # attach_existing=True: this class owns the engine.
                backend_factory=lambda args: SilSocketBackend(
                    args.host, port, auto_start=False, attach_existing=True
                ),
            )
        passed = [line for line in streamed if line.startswith("[PASS]")]
        self.assertEqual(status_code, 0, "\n".join(streamed))
        self.assertGreaterEqual(
            len(passed), 7, f"expected per-check lines during the run, got {passed}"
        )

    def test_emergency_break_fails_when_a_fresh_engine_cannot_arm_in_time(self):
        """
        The reproduced false success, against the real engine.

        The defect under test is the original one: run against a board that is
        still inside its 3000 ms arming window, the old check reported "[PASS] ...
        a 1800 us command afterwards did not resume thrust" and exited 0, because
        a disarmed board refuses thrust for reasons that have nothing to do with
        the E-stop latch.

        Here the board genuinely cannot arm within the budget, so the latch claim
        is unobservable and the check must fail. The engine needs roughly 2.5 s of
        wall clock to serve its 3000 ms arming window, and the simulated rate is
        raised so the check's own budget is far shorter than that: it cannot wait
        the window out, which is the situation the original defect lived in.
        """
        port = self.server.port
        # attach_existing=True: this class owns the engine.
        tester = VehicleStimulusTester(
            SilSocketBackend("127.0.0.1", port, auto_start=False, attach_existing=True)
        )
        tester.backend.connect()
        self.addCleanup(tester.backend.close)
        result = tester.test_emergency_break(
            timeout_s=0.5,
            sim_rate=1e6,        # budget collapses to slack_s of wall time
            slack_s=0.05,
            unauthorized_window_s=0.1,
            estop_window_s=0.1,
            first_output_timeout_s=0.5,
        )
        self.assertFalse(
            result.passed,
            f"a board that never accepted thrust must not report a proven latch:\n{result.detail}",
        )
        self.assertIn("UNOBSERVABLE", result.detail)
        self.assertIn("ESC_STATE_ACTIVE", result.detail)
        self.assertIn("FAILED rather than as a partial pass", result.detail)

    def test_emergency_break_never_exits_zero_without_the_latch_evidence(self):
        """
        Invariant, against the real engine: exit 0 requires the earned evidence.

        Run alone the check now waits out the board's mandatory arming window and
        can therefore earn the claim. That is the honest outcome, and this test
        pins the condition under which it is allowed: a pass must name the
        pre-ESTOP ACTIVE observation it depends on. A pass without that evidence is
        the original defect, whatever the exit code.
        """
        status_code, printed = self._run(
            ["--mode", "sil", "--port", str(self.server.port), "--emergency-break"]
        )
        if status_code == 0:
            self.assertIn("The latch claim is earned", printed)
            self.assertIn("ESC ACTIVE, accepting thrust", printed)
        else:
            self.assertNotIn("did not resume thrust", printed.split("UNOBSERVABLE")[-1])
            self.assertIn("UNOBSERVABLE", printed)

    def test_emergency_break_passes_once_the_board_is_armed(self):
        """
        The green half of the pair, against the real engine.

        ``--node2-arm`` runs first and leaves the board ESC ACTIVE, so the
        emergency check can observe the board accepting thrust and its post-ESTOP
        claim becomes earned.
        """
        status_code, printed = self._run(
            ["--mode", "sil", "--port", str(self.server.port), "--node2-arm", "--emergency-break"]
        )
        self.assertEqual(status_code, 0, f"an armed board must pass:\n{printed}")
        self.assertIn("The latch claim is earned", printed)
        self.assertIn("ESC ACTIVE, accepting thrust", printed)


# --------------------------------------------------------------------------
# Real-engine integration coverage
# --------------------------------------------------------------------------
#
# Everything above proves what the TOOL does with a scripted bus. What follows
# drives each vehicle node through the real C engine, so a regression in a
# node's own firmware is caught rather than asserted about in prose.
#
# The numbers asserted below are read out of the node firmware and the mock plant
# that the SIL engine actually links (tests/CMakeLists.txt:288-299), not guessed.
# A range would be the wrong instrument here: a node that started reporting a
# different but still plausible pressure would sail through a range assertion,
# and this suite is only worth having if the exact reported value is pinned.

# Node 1 (0x210) is a fixed synthetic BME280 reading. The 6-DOF plant drives the
# MS5837, the IMU, the TPS25990 bricks and the INA226
# (tests/mocks/mock_physics.c:332-361) but never the BME280, so the enclosure
# values are exactly the sensor mock's reset defaults, sampled by node 1 at 10 Hz
# (nodes/node1_pi_shield/Core/Src/app.c:159) and copied straight into the record
# at nodes/node1_pi_shield/Core/Src/app.c:206-212.
SIL_ENV_PRESSURE_HPA = 1013.25  # tests/mocks/mock_sensors.c:50
SIL_ENV_HUMIDITY_PCT = 35.0     # tests/mocks/mock_sensors.c:51
SIL_ENV_TEMPERATURE_C = 24.0    # tests/mocks/mock_sensors.c:52
# Dry, and provably so: 35 % is below the 80 % humidity trip
# (shared/include/rov_safety.h:19) evaluated at node1 app.c:181, the constant
# pressure gives a 0 hPa rise against the 15 hPa vacuum-decay trip
# (shared/include/rov_safety.h:18) at node1 app.c:192-194, and both floor probes
# read dry (tests/mocks/mock_bsp.c:33-34 via node1 app.c:204).
SIL_ENV_LEAK_FLAGS = 0
SIL_ENV_PAYLOAD_LEN = 13  # struct "<3fB" (sil_protocol._STRUCT_3FB)

# Node 3 (0x300) IS plant driven: the engine enables the plant
# (tests/sil_bridge_server.c:218) and every 10 ms tick the plant overwrites the
# brick sensors (tests/mocks/mock_bsp.c:50-53 -> mock_physics_step). With all
# eight thrusters at neutral the reported numbers are fixed, not sampled.
SIL_TETHER_MV = 48000  # mock_physics.c:80 holds the tether at 48.0 V;
                       # nodes/node3_power_slab/Core/Src/app.c:176-181 scales it by 1000
SIL_V5_MV = 5200       # mock_physics.c:350 writes 5.2 V for brick 0 (the logic rail);
                       # node3 app.c:100-105
SIL_V5_MA = 2500       # mock_physics.c:350 writes 2.5 A for brick 0; node3 app.c:101-106
# mock_physics.c:176 sums two thruster currents per brick and
# mock_physics.c:71-74 gives a neutral thruster 0.2 A, so 0.4 A per brick;
# node3 app.c:111-115 scales that by 1000.
SIL_V12_MA = (400, 400, 400, 400)
# node3 app.c:87 seeds the maximum at 250 tenths, the logic brick reports 30.0 C
# (mock_physics.c:350 -> 300 tenths) and node3 app.c:128-130 keeps the largest.
SIL_PCB_TENTHS_C = 300
SIL_POWER_PAYLOAD_LEN = 20  # struct "<HHHH4HhH" (sil_protocol._STRUCT_POWER)
# The tether current is a quotient rather than a literal: node3 app.c:121 sums
# the five brick output powers and node3 app.c:160-161 divides by the measured
# 48.0 V tether, giving about 32.2 W / 48.0 V = 0.671 A, which node3 app.c:182
# truncates to milliamps. Asserted as a tight band because it is a float
# truncation of a computed sum, not a constant anybody chose.
SIL_TETHER_MA_MIN = 600
SIL_TETHER_MA_MAX = 800

# The emergency break latches for the life of the engine
# (nodes/node2_control_board/Core/Src/app.c:116 sets ESC_STATE_DISARMED and
# shared/src/rov_safety.c has no clear path), so it is always the last thing a
# test asks a fresh engine to do.
AUTHORIZED_BREAK_FRAME = can_stimulus.EMERGENCY_FRAME
UNAUTHORIZED_BREAK_FRAME = can_stimulus.UNAUTHORIZED_FRAME


class ObservingSilBackend(SilSocketBackend):
    """
    The real SIL transport plus a test-side witness.

    Three observations the checks themselves do not keep, so a test can assert on
    what the firmware actually produced instead of on the check's summary of it:

    * ``readbacks`` - every 0x7FE output-status snapshot, in arrival order, with
      the engine's own virtual clock attached;
    * ``received`` - every non-0x7FE frame the engine published;
    * ``sent_frames`` - every frame written, with its wall-clock time, so a 20 Hz
      resend is measured rather than assumed.

    It subclasses :class:`SilSocketBackend` rather than wrapping it, so the
    framing, the decoding, and the 0x7FE routing all remain the tool's: nothing
    here can make a test agree with the tool by disagreeing with the wire. A
    malformed 0x7FE still raises from ``OutputStatus.decode`` inside
    ``drain_frames``, which is the correct outcome - a decode error is a failed
    test, never a warning.

    ``received`` is collected from ``drain_frames``' own return value rather than
    from a frame listener, because ``_pump`` installs and then restores its own
    listener around every check; a listener registered at connect time would only
    ever see the gaps between checks.  The explicit 0x7FE filter below is now
    redundant - ``drain_frames`` withholds that channel by default, which
    ``TestSilSocketFraming`` pins - and is kept so this witness can never mistake
    a mock-BSP snapshot for telemetry even if that guarantee is ever lost.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.readbacks = []
        self.received = []
        self.sent_frames = []
        self._last_readback = None

    def send_frame(self, can_id, payload):
        self.sent_frames.append((time.monotonic(), can_id, bytes(payload)))
        super().send_frame(can_id, payload)

    def drain_frames(self, timeout_s=0.0, include_output_status=False):
        collected = super().drain_frames(timeout_s, include_output_status)
        for can_id, payload in collected:
            if can_id != CAN_ID_SIL_OUTPUT_STATUS:
                self.received.append((can_id, payload))
        self._capture_readback()
        return collected

    def _capture_readback(self):
        current = self.latest_outputs()
        if current is not None and current is not self._last_readback:
            self._last_readback = current
            self.readbacks.append(current)
        return current

    def pump_until(self, predicate, timeout_s):
        """
        Bounded wait for a 0x7FE snapshot satisfying ``predicate``.

        This is a *precondition* wait, not a stimulus check: it answers "is the
        engine in the state this test needs", and returns ``None`` when it is not
        so the caller can fail with a message that names the missing
        precondition. No verdict about the vehicle is ever taken from here, and
        the tool's own checks are never driven through it.
        """
        deadline = time.monotonic() + max(0.0, timeout_s)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return None
            self.drain_frames(min(can_stimulus.READ_POLL_S, remaining))
            current = self._capture_readback()
            if current is not None and predicate(current):
                return current

    def sends_of(self, can_id):
        """Every frame written to ``can_id``, as ``(monotonic, payload)`` pairs."""
        return [(when, payload) for when, frame_id, payload in self.sent_frames if frame_id == can_id]


class EngineRun(NamedTuple):
    """
    The engine a test is talking to, the backend on it, and the child it owns.

    ``process`` exists because ``SilServerProcess`` cannot answer "did the child
    actually die?" after teardown. ``stop()`` assigns ``self.process = None`` as
    its **last** statement (sil_test_support.py:244-252), so ``is_running``
    (sil_test_support.py:206-208) is False for every engine that has been through
    ``__exit__`` - including one that ignored ``terminate()`` and had to be killed.
    Asserting on ``is_running`` after teardown is therefore vacuous: it can only
    ever pass.

    Holding the ``Popen`` keeps the OS-level fact reachable afterwards, and
    ``poll() is not None`` is exactly that fact: the child has been reaped. This
    needs no change to Task 1's fixture, which is left byte-intact.
    """

    server: SilServerProcess
    backend: ObservingSilBackend
    process: subprocess.Popen


def _assert_engine_reaped(test, run):
    """
    The engine child must have exited, checked at the OS level.

    Deliberately not ``assertFalse(run.server.is_running)``: that attribute is
    False by construction after teardown, so it would pass even if a child had
    survived. ``Popen.poll()`` re-reads the process table, so it fails when one is
    still running, which is the only failure this claim is about.
    """
    test.assertIsNotNone(
        run.process.poll(),
        f"the sil_bridge_server child (pid {run.process.pid}) is still running after the test's "
        "context manager exited; terminate() was ignored and kill() did not finish, so this "
        "test leaked a process",
    )


def _force_kill(process):
    """addCleanup helper: never leave the probe process behind, pass or fail."""
    if process.poll() is None:
        process.kill()
        process.wait(timeout=5)


class TestEngineTeardownAssertion(unittest.TestCase):
    """
    The teardown assertion must be capable of failing, or it is decoration.

    The vacuous form it replaced (``assertFalse(server.is_running)`` after
    ``__exit__``) passed for every engine that ever existed, because ``stop()``
    nulls the fixture's own handle as its last statement. These two tests pin the
    replacement against that failure mode in both directions, so the property
    "nothing was left running" is a claim the suite actually makes rather than
    one it appears to make. They need no engine binary and cost no wall time.
    """

    def _probe(self, code):
        process = subprocess.Popen(
            [sys.executable, "-c", code],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        self.addCleanup(_force_kill, process)
        return process

    def test_a_surviving_child_fails_the_teardown_assertion(self):
        process = self._probe("import time; time.sleep(30)")
        self.assertIsNone(
            process.poll(), "the probe must be alive for this test to mean anything"
        )
        with self.assertRaises(AssertionError):
            _assert_engine_reaped(self, EngineRun(server=None, backend=None, process=process))

    def test_a_reaped_child_passes_the_teardown_assertion(self):
        process = self._probe("pass")
        process.wait(timeout=5)
        self.assertIsNotNone(process.poll(), "the probe must have exited")
        _assert_engine_reaped(self, EngineRun(server=None, backend=None, process=process))


@contextlib.contextmanager
def _engine(executable, test=None):
    """
    A private engine, the backend connected to it, and the child process it owns.

    Each test gets its own process on its own ephemeral port - ``SilServerProcess``
    reserves the port through ``free_tcp_port``, so no two tests and nothing
    outside this file can collide on 8765 - and the server is terminated on
    every exit path, including an assertion failure or a skip raised inside the
    block. The caller asserts on ``EngineRun.process.poll()`` afterwards, so
    "nothing was left running" is re-read from the OS rather than taken on trust
    from a fixture attribute that teardown has already cleared.

    ``test`` is the calling TestCase, published on it as ``server`` for as long
    as the engine lives.  That is how these tests get the engine's captured
    stdout/stderr attached to any assertion failure: ``ServerBackedTestCase``
    reads ``self.server``.  Passing it is optional so the helper stays usable
    from a plain function, but every call site here is a TestCase.
    """
    server = SilServerProcess(executable)
    with server:
        # attach_existing=True: _engine started this engine, so the backend behind
        # it is attaching to its own rather than commandeering a stranger's.
        backend = ObservingSilBackend("127.0.0.1", server.port, auto_start=False, attach_existing=True)
        backend.connect()
        # Captured while the engine is still up, because stop() nulls its own handle.
        process = server.process
        if test is not None:
            test.server = server
        try:
            yield EngineRun(server=server, backend=backend, process=process)
        finally:
            backend.close()


class SilNodeIntegrationTests(ServerBackedTestCase):
    """
    One end-to-end test per vehicle node, each against its own real engine.

    Every assertion here is made twice over, deliberately: once on the
    ``CheckResult`` the tool returned, and once on the bytes the C engine
    actually put on the wire. The first pins the tool's verdict; the second pins
    the vehicle, and the second is the one that fails when a node's firmware
    regresses. Nothing is asserted on "the send did not throw".
    """

    def setUp(self):
        # Resolved per test rather than per class, so an unbuilt binary skips
        # every integration test with the build instructions instead of erroring
        # on a missing file. A skip is honest here; a false pass is not.
        self.server_exe = require_server_executable()

    # -- node 1: Pi shield -------------------------------------------------
    def test_node1_reports_the_simulated_bme280_on_0x210(self):
        """
        Node 1's environment telemetry, decoded from the real engine.

        Asserts the 0x210 frames really arrived (at least two, which is what
        distinguishes a live stream from one stale frame, because the server
        emits the first immediately and then 1 Hz -
        tests/sil_bridge_server.c:376-382), that each is a full 13-byte record,
        and that every field is the value the firmware computes from the sensor
        mock. The 6-DOF plant never drives the BME280
        (tests/mocks/mock_physics.c:332-361), so 1013.25 hPa / 35 % / 24.0 C /
        leak_flags 0 is a determinate expectation, not a plausible range: a
        regression that halved the reported humidity would be caught here and
        would sail through a range check.
        """
        with _engine(self.server_exe, self) as run:
            backend = run.backend
            result = VehicleStimulusTester(backend).monitor_node1_env(3.0)
            self.assertTrue(result.passed, result.detail)

            frames = [payload for can_id, payload in backend.received if can_id == CAN_ID_ENV_TELEMETRY]
            self.assertGreaterEqual(
                len(frames), 2, f"expected a live 0x210 stream, got {len(frames)} frame(s)"
            )
            for index, payload in enumerate(frames):
                self.assertEqual(
                    len(payload),
                    SIL_ENV_PAYLOAD_LEN,
                    f"0x210 frame {index} is {len(payload)} B; the packed rov_env_telemetry_t "
                    "is 13 B and a short record cannot be a valid reading",
                )
                record = EnvTelemetry.unpack(payload)
                # Exact equality, not a tolerance. All three values are exactly
                # representable in IEEE-754 binary32 - 1013.25 is 1013 + 1/4, and
                # 35.0 and 24.0 are integers - so they survive the mock's float,
                # the C struct's float, and struct's "<3f" round trip with no
                # rounding at any hop. A tolerance here would let a 0.0004 drift
                # through a test whose stated purpose is catching drift, and would
                # contradict the principle recorded at the top of this section.
                self.assertEqual(
                    record.pressure_hpa,
                    SIL_ENV_PRESSURE_HPA,
                    f"0x210 frame {index}: node1 app.c:206 publishes the BME280 reading verbatim",
                )
                self.assertEqual(
                    record.humidity_pct,
                    SIL_ENV_HUMIDITY_PCT,
                    f"0x210 frame {index}: node1 app.c:208",
                )
                self.assertEqual(
                    record.temperature_c,
                    SIL_ENV_TEMPERATURE_C,
                    f"0x210 frame {index}: node1 app.c:210",
                )
                self.assertEqual(
                    record.leak_flags,
                    SIL_ENV_LEAK_FLAGS,
                    f"0x210 frame {index} flags a leak; a set bit means node 1 should already "
                    "have broadcast an emergency break (node1 app.c:220-222)",
                )
        _assert_engine_reaped(self, run)

    # -- node 2: control board --------------------------------------------
    def test_node2_arming_gate_is_neutral_before_3000ms_and_open_after(self):
        """
        The mandatory ESC arming gate, in both directions, on the real board.

        A fresh engine per test is mandatory, not a nicety: the gate opens on the
        engine's virtual clock and cannot be observed retroactively, so
        ``test_node2_arming`` legitimately fails against a board whose sim clock
        has already passed ``ESC_ARMING_TIME_MS``.

        Asserts on the engine's own 0x7FE readback, independently of the check:
        that at least ``MIN_ARMING_SAMPLES`` snapshots landed strictly inside the
        window, that every one of them held all eight channels at 1500 us while a
        1700 us command was being resent, and that the first snapshot to leave
        neutral is at or after the boundary. That third assertion is what
        separates a real arming gate from "the outputs happened to be neutral".
        """
        with _engine(self.server_exe, self) as run:
            backend = run.backend
            result = VehicleStimulusTester(backend).test_node2_arming()
            self.assertTrue(result.passed, result.detail)

            inside = [s for s in backend.readbacks if s.sim_time_ms < ESC_ARMING_SIM_MS]
            self.assertGreaterEqual(
                len(inside),
                can_stimulus.MIN_ARMING_SAMPLES,
                "the arming window must be watched, not inferred from a single post-boundary "
                f"snapshot; only {len(inside)} readback(s) landed inside {ESC_ARMING_SIM_MS} ms",
            )
            intruders = [s for s in inside if tuple(s.pwms) != NEUTRAL_PWMS]
            self.assertFalse(
                intruders,
                "a channel left neutral while the arming gate was still open "
                f"(node2 app.c:192-194 must hold {NEUTRAL_US} us throughout); first offender "
                f"sim_time_ms={intruders[0].sim_time_ms} showing {list(intruders[0].pwms)}"
                if intruders
                else "",
            )
            after = [s for s in backend.readbacks if s.sim_time_ms >= ESC_ARMING_SIM_MS]
            self.assertTrue(after, "no readback was published after the arming window closed")
            moving = [s for s in after if any(pulse != NEUTRAL_US for pulse in s.pwms)]
            self.assertTrue(
                moving,
                f"the gate never opened: every readback at or after {ESC_ARMING_SIM_MS} ms of sim "
                f"time was still neutral, last was {after[-1].describe()}",
            )
            self.assertGreaterEqual(
                moving[0].sim_time_ms,
                ESC_ARMING_SIM_MS,
                "the board started moving before its arming window closed",
            )
            self.assertEqual(
                [s.sim_time_ms for s in backend.readbacks],
                sorted(s.sim_time_ms for s in backend.readbacks),
                "the engine's virtual clock must never run backwards",
            )
        _assert_engine_reaped(self, run)

    def test_node2_pwm_resends_at_20hz_and_holds_on_the_firmware_readback(self):
        """
        One thruster channel, driven and held, on the real board.

        Two independent claims:

        1. the tool resends the 0x100 frame at 20 Hz, measured from the recorded
           sends, and no gap between two commands reaches the firmware's 100 ms
           heartbeat window (shared/include/rov_parameters.h:71);
        2. the ENGINE-produced 0x7FE readback shows the commanded value on the
           driven channel and 1500 us on the other seven, for several snapshots -
           not merely that the write did not raise.

        The gate is established first with ``test_node2_arming`` rather than
        assumed: node2 app.c:127-129 refuses thruster commands until the ESC
        state is ACTIVE, so a cold engine cannot hold a PWM at all.

        The tail of the test then demonstrates *why* 20 Hz matters, on real
        firmware: the command is resent, thrust is observed, the resend stops,
        and node2 app.c:183-189 must force every channel back to neutral
        immediately (``node2_force_neutral`` writes 1500 us with no slew, so the
        drop is not gradual).
        """
        channel = 3
        pulse_us = 1750
        hold_s = 1.0
        with _engine(self.server_exe, self) as run:
            backend = run.backend
            tester = VehicleStimulusTester(backend)
            armed = tester.test_node2_arming()
            self.assertTrue(armed.passed, f"arming precondition failed: {armed.detail}")

            # Two marks, not one. The arming precondition above drove the board to
            # SMOKE_SINGLE_PWM_US (1700) and filled backend.readbacks with those
            # snapshots; windowing only the SENT frames would let an arming-era
            # readback satisfy the floor below if that constant were ever retuned to
            # this test's pulse, which would make "the readback is the evidence"
            # vacuous.
            send_mark = len(backend.sent_frames)
            readback_mark = len(backend.readbacks)
            result = tester.test_node2_pwm(channel, pulse_us, duration_s=hold_s)
            self.assertTrue(result.passed, result.detail)

            # Only this check's traffic: the arming precondition above wrote its
            # own 0x100 frames (1700 us, can_stimulus.SMOKE_SINGLE_PWM_US) and
            # those are not evidence about this command.
            phase = backend.sent_frames[send_mark:]
            for _, frame_id, _ in phase:
                self.assertEqual(frame_id, CAN_ID_THRUSTER_CMD, "this check writes nothing else")
            vectors = [ThrusterCommand.unpack(payload).pwm_us for _, _, payload in phase]
            # The check holds the command and then releases it, so the 0x100
            # stream legitimately carries two vectors: the commanded one and the
            # all-neutral one. Each is asserted against its own role.
            release_at = next(
                (index for index, pwms in enumerate(vectors) if all(p == NEUTRAL_US for p in pwms)),
                None,
            )
            self.assertIsNotNone(release_at, "the command was never released back to neutral")
            commanded, released = vectors[:release_at], vectors[release_at:]
            self.assertGreaterEqual(
                len(commanded),
                10,
                f"only {len(commanded)} 0x100 frame(s) carried the command; the 20 Hz resend is the point",
            )
            for pwms in commanded:
                self.assertEqual(len(pwms), 8, "an 8-channel frame is required")
                self.assertEqual(
                    pwms[channel],
                    pulse_us,
                    f"a resent frame carried {pwms[channel]} us on channel {channel} instead of "
                    f"{pulse_us}: the resend must repeat the command, not the first one only",
                )
            self.assertTrue(
                all(all(p == NEUTRAL_US for p in pwms) for pwms in released),
                "the release must command neutral on every channel",
            )
            sends = [when for when, _, _ in phase[:release_at]]
            gaps = [later - earlier for earlier, later in zip(sends, sends[1:])]
            resend_hz = (len(sends) - 1) / (sends[-1] - sends[0])
            self.assertLessEqual(
                max(gaps),
                can_stimulus.HEARTBEAT_TIMEOUT_S,
                f"the longest gap between 0x100 frames was {max(gaps) * 1000:.1f} ms, which the "
                "100 ms watchdog (rov_parameters.h:71, node2 app.c:183-189) would have turned "
                "into a neutral drop",
            )
            self.assertGreaterEqual(
                resend_hz, 15.0, f"0x100 arrived at only {resend_hz:.1f} Hz, not the 20 Hz resend"
            )
            self.assertLessEqual(
                resend_hz, 25.0, f"0x100 arrived at {resend_hz:.1f} Hz, faster than the 20 Hz resend"
            )

            # The evidence is the readback, not the write - and only the readbacks
            # this check produced, per the readback mark set above.
            held = [s for s in backend.readbacks[readback_mark:] if s.pwms[channel] == pulse_us]
            self.assertGreaterEqual(
                len(held),
                3,
                f"only {len(held)} 0x7FE snapshot(s) inside the PWM check's own window showed "
                f"{pulse_us} us on channel {channel}",
            )
            for snapshot in held:
                self.assertEqual(snapshot.pwms[channel], pulse_us)
                others = [index for index in range(8) if index != channel]
                self.assertEqual(
                    [snapshot.pwms[index] for index in others],
                    [NEUTRAL_US] * len(others),
                    f"an uncommanded channel moved at sim_time_ms={snapshot.sim_time_ms}",
                )

            # Why the resend exists: go silent and the board must fail safe.
            # The command is fed at the same 20 Hz the tool uses, because a
            # slower feed would itself trip the 100 ms watchdog and the board
            # would never reach the target - which is the coupling this test
            # exists to make visible.
            thrust = ThrusterCommand(
                pwm_us=[pulse_us if index == channel else NEUTRAL_US for index in range(8)]
            ).pack()
            deadline = time.monotonic() + 3.0
            next_send = time.monotonic()
            driven = None
            while driven is None:
                now = time.monotonic()
                if now >= deadline:
                    break
                if now >= next_send:
                    backend.send_frame(CAN_ID_THRUSTER_CMD, thrust)
                    next_send = now + can_stimulus.COMMAND_PERIOD_S
                driven = backend.pump_until(
                    lambda s: s.pwms[channel] == pulse_us,
                    min(can_stimulus.COMMAND_PERIOD_S, deadline - time.monotonic()),
                )
            self.assertIsNotNone(
                driven,
                "an armed board must accept a fresh thruster command; if it does not, every "
                "observation above is vacuous",
            )
            self.assertTrue(
                any(pulse != NEUTRAL_US for pulse in backend.readbacks[-1].pwms),
                "the last readback before the silence must still show thrust, or the drop below "
                "proves nothing",
            )
            silence_start = backend.readbacks[-1]
            dropped = backend.pump_until(lambda s: tuple(s.pwms) == NEUTRAL_PWMS, 2.0)
            self.assertIsNotNone(
                dropped,
                f"the 100 ms heartbeat did not fire: the board stayed at "
                f"{list(silence_start.pwms)} after the command stopped arriving",
            )
            self.assertLessEqual(
                dropped.sim_time_ms - silence_start.sim_time_ms,
                250,
                f"the watchdog took {dropped.sim_time_ms - silence_start.sim_time_ms} ms of sim time "
                "to drop the channel; shared/src/rov_safety.c:34 fires on the first tick past 100 ms",
            )
            # Between the last fed command and the trip the board must still be
            # HOLDING the commanded value: a decaying channel here would mean the
            # drop below was a slew, not the watchdog.
            stragglers = [
                s
                for s in backend.readbacks
                if silence_start.sim_time_ms < s.sim_time_ms < dropped.sim_time_ms
            ]
            self.assertTrue(
                stragglers,
                "the channel dropped instantly instead of after the 100 ms heartbeat window, so "
                "the drop below is not evidence that the watchdog fired",
            )
            for snapshot in stragglers:
                self.assertEqual(
                    snapshot.pwms[channel],
                    pulse_us,
                    f"the channel decayed to {snapshot.pwms[channel]} us at "
                    f"sim_time_ms={snapshot.sim_time_ms} while the command was still being fed",
                )
            # Keep reading past the trip so the "nothing came back" claim below is
            # an observation rather than a gap in the record.
            backend.pump_until(lambda s: s.sim_time_ms > dropped.sim_time_ms, 0.6)
            after_drop = [
                s
                for s in backend.readbacks
                if s.sim_time_ms > dropped.sim_time_ms and any(p != NEUTRAL_US for p in s.pwms)
            ]
            self.assertEqual(
                after_drop,
                [],
                f"thrust came back after the watchdog dropped the channel, at "
                f"sim_time_ms={after_drop[0].sim_time_ms}" if after_drop else "",
            )
        _assert_engine_reaped(self, run)

    def test_node2_solenoid_mask_changes_the_firmware_readback(self):
        """
        A single-coil valve mask, on the real board.

        ``SMOKE_SOLENOID_MASK`` (0x0001, valve 0 coil A) is the known-good input:
        it energises exactly one of the two opposing coils per valve, so the
        interlock at shared/src/rov_can_protocol.c:120-126 accepts it and
        nodes/node2_control_board/Core/Src/app.c:150-153 actually calls
        ``bsp_solenoid_set``. A mask with both coils of a pair set would be
        zeroed and rejected, and the readback would then report the *previous*
        mask - which is why the mask is validated before it is used as evidence.

        Asserts the change rather than the value: a snapshot showing the mask
        must be preceded by one that did not, and followed by one back at 0x0000.
        """
        mask = can_stimulus.SMOKE_SOLENOID_MASK
        self.assertIsNone(
            can_stimulus.conflicting_solenoid_valve(mask),
            f"0x{mask:04X} energises both coils of a valve and the firmware would reject it",
        )
        with _engine(self.server_exe, self) as run:
            backend = run.backend
            result = VehicleStimulusTester(backend).test_node2_solenoid(mask)
            self.assertTrue(result.passed, result.detail)

            first_on = next(
                (index for index, s in enumerate(backend.readbacks) if s.solenoids == mask),
                None,
            )
            self.assertIsNotNone(
                first_on,
                "the engine's own 0x7FE readback must show the mask the board was given",
            )
            self.assertTrue(
                any(s.solenoids != mask for s in backend.readbacks[:first_on]),
                f"every readback up to the first match already reported 0x{mask:04X}; the board "
                "starts at 0x0000 (node2 app.c:72), so a pre-existing match would mean the "
                "frame was ignored and the change never observed",
            )
            self.assertTrue(
                any(s.solenoids == 0 for s in backend.readbacks[first_on + 1:]),
                "the board must return to 0x0000 after being commanded to release",
            )
        _assert_engine_reaped(self, run)

    def test_node2_emergency_break_latches_the_brake_at_neutral(self):
        """
        The emergency break, on the real board, with the signature check intact.

        Asserts that both frames the check sends really went out - the authorized
        ``AA 55`` and the unauthorized ``AA 56`` that must be ignored
        (nodes/node2_control_board/Core/Src/app.c:112) - and that the engine's own
        0x7FE readback shows the brake latched with all eight channels at 1500 us
        (app.c:115-118) and that nothing moved afterwards. ``test_emergency_break``
        arranges its own precondition, so this cannot pass against a board that
        was never ACTIVE.
        """
        with _engine(self.server_exe, self) as run:
            backend = run.backend
            result = VehicleStimulusTester(backend).test_emergency_break()
            self.assertTrue(result.passed, result.detail)

            written = [payload for _, payload in backend.sends_of(CAN_ID_EMERGENCY_BREAK)]
            self.assertIn(
                AUTHORIZED_BREAK_FRAME,
                written,
                "the authorized 0xAA 0x55 frame must actually be written, signature included",
            )
            self.assertIn(
                UNAUTHORIZED_BREAK_FRAME,
                written,
                "the negative case is the check's own evidence: without the 0xAA 0x56 frame "
                "there is nothing showing the signature is enforced",
            )

            braked = [s for s in backend.readbacks if s.brake_active]
            self.assertTrue(
                braked,
                "no 0x7FE snapshot reported brake_active, so the latch was never observed on the "
                "firmware readback",
            )
            for snapshot in braked:
                self.assertEqual(
                    tuple(snapshot.pwms),
                    NEUTRAL_PWMS,
                    f"the brake latched at sim_time_ms={snapshot.sim_time_ms} with "
                    f"{list(snapshot.pwms)}; app.c:118 calls node2_force_neutral()",
                )
            latch_at = backend.readbacks.index(braked[0])
            resumed = [
                s
                for s in backend.readbacks[latch_at:]
                if any(pulse != NEUTRAL_US for pulse in s.pwms)
            ]
            self.assertFalse(
                resumed,
                "a channel left neutral after the brake latched, at "
                f"sim_time_ms={resumed[0].sim_time_ms} showing {list(resumed[0].pwms)}; app.c:116 "
                "sets ESC_STATE_DISARMED, which never clears"
                if resumed
                else "",
            )
        _assert_engine_reaped(self, run)

    # -- node 3: power slab -----------------------------------------------
    def test_node3_reports_the_plant_power_telemetry_on_0x300(self):
        """
        Node 3's power telemetry, decoded from the real engine.

        Requires at least one 0x300 frame (the server emits the first on connect
        and then 1 Hz - tests/sil_bridge_server.c:383-389), that it is a full
        20-byte ``PowerTelemetry`` record, and that every field carries the value
        the power slab computes from the plant-driven brick sensors. The tether
        current is a computed quotient, so it is asserted as a tight band with
        its derivation; everything else is pinned exactly. The fault bit must be
        clear: a set bit means the slab latched a fault and broadcast a 0x005
        eFuse alert (nodes/node3_power_slab/Core/Src/app.c:185-195), which would
        have tripped this board's brake.
        """
        with _engine(self.server_exe, self) as run:
            backend = run.backend
            result = VehicleStimulusTester(backend).monitor_node3_power(3.0)
            self.assertTrue(result.passed, result.detail)

            frames = [payload for can_id, payload in backend.received if can_id == CAN_ID_POWER_TELEMETRY]
            self.assertGreaterEqual(
                len(frames), 2, f"expected a live 0x300 stream, got {len(frames)} frame(s)"
            )
            for index, payload in enumerate(frames):
                self.assertEqual(
                    len(payload),
                    SIL_POWER_PAYLOAD_LEN,
                    f"0x300 frame {index} is {len(payload)} B; the packed rov_power_telemetry_t "
                    "is 20 B and a short record cannot be a valid reading",
                )
                record = PowerTelemetry.unpack(payload)
                self.assertEqual(
                    record.tether_voltage_mv,
                    SIL_TETHER_MV,
                    f"0x300 frame {index}: node3 app.c:181 publishes the measured tether in mV",
                )
                self.assertEqual(
                    record.v5_voltage_mv,
                    SIL_V5_MV,
                    f"0x300 frame {index}: node3 app.c:105 publishes brick 0's output rail",
                )
                self.assertEqual(
                    record.v5_current_ma,
                    SIL_V5_MA,
                    f"0x300 frame {index}: node3 app.c:106 publishes brick 0's load current",
                )
                self.assertEqual(
                    tuple(record.v12_current_ma),
                    SIL_V12_MA,
                    f"0x300 frame {index}: node3 app.c:115 publishes one current per 12 V brick",
                )
                self.assertEqual(
                    len(record.v12_current_ma), 4, "four 12 V bricks are reported (rov_parameters.h:25)"
                )
                self.assertGreaterEqual(
                    record.tether_current_ma,
                    SIL_TETHER_MA_MIN,
                    f"0x300 frame {index} reports a tether current of "
                    f"{record.tether_current_ma / 1000.0:.3f} A, below the "
                    f"{SIL_TETHER_MA_MIN / 1000.0:.2f} A the plant's summed brick power "
                    "produces (node3 app.c:121, :160-161)",
                )
                self.assertLessEqual(
                    record.tether_current_ma,
                    SIL_TETHER_MA_MAX,
                    f"0x300 frame {index} reports a tether current of "
                    f"{record.tether_current_ma / 1000.0:.3f} A. node3 app.c:160-161 divides "
                    "the summed brick power by the 48.0 V tether, which is about "
                    f"{SIL_TETHER_MA_MIN / 1000.0:.2f}-{SIL_TETHER_MA_MAX / 1000.0:.2f} A here",
                )
                self.assertLessEqual(
                    record.tether_current_ma,
                    25000,
                    f"0x300 frame {index} reports {record.tether_current_ma / 1000.0:.2f} A, "
                    "above the 25 A tether limit (rov_parameters.h:18)",
                )
                self.assertEqual(
                    record.pcb_temp_c_tenths,
                    SIL_PCB_TENTHS_C,
                    f"0x300 frame {index}: node3 app.c:183 publishes the hottest sensor in tenths "
                    f"of a degree, so {record.pcb_temp_c_tenths / 10.0:.1f} C",
                )
                self.assertEqual(
                    record.status_flags & 0x0001,
                    0,
                    f"0x300 frame {index} latched a fault (status_flags=0x{record.status_flags:04X}); "
                    "node3 app.c:185-195 would have broadcast a 0x005 eFuse alert that trips this "
                    "board's brake (node2 app.c:121-126)",
                )
        _assert_engine_reaped(self, run)

    # -- the property this suite exists for --------------------------------
    def test_a_node_that_stops_acting_produces_failed_checks_not_passes(self):
        """
        The regression this whole class exists to catch.

        A positive test can only catch a broken node if the check is capable of
        failing. The cheapest honest way to show it is: take a real engine, make
        it genuinely incapable of the thing a check requires, and assert the
        check reports FAILURE. Here the vehicle is latched into the emergency
        state, which is permanent for the life of the engine
        (nodes/node2_control_board/Core/Src/app.c:116), and both the thruster and
        the solenoid commands the tool sends afterwards are refused by the
        firmware: app.c:127-129 refuses thrust while the ESC state is not ACTIVE,
        and app.c:150 refuses a solenoid mask while the break is active.

        Pinned: neither check may report a pass, neither may raise, and the engine
        must never leave neutral. If a future change made either check pass
        vacuously - or made it pass by swallowing a missing readback - this test
        breaks, and with it the claim that the passing node tests above mean
        anything. No firmware is mutated and no assertion is faked: the vehicle
        really is stopped, and the really-produced check really does fail.
        """
        with _engine(self.server_exe, self) as run:
            backend = run.backend
            backend.send_frame(CAN_ID_EMERGENCY_BREAK, AUTHORIZED_BREAK_FRAME)
            latched = backend.pump_until(lambda s: s.brake_active, 3.0)
            self.assertIsNotNone(
                latched,
                "the authorized frame did not latch the brake, so this test would be proving "
                "nothing about a stopped vehicle",
            )
            self.assertEqual(tuple(latched.pwms), NEUTRAL_PWMS, "the trip must force neutral")

            tester = VehicleStimulusTester(backend)
            with self.subTest("thruster command against a latched board"):
                pwm = tester.test_node2_pwm(0, 1700, duration_s=0.2, timeout_s=1.0)
                self.assertFalse(
                    pwm.passed,
                    "a board latched into ESC_STATE_DISARMED must not report a held 1700 us "
                    f"command as a pass:\n{pwm.detail}",
                )
                self.assertIn("never reached 1700 us", pwm.detail)
            with self.subTest("solenoid command against a latched board"):
                solenoid = tester.test_node2_solenoid(can_stimulus.SMOKE_SOLENOID_MASK, timeout_s=0.8)
                self.assertFalse(
                    solenoid.passed,
                    "the firmware refuses 0x110 while the break is active (node2 app.c:150), so "
                    f"the readback can never change and this must fail:\n{solenoid.detail}",
                )
                self.assertIn("never reported it back", solenoid.detail)

            moved = [s for s in backend.readbacks if any(pulse != NEUTRAL_US for pulse in s.pwms)]
            self.assertFalse(
                moved,
                "a latched board must hold neutral, but sim_time_ms="
                f"{moved[0].sim_time_ms} showed {list(moved[0].pwms)}"
                if moved
                else "",
            )
        _assert_engine_reaped(self, run)


if __name__ == "__main__":
    unittest.main()
