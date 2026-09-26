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
        status_code, printed = run_cli(["--mode", "sil", "--auto", "--node2-solenoid", "3"])
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
        Invariant: the id is validated like the payload.

        Without this, ``--raw-send -1:FF`` produced the check name
        ``raw_send_0x-1`` and a frame address nothing.
        """
        backend = FakeBackend()
        tester = VehicleStimulusTester(backend)
        for bad in (-1, 0x20000000, "0x100"):
            with self.subTest(can_id=bad):
                with self.assertRaises((ValueError, TypeError)):
                    tester.raw_send(bad, b"\x01")
        self.assertEqual(backend.sent, [])

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
            argv, backend_factory=lambda args: SilSocketBackend(args.host, port, auto_start=False)
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
                backend_factory=lambda args: SilSocketBackend(args.host, port, auto_start=False),
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
        tester = VehicleStimulusTester(SilSocketBackend("127.0.0.1", port, auto_start=False))
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


if __name__ == "__main__":
    unittest.main()
