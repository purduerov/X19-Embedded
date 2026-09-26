"""
Host-side tests for the Streamlit SIL dashboard control worker.

The dashboard used to emit exactly one PWM frame per operator action.  Node 2's
100 ms heartbeat watchdog then expired and forced the outputs back to neutral,
so sustained thrust was impossible.  ``SilDashboardClient`` now owns an explicit
deadman state machine and resends the last accepted command at 20 Hz, which is
what these tests pin.

Everything here drives the *real* native ``sil_bridge_server`` over the real SIL
TCP framing, so an assertion is a statement about native C firmware behavior and
not about a Python model.  Tests that need the binary are skipped when it has
not been built.

Run from the repository root::

    python -m unittest tests.sil_stimulus.test_dashboard_control -v
"""

import socket
import sys
import threading
import time
import unittest
from pathlib import Path

try:
    from .sil_test_support import (
        REPO_ROOT,
        SilServerProcess,
        free_tcp_port,
        require_server_executable,
    )
except ImportError:  # pragma: no cover - direct execution fallback
    from sil_test_support import (  # type: ignore[no-redef]
        REPO_ROOT,
        SilServerProcess,
        free_tcp_port,
        require_server_executable,
    )

# tests/sil_stimulus/test_dashboard_control.py -> tests/sil_stimulus -> tests
DASHBOARD_DIR = REPO_ROOT / "tests" / "sil_dashboard"
if str(DASHBOARD_DIR) not in sys.path:
    sys.path.insert(0, str(DASHBOARD_DIR))

import sil_dashboard_client  # noqa: E402  (path setup must precede the import)
from sil_dashboard_client import ControlState, SilDashboardClient  # noqa: E402

# Mirrors the firmware's ROV_PWM_STOP_US / ROV_PWM_MIN_US / ROV_PWM_MAX_US and
# ROV_HEARTBEAT_TIMEOUT_MS in shared/include/rov_parameters.h.
NEUTRAL_US = 1500
PWM_MIN_US = 1000
PWM_MAX_US = 2000
NEUTRAL_PWMS = [NEUTRAL_US] * 8
HEARTBEAT_TIMEOUT_S = 0.100
CONTROL_PERIOD_S = 0.050
# Semantic upper bound on the mean control period, expressed against the firmware
# watchdog budget rather than against CONTROL_PERIOD_S.  Node 2 forces neutral
# when the newest thruster command is older than HEARTBEAT_TIMEOUT_S, so the
# period has to fit inside that budget with margin.  Half the window would be the
# obvious choice, but it happens to equal CONTROL_PERIOD_S exactly, which makes
# the bound degenerate (a correct loop sits precisely on it).  Three quarters
# says what actually matters: a correct 50 ms period spends half the budget and
# keeps a 50% safety factor, and any loop drifting past 75 ms is unambiguously
# broken.
SEMANTIC_MEAN_GAP_MAX_S = HEARTBEAT_TIMEOUT_S * 0.75
# Node 2 refuses thruster commands until ESC_ARMING_TIME_MS of virtual time has
# elapsed (nodes/node2_control_board/Core/Src/app.c).
ESC_ARMING_SIM_MS = 3000
SIM_MS_PER_WALL_S = 0.8  # generous lower bound; the C engine sleeps 10 ms per 10 ms tick


class _ThrusterSendSpy:
    """
    Records every thruster frame the client transmits, with its wall-clock time.

    Wrapping ``_send_thruster_frame`` measures the worker's true cadence
    without adding test-only counters to the production class.  The timestamp
    is taken when the send is *initiated*, not when it completes, so an injected
    send cost correctly shows up as gap rather than hiding inside it.
    """

    def __init__(self, client: SilDashboardClient) -> None:
        self._client = client
        # Captured before the attribute is replaced, so restore() can put the
        # real bound method back.
        self._original = client._send_thruster_frame
        self.delegate = self._original
        # [start_time, duration_or_None] pairs. The slot is published under the
        # lock before the send runs, so a reader can never observe a start with
        # no matching entry, and the duration is filled in afterwards.
        self.entries: list = []
        self.times: list = []
        self.payloads: list = []
        self.lock = threading.Lock()

        def spy(pwms):
            entry = [time.monotonic(), None]
            with self.lock:
                self.entries.append(entry)
                self.times.append(entry[0])
                self.payloads.append(list(pwms))
            try:
                return self.delegate(pwms)
            finally:
                with self.lock:
                    entry[1] = time.monotonic() - entry[0]

        client._send_thruster_frame = spy

    def completed_entries(self):
        """Entries whose measured duration is known, in send order."""
        with self.lock:
            return [list(entry) for entry in self.entries if entry[1] is not None]

    def restore(self) -> None:
        """Put the real ``_send_thruster_frame`` back on the client."""
        self.delegate = self._original
        self._client._send_thruster_frame = self._original

    def snapshot(self):
        with self.lock:
            return list(self.times), list(self.payloads)

    def count(self) -> int:
        with self.lock:
            return len(self.times)

    def wait_for_count(self, minimum: int, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.count() >= minimum:
                return True
            time.sleep(0.005)
        return self.count() >= minimum

    def quiet_for(self, duration_s: float) -> bool:
        """True when no further frame is transmitted for ``duration_s``."""
        before = self.count()
        time.sleep(duration_s)
        return self.count() == before

    def inject_send_latency(self, *costs: float):
        """
        Context manager making successive sends cost ``costs[i % len]`` seconds.

        A 73-byte loopback send takes microseconds, so a loop paced by
        ``Event.wait(0.05)`` is indistinguishable from one paced by a monotonic
        deadline on an idle host.  Real hosts are not idle: the GIL is contended
        by the ~200 Hz telemetry reader and a busy link stretches the write.

        Passing more than one cost makes the send time *alternate*, which is
        what makes the pacing contract observable and the check immune to
        scheduler noise: a deadline loop holds the period steady through a cheap
        send and an expensive one alike, while a bare wait adds the whole send
        duration to the period every time.  Both cost classes see the same
        scheduling jitter, so comparing their means cancels it.

        The cost is injected with ``time.sleep``, not a busy loop, because a
        real blocked socket write releases the GIL.  Burning CPU here would
        starve the telemetry reader and measure GIL contention rather than the
        pacing logic.
        """
        spy = self
        previous = self.delegate

        class _Latency:
            def __enter__(self_inner):
                counter = {"n": 0}

                def variable(pwms):
                    index = counter["n"]
                    counter["n"] += 1
                    time.sleep(costs[index % len(costs)])  # a blocked write
                    return previous(pwms)

                spy.delegate = variable
                return self_inner

            def __exit__(self_inner, exc_type, exc, tb):
                spy.delegate = previous

        return _Latency()


class TestDashboardControlStateMachine(unittest.TestCase):
    """
    The state machine itself, including argument validation.

    These need no native binary: a client that was never connected is still
    required to reject unsafe commands and to report a coherent state.
    """

    def setUp(self) -> None:
        self.client = SilDashboardClient(port=1)
        self.addCleanup(self.client.disconnect)

    def test_initial_state_is_disconnected(self):
        self.assertIs(self.client.control_state, ControlState.DISCONNECTED)

    def test_connect_failure_leaves_state_disconnected(self):
        # Port 1 is not served; connect() must fail closed.
        self.assertFalse(self.client.connect())
        self.assertIs(self.client.control_state, ControlState.DISCONNECTED)

    def test_start_control_loop_requires_a_connected_transport(self):
        self.assertFalse(self.client.start_control_loop())
        self.assertIs(self.client.control_state, ControlState.DISCONNECTED)
        self.assertIsNone(self.client._control_thread)

    def test_send_pwms_requires_exactly_eight_channels(self):
        with self.assertRaises(ValueError):
            self.client.send_pwms([1650] * 7)
        with self.assertRaises(ValueError):
            self.client.send_pwms([1650] * 9)

    def test_send_pwms_rejects_out_of_range_values(self):
        for bad in (PWM_MIN_US - 1, PWM_MAX_US + 1, 0, -1650):
            with self.subTest(pwm=bad):
                with self.assertRaises(ValueError):
                    self.client.send_pwms([bad] * 8)

    def test_send_pwms_rejects_non_finite_and_fractional_values(self):
        for bad in (float("nan"), float("inf"), 1500.5):
            with self.subTest(pwm=bad):
                with self.assertRaises(ValueError):
                    self.client.send_pwms([bad] * 8)

    def test_send_pwms_rejects_a_non_numeric_command_with_typeerror(self):
        """
        Invariant: the failure modes follow the usual Python split, so a caller
        bug (wrong type) is distinguishable from a bad value.
        """
        for bad in (None, "1500,1500,1500,1500,1500,1500,1500,1500", 1650, 3.5):
            with self.subTest(command=bad):
                with self.assertRaises(TypeError):
                    self.client.send_pwms(bad)
        for bad in ([None] * 8, ["1650"] * 8, [True] * 8, [object()] * 8):
            with self.subTest(command=bad[0]):
                with self.assertRaises(TypeError):
                    self.client.send_pwms(bad)

    def test_rejected_command_leaves_the_worker_command_untouched(self):
        out_of_range = [1650] * 8
        out_of_range[3] = PWM_MAX_US + 1
        with self.assertRaises(ValueError):
            self.client.send_pwms(out_of_range)
        self.assertEqual(self.client._last_command, NEUTRAL_PWMS)
        self.assertEqual(self.client.pwms, NEUTRAL_PWMS)
        self.assertIs(self.client.control_state, ControlState.DISCONNECTED)

    def test_stop_control_loop_is_idempotent_while_disconnected(self):
        self.assertTrue(self.client.stop_control_loop())
        self.assertTrue(self.client.stop_control_loop())
        # A stop on a client with no transport is honest about it: the state
        # stays DISCONNECTED rather than advertising a live STOPPED control path.
        self.assertIs(self.client.control_state, ControlState.DISCONNECTED)
        self.assertEqual(self.client._last_command, NEUTRAL_PWMS)

    def test_request_emergency_break_while_disconnected_only_latches(self):
        self.assertFalse(self.client.request_emergency_break())
        self.assertIs(self.client.control_state, ControlState.ESTOP)
        self.assertIsNone(self.client._control_thread)


class TestSolenoidInterlock(unittest.TestCase):
    """
    Invariant: the UI cannot ask for a mask the firmware rejects wholesale.

    ``shared/src/rov_can_protocol.c:120-126`` walks the five valves and, for any
    valve with BOTH coils energised, zeroes the whole mask and returns
    ``ROV_ERR_INVALID_ARG``; ``nodes/node2_control_board/Core/Src/app.c:150-153``
    then skips ``bsp_solenoid_set`` entirely, so the outputs keep whatever mask
    was already there. ``tools/can_stimulus.py`` refuses such a mask and explains
    exactly that, and the dashboard's ten independent toggles could produce one
    anyway: valve 0 Extend on (0x001), then valve 1 Retract (0x007) energises both
    coils of valve 1, the firmware zeroes the mask, and valve 0 silently drops too
    while the UI records 0x007 as the command target.

    The rule now lives with the code that packs 0x110, so the tool and the UI
    cannot hold two copies of a firmware decision that will drift.
    """

    def test_the_tool_and_the_client_share_one_interlock_rule(self):
        from tools import can_stimulus  # noqa: PLC0415 - only needed by this test

        self.assertIs(
            can_stimulus.conflicting_solenoid_valve,
            sil_dashboard_client.conflicting_solenoid_valve,
            "one firmware rule, one implementation: a second copy would drift and "
            "the drift would be invisible",
        )

    def test_a_conflicting_mask_is_named_rather_than_silently_masked(self):
        # 0x007 is bits 0,1,2: valve 0's pair is fully energised, so valve 0 is the
        # FIRST conflict even though valve 1 also has a coil lit. 0x002 is one coil
        # of valve 0 and is therefore legal.
        for mask, valve in ((0x007, 0), (0x003, 0), (0x00C, 1), (0x3FF, 0), (0x155, None), (0x002, None)):
            with self.subTest(mask=f"0x{mask:03X}"):
                self.assertEqual(
                    sil_dashboard_client.conflicting_solenoid_valve(mask),
                    valve,
                )

    def _armed_client(self):
        """A client that believes it is connected, with a real (unconnected) socket."""
        client = SilDashboardClient()
        client.connected = True
        client.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.addCleanup(client.disconnect)
        return client

    def test_send_solenoids_refuses_a_mask_that_energises_both_coils(self):
        client = self._armed_client()
        written = []
        client._send_frame = lambda *args, **kwargs: written.append(args[1])

        self.assertFalse(
            client.send_solenoids(0x007),
            "a mask the firmware rejects wholesale must not be reported as sent",
        )
        self.assertEqual(written, [], "the refusal has to come before the write")
        self.assertEqual(
            client.solenoid_mask,
            0x000,
            "a refused mask must not become the recorded command target; the board "
            "never applied it",
        )
        self.assertIn("both coils of valve 0", client.last_solenoid_refusal)
        self.assertIn("rov_can_protocol.c:120-126", client.last_solenoid_refusal)

    def test_a_valid_mask_still_goes_out_unchanged(self):
        client = self._armed_client()
        written = []
        client._send_frame = lambda *args, **kwargs: written.append((args[1], args[2]))

        self.assertTrue(client.send_solenoids(0x0155))
        self.assertEqual([can_id for can_id, _ in written], [sil_dashboard_client.CAN_ID_SOLENOID_CMD])
        self.assertEqual(client.solenoid_mask, 0x155)
        self.assertIsNone(client.last_solenoid_refusal)


class TestDashboardControlAgainstServer(unittest.TestCase):
    """
    End-to-end control-path tests against the native SIL server.

    Skipped when the native binary has not been built; a skip is honest here,
    a silent pass would not be.
    """

    @classmethod
    def setUpClass(cls) -> None:
        cls.server_exe = require_server_executable()

    def setUp(self) -> None:
        self.server = SilServerProcess(self.server_exe)
        self.server.start()
        self.addCleanup(self.server.stop)

        self.client = SilDashboardClient(port=self.server.port)
        self.assertTrue(self.client.connect(), "Dashboard client must reach the SIL server")
        self.addCleanup(self.client.stop_server_process)
        self.spy = _ThrusterSendSpy(self.client)
        self.addCleanup(self.spy.restore)

    def _wait_for(self, predicate, timeout_s: float, what: str) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.02)
        self.fail(f"Timed out after {timeout_s:.1f} s waiting for {what}; "
                  f"control_state={self.client.control_state}, "
                  f"actual_pwms={self.client.actual_pwms}, sim_time_ms={self.client.sim_time_ms}")

    def _wait_until_esc_armed(self) -> None:
        """Block until the board leaves its mandatory 3000 ms arming period."""
        self._wait_for(
            lambda: self.client.sim_time_ms >= ESC_ARMING_SIM_MS,
            timeout_s=ESC_ARMING_SIM_MS / 1000.0 / SIM_MS_PER_WALL_S + 6.0,
            what="the control board to finish ESC arming",
        )

    def _wait_for_neutral(self, timeout_s: float) -> None:
        self._wait_for(
            lambda: self.client.actual_pwms == NEUTRAL_PWMS,
            timeout_s=timeout_s,
            what="the board outputs to return to neutral",
        )

    def assert_worker_stopped(self) -> None:
        """No control worker may still be scheduled after a stop transition."""
        thread = self.client._control_thread
        self.assertTrue(
            thread is None or not thread.is_alive(),
            "The 20 Hz control worker is still running after a stop transition",
        )

    def _wait_for_live_neutral_status(self, timeout_s: float) -> None:
        """Require a *fresh* neutral snapshot, not merely a stale neutral value."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._wait_for(
                lambda: self.client.actual_pwms == NEUTRAL_PWMS
                and self.client.output_status_received_monotonic is not None
                and time.monotonic() - self.client.output_status_received_monotonic < 0.2,
                timeout_s=max(0.05, deadline - time.monotonic()),
                what="a fresh neutral output-status snapshot",
            )
            return

    # ------------------------------------------------------------------
    # Sustained control
    # ------------------------------------------------------------------

    def test_command_is_resent_until_stop(self):
        """
        Invariant: a non-neutral command survives far longer than the firmware
        heartbeat window, then returns to neutral on an explicit stop.
        """
        self._wait_until_esc_armed()

        self.client.send_pwms([1650] * 8)
        self.assertTrue(self.client.start_control_loop(), "start_control_loop must be idempotent-True")
        self.assertIs(self.client.control_state, ControlState.RUNNING)

        # Let the 1 kHz slew ramp reach the commanded value.
        self._wait_for(
            lambda: max(self.client.actual_pwms) > NEUTRAL_US,
            timeout_s=2.0,
            what="the thruster outputs to leave neutral",
        )

        # Observe for longer than the 100 ms heartbeat window: a single-shot
        # command would already have decayed back to neutral by now.
        self._wait_for(
            lambda: self.client.actual_pwms == [1650] * 8,
            timeout_s=2.0,
            what="the outputs to reach the commanded 1650 us",
        )
        samples = []
        observation_deadline = time.monotonic() + 0.60
        while time.monotonic() < observation_deadline:
            samples.append(list(self.client.actual_pwms))
            time.sleep(0.02)
        self.assertTrue(samples, "the sustained observation window must not be empty")
        self.assertTrue(
            all(min(sample) > NEUTRAL_US for sample in samples),
            f"A single-shot command decays to neutral after 100 ms; "
            f"minimum observed channel was {min(min(s) for s in samples)} us",
        )
        self.assertEqual(self.client.actual_pwms, [1650] * 8)

        self.client.request_stop()
        self.assertIs(self.client.control_state, ControlState.STOPPED)
        self._wait_for_neutral(timeout_s=1.0)
        self.assertEqual(self.client.actual_pwms, NEUTRAL_PWMS)

    def test_control_worker_holds_20hz_over_a_two_second_window(self):
        """
        Invariant: the worker is paced by a monotonic deadline, not by a bare
        sleep, so the observed rate stays at 20 Hz and no gap ever reaches the
        100 ms firmware heartbeat window while the transport is loaded.

        A NON-neutral command is used deliberately: 20 Hz resend is only correct
        while the operator is actually commanding thrust.  An all-neutral
        command must stop the worker so the watchdog lapses, which
        ``test_all_neutral_command_arms_and_stops_the_worker`` pins.
        """
        window_s = 2.0
        self.client.send_pwms([1600] * 8)
        self.assertTrue(self.client.start_control_loop())
        self.assertIs(self.client.control_state, ControlState.RUNNING)

        # Discard the first frame so the window measures the worker's own cadence.
        self.assertTrue(self.spy.wait_for_count(1, timeout_s=1.0))
        start = self.spy.snapshot()[0][-1]

        time.sleep(window_s)

        times, _ = self.spy.snapshot()
        observed = [t for t in times if start < t <= start + window_s + CONTROL_PERIOD_S]
        gaps = [b - a for a, b in zip([start] + observed, observed)]

        self.assertGreaterEqual(
            len(observed), int(window_s * 20 * 0.8),
            f"Worker must sustain ~20 Hz; only {len(observed)} frames in {window_s} s. gaps={[round(g, 4) for g in gaps]}",
        )
        self.assertLessEqual(
            len(observed), int(window_s * 20 * 1.2) + 2,
            f"Worker must not over-send; {len(observed)} frames in {window_s} s. gaps={[round(g, 4) for g in gaps]}",
        )
        self.assertLessEqual(
            max(gaps), HEARTBEAT_TIMEOUT_S,
            f"Every gap must stay inside the {HEARTBEAT_TIMEOUT_S * 1000:.0f} ms firmware "
            f"heartbeat window; worst gap was {max(gaps) * 1000:.1f} ms. "
            f"gaps={[round(g, 4) for g in gaps]}",
        )
        mean_gap = sum(gaps) / len(gaps)
        # Semantic bound first, then the tight band.  Mean gap is the number the
        # firmware actually cares about: the watchdog compares the newest command
        # timestamp against now, so the mean is the margin that keeps a healthy
        # heartbeat alive with twice the timeout to spare.  The tight band then
        # says the same thing precisely and catches per-iteration drift the mean
        # would average away.
        self.assertLess(
            mean_gap, SEMANTIC_MEAN_GAP_MAX_S,
            f"Mean gap {mean_gap * 1000:.1f} ms must fit inside the "
            f"{HEARTBEAT_TIMEOUT_S * 1000:.0f} ms firmware watchdog budget with margin "
            f"(limit {SEMANTIC_MEAN_GAP_MAX_S * 1000:.0f} ms), so a single slow tick "
            f"cannot expire the heartbeat. gaps={[round(g, 4) for g in gaps]}",
        )
        self.assertAlmostEqual(
            mean_gap, CONTROL_PERIOD_S, delta=0.006,
            msg=(f"Mean gap drifted to {mean_gap * 1000:.2f} ms instead of "
                 f"{CONTROL_PERIOD_S * 1000:.0f} ms, which means the period accumulates "
                 f"the send duration every iteration. gaps={[round(g, 4) for g in gaps]}"),
        )

    def test_control_worker_period_does_not_track_the_send_cost(self):
        """
        Invariant: the 50 ms period is measured from a monotonic deadline, so a
        slow send is absorbed by the remaining budget instead of being added to
        every period.

        This is the assertion that separates a real deadline loop from a bare
        ``Event.wait(0.05)``.  The distinction is deliberately *not* the mean
        rate: a bare wait self-corrects through its overrun branch, so both
        implementations hold 20 Hz on average and a rate-only check passes for
        either.  What separates them is that a bare wait pays the whole send
        duration on top of the period, so its period grows with the send cost.

        The send time therefore alternates between a cheap and an expensive
        frame.  A deadline loop reports the same mean period for both; a bare
        wait reports a period that is ~26 ms longer for the expensive frames.
        Because both classes see the same scheduler jitter, comparing their
        means cancels that noise, which keeps the check stable on a loaded
        host instead of tripping on an unrelated garbage-collection pause.
        """
        cheap_s, expensive_s = 0.004, 0.020
        window_s = 2.0

        # Non-neutral on purpose: the 20 Hz resend is only correct while the
        # operator is commanding thrust.
        self.client.send_pwms([1600] * 8)
        with self.spy.inject_send_latency(cheap_s, expensive_s):
            self.assertTrue(self.client.start_control_loop())
            # Discard the immediate send_pwms frame and the worker's first tick,
            # which legitimately land on top of each other.
            self.assertTrue(self.spy.wait_for_count(3, timeout_s=1.0))
            start = self.spy.snapshot()[0][-1]
            time.sleep(window_s)

        entries = self.spy.completed_entries()

        # Gap after each send, tagged with the measured cost of that send.
        cheap_gaps, expensive_gaps, all_gaps = [], [], []
        previous_time = start
        for stamp, duration in entries:
            if not start < stamp <= start + window_s + CONTROL_PERIOD_S:
                continue
            gap = stamp - previous_time
            previous_time = stamp
            all_gaps.append(gap)
            if duration < (cheap_s + expensive_s) / 2.0:
                cheap_gaps.append(gap)
            else:
                expensive_gaps.append(gap)

        self.assertGreater(len(cheap_gaps), 5, "not enough cheap samples to judge the cadence")
        self.assertGreater(len(expensive_gaps), 5, "not enough expensive samples to judge the cadence")

        cheap_mean = sum(cheap_gaps) / len(cheap_gaps)
        expensive_mean = sum(expensive_gaps) / len(expensive_gaps)
        skew = expensive_mean - cheap_mean
        self.assertLess(
            abs(skew), 0.008,
            f"The period tracks the send cost: {cheap_mean * 1000:.1f} ms after a "
            f"{cheap_s * 1000:.0f} ms send but {expensive_mean * 1000:.1f} ms after a "
            f"{expensive_s * 1000:.0f} ms send (skew {skew * 1000:.1f} ms). A deadline-paced "
            f"loop waits only the budget the send left over; a bare Event.wait adds the "
            f"whole send duration to every period. "
            f"cheap={[round(g, 4) for g in cheap_gaps]} expensive={[round(g, 4) for g in expensive_gaps]}",
        )
        for label, mean in (("cheap", cheap_mean), ("expensive", expensive_mean)):
            self.assertAlmostEqual(
                mean, CONTROL_PERIOD_S, delta=0.006,
                msg=f"Mean {label} period drifted to {mean * 1000:.2f} ms",
            )
        overall_mean = sum(all_gaps) / len(all_gaps)
        self.assertLess(
            overall_mean, SEMANTIC_MEAN_GAP_MAX_S,
            f"Overall mean period {overall_mean * 1000:.1f} ms must fit inside the "
            f"{HEARTBEAT_TIMEOUT_S * 1000:.0f} ms firmware watchdog budget with margin "
            f"(limit {SEMANTIC_MEAN_GAP_MAX_S * 1000:.0f} ms). "
            f"cheap={cheap_mean * 1000:.1f} ms expensive={expensive_mean * 1000:.1f} ms",
        )
        self.assertLessEqual(
            max(all_gaps), HEARTBEAT_TIMEOUT_S,
            f"A drifting period eventually crosses the 100 ms firmware heartbeat window; "
            f"worst gap was {max(all_gaps) * 1000:.1f} ms.",
        )

    def test_all_neutral_command_arms_and_stops_the_worker(self):
        """
        Invariant: an all-neutral command is the operator's All Stop. It must
        leave the control path ARMED with no worker, and it must stop emitting
        frames so the firmware heartbeat lapses.

        This is the safety ruling, not a cosmetic one.  ``node2_force_neutral()``
        calls ``bsp_solenoid_set(0)`` and is invoked on ``heartbeat_lost``
        (nodes/node2_control_board/Core/Src/app.c), so the heartbeat is the
        only thing that releases the solenoids.  A 20 Hz resend of a neutral
        command would refresh that heartbeat forever and hold every solenoid
        energized behind a dashboard that claims to be running.
        """
        self.client.send_pwms([1650] * 8)
        self.assertIs(self.client.control_state, ControlState.RUNNING)
        self.assertIsNotNone(self.client._control_thread)
        self.assertTrue(self.client._control_thread.is_alive())
        self.assertTrue(self.spy.wait_for_count(1, timeout_s=1.0))
        # Let the worker prove it really is transmitting at 20 Hz.
        self.assertTrue(self.spy.wait_for_count(6, timeout_s=1.0))

        self.assertTrue(self.client.send_pwms(NEUTRAL_PWMS))

        self.assertIs(
            self.client.control_state, ControlState.ARMED,
            "An all-neutral command must ARM, not RUN: a running worker would keep "
            "refreshing the firmware heartbeat and never release the solenoids",
        )
        self.assertIsNone(self.client._control_thread, "The 20 Hz worker must be stopped by All Stop")
        self.assertEqual(self.client._last_command, NEUTRAL_PWMS)

        # The only frame this may put on the wire is the one confirming neutral
        # inside stop_control_loop; after that the worker must be silent so the
        # watchdog can lapse. 0.20 s is four control periods.
        self.assertTrue(
            self.spy.quiet_for(0.20),
            "A neutral command must stop the 20 Hz cadence so the firmware heartbeat "
            "lapses and node2_force_neutral() releases the solenoids",
        )
        _, payloads = self.spy.snapshot()
        self.assertEqual(payloads[-1], NEUTRAL_PWMS, "The last frame on the wire must be neutral")

        # ARMED is not terminal: a new non-neutral command re-arms RUNNING.
        self.assertTrue(self.client.send_pwms([1600] * 8))
        self.assertIs(self.client.control_state, ControlState.RUNNING)
        self.assertIsNotNone(self.client._control_thread)
        self.assertTrue(self.client._control_thread.is_alive())

    def test_all_neutral_pilot_command_also_stops_the_worker(self):
        """
        Invariant: the same ruling holds for the pilot adapter. Centering the
        stick maps to all-neutral, so it must stop the worker exactly as the
        explicit All Stop button does.
        """
        self.client.send_surface_pilot_command(0.5, 0.0, 0.0, 0.0)
        self.assertIs(self.client.control_state, ControlState.RUNNING)
        self.assertIsNotNone(self.client._control_thread)
        self.assertTrue(self.client._control_thread.is_alive())

        self.client.send_surface_pilot_command(0.0, 0.0, 0.0, 0.0)
        self.assertEqual(self.client.pwms, NEUTRAL_PWMS)
        self.assertIs(
            self.client.control_state, ControlState.ARMED,
            "Centering the pilot stick must ARM and stop the worker, not keep "
            "refreshing the firmware heartbeat",
        )
        self.assertIsNone(self.client._control_thread)
        self.assertTrue(self.spy.quiet_for(0.20), "A centered stick must silence the 20 Hz cadence")

    def test_start_control_loop_is_idempotent(self):
        """Invariant: a second start call must not spawn a second worker."""
        self.client.send_pwms([1650] * 8)
        first = self.client._control_thread
        self.assertIsNotNone(first)
        self.assertTrue(first.is_alive())

        self.assertTrue(self.client.start_control_loop())
        self.assertIs(self.client._control_thread, first, "start_control_loop must reuse the running worker")

        baseline = threading.active_count()
        for _ in range(5):
            self.assertTrue(self.client.start_control_loop())
        self.assertLessEqual(
            threading.active_count(), baseline,
            "Repeated start_control_loop calls must not create new threads",
        )

        # A doubled worker would transmit at 40 Hz; confirm the rate is still 20.
        self.assertTrue(self.spy.wait_for_count(1, timeout_s=1.0))
        start = self.spy.snapshot()[0][-1]
        time.sleep(1.0)
        times, _ = self.spy.snapshot()
        observed = [t for t in times if start < t <= start + 1.05]
        self.assertLessEqual(len(observed), 24, f"A duplicate worker doubled the send rate: {len(observed)} frames in 1 s")

    def test_pilot_command_entry_point_sustains_thrust(self):
        """
        Invariant: the Streamlit pilot adapter still works as a deadman
        command source rather than a one-shot.
        """
        self._wait_until_esc_armed()
        self.client.send_surface_pilot_command(0.5, 0.0, 0.0, 0.0)
        self.assertIs(self.client.control_state, ControlState.RUNNING)
        self.assertTrue(all(PWM_MIN_US <= pwm <= PWM_MAX_US for pwm in self.client.pwms))

        self._wait_for(
            lambda: max(self.client.actual_pwms) > NEUTRAL_US,
            timeout_s=2.0,
            what="the pilot command to reach the board",
        )
        self._wait_for(
            lambda: self.client.actual_pwms == self.client.pwms,
            timeout_s=2.0,
            what="the outputs to match the pilot allocation",
        )
        time.sleep(0.40)  # > 100 ms heartbeat window
        self.assertEqual(
            self.client.actual_pwms, self.client.pwms,
            "The pilot command must persist past the 100 ms heartbeat window",
        )

    # ------------------------------------------------------------------
    # Fail-safe transitions
    # ------------------------------------------------------------------

    def test_socket_error_in_the_worker_forces_neutral_and_stops_transmitting(self):
        """
        Invariant: a transport error stops transmission immediately. The
        firmware watchdog, not a neutral frame this process can no longer send,
        is what guarantees the outputs go neutral.
        """
        self._wait_until_esc_armed()
        self.client.send_pwms([1650] * 8)
        self._wait_for(
            lambda: max(self.client.actual_pwms) > NEUTRAL_US,
            timeout_s=2.0,
            what="the thruster outputs to leave neutral",
        )

        original = self.client._send_thruster_frame
        boom = {"count": 0}

        def failing(_pwms):
            boom["count"] += 1
            raise OSError("simulated transport failure")

        self.client._send_thruster_frame = failing
        try:
            self.assertTrue(self.spy.wait_for_count(1, timeout_s=1.0))
            self._wait_for(
                lambda: self.client.control_state is ControlState.DISCONNECTED,
                timeout_s=1.0,
                what="the worker to fault and drop the transport",
            )
        finally:
            self.client._send_thruster_frame = original

        self.assertFalse(self.client.connected, "A transport error must mark the client disconnected")
        self.assertEqual(self.client._last_command, NEUTRAL_PWMS, "The worker command must be forced neutral")
        self.assert_worker_stopped()
        # The OSError path deliberately skips the neutral frame: the socket is
        # already gone, so the worker must not retry against a dead transport.
        # Bringing the outputs back to neutral is the firmware watchdog's job.
        self.assertEqual(boom["count"], 1, "Exactly one failed send, no retry storm")
        self.assertTrue(
            self.spy.quiet_for(0.30),
            "A faulted worker must stop transmitting instead of retrying a dead socket",
        )

    def test_unexpected_worker_exception_forces_neutral(self):
        """Invariant: a non-OSError worker fault still drives outputs to neutral."""
        self._wait_until_esc_armed()
        self.client.send_pwms([1650] * 8)
        self._wait_for(
            lambda: max(self.client.actual_pwms) > NEUTRAL_US,
            timeout_s=2.0,
            what="the thruster outputs to leave neutral",
        )

        original = self.client._send_thruster_frame
        sent = {"n": 0}

        def flaky(_pwms):
            sent["n"] += 1
            if sent["n"] == 2:
                raise RuntimeError("simulated worker fault")
            return original(_pwms)

        self.client._send_thruster_frame = flaky
        self._wait_for(
            lambda: self.client.control_state is ControlState.STOPPED,
            timeout_s=1.0,
            what="the worker to fault into STOPPED",
        )
        self.client._send_thruster_frame = original

        self.assertTrue(self.client.connected, "A worker bug is not a transport failure")
        self.assert_worker_stopped()
        _, payloads = self.spy.snapshot()
        self.assertTrue(
            payloads[-1] == NEUTRAL_PWMS,
            f"An unexpected worker fault must emit a neutral frame; last sent was {payloads[-1]}",
        )
        self._wait_for_neutral(timeout_s=1.0)
        self.assertEqual(self.client.actual_pwms, NEUTRAL_PWMS)

    def test_disconnect_stops_transmission_and_the_watchdog_drops_to_neutral(self):
        """
        Invariant: after an explicit disconnect the worker stops transmitting,
        and the firmware heartbeat watchdog forces neutral within 100 ms.
        """
        self._wait_until_esc_armed()
        self.client.send_pwms([1650] * 8)
        self._wait_for(
            lambda: self.client.actual_pwms == [1650] * 8,
            timeout_s=2.0,
            what="the outputs to reach 1650 us",
        )

        self.client.disconnect()
        self.assertIs(self.client.control_state, ControlState.DISCONNECTED)
        self.assert_worker_stopped()
        self.assertTrue(
            self.spy.quiet_for(0.30),
            "No thruster frame may be transmitted after the transport is disconnected",
        )

        # Reconnect (which the operator does with the engine restart button) and
        # read back the board: the watchdog, not the dashboard, neutralized it.
        self.assertTrue(self.client.connect())
        self.assertIs(self.client.control_state, ControlState.ARMED)
        self._wait_for_neutral(timeout_s=2.0)
        self._wait_for_live_neutral_status(timeout_s=2.0)
        self.assertEqual(self.client.actual_pwms, NEUTRAL_PWMS)

    def test_stop_control_loop_never_sends_after_the_socket_is_closed(self):
        """Invariant: idempotent stop on a dead transport raises nothing and sends nothing."""
        self.client.send_pwms([1650] * 8)
        self.client.sock.close()
        self.client.sock = None
        self.client.connected = False

        self.assertTrue(self.client.stop_control_loop())
        self.assertTrue(self.client.stop_control_loop())
        self.assertIs(self.client.control_state, ControlState.DISCONNECTED)
        self.assertEqual(self.client._last_command, NEUTRAL_PWMS)
        _, payloads = self.spy.snapshot()
        self.assertEqual(payloads[-1], [1650] * 8, "No neutral frame may be attempted on a dead socket")

    def test_request_stop_is_idempotent_and_idle_safe(self):
        """Invariant: repeated stops keep the outputs neutral and transmit nothing new."""
        self._wait_until_esc_armed()
        self.client.send_pwms([1650] * 8)
        self._wait_for(
            lambda: self.client.actual_pwms == [1650] * 8,
            timeout_s=2.0,
            what="the outputs to reach 1650 us",
        )

        self.client.request_stop()
        self.assertIs(self.client.control_state, ControlState.STOPPED)
        self._wait_for_neutral(timeout_s=1.0)
        self.assert_worker_stopped()

        # Each explicit stop may re-assert one neutral frame, but once the
        # worker is joined nothing may keep the 20 Hz cadence alive.
        for _ in range(5):
            self.assertTrue(self.client.request_stop())
        self.assertIs(self.client.control_state, ControlState.STOPPED)
        self.assertTrue(
            self.spy.quiet_for(0.30),
            "A stopped worker must not keep transmitting neutral frames",
        )

    # ------------------------------------------------------------------
    # Emergency break
    # ------------------------------------------------------------------

    def test_emergency_break_latches_and_blocks_further_commands(self):
        """
        Invariant: the authorized 0xAA 0x55 signature trips the board, the state
        latches in ESTOP, and no later command can resume thrust.
        """
        self._wait_until_esc_armed()
        self.client.send_pwms([1650] * 8)
        self._wait_for(
            lambda: self.client.actual_pwms == [1650] * 8,
            timeout_s=2.0,
            what="the outputs to reach 1650 us",
        )

        self.assertTrue(self.client.trigger_emergency_break())
        self.assertIs(self.client.control_state, ControlState.ESTOP)
        self.assert_worker_stopped()

        # The firmware only latches on the 0xAA 0x55 signature, so a tripped
        # brake is proof the authorized frame really reached native C.
        self._wait_for(
            lambda: self.client.emergency_break_tripped,
            timeout_s=2.0,
            what="the board to latch the emergency brake",
        )
        self._wait_for_neutral(timeout_s=2.0)

        signature_records = [
            record
            for record in list(self.client.packet_log)
            if record.name == "EMERGENCY_BREAK" and record.direction == "Core -> STM32"
        ]
        self.assertTrue(signature_records, "The authorized emergency frame must appear in the packet log")
        self.assertTrue(
            signature_records[-1].hex_data.startswith("AA 55 01"),
            f"Emergency frame must be 0xAA 0x55 0x01; logged {signature_records[-1].hex_data}",
        )

        # Latched: nothing the operator does may resume thrust.
        baseline = self.spy.count()
        displayed_before = list(self.client.pwms)
        self.assertFalse(self.client.send_pwms([1800] * 8), "send_pwms must refuse while ESTOP is latched")
        self.assertFalse(self.client.start_control_loop(), "start_control_loop must refuse while latched")
        self.client.send_surface_pilot_command(1.0, 0.0, 0.0, 0.0)
        self.assertIs(self.client.control_state, ControlState.ESTOP, "ESTOP must not clear silently")
        self.assertEqual(self.client._last_command, NEUTRAL_PWMS)
        self.assertEqual(
            self.client.pwms, displayed_before,
            "A command refused while ESTOP is latched must be a complete no-op",
        )
        self.assertTrue(
            self.spy.quiet_for(0.30),
            "No thruster frame may be transmitted while ESTOP is latched",
        )
        self.assertGreaterEqual(self.spy.count(), baseline)
        self.assertEqual(self.client.actual_pwms, NEUTRAL_PWMS)

        # Only an explicit reconnect clears the latch.
        self.client.disconnect()
        self.assertTrue(self.client.connect())
        self.assertIs(self.client.control_state, ControlState.ARMED)
        self.assertTrue(self.client.send_pwms([1600] * 8), "A reconnect must re-arm the control path")

    def test_stop_after_emergency_break_cannot_downgrade_the_estop_latch(self):
        """
        Invariant: the ESTOP latch outranks every later stop transition.

        ``_transition_stopped`` is the single shared exit path, so an explicit
        ``request_stop()`` issued *after* an emergency break runs the same code
        as a worker fault would.  If the latch-preserving guard were dropped, a
        post-E-stop stop or socket error would rewrite ``control_state`` to
        STOPPED and the dashboard would present a tripped emergency break as a
        routine, already-handled stop.  That is the "looks safe again" failure,
        and it is why this is asserted directly rather than only incidentally.
        """
        self._wait_until_esc_armed()
        self.client.send_pwms([1650] * 8)
        self._wait_for(
            lambda: self.client.actual_pwms == [1650] * 8,
            timeout_s=2.0,
            what="the outputs to reach 1650 us",
        )

        self.assertTrue(self.client.trigger_emergency_break())
        self.assertIs(self.client.control_state, ControlState.ESTOP)
        self._wait_for(
            lambda: self.client.emergency_break_tripped,
            timeout_s=2.0,
            what="the board to latch the emergency brake",
        )
        frames_at_estop = self.spy.count()

        # The dangerous ordering: a stop arriving after the latch.
        self.assertTrue(self.client.request_stop())
        self.assertIs(
            self.client.control_state, ControlState.ESTOP,
            "An explicit stop after an emergency break must not downgrade ESTOP; "
            "otherwise a tripped brake is presented as a routine handled stop",
        )

        # stop_control_loop still does its job: one neutral frame, worker joined.
        self.assertEqual(self.client._last_command, NEUTRAL_PWMS)
        self.assert_worker_stopped()
        _, payloads = self.spy.snapshot()
        self.assertEqual(payloads[frames_at_estop:], [NEUTRAL_PWMS], "A post-E-stop stop may only re-assert neutral")
        self.assertTrue(
            all(p == NEUTRAL_PWMS for p in payloads[frames_at_estop:]),
            "No non-neutral frame may be emitted after the ESTOP latch",
        )

        # And the latch is still fully in force.
        self.assertFalse(self.client.start_control_loop())
        self.assertFalse(self.client.send_pwms([1700] * 8))
        self.assertIs(self.client.control_state, ControlState.ESTOP)
        self.assertTrue(self.spy.quiet_for(0.20))
        self._wait_for_neutral(timeout_s=1.0)


class TestDashboardEngineRestart(unittest.TestCase):
    """
    The engine-restart disarm path, driven through the real production code.

    Separate from ``TestDashboardControlAgainstServer`` because this class lets
    the client own the SIL server process itself (via
    ``start_server_process()``) instead of sharing one started by
    ``SilServerProcess``. Two servers cannot bind the same port, so the two
    fixtures are mutually exclusive.
    """

    @classmethod
    def setUpClass(cls) -> None:
        require_server_executable()

    def setUp(self) -> None:
        self.client = SilDashboardClient(port=free_tcp_port())
        self.addCleanup(self.client.stop_server_process)
        self.assertTrue(
            self.client.start_server_process(),
            "start_server_process() must launch the real sil_bridge_server",
        )
        self._connect()

    def _connect(self) -> None:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self.client.connect():
                return
            time.sleep(0.05)
        self.fail("The client never connected to the engine it started")

    def test_engine_restart_clears_the_estop_latch(self):
        """
        Invariant: ``start_server_process()`` is a real, independent disarm
        mechanism for the ESTOP latch, not just ``connect()`` in disguise.

        The dashboard's "Restart Engine" button calls stop_server_process() then
        start_server_process() then connect(), so both are live disarm paths and
        the latch must clear at the restart step itself.  The latch is observed
        (not assigned) between the restart and the reconnect, which is what
        distinguishes the two mechanisms.
        """
        self.assertTrue(self.client.trigger_emergency_break())
        self.assertIs(self.client.control_state, ControlState.ESTOP)
        self.assertFalse(
            self.client.send_pwms([1650] * 8),
            "A latched ESTOP must refuse commands before the restart",
        )
        self.assertTrue(
            self.client._estop_latched,
            "request_emergency_break() must set the latch",
        )

        # Exactly what the "Restart Engine" button does.
        self.client.stop_server_process()
        self.assertIs(
            self.client.control_state, ControlState.DISCONNECTED,
            "stop_server_process() disconnects but must NOT clear the latch",
        )
        self.assertTrue(
            self.client._estop_latched,
            "stop_server_process() must leave the ESTOP latch in place",
        )

        self.assertTrue(
            self.client.start_server_process(),
            "Restarting the engine must launch a new sil_bridge_server",
        )
        self.assertFalse(
            self.client._estop_latched,
            "start_server_process() is the operator's explicit restart and must "
            "clear the ESTOP latch by itself, before any reconnect",
        )

        self._connect()
        self.assertIs(self.client.control_state, ControlState.ARMED)
        self.assertTrue(
            self.client.send_pwms([1650] * 8),
            "After an engine restart the control path must accept commands again",
        )
        self.assertIs(self.client.control_state, ControlState.RUNNING)

    def test_engine_restart_is_required_to_rearm_after_estop(self):
        """
        Invariant: while the engine keeps running the latch holds, so a stop and
        reconnect without a restart does not silently hand thrust back.
        """
        self.assertTrue(self.client.trigger_emergency_break())
        self.assertIs(self.client.control_state, ControlState.ESTOP)

        self.client.disconnect()
        self.assertIs(self.client.control_state, ControlState.DISCONNECTED)
        self.assertTrue(
            self.client._estop_latched,
            "A bare disconnect must not clear the ESTOP latch; the operator has to "
            "restart the engine",
        )


if __name__ == "__main__":
    unittest.main()
