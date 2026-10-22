"""
VehicleStimulusTester implementation for can_stimulus.
"""

from __future__ import annotations

import contextlib
import math
import sys
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

TOOLS_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = TOOLS_DIR.parent
SIL_BRIDGE_DIR = REPO_ROOT / "tests" / "sil_companion_bridge"
SIL_STIMULUS_DIR = REPO_ROOT / "tests" / "sil_stimulus"
SIL_DASHBOARD_DIR = REPO_ROOT / "tests" / "sil_dashboard"
for _path in (SIL_STIMULUS_DIR, SIL_BRIDGE_DIR, SIL_DASHBOARD_DIR):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from sil_protocol import (
    CAN_ID_EMERGENCY_BREAK,
    CAN_ID_ENV_TELEMETRY,
    CAN_ID_NAV_TELEMETRY,
    CAN_ID_POWER_TELEMETRY,
    CAN_ID_SIL_OUTPUT_STATUS,
    CAN_ID_SOLENOID_CMD,
    CAN_ID_THRUSTER_CMD,
    EnvTelemetry,
    NavTelemetry,
    PowerTelemetry,
    SolenoidCommand,
    ThrusterCommand,
    unpack_sil_can_frame,
)
from sil_dashboard_client import conflicting_solenoid_valve

from .constants import (
    COMMAND_PERIOD_S,
    DEFAULT_RECV_TIMEOUT_S,
    EMERGENCY_FRAME,
    EMERGENCY_TIMEOUT_S,
    ESC_ARMING_SIM_MS,
    ESTOP_LATCH_WINDOW_S,
    FIRST_OUTPUT_TIMEOUT_S,
    HEARTBEAT_TIMEOUT_S,
    MIN_ARMING_SAMPLES,
    MIN_TELEMETRY_HZ,
    NEUTRAL_PWMS,
    NEUTRAL_TIMEOUT_S,
    NEUTRAL_US,
    NUM_THRUSTERS,
    POST_ESTOP_PROBE_PWM_US,
    POWER_TELEMETRY_SIM_PERIOD_MS,
    PWM_MAX_US,
    PWM_MIN_US,
    PWM_SLEW_RATE_US_PER_MS,
    RAMP_TIMEOUT_S,
    READ_POLL_S,
    SAFETY_SIGNATURES,
    SIM_MS_PER_WALL_S,
    SIM_WALL_SLACK_S,
    SMOKE_ALL_PWM_US,
    SMOKE_SINGLE_PWM_US,
    SMOKE_SOLENOID_MASK,
    SOLENOID_MASK_MAX,
    SOLENOID_OBSERVE_TIMEOUT_S,
    UNAUTHORIZED_FRAME,
    UNAUTHORIZED_FRAME_WINDOW_S,
)
from .models import CheckResult, OutputStatus, StimulusReport, energised_coils
from .protocol import (
    _validate_channel,
    _validate_pulse_us,
    describe_can_id,
    validate_can_id,
    validate_payload,
)
from .backends import PythonCanBackend, UartBackend, _BaseBackend


class VehicleStimulusTester:
    """
    Named checks that each answer one question about the vehicle.

    Every method returns a ``CheckResult``.  Observation failures (timeout, decode
    error, wrong signature, missing readback) are returned as failures with the
    evidence in ``detail``; they are never raised.  Argument errors *are* raised,
    because a caller bug is not an observation and must not be reported as a
    vehicle failure.

    Two invariants are enforced here rather than left to each check:

    * a check that needs the 0x7FE readback refuses **before it writes anything**
      on a transport that cannot supply one, so a non-SIL mode never puts an
      unverifiable actuation on a bus (:meth:`_require_output_readback`);
    * a check that claims a precondition it did not observe fails instead of
      quietly dropping the claim (:meth:`test_emergency_break`).
    """

    def __init__(self, backend: _BaseBackend, on_result: Optional[Callable[[CheckResult], None]] = None) -> None:
        self.backend = backend
        # Called with each result as it completes, so a long run leaves evidence
        # behind even if the process is killed part way through.
        self.on_result = on_result

    # -- plumbing ----------------------------------------------------------
    def _require_output_readback(self, name: str) -> Optional[CheckResult]:
        """
        Refuse a readback-dependent check on a transport that has no 0x7FE.

        Returns a FAILED ``CheckResult`` to return instead of running, or None to
        proceed.  This is checked before the first ``send_frame`` on purpose: a
        refusal that arrives after the actuation is a refusal that is too late.
        """
        if getattr(self.backend, "provides_output_readback", False):
            return None
        return CheckResult(
            name,
            False,
            f"refused before sending any frame: the {describe_can_id(CAN_ID_SIL_OUTPUT_STATUS)} "
            "output-status readback is a SIL-only mock-BSP channel (sil_protocol.py:22) and this "
            "transport cannot supply it, so no actuation sent over "
            f"{self.backend.describe_destination(0)} could be verified. A thruster or valve "
            "command that moves something and cannot be read back is a blind actuation, so this "
            "check is SIL-only. Use --mode sil, or select the receive-only checks "
            "(--node1-env, --node3-power, --node2-depth, --sniff).",
        )

    def run_plan(self, plan: Sequence[Tuple[str, Callable[[], CheckResult]]]) -> StimulusReport:
        """Run a name/action plan, streaming each result as it completes."""
        results: List[CheckResult] = []
        for name, action in plan:
            result = self.guard(name, action)
            results.append(result)
            if self.on_result is not None:
                self.on_result(result)
        return StimulusReport(results)

    def guard(self, name: str, action: Callable[[], CheckResult]) -> CheckResult:
        """
        Run a check and convert any exception into a FAILED ``CheckResult``.

        This is the whole point of the tool.  The prototype raised
        ``AttributeError`` out of its own check and the process still exited 0, so
        a crash was indistinguishable from success.  Here the exception text is
        preserved in ``detail`` and the verdict is a failure.
        """
        try:
            result = action()
        except Exception as exc:  # noqa: BLE001 - deliberately total
            return CheckResult(name, False, f"check raised {type(exc).__name__}: {exc}")
        if not isinstance(result, CheckResult):
            return CheckResult(
                name, False, f"check returned {type(result).__name__} instead of a CheckResult"
            )
        return result

    def _pump(
        self,
        deadline: float,
        send: Optional[Callable[[], None]] = None,
        on_frame: Optional[Callable[[int, bytes], None]] = None,
        on_output: Optional[Callable[[OutputStatus], None]] = None,
        until: Optional[Callable[[OutputStatus], bool]] = None,
    ) -> List[OutputStatus]:
        """
        Bounded transport loop used by every check.

        Resends the command at :data:`COMMAND_PERIOD_S` so the firmware's 100 ms
        heartbeat stays fed, and reads in :data:`READ_POLL_S` slices so a check
        never blocks longer than its own deadline.  Returns the fresh 0x7FE
        snapshots observed; ``until`` ends the loop as soon as it is satisfied.
        """
        snapshots: List[OutputStatus] = []
        previous_listener = self.backend.set_frame_listener(on_frame)
        next_send_due = time.monotonic()
        try:
            while True:
                now = time.monotonic()
                if now >= deadline:
                    break
                if send is not None and now >= next_send_due:
                    send()
                    next_send_due = now + COMMAND_PERIOD_S
                slice_s = max(0.0, min(READ_POLL_S, deadline - now))
                before = self.backend.latest_outputs()
                if hasattr(self.backend, "drain_frames"):
                    self.backend.drain_frames(slice_s)
                else:
                    # PythonCanBackend and UartBackend have no drain_frames: they
                    # hand back one frame at a time and no 0x7FE readback, so the
                    # readback-dependent checks have already refused. The
                    # receive-only checks still run here, one frame per slice.
                    self.backend.recv_frame(timeout_s=slice_s)
                current = self.backend.latest_outputs()
                if current is not None and current is not before:
                    snapshots.append(current)
                    if on_output is not None:
                        on_output(current)
                    if until is not None and until(current):
                        break
        finally:
            self.backend.set_frame_listener(previous_listener)
        return snapshots

    def _await_output(
        self,
        predicate: Callable[[OutputStatus], bool],
        timeout_s: float,
        send: Optional[Callable[[], None]] = None,
    ) -> Optional[OutputStatus]:
        """Bounded poll for the first fresh snapshot satisfying ``predicate``."""
        snapshots = self._pump(time.monotonic() + max(0.0, timeout_s), send=send, until=predicate)
        if snapshots and predicate(snapshots[-1]):
            return snapshots[-1]
        return None

    def _first_output(self, timeout_s: float) -> Optional[OutputStatus]:
        return self._await_output(lambda snapshot: True, timeout_s)

    def _collect(
        self,
        can_id: int,
        duration_s: float,
        send: Optional[Callable[[], None]] = None,
        on_output: Optional[Callable[[OutputStatus], None]] = None,
    ) -> List[bytes]:
        payloads: List[bytes] = []

        def on_frame(frame_id: int, payload: bytes) -> None:
            if frame_id == can_id:
                payloads.append(payload)

        self._pump(
            time.monotonic() + max(0.0, duration_s),
            send=send,
            on_frame=on_frame,
            on_output=on_output,
        )
        return payloads

    def _no_readback_failure(self, name: str, timeout_s: float) -> CheckResult:
        return CheckResult(
            name,
            False,
            f"no 0x7FE output-status readback within {timeout_s:.1f} s. The SIL engine only "
            "publishes that channel to a connected client (sil_bridge_server.c:339-359), so this "
            "means the engine is not running, the connection was dropped, or the client is not "
            "reading - the request is NOT confirmed.",
        )

    def _release_to_neutral(self, timeout_s: float = NEUTRAL_TIMEOUT_S) -> bool:
        """Stop commanding and require the outputs to come back to neutral."""
        neutral = ThrusterCommand(pwm_us=list(NEUTRAL_PWMS)).pack()

        def send_neutral() -> None:
            self.backend.send_frame(CAN_ID_THRUSTER_CMD, neutral)

        released = self._await_output(
            lambda snapshot: tuple(snapshot.pwms) == NEUTRAL_PWMS, timeout_s, send=send_neutral
        )
        return released is not None

    def _ramp_budget_s(self, pulse_us: int, sim_rate: float, slack_s: float) -> float:
        """
        Wall-clock budget for a slew ramp to ``pulse_us`` and back.

        The 2 us/ms ramp is charged in *simulated* milliseconds
        (``rov_parameters.h:45``) and converted at ``sim_rate``, so a test can
        raise the rate instead of waiting out half a second of virtual time.
        """
        travel_ms = abs(pulse_us - NEUTRAL_US) / PWM_SLEW_RATE_US_PER_MS
        return travel_ms / 1000.0 / max(1e-6, sim_rate) + slack_s

    def _sim_period_wall_budget_s(self, sim_period_ms: int = POWER_TELEMETRY_SIM_PERIOD_MS) -> float:
        """
        Wall-clock budget for the engine to live through ``sim_period_ms``.

        ``SIM_MS_PER_WALL_S`` is a deliberately generous LOWER bound on the
        engine's tick rate, so dividing by it OVER-estimates the wall time a
        given amount of simulated time needs.  That is the safe direction for a
        budget -- a host that is slower still fits, a host that is faster simply
        stops early -- and it is the same input ``_ramp_budget_s`` uses.  It stays
        a BUDGET input: nothing here reports a rate derived from it, because a
        lower bound is not a measurement.
        """
        return sim_period_ms / 1000.0 / SIM_MS_PER_WALL_S + SIM_WALL_SLACK_S

    # -- node 1: Pi shield -------------------------------------------------
    def monitor_node1_env(self, duration_s: float = 3.0) -> CheckResult:
        """
        Watch 0x210 enclosure telemetry from the Pi shield.

        Asserts that at least two frames arrive in the window, that every frame
        decodes as a full 17-byte ``EnvTelemetry`` record, that the values are
        finite and physically plausible, and that no leak is flagged.  The count
        of two is grounded in the server: the first 0x210 is sent immediately on
        connect and the rest at 1 Hz (sil_bridge_server.c:376-382), so a 3.0 s
        window must contain more than one.  A nonzero leak flag in a nominal
        simulation is a genuine failure, because the shield would already have
        broadcast an emergency break (node1 app.c:220-222).
        """
        name = "node1_env"
        payloads = self._collect(CAN_ID_ENV_TELEMETRY, duration_s)
        if not payloads:
            return CheckResult(
                name,
                False,
                f"no 0x210 environment telemetry within {duration_s:.1f} s; the Pi shield streams "
                "it at 10 Hz (node1 app.c:159) and the server forwards the first frame immediately "
                f"then 1 Hz (sil_bridge_server.c:376-382). Received nothing.",
            )
        if len(payloads) < 2:
            return CheckResult(
                name,
                False,
                f"only {len(payloads)} 0x210 frame(s) in {duration_s:.1f} s; the server emits the "
                "first frame on connect and then one per second, so at least 2 are required to "
                "distinguish a live stream from a single stale frame.",
            )
        try:
            records = [EnvTelemetry.unpack(payload) for payload in payloads]
        except ValueError as exc:
            return CheckResult(
                name, False, f"0x210 frame could not be decoded as EnvTelemetry: {exc}"
            )
        for index, record in enumerate(records):
            for field in ("pressure_hpa", "humidity_pct", "temperature_c"):
                if not math.isfinite(getattr(record, field)):
                    return CheckResult(
                        name, False, f"0x210 frame {index} has a non-finite {field}: {record}"
                    )
            if not 500.0 <= record.pressure_hpa <= 1200.0:
                return CheckResult(
                    name,
                    False,
                    f"0x210 frame {index} reports {record.pressure_hpa:.2f} hPa, outside the "
                    "500-1200 hPa range a submerged enclosure can hold",
                )
            if not 0.0 <= record.humidity_pct <= 100.0:
                return CheckResult(
                    name, False, f"0x210 frame {index} reports {record.humidity_pct:.2f} % RH"
                )
            if not -40.0 <= record.temperature_c <= 85.0:
                return CheckResult(
                    name, False, f"0x210 frame {index} reports {record.temperature_c:.2f} C"
                )
            if record.leak_flags != 0:
                return CheckResult(
                    name,
                    False,
                    f"0x210 frame {index} reports leak_flags=0x{record.leak_flags:02X}; a nominal "
                    "enclosure must be dry. A set flag means the shield should already have "
                    "broadcast an emergency break (node1 app.c:220-222).",
                )
        last = records[-1]
        return CheckResult(
            name,
            True,
            f"{len(records)} frame(s) decoded in {duration_s:.1f} s: "
            f"pressure={last.pressure_hpa:.2f} hPa humidity={last.humidity_pct:.1f} % "
            f"temperature={last.temperature_c:.1f} C leak_flags=0x{last.leak_flags:02X} "
            f"rail={last.rail_voltage_mv} mV @ {last.rail_current_ma} mA",
        )

    # -- node 2: control board --------------------------------------------
    def test_node2_arming(
        self,
        sim_rate: float = SIM_MS_PER_WALL_S,
        slack_s: float = SIM_WALL_SLACK_S,
        first_output_timeout_s: float = FIRST_OUTPUT_TIMEOUT_S,
    ) -> CheckResult:
        """
        Prove the mandatory ESC arming gate, in both directions.

        ``nodes/node2_control_board/Core/Src/app.c`` refuses every thruster
        command while ``g_esc_state != ESC_STATE_ACTIVE`` (app.c:127-129) and
        holds all eight channels at neutral for the whole
        ``ESC_ARMING_TIME_MS`` window (app.c:192-194), where
        ``ESC_ARMING_TIME_MS`` is 3000 ms of *simulated* time (app.c:24,99-102).

        Asserts: a neutral observation taken while the window is still open; that
        an aggressive command sent during the window is ignored (the gate, not
        an absence of traffic, is what holds neutral); that the same command takes
        effect once the window closes; and that releasing it returns the outputs
        to neutral.

        The neutral run is required to cover the window with
        :data:`MIN_ARMING_SAMPLES` observations *strictly inside* it.  Without
        that floor a single post-3000 snapshot would make the intruder list empty
        and the gate would look verified having never been watched.

        ``sim_rate`` and ``slack_s`` let the tests drive these branches without
        waiting out 3000 ms of virtual time; production callers use the defaults.
        """
        name = "node2_arming"
        refused = self._require_output_readback(name)
        if refused is not None:
            return refused
        first = self._first_output(first_output_timeout_s)
        if first is None:
            return self._no_readback_failure(name, first_output_timeout_s)
        if first.sim_time_ms >= ESC_ARMING_SIM_MS:
            return CheckResult(
                name,
                False,
                f"the arming gate was already open when the first snapshot arrived "
                f"(sim_time_ms={first.sim_time_ms} >= {ESC_ARMING_SIM_MS}), so the mandatory "
                f"{ESC_ARMING_SIM_MS} ms window could not be observed. Restart the engine (or let "
                "this tool launch one) and run the arming check first.",
            )
        if tuple(first.pwms) != NEUTRAL_PWMS:
            return CheckResult(
                name,
                False,
                f"outputs were not neutral before arming: {list(first.pwms)} at "
                f"sim_time_ms={first.sim_time_ms}; app.c:63-67 parks every channel at "
                f"{NEUTRAL_US} us during boot and arming",
            )

        probe_pwms = [SMOKE_SINGLE_PWM_US] * NUM_THRUSTERS
        command = ThrusterCommand(pwm_us=probe_pwms).pack()

        def send_command() -> None:
            self.backend.send_frame(CAN_ID_THRUSTER_CMD, command)

        remaining_sim_ms = ESC_ARMING_SIM_MS - first.sim_time_ms
        window_budget_s = remaining_sim_ms / 1000.0 / max(1e-6, sim_rate) + slack_s
        during = self._pump(
            time.monotonic() + window_budget_s,
            send=send_command,
            until=lambda snapshot: snapshot.sim_time_ms >= ESC_ARMING_SIM_MS,
        )
        if not during or during[-1].sim_time_ms < ESC_ARMING_SIM_MS:
            reached = during[-1].sim_time_ms if during else "no snapshots"
            return CheckResult(
                name,
                False,
                f"the {ESC_ARMING_SIM_MS} ms arming window never elapsed within {window_budget_s:.1f} "
                f"s of wall time (sim clock reached {reached} ms). The engine's virtual clock is "
                "not advancing, so no arming verdict is possible.",
            )
        inside = [snapshot for snapshot in during if snapshot.sim_time_ms < ESC_ARMING_SIM_MS]
        if len(inside) < MIN_ARMING_SAMPLES:
            return CheckResult(
                name,
                False,
                f"only {len(inside)} observation(s) landed strictly inside the {ESC_ARMING_SIM_MS} ms "
                f"window, below the {MIN_ARMING_SAMPLES} required. A single snapshot at or after the "
                "boundary would make the window look observed when it was not, so the gate holding "
                "neutral is unverified.",
            )
        intruders = [
            snapshot for snapshot in inside if tuple(snapshot.pwms) != NEUTRAL_PWMS
        ]
        if intruders:
            return CheckResult(
                name,
                False,
                f"outputs left neutral at sim_time_ms={intruders[0].sim_time_ms} < "
                f"{ESC_ARMING_SIM_MS} while a {SMOKE_SINGLE_PWM_US} us command was being resent; "
                f"app.c:192-194 must hold neutral for the whole arming window. First offender: "
                f"{intruders[0].describe()}",
            )

        opened = self._await_output(
            lambda snapshot: snapshot.sim_time_ms >= ESC_ARMING_SIM_MS
            and any(pulse != NEUTRAL_US for pulse in snapshot.pwms),
            self._ramp_budget_s(SMOKE_SINGLE_PWM_US, sim_rate, slack_s),
            send=send_command,
        )
        if opened is None:
            last = self.backend.latest_outputs()
            seen = "no readback" if last is None else last.describe()
            return CheckResult(
                name,
                False,
                f"the arming window closed at {ESC_ARMING_SIM_MS} ms but the outputs never left "
                f"neutral afterwards, so the gate never opened ({seen}). Either the command is not "
                "reaching node 2 or the board stays disarmed.",
            )
        if not self._release_to_neutral(self._ramp_budget_s(SMOKE_SINGLE_PWM_US, sim_rate, slack_s)):
            return CheckResult(
                name,
                False,
                f"thrust was observed at sim_time_ms={opened.sim_time_ms} but the outputs did not "
                f"return to {NEUTRAL_US} us after the command was released",
            )
        return CheckResult(
            name,
            True,
            f"gate held neutral across {len(inside)} observation(s) inside the first "
            f"{ESC_ARMING_SIM_MS} ms of sim time against a resent {SMOKE_SINGLE_PWM_US} us command, "
            f"then took effect at sim_time_ms={opened.sim_time_ms} ({list(opened.pwms)}) and "
            "released back to neutral",
        )

    def test_node2_pwm(
        self,
        channel: int,
        pulse_us: int,
        duration_s: float = 1.0,
        timeout_s: float = RAMP_TIMEOUT_S,
        sim_rate: float = SIM_MS_PER_WALL_S,
        slack_s: float = SIM_WALL_SLACK_S,
    ) -> CheckResult:
        """
        Drive one thruster channel and require it to hold.

        The interesting part is the *hold*: node 2 drops to neutral when no
        thruster command arrives for 100 ms (``rov_parameters.h:71``,
        ``rov_safety.c:29-35``, ``app.c:182-189``), so a single frame is not
        evidence of anything.  The command is resent at 20 Hz and every snapshot
        inside the observation window must hold the commanded value, and the other
        seven channels must stay at neutral throughout.

        ``sim_rate``/``slack_s``/``timeout_s`` exist so the failure branches are
        testable without waiting out a slew ramp of half a second of virtual time.
        """
        channel = _validate_channel(channel)
        pulse_us = _validate_pulse_us(pulse_us)
        name = f"node2_pwm_ch{channel}"
        refused = self._require_output_readback(name)
        if refused is not None:
            return refused
        pwms = [NEUTRAL_US] * NUM_THRUSTERS
        pwms[channel] = pulse_us
        command = ThrusterCommand(pwm_us=pwms).pack()

        def send_command() -> None:
            self.backend.send_frame(CAN_ID_THRUSTER_CMD, command)

        reached = self._await_output(
            lambda snapshot: snapshot.pwms[channel] == pulse_us, timeout_s, send=send_command
        )
        if reached is None:
            last = self.backend.latest_outputs()
            seen = "no readback" if last is None else last.describe()
            # The generic message, and the facts behind it. Built first so both
            # branches below carry the same evidence; the gate-closed branch then
            # leads with the conclusion, because "the command was never accepted"
            # and "the board cannot accept commands yet" call for different
            # operator actions and must not read the same.
            detail = (
                f"channel {channel} never reached {pulse_us} us within {timeout_s:.1f} s "
                f"({seen}). The 2 us/ms slew ramp (rov_parameters.h:45) needs "
                f"{abs(pulse_us - NEUTRAL_US) // PWM_SLEW_RATE_US_PER_MS} ms of sim time, and the "
                "command is only accepted once the ESC gate is open (app.c:127-129)."
            )
            if last is not None and last.sim_time_ms < ESC_ARMING_SIM_MS:
                # The budget expired while the mandatory arming window was still
                # running, so this check has not examined the PWM path at all.
                # RAMP_TIMEOUT_S is a fixed SIM_WALL_SLACK_S cushion on top of the
                # ramp, and the arming window is 3000 ms of *simulated* time, so a
                # host whose ticks are slower than the default assumption spends
                # its whole budget arming. Saying only "never reached" would send
                # the operator after a broken PWM path that was never exercised.
                return CheckResult(
                    name,
                    False,
                    f"the ESC gate was still closed at sim_time_ms={last.sim_time_ms}, below the "
                    f"mandatory {ESC_ARMING_SIM_MS} ms arming window (app.c:24), so this check has "
                    f"NOT examined the PWM path: the {timeout_s:.1f} s budget ran out before the "
                    "board was ever able to accept a thruster command. This is a too-short "
                    "timeout_s or a slow host, not a broken PWM path - raise timeout_s, or run "
                    f"--node2-arm first and re-run this check. For the record: {detail}",
                )
            return CheckResult(name, False, detail)
        samples = self._pump(time.monotonic() + max(0.0, duration_s), send=send_command)
        if not samples:
            return CheckResult(
                name,
                False,
                f"channel {channel} reached {pulse_us} us but no fresh 0x7FE snapshot arrived during "
                f"the {duration_s:.1f} s observation window, so the hold is unverified",
            )
        drifted = [s for s in samples if s.pwms[channel] != pulse_us]
        if drifted:
            return CheckResult(
                name,
                False,
                f"channel {channel} did not hold {pulse_us} us for the whole {duration_s:.1f} s "
                f"window: {len(drifted)}/{len(samples)} snapshot(s) disagreed, first at "
                f"sim_time_ms={drifted[0].sim_time_ms} showing {list(drifted[0].pwms)}. A 100 ms "
                "watchdog lapse would look exactly like this (app.c:182-189).",
            )
        polluted = [
            index
            for index in range(NUM_THRUSTERS)
            if index != channel and any(s.pwms[index] != NEUTRAL_US for s in samples)
        ]
        if polluted:
            return CheckResult(
                name,
                False,
                f"channel(s) {polluted} moved off {NEUTRAL_US} us while only channel {channel} was "
                "commanded; the board must apply an 8-channel frame per channel",
            )
        if not self._release_to_neutral(self._ramp_budget_s(pulse_us, sim_rate, slack_s)):
            return CheckResult(
                name,
                False,
                f"channel {channel} held {pulse_us} us, but the outputs did not return to "
                f"{NEUTRAL_US} us after the command was released",
            )
        return CheckResult(
            name,
            True,
            f"channel {channel} held {pulse_us} us across {len(samples)} snapshot(s) in "
            f"{duration_s:.1f} s (> the {int(HEARTBEAT_TIMEOUT_S * 1000)} ms watchdog) with the other "
            "7 channels at neutral, then released to neutral",
        )

    def test_node2_all(
        self,
        pulse_us: int,
        duration_s: float = 1.0,
        timeout_s: float = RAMP_TIMEOUT_S,
        sim_rate: float = SIM_MS_PER_WALL_S,
        slack_s: float = SIM_WALL_SLACK_S,
    ) -> CheckResult:
        """
        Drive all eight channels and require the whole vector to hold.

        Same heartbeat argument as :meth:`test_node2_pwm`, applied to the full
        allocation, so a partial application (a dropped frame, a short payload) is
        caught.
        """
        pulse_us = _validate_pulse_us(pulse_us)
        name = "node2_all"
        refused = self._require_output_readback(name)
        if refused is not None:
            return refused
        command = ThrusterCommand(pwm_us=[pulse_us] * NUM_THRUSTERS).pack()

        def send_command() -> None:
            self.backend.send_frame(CAN_ID_THRUSTER_CMD, command)

        target = (pulse_us,) * NUM_THRUSTERS
        reached = self._await_output(
            lambda snapshot: tuple(snapshot.pwms) == target, timeout_s, send=send_command
        )
        if reached is None:
            last = self.backend.latest_outputs()
            seen = "no readback" if last is None else last.describe()
            return CheckResult(
                name,
                False,
                f"the board never reached {pulse_us} us on all {NUM_THRUSTERS} channels within "
                f"{timeout_s:.1f} s ({seen}). The 2 us/ms slew ramp (rov_parameters.h:45) needs "
                f"{abs(pulse_us - NEUTRAL_US) // PWM_SLEW_RATE_US_PER_MS} ms of sim time.",
            )
        samples = self._pump(time.monotonic() + max(0.0, duration_s), send=send_command)
        if not samples:
            return CheckResult(
                name,
                False,
                f"all channels reached {pulse_us} us but no fresh 0x7FE snapshot arrived during the "
                f"{duration_s:.1f} s observation window",
            )
        drifted = [s for s in samples if tuple(s.pwms) != target]
        if drifted:
            first_bad = drifted[0]
            channels = [i for i, v in enumerate(first_bad.pwms) if v != pulse_us]
            return CheckResult(
                name,
                False,
                f"the {pulse_us} us allocation did not hold for the whole {duration_s:.1f} s window: "
                f"{len(drifted)}/{len(samples)} snapshot(s) disagreed, channel(s) {channels} at "
                f"sim_time_ms={first_bad.sim_time_ms} showing {list(first_bad.pwms)}",
            )
        if not self._release_to_neutral(self._ramp_budget_s(pulse_us, sim_rate, slack_s)):
            return CheckResult(
                name,
                False,
                f"all channels held {pulse_us} us, but the outputs did not return to {NEUTRAL_US} us "
                "after the command was released",
            )
        return CheckResult(
            name,
            True,
            f"all {NUM_THRUSTERS} channels held {pulse_us} us across {len(samples)} snapshot(s) in "
            f"{duration_s:.1f} s (> the {int(HEARTBEAT_TIMEOUT_S * 1000)} ms watchdog), then "
            "released to neutral",
        )

    def test_node2_solenoid(
        self, mask: int, timeout_s: float = SOLENOID_OBSERVE_TIMEOUT_S
    ) -> CheckResult:
        """
        Actuate a solenoid mask and require the board to report exactly that mask.

        The mask is validated against the real interlock first, because an invalid
        mask cannot be used as evidence.  ``shared/src/rov_can_protocol.c:118-126``
        masks to ten channels and then rejects any valve with both opposing coils
        energised by zeroing the *whole* mask and returning ``ROV_ERR_INVALID_ARG``;
        ``app.c:150-153`` only calls ``bsp_solenoid_set`` on ``ROV_OK``, so the
        hardware keeps its previous mask.  A tool that sent such a mask and then
        read back a stale or all-zero mask would be reporting the wrong thing
        entirely, so the check refuses it up front and says why.

        A neutral thruster command is resent while polling: once the ESC gate is
        open, a heartbeat lapse calls ``node2_force_neutral()``, which zeroes the
        solenoids (``app.c:49-52,182-189``).
        """
        name = "node2_solenoid"
        if not isinstance(mask, int) or isinstance(mask, bool):
            raise TypeError(f"solenoid mask must be an int, got {type(mask).__name__}")
        if not 0 <= mask <= SOLENOID_MASK_MAX:
            raise ValueError(
                f"solenoid mask 0x{mask:X} is outside the 10-bit field (rov_parameters.h:54)"
            )
        if mask == 0:
            # 0x0000 is the released state the board already holds
            # (app.c:72 zeroes it at init), so commanding it produces no state
            # change to observe: the readback would satisfy any predicate on the
            # first snapshot from a board that never received the frame. It is
            # refused here rather than reported as a successful actuation.
            raise ValueError(
                "solenoid mask 0x0000 is refused: it commands the released state the board "
                "already holds (app.c:72), so it cannot be evidenced by a readback - a de-energised "
                "mask and an ignored frame look identical. Pass a mask that energises at least one "
                f"coil, such as 0x{SMOKE_SOLENOID_MASK:04X} (valve 0 coil A)."
            )
        refused = self._require_output_readback(name)
        if refused is not None:
            return refused
        conflict = conflicting_solenoid_valve(mask)
        if conflict is not None:
            pair_mask = 0x3 << (2 * conflict)
            return CheckResult(
                name,
                False,
                f"mask 0x{mask:04X} energises both opposing coils of valve {conflict} "
                f"(bits {2 * conflict} and {2 * conflict + 1}, pair mask 0x{pair_mask:04X}). "
                "rov_can_unpack_solenoid_cmd (shared/src/rov_can_protocol.c:120-126) zeroes the "
                "whole mask and returns ROV_ERR_INVALID_ARG, and app.c:150-153 then skips "
                "bsp_solenoid_set, so the readback would still show the previous mask and say "
                "nothing about this frame. Re-run with a single-coil mask such as "
                f"0x{SMOKE_SOLENOID_MASK:04X} (valve 0 coil A).",
            )

        coils = energised_coils(mask)
        neutral = ThrusterCommand(pwm_us=list(NEUTRAL_PWMS)).pack()

        def send_neutral() -> None:
            self.backend.send_frame(CAN_ID_THRUSTER_CMD, neutral)

        # Read the pre-command state before writing, so the pass can require an
        # observed transition rather than merely a value that was already true.
        # Bounded, because a missing readback is a failed check, not a hang.
        pre_existing = self._await_output(lambda snapshot: True, min(timeout_s, 0.5))
        if pre_existing is None:
            return self._no_readback_failure(name, min(timeout_s, 0.5))
        previous_mask = pre_existing.solenoids
        if previous_mask == mask:
            return CheckResult(
                name,
                False,
                f"the board already reports solenoid mask 0x{mask:04X} at sim_time_ms="
                f"{pre_existing.sim_time_ms}, before this check sent anything. Commanding the state "
                "the board is already in produces an identical readback, so the frame cannot be "
                "distinguished from one that was ignored. Pick a mask the board is not holding, or "
                "restart the engine to clear the latched mask.",
            )
        self.backend.send_frame(CAN_ID_SOLENOID_CMD, SolenoidCommand(solenoid_mask=mask).pack())
        actuated = self._await_output(
            lambda snapshot: snapshot.solenoids == mask, timeout_s, send=send_neutral
        )
        if actuated is None:
            last = self.backend.latest_outputs()
            seen = "no 0x7FE readback" if last is None else last.describe()
            return CheckResult(
                name,
                False,
                f"0x110 mask 0x{mask:04X} was written but the board never reported it back within "
                f"{timeout_s:.1f} s ({seen}). Either the frame did not reach node 2 or the mask was "
                "rejected; the readback is the only evidence of actuation.",
            )
        self.backend.send_frame(CAN_ID_SOLENOID_CMD, SolenoidCommand(solenoid_mask=0).pack())
        released = self._await_output(
            lambda snapshot: snapshot.solenoids == 0, timeout_s, send=send_neutral
        )
        if released is None:
            return CheckResult(
                name,
                False,
                f"0x110 mask 0x{mask:04X} actuated (readback 0x{actuated.solenoids:04X} at "
                f"sim_time_ms={actuated.sim_time_ms}) but the board did not return to 0x0000 after "
                "being commanded to release",
            )
        return CheckResult(
            name,
            True,
            f"0x110 mask 0x{mask:04X} energised {', '.join(coils)} and the board reported that mask "
            f"back at sim_time_ms={actuated.sim_time_ms}, a change from the pre-command "
            f"0x{previous_mask:04X} observed at sim_time_ms={pre_existing.sim_time_ms}. The mask "
            "energises one coil per valve, so rov_can_unpack_solenoid_cmd accepts it, and it was "
            f"released back to 0x0000 at sim_time_ms={released.sim_time_ms}.",
        )

    def monitor_node2_depth(self, duration_s: float = 2.0) -> CheckResult:
        """
        Watch 0x200 navigation telemetry, which carries the depth reading.

        The server forwards nav telemetry immediately at the board's 100 Hz rate
        (``sil_bridge_server.c:372-375``, ``rov_parameters.h:66``), so a 2.0 s
        window must carry a real stream.  Every frame must decode as a full
        ``NavTelemetry`` record with finite values, and the depth must respect the
        board's own acceptance range: ``app.c:228-234`` rejects anything below
        -0.5 m and clamps a small negative reading to 0.0 m.

        The 100 Hz is a *simulated* time rate, and the floor below is expressed in
        that clock too.  The C engine sleeps 10 ms of wall time per 10 ms tick
        (``sil_bridge_server.c:427``) and Windows ``Sleep`` has roughly 15.6 ms
        granularity, so a host observes about 64 Hz of wall clock for a 100 Hz
        stream; a floor in wall time would be a statement about the host's timer
        as much as about the board.  Every 0x7FE snapshot carries the engine's own
        clock, so the rate is measured against that and the wall figure is reported
        beside it, labelled as wall.
        """
        name = "node2_depth"
        sim_ms: List[int] = []
        payloads = self._collect(
            CAN_ID_NAV_TELEMETRY,
            duration_s,
            on_output=lambda snapshot: sim_ms.append(snapshot.sim_time_ms),
        )
        if not payloads:
            return CheckResult(
                name,
                False,
                f"no 0x200 navigation telemetry within {duration_s:.1f} s; node 2 streams it at "
                "100 Hz (node2 app.c:218-262) and the server forwards it unthrottled "
                "(sil_bridge_server.c:372-375). Received nothing.",
            )
        wall_hz = len(payloads) / max(1e-6, duration_s)
        # Simulated span across the snapshots that bracket the window. A span of
        # zero means a clock that did not move, which is not a rate at all, so the
        # wall figure stands in and the message says which clock it came from.
        sim_span_ms = sim_ms[-1] - sim_ms[0] if len(sim_ms) >= 2 else 0
        sim_hz = len(payloads) / (sim_span_ms / 1000.0) if sim_span_ms > 0 else None
        measured_hz, clock = (sim_hz, "simulated") if sim_hz is not None else (wall_hz, "wall")
        if measured_hz < MIN_TELEMETRY_HZ:
            return CheckResult(
                name,
                False,
                f"0x200 arrived at only {measured_hz:.1f} Hz {clock} ({len(payloads)} frames in "
                f"{duration_s:.1f} s); the board is specified for 100 Hz (rov_parameters.h:66) and "
                f"the server forwards it unthrottled, so anything under {MIN_TELEMETRY_HZ:.0f} Hz "
                "is a broken stream",
            )
        try:
            records = [NavTelemetry.unpack(payload) for payload in payloads]
        except ValueError as exc:
            return CheckResult(
                name, False, f"0x200 frame could not be decoded as NavTelemetry: {exc}"
            )
        for index, record in enumerate(records):
            values = {
                "q_w": record.q_w,
                "q_x": record.q_x,
                "q_y": record.q_y,
                "q_z": record.q_z,
                "gyro_x_rad_s": record.gyro_x_rad_s,
                "gyro_y_rad_s": record.gyro_y_rad_s,
                "gyro_z_rad_s": record.gyro_z_rad_s,
                "depth_meters": record.depth_meters,
            }
            for field, value in values.items():
                if not math.isfinite(value):
                    return CheckResult(
                        name, False, f"0x200 frame {index} has a non-finite {field}: {value}"
                    )
            if record.depth_meters < 0.0:
                return CheckResult(
                    name,
                    False,
                    f"0x200 frame {index} reports depth {record.depth_meters:.3f} m; node 2 clamps a "
                    "small negative reading to 0.0 m and rejects anything below -0.5 m "
                    "(node2 app.c:228-234)",
                )
            if record.depth_meters > 300.0:
                return CheckResult(
                    name,
                    False,
                    f"0x200 frame {index} reports depth {record.depth_meters:.2f} m, deeper than any "
                    "planned dive",
                )
        last = records[-1]
        if sim_hz is None:
            rate_text = (
                f"{len(records)} frame(s) at {wall_hz:.0f} Hz wall in {duration_s:.1f} s; the "
                "simulated rate is UNMEASURED because no 0x7FE output-status snapshot carried the "
                "engine's clock, so the floor above was applied to wall time"
            )
        else:
            rate_text = (
                f"{len(records)} frame(s) at {sim_hz:.0f} Hz simulated ({wall_hz:.0f} Hz wall), "
                f"measured over {sim_span_ms:.0f} ms of engine time, against the 100 Hz design rate"
            )
        return CheckResult(
            name,
            True,
            f"{rate_text}; depth={last.depth_meters:.3f} m imu_status=0x{last.imu_status:02X} "
            f"gyro=({last.gyro_x_rad_s:+.4f},{last.gyro_y_rad_s:+.4f},{last.gyro_z_rad_s:+.4f})",
        )

    # -- node 3: power slab -----------------------------------------------
    def monitor_node3_power(self, duration_s: float = 3.0) -> CheckResult:
        """
        Watch 0x300 power telemetry from the power slab.

        The server sends the first 0x300 immediately on connect and then one per
        ``POWER_TELEMETRY_SIM_PERIOD_MS`` (1000 ms) of **simulated** time
        (``sil_bridge_server.c:407-413``), so "more than one frame arrived" is a
        statement about the engine's own clock and nothing else.  Every frame must
        decode as a full 20-byte ``PowerTelemetry`` record, the rails must sit in
        their documented envelopes, the four 12 V brick currents must be present
        and sane, and the fault bit (``status_flags & 0x0001``, node3
        ``app.c:186,197``) must be clear.  A set fault bit means the slab latched
        a fault and broadcast a 0x005 eFuse alert, which trips the control board's
        brake (``app.c:121-126``).

        WHY THE WINDOW IS NOT COUNTED IN WALL TIME
        ------------------------------------------
        A 3.0 s *wall* window is 3.0 s of simulated time only on a host that keeps
        up with the engine.  The engine advances the sim clock 10 ms per loop
        iteration and then sleeps 10 ms (``sil_bridge_server.c:288,451``), so a
        host that cannot hold that rate runs its sim clock slower than real time
        in proportion to how far behind it falls.  At a third of real time a
        3.0 s wall window covers 1000 ms of simulated time, which is exactly one
        period: one 0x300 on connect and nothing else.  Counting frames in wall
        time therefore fails a Power Slab that is streaming perfectly, and it
        blames the slab for a shortfall that is entirely the host's.

        So the count is judged against the sim span actually observed, from the
        same ``latest_outputs().sim_time_ms`` readings ``monitor_node2_depth``
        uses, and when that span is too short to contain a period the window is
        extended on the engine's clock rather than the check failing.  The
        extension is bounded by the file's usual slow-host budget and stops the
        instant a second frame lands.

        Three outcomes, and no fourth:

        * two or more frames, sim span at least one period -- the slab is
          demonstrably alive and the frames are validated;
        * fewer than two frames while the sim clock covered a full period -- a
          genuinely starved stream, which still fails;
        * fewer than two frames and no usable sim clock (a transport with no
          0x7FE readback, or one that published none) -- the rate is UNMEASURED,
          the wall window stands, and the message says so instead of inventing a
          simulated rate.  It still fails: a check that could not be run must not
          report success.
        """
        name = "node3_power"
        payloads: List[bytes] = []
        sim_ms: List[int] = []

        def on_frame(frame_id: int, payload: bytes) -> None:
            if frame_id == CAN_ID_POWER_TELEMETRY:
                payloads.append(payload)

        def on_output(snapshot: OutputStatus) -> None:
            sim_ms.append(snapshot.sim_time_ms)

        def sim_span() -> int:
            return sim_ms[-1] - sim_ms[0] if len(sim_ms) >= 2 else 0

        # First window: exactly what this check has always collected, so on a host
        # whose engine keeps up the evidence gathered and the message are
        # unchanged by this fix.
        self._pump(time.monotonic() + duration_s, on_frame=on_frame, on_output=on_output)

        span_ms = sim_span()
        if len(payloads) < 2 and sim_ms and span_ms < POWER_TELEMETRY_SIM_PERIOD_MS:
            # The engine's own clock has not yet covered one 0x300 period inside
            # the caller's wall window, so the frame count carries no information
            # about the stream. Keep pumping on the sim clock, bounded, and stop
            # as soon as a second frame arrives.
            self._pump(
                time.monotonic() + self._sim_period_wall_budget_s(),
                on_frame=on_frame,
                on_output=on_output,
                until=lambda _snapshot: len(payloads) >= 2,
            )
            span_ms = sim_span()

        if not payloads:
            if sim_ms:
                detail = (
                    f"no 0x300 power telemetry within {duration_s:.1f} s of wall time; node 3 "
                    "publishes it at 20 Hz of its own clock (node3 app.c:78-207) and the server "
                    "forwards the first frame immediately on accept and then one per "
                    f"{POWER_TELEMETRY_SIM_PERIOD_MS} ms of SIMULATED time "
                    "(sil_bridge_server.c:407-413). Received nothing, and the engine's clock did "
                    f"advance {span_ms:.0f} ms of simulated time inside that window, so this is a "
                    "dead power stream, not a slow host."
                )
            else:
                detail = (
                    f"no 0x300 power telemetry within {duration_s:.1f} s of wall time; node 3 "
                    "publishes it at 20 Hz of its own clock (node3 app.c:78-207) and the server "
                    "forwards the first frame immediately on accept and then one per "
                    f"{POWER_TELEMETRY_SIM_PERIOD_MS} ms of SIMULATED time "
                    "(sil_bridge_server.c:407-413). Received nothing, and this transport published "
                    "no 0x7FE output-status snapshot, so the simulated rate could not be measured "
                    "at all and the window above is wall time only."
                )
            return CheckResult(name, False, detail)
        if len(payloads) < 2:
            if sim_ms:
                detail = (
                    f"only {len(payloads)} 0x300 frame(s), and the engine's own clock advanced "
                    f"{span_ms:.0f} ms of SIMULATED time during the window, so a second frame was "
                    f"due within the {POWER_TELEMETRY_SIM_PERIOD_MS} ms design period and did not "
                    "arrive. At least 2 are required to distinguish a live stream from a single "
                    "stale frame. This is measured on the simulated clock on purpose: the server "
                    "gates the repeat on "
                    f"`now_ms - s_last_tx_pwr >= {POWER_TELEMETRY_SIM_PERIOD_MS}` where `now_ms` is "
                    "time_get_ms() (sil_bridge_server.c:407-413), so a host whose engine runs "
                    "slower than real time still gets a full period of engine time before this is "
                    "called."
                )
            else:
                detail = (
                    f"only {len(payloads)} 0x300 frame(s) in {duration_s:.1f} s of WALL time, so "
                    "at least 2 are required to distinguish a live stream from a single stale "
                    "frame. The simulated rate is UNMEASURED because no 0x7FE output-status "
                    "snapshot carried the engine's clock, so the wall figure is all there is; on "
                    "the real SIL engine the repeat is gated on "
                    f"{POWER_TELEMETRY_SIM_PERIOD_MS} ms of simulated time "
                    "(sil_bridge_server.c:407-413) and a slow host would have been given the extra "
                    "time it needs before this was called."
                )
            return CheckResult(name, False, detail)
        try:
            records = [PowerTelemetry.unpack(payload) for payload in payloads]
        except ValueError as exc:
            return CheckResult(
                name, False, f"0x300 frame could not be decoded as PowerTelemetry: {exc}"
            )
        for index, record in enumerate(records):
            if not 40000 <= record.tether_voltage_mv <= 60000:
                return CheckResult(
                    name,
                    False,
                    f"0x300 frame {index} reports a tether of {record.tether_voltage_mv} mV, outside "
                    "the 40-60 V tether envelope (rov_parameters.h:17)",
                )
            if not 4000 <= record.v5_voltage_mv <= 7000:
                return CheckResult(
                    name,
                    False,
                    f"0x300 frame {index} reports a 5 V rail of {record.v5_voltage_mv} mV, outside "
                    "the 4.0-7.0 V logic rail envelope (rov_parameters.h:21)",
                )
            if record.tether_current_ma > 25000:
                return CheckResult(
                    name,
                    False,
                    f"0x300 frame {index} reports a tether current of {record.tether_current_ma} mA, "
                    "above the 25 A limit (rov_parameters.h:18)",
                )
            if len(record.v12_current_ma) != 4:
                return CheckResult(
                    name,
                    False,
                    f"0x300 frame {index} carries {len(record.v12_current_ma)} 12 V brick reading(s); "
                    "4 are required (rov_parameters.h:25)",
                )
            for brick, current_ma in enumerate(record.v12_current_ma, start=1):
                if current_ma > 25000:
                    return CheckResult(
                        name,
                        False,
                        f"0x300 frame {index} 12 V brick {brick} draws {current_ma} mA, above the "
                        "25 A limit (rov_parameters.h:26)",
                    )
            if not -400 <= record.pcb_temp_c_tenths <= 1200:
                return CheckResult(
                    name,
                    False,
                    f"0x300 frame {index} reports a PCB temperature of "
                    f"{record.pcb_temp_c_tenths / 10.0:.1f} C",
                )
            if record.status_flags & 0x0001:
                return CheckResult(
                    name,
                    False,
                    f"0x300 frame {index} has the fault bit set (status_flags=0x"
                    f"{record.status_flags:04X}); node 3 latched a fault and broadcast a 0x005 eFuse "
                    "alert (node3 app.c:185-195), which trips the control board brake "
                    "(node2 app.c:121-126).",
                )
        last = records[-1]
        if sim_ms:
            clock_text = (
                f"{len(records)} frame(s) over {span_ms:.0f} ms of SIMULATED time, the engine's "
                f"own clock, against the {POWER_TELEMETRY_SIM_PERIOD_MS} ms design period"
            )
        else:
            # No 0x7FE, so the only span available is the caller's wall window. Say
            # that rather than letting "decoded in 3.0 s" read as a cadence claim.
            clock_text = (
                f"{len(records)} frame(s) within {duration_s:.1f} s of WALL time; the simulated "
                "rate is UNMEASURED because no 0x7FE output-status snapshot carried the engine's "
                "clock"
            )
        return CheckResult(
            name,
            True,
            f"{clock_text}: tether="
            f"{last.tether_voltage_mv / 1000.0:.1f} V @ {last.tether_current_ma / 1000.0:.2f} A, "
            f"5 V rail={last.v5_voltage_mv / 1000.0:.2f} V @ {last.v5_current_ma / 1000.0:.2f} A, "
            f"12 V bricks={last.v12_current_ma} mA, pcb={last.pcb_temp_c_tenths / 10.0:.1f} C, "
            f"status_flags=0x{last.status_flags:04X}",
        )

    # -- vehicle-wide safety ----------------------------------------------
    def test_emergency_break(
        self,
        timeout_s: float = EMERGENCY_TIMEOUT_S,
        sim_rate: float = SIM_MS_PER_WALL_S,
        slack_s: float = SIM_WALL_SLACK_S,
        unauthorized_window_s: float = UNAUTHORIZED_FRAME_WINDOW_S,
        estop_window_s: float = ESTOP_LATCH_WINDOW_S,
        first_output_timeout_s: float = FIRST_OUTPUT_TIMEOUT_S,
    ) -> CheckResult:
        """
        Trip the emergency break with the authorized frame and require a latch.

        ``nodes/node2_control_board/Core/Src/app.c:110-119`` only reacts to 0x001
        when the payload begins ``0xAA 0x55``; it then trips the hardware brake,
        sets ``ESC_STATE_DISARMED``, and forces neutral.

        The post-ESTOP sub-claim is the whole reason this check is hard, and it is
        worthless without a precondition.  ``app.c:127-129`` refuses thruster
        commands whenever ``g_esc_state != ESC_STATE_ACTIVE`` - not only while the
        break is active - and ``app.c:192-193`` forces neutral for the whole
        arming period.  So "a 1800 us command afterwards did not resume thrust"
        proves the latch *only* if the board was demonstrably ACTIVE and accepting
        thrust immediately before the trip.  Against a board that never went
        ACTIVE the same observation is what a disarmed board always looks like.

        This check therefore arranges its own precondition and fails rather than
        dropping the claim:

        1. the brake is not already latched, so a later trip is attributable to
           this frame;
        2. an *unauthorized* payload (``0xAA 0x56``) does NOT trip it - this is
           what makes the trip meaningful instead of coincidental;
        3. the board is driven to a **non-neutral readback**, which is the proof
           that it was ACTIVE and accepting thrust.  If that cannot be observed
           within its own bounded budget the check FAILS and says the post-ESTOP
           claim is unobservable; it does not report a partial pass;
        4. exactly ``b"\\xAA\\x55\\x01"`` latches the brake with all eight channels
           at neutral;
        5. a full-thrust command sent afterwards does not resume thrust - now a
           statement about the latch, because step 3 established the board would
           otherwise have moved.

        ``sim_rate``, ``slack_s``, ``unauthorized_window_s`` and ``estop_window_s``
        exist so the tests can drive these branches without waiting out 3000 ms of
        virtual time; production callers use the module defaults.
        """
        name = "emergency_break"
        refused = self._require_output_readback(name)
        if refused is not None:
            return refused
        initial = self._first_output(timeout_s)
        if initial is None:
            return self._no_readback_failure(name, timeout_s)
        if initial.brake_active:
            return CheckResult(
                name,
                False,
                f"the emergency brake was already tripped at sim_time_ms={initial.sim_time_ms} "
                "before this check sent anything, so a trip could not be attributed to the "
                "authorized 0xAA 0x55 signature. Restart the engine to clear the latch and re-run.",
            )

        self.backend.send_frame(CAN_ID_EMERGENCY_BREAK, UNAUTHORIZED_FRAME)
        false_trips: List[OutputStatus] = []
        self._pump(
            time.monotonic() + unauthorized_window_s,
            on_output=lambda snapshot: false_trips.append(snapshot)
            if snapshot.brake_active
            else None,
        )
        if false_trips:
            return CheckResult(
                name,
                False,
                f"an unauthorized 0xAA 0x56 frame tripped the brake at sim_time_ms="
                f"{false_trips[0].sim_time_ms}; app.c:112 requires the 0xAA 0x55 signature, so the "
                "authorization check is not holding.",
            )

        armed = self._arm_before_trip(sim_rate, slack_s, first_output_timeout_s)
        if not isinstance(armed, OutputStatus):
            return armed
        if not self._release_to_neutral(self._ramp_budget_s(POST_ESTOP_PROBE_PWM_US, sim_rate, slack_s)):
            return CheckResult(
                name,
                False,
                f"the board accepted thrust ({list(armed.pwms)} at sim_time_ms={armed.sim_time_ms}) "
                f"but did not return to {NEUTRAL_US} us before the emergency frame, so the trip "
                "would be observed from an unknown starting state",
            )

        self.backend.send_frame(CAN_ID_EMERGENCY_BREAK, EMERGENCY_FRAME)
        latched = self._await_output(lambda snapshot: snapshot.brake_active, timeout_s)
        if latched is None:
            last = self.backend.latest_outputs()
            seen = "no readback" if last is None else last.describe()
            return CheckResult(
                name,
                False,
                f"the authorized frame {EMERGENCY_FRAME.hex(' ').upper()} did not latch the "
                f"emergency brake within {timeout_s:.1f} s ({seen}). app.c:110-119 should have "
                "tripped bsp_emergency_brake_trip and forced neutral.",
            )
        if tuple(latched.pwms) != NEUTRAL_PWMS:
            return CheckResult(
                name,
                False,
                f"the brake latched at sim_time_ms={latched.sim_time_ms} but the channels are not "
                f"all at {NEUTRAL_US} us: {list(latched.pwms)}. app.c:118 calls "
                "node2_force_neutral(), so every channel must read neutral.",
            )
        resumed: List[OutputStatus] = []
        probe = ThrusterCommand(pwm_us=[POST_ESTOP_PROBE_PWM_US] * NUM_THRUSTERS).pack()
        self._pump(
            time.monotonic() + estop_window_s,
            send=lambda: self.backend.send_frame(CAN_ID_THRUSTER_CMD, probe),
            on_output=lambda snapshot: resumed.append(snapshot)
            if tuple(snapshot.pwms) != NEUTRAL_PWMS
            else None,
        )
        if resumed:
            return CheckResult(
                name,
                False,
                f"a {POST_ESTOP_PROBE_PWM_US} us command resumed thrust at sim_time_ms="
                f"{resumed[0].sim_time_ms} ({list(resumed[0].pwms)}); app.c:116 sets "
                "ESC_STATE_DISARMED and app.c:129 refuses commands while the break is active, so "
                "the latch is not holding.",
            )
        return CheckResult(
            name,
            True,
            f"{EMERGENCY_FRAME.hex(' ').upper()} latched the brake at sim_time_ms="
            f"{latched.sim_time_ms} with all channels at {NEUTRAL_US} us. The unauthorized "
            f"{UNAUTHORIZED_FRAME.hex(' ').upper()} frame was correctly ignored. The latch claim is "
            f"earned: the board was observed at {list(armed.pwms)} (ESC ACTIVE, accepting thrust) "
            f"at sim_time_ms={armed.sim_time_ms}, immediately before the trip, and a "
            f"{POST_ESTOP_PROBE_PWM_US} us command afterwards did not resume thrust.",
        )

    def _arm_before_trip(
        self, sim_rate: float, slack_s: float, first_output_timeout_s: float
    ) -> object:
        """
        Drive the board to a non-neutral readback, or explain why that is impossible.

        This is the precondition the post-ESTOP sub-claim needs, and this check
        establishes it for itself rather than assuming it.  Returns the
        ``OutputStatus`` that proves the board was ACTIVE, or a FAILED
        ``CheckResult`` - never a silent skip, because a skipped precondition
        would turn the latch claim into an unearned one.
        """
        name = "emergency_break_precondition"
        probe = ThrusterCommand(pwm_us=[POST_ESTOP_PROBE_PWM_US] * NUM_THRUSTERS).pack()
        first = self._first_output(first_output_timeout_s)
        if first is None:
            return self._no_readback_failure(name, first_output_timeout_s)
        if first.brake_active:
            return CheckResult(
                name,
                False,
                f"the brake is tripped at sim_time_ms={first.sim_time_ms}, so the board is "
                "permanently DISARMED (app.c:116) and can never accept thrust again. Restart the "
                "engine to clear the latch.",
            )
        remaining_sim_ms = max(0, ESC_ARMING_SIM_MS - first.sim_time_ms)
        # Budget covers the rest of the arming window plus the slew ramp, both
        # converted from virtual time to wall time.
        budget_s = (
            remaining_sim_ms / 1000.0
            + abs(POST_ESTOP_PROBE_PWM_US - NEUTRAL_US) / PWM_SLEW_RATE_US_PER_MS
        ) / max(1e-6, sim_rate) + slack_s
        armed = self._await_output(
            lambda snapshot: any(pulse != NEUTRAL_US for pulse in snapshot.pwms),
            budget_s,
            send=lambda: self.backend.send_frame(CAN_ID_THRUSTER_CMD, probe),
        )
        if armed is None:
            last = self.backend.latest_outputs()
            seen = "no 0x7FE readback" if last is None else last.describe()
            return CheckResult(
                name,
                False,
                f"the board never accepted a {POST_ESTOP_PROBE_PWM_US} us command within "
                f"{budget_s:.1f} s of wall time, starting from sim_time_ms={first.sim_time_ms} "
                f"({seen}). The post-ESTOP latch claim is therefore UNOBSERVABLE against a board "
                "that never reached ESC_STATE_ACTIVE: app.c:127-129 refuses thruster commands "
                "whenever the ESC state is not ACTIVE and app.c:192-193 holds neutral throughout "
                "arming, so a 1800 us frame after the trip proves nothing here. This check is "
                "reported as FAILED rather than as a partial pass.",
            )
        return armed

    # -- ad hoc ------------------------------------------------------------
    def raw_send(self, can_id: int, payload: bytes) -> CheckResult:
        """
        Write one operator-supplied raw frame and report only that it was written.

        No readback is claimed: asserting "the frame went out" is a statement
        about the transport, so that is exactly what is asserted, and the text
        names the real destination - a UART port, which carries no arbitration id,
        is described as such rather than being reported as "CAN 0xNNN".

        ``raw_send`` is the one place a non-SIL transport may still write.  That
        is deliberate: the operator typed the exact id and bytes, and the tool is a
        pipe for them.  Every *check* refuses to actuate without a readback; this
        does not second-guess an explicit frame, it just refuses to misdescribe it.

        The two safety-critical IDs are gated on their authorization signature,
        because the firmware silently ignores an unsigned frame
        (``node2 app.c:112`` and ``:121``).  Writing one and reporting success
        would be the same defect as a check that reports a pass it did not earn.
        """
        # Validated FIRST, so the name is never built from a value the function is
        # about to reject: a negative id used to report itself as raw_send_-1 and
        # then raise.
        payload = validate_payload(payload)
        can_id = validate_can_id(can_id)
        name = f"raw_send_0x{can_id:03X}"
        shown = payload.hex(" ").upper() or "<empty>"
        required = SAFETY_SIGNATURES.get(can_id)
        if required is not None and not payload.startswith(required):
            return CheckResult(
                name,
                False,
                f"{describe_can_id(can_id)} is only acted on when the payload starts with the "
                f"authorization signature {required.hex(' ').upper()} (node2 app.c:112/:121), so "
                f"the {len(payload)}-byte payload {shown} would have been ignored. Nothing was "
                "written; add the signature or pick a different CAN id.",
            )
        self.backend.send_frame(can_id, payload)
        return CheckResult(
            name,
            True,
            f"wrote {len(payload)} byte(s) {shown} to {self.backend.describe_destination(can_id)} "
            "with no transport error; this asserts the write only, not any vehicle response",
        )

    def sniff(self, duration_s: float = 1.0) -> CheckResult:
        """Decode and print every frame seen in the window; pass iff there was one."""
        name = "sniff"
        counts: Dict[int, int] = {}
        printed: Dict[int, int] = {}

        def on_frame(can_id: int, payload: bytes) -> None:
            counts[can_id] = counts.get(can_id, 0) + 1
            if printed.get(can_id, 0) < 5:
                printed[can_id] = printed.get(can_id, 0) + 1
                print(f"  [0x{can_id:03X}] {len(payload):2d} B {payload[:16].hex(' ').upper()}")

        self._pump(time.monotonic() + max(0.0, duration_s), on_frame=on_frame)
        if not counts:
            return CheckResult(
                name,
                False,
                f"no frames decoded in {duration_s:.1f} s; a live SIL engine publishes 0x7FE at "
                "100 Hz plus telemetry (sil_bridge_server.c:339-412), so silence means the engine "
                "is not running or the connection is dead",
            )
        summary = ", ".join(f"0x{can_id:03X} x{count}" for can_id, count in sorted(counts.items()))
        return CheckResult(
            name, True, f"decoded {sum(counts.values())} frame(s) in {duration_s:.1f} s: {summary}"
        )

    # -- aggregate ---------------------------------------------------------
    def run_full_smoke(
        self,
        pwm_us: int = SMOKE_SINGLE_PWM_US,
        all_pwm_us: int = SMOKE_ALL_PWM_US,
        solenoid_mask: int = SMOKE_SOLENOID_MASK,
    ) -> StimulusReport:
        """
        Run the whole vehicle in one report, in the only order that is meaningful.

        * The arming gate is first, and nothing before it may send a thruster
          command.  The 3000 ms window (``app.c:24``) is measured on the engine's
          virtual clock, so it closes whether or not anyone is watching; once it
          closes the gate can no longer be observed and the check would be
          reporting on something it never saw.
        * The emergency break is last.  It latches for the life of the engine
          (``rov_safety.c`` has no clear path and ``app.c:116`` sets
          ``ESC_STATE_DISARMED``), so every check after it would only be observing
          a permanently disarmed board.
        """
        sequence: List[Tuple[str, Callable[[], CheckResult]]] = [
            ("node2_arming", self.test_node2_arming),
            ("node1_env", self.monitor_node1_env),
            ("node3_power", self.monitor_node3_power),
            ("node2_depth", self.monitor_node2_depth),
            ("node2_pwm", lambda: self.test_node2_pwm(0, pwm_us)),
            ("node2_all", lambda: self.test_node2_all(all_pwm_us)),
            ("node2_solenoid", lambda: self.test_node2_solenoid(solenoid_mask)),
            ("emergency_break", self.test_emergency_break),
        ]
        return self.run_plan(sequence)


