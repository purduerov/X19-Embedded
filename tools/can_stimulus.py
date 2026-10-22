#!/usr/bin/env python3
"""
Purdue ROV X19 fake-CAN stimulus tool (canonical façade).

Modularized into `tools.can_stimulus_pkg`.
This module maintains 100% backward compatibility for all CLI invocations,
public exports, and test introspection contracts.
"""

from __future__ import annotations

import contextlib
import sys
from pathlib import Path
from typing import Optional, Sequence

TOOLS_DIR = Path(__file__).resolve().parent
REPO_ROOT = TOOLS_DIR.parent
SIL_BRIDGE_DIR = REPO_ROOT / "tests" / "sil_companion_bridge"
SIL_STIMULUS_DIR = REPO_ROOT / "tests" / "sil_stimulus"
SIL_DASHBOARD_DIR = REPO_ROOT / "tests" / "sil_dashboard"
for _path in (REPO_ROOT, TOOLS_DIR, SIL_STIMULUS_DIR, SIL_BRIDGE_DIR, SIL_DASHBOARD_DIR):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from tools.can_stimulus_pkg import (
    ACTION_FLAGS,
    CAN_ID_EFUSE_FAULT_ALERT,
    CAN_ID_EMERGENCY_BREAK,
    CAN_ID_ENV_TELEMETRY,
    CAN_ID_NAV_TELEMETRY,
    CAN_ID_POWER_TELEMETRY,
    CAN_ID_SIL_OUTPUT_STATUS,
    CAN_ID_SOLENOID_CMD,
    CAN_ID_THRUSTER_CMD,
    CAN_ID_TIME_SYNC_MASTER,
    CAN_ID_TIME_SYNC_REQ,
    CAN_ID_TIME_SYNC_RESP,
    CAN_ID_USB_HUB_TELEMETRY,
    COMMAND_PERIOD_S,
    DEFAULT_RECV_TIMEOUT_S,
    DEFAULT_SIL_HOST,
    DEFAULT_SIL_PORT,
    EMERGENCY_FRAME,
    EMERGENCY_SIGNATURE,
    EMERGENCY_TIMEOUT_S,
    ESC_ARMING_SIM_MS,
    ESTOP_LATCH_WINDOW_S,
    FIRST_OUTPUT_TIMEOUT_S,
    HEARTBEAT_TIMEOUT_S,
    MAX_SIL_PAYLOAD,
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
    SERVER_SHUTDOWN_TIMEOUT_S,
    SERVER_START_TIMEOUT_S,
    SIL_PACKET_SIZE,
    SIM_MS_PER_WALL_S,
    SIM_WALL_SLACK_S,
    SMOKE_ALL_PWM_US,
    SMOKE_SINGLE_PWM_US,
    SMOKE_SOLENOID_MASK,
    SOCKET_TIMEOUT_S,
    SOLENOID_MASK_BITS,
    SOLENOID_MASK_MAX,
    SOLENOID_OBSERVE_TIMEOUT_S,
    SOLENOID_VALVES,
    UNAUTHORIZED_FRAME,
    UNAUTHORIZED_FRAME_WINDOW_S,
    CheckResult,
    EnvTelemetry,
    ForeignEngineError,
    NavTelemetry,
    OutputStatus,
    PowerTelemetry,
    PythonCanBackend,
    SilSocketBackend,
    SolenoidCommand,
    StimulusReport,
    ThrusterCommand,
    UartBackend,
    VehicleStimulusTester,
    _BaseBackend,
    _optional_module_available,
    _require_optional_module,
    _raw_frame,
    _validate_channel,
    _validate_pulse_us,
    build_backend,
    build_parser,
    conflicting_solenoid_valve,
    describe_can_id,
    energised_coils,
    format_result,
    int_literal,
    pack_sil_can_frame,
    print_report,
    require_server_executable,
    run_selected_action,
    selected_action_flags,
    target_label,
    unpack_sil_can_frame,
    unpack_sil_output_status,
    validate_can_id,
    validate_payload,
)

__all__ = [
    "CAN_ID_EFUSE_FAULT_ALERT",
    "CAN_ID_EMERGENCY_BREAK",
    "CAN_ID_ENV_TELEMETRY",
    "CAN_ID_NAV_TELEMETRY",
    "CAN_ID_POWER_TELEMETRY",
    "CAN_ID_SIL_OUTPUT_STATUS",
    "CAN_ID_SOLENOID_CMD",
    "CAN_ID_THRUSTER_CMD",
    "CAN_ID_TIME_SYNC_MASTER",
    "CAN_ID_TIME_SYNC_REQ",
    "CAN_ID_TIME_SYNC_RESP",
    "CAN_ID_USB_HUB_TELEMETRY",
    "ESC_ARMING_SIM_MS",
    "ForeignEngineError",
    "MIN_ARMING_SAMPLES",
    "NEUTRAL_PWMS",
    "NEUTRAL_US",
    "PWM_MAX_US",
    "PWM_MIN_US",
    "SIL_PACKET_SIZE",
    "CheckResult",
    "OutputStatus",
    "PythonCanBackend",
    "SilSocketBackend",
    "StimulusReport",
    "UartBackend",
    "VehicleStimulusTester",
    "build_backend",
    "build_parser",
    "conflicting_solenoid_valve",
    "describe_can_id",
    "energised_coils",
    "format_result",
    "int_literal",
    "main",
    "pack_sil_can_frame",
    "print_report",
    "run_selected_action",
    "selected_action_flags",
    "unpack_sil_can_frame",
    "unpack_sil_output_status",
    "validate_can_id",
    "validate_payload",
]


def main(
    argv: Optional[Sequence[str]] = None,
    backend_factory=None,
    parser_factory=None,
) -> int:
    """
    Canonical CLI entry point. Returns 0 only when every requested check passed.

    Four independent layers guarantee a crashed or timed-out check can never look
    like a success: each check converts its own observation failures into failed
    results, ``tester.guard`` converts anything that still escapes into a failed
    result, and the ``except`` clauses here turn a failure to reach the transport,
    or to run the sequence at all, into a nonzero exit.  On top of that, every
    result is streamed as it completes, so a run that is killed part way through
    still leaves per-check evidence behind.

    ``backend_factory`` is resolved from the module global on every call rather
    than captured as a default argument, so a test can substitute a transport.
    ``parser_factory`` is the same seam for the parser, and it is how
    ``tools/node2_stimulus.py`` puts its own ``prog`` and legacy epilogue on the
    canonical options without mutating anything this module owns.
    """
    backend_factory = backend_factory or build_backend
    parser_factory = parser_factory or build_parser
    args = parser_factory().parse_args(argv)
    selected = selected_action_flags(args)
    if args.auto and selected:
        # --auto runs the whole sequence; a flag alongside it would be dropped in
        # silence, which is how an operator ends up believing a check ran.
        print(
            f"[WARN] --auto runs the full sequence, so these flags are ignored: "
            f"{', '.join(selected)}. Drop --auto to run only the flags you listed."
        )
        selected = []
    if not args.auto and not selected:
        print(
            "[FAIL] no action was selected, so nothing was verified. Pass --auto for the whole "
            f"vehicle, or one of: {', '.join(ACTION_FLAGS)}."
        )
        return 2
    if args.auto and args.mode != "sil":
        print(
            f"[FAIL] --auto needs the {describe_can_id(CAN_ID_SIL_OUTPUT_STATUS)} output-status "
            f"readback, which is a SIL-only channel, so it cannot run in --mode {args.mode}. "
            "--auto drives thrusters and a pneumatic valve; running it where the result cannot be "
            "read back would be a blind actuation. Use --mode sil, or select the receive-only "
            "checks individually."
        )
        return 2
    if args.auto and args.attach_existing:
        # Refused in the pre-flight, before a transport is even built, for the same
        # reason the ownership check is: --auto drives all eight thrusters, actuates
        # a valve, and latches the brake, so there is no harmless version of it
        # against an engine this run did not start. The operator names the checks
        # they want instead, and the refusal names the way out.
        print(
            "[FAIL] refused: --auto cannot be combined with --attach-existing. --auto drives all "
            "eight thrusters, actuates a valve, and latches the emergency brake, so running it "
            "against an engine this run did not start commandeers somebody else's vehicle. Drop "
            "--auto and name the checks you want to run against it."
        )
        return 2

    target = target_label(args)
    print(
        f"X19 CAN stimulus: mode={args.mode} target={target} "
        f"({'full smoke' if args.auto else 'selected checks'})"
    )
    backend: Optional[_BaseBackend] = None
    try:
        backend = backend_factory(args)
        backend.connect()
    except ForeignEngineError as exc:
        # Its own branch, not the generic one below: the generic wording ("could
        # not open the transport") describes a fault, and this is a decision. A
        # refusal dressed as a connection failure is how the precondition becomes
        # invisible to the operator reading the log.
        print(f"[FAIL] refused: {exc}")
        if backend is not None:
            with contextlib.suppress(Exception):
                backend.close()
        return 2
    except Exception as exc:  # noqa: BLE001 - a failed connect is a failed run
        print(
            f"[FAIL] could not open the {args.mode} transport at {target}: "
            f"{type(exc).__name__}: {exc}"
        )
        if backend is not None:
            with contextlib.suppress(Exception):
                backend.close()
        return 1

    # Streamed as each check completes: a 30 s run must not print nothing until
    # the end, or a killed process leaves no evidence of what had already run.
    tester = VehicleStimulusTester(backend, on_result=lambda result: print(format_result(result)))
    try:
        if args.auto:
            report = tester.run_full_smoke()
        else:
            report = run_selected_action(tester, args)
    except Exception as exc:  # noqa: BLE001 - never report a crash as success
        print(f"[FAIL] the stimulus run aborted: {type(exc).__name__}: {exc}")
        return 1
    finally:
        with contextlib.suppress(Exception):
            backend.close()

    # One implementation of the operator-facing text, not a second copy of it:
    # every per-check line was already streamed above, so this is exactly
    # format(include_checks=False). Spelling the summary out again here is how
    # the copy the operator reads and the one the tests exercise drift apart.
    print_report(report, include_checks=False)
    return 0 if report.ok else 1


if __name__ == "__main__":
    sys.exit(main())
