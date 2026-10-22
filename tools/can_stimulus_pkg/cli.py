"""
CLI argument parsing and command dispatch for can_stimulus.
"""

from __future__ import annotations

import argparse
import contextlib
from typing import Callable, List, Optional, Sequence, Tuple

from .backends import ForeignEngineError, _BaseBackend, build_backend
from .constants import (
    CAN_ID_SIL_OUTPUT_STATUS,
    DEFAULT_SIL_HOST,
    DEFAULT_SIL_PORT,
)
from .models import CheckResult, StimulusReport, format_result
from .protocol import _raw_frame, describe_can_id, int_literal
from .tester import VehicleStimulusTester

ACTION_FLAGS = (
    "--node1-env",
    "--node2-arm",
    "--node2-thruster",
    "--node2-all",
    "--node2-depth",
    "--node2-solenoid",
    "--node3-power",
    "--emergency-break",
    "--raw-send",
    "--sniff",
)


def build_parser(
    prog: str = "can_stimulus.py", epilog: Optional[str] = None
) -> argparse.ArgumentParser:
    """
    Build the canonical parser, with the two fields a wrapper legitimately owns.

    ``prog`` defaults to this file's own name and is not derived from
    ``sys.argv[0]``: the tool is run as a script *and* imported by the tests and
    by ``tools/node2_stimulus.py``, so an argv-derived name would read ``-c`` or
    the wrapper's path. Pinning it is what makes the usage line name the program
    the operator ran.

    ``epilog`` and the formatter are the other half of that: a wrapper needs to
    print its own legacy table under the canonical flags, and it gets there by
    passing them here rather than by mutating a parser it does not own. That is
    why there is no second parser and no monkey-patch anywhere in this file.
    """
    parser = argparse.ArgumentParser(
        prog=prog,
        description=(
            "Inject stimulus at the X19 CAN boundary and verify the result. "
            "Exits nonzero unless every requested check passed."
        ),
        epilog=epilog,
        formatter_class=argparse.RawDescriptionHelpFormatter if epilog else argparse.HelpFormatter,
    )
    parser.add_argument(
        "--mode",
        choices=("sil", "can", "uart"),
        default="sil",
        help=(
            "sil drives the native SIL engine over TCP and is the only mode that can verify an "
            "actuation, because the CAN 0x7FE output-status readback exists only there. can and "
            "uart are RECEIVE-ONLY: they observe a real link, and every check that would need to "
            "actuate a thruster or a valve refuses before sending a frame - with one deliberate "
            "exception, --raw-send, which is a pipe for a frame you name and therefore writes "
            "whatever you hand it. Both need optional hardware packages (python-can, pyserial)."
        ),
    )
    parser.add_argument("--host", default=DEFAULT_SIL_HOST, help="SIL engine host")
    parser.add_argument("--port", type=int, default=DEFAULT_SIL_PORT, help="SIL engine TCP port")
    parser.add_argument("--interface", default="can0", help="python-can interface for --mode can")
    parser.add_argument("--uart-port", default="/dev/ttyUSB0", help="serial port for --mode uart")
    parser.add_argument("--baudrate", type=int, default=115200, help="baud rate for --mode uart")
    parser.add_argument(
        "--attach-existing",
        action="store_true",
        help=(
            "deliberately drive the engine that is already listening on the port instead of "
            "refusing to touch an engine this run did not start. The default is to refuse, "
            "because every check here actuates and the emergency break latches the brake of "
            "whatever engine it reaches, for the life of that engine. Cannot be combined with "
            "--auto."
        ),
    )

    parser.add_argument("--node1-env", action="store_true", help="monitor 0x210 enclosure telemetry")
    parser.add_argument("--node2-arm", action="store_true", help="verify the 3000 ms ESC arming gate")
    parser.add_argument(
        "--node2-thruster",
        nargs=2,
        type=int,
        metavar=("CHANNEL", "PULSE_US"),
        help="drive one thruster channel and require it to hold",
    )
    parser.add_argument(
        "--node2-all", type=int, metavar="PULSE_US", help="drive all eight thruster channels"
    )
    parser.add_argument(
        "--node2-depth", type=float, metavar="SECONDS", help="monitor 0x200 depth telemetry"
    )
    parser.add_argument(
        "--node2-solenoid",
        type=int_literal,
        metavar="MASK",
        help="actuate a solenoid mask (one coil per valve; decimal or 0x/0o/0b)",
    )
    parser.add_argument("--node3-power", action="store_true", help="monitor 0x300 power telemetry")
    parser.add_argument("--emergency-break", action="store_true", help="trip and verify the emergency brake")
    parser.add_argument(
        "--raw-send",
        action="append",
        type=_raw_frame,
        metavar="ID:HEX",
        help="write one raw frame, for example 0x100:D405DC05DC05DC05DC05DC05DC05DC05",
    )
    parser.add_argument(
        "--sniff", type=float, metavar="SECONDS", help="decode and print every frame for a while"
    )
    parser.add_argument(
        "--auto", action="store_true", help="run the whole vehicle smoke sequence"
    )
    return parser


def target_label(args: argparse.Namespace) -> str:
    """Human-readable description of what this invocation talks to."""
    if args.mode == "sil":
        return f"{args.host}:{args.port}"
    if args.mode == "can":
        return f"interface={args.interface}"
    return f"{args.uart_port} at {args.baudrate} baud"


def run_selected_action(tester: VehicleStimulusTester, args: argparse.Namespace) -> StimulusReport:
    """
    Map every supplied flag to exactly one check and return them all.

    Each dispatch goes through ``tester.guard``, so a check that raises becomes a
    failed entry in the report rather than an exception escaping the CLI - the
    prototype's ``AttributeError: 'Namespace' object has no attribute 'port'``
    must not be able to reach a caller as anything but a reported failure.
    """
    plan: List[Tuple[str, Callable[[], CheckResult]]] = []
    if getattr(args, "node1_env", False):
        plan.append(("node1_env", lambda: tester.monitor_node1_env()))
    if getattr(args, "node2_arm", False):
        plan.append(("node2_arming", tester.test_node2_arming))
    if getattr(args, "node2_thruster", None) is not None:
        channel, pulse_us = args.node2_thruster
        plan.append(
            ("node2_pwm", lambda: tester.test_node2_pwm(int(channel), int(pulse_us)))
        )
    if getattr(args, "node2_all", None) is not None:
        plan.append(("node2_all", lambda: tester.test_node2_all(int(args.node2_all))))
    if getattr(args, "node2_depth", None) is not None:
        plan.append(("node2_depth", lambda: tester.monitor_node2_depth(float(args.node2_depth))))
    if getattr(args, "node2_solenoid", None) is not None:
        plan.append(
            ("node2_solenoid", lambda: tester.test_node2_solenoid(int(args.node2_solenoid)))
        )
    if getattr(args, "node3_power", False):
        plan.append(("node3_power", lambda: tester.monitor_node3_power()))
    if getattr(args, "emergency_break", False):
        plan.append(("emergency_break", tester.test_emergency_break))
    for index, (can_id, payload) in enumerate(getattr(args, "raw_send", None) or []):
        plan.append(
            (f"raw_send_0x{can_id:03X}_{index}", lambda cid=can_id, data=payload: tester.raw_send(cid, data))
        )
    if getattr(args, "sniff", None) is not None:
        plan.append(("sniff", lambda: tester.sniff(float(args.sniff))))
    return tester.run_plan(plan)


def print_report(report: StimulusReport, include_checks: bool = True) -> None:
    """
    The one way the operator-facing report text is produced.

    ``include_checks=False`` is for a caller that has already streamed the
    per-check lines as they completed, which is what ``main`` does; it still
    prints the ``Summary:`` line and the whole failure block.
    """
    print(report.format(include_checks=include_checks))


def selected_action_flags(args: argparse.Namespace) -> List[str]:
    """
    Which action flags were supplied.

    A flag counts as selected unless it is absent or a bare ``False``.  Truthiness
    is the wrong test: ``--sniff 0``, ``--node2-depth 0`` and ``--node2-all 0`` are
    real requests, and reporting them as "no action was selected" would hide the
    operator's intent behind a usage error.  ``run_selected_action`` already uses
    ``is not None``, and the two must agree.
    """
    selected = []
    for flag in ACTION_FLAGS:
        value = getattr(args, flag.lstrip("-").replace("-", "_"), None)
        if value is None or value is False:
            continue
        selected.append(flag)
    return selected
