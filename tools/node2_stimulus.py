#!/usr/bin/env python3
"""
Node 2 stimulus CLI compatibility wrapper over the canonical tool.

``tools/node2_stimulus.py`` used to be an independent prototype, and it was broken
outright: ``--auto`` died on ``AttributeError: 'Namespace' object has no attribute
'port'``, because its parser never declared ``--port`` while its ``main()`` read
``args.port`` - and a sibling script printed a clean summary of that same run. This
file keeps the old command line working and routes it into
``tools/can_stimulus.py``, the one implementation that verifies what it reports.

This file is a translation layer and nothing else: it renames flags, refuses the
ones it cannot honour, and returns the canonical tool's exit status. No packet
format, no backend, no check, no second argparse - the moment one appeared here
there would be two places to keep in agreement about the vehicle. Nor is it a
verdict: it prints no [PASS] and no [FAIL] of its own, and the status is
``can_stimulus``'s unchanged (0 only when every selected check passed, 1 for a
failed check or an unreachable engine, 2 for a usage error). The old prototype
returned 0 no matter what, and swallowing a nonzero here would reproduce that
defect one layer up.

What each legacy flag does now is written once, in ``LEGACY_NOTES``; ``--help`` is
generated from it, so the help an operator reads cannot drift from the table this
file translates.

Behaviour changes an operator must know about:

* ``--test-emergency`` is no longer a monitor. The prototype spun thrusters at
  1650 us and then transmitted ``b"\\x00" * 8`` on CAN 0x001 - an UNAUTHORIZED
  payload, which ``node2_control_board/Core/Src/app.c:112`` ignores because it does
  not begin ``0xAA 0x55`` - and printed [PASS] from a PWM value that was already
  neutral, so the text evidenced nothing. ``--emergency-break`` arranges its own
  precondition: it drives all eight thrusters to 1800 us, ramps them back to prove
  the board was ESC ACTIVE, sends the authorized ``AA 55 01`` frame, and requires
  the latch to hold. It actuates every thruster, costs a few seconds (more on a
  fresh engine, which must also wait out the 3000 ms arming window first), and
  latches the brake for the life of the engine.
* ``--test-arm`` legitimately FAILS against a warm engine: the 3000 ms arming
  window closes on the engine's virtual clock whether or not anyone watches, so the
  gate is observable only on a fresh engine.
* ``--test-solenoid`` takes a DECIMAL mask. The prototype parsed with ``int(x, 0)``,
  so ``0x0001`` worked; the canonical parser uses ``type=int`` and rejects a hex
  literal with exit 2. The wrapper does not re-parse values to hide this - a second
  parser is the divergence this work exists to remove, and a loud exit 2 cannot be
  mistaken for a pass.
* ``--bitrate``, ``--data-bitrate`` and ``--channel`` are refused, not dropped: the
  canonical tool never configures bus bitrates, and its single ``--interface`` cannot
  carry both an interface and a channel. A silently dropped flag leaves the operator
  believing a check ran, which is the defect this file exists to end.
"""

from __future__ import annotations

import argparse
import sys
import textwrap
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence

# can_stimulus is a sibling script, not an installed package. Reaching it by
# absolute path from __file__ rather than the working directory is what makes this
# import identically as `python tools/node2_stimulus.py` and `from tools import
# node2_stimulus`.
TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import can_stimulus  # noqa: E402  (path setup must precede the import)

__all__ = [
    "ACTUATING_LEGACY_FLAGS",
    "LEGACY_ACTION_FLAGS",
    "LEGACY_FLAGS",
    "LEGACY_NOTES",
    "UNSUPPORTED_LEGACY_FLAGS",
    "UnsupportedLegacyFlag",
    "main",
    "translate_legacy_args",
]

# The prototype's action flags, in its parser's order. --auto is absent on purpose:
# it is spelled the same in both tools, so it passes through untouched.
LEGACY_ACTION_FLAGS = (
    "--test-thruster", "--test-all", "--test-arm",
    "--test-depth", "--test-solenoid", "--test-emergency",
)

# Legacy flag -> canonical flag. Every entry was checked against the real
# build_parser(), arity included: the prototype took 2 values for --test-thruster, 1
# for --test-all/--test-depth/--test-solenoid/--serial-port and none for
# --test-arm/--test-emergency, and so do their canonical flags.
LEGACY_FLAGS: Dict[str, str] = {
    "--test-thruster": "--node2-thruster",  # CHANNEL PULSE_US
    "--test-all": "--node2-all",  # PULSE_US
    "--test-arm": "--node2-arm",  # no value
    "--test-depth": "--node2-depth",  # SECONDS
    "--test-solenoid": "--node2-solenoid",  # MASK, decimal
    "--test-emergency": "--emergency-break",  # no value; ACTUATES
    "--serial-port": "--uart-port",  # rename only; the "auto" default is gone
}

# What each of those does now, in the operator's terms. This is the single source of
# the --help table, and where a behaviour change has to be written down.
LEGACY_NOTES: Dict[str, str] = {
    "--test-thruster": (
        "Drives one thruster channel to PULSE_US and requires it to HOLD past the "
        "100 ms watchdog with the other seven at 1500 us, then releases to neutral. "
        "ACTUATES A THRUSTER."
    ),
    "--test-all": (
        "Drives all eight channels, requires the whole vector to hold, then releases. "
        "ACTUATES ALL EIGHT THRUSTERS."
    ),
    "--test-arm": (
        "Proves the mandatory 3000 ms ESC arming gate held neutral while a 1700 us "
        "command was resent, then took effect once it closed. ACTUATES A THRUSTER, and "
        "it FAILS against an engine already past 3000 ms of virtual time: the window "
        "cannot be observed retroactively, so run it first, on a fresh engine."
    ),
    "--test-depth": (
        "Monitors 0x200 depth telemetry. Receive-only: actuates nothing. A legacy "
        "--test-depth 0 fell through to the old interactive menu; it is now a real "
        "request that fails on a 0-second window."
    ),
    "--test-solenoid": (
        "Energises a solenoid mask, requires the readback to CHANGE, then releases it. "
        "ACTUATES A PNEUMATIC VALVE. Decimal only: 0x0001 worked in the prototype and "
        "is a usage error here (exit 2)."
    ),
    "--test-emergency": (
        "THE BIGGEST CHANGE. The prototype spun thrusters at 1650 us then sent an "
        "UNAUTHORIZED 0x001 payload (00 00 ...), which node 2 ignores, so its [PASS] "
        "text proved nothing. This drives all eight thrusters to 1800 us and ramps them "
        "back to earn the evidence that the board was ESC ACTIVE, sends the AUTHORIZED "
        "AA 55 01 frame, and requires the latch to hold. ACTUATES ALL EIGHT THRUSTERS, "
        "costs a few seconds (more on a fresh engine, where it must also wait out the "
        "3000 ms arming window first), and latches the brake for the life of the engine."
    ),
    "--serial-port": (
        "Renamed. The prototype's default \"auto\" (ST-Link VCP auto-detection) does not "
        "exist here: pass an explicit port, or omit the flag."
    ),
}

# The legacy flags that move something, used to say so on stderr. --test-depth and
# --serial-port are receive-only and are told so instead.
ACTUATING_LEGACY_FLAGS = frozenset(
    {"--test-thruster", "--test-all", "--test-arm", "--test-solenoid", "--test-emergency"}
)

# Legacy flags with no canonical counterpart, and why. Refused, never dropped.
UNSUPPORTED_LEGACY_FLAGS: Dict[str, str] = {
    "--bitrate": "the canonical tool never configures CAN bus bitrates; set them on the "
                 "interface itself and re-run without it",
    "--data-bitrate": "nor the CAN FD data bitrate; set it on the interface itself",
    "--channel": "the canonical tool takes one --interface naming both the interface and "
                 "the channel, so two settings cannot both survive; pass --interface can0",
}

_EPILOG_WIDTH = 76


class UnsupportedLegacyFlag(ValueError):
    """A legacy flag the canonical tool cannot honour. Refused, never dropped."""


def translate_legacy_args(argv: Sequence[str]) -> List[str]:
    """
    Rename the legacy flags in ``argv`` and return the canonical command line.

    Values are never inspected, reordered or validated: that is the canonical
    parser's job, and a second opinion here would be a second parser. Only the flag
    token is touched, including in the ``--flag=value`` spelling argparse also
    accepts, which would otherwise reach it unrecognised.

    Raises :class:`UnsupportedLegacyFlag` for a legacy flag with no honest
    translation. Failing loudly is the point: a dropped flag is indistinguishable
    from a flag that quietly did nothing.
    """
    translated: List[str] = []
    for arg in argv:
        flag, separator, inline = str(arg).partition("=")
        if flag in UNSUPPORTED_LEGACY_FLAGS:
            raise UnsupportedLegacyFlag(
                f"[FAIL] {flag} is not supported: {UNSUPPORTED_LEGACY_FLAGS[flag]}."
            )
        canonical = LEGACY_FLAGS.get(flag)
        if canonical is None:
            translated.append(str(arg))
        else:
            translated.append(f"{canonical}{separator}{inline}" if separator else canonical)
    return translated


def _warn_about_legacy_flags(argv: Sequence[str]) -> None:
    """
    Name the canonical flag each legacy flag became, on stderr, with no verdict.

    stderr, so the canonical report on stdout stays exactly what the tool that
    earned it printed. ``dict.fromkeys`` keeps the first occurrence of each flag and
    its order, so a flag given twice is announced once.
    """
    for flag in dict.fromkeys(str(a).partition("=")[0] for a in argv):
        if flag in LEGACY_FLAGS:
            actuation = (
                ", and may ACTUATE thrusters to 1800 us or energise a valve"
                if flag in ACTUATING_LEGACY_FLAGS
                else ", and actuates nothing"
            )
            print(
                f"[COMPAT] {flag} is a legacy flag; it now runs {LEGACY_FLAGS[flag]}"
                f"{actuation}. See 'python tools/node2_stimulus.py --help'.",
                file=sys.stderr,
            )


def _build_legacy_epilog() -> str:
    """Render the legacy table for --help out of LEGACY_FLAGS and LEGACY_NOTES."""
    lines = [
        "",
        "legacy Node 2 flags accepted here, and what each one does now. Each is",
        "translated to the flag beside it and verified by tools/can_stimulus.py:",
        "",
    ]
    for legacy in LEGACY_ACTION_FLAGS + ("--serial-port",):
        lines.append(f"  {legacy} -> {LEGACY_FLAGS[legacy]}")
        lines.append(textwrap.fill(LEGACY_NOTES[legacy], width=_EPILOG_WIDTH,
                                   initial_indent="      ", subsequent_indent="      "))
        lines.append("")
    lines.append("refused with exit 2 rather than dropped:")
    lines += [textwrap.fill(f"  {flag}: {reason}", width=_EPILOG_WIDTH,
                            subsequent_indent="      ")
              for flag, reason in sorted(UNSUPPORTED_LEGACY_FLAGS.items())]
    lines.append("")
    lines.append(textwrap.fill(
        "Exit status is the canonical tool's, unchanged: 0 only when every selected check "
        "passed, 1 for a failed check or an unreachable engine, 2 for a usage error. This "
        "wrapper prints no verdict of its own.", width=_EPILOG_WIDTH))
    return "\n".join(lines)


def _parser_with_legacy_epilog(original: Callable[[], argparse.ArgumentParser]) -> Callable:
    """
    Build a replacement for ``can_stimulus.build_parser`` carrying the legacy table.

    The parser comes from ``original``, never from the module global: that global is
    about to BE this function, so looking it up again would recurse until the stack ran
    out. Re-declaring the options here instead would be a second parser, so the real one
    is reused and only its epilog is added. RawDescriptionHelpFormatter is required
    because argparse's default collapses an epilog's line breaks into one wrapped
    paragraph, which would destroy the table.
    """

    def build() -> argparse.ArgumentParser:
        parser = original()
        parser.formatter_class = argparse.RawDescriptionHelpFormatter
        parser.epilog = _build_legacy_epilog()
        return parser

    return build


def main(argv: Optional[Sequence[str]] = None) -> int:
    """
    Translate, delegate, and return the canonical tool's status.

    The only status produced here is 2 for a refused legacy flag. Everything else is
    ``can_stimulus.main``'s, including the 2 argparse raises as a SystemExit for a bad
    command line and the 0 it raises for --help.
    """
    raw = list(sys.argv[1:] if argv is None else argv)
    try:
        translated = translate_legacy_args(raw)
    except UnsupportedLegacyFlag as exc:
        print(str(exc))
        return 2
    _warn_about_legacy_flags(raw)

    original_build_parser = can_stimulus.build_parser
    can_stimulus.build_parser = _parser_with_legacy_epilog(original_build_parser)
    try:
        return can_stimulus.main(translated)
    except SystemExit as exc:
        # main() may only return an int, and a non-integer status cannot be a pass.
        return 0 if exc.code is None else (exc.code if isinstance(exc.code, int) else 1)
    finally:
        can_stimulus.build_parser = original_build_parser


if __name__ == "__main__":
    sys.exit(main())
