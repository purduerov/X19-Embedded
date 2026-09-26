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
verdict: it prints no check verdict of its own on stdout - no [PASS], no [FAIL],
no Summary - and the status is ``can_stimulus``'s unchanged, coerced by
``_status`` so that only an explicit integer can become 0 (0 only when every
selected check passed, 1 for a failed check or an unreachable engine, 2 for a
usage error). The old prototype returned 0 no matter what, and swallowing a
nonzero here would reproduce that defect one layer up. The one thing this file
does print, on stderr, is a refusal to translate a command line at all, which is
a usage error and not a statement about the vehicle.

Each flag's description lives once, in ``LEGACY_NOTES``, and ``--help`` is generated
from it, so the help an operator reads cannot drift from the table this file
translates. The summary below is prose, not a second source: it is pinned by
test so it cannot silently drop a flag that actuates something.

Behaviour changes an operator must know about. Every flag named in this section
actuates part of the vehicle except where it says otherwise:

* ``--test-thruster`` and ``--test-all`` now require the commanded pulse to HOLD
  past the 100 ms watchdog with the other channels at 1500 us, then release to
  neutral, rather than drive for a fixed time and print whatever it liked. Both
  ACTUATE thrusters.
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
  gate is observable only on a fresh engine. It drives a 1700 us command, so it
  actuates a thruster.
* ``--test-solenoid`` energises a valve and ACTUATES it. It takes a DECIMAL mask:
  the prototype parsed with ``int(x, 0)``, so ``0x0001`` worked; the canonical
  parser uses ``type=int`` and rejects a hex literal with exit 2. The wrapper does
  not re-parse values to hide this - a second parser is the divergence this work
  exists to remove, and a loud exit 2 cannot be mistaken for a pass.
* ``--test-depth`` actuates nothing; it only listens.
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
from typing import Dict, List, Optional, Sequence

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


def _status(value: object) -> int:
    """
    Coerce a status this wrapper did not author into an exit status.

    Only an explicit integer is trusted, and ``bool`` is excluded even though it is
    an ``int`` subclass, because ``False`` would otherwise become exit 0. Everything
    else becomes 1: a failure, never a pass.

    The case that matters is ``None``. The prototype this file replaces declared
    ``def main():`` with no ``return`` statement anywhere, so it returned None on
    every path - and ``sys.exit(None)`` exits 0. A canonical tool that ever
    regressed to that shape would have every failed check reported as a pass, and
    this function is the last thing standing between that and a green build.
    """
    return value if isinstance(value, int) and not isinstance(value, bool) else 1


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
            # No [FAIL] tag and no Summary: a verdict is the canonical tool's to
            # print, and this file originates none. This is a refusal to translate a
            # command line at all, which is a usage error, not a statement about the
            # vehicle - and main() sends it to stderr for the same reason it sends
            # the migration notice there.
            raise UnsupportedLegacyFlag(
                f"{flag} is not supported: {UNSUPPORTED_LEGACY_FLAGS[flag]}."
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
            # No pulse width here. The notice is printed for every actuating flag,
            # and 1800 us is true of --test-emergency alone: --test-thruster drives
            # to whatever the operator asked for. Over-warning is harmless, a specific
            # wrong number is not, so it points at --help for the real figure.
            actuation = (
                ", and may ACTUATE thrusters or energise a valve"
                if flag in ACTUATING_LEGACY_FLAGS
                else ", and actuates nothing"
            )
            print(
                f"[COMPAT] {flag} is a legacy flag; it now runs {LEGACY_FLAGS[flag]}"
                f"{actuation}. See 'python tools/node2_stimulus.py --help'.",
                file=sys.stderr,
            )


def _build_legacy_epilog() -> str:
    """
    Render the legacy table for --help out of LEGACY_FLAGS and LEGACY_NOTES.

    Iterating LEGACY_FLAGS rather than a separate list is the point: a hand-kept
    list can fall behind the table, and an eighth mapping would then ship with a note
    and no help entry.
    """
    lines = [
        "",
        "legacy Node 2 flags accepted here, and what each one does now. Each is",
        "translated to the flag beside it and verified by tools/can_stimulus.py:",
        "",
    ]
    for legacy, canonical in LEGACY_FLAGS.items():
        lines.append(f"  {legacy} -> {canonical}")
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
        "wrapper prints no check verdict of its own; the only diagnostic it can add is a "
        "refusal to translate a command line, on stderr.", width=_EPILOG_WIDTH))
    return "\n".join(lines)


def _parser_with_legacy_epilog() -> argparse.ArgumentParser:
    """
    The canonical parser, carrying this program's name and the legacy table.

    The options come from ``can_stimulus.build_parser`` and are never re-declared
    here: a second parser is the exact divergence this file exists to remove, and
    it would drift the moment the canonical tool gained a flag.  What the wrapper
    owns is the presentation - its own ``prog``, and the legacy-to-canonical table
    under the canonical flags - and ``build_parser`` takes both as keyword
    arguments precisely so it can own them.

    RawDescriptionHelpFormatter comes from that same call, because argparse's
    default collapses an epilog's line breaks into one wrapped paragraph, which
    would destroy the table.  It is requested only when there is an epilog, so the
    canonical tool's own ``--help`` keeps the default formatter.

    ``prog`` is the program the operator actually ran, not this file's import path:
    ``can_stimulus.build_parser`` hard-codes its own name rather than deriving one
    from ``sys.argv[0]``, which for a direct run would be this file and for
    ``from tools import node2_stimulus`` would be ``-c``.  Pinning it here means
    the usage line and every argparse error name the program that was invoked
    instead of ``can_stimulus.py``.
    """
    return can_stimulus.build_parser(prog="node2_stimulus.py", epilog=_build_legacy_epilog())


def main(argv: Optional[Sequence[str]] = None) -> int:
    """
    Translate, delegate, and return the canonical tool's status.

    The only status produced here is 2 for a refused legacy flag. Everything else is
    ``can_stimulus.main``'s, including the 2 argparse raises as a SystemExit for a bad
    command line and the 0 it raises for --help - and every status that arrives from
    the canonical tool goes through :func:`_status` first, so a None or a non-integer
    can never be laundered into a pass.

    The parser is configured by handing ``can_stimulus.main`` a ``parser_factory``,
    which is the same injection seam it already exposes for the transport.  Nothing
    in this module is monkey-patched.  An earlier version swapped
    ``can_stimulus.build_parser`` for the duration of the call and restored it in a
    ``finally``.  Nothing was broken - a CLI is single-threaded and the restore always
    ran - but it was the one place this translation layer reached into the canonical
    tool's state, so a concurrent canonical invocation in the same process would have
    inherited the wrong ``prog`` and epilog.  The kwargs are three lines; the
    patch-and-restore was a coupling nobody had to have.
    """
    raw = list(sys.argv[1:] if argv is None else argv)
    try:
        translated = translate_legacy_args(raw)
    except UnsupportedLegacyFlag as exc:
        print(str(exc), file=sys.stderr)
        return 2
    _warn_about_legacy_flags(raw)

    try:
        return _status(can_stimulus.main(translated, parser_factory=_parser_with_legacy_epilog))
    except SystemExit as exc:
        # main() may only return an int, so argparse's exit becomes one. A bare
        # exit() carries no status and Python's own convention for that is 0; that is
        # the one place a missing value legitimately means success, and it is
        # deliberately NOT the rule for a return value.
        return 0 if exc.code is None else _status(exc.code)


if __name__ == "__main__":
    sys.exit(main())
