"""
Tests for the Node 2 stimulus compatibility wrapper (``tools/node2_stimulus.py``).

The wrapper exists because ``tools/node2_stimulus.py`` was a divergent prototype
whose ``--auto`` run died on ``AttributeError: 'Namespace' object has no
attribute 'port'`` while a sibling script claimed success.  The defect class this
file pins is therefore not "does a check pass" but "can the wrapper lie":

* it must translate the legacy flags and nothing else - no packet format, no
  backend, no check, no second argparse;
* it must hand the canonical tool its own exit code, unchanged.  A wrapper that
  maps a failing run to 0 would recreate the original defect one layer up, and
  that is the single most important test in this file;
* it must not print a verdict of its own.  Every ``[PASS]``/``[FAIL]`` line an
  operator sees has to come from ``tools/can_stimulus.py``, which is the thing
  that earned it;
* a legacy flag it cannot honour must be REFUSED, never silently dropped - a
  dropped flag leaves the operator believing a check ran;
* the translation table must still match the real parser.  ``test_table_matches_
  the_canonical_parser`` reads ``build_parser()`` and fails if a canonical flag is
  ever renamed out from under the table.

No test here starts an engine or binds the default port 8765.  The subprocess
cases only exercise ``--help`` and the usage-rejection paths, both of which return
before a transport is built.

Run from the repository root::

    python -m unittest tests.sil_stimulus.test_stimulus_cli -v
"""

from __future__ import annotations

import contextlib
import io
import os
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from typing import List, Optional
from unittest import mock

# tests/sil_stimulus/test_stimulus_cli.py -> tests/sil_stimulus -> tests -> repo root
REPO_ROOT = Path(__file__).resolve().parents[2]
WRAPPER_PATH = REPO_ROOT / "tools" / "node2_stimulus.py"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

# `can_stimulus` (imported here as tools.can_stimulus) and the `can_stimulus` the
# wrapper holds are TWO DIFFERENT module objects: the wrapper reaches its sibling by
# absolute path from __file__, so it registers under the top-level name, and patching
# one does not touch the other. Every patch in this file therefore goes through
# `node2_stimulus.can_stimulus`, and
# test_the_two_can_stimulus_module_objects_are_distinct pins the distinction.
from tools import can_stimulus, node2_stimulus  # noqa: E402  (path setup must precede it)
from tools.node2_stimulus import (  # noqa: E402
    ACTUATING_LEGACY_FLAGS,
    LEGACY_ACTION_FLAGS,
    LEGACY_FLAGS,
    LEGACY_NOTES,
    UNSUPPORTED_LEGACY_FLAGS,
    UnsupportedLegacyFlag,
    _build_legacy_epilog,
    main,
    translate_legacy_args,
)

SUBPROCESS_TIMEOUT_S = 90


# --------------------------------------------------------------------------
# Fakes
# --------------------------------------------------------------------------
class FakeCanonicalMain:
    """
    Stand-in for ``can_stimulus.main``.

    Records every argv it was handed so a test can assert the wrapper *delegated*
    rather than reimplemented, and returns (or raises) whatever the test needs in
    order to pin the exit code the wrapper must propagate.
    """

    def __init__(
        self,
        code: object = 0,
        raises: Optional[BaseException] = None,
        prints: str = "",
    ) -> None:
        self.code = code
        self.raises = raises
        self.prints = prints
        self.calls: List[Optional[List[str]]] = []

    def __call__(self, argv=None, backend_factory=None) -> int:
        self.calls.append(None if argv is None else list(argv))
        if self.raises is not None:
            raise self.raises
        if self.prints:
            print(self.prints)
        return self.code


@contextlib.contextmanager
def canonical_main_is(fake: FakeCanonicalMain):
    """Point the wrapper at a fake canonical tool for the duration of the block."""
    with mock.patch.object(node2_stimulus.can_stimulus, "main", fake):
        yield fake


@contextlib.contextmanager
def captured_output() -> "io.StringIO":
    """Capture stdout and stderr as two StringIOs yielded as a pair."""
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        yield out, err


def canonical_action(option: str):
    """The argparse action the canonical parser really declares for ``option``."""
    for action in can_stimulus.build_parser()._actions:
        if option in action.option_strings:
            return action
    raise AssertionError(f"the canonical parser has no {option}")


def no_autostart_env() -> "dict[str, str]":
    """
    An environment in which the SIL engine cannot possibly be launched.

    ``sil_test_support.server_executable()`` honours ``X19_SIL_SERVER`` and returns
    None when the path is not a file, so pointing it at a name that cannot exist
    makes auto-start IMPOSSIBLE rather than merely unlikely. Without this, a future
    refactor that moved the no-action check after ``backend_factory(args)`` would
    spawn and reap a real engine on port 8765 during a test that was asserting no
    engine had started - and a stdout text assertion would not have noticed.
    """
    return {**os.environ, "X19_SIL_SERVER": str(REPO_ROOT / "no-such-sil-bridge-server")}


def run_wrapper(*args: str, cwd: Optional[Path] = None) -> subprocess.CompletedProcess:
    """Run the wrapper the way an operator does, with sys.executable and no engine."""
    try:
        return subprocess.run(
            [sys.executable, str(WRAPPER_PATH), *args],
            cwd=str(cwd or REPO_ROOT),
            capture_output=True,
            text=True,
            timeout=SUBPROCESS_TIMEOUT_S,
            env=no_autostart_env(),
        )
    except OSError as exc:  # pragma: no cover - interpreter cannot be launched
        raise unittest.SkipTest(f"could not launch {sys.executable} as a subprocess: {exc}")


# --------------------------------------------------------------------------
# The translation table
# --------------------------------------------------------------------------
class TestLegacyFlagTranslation(unittest.TestCase):
    """Every legacy flag must reach the canonical tool under its canonical name."""

    def test_every_legacy_flag_translates(self):
        for legacy, expected in (
            (["--test-thruster", "0", "1600"], ["--node2-thruster", "0", "1600"]),
            (["--test-all", "1650"], ["--node2-all", "1650"]),
            (["--test-arm"], ["--node2-arm"]),
            (["--test-depth", "2.5"], ["--node2-depth", "2.5"]),
            (["--test-solenoid", "1"], ["--node2-solenoid", "1"]),
            (["--test-emergency"], ["--emergency-break"]),
        ):
            with self.subTest(legacy=legacy):
                self.assertEqual(translate_legacy_args(legacy), expected)

    def test_multi_argument_flag_keeps_both_values_in_order(self):
        # --node2-thruster is nargs=2: the channel and the pulse must not be
        # reordered, dropped, or merged.
        self.assertEqual(
            translate_legacy_args(["--test-thruster", "7", "2000"]),
            ["--node2-thruster", "7", "2000"],
        )
        self.assertEqual(
            translate_legacy_args(["--test-thruster", "0", "1000"]),
            ["--node2-thruster", "0", "1000"],
        )

    def test_values_are_preserved_verbatim(self):
        """
        Zero, negative, and fractional values survive untouched.

        The wrapper must not validate values: deciding that 0 is nonsense, or
        that -1 is out of range, is the canonical tool's job and its verdict is
        the one that counts.  Filtering here would also mean a second parser.
        """
        for legacy in (
            ["--test-all", "0"],
            ["--test-depth", "0"],
            ["--test-solenoid", "0"],
            ["--test-solenoid", "-1"],
            ["--test-thruster", "-1", "1500"],
            ["--test-depth", "0.0"],
            ["--test-thruster", "0", "1500"],
        ):
            with self.subTest(legacy=legacy):
                self.assertEqual(
                    translate_legacy_args(legacy), [LEGACY_FLAGS[legacy[0]], *legacy[1:]]
                )

    def test_inline_value_spellings_are_translated_too(self):
        # argparse accepts --flag=value, so the wrapper must translate the flag
        # part of that spelling as well or the operator gets a confusing
        # "unrecognized arguments" for a flag that does exist.
        self.assertEqual(
            translate_legacy_args(["--test-all=1650"]), ["--node2-all=1650"]
        )
        self.assertEqual(
            translate_legacy_args(["--test-emergency=1"]), ["--emergency-break=1"]
        )

    def test_canonical_and_unrelated_flags_pass_through_untouched(self):
        for argv in (
            ["--node2-thruster", "0", "1600"],
            ["--node1-env"],
            ["--node3-power"],
            ["--node2-solenoid", "1"],
            ["--emergency-break"],
            ["--raw-send", "0x100:AABB"],
            ["--sniff", "0"],
            ["--help"],
            ["-h"],
        ):
            with self.subTest(argv=argv):
                self.assertEqual(translate_legacy_args(argv), list(argv))

    def test_transport_flags_pass_through_untouched(self):
        for argv in (
            ["--mode", "sil"],
            ["--host", "10.0.0.5"],
            ["--port", "9000"],
            ["--port=9000"],
            ["--interface", "socketcan"],
            ["--baudrate", "9600"],
            ["--uart-port", "COM3"],
            ["--auto"],
        ):
            with self.subTest(argv=argv):
                self.assertEqual(translate_legacy_args(argv), list(argv))

    def test_legacy_and_canonical_flags_mix_without_reordering(self):
        self.assertEqual(
            translate_legacy_args(
                ["--mode", "sil", "--test-thruster", "3", "1400", "--sniff", "1"]
            ),
            ["--mode", "sil", "--node2-thruster", "3", "1400", "--sniff", "1"],
        )

    def test_serial_port_is_renamed_to_uart_port(self):
        self.assertEqual(
            translate_legacy_args(["--serial-port", "COM3"]), ["--uart-port", "COM3"]
        )

    def test_translation_is_pure(self):
        # The caller's list must not be mutated and the function must be
        # repeatable: main() translates once and hands the result to argparse.
        argv = ["--test-emergency", "--port", "8765"]
        first = translate_legacy_args(argv)
        self.assertEqual(argv, ["--test-emergency", "--port", "8765"])
        self.assertEqual(first, ["--emergency-break", "--port", "8765"])
        self.assertEqual(translate_legacy_args(argv), first)

    def test_empty_argv_translates_to_empty_argv(self):
        self.assertEqual(translate_legacy_args([]), [])


class TestUnsupportedLegacyFlags(unittest.TestCase):
    """A flag the wrapper cannot honour must be refused loudly, never dropped."""

    def test_unsupported_flags_raise(self):
        for argv in (["--bitrate", "1000000"], ["--data-bitrate", "5000000"], ["--channel", "can0"]):
            with self.subTest(argv=argv):
                with self.assertRaises(UnsupportedLegacyFlag):
                    translate_legacy_args(argv)

    def test_inline_unsupported_flags_raise(self):
        with self.assertRaises(UnsupportedLegacyFlag):
            translate_legacy_args(["--channel=can0"])

    def test_refusal_names_the_flag_and_explains_the_replacement(self):
        with self.assertRaises(UnsupportedLegacyFlag) as caught:
            translate_legacy_args(["--channel", "can0"])
        message = str(caught.exception)
        self.assertIn("--channel", message)
        self.assertIn("--interface", message)

    def test_unsupported_flag_message_says_why_there_is_no_equivalent(self):
        for flag in ("--bitrate", "--data-bitrate"):
            with self.subTest(flag=flag):
                with self.assertRaises(UnsupportedLegacyFlag) as caught:
                    translate_legacy_args([flag, "500000"])
                self.assertIn(flag, str(caught.exception))

    def test_unsupported_flag_exits_two_without_running_the_canonical_tool(self):
        fake = FakeCanonicalMain(code=0)
        with canonical_main_is(fake), captured_output() as (out, err):
            self.assertEqual(main(["--channel", "can0", "--test-arm"]), 2)
        self.assertEqual(fake.calls, [], "the canonical tool must not run a refused command line")
        # The diagnostic goes to stderr so stdout stays exactly what the tool that
        # earned it printed, and it carries no [PASS]/[FAIL] tag: a verdict is the
        # canonical tool's to print, and this wrapper originates no check verdict.
        self.assertIn("--channel", err.getvalue())
        self.assertIn("--interface", err.getvalue())
        self.assertEqual(out.getvalue(), "")

    def test_the_refusal_diagnostic_is_not_tagged_as_a_verdict(self):
        # The prototype's whole failure was text that looked like a result. A
        # refusal must not borrow that vocabulary, in either direction.
        with self.assertRaises(UnsupportedLegacyFlag) as caught:
            translate_legacy_args(["--bitrate", "1000000"])
        message = str(caught.exception)
        self.assertNotIn("[FAIL]", message)
        self.assertNotIn("[PASS]", message)
        self.assertNotIn("Summary", message)

    def test_unsupported_flag_is_refused_even_alongside_valid_legacy_flags(self):
        # The operator asked for two things and one of them cannot be honoured.
        # Refusing the whole command line is the only honest outcome: running
        # the half we can would leave the dropped half looking like it ran.
        fake = FakeCanonicalMain(code=0)
        with canonical_main_is(fake), captured_output():
            self.assertEqual(main(["--test-arm", "--bitrate", "1000000"]), 2)
        self.assertEqual(fake.calls, [])


# --------------------------------------------------------------------------
# The table against the real parser
# --------------------------------------------------------------------------
class TestTableMatchesCanonicalParser(unittest.TestCase):
    """
    The table is only correct while the canonical parser still has these flags.

    A renamed or re-typed canonical flag would otherwise leave the wrapper
    forwarding a name argparse no longer knows, and the operator would be the one
    to find out.
    """

    def test_every_mapped_flag_is_a_real_canonical_option(self):
        for legacy, canonical in sorted(LEGACY_FLAGS.items()):
            with self.subTest(legacy=legacy):
                self.assertIn(
                    canonical,
                    [
                        option
                        for action in can_stimulus.build_parser()._actions
                        for option in action.option_strings
                    ],
                    f"{legacy} maps to {canonical}, which build_parser() does not declare",
                )

    def test_mapped_flags_consume_the_same_number_of_values_as_the_prototype_did(self):
        # Arity taken from the deleted prototype's parser (tools/node2_stimulus.py
        # in the pre-unification tree), cross-checked against the canonical one.
        legacy_arity = {
            "--test-thruster": 2,
            "--test-all": 1,
            "--test-arm": 0,
            "--test-depth": 1,
            "--test-solenoid": 1,
            "--test-emergency": 0,
            "--serial-port": 1,
        }
        self.assertEqual(set(legacy_arity), set(LEGACY_FLAGS))
        for legacy, arity in sorted(legacy_arity.items()):
            with self.subTest(legacy=legacy):
                action = canonical_action(LEGACY_FLAGS[legacy])
                self.assertEqual(
                    {0: 0, 1: None, 2: 2}[arity],
                    action.nargs,
                    f"{legacy} took {arity} value(s) but {action.option_strings} takes {action.nargs}",
                )

    def test_numeric_legacy_flags_still_reach_a_numeric_canonical_option(self):
        for legacy, expected_type in (
            ("--test-thruster", int),
            ("--test-all", int),
            ("--test-depth", float),
        ):
            with self.subTest(legacy=legacy):
                self.assertIs(canonical_action(LEGACY_FLAGS[legacy]).type, expected_type)

    def test_the_mask_flag_reaches_the_prefix_aware_parser(self):
        # Not int: that is the form that rejected 0x0001. Not int(x, 0) either:
        # that one accepts 0x but newly rejects a zero-padded decimal, trading the
        # old break for a new one.
        self.assertIs(
            canonical_action(LEGACY_FLAGS["--test-solenoid"]).type,
            can_stimulus.int_literal,
        )

    def test_boolean_legacy_flags_map_to_boolean_canonical_options(self):
        for legacy in ("--test-arm", "--test-emergency"):
            with self.subTest(legacy=legacy):
                self.assertEqual(canonical_action(LEGACY_FLAGS[legacy]).nargs, 0)

    def test_no_unsupported_flag_is_actually_supported(self):
        for flag in UNSUPPORTED_LEGACY_FLAGS:
            with self.subTest(flag=flag):
                self.assertNotIn(
                    flag,
                    [
                        option
                        for action in can_stimulus.build_parser()._actions
                        for option in action.option_strings
                    ],
                    f"{flag} is refused but the canonical parser accepts it; drop the refusal",
                )

    def test_notes_cover_every_mapped_flag_exactly_once(self):
        # LEGACY_NOTES is what --help is generated from, so a flag that translates
        # without a note would be one an operator gets no explanation for.
        self.assertEqual(sorted(LEGACY_NOTES), sorted(LEGACY_FLAGS))

    def test_the_epilog_renders_every_mapped_flag(self):
        """
        The rendered --help must carry an entry for every flag in the table.

        Key-set equality alone was not enough: the epilog used to iterate a separate
        hand-maintained list, so a new mapping could ship with a note and no help
        entry and no test would fail. It is generated from LEGACY_FLAGS now, and this
        asserts the rendered text, not the source list.
        """
        epilog = _build_legacy_epilog()
        for legacy, canonical in sorted(LEGACY_FLAGS.items()):
            with self.subTest(legacy=legacy):
                self.assertIn(legacy, epilog)
                self.assertIn(canonical, epilog)

    def test_the_module_docstring_names_every_actuating_legacy_flag(self):
        """
        The docstring is a second, hand-maintained copy of the disclosure.

        It cannot be generated into a module docstring, so instead it is pinned: every
        flag that actuates something must be named there, or an operator reading only
        the docstring would be left to discover the change by running it.
        """
        for flag in sorted(ACTUATING_LEGACY_FLAGS):
            with self.subTest(flag=flag):
                self.assertIn(flag, node2_stimulus.__doc__)

    def test_the_module_docstring_states_the_exit_code_contract(self):
        self.assertIn("exit status", node2_stimulus.__doc__.lower())

    def test_every_legacy_action_flag_is_in_the_table(self):
        # The prototype's action flags, from its parser. --auto is deliberately
        # absent: it is spelled the same in both tools, so translating it would
        # be noise, and a wrapper that renamed it would break scripts.
        self.assertEqual(
            sorted(LEGACY_ACTION_FLAGS),
            [
                "--test-all",
                "--test-arm",
                "--test-depth",
                "--test-emergency",
                "--test-solenoid",
                "--test-thruster",
            ],
        )
        self.assertNotIn("--auto", LEGACY_FLAGS)

    def test_emergency_flag_maps_to_the_break_check_not_a_monitor(self):
        # The prototype's test_emergency_cutoff was effectively a monitor: it
        # sent an UNAUTHORIZED payload that node 2 ignores, then printed [PASS]
        # from a PWM value that was already neutral. The canonical check actually
        # actuates and actually trips, so the mapping must reach it.
        action = canonical_action(LEGACY_FLAGS["--test-emergency"])
        self.assertIn("--emergency-break", action.option_strings)


# --------------------------------------------------------------------------
# Exit-code propagation: the load-bearing tests
# --------------------------------------------------------------------------
class TestExitCodePropagation(unittest.TestCase):
    """
    The wrapper must return the canonical tool's exit code, exactly.

    This is the whole reason the wrapper exists.  The prototype exited 0 while
    its own checks were raising; a wrapper that swallowed a nonzero would
    reproduce that defect with better manners.
    """

    def test_failing_canonical_report_exits_nonzero(self):
        fake = FakeCanonicalMain(code=1)
        with canonical_main_is(fake), captured_output() as (out, _err):
            self.assertEqual(main(["--test-thruster", "0", "1600"]), 1)
        self.assertEqual(len(fake.calls), 1)
        self.assertEqual(out.getvalue(), "", "the wrapper must print no verdict of its own")

    def test_passing_canonical_report_exits_zero(self):
        fake = FakeCanonicalMain(code=0)
        with canonical_main_is(fake), captured_output():
            self.assertEqual(main(["--test-emergency"]), 0)
        self.assertEqual(fake.calls, [["--emergency-break"]])

    def test_argparse_rejection_exit_code_is_preserved(self):
        # argparse raises SystemExit(2) for a bad command line. The wrapper must
        # surface that as a 2, never as a 0 and never as a traceback.
        fake = FakeCanonicalMain(raises=SystemExit(2))
        with canonical_main_is(fake), captured_output():
            self.assertEqual(main(["--test-thruster", "0"]), 2)

    def test_help_exit_code_is_preserved(self):
        fake = FakeCanonicalMain(raises=SystemExit(0))
        with canonical_main_is(fake), captured_output():
            self.assertEqual(main(["--help"]), 0)

    def test_a_bare_exit_is_reported_as_zero(self):
        fake = FakeCanonicalMain(raises=SystemExit(None))
        with canonical_main_is(fake), captured_output():
            self.assertEqual(main(["--auto"]), 0)

    def test_non_systemexit_from_the_canonical_tool_propagates(self):
        """
        A crash in the canonical tool must escape, not be laundered into a status.

        The prototype raised ``AttributeError`` out of its own check and the
        process still exited 0. ``except Exception: return 0`` around the delegation
        would reintroduce exactly that, so the only way to be sure it is not there
        is to make the fake raise something that is not a SystemExit and require it
        to arrive. ``guard`` already converts an exception inside a CHECK into a
        failed CheckResult; an exception here escaped the canonical main itself, and
        a wrapper is not entitled to decide it means success.
        """
        fake = FakeCanonicalMain(raises=RuntimeError("boom"))
        with canonical_main_is(fake), captured_output():
            with self.assertRaises(RuntimeError) as caught:
                main(["--test-thruster", "0", "1600"])
        self.assertIn("boom", str(caught.exception))
        self.assertEqual(len(fake.calls), 1)

    def test_none_returned_by_the_canonical_tool_is_never_a_pass(self):
        """
        A ``None`` return must not become exit 0.

        This is the prototype's exact shape: ``def main():`` with no ``return``
        statement anywhere, so it returned None on every path, and ``sys.exit(None)``
        exits 0. If ``can_stimulus.main`` ever regressed to that, every failed check
        would be reported as a pass. The wrapper is the last thing standing between
        that regression and a green build, so it must convert anything that is not
        an explicit integer into a nonzero status.

        The assertion is on the exact value, not ``assertNotEqual(0, result)``: that
        weaker form passes for ``None``, because ``None != 0``, and the defect it has
        to catch is precisely that None reaching ``sys.exit``.
        """
        fake = FakeCanonicalMain(code=None)
        with canonical_main_is(fake), captured_output():
            self.assertEqual(1, main(["--test-emergency"]))

    def test_a_non_integer_status_is_never_a_pass(self):
        # False is an int in Python, and False would map to exit 0 under a naive
        # isinstance check, so the guard excludes bool explicitly. 0.0 and "0" compare
        # equal to 0, so only the exact value catches them.
        for code in (False, "0", 0.0, [0], None):
            with self.subTest(code=code):
                fake = FakeCanonicalMain(code=code)
                with canonical_main_is(fake), captured_output():
                    self.assertEqual(1, main(["--test-emergency"]))

    def test_wrapper_hands_the_canonical_tool_the_translated_argv(self):
        fake = FakeCanonicalMain(code=0)
        with canonical_main_is(fake), captured_output():
            main(["--mode", "sil", "--test-arm", "--test-emergency", "--port", "9001"])
        self.assertEqual(
            fake.calls,
            [["--mode", "sil", "--node2-arm", "--emergency-break", "--port", "9001"]],
        )

    def test_wrapper_does_not_build_a_transport_or_a_tester_itself(self):
        # Everything that touches the bus belongs to the canonical tool. Every name
        # the wrapper could have used is patched to explode, INCLUDING the backend
        # classes themselves: patching only build_backend would let a wrapper that
        # constructed SilSocketBackend(...) directly sail straight through. Both are
        # patched on the module object the WRAPPER holds (see the note at the
        # import), and the canonical tool's own main is faked, so a hit here can only
        # come from the wrapper.
        held = node2_stimulus.can_stimulus
        explode = AssertionError("the wrapper built its own transport")
        fake = FakeCanonicalMain(code=0)
        with contextlib.ExitStack() as stack:
            for name in (
                "build_backend",
                "VehicleStimulusTester",
                "SilSocketBackend",
                "PythonCanBackend",
                "UartBackend",
            ):
                stack.enter_context(
                    mock.patch.object(held, name, side_effect=explode)
                )
            stack.enter_context(canonical_main_is(fake))
            stack.enter_context(captured_output())
            self.assertEqual(main(["--test-emergency"]), 0)
            self.assertEqual(main(["--channel", "can0"]), 2)
        self.assertEqual(fake.calls, [["--emergency-break"]])

    def test_the_two_can_stimulus_module_objects_are_distinct(self):
        # Recorded because it has already cost one test: patching this file's
        # `can_stimulus` does not affect the wrapper's, and the miss would have run
        # the real canonical main, which auto-starts an engine and binds 8765.
        self.assertIsNot(can_stimulus, node2_stimulus.can_stimulus)
        self.assertEqual(Path(node2_stimulus.can_stimulus.__file__), Path(can_stimulus.__file__))

    def test_main_uses_sys_argv_when_no_argv_is_given(self):
        fake = FakeCanonicalMain(code=0)
        with canonical_main_is(fake), captured_output(), \
                mock.patch.object(sys, "argv", ["node2_stimulus.py", "--test-depth", "3"]):
            self.assertEqual(main(), 0)
        self.assertEqual(fake.calls, [["--node2-depth", "3"]])


class TestOperatorFacingOutput(unittest.TestCase):
    """What the operator is told, and what they are not told."""

    def test_legacy_invocation_names_the_canonical_replacement_on_stderr(self):
        fake = FakeCanonicalMain(code=0)
        with canonical_main_is(fake), captured_output() as (out, err):
            main(["--test-emergency"])
        self.assertIn("--emergency-break", err.getvalue())
        self.assertIn("--test-emergency", err.getvalue())

    def test_canonical_invocation_prints_no_migration_notice(self):
        fake = FakeCanonicalMain(code=0)
        with canonical_main_is(fake), captured_output() as (_out, err):
            main(["--emergency-break"])
        self.assertEqual(err.getvalue(), "")

    def test_the_migration_notice_warns_that_thrusters_actuate(self):
        fake = FakeCanonicalMain(code=0)
        with canonical_main_is(fake), captured_output() as (_out, err):
            main(["--test-emergency"])
        notice = err.getvalue().lower()
        self.assertIn("thrust", notice)

    def test_the_migration_notice_quotes_no_pwm_value(self):
        """
        1800 us belongs to --test-emergency alone, and a wrong number is not safe.

        The notice is printed for any actuating flag, so a fixed "to 1800 us" told an
        operator running ``--test-thruster 0 1600`` a number the tool would never use.
        Over-warning is fine; a specific wrong value is not, so the notice carries no
        number at all and points at --help, which has the correct per-flag figures.
        """
        for argv in (["--test-thruster", "0", "1600"], ["--test-all", "1650"], ["--test-emergency"]):
            with self.subTest(argv=argv):
                fake = FakeCanonicalMain(code=0)
                with canonical_main_is(fake), captured_output() as (_out, err):
                    main(argv)
                notice = err.getvalue()
                self.assertTrue(notice.strip(), "an actuating legacy flag must warn")
                # A number with a unit, not a bare "us": that occurs inside "thrusters".
                self.assertIsNone(re.search(r"\d+\s*us", notice), notice)

    def test_receive_only_legacy_flag_is_not_warned_about_actuation(self):
        # --test-depth only listens. Claiming otherwise would be its own kind of
        # lie, and an operator who distrusts the warning stops reading it.
        fake = FakeCanonicalMain(code=0)
        with canonical_main_is(fake), captured_output() as (_out, err):
            main(["--test-depth", "2"])
        notice = err.getvalue()
        self.assertIn("--node2-depth", notice)
        self.assertNotIn("ACTUAT", notice)

    def test_wrapper_never_prints_a_pass_line(self):
        for code in (0, 1, 2):
            with self.subTest(code=code):
                fake = FakeCanonicalMain(code=code)
                with canonical_main_is(fake), captured_output() as (out, _err):
                    main(["--test-all", "1650"])
                self.assertNotIn("PASS", out.getvalue())
                self.assertNotIn("pass", out.getvalue().lower())

    def test_canonical_output_reaches_stdout_byte_for_byte(self):
        """
        The report on stdout is exactly what the canonical tool printed.

        ``test_wrapper_never_prints_a_pass_line`` can only catch the wrapper emitting
        a pass line of its own; it cannot catch the wrapper mixing its own text into
        the tool's output. Equality does both, and it is what backs the report's claim
        that stdout stays byte-for-byte what the tool that earned it printed.
        """
        report = (
            "[PASS] node2_pwm_ch0: channel 0 held 1600 us across 32 snapshot(s) in 1.0 s\n"
            "Summary: 1 check(s), 1 passed, 0 failed"
        )
        fake = FakeCanonicalMain(code=0, prints=report)
        with canonical_main_is(fake), captured_output() as (out, err):
            self.assertEqual(main(["--test-thruster", "0", "1600"]), 0)
        self.assertEqual(out.getvalue(), report + "\n")
        # The wrapper's own notice is on stderr and does not appear in the report.
        self.assertIn("[COMPAT]", err.getvalue())
        self.assertNotIn("[COMPAT]", out.getvalue())


# --------------------------------------------------------------------------
# Both entry paths, as a real process
# --------------------------------------------------------------------------
class TestSubprocessEntryPoints(unittest.TestCase):
    """
    The wrapper must behave the same as a script and as an import.

    Every case here is chosen to return before a transport is built, so nothing
    binds port 8765 and no engine is started.  Paths come from ``__file__`` and
    the interpreter is ``sys.executable``; a host that cannot launch one skips.
    """

    def test_help_exits_zero(self):
        completed = run_wrapper("--help")
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)

    def test_help_names_the_program_the_operator_ran(self):
        # argparse's default prog is basename(sys.argv[0]), so the usage line and every
        # error prefix would have read "can_stimulus.py" - telling the operator they
        # invoked a program, and a flag, that they did not.
        text = run_wrapper("--help").stdout
        self.assertIn("usage: node2_stimulus.py", text)
        self.assertNotIn("usage: can_stimulus.py", text)

    def test_an_argparse_error_names_this_program(self):
        # A non-numeric pulse, so the error is argparse's own and names this
        # program. It used to be --test-solenoid 0x0001, which stopped being an
        # error once the canonical parser learned the hex spelling.
        completed = run_wrapper("--test-thruster", "0", "not-a-number")
        self.assertEqual(completed.returncode, 2, completed.stdout + completed.stderr)
        self.assertIn("node2_stimulus.py: error:", completed.stderr)

    def test_help_lists_every_legacy_flag_and_its_canonical_name(self):
        text = run_wrapper("--help").stdout
        for legacy, canonical in sorted(LEGACY_FLAGS.items()):
            with self.subTest(legacy=legacy):
                self.assertIn(legacy, text)
                self.assertIn(canonical, text)

    def test_help_warns_that_the_legacy_flags_actuate_thrusters(self):
        text = run_wrapper("--help").stdout
        self.assertIn("--test-emergency", text)
        # The prototype's emergency check was a monitor; the canonical one drives
        # all eight thrusters. An operator must be told that here, not discover it.
        self.assertIn("1800", text)
        lowered = text.lower()
        self.assertIn("thrust", lowered)
        self.assertIn("actuat", lowered)

    def test_help_documents_the_fresh_engine_requirement_of_the_arming_check(self):
        text = run_wrapper("--help").stdout
        self.assertIn("--test-arm", text)
        self.assertIn("3000", text)

    def test_help_documents_the_refused_legacy_flags(self):
        text = run_wrapper("--help").stdout
        for flag in sorted(UNSUPPORTED_LEGACY_FLAGS):
            with self.subTest(flag=flag):
                self.assertIn(flag, text)

    def test_help_still_documents_the_canonical_flags(self):
        text = run_wrapper("--help").stdout
        for canonical in sorted(LEGACY_FLAGS.values()):
            with self.subTest(canonical=canonical):
                self.assertIn(canonical, text)

    def test_no_action_exits_two_without_starting_an_engine(self):
        completed = run_wrapper()
        self.assertEqual(completed.returncode, 2, completed.stdout + completed.stderr)
        self.assertIn("no action was selected", completed.stdout)
        # The canonical header only prints once a backend has been built.
        self.assertNotIn("X19 CAN stimulus", completed.stdout)

    def test_unknown_flag_exits_two(self):
        completed = run_wrapper("--test-nonexistent")
        self.assertEqual(completed.returncode, 2, completed.stdout + completed.stderr)
        self.assertNotIn("X19 CAN stimulus", completed.stdout)

    def test_legacy_flag_missing_its_value_exits_two(self):
        completed = run_wrapper("--test-thruster", "0")
        self.assertEqual(completed.returncode, 2, completed.stdout + completed.stderr)

    def test_unsupported_legacy_flag_exits_two_in_a_subprocess(self):
        completed = run_wrapper("--bitrate", "1000000")
        self.assertEqual(completed.returncode, 2, completed.stdout + completed.stderr)
        # stderr, like the migration notice: stdout stays the canonical report's alone.
        self.assertIn("--bitrate", completed.stderr)
        self.assertNotIn("[FAIL]", completed.stdout + completed.stderr)
        self.assertEqual(completed.stdout, "")

    def test_a_hex_solenoid_mask_reaches_the_canonical_tool_as_one(self):
        """
        The hex break is closed upstream, so the legacy form works again.

        The prototype parsed masks with ``int(x, 0)``, so ``0x0001`` worked there.
        The canonical parser briefly used ``type=int`` and made it a usage error,
        which Task 6's fix round refused to accept.  It now uses the shared
        prefix-aware parser, so this reaches the tool as ``--node2-solenoid 1``.

        ``no_autostart_env`` points ``X19_SIL_SERVER`` at a path that does not
        exist, so no engine is launched and the run stops at the transport.  That
        is enough: reaching the transport at all means argparse accepted the mask,
        which is the whole claim.
        """
        completed = run_wrapper("--test-solenoid", "0x0001")
        combined = completed.stdout + completed.stderr
        self.assertNotIn("usage:", combined, "the hex spelling must not be a usage error")
        self.assertNotIn("error:", combined, f"the hex spelling must not be a usage error:\n{combined}")
        self.assertIn(
            "X19 CAN stimulus",
            completed.stdout,
            "the tool must have parsed the command line and started running",
        )

    def test_runs_from_an_unrelated_working_directory(self):
        # The wrapper reaches the canonical tool by absolute path from __file__,
        # so it must not depend on being run from the repository root.
        with tempfile.TemporaryDirectory() as elsewhere:
            completed = run_wrapper("--help", cwd=Path(elsewhere))
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        self.assertIn("--test-emergency", completed.stdout)

    def test_importable_as_a_package_module(self):
        script = (
            "from tools import node2_stimulus as w;"
            "print(w.translate_legacy_args(['--test-emergency', '--port', '9']))"
        )
        try:
            completed = subprocess.run(
                [sys.executable, "-c", script],
                cwd=str(REPO_ROOT),
                capture_output=True,
                text=True,
                timeout=SUBPROCESS_TIMEOUT_S,
                env=no_autostart_env(),
            )
        except OSError as exc:  # pragma: no cover - interpreter cannot be launched
            raise unittest.SkipTest(f"could not launch {sys.executable} as a subprocess: {exc}")
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        self.assertEqual(completed.stdout.strip(), "['--emergency-break', '--port', '9']")

    def test_help_output_is_identical_from_both_entry_paths(self):
        as_script = run_wrapper("--help").stdout
        try:
            completed = subprocess.run(
                [sys.executable, "-c", "from tools import node2_stimulus as w; raise SystemExit(w.main(['--help']))"],
                cwd=str(REPO_ROOT),
                capture_output=True,
                text=True,
                timeout=SUBPROCESS_TIMEOUT_S,
                env=no_autostart_env(),
            )
        except OSError as exc:  # pragma: no cover - interpreter cannot be launched
            raise unittest.SkipTest(f"could not launch {sys.executable} as a subprocess: {exc}")
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        self.assertEqual(completed.stdout, as_script)


if __name__ == "__main__":
    unittest.main()
