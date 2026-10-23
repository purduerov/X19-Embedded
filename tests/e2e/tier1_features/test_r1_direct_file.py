"""Tier 1 Feature Coverage: R1 Direct File Execution.

Tests that developers can pass any .c source file directly as the first positional
argument to build, flash, and run without requiring -f or -n sandbox.
Authoritative source: ORIGINAL_REQUEST.md § R1, PROJECT.md § Feature 5.
"""

from __future__ import annotations

import os
import unittest
from pathlib import Path

from tests.e2e.e2e_context import (
    REPO_ROOT,
    VALID_SCRATCH_C,
    run_rov_cli,
    temp_scratch_file,
)


def _supports_positional_file() -> bool:
    """Check if the CLI parser has implemented positional target argument (M2)."""
    try:
        from tools.rov_cli.config import load_config, resolve_target
        cfg = load_config()
        node, board, file_path = resolve_target("dummy.c", "f411", cfg)
        return node == "sandbox" and file_path is not None
    except (ImportError, AttributeError):
        pass

    # Probe CLI --help for positional target argument
    res = run_rov_cli(["build", "--help"])
    return "target" in res.stdout or "[target]" in res.stdout


class TestR1DirectFileExecution(unittest.TestCase):
    """Verify positional .c file execution for build, flash, and run."""

    def test_r1_target_resolver_contract(self):
        """Contract: resolve_target('<file>.c', board, config) returns ('sandbox', board, resolved_path)."""
        if not _supports_positional_file() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI positional argument resolution")

        from tools.rov_cli.config import load_config, resolve_target
        cfg = load_config()

        with temp_scratch_file(VALID_SCRATCH_C) as scratch:
            node, board, resolved_path = resolve_target(str(scratch), "f411", cfg)
            self.assertEqual(node, "sandbox", "Positional .c file must resolve to sandbox node")
            self.assertEqual(board, "f411", "Board parameter must be preserved")
            self.assertIsNotNone(resolved_path, "Resolved file path must not be None")
            self.assertTrue(Path(resolved_path).is_file(), "Resolved path must point to an existing file")
            self.assertEqual(Path(resolved_path).resolve(), scratch.resolve())

    def test_r1_build_direct_c_file_relative_path(self):
        """Verify: python rov.py build <relative_file.c> builds target binary."""
        if not _supports_positional_file() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI positional argument resolution")

        with temp_scratch_file(VALID_SCRATCH_C, directory=REPO_ROOT) as scratch:
            rel_path = scratch.relative_to(REPO_ROOT).as_posix()
            res = run_rov_cli(["build", rel_path, "-b", "f411"])
            self.assertEqual(res.returncode, 0, f"build {rel_path} failed: {res.stderr}\n{res.stdout}")
            self.assertIn("[SUCCESS]", res.stdout, "Build output must indicate success")

    def test_r1_build_direct_c_file_absolute_path(self):
        """Verify: python rov.py build <absolute_path.c> builds target binary."""
        if not _supports_positional_file() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI positional argument resolution")

        with temp_scratch_file(VALID_SCRATCH_C) as scratch:
            abs_path = str(scratch.resolve())
            res = run_rov_cli(["build", abs_path, "-b", "f411"])
            self.assertEqual(res.returncode, 0, f"build {abs_path} failed: {res.stderr}\n{res.stdout}")
            self.assertIn("[SUCCESS]", res.stdout, "Build output must indicate success")

    def test_r1_build_direct_c_file_with_g474_board(self):
        """Verify: python rov.py build <file.c> -b g474 builds for G474 board."""
        if not _supports_positional_file() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI positional argument resolution")

        with temp_scratch_file(VALID_SCRATCH_C) as scratch:
            res = run_rov_cli(["build", str(scratch), "-b", "g474"])
            self.assertEqual(res.returncode, 0, f"build for g474 failed: {res.stderr}\n{res.stdout}")
            self.assertIn("[SUCCESS]", res.stdout)

    def test_r1_flash_accepts_direct_c_file_without_flag(self):
        """Verify: python rov.py flash <file.c> accepts positional file without -f."""
        if not _supports_positional_file() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI positional argument resolution")

        with temp_scratch_file(VALID_SCRATCH_C) as scratch:
            # Running flash without connected board should fail at programmer step, NOT at arg parsing
            res = run_rov_cli(["flash", str(scratch), "-b", "f411"])
            self.assertNotIn("unrecognized arguments", res.stderr, "Positional file must not be rejected by parser")
            self.assertNotIn("unrecognized arguments", res.stdout)

    def test_r1_run_accepts_direct_c_file_without_flag(self):
        """Verify: python rov.py run <file.c> accepts positional file without -f."""
        if not _supports_positional_file() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI positional argument resolution")

        with temp_scratch_file(VALID_SCRATCH_C) as scratch:
            res = run_rov_cli(["run", str(scratch), "-b", "f411", "-t", "0.1"])
            self.assertNotIn("unrecognized arguments", res.stderr, "Positional file must not be rejected by parser")
            self.assertNotIn("unrecognized arguments", res.stdout)


if __name__ == "__main__":
    unittest.main()
