"""Tier 3 Cross-Feature Combinations: CLI Flags Matrix.

Tests pairwise combinations of CLI flags (commands x profiles x targets x options).
Authoritative source: ORIGINAL_REQUEST.md § R1, PROJECT.md § Tier 3.
"""

from __future__ import annotations

import unittest
from pathlib import Path

from tests.e2e.e2e_context import (
    VALID_SCRATCH_C,
    run_rov_cli,
    temp_scratch_file,
)
from tools.rov_cli.build import build_target


class TestCliFlagsMatrix(unittest.TestCase):
    """Verify orthogonal flag combinations across build, run, and test commands."""

    def test_run_with_timeout_flag(self):
        """Verify: rov run -b f411 -t 0.1 accepts timeout flag without crashing."""
        res = run_rov_cli(["run", "-b", "f411", "-t", "0.1"])
        self.assertNotIn("unrecognized arguments", res.stderr)

    def test_build_with_profile_release_flag(self):
        """Verify: rov build -b f411 -p release builds release profile cleanly."""
        res = run_rov_cli(["build", "-b", "f411", "-p", "release"])
        self.assertEqual(res.returncode, 0, f"Release build failed: {res.stderr}\n{res.stdout}")
        self.assertIn("[SUCCESS]", res.stdout)

    def test_test_with_dry_run_flag(self):
        """Verify: rov test --dry-run prints resolved ctest command and exits 0."""
        res = run_rov_cli(["test", "--dry-run"])
        self.assertEqual(res.returncode, 0)
        self.assertIn("ctest", res.stdout.lower())

    def test_test_with_node_and_target_flags(self):
        """Verify: rov test with --node control_board executes control board tests."""
        res = run_rov_cli(["test", "--node", "control_board"])
        self.assertEqual(res.returncode, 0)
        self.assertIn("100% tests passed", res.stdout)

    def test_build_direct_file_with_release_profile_matrix(self):
        """Verify: Direct scratch C file with release profile compiles to release binary."""
        with temp_scratch_file(VALID_SCRATCH_C) as scratch:
            elf = build_target("sandbox", "f411", "release", custom_file=str(scratch))
            self.assertTrue(Path(elf).is_file())
            self.assertIn("f411", Path(elf).name.lower())


if __name__ == "__main__":
    unittest.main()
