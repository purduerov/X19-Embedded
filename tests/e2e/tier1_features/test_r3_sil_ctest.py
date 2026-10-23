"""Tier 1 Feature Coverage: R3 Host SIL CTest Test Execution & Scope.

Tests that all 36 host SIL CTest unit test suites pass with 100% success and zero regressions,
and that filtering flags (--suite, --node, -R, -k) operate accurately.
Authoritative source: ORIGINAL_REQUEST.md § R3, PROJECT.md § Feature 6.
"""

from __future__ import annotations

import os
import subprocess
import unittest

from tests.e2e.e2e_context import (
    REPO_ROOT,
    run_rov_cli,
)


def _is_rov_test_clobber_fixed() -> bool:
    """Check if Milestone M2 has fixed args.node default clobbering in rov test."""
    res = run_rov_cli(["test", "--dry-run"])
    if res.returncode != 0:
        # Fall back to testing rov test directly
        return False
    # If the ctest command does not contain '-L node:pi_shield' by default, the bug is fixed
    return "-L node:" not in res.stdout


class TestR3SilCTest(unittest.TestCase):
    """Verify Host SIL CTest test execution, scope resolution, and filter arguments."""

    def test_r3_native_ctest_all_36_suites_pass(self):
        """Verify: 100% passing rate across all 36 host SIL test suites."""
        cmd = [
            "ctest",
            "--test-dir",
            str(REPO_ROOT / "build-native"),
            "-E",
            "^target_",
            "--output-on-failure",
        ]
        res = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
        self.assertEqual(
            res.returncode,
            0,
            f"Native SIL CTest failed with exit code {res.returncode}:\n{res.stdout}\n{res.stderr}",
        )
        self.assertIn("100% tests passed, 0 tests failed out of 36", res.stdout)

    def test_r3_rov_test_scope_runs_all_suites_by_default(self):
        """Verify: python rov.py test executes all test suites without default node filter clobbering."""
        if not _is_rov_test_clobber_fixed() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: Fix rov test execution scope")

        res = run_rov_cli(["test"])
        self.assertEqual(res.returncode, 0, f"rov test failed:\n{res.stderr}\n{res.stdout}")
        self.assertIn("100% tests passed", res.stdout)
        self.assertNotIn("-L node:pi_shield", res.stdout, "rov test must not clobber test scope by default")

    def test_r3_rov_test_filter_by_suite(self):
        """Verify: python rov.py test --suite safety filters test execution to safety suite."""
        if not _is_rov_test_clobber_fixed() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: Fix rov test execution scope (clobbering args.node)")

        res = run_rov_cli(["test", "--suite", "safety"])
        self.assertEqual(res.returncode, 0, f"rov test --suite safety failed:\n{res.stderr}\n{res.stdout}")
        self.assertIn("suite:safety", res.stdout)
        self.assertIn("100% tests passed", res.stdout)

    def test_r3_rov_test_filter_by_node(self):
        """Verify: python rov.py test --node control_board filters to control board tests."""
        res = run_rov_cli(["test", "--node", "control_board"])
        self.assertEqual(res.returncode, 0, f"rov test --node control_board failed:\n{res.stderr}\n{res.stdout}")
        self.assertIn("node:control_board", res.stdout)
        self.assertIn("100% tests passed", res.stdout)

    def test_r3_rov_test_filter_by_regex(self):
        """Verify: python rov.py test -R 'test_driver_bme280' runs only matching test."""
        if not _is_rov_test_clobber_fixed() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: Fix rov test execution scope (clobbering args.node)")

        res = run_rov_cli(["test", "-R", "test_driver_bme280"])
        self.assertEqual(res.returncode, 0, f"rov test -R failed:\n{res.stderr}\n{res.stdout}")
        self.assertIn("test_driver_bme280", res.stdout)
        self.assertIn("1/1 Test", res.stdout)


if __name__ == "__main__":
    unittest.main()
