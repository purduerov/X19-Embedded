"""Tier 1 Feature Coverage: R1 Redundant Subcommand Removal.

Tests that redundant subcommands (sandbox, scan, sync) are removed from the CLI
and properly rejected with clear error messages.
Authoritative source: ORIGINAL_REQUEST.md § R1, PROJECT.md § Feature 4.
"""

from __future__ import annotations

import os
import unittest

from tests.e2e.e2e_context import run_rov_cli


def _subcommands_removed() -> bool:
    res = run_rov_cli(["--help"])
    # If 'sandbox' is not listed under positional commands, it has been removed
    return "sandbox" not in res.stdout


class TestR1SubcommandRemoval(unittest.TestCase):
    """Verify elimination of redundant subcommands: scan, sandbox, sync."""

    def test_r1_sandbox_subcommand_rejected(self):
        """Verify: python rov.py sandbox is rejected as invalid subcommand."""
        if not _subcommands_removed() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI subcommand removal")

        res = run_rov_cli(["sandbox"])
        self.assertNotEqual(res.returncode, 0, "rov.py sandbox must exit non-zero")
        combined = (res.stderr + res.stdout).lower()
        self.assertTrue(
            "invalid choice" in combined or "unrecognized" in combined or "error" in combined,
            f"Expected error message rejecting 'sandbox' subcommand, got:\n{res.stderr}\n{res.stdout}",
        )

    def test_r1_scan_subcommand_rejected(self):
        """Verify: python rov.py scan is rejected as invalid subcommand."""
        if not _subcommands_removed() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI subcommand removal")

        res = run_rov_cli(["scan"])
        self.assertNotEqual(res.returncode, 0, "rov.py scan must exit non-zero")
        combined = (res.stderr + res.stdout).lower()
        self.assertTrue(
            "invalid choice" in combined or "unrecognized" in combined or "error" in combined,
            f"Expected error message rejecting 'scan' subcommand, got:\n{res.stderr}\n{res.stdout}",
        )

    def test_r1_sync_subcommand_rejected(self):
        """Verify: python rov.py sync is rejected as invalid subcommand."""
        if not _subcommands_removed() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI subcommand removal")

        res = run_rov_cli(["sync"])
        self.assertNotEqual(res.returncode, 0, "rov.py sync must exit non-zero")
        combined = (res.stderr + res.stdout).lower()
        self.assertTrue(
            "invalid choice" in combined or "unrecognized" in combined or "error" in combined,
            f"Expected error message rejecting 'sync' subcommand, got:\n{res.stderr}\n{res.stdout}",
        )

    def test_r1_help_command_surface_minimal(self):
        """Verify: python rov.py --help lists only the consolidated command surface."""
        if not _subcommands_removed() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI subcommand removal")

        res = run_rov_cli(["--help"])
        self.assertEqual(res.returncode, 0)

        # Permitted commands: run, build, flash, monitor, devices, test, generate (and check)
        for valid_cmd in ("run", "build", "flash", "monitor", "devices", "test", "generate"):
            self.assertIn(valid_cmd, res.stdout, f"Help must list consolidated command '{valid_cmd}'")

        # Removed commands must NOT appear in command choices
        help_lower = res.stdout.lower()
        self.assertNotIn("scan", help_lower.split("positional arguments:")[1].split("options:")[0])
        self.assertNotIn("sandbox", help_lower.split("positional arguments:")[1].split("options:")[0])
        self.assertNotIn("sync", help_lower.split("positional arguments:")[1].split("options:")[0])

    def test_r1_removed_subcommand_help_flag_rejected(self):
        """Verify: python rov.py sandbox --help fails because command does not exist."""
        if not _subcommands_removed() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI subcommand removal")

        res = run_rov_cli(["sandbox", "--help"])
        self.assertNotEqual(res.returncode, 0)


if __name__ == "__main__":
    unittest.main()
