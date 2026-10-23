"""Tier 2 Boundary & Corner Cases: Non-Existent Board Handling.

Tests boundary conditions when invalid, empty, or unconfigured board targets are supplied.
Authoritative source: ORIGINAL_REQUEST.md § R1 & R2, PROJECT.md § Boundary Cases.
"""

from __future__ import annotations

import unittest

from tests.e2e.e2e_context import run_rov_cli
from tools.rov_cli.build import build_target


class TestNonexistentBoard(unittest.TestCase):
    """Verify clean rejection and helpful diagnostics for missing/invalid board names."""

    def test_build_target_unknown_board_raises_system_exit(self):
        """Contract: build_target with unknown board exits with error listing valid boards."""
        with self.assertRaises(SystemExit) as cm:
            build_target("pi_shield", "nonexistent_board_xyz")
        error_msg = str(cm.exception)
        self.assertIn("Unknown board", error_msg)
        self.assertIn("Valid boards", error_msg)

    def test_cli_build_unknown_board_exits_nonzero(self):
        """Verify: rov build -b nonexistent_board_xyz exits non-zero with error diagnostics."""
        res = run_rov_cli(["build", "-b", "nonexistent_board_xyz"])
        self.assertNotEqual(res.returncode, 0)
        combined = (res.stderr + res.stdout).lower()
        self.assertTrue(
            "unknown board" in combined or "error" in combined,
            f"Expected unknown board error, got:\n{res.stderr}\n{res.stdout}",
        )

    def test_cli_build_empty_board_string_rejected(self):
        """Verify: rov build -b '' is rejected cleanly."""
        res = run_rov_cli(["build", "-b", ""])
        self.assertNotEqual(res.returncode, 0)

    def test_cli_build_board_special_characters(self):
        """Verify: rov build with special characters in board name fails gracefully."""
        res = run_rov_cli(["build", "-b", "board/../../etc"])
        self.assertNotEqual(res.returncode, 0)

    def test_cli_error_message_suggests_available_boards(self):
        """Verify: Error output for unknown board lists valid available boards."""
        res = run_rov_cli(["build", "-b", "mystery_board"])
        self.assertNotEqual(res.returncode, 0)
        combined = res.stderr + res.stdout
        self.assertIn("Valid boards", combined)
        self.assertTrue("f411" in combined or "g474" in combined)


if __name__ == "__main__":
    unittest.main()
