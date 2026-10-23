"""Tier 2 Boundary & Corner Cases: Non-Existent File Handling.

Tests boundary conditions when invalid or missing files are supplied to build/run/flash.
Authoritative source: ORIGINAL_REQUEST.md § R1, PROJECT.md § Boundary Cases.
"""

from __future__ import annotations

import unittest
from pathlib import Path

from tests.e2e.e2e_context import (
    REPO_ROOT,
    run_rov_cli,
)
from tools.rov_cli.build import build_target


class TestNonexistentFile(unittest.TestCase):
    """Verify clean rejection and error diagnostics for missing/invalid file inputs."""

    def test_build_target_nonexistent_file_raises_system_exit(self):
        """Contract: build_target with missing custom_file exits cleanly with descriptive error."""
        missing = REPO_ROOT / "definitely_does_not_exist_98765.c"
        with self.assertRaises(SystemExit) as cm:
            build_target("sandbox", "f411", "debug", custom_file=str(missing))
        self.assertIn("does not exist", str(cm.exception))

    def test_cli_build_missing_relative_file_exits_nonzero(self):
        """Verify: rov build with missing relative file exits non-zero with error message."""
        res = run_rov_cli(["build", "-f", "nonexistent_source_123.c", "-b", "f411"])
        self.assertNotEqual(res.returncode, 0)
        self.assertTrue(
            "does not exist" in res.stderr.lower() or "does not exist" in res.stdout.lower() or "error" in (res.stderr + res.stdout).lower(),
        )

    def test_cli_build_directory_instead_of_file_fails(self):
        """Verify: Passing a directory path as scratch file is rejected cleanly."""
        dir_path = REPO_ROOT / "tools"
        with self.assertRaises(SystemExit) as cm:
            build_target("sandbox", "f411", "debug", custom_file=str(dir_path))
        self.assertIn("does not exist", str(cm.exception))

    def test_cli_build_missing_file_with_special_characters(self):
        """Verify: Non-existent file with spaces and symbols in path fails without unhandled crash."""
        special_path = REPO_ROOT / "non existent [file] #1.c"
        with self.assertRaises(SystemExit):
            build_target("sandbox", "f411", "debug", custom_file=str(special_path))

    def test_cli_target_resolver_nonexistent_file_handling(self):
        """Contract: If resolve_target exists, passing an invalid path ending in .c raises or resolves safely."""
        try:
            from tools.rov_cli.config import load_config, resolve_target
            cfg = load_config()
            # If target_arg ends in .c but does not exist, resolve_target or build must reject it
            node, board, resolved = resolve_target("missing_file.c", "f411", cfg)
            # When resolved, build_target must fail
            with self.assertRaises(SystemExit):
                build_target(node, board, "debug", custom_file=resolved)
        except (ImportError, AttributeError):
            self.skipTest("Pending Milestone M2: resolve_target implementation")


if __name__ == "__main__":
    unittest.main()
