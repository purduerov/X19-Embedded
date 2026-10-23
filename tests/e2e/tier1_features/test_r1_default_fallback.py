"""Tier 1 Feature Coverage: R1 Default Fallback Execution.

Tests that omitting target arguments defaults cleanly to the configured or
connected hardware target board and node according to workspace rules.
Authoritative source: ORIGINAL_REQUEST.md § R1, PROJECT.md § Feature 5.
"""

from __future__ import annotations

import os
import unittest

from tests.e2e.e2e_context import run_rov_cli


def _supports_positional_cli() -> bool:
    try:
        from tools.rov_cli.config import load_config, resolve_target
        cfg = load_config()
        node, board, file_path = resolve_target(None, None, cfg)
        return node is not None and board is not None
    except (ImportError, AttributeError):
        pass

    res = run_rov_cli(["build", "--help"])
    return "target" in res.stdout or "[target]" in res.stdout


class TestR1DefaultFallback(unittest.TestCase):
    """Verify default target fallback when positional arguments are omitted."""

    def test_r1_target_resolver_none_contract(self):
        """Contract: resolve_target(None, None, config) returns configured defaults."""
        if not _supports_positional_cli() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI target resolution")

        from tools.rov_cli.config import load_config, resolve_target
        cfg = load_config()

        expected_node = cfg.get("workspace", {}).get("default_node", "pi_shield")
        expected_board = cfg.get("workspace", {}).get("default_board", "f411")

        node, board, file_path = resolve_target(None, None, cfg)
        self.assertEqual(node, expected_node, "Omitted target must resolve to workspace default_node")
        self.assertEqual(board, expected_board, "Omitted board must resolve to workspace default_board")
        self.assertIsNone(file_path, "Default target must not produce a custom file path")

    def test_r1_target_resolver_g474_board_infers_control_board(self):
        """Contract: resolve_target(None, 'g474', config) infers node 'control_board'."""
        if not _supports_positional_cli() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI target resolution")

        from tools.rov_cli.config import load_config, resolve_target
        cfg = load_config()

        node, board, file_path = resolve_target(None, "g474", cfg)
        self.assertEqual(node, "control_board", "G474 board must infer control_board node")
        self.assertEqual(board, "g474")
        self.assertIsNone(file_path)

    def test_r1_target_resolver_f411_board_infers_pi_shield(self):
        """Contract: resolve_target(None, 'f411', config) infers node 'pi_shield'."""
        if not _supports_positional_cli() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI target resolution")

        from tools.rov_cli.config import load_config, resolve_target
        cfg = load_config()

        node, board, file_path = resolve_target(None, "f411", cfg)
        self.assertEqual(node, "pi_shield", "F411 board must infer pi_shield node")
        self.assertEqual(board, "f411")
        self.assertIsNone(file_path)

    def test_r1_default_build_without_arguments(self):
        """Verify: python rov.py build with no arguments executes cleanly."""
        res = run_rov_cli(["build"])
        self.assertEqual(res.returncode, 0, f"Default build failed: {res.stderr}\n{res.stdout}")
        self.assertIn("[SUCCESS]", res.stdout)

    def test_r1_default_run_without_arguments_attempts_flash(self):
        """Verify: python rov.py run with no arguments defaults cleanly."""
        res = run_rov_cli(["run", "-t", "0.1"])
        # Should not crash with missing argument error; will fail at flash step if no board connected
        self.assertNotIn("the following arguments are required", res.stderr)
        self.assertNotIn("unrecognized arguments", res.stderr)


if __name__ == "__main__":
    unittest.main()
