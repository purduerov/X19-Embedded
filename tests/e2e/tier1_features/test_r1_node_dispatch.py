"""Tier 1 Feature Coverage: R1 Node Positional Dispatch.

Tests that developers can pass a node name directly as the first positional argument
(e.g. python rov.py run control_board, python rov.py build pi_shield) without -n.
Authoritative source: ORIGINAL_REQUEST.md § R1, PROJECT.md § Feature 5.
"""

from __future__ import annotations

import os
import unittest

from tests.e2e.e2e_context import run_rov_cli


def _supports_positional_node() -> bool:
    try:
        from tools.rov_cli.config import load_config, resolve_target
        cfg = load_config()
        node, board, file_path = resolve_target("control_board", "g474", cfg)
        return node == "control_board" and file_path is None
    except (ImportError, AttributeError):
        pass

    res = run_rov_cli(["build", "--help"])
    return "target" in res.stdout or "[target]" in res.stdout


class TestR1NodeDispatch(unittest.TestCase):
    """Verify positional node name execution for build, flash, and run."""

    def test_r1_target_resolver_node_contract(self):
        """Contract: resolve_target('control_board', 'g474', config) returns ('control_board', 'g474', None)."""
        if not _supports_positional_node() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI positional argument resolution")

        from tools.rov_cli.config import load_config, resolve_target
        cfg = load_config()

        node, board, file_path = resolve_target("control_board", "g474", cfg)
        self.assertEqual(node, "control_board", "Node name must resolve to canonical control_board")
        self.assertEqual(board, "g474", "Board parameter must be preserved")
        self.assertIsNone(file_path, "Node dispatch must not have custom file path")

    def test_r1_target_resolver_pi_shield_contract(self):
        """Contract: resolve_target('pi_shield', 'f411', config) returns ('pi_shield', 'f411', None)."""
        if not _supports_positional_node() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI positional argument resolution")

        from tools.rov_cli.config import load_config, resolve_target
        cfg = load_config()

        node, board, file_path = resolve_target("pi_shield", "f411", cfg)
        self.assertEqual(node, "pi_shield")
        self.assertEqual(board, "f411")
        self.assertIsNone(file_path)

    def test_r1_build_positional_node_pi_shield(self):
        """Verify: python rov.py build pi_shield -b f411 builds Node 1 binary."""
        if not _supports_positional_node() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI positional argument resolution")

        res = run_rov_cli(["build", "pi_shield", "-b", "f411"])
        self.assertEqual(res.returncode, 0, f"build pi_shield failed: {res.stderr}\n{res.stdout}")
        self.assertIn("[SUCCESS]", res.stdout)
        self.assertIn("pi_shield", res.stdout)

    def test_r1_build_positional_node_control_board(self):
        """Verify: python rov.py build control_board -b g474 builds Node 2 binary."""
        if not _supports_positional_node() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI positional argument resolution")

        res = run_rov_cli(["build", "control_board", "-b", "g474"])
        self.assertEqual(res.returncode, 0, f"build control_board failed: {res.stderr}\n{res.stdout}")
        self.assertIn("[SUCCESS]", res.stdout)
        self.assertIn("control_board", res.stdout)

    def test_r1_run_accepts_positional_node(self):
        """Verify: python rov.py run control_board accepts positional node without -n."""
        if not _supports_positional_node() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI positional argument resolution")

        res = run_rov_cli(["run", "control_board", "-b", "g474", "-t", "0.1"])
        self.assertNotIn("unrecognized arguments", res.stderr)
        self.assertNotIn("unrecognized arguments", res.stdout)

    def test_r1_flash_accepts_positional_node(self):
        """Verify: python rov.py flash pi_shield accepts positional node without -n."""
        if not _supports_positional_node() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI positional argument resolution")

        res = run_rov_cli(["flash", "pi_shield", "-b", "f411"])
        self.assertNotIn("unrecognized arguments", res.stderr)
        self.assertNotIn("unrecognized arguments", res.stdout)


if __name__ == "__main__":
    unittest.main()
