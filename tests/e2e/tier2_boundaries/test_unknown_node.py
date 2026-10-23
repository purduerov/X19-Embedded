"""Tier 2 Boundary & Corner Cases: Unknown Node Handling.

Tests boundary conditions when invalid node names, typos, or unsupported node/board
combinations are supplied to the build system.
Authoritative source: ORIGINAL_REQUEST.md § R1 & R3, PROJECT.md § Boundary Cases.
"""

from __future__ import annotations

import unittest

from tests.e2e.e2e_context import run_rov_cli
from tools.rov_cli.build import build_target
from tools.rov_cli.config import load_config


class TestUnknownNode(unittest.TestCase):
    """Verify clean rejection and helpful diagnostics for missing/invalid node targets."""

    def test_build_target_unknown_node_raises_system_exit(self):
        """Contract: build_target with unknown node exits with error listing valid nodes."""
        with self.assertRaises(SystemExit) as cm:
            build_target("unknown_node_abc", "f411")
        error_msg = str(cm.exception)
        self.assertIn("Unknown node", error_msg)
        self.assertIn("Valid nodes", error_msg)

    def test_cli_build_unknown_node_flag_exits_nonzero(self):
        """Verify: rov build -n unknown_node_abc exits non-zero with error diagnostics."""
        res = run_rov_cli(["build", "-n", "unknown_node_abc", "-b", "f411"])
        self.assertNotEqual(res.returncode, 0)
        combined = (res.stderr + res.stdout).lower()
        self.assertTrue(
            "unknown node" in combined or "error" in combined,
            f"Expected unknown node error, got:\n{res.stderr}\n{res.stdout}",
        )

    def test_node_not_configured_for_board_rejected(self):
        """Verify: Building a node on a board not in its supported_boards list is cleanly rejected."""
        cfg = load_config()
        # Find a node and an unconfigured board
        nodes = cfg.get("nodes", {})
        boards = cfg.get("boards", {})
        if not nodes or not boards:
            self.skipTest("No nodes/boards configured")

        # Pick first node and find an unsupported board if any
        for node_name, node_cfg in nodes.items():
            supported = node_cfg.get("supported_boards", [])
            for b_name in boards.keys():
                if b_name not in supported:
                    with self.assertRaises(SystemExit) as cm:
                        build_target(node_name, b_name)
                    self.assertIn("not configured", str(cm.exception).lower())
                    return

    def test_cli_build_node_typo_diagnostics(self):
        """Verify: Common typos like 'pishield' instead of 'pi_shield' produce clear error."""
        res = run_rov_cli(["build", "-n", "pishield", "-b", "f411"])
        self.assertNotEqual(res.returncode, 0)
        combined = res.stderr + res.stdout
        self.assertIn("Valid nodes", combined)

    def test_cli_error_message_suggests_valid_nodes(self):
        """Verify: Error message lists standard vehicle nodes (pi_shield, control_board, power_slab)."""
        res = run_rov_cli(["build", "-n", "bad_node_name", "-b", "f411"])
        self.assertNotEqual(res.returncode, 0)
        combined = res.stderr + res.stdout
        self.assertTrue("pi_shield" in combined or "control_board" in combined)


if __name__ == "__main__":
    unittest.main()
