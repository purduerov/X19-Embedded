"""Tier 4 Real-World Scenario: Custom Board Integration Full Workflow.

Models a developer adding a new custom STM32 hardware board to the monorepo:
1. Placing a new board directory with .ioc in boards/
2. Verifying the CLI dynamically registers the board without editing rov.toml
3. Compiling a scratch C test program targeting the newly added board
4. Verifying valid ELF binary output
Authoritative source: ORIGINAL_REQUEST.md § R2, PROJECT.md § Tier 4.
"""

from __future__ import annotations

import unittest
from pathlib import Path

from tests.e2e.e2e_context import (
    VALID_SCRATCH_C,
    temp_board,
    temp_scratch_file,
)
from tools.rov_cli.build import build_target, discover_ioc_targets
from tools.rov_cli.config import load_config


class TestCustomBoardFullWorkflow(unittest.TestCase):
    """Verify end-to-end lifecycle when adding and targeting a new custom board."""

    def test_custom_board_addition_and_compilation_workflow(self):
        """Workflow: Add boards/test_subsea_hat -> auto-discover -> compile scratch file."""
        board_name = "test_subsea_hat"
        with temp_board(board_name, mcu="STM32F411RET6", family="STM32F4") as b_dir:
            # 1. Verify directory creation
            self.assertTrue(b_dir.is_dir())

            # 2. Verify dynamic discovery by config loader
            cfg = load_config()
            self.assertIn(board_name, cfg.get("boards", {}))
            board_meta = cfg["boards"][board_name]
            self.assertEqual(board_meta["mcu"], "STM32F411RET6")
            self.assertEqual(board_meta["architecture"], "arm-cortex-m4")

            # 3. Verify discovery by CubeMX target lookup
            lookup, all_ioc_paths = discover_ioc_targets()
            self.assertIn(board_name, lookup)

            # 4. Compile a scratch C program targeting the newly added board
            with temp_scratch_file(VALID_SCRATCH_C) as scratch:
                elf = build_target("sandbox", board_name, "debug", custom_file=str(scratch))
                self.assertTrue(Path(elf).is_file())
                self.assertIn(board_name, Path(elf).name)
                self.assertEqual(Path(elf).read_bytes()[:4], b"\x7fELF")


if __name__ == "__main__":
    unittest.main()
