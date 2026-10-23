"""Tier 3 Cross-Feature Combinations: Direct File Execution with Custom Board Targets.

Tests pairwise interaction between direct scratch C file execution and various hardware
board targets (F411, G474, STM32C5, auto-discovered boards, profiles).
Authoritative source: ORIGINAL_REQUEST.md § R1 & R2, PROJECT.md § Tier 3.
"""

from __future__ import annotations

import os
import unittest
from pathlib import Path

from tests.e2e.e2e_context import (
    REPO_ROOT,
    VALID_SCRATCH_C,
    run_rov_cli,
    temp_board,
    temp_scratch_file,
)
from tools.rov_cli.build import build_target


def _supports_positional() -> bool:
    try:
        from tools.rov_cli.config import load_config, resolve_target
        cfg = load_config()
        node, board, file_path = resolve_target("dummy.c", "f411", cfg)
        return node == "sandbox" and file_path is not None
    except (ImportError, AttributeError):
        return False


def _is_c5_ready() -> bool:
    c5_dir = REPO_ROOT / "boards" / "stm32c5"
    if not (c5_dir / "CMakeLists.txt").is_file():
        return False
    res = run_rov_cli(["build", "-b", "stm32c5"])
    return res.returncode == 0


class TestDirectFileCustomBoard(unittest.TestCase):
    """Verify pairwise interactions between custom .c files and board targets."""

    def test_direct_file_with_f411_board(self):
        """Verify: Building a custom scratch C file targeting F411 produces sandbox ELF."""
        with temp_scratch_file(VALID_SCRATCH_C) as scratch:
            elf = build_target("sandbox", "f411", "debug", custom_file=str(scratch))
            self.assertTrue(Path(elf).is_file(), f"Binary must exist: {elf}")
            self.assertIn("f411", Path(elf).name.lower())

    def test_direct_file_with_g474_board(self):
        """Verify: Building a custom scratch C file targeting G474 produces sandbox ELF."""
        with temp_scratch_file(VALID_SCRATCH_C) as scratch:
            elf = build_target("sandbox", "g474", "debug", custom_file=str(scratch))
            self.assertTrue(Path(elf).is_file(), f"Binary must exist: {elf}")
            self.assertIn("g474", Path(elf).name.lower())

    def test_direct_file_with_stm32c5_board(self):
        """Verify: Building a custom scratch C file targeting STM32C5 produces ELF."""
        if not _is_c5_ready() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M1: STM32C5 Target Decoupling & Scaffolding")

        with temp_scratch_file(VALID_SCRATCH_C) as scratch:
            elf = build_target("sandbox", "stm32c5", "debug", custom_file=str(scratch))
            self.assertTrue(Path(elf).is_file(), f"Binary must exist: {elf}")

    def test_direct_file_with_release_profile_on_g474(self):
        """Verify: Building a scratch C file on G474 with release profile compiles."""
        with temp_scratch_file(VALID_SCRATCH_C) as scratch:
            elf = build_target("sandbox", "g474", "release", custom_file=str(scratch))
            self.assertTrue(Path(elf).is_file())

    def test_positional_cli_direct_file_with_board_override(self):
        """Verify: python rov.py build <file.c> -b g474 compiles via positional CLI."""
        if not _supports_positional() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M2: CLI positional argument resolution")

        with temp_scratch_file(VALID_SCRATCH_C) as scratch:
            res = run_rov_cli(["build", str(scratch), "-b", "g474"])
            self.assertEqual(res.returncode, 0)
            self.assertIn("[SUCCESS]", res.stdout)


if __name__ == "__main__":
    unittest.main()
