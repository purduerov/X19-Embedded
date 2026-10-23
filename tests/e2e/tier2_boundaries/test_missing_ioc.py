"""Tier 2 Boundary & Corner Cases: Missing & Corrupt .ioc Handling.

Tests boundary conditions when .ioc files are absent, corrupted, or contain incomplete
MicroXplorer peripheral configuration metadata.
Authoritative source: ORIGINAL_REQUEST.md § R2, PROJECT.md § Boundary Cases.
"""

from __future__ import annotations

import shutil
import unittest
from pathlib import Path

from tests.e2e.e2e_context import (
    REPO_ROOT,
    temp_board,
)
from tools.rov_cli.build import discover_ioc_targets, generate_cubemx
from tools.rov_cli.config import _parse_ioc_mcu, load_config


class TestMissingIoc(unittest.TestCase):
    """Verify robust discovery and fallback behavior for absent or corrupted .ioc files."""

    def test_board_directory_without_ioc_handled_safely(self):
        """Verify: An empty directory in boards/ does not cause load_config() to crash."""
        empty_board_dir = REPO_ROOT / "boards" / "tmp_empty_board"
        empty_board_dir.mkdir(parents=True, exist_ok=True)
        try:
            cfg = load_config()
            self.assertIn("boards", cfg)
        finally:
            shutil.rmtree(empty_board_dir, ignore_errors=True)

    def test_corrupt_or_empty_ioc_falls_back_to_defaults(self):
        """Contract: A 0-byte or corrupt .ioc file safely falls back to standard default MCU/arch."""
        with temp_board("tmp_corrupt_ioc") as b_dir:
            ioc_file = b_dir / "tmp_corrupt_ioc.ioc"
            ioc_file.write_text("", encoding="utf-8")  # Empty file

            mcu, arch = _parse_ioc_mcu(ioc_file)
            self.assertEqual(mcu, "STM32G474RET6", "Empty .ioc must fall back to default G474 MCU")
            self.assertEqual(arch, "arm-cortex-m4", "Empty .ioc must fall back to arm-cortex-m4 arch")

    def test_ioc_bak_files_are_ignored(self):
        """Verify: .ioc.bak backup files created by STM32CubeMX are ignored by discover_ioc_targets."""
        with temp_board("tmp_bak_board") as b_dir:
            bak_file = b_dir / "backup_peripheral.ioc.bak"
            bak_file.write_text("Mcu.CPN=STM32F411RET6\n", encoding="utf-8")

            lookup, all_paths = discover_ioc_targets()
            for path in all_paths:
                self.assertFalse(path.name.endswith(".bak"), f"Found backup file in targets: {path}")

    def test_ioc_missing_family_field_falls_back_safely(self):
        """Verify: .ioc with Mcu.CPN but missing Mcu.Family defaults arch safely."""
        with temp_board("tmp_no_family") as b_dir:
            ioc_file = b_dir / "tmp_no_family.ioc"
            ioc_file.write_text("Mcu.CPN=STM32F401RET6\n", encoding="utf-8")

            mcu, arch = _parse_ioc_mcu(ioc_file)
            self.assertEqual(mcu, "STM32F401RET6")
            self.assertEqual(arch, "arm-cortex-m4")

    def test_generate_cubemx_missing_ioc_in_directory_exits_cleanly(self):
        """Verify: generate_cubemx on directory lacking .ioc exits with descriptive error."""
        empty_dir = REPO_ROOT / "boards" / "tmp_no_ioc_gen"
        empty_dir.mkdir(parents=True, exist_ok=True)
        try:
            with self.assertRaises(SystemExit) as cm:
                generate_cubemx(target=str(empty_dir))
            self.assertIn("No .ioc file found", str(cm.exception))
        finally:
            shutil.rmtree(empty_dir, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
