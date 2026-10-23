"""Tier 1 Feature Coverage: R2 Frictionless Custom Board & .ioc Discovery.

Tests that any board or dedicated test configuration with an .ioc file placed inside
boards/<board_name>/ is automatically discovered by the build system and CLI without
requiring manual edits to rov.toml.
Authoritative source: ORIGINAL_REQUEST.md § R2, PROJECT.md § Feature 7 & 8.
"""

from __future__ import annotations

import unittest
from pathlib import Path

from tests.e2e.e2e_context import (
    REPO_ROOT,
    VALID_SCRATCH_C,
    run_rov_cli,
    temp_board,
    temp_scratch_file,
)
from tools.rov_cli.build import discover_ioc_targets
from tools.rov_cli.config import _parse_ioc_mcu, load_config


class TestR2BoardDiscovery(unittest.TestCase):
    """Verify dynamic board and .ioc discovery across boards/."""

    def test_r2_existing_standard_boards_discovered(self):
        """Verify standard baseline boards (f411, g474) are loaded in configuration."""
        cfg = load_config()
        boards = cfg.get("boards", {})
        self.assertTrue(
            "f411" in boards or "f411_nucleo" in boards,
            f"F411 board must be discovered in boards map: {list(boards.keys())}",
        )
        self.assertTrue(
            "g474" in boards or "g474_nucleo" in boards,
            f"G474 board must be discovered in boards map: {list(boards.keys())}",
        )

    def test_r2_discover_new_custom_board_without_toml_edit(self):
        """Verify: Adding a new directory boards/e2e_custom_board with .ioc is auto-discovered."""
        custom_board = "e2e_custom_board"
        with temp_board(custom_board, mcu="STM32F411RET6", family="STM32F4") as b_dir:
            self.assertTrue(b_dir.is_dir(), "Temporary board directory must exist")
            cfg = load_config()
            boards = cfg.get("boards", {})
            self.assertIn(
                custom_board,
                boards,
                f"Newly created board '{custom_board}' must be dynamically discovered without editing rov.toml",
            )
            board_entry = boards[custom_board]
            self.assertEqual(board_entry.get("mcu"), "STM32F411RET6")
            self.assertEqual(board_entry.get("architecture"), "arm-cortex-m4")

    def test_r2_parse_ioc_mcu_and_architecture_matrix(self):
        """Verify: _parse_ioc_mcu accurately detects MCU model and Cortex-M architecture."""
        test_cases = [
            ("Mcu.CPN=STM32C542CCT6\nMcu.Family=STM32C5\n", "STM32C542CCT6", "arm-cortex-m33"),
            ("Mcu.CPN=STM32G474RET6\nMcu.Family=STM32G4\n", "STM32G474RET6", "arm-cortex-m4"),
            ("Mcu.CPN=STM32F411RET6\nMcu.Family=STM32F4\n", "STM32F411RET6", "arm-cortex-m4"),
            ("Mcu.CPN=STM32H743ZIT6\nMcu.Family=STM32H7\n", "STM32H743ZIT6", "arm-cortex-m7"),
        ]

        for content, expected_mcu, expected_arch in test_cases:
            with temp_board("tmp_parse_test") as b_dir:
                ioc_path = b_dir / "tmp_parse_test.ioc"
                ioc_path.write_text(content, encoding="utf-8")
                mcu, arch = _parse_ioc_mcu(ioc_path)
                self.assertEqual(mcu, expected_mcu)
                self.assertEqual(arch, expected_arch)

    def test_r2_short_name_alias_registration(self):
        """Verify: boards named <short>_<suffix> register the short alias in boards map."""
        short_name = "testalpha"
        full_name = f"{short_name}_nucleo"
        with temp_board(full_name, mcu="STM32G474RET6", family="STM32G4"):
            cfg = load_config()
            boards = cfg.get("boards", {})
            self.assertIn(full_name, boards)
            self.assertIn(short_name, boards, f"Short name alias '{short_name}' must be registered")

    def test_r2_discover_ioc_targets_finds_all_unique_ioc_files(self):
        """Verify: discover_ioc_targets dynamically finds all .ioc files across repo."""
        lookup, unique_paths = discover_ioc_targets()
        self.assertGreater(len(unique_paths), 0, "Must discover at least baseline .ioc targets")

        # Baseline boards must be present in lookup
        for baseline in ("f411", "g474"):
            self.assertIn(baseline, lookup, f"Lookup must contain baseline '{baseline}'")
            self.assertTrue(lookup[baseline].is_file(), f"Target for '{baseline}' must be a file")

    def test_r2_generate_scaffolds_cmakelists_on_minimal_ioc(self):
        """Verify: 'rov generate <board>' scaffolds CMakeLists.txt on minimal .ioc without CubeMX output.

        Authoritative source: ORIGINAL_REQUEST.md § R2, Challenger M3 R1 handoff.
        """
        custom_board = "e2e_minimal_board"
        with temp_board(custom_board, mcu="STM32F411RETx", family="STM32F4") as b_dir:
            # Precondition: CMakeLists.txt must not exist
            cmake_file = b_dir / "CMakeLists.txt"
            self.assertFalse(cmake_file.exists(), "CMakeLists.txt must not exist prior to generate")

            # Execute rov generate <custom_board>
            res = run_rov_cli(["generate", custom_board])

            # Assert exit code 0 (non-fatal completion)
            self.assertEqual(
                res.returncode, 0,
                f"rov generate must exit with code 0 on minimal .ioc. Stderr: {res.stderr}\nStdout: {res.stdout}",
            )

            # Assert CMakeLists.txt was created and has non-zero size
            self.assertTrue(cmake_file.is_file(), "CMakeLists.txt was not scaffolded by rov generate")
            self.assertGreater(cmake_file.stat().st_size, 0, "Scaffolded CMakeLists.txt must not be empty")

            # Verify content of scaffolded CMakeLists.txt
            content = cmake_file.read_text(encoding="utf-8")
            self.assertIn(f"{custom_board}_sandbox.elf", content)
            self.assertIn("add_board_target_executable", content)
            self.assertIn("FALLBACK_CORE_SRCS", content)
            self.assertIn("rov_shared", content)
            self.assertIn("rov_drivers", content)
            self.assertIn("rov_services", content)

            # Verify non-fatal notification in stdout
            self.assertTrue(
                "[SUCCESS] Scaffolded" in res.stdout or "Notice:" in res.stdout,
                f"Expected scaffold success or informative notice in output: {res.stdout}",
            )

    def test_r2_generate_zz_can_succeeds_without_error(self):
        """Verify: 'rov generate zz_can' completes with code 0 without unhandled exit 1.

        Authoritative source: ORIGINAL_REQUEST.md § R2, Challenger M3 R1 handoff.
        """
        zz_can_dir = REPO_ROOT / "boards" / "zz_can"
        self.assertTrue(zz_can_dir.is_dir(), "boards/zz_can must exist in repository")
        self.assertTrue((zz_can_dir / "zz_can.ioc").is_file(), "boards/zz_can/zz_can.ioc must exist")

        # Execute generate on existing zz_can target
        res = run_rov_cli(["generate", "zz_can"])

        self.assertEqual(
            res.returncode, 0,
            f"rov generate zz_can must exit with 0. Stderr: {res.stderr}\nStdout: {res.stdout}",
        )

        # Ensure existing CMakeLists.txt was preserved and valid
        cmake_path = zz_can_dir / "CMakeLists.txt"
        self.assertTrue(cmake_path.is_file())
        self.assertIn("zz_can_sandbox.elf", cmake_path.read_text(encoding="utf-8"))

        # Confirm output contains non-blocking notice regarding missing Core/Src/main.c
        self.assertIn("Notice:", res.stdout)
        self.assertNotIn("Error: CubeMX ran but produced no Core/Src/main.c", res.stdout)

    def test_r2_scratch_build_after_generate_on_custom_board(self):
        """Verify: Immediate compilation of scratch C file on a newly generated custom board."""
        board_name = "test_gen_board"
        with temp_board(board_name, mcu="STM32F411RETx", family="STM32F4") as b_dir:
            # Step 1 & 2: Generate board
            res_gen = run_rov_cli(["generate", board_name])
            self.assertEqual(res_gen.returncode, 0, f"Generate failed: {res_gen.stderr}")
            self.assertTrue((b_dir / "CMakeLists.txt").is_file())

            # Step 3 & 4: Build scratch file
            with temp_scratch_file(VALID_SCRATCH_C) as scratch:
                res_build = run_rov_cli(["build", str(scratch), "-b", board_name])
                self.assertEqual(
                    res_build.returncode, 0,
                    f"Scratch build failed on newly generated board:\nStdout: {res_build.stdout}\nStderr: {res_build.stderr}",
                )

                # Step 5: Verify ELF output
                expected_elf = REPO_ROOT / "build" / "arm-c542-debug" / "boards" / board_name / f"{board_name}_sandbox.elf"
                self.assertTrue(expected_elf.is_file(), f"Output binary missing: {expected_elf}")

                # Verify ELF header magic (\x7fELF)
                header = expected_elf.read_bytes()[:4]
                self.assertEqual(header, b"\x7fELF", "Output binary must be a valid ELF binary")


if __name__ == "__main__":
    unittest.main()
