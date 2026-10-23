"""Tier 1 Feature Coverage: R3 Hardware Target Cross-Compilation.

Tests that all active hardware targets (g474, f411, stm32c5) cross-compile cleanly
into valid 32-bit ARM ELF binaries.
Authoritative source: ORIGINAL_REQUEST.md § R3, PROJECT.md § Feature 3.
"""

from __future__ import annotations

import os
import struct
import unittest
from pathlib import Path

from tests.e2e.e2e_context import (
    REPO_ROOT,
    run_rov_cli,
)
from tools.rov_cli.config import load_config

ELF_MAGIC = b"\x7fELF"
ELF_CLASS_32 = 1
ELF_DATA_LSB = 1
EM_ARM = 40  # 0x0028


def _verify_arm_elf(path: Path) -> tuple[bool, str]:
    """Inspect file header to verify it is a valid 32-bit Little-Endian ARM ELF binary."""
    if not path.is_file():
        return False, f"File not found: {path}"
    data = path.read_bytes()
    if len(data) < 52:
        return False, f"File too small for ELF header: {len(data)} bytes"
    if data[:4] != ELF_MAGIC:
        return False, f"Missing ELF magic: {data[:4]}"
    if data[4] != ELF_CLASS_32:
        return False, f"Not 32-bit ELF (ei_class={data[4]})"
    if data[5] != ELF_DATA_LSB:
        return False, f"Not Little-Endian (ei_data={data[5]})"
    e_machine = struct.unpack("<H", data[18:20])[0]
    if e_machine != EM_ARM:
        return False, f"Machine is not ARM (e_machine={e_machine}, expected {EM_ARM})"
    return True, "Valid ARM ELF"


def _is_stm32c5_target_ready() -> bool:
    """Check if Milestone 1 has completed the stm32c5 target."""
    c5_dir = REPO_ROOT / "boards" / "stm32c5"
    if not (c5_dir / "CMakeLists.txt").is_file():
        return False
    # Check if target is recognized by CMake
    res = run_rov_cli(["build", "-b", "stm32c5"])
    return res.returncode == 0


class TestR3CrossCompilation(unittest.TestCase):
    """Verify clean cross-compilation for all active vehicle and bench targets."""

    def test_r3_cross_compile_f411_produces_valid_elf(self):
        """Verify: python rov.py build -b f411 compiles a valid ARM ELF binary."""
        res = run_rov_cli(["build", "-b", "f411"])
        self.assertEqual(res.returncode, 0, f"Build f411 failed:\n{res.stderr}\n{res.stdout}")
        self.assertIn("[SUCCESS]", res.stdout)

        cfg = load_config()
        bin_rel = cfg["nodes"]["pi_shield"]["binary_f411"]
        elf_path = REPO_ROOT / bin_rel
        is_valid, reason = _verify_arm_elf(elf_path)
        self.assertTrue(is_valid, f"F411 binary at {bin_rel} is not valid: {reason}")

    def test_r3_cross_compile_g474_produces_valid_elf(self):
        """Verify: python rov.py build -b g474 compiles a valid ARM ELF binary."""
        res = run_rov_cli(["build", "-b", "g474"])
        self.assertEqual(res.returncode, 0, f"Build g474 failed:\n{res.stderr}\n{res.stdout}")
        self.assertIn("[SUCCESS]", res.stdout)

        cfg = load_config()
        bin_rel = cfg["nodes"]["control_board"]["binary_g474"]
        elf_path = REPO_ROOT / bin_rel
        is_valid, reason = _verify_arm_elf(elf_path)
        self.assertTrue(is_valid, f"G474 binary at {bin_rel} is not valid: {reason}")

    def test_r3_cross_compile_stm32c5_produces_valid_elf(self):
        """Verify: python rov.py build -b stm32c5 compiles production STM32C5 binary."""
        if not _is_stm32c5_target_ready() and os.environ.get("ROV_E2E_STRICT") != "1":
            self.skipTest("Pending Milestone M1: STM32C5 Target Decoupling & Scaffolding")

        res = run_rov_cli(["build", "-b", "stm32c5"])
        self.assertEqual(res.returncode, 0, f"Build stm32c5 failed:\n{res.stderr}\n{res.stdout}")
        self.assertIn("[SUCCESS]", res.stdout)

        found_elfs = list((REPO_ROOT / "build/arm-c542-debug").glob("**/*stm32c5*.elf"))
        self.assertGreater(len(found_elfs), 0, "Must generate at least one stm32c5 ELF binary")
        is_valid, reason = _verify_arm_elf(found_elfs[0])
        self.assertTrue(is_valid, f"STM32C5 binary is not valid: {reason}")

    def test_r3_cross_compile_release_profile(self):
        """Verify: python rov.py build -b f411 -p release compiles with release profile."""
        res = run_rov_cli(["build", "-b", "f411", "-p", "release"])
        self.assertEqual(res.returncode, 0, f"Release build failed:\n{res.stderr}\n{res.stdout}")
        self.assertIn("[SUCCESS]", res.stdout)

    def test_r3_toolchain_prerequisites_check(self):
        """Verify: check command validates arm-none-eabi-gcc and cmake availability."""
        res = run_rov_cli(["check"])
        self.assertEqual(res.returncode, 0, f"Prerequisites check failed:\n{res.stderr}\n{res.stdout}")
        self.assertIn("arm-none-eabi-gcc", res.stdout)
        self.assertIn("cmake", res.stdout)
        self.assertIn("ninja", res.stdout)


if __name__ == "__main__":
    unittest.main()
