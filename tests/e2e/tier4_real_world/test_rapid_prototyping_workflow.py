"""Tier 4 Real-World Scenario: Rapid Prototyping Developer Workflow.

Models a developer prototyping a new algorithm:
1. Writing a standalone scratch C file implementing app_main()
2. Compiling the scratch file for bench hardware (NUCLEO-F411)
3. Verifying output binary generation
4. Running the host SIL test suite to verify no regressions in vehicle safety logic
Authoritative source: ORIGINAL_REQUEST.md § R1, PROJECT.md § Tier 4.
"""

from __future__ import annotations

import subprocess
import unittest
from pathlib import Path

from tests.e2e.e2e_context import (
    REPO_ROOT,
    temp_scratch_file,
)
from tools.rov_cli.build import build_target

ALGORITHM_PROTOTYPE_C = """#include <stdint.h>
#include <stdbool.h>

static uint16_t clamp_pwm(uint16_t pulse_us) {
    if (pulse_us < 1100) return 1100;
    if (pulse_us > 1900) return 1900;
    return pulse_us;
}

void app_main(void) {
    volatile uint16_t thruster_pwm = 1500;
    thruster_pwm = clamp_pwm(1750);
    (void)thruster_pwm;
}
"""


class TestRapidPrototypingWorkflow(unittest.TestCase):
    """Verify rapid iteration workflow from scratch C file to compiled binary and regression tests."""

    def test_prototype_compile_and_verify_regression_free(self):
        """Workflow: Prototype PWM clamp in scratch C -> build for F411 -> verify SIL safety tests."""
        # 1. Compile prototype algorithm in sandbox
        with temp_scratch_file(ALGORITHM_PROTOTYPE_C) as scratch:
            elf = build_target("sandbox", "f411", "debug", custom_file=str(scratch))
            self.assertTrue(Path(elf).is_file(), f"Target ELF must exist: {elf}")

            # 2. Verify that host SIL test suites remain 100% passing
            cmd = [
                "ctest",
                "--test-dir",
                str(REPO_ROOT / "build-native"),
                "-R",
                "^test_safety$",
                "--output-on-failure",
            ]
            res = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
            self.assertEqual(res.returncode, 0, f"Safety test regression detected: {res.stdout}")
            self.assertIn("Passed", res.stdout)


if __name__ == "__main__":
    unittest.main()
