"""Tier 2 Boundary & Corner Cases: Invalid C Syntax & Compilation Failures.

Tests compiler diagnostic reporting and error handling when scratch C files contain
syntax errors, missing symbols, or preprocessor errors.
Authoritative source: ORIGINAL_REQUEST.md § R1, PROJECT.md § Boundary Cases.
"""

from __future__ import annotations

import unittest

from tests.e2e.e2e_context import (
    MISSING_APP_MAIN_C,
    PREPROCESSOR_ERROR_C,
    SYNTAX_ERROR_C,
    run_rov_cli,
    temp_scratch_file,
)
from tools.rov_cli.build import build_target


class TestInvalidCSyntax(unittest.TestCase):
    """Verify clean compilation failures when invalid C code is provided."""

    def test_invalid_syntax_c_file_compilation_fails(self):
        """Verify: Scratch file with syntax errors causes compilation failure with non-zero exit."""
        with temp_scratch_file(SYNTAX_ERROR_C) as scratch:
            with self.assertRaises(SystemExit) as cm:
                build_target("sandbox", "f411", "debug", custom_file=str(scratch))
            self.assertNotEqual(cm.exception.code, 0)

    def test_missing_app_main_symbol_link_fails(self):
        """Verify: Scratch file missing app_main() fails at link time with undefined reference."""
        with temp_scratch_file(MISSING_APP_MAIN_C) as scratch:
            with self.assertRaises(SystemExit) as cm:
                build_target("sandbox", "f411", "debug", custom_file=str(scratch))
            self.assertNotEqual(cm.exception.code, 0)

    def test_preprocessor_error_directive_aborts_compilation(self):
        """Verify: C file with #error directive halts compilation."""
        with temp_scratch_file(PREPROCESSOR_ERROR_C) as scratch:
            with self.assertRaises(SystemExit) as cm:
                build_target("sandbox", "f411", "debug", custom_file=str(scratch))
            self.assertNotEqual(cm.exception.code, 0)

    def test_empty_c_file_link_fails(self):
        """Verify: Completely empty 0-byte .c file fails linking due to missing app_main."""
        with temp_scratch_file("") as scratch:
            with self.assertRaises(SystemExit) as cm:
                build_target("sandbox", "f411", "debug", custom_file=str(scratch))
            self.assertNotEqual(cm.exception.code, 0)

    def test_cli_reports_compiler_diagnostics_on_syntax_error(self):
        """Verify: rov CLI surfaces GCC diagnostic output on build failure."""
        with temp_scratch_file(SYNTAX_ERROR_C) as scratch:
            res = run_rov_cli(["build", "-f", str(scratch), "-b", "f411"])
            self.assertNotEqual(res.returncode, 0)
            combined = res.stderr + res.stdout
            self.assertTrue(
                "error:" in combined or "failed" in combined.lower(),
                f"CLI must report compiler diagnostic error, got:\n{combined}",
            )


if __name__ == "__main__":
    unittest.main()
