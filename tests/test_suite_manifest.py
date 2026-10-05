"""Drift guard: the silently enumerates suites named in the SIL guide must match CTest.

Run under CTest with the native build tree already configured:
    python tests/test_suite_manifest.py
It fails when CTest registers a `test_*` suite that the guide omits, or when the
guide advertises a suite CTest does not know about.
"""
import re
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def _guide_suites() -> set[str]:
    text = (ROOT / "docs" / "sil_simulation_guide.md").read_text(encoding="utf-8")
    return set(re.findall(r"`(test_\w+)`", text))


def _ctest_suites() -> set[str]:
    out = subprocess.run(
        ["ctest", "--test-dir", str(ROOT / "build-native"), "-N"],
        capture_output=True, text=True, check=True,
    )
    return set(re.findall(r"Test\s+#?\d+:\s+(test_\w+)", out.stdout))


class TestSuiteManifest(unittest.TestCase):
    def test_guide_lists_every_ctest_suite(self):
        guide = _guide_suites()
        for name in sorted(_ctest_suites()):
            self.assertIn(name, guide, f"{name} is registered with CTest but missing from the guide")


if __name__ == "__main__":
    unittest.main()
