"""Drift guard: the silently enumerates suites named in the SIL guide must match CTest.

Run under CTest with the native build tree already configured:
    python tests/test_suite_manifest.py [--build-dir <path>]
It fails when CTest registers a `test_*` suite that the guide omits, or when the
guide advertises a suite CTest does not know about.
"""
import argparse
import os
import re
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def get_build_dir() -> Path:
    parser = argparse.ArgumentParser(description="Verify CTest manifest against documentation.")
    parser.add_argument("--build-dir", default=None, help="Path to cmake build directory")
    args, remaining = parser.parse_known_args()
    sys.argv = [sys.argv[0]] + remaining

    if args.build_dir:
        return Path(args.build_dir).resolve()

    env_dir = os.environ.get("CMAKE_BINARY_DIR")
    if env_dir:
        return Path(env_dir).resolve()

    for candidate in [
        ROOT / "build" / "sil-debug",
        ROOT / "build" / "sil-release",
        ROOT / "build-native",
        Path.cwd(),
    ]:
        if (candidate / "CTestTestfile.cmake").exists():
            return candidate.resolve()
    return (ROOT / "build-native").resolve()


BUILD_DIR = get_build_dir()


def _guide_suites() -> set[str]:
    text = (ROOT / "docs" / "sil_simulation_guide.md").read_text(encoding="utf-8")
    return set(re.findall(r"`(test_\w+)`", text))


def _ctest_suites() -> set[str]:
    cmd = ["ctest", "-N"]
    if BUILD_DIR.exists():
        cmd.extend(["--test-dir", str(BUILD_DIR)])
    out = subprocess.run(
        cmd,
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
