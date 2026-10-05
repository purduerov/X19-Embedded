"""Consistency check: the CMake test registry must agree with rov.toml.

The CMake node registry (x19_add_node_test calls) identify nodes with
`NODE <key>`. rov.toml is authoritative for the CLI. This test fails when a
vehicle SIL node is missing from one source so the two cannot silently drift.

Scoped to the vehicle SIL nodes (pi_shield, control_board, power_slab): the
`rnd` and `sandbox` entries in rov.toml have no x19_add_node_test target.
"""
import re
import tomllib
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CMAKE_FILE = ROOT / "tests" / "CMakeLists.txt"
ROV_TOML = ROOT / "rov.toml"

SIL_NODE_KEYS = ("pi_shield", "control_board", "power_slab")


class TestRegistryConsistency(unittest.TestCase):
    def test_every_silver_node_has_a_cmake_target(self):
        cmake = CMAKE_FILE.read_text(encoding="utf-8")
        rov = tomllib.loads(ROV_TOML.read_text(encoding="utf-8"))
        missing = []
        for key in SIL_NODE_KEYS:
            if key not in rov.get("nodes", {}):
                missing.append(f"{key} missing from rov.toml")
            if not re.search(rf"NODE\s+{re.escape(key)}\b", cmake):
                missing.append(f"{key} missing from tests/CMakeLists.txt")
        self.assertEqual([], missing, "\n".join(missing))

    def test_node_supported_boards_match_supported_host(self):
        rov = tomllib.loads(ROV_TOML.read_text(encoding="utf-8"))
        for key in SIL_NODE_KEYS:
            boards = rov["nodes"][key].get("supported_boards", [])
            self.assertIn("host", boards, f"{key} must list the `host` board")


if __name__ == "__main__":
    unittest.main()
