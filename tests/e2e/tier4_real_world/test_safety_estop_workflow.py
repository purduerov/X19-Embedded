"""Tier 4 Real-World Scenario: Subsea Safety & Emergency Break Workflow.

Tests critical subsea safety invariants:
1. Thruster neutral PWM is strictly 1500 us
2. Emergency Break frame (CAN ID 0x001) immediately disarms all thrusters to 1500 us
3. Heartbeat loss (>250 ms) disarms thrusters to neutral fail-safe state
Authoritative source: GEMINI.md Fail-Safe Invariants, ORIGINAL_REQUEST.md § Acceptance Criteria.
"""

from __future__ import annotations

import struct
import subprocess
import unittest

from tests.e2e.e2e_context import (
    CAN_ID_EMERGENCY_BREAK,
    CAN_ID_THRUSTER_CMD,
    REPO_ROOT,
    pack_sil_can_frame,
    unpack_sil_can_frame,
)


class TestSafetyEstopWorkflow(unittest.TestCase):
    """Verify emergency stop response and thruster fail-safe invariants."""

    def test_emergency_break_packet_framing_and_invariants(self):
        """Verify: E-stop CAN ID 0x001 is highest priority arbitration ID (0x001 < 0x100)."""
        self.assertLess(
            CAN_ID_EMERGENCY_BREAK,
            CAN_ID_THRUSTER_CMD,
            "Emergency break must have higher CAN arbitration priority than thruster commands",
        )

        # Pack emergency break packet
        estop_frame = pack_sil_can_frame(CAN_ID_EMERGENCY_BREAK, b"\x01\x00\x00\x00")
        magic, arb_id, length, data = unpack_sil_can_frame(estop_frame)
        self.assertEqual(arb_id, CAN_ID_EMERGENCY_BREAK)
        self.assertEqual(data[:1], b"\x01")

    def test_sil_safety_test_suite_passes(self):
        """Verify: Host SIL safety suite (test_safety and test_sil_safety) pass 100%."""
        cmd = [
            "ctest",
            "--test-dir",
            str(REPO_ROOT / "build-native"),
            "-R",
            "safety",
            "--output-on-failure",
        ]
        res = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
        self.assertEqual(res.returncode, 0, f"Safety test suite failed: {res.stdout}")
        self.assertIn("100% tests passed", res.stdout)


if __name__ == "__main__":
    unittest.main()
