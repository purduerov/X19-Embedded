"""Tier 4 Real-World Scenario: Telemetry Streaming & Decoding Workflow.

Tests high-frequency subsea telemetry streams (Nav 100 Hz, Env 10 Hz, Power 20 Hz),
verifying packet encoding, decoding, and telemetry validation.
Authoritative source: ORIGINAL_REQUEST.md § R4, PROJECT.md § Tier 4.
"""

from __future__ import annotations

import struct
import unittest

from tests.e2e.e2e_context import (
    CAN_ID_ENV_TELEMETRY,
    CAN_ID_NAV_TELEMETRY,
    CAN_ID_POWER_TELEMETRY,
    pack_sil_can_frame,
    unpack_sil_can_frame,
)


class TestTelemetryStreamWorkflow(unittest.TestCase):
    """Verify telemetry streaming, packet layouts, and decoding pipelines."""

    def test_nav_telemetry_stream_encoding_and_decoding(self):
        """Verify: 100 Hz Nav telemetry (0x200) packs and unpacks accurately."""
        # 3-axis accel (3 x float), 3-axis gyro (3 x float), depth (float), temp (float) = 32 bytes
        nav_format = "<8f"
        test_values = (0.01, -0.02, 9.81, 0.05, -0.04, 0.02, 10.5, 22.4)
        nav_payload = struct.pack(nav_format, *test_values)

        frame = pack_sil_can_frame(CAN_ID_NAV_TELEMETRY, nav_payload)
        _, arb_id, length, data = unpack_sil_can_frame(frame)

        self.assertEqual(arb_id, CAN_ID_NAV_TELEMETRY)
        self.assertEqual(length, 32)
        unpacked_values = struct.unpack(nav_format, data)
        for expected, actual in zip(test_values, unpacked_values):
            self.assertAlmostEqual(expected, actual, places=4)

    def test_env_telemetry_stream_encoding_and_decoding(self):
        """Verify: 10 Hz Enclosure Environmental telemetry (0x210) packs and unpacks accurately."""
        # temp (float), humidity (float), pressure (float), leak_flags (uint32) = 16 bytes
        env_format = "<fffI"
        test_values = (24.5, 35.0, 1013.25, 0)
        env_payload = struct.pack(env_format, *test_values)

        frame = pack_sil_can_frame(CAN_ID_ENV_TELEMETRY, env_payload)
        _, arb_id, length, data = unpack_sil_can_frame(frame)

        self.assertEqual(arb_id, CAN_ID_ENV_TELEMETRY)
        self.assertEqual(length, 16)
        t, h, p, leaks = struct.unpack(env_format, data)
        self.assertAlmostEqual(t, 24.5, places=2)
        self.assertAlmostEqual(h, 35.0, places=2)
        self.assertAlmostEqual(p, 1013.25, places=2)
        self.assertEqual(leaks, 0)

    def test_power_telemetry_stream_encoding_and_decoding(self):
        """Verify: 20 Hz Power telemetry (0x300) packs and unpacks accurately."""
        # bus_voltage (float), total_current (float), power (float), fault_mask (uint32)
        power_format = "<fffI"
        test_values = (48.2, 12.5, 602.5, 0)
        power_payload = struct.pack(power_format, *test_values)

        frame = pack_sil_can_frame(CAN_ID_POWER_TELEMETRY, power_payload)
        _, arb_id, length, data = unpack_sil_can_frame(frame)

        self.assertEqual(arb_id, CAN_ID_POWER_TELEMETRY)
        v, i, p, faults = struct.unpack(power_format, data)
        self.assertAlmostEqual(v, 48.2, places=2)
        self.assertAlmostEqual(i, 12.5, places=2)
        self.assertAlmostEqual(p, 602.5, places=2)
        self.assertEqual(faults, 0)


if __name__ == "__main__":
    unittest.main()
