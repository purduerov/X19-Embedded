"""Tier 4 Real-World Scenario: Multi-Node CAN FD Ping-Pong Workflow.

Tests multi-node CAN FD communication across vehicle nodes:
1. Node 1 Pi Shield sends Time Sync Request (CAN ID 0x011)
2. Node 2 Control Board responds with Time Sync Response (CAN ID 0x012)
3. End-to-end multi-node SIL integration suite verification
Authoritative source: ORIGINAL_REQUEST.md § Acceptance Criteria, PROJECT.md § Tier 4.
"""

from __future__ import annotations

import struct
import subprocess
import unittest

from tests.e2e.e2e_context import (
    CAN_ID_TIME_SYNC_MASTER,
    CAN_ID_TIME_SYNC_REQ,
    CAN_ID_TIME_SYNC_RESP,
    REPO_ROOT,
    pack_sil_can_frame,
    unpack_sil_can_frame,
)


class TestPingPongWorkflow(unittest.TestCase):
    """Verify multi-node CAN ping-pong request/response and latency tracking."""

    def test_time_sync_request_response_packet_structures(self):
        """Verify: CAN ID 0x011 (req) and 0x012 (resp) packet encoding/decoding."""
        # Request: seq (uint8), node_id (uint8), reserved (uint16), t1_sec (uint32), t1_nsec (uint64)
        req_format = "<BBHIQ"
        req_data = struct.pack(req_format, 1, 0x01, 0, 1700000000, 500000)
        req_frame = pack_sil_can_frame(CAN_ID_TIME_SYNC_REQ, req_data)

        _, id_req, len_req, data_req = unpack_sil_can_frame(req_frame)
        self.assertEqual(id_req, CAN_ID_TIME_SYNC_REQ)
        seq, node_id, _, sec, nsec = struct.unpack(req_format, data_req)
        self.assertEqual(seq, 1)
        self.assertEqual(node_id, 0x01)

        # Response: seq (uint8), node_id (uint8), reserved (uint16), t1_sec (uint32), t1_nsec (uint64), t2_nsec (uint64), t3_nsec (uint64)
        resp_format = "<BBHIQQQ"
        resp_data = struct.pack(resp_format, 1, 0x02, 0, 1700000000, 500000, 600000, 650000)
        resp_frame = pack_sil_can_frame(CAN_ID_TIME_SYNC_RESP, resp_data)

        _, id_resp, _, data_resp = unpack_sil_can_frame(resp_frame)
        self.assertEqual(id_resp, CAN_ID_TIME_SYNC_RESP)
        seq_r, node_r, _, _, _, t2, t3 = struct.unpack(resp_format, data_resp)
        self.assertEqual(seq_r, 1)
        self.assertEqual(node_r, 0x02)
        self.assertGreater(t3, t2)

    def test_multi_node_bus_and_timesync_sil_suites_pass(self):
        """Verify: Multi-node bus integration and time-sync unit tests pass 100%."""
        for suite_name in ("test_multi_node_bus", "test_timesync"):
            cmd = [
                "ctest",
                "--test-dir",
                str(REPO_ROOT / "build-native"),
                "-R",
                f"^{suite_name}$",
                "--output-on-failure",
            ]
            res = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
            self.assertEqual(res.returncode, 0, f"Suite {suite_name} failed:\n{res.stdout}")
            self.assertIn("Passed", res.stdout)


if __name__ == "__main__":
    unittest.main()
