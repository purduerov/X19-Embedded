import unittest
from unittest.mock import patch, mock_open, MagicMock
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'bootloader', 'tools'))
import can_flash
from can_flash import flash_node

class TestCanFlash(unittest.TestCase):
    def test_flash_node_exceeds_max_chunks(self):
        dummy_data = b'x' * (60 * 65536)
        with patch('builtins.open', new_callable=mock_open, read_data=dummy_data):
            mock_can = MagicMock()
            # Return an ACK message so wait_for_ack succeeds instantly
            ack_msg = MagicMock()
            ack_msg.arbitration_id = 0x700
            ack_msg.data = [0x06, 0x01]  # ACK, pi_shield (0x01)
            mock_can.Bus.return_value.recv.return_value = ack_msg

            with patch.object(can_flash, 'can', mock_can):
                result = flash_node("can0", "pi_shield", "dummy.bin")
                self.assertFalse(result)

if __name__ == '__main__':
    unittest.main()
