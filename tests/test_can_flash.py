import pytest
from unittest.mock import patch, mock_open
import sys
import os

# Add bootloader/tools to sys.path to import can_flash
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'bootloader', 'tools'))

from can_flash import flash_node

def test_flash_node_exceeds_max_chunks():
    # Mocking open to return a file read of size greater than 65535 * 60 bytes
    dummy_data = b'x' * (60 * 65536)
    with patch('builtins.open', new_callable=mock_open, read_data=dummy_data):
        # We also need to mock `can.Bus` to avoid trying to open a real CAN interface
        with patch('can_flash.can.Bus'):
            result = flash_node("can0", "pi_shield", "dummy.bin")
            assert result is False
