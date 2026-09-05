import time
import unittest
import can
import tools.can_sniffer as sniffer


class TestCanSniffer(unittest.TestCase):

    def test_format_timestamp_correctness(self):
        # Test that _format_timestamp produces expected time formatting
        ts = 1700000000.123
        expected = time.strftime("%H:%M:%S", time.localtime(ts))
        actual = sniffer._format_timestamp(ts)
        self.assertEqual(actual, expected)

    def test_format_timestamp_caching(self):
        # Test that timestamp caching reuses the string within the same second
        ts1 = 1700000005.100
        ts2 = 1700000005.900
        str1 = sniffer._format_timestamp(ts1)
        str2 = sniffer._format_timestamp(ts2)
        self.assertIs(str1, str2)

        # Test new second updates the cached string
        ts3 = 1700000006.000
        str3 = sniffer._format_timestamp(ts3)
        self.assertEqual(str3, time.strftime("%H:%M:%S", time.localtime(ts3)))

    def test_decode_msg_handles_valid_messages(self):
        # Test decoding messages without exception
        msg_nav = can.Message(
            arbitration_id=sniffer.CAN_ID_NAV_TELEMETRY,
            data=bytes(33),
            timestamp=1700000010.500,
        )
        sniffer.decode_msg(msg_nav)

        msg_emergency = can.Message(
            arbitration_id=sniffer.CAN_ID_EMERGENCY_BREAK,
            data=bytes(8),
            timestamp=1700000010.600,
        )
        sniffer.decode_msg(msg_emergency)


if __name__ == "__main__":
    unittest.main()
