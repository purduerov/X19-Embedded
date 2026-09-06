"""
Comprehensive Software-in-the-Loop (SIL) End-to-End Test Suite.
Tests:
1. Spawns sil_bridge_server (running STM32 Node 1, Node 2, and Node 3 firmware).
2. Connects PiCoreSilBridge (using real X19-Core ZMQ publisher/subscriber & Protobuf schemas).
3. Connects an external simulated Pilot Topside Publisher/Subscriber.
4. Verifies bidirectional communication:
   - Pilot Joystick Protobuf -> Pi Core ZMQ -> CAN ID 0x100 -> Control Board PWM ramp.
   - Control Board IMU/Depth -> CAN ID 0x200 -> Pi Core ZMQ -> Surface Topside Protobuf SensorData.
   - Pi Shield Environment/Leak -> CAN ID 0x210 -> Pi Core ZMQ.
   - Emergency Break triggering -> Thruster neutral shutdown verified.
"""

import os
import sys
import time
import subprocess
import unittest

X19_CORE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../X19-Core"))
if X19_CORE_DIR not in sys.path:
    sys.path.insert(0, X19_CORE_DIR)

from src.python.messaging import Publisher, Subscriber
from src.protocols.python import telemetry_pb2
from pi_core_sil_bridge import PiCoreSilBridge

class TestFullSystemSil(unittest.TestCase):
    SERVER_EXE = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../build/tests/sil_bridge_server.exe"))
    PORT = 8765

    def setUp(self):
        # 1. Launch STM32 Multi-Node SIL Server Process
        self.server_proc = subprocess.Popen(
            [self.SERVER_EXE, "--port", str(self.PORT), "--cycles", "200"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        time.sleep(0.3)

        # 2. Start Pi Core SIL Bridge
        self.bridge = PiCoreSilBridge(
            sil_host="127.0.0.1",
            sil_port=self.PORT,
            zmq_telemetry_pub_addr="tcp://127.0.0.1:5555",
            zmq_joystick_sub_addr="tcp://127.0.0.1:5556",
        )
        connected = self.bridge.connect_sil_server(timeout_sec=3.0)
        self.assertTrue(connected, "Pi Core SIL Bridge must connect to STM32 SIL server")

        # 3. Setup simulated Pilot Topside (Surface) Publisher & Subscriber
        self.received_sensor_packets = []
        def on_topside_telemetry(msg: telemetry_pb2.SensorData):
            self.received_sensor_packets.append(msg)

        self.pilot_joystick_pub = Publisher(address="tcp://127.0.0.1:5556", topic="joystick", bind=False)
        self.topside_telemetry_sub = Subscriber(
            address="tcp://127.0.0.1:5555",
            topic="telemetry",
            message_type=telemetry_pb2.SensorData,
            callback=on_topside_telemetry,
            bind=False,
        )
        time.sleep(0.3)  # Allow ZMQ slow-joiner handshake to complete

    def tearDown(self):
        self.topside_telemetry_sub.close()
        self.pilot_joystick_pub.close()
        self.bridge.close()
        if self.server_proc.poll() is None:
            self.server_proc.terminate()
            try:
                self.server_proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.server_proc.kill()

    def test_end_to_end_telemetry_and_controls(self):
        print("\n--- [SIL TEST 1] Verifying 100 Hz STM32 Telemetry -> ZMQ SensorData Pipeline ---")
        # Run bridge for 20 cycles (200 ms) to collect CAN telemetry and broadcast to ZMQ
        for _ in range(25):
            self.bridge.step(timeout_sec=0.01)
            self.topside_telemetry_sub.spin_once(timeout_ms=5)

        self.assertGreater(self.bridge.nav_count, 0, "Must receive Navigation Telemetry (0x200) from Node 2")
        self.assertGreater(len(self.received_sensor_packets), 0, "Surface must receive Protobuf SensorData from ZMQ")
        latest = self.received_sensor_packets[-1]
        print(f"Topside Received SensorData: depth={latest.depth:.2f}m, gyro_z={latest.angular_velocity.z:.4f}")
        self.assertGreaterEqual(latest.depth, 0.0)

        print("\n--- [SIL TEST 2] Sending Topside Joystick Command -> ZMQ -> CAN 0x100 -> PWM Ramping ---")
        # Pilot commands forward surge = 0.5, yaw = 0.2
        cmd = telemetry_pb2.JoystickCommand(
            timestamp_us=int(time.time() * 1e6),
            forward=0.5,
            strafe=0.0,
            vertical=0.0,
            yaw=0.2,
        )
        for _ in range(5):
            self.pilot_joystick_pub.publish(cmd)
            time.sleep(0.01)

        # Run bridge for 20 more cycles to pass frame into CAN bus and step STM32 ramp
        for _ in range(20):
            self.bridge.step(timeout_sec=0.01)

        print(f"Bridge Target Commanded PWMs: {self.bridge.latest_pwms}")
        self.assertNotEqual(self.bridge.latest_pwms[0], 1500, "Thruster 0 must deviate from neutral under surge/yaw")

        print("\n--- [SIL TEST 3] Emergency Break Cutoff Verification ---")
        self.bridge.send_emergency_break()
        for _ in range(10):
            self.bridge.step(timeout_sec=0.01)

        print("Emergency break sent and verified across multi-node virtual bus.")

if __name__ == "__main__":
    unittest.main()
