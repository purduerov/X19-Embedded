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

import math
import os
import socket
import sys
import time
import subprocess
import unittest
import importlib.util

X19_CORE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../X19-Core"))
if X19_CORE_DIR not in sys.path:
    sys.path.insert(0, X19_CORE_DIR)

X19_SURFACE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../X19-Surface"))


def _load_repo_module(module_name: str, file_path: str):
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load {module_name} from {file_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


# Core owns the vehicle-side ZMQ endpoints; Surface owns the topside consumer.
from src.python.messaging import Publisher
from src.protocols.python import telemetry_pb2 as core_telemetry_pb2

surface_subscriber_module = _load_repo_module(
    "x19_surface_sil_subscriber",
    os.path.join(X19_SURFACE_DIR, "src", "zmq", "python", "messaging", "subscriber.py"),
)
surface_telemetry_pb2 = _load_repo_module(
    "x19_surface_sil_telemetry_pb2",
    os.path.join(X19_SURFACE_DIR, "src", "zmq", "protocols", "python", "telemetry_pb2.py"),
)
SurfaceSubscriber = surface_subscriber_module.Subscriber
from pi_core_sil_bridge import PiCoreSilBridge

def find_server_binary() -> str:
    base = os.path.dirname(__file__)
    configured_binary = os.environ.get("X19_SIL_SERVER")
    if configured_binary:
        if not os.path.isfile(configured_binary):
            raise FileNotFoundError(f"X19_SIL_SERVER does not exist: {configured_binary}")
        return os.path.abspath(configured_binary)
    candidates = [
        os.path.abspath(os.path.join(base, "../../build-native/tests/sil_bridge_server")),
        os.path.abspath(os.path.join(base, "../../build-native/tests/sil_bridge_server.exe")),
        os.path.abspath(os.path.join(base, "../../build/tests/sil_bridge_server.exe")),
        os.path.abspath(os.path.join(base, "../../build/tests/sil_bridge_server")),
        os.path.abspath(os.path.join(base, "../../tests/build/tests/sil_bridge_server.exe")),
        os.path.abspath(os.path.join(base, "../../tests/build/tests/sil_bridge_server")),
    ]
    for c in candidates:
        if os.path.isfile(c) and os.access(c, os.X_OK | os.R_OK):
            return c
    # Fallback to default
    return candidates[0]

class TestFullSystemSil(unittest.TestCase):
    @staticmethod
    def _free_tcp_port() -> int:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.bind(("127.0.0.1", 0))
            return probe.getsockname()[1]

    def setUp(self):
        self.server_proc = None
        self.bridge = None
        self.pilot_joystick_pub = None
        self.topside_telemetry_sub = None
        self.addCleanup(self._cleanup)

        server_exe = find_server_binary()
        self.sil_port = self._free_tcp_port()
        telemetry_port = self._free_tcp_port()
        joystick_port = self._free_tcp_port()
        self.server_proc = subprocess.Popen(
            [server_exe, "--port", str(self.sil_port), "--cycles", "0"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )

        self.bridge = PiCoreSilBridge(
            sil_host="127.0.0.1",
            sil_port=self.sil_port,
            zmq_telemetry_pub_addr=f"tcp://127.0.0.1:{telemetry_port}",
            zmq_joystick_sub_addr=f"tcp://127.0.0.1:{joystick_port}",
        )
        self.assertTrue(
            self.bridge.connect_sil_server(timeout_sec=3.0),
            "Pi Core SIL Bridge must connect to STM32 SIL server",
        )

        self.received_sensor_packets = []

        def on_topside_telemetry(msg: telemetry_pb2.SensorData):
            self.received_sensor_packets.append(msg)

        self.pilot_joystick_pub = Publisher(
            address=f"tcp://127.0.0.1:{joystick_port}", topic="joystick", bind=False
        )
        self.topside_telemetry_sub = SurfaceSubscriber(
            address=f"tcp://127.0.0.1:{telemetry_port}",
            topic="telemetry",
            message_type=surface_telemetry_pb2.SensorData,
            callback=on_topside_telemetry,
            bind=False,
        )

    def _cleanup(self):
        for resource in (self.topside_telemetry_sub, self.pilot_joystick_pub, self.bridge):
            if resource is not None:
                resource.close()
        if self.server_proc is not None and self.server_proc.poll() is None:
            self.server_proc.terminate()
            try:
                self.server_proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.server_proc.kill()
                self.server_proc.wait(timeout=2.0)

    def _pump_until(self, predicate, timeout_sec=4.0, tick=None):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if tick is not None:
                tick()
            self.bridge.step(timeout_sec=0.005)
            self.topside_telemetry_sub.spin_once(timeout_ms=1)
            if predicate():
                return True
        return predicate()

    def test_end_to_end_telemetry_and_controls(self):
        print("\n--- [SIL TEST 1] Verifying firmware telemetry and mocked output status ---")
        telemetry_ready = self._pump_until(
            lambda: self.bridge.nav_count > 0
            and self.bridge.env_count > 0
            and self.bridge.power_count > 0
            and any(15.0 <= packet.temperature <= 35.0 for packet in self.received_sensor_packets)
            and self.bridge.latest_node_pwms == [1500] * 8
        )
        self.assertTrue(telemetry_ready, "Navigation, environment, power, and output status must arrive")
        latest = self.received_sensor_packets[-1]
        print(f"Topside SensorData: depth={latest.depth:.2f}m, gyro_z={latest.angular_velocity.z:.4f}")
        for value in (
            latest.depth,
            latest.angular_velocity.x,
            latest.angular_velocity.y,
            latest.angular_velocity.z,
            latest.temperature,
        ):
            self.assertTrue(math.isfinite(value), "Decoded telemetry values must be finite")
        self.assertGreaterEqual(latest.depth, 0.0)
        self.assertLess(latest.depth, 100.0, "Depth must remain within the modeled vehicle range")
        self.assertGreaterEqual(self.bridge.latest_temp, 15.0)
        self.assertLessEqual(self.bridge.latest_temp, 35.0)
        self.assertGreaterEqual(latest.temperature, 15.0)
        self.assertLessEqual(latest.temperature, 35.0)
        self.assertIsNotNone(self.bridge.latest_env)
        self.assertGreaterEqual(self.bridge.latest_env.pressure_hpa, 800.0)
        self.assertLessEqual(self.bridge.latest_env.pressure_hpa, 1200.0)
        self.assertGreaterEqual(self.bridge.latest_env.humidity_pct, 0.0)
        self.assertLessEqual(self.bridge.latest_env.humidity_pct, 100.0)
        self.assertIsNotNone(self.bridge.latest_power)
        self.assertGreaterEqual(self.bridge.latest_power.tether_voltage_mv, 40000)
        self.assertLessEqual(self.bridge.latest_power.tether_voltage_mv, 56000)

        print("\n--- [SIL TEST 2] Checking pilot command against actual mocked PWM outputs ---")
        armed = self._pump_until(lambda: self.bridge.sim_time_ms >= 3000, timeout_sec=10.0)
        self.assertTrue(
            armed,
            f"Control Board must complete its 3000 ms ESC arming period (SIL time: {self.bridge.sim_time_ms} ms)",
        )
        cmd = surface_telemetry_pb2.JoystickCommand(
            timestamp_us=int(time.time() * 1e6),
            forward=0.5,
            strafe=0.0,
            vertical=0.0,
            yaw=0.2,
        )
        command_reached_node = self._pump_until(
            lambda: self.bridge.latest_pwms[0] != 1500
            and self.bridge.latest_node_pwms[0] != 1500,
            tick=lambda: self.pilot_joystick_pub.publish(cmd),
        )
        self.assertTrue(command_reached_node, "Joystick command must move the simulated Control Board PWM")
        self.assertTrue(all(1000 <= pwm <= 2000 for pwm in self.bridge.latest_node_pwms))

        print("\n--- [SIL TEST 3] Checking a valid pneumatic command reaches the board ---")
        self.bridge.send_solenoid_cmd(0x0011)
        solenoids_reached_node = self._pump_until(
            lambda: self.bridge.latest_node_solenoid_mask == 0x0011
        )
        self.assertTrue(solenoids_reached_node, "Solenoid command must reach the mocked board outputs")

        print("\n--- [SIL TEST 4] Verifying emergency brake and locked pneumatic outputs ---")
        self.bridge.send_emergency_break()
        brake_reached_node = self._pump_until(
            lambda: self.bridge.node_emergency_brake_active
            and self.bridge.latest_node_pwms == [1500] * 8
        )
        self.assertTrue(self.bridge.emergency_break_sent, "Bridge must send the emergency-break command")
        self.assertTrue(brake_reached_node, "Firmware must trip the brake and set all PWM outputs to neutral")

        joystick_count_before = self.bridge.joystick_count
        self.pilot_joystick_pub.publish(cmd)
        joystick_blocked = self._pump_until(
            lambda: self.bridge.joystick_count > joystick_count_before,
            timeout_sec=2.0,
        )
        self.assertTrue(joystick_blocked, "Bridge must receive pilot input during the latched emergency stop")
        self.assertEqual(self.bridge.latest_pwms, [1500] * 8)
        self.assertEqual(self.bridge.latest_node_pwms, [1500] * 8)
        self.assertEqual(self.bridge.latest_node_solenoid_mask, 0)

        status_count_before = self.bridge.node_status_count
        self.bridge.send_solenoid_cmd(0x0022)
        post_estop_status_received = self._pump_until(
            lambda: self.bridge.node_status_count > status_count_before
        )
        self.assertTrue(post_estop_status_received, "Board output status must continue after E-stop")
        self.assertEqual(self.bridge.latest_node_solenoid_mask, 0, "E-stop must keep pneumatics de-energized")

if __name__ == "__main__":
    unittest.main()
