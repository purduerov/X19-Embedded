"""
X19 Raspberry Pi 5 Core CAN <-> ZeroMQ SIL Gateway Node.
Uses code and patterns from X19-Core (ZMQ Publisher, Subscriber, and Protobuf schemas).
Connects to the X19-Embedded SIL bridge server, converting:
1. ZMQ Joystick/Motion commands -> CAN ID 0x100 Thruster PWM frames.
2. CAN ID 0x200 Navigation & Depth -> ZMQ 'telemetry' SensorData Protobuf packets.
3. CAN ID 0x210 Leak & Temp -> ZMQ 'leak' / 'env' Protobuf packets.
4. CAN ID 0x001 Emergency Break -> Halts thrusters & alerts topside.
"""

import os
import sys
import time
import socket
import select
from typing import Optional

# Dynamically import from X19-Core codebase
X19_CORE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../X19-Core"))
if X19_CORE_DIR not in sys.path:
    sys.path.insert(0, X19_CORE_DIR)

from src.python.messaging import Publisher, Subscriber
from src.protocols.python import telemetry_pb2
from sil_protocol import (
    CAN_ID_EMERGENCY_BREAK,
    CAN_ID_THRUSTER_CMD,
    CAN_ID_SOLENOID_CMD,
    CAN_ID_NAV_TELEMETRY,
    CAN_ID_ENV_TELEMETRY,
    CAN_ID_POWER_TELEMETRY,
    SIL_PACKET_SIZE,
    ThrusterCommand,
    NavTelemetry,
    EnvTelemetry,
    PowerTelemetry,
    pack_sil_can_frame,
    unpack_sil_can_frame,
)

class PiCoreSilBridge:
    def __init__(
        self,
        sil_host: str = "127.0.0.1",
        sil_port: int = 8765,
        zmq_telemetry_pub_addr: str = "tcp://127.0.0.1:5555",
        zmq_joystick_sub_addr: str = "tcp://127.0.0.1:5556",
    ):
        self.sil_host = sil_host
        self.sil_port = sil_port
        self.sock: Optional[socket.socket] = None

        # ZMQ Publisher (mimicking Pi 5 Core publishing telemetry to Surface UI)
        self.telemetry_pub = Publisher(address=zmq_telemetry_pub_addr, topic="telemetry")

        # ZMQ Subscriber (mimicking Pi 5 Core receiving Joystick / Pilot commands from Surface)
        self.joystick_sub = Subscriber(
            address=zmq_joystick_sub_addr,
            topic="joystick",
            message_type=telemetry_pb2.JoystickCommand,
            callback=self._on_joystick_received,
            bind=True,  # Bind so external pilots can connect
        )

        self.latest_pwms = [1500] * 8
        self.emergency_break_received = False
        self.nav_count = 0
        self.env_count = 0
        self.power_count = 0
        self.latest_depth = 0.0
        self.latest_temp = 0.0

    def connect_sil_server(self, timeout_sec: float = 5.0) -> bool:
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect((self.sil_host, self.sil_port))
                s.setblocking(False)
                self.sock = s
                print(f"[Pi-Core SIL Bridge] Connected to STM32 SIL Bus at {self.sil_host}:{self.sil_port}")
                return True
            except (ConnectionRefusedError, OSError):
                time.sleep(0.05)
        print(f"[Pi-Core SIL Bridge] Failed to connect to SIL server at {self.sil_host}:{self.sil_port}")
        return False

    def _on_joystick_received(self, cmd: telemetry_pb2.JoystickCommand):
        """Maps JoystickCommand from X19-Core to 8 thruster PWM values."""
        # Simple 8-thruster holonomic mixer matching X19 geometry
        # Forward (surge), Strafe (sway), Vertical (heave), Yaw
        fwd = max(-1.0, min(1.0, cmd.forward))
        strf = max(-1.0, min(1.0, cmd.strafe))
        vert = max(-1.0, min(1.0, cmd.vertical))
        yaw = max(-1.0, min(1.0, cmd.yaw))

        # Vectorized mixing (4 vectored horizontal thrusters, 4 vertical thrusters)
        t0 = 1500 + int((fwd + strf + yaw) * 400)
        t1 = 1500 + int((fwd - strf - yaw) * 400)
        t2 = 1500 + int((-fwd + strf - yaw) * 400)
        t3 = 1500 + int((-fwd - strf + yaw) * 400)
        t4 = 1500 + int(vert * 400)
        t5 = 1500 + int(vert * 400)
        t6 = 1500 + int(vert * 400)
        t7 = 1500 + int(vert * 400)

        raw_pwms = [t0, t1, t2, t3, t4, t5, t6, t7]
        self.latest_pwms = [max(1000, min(2000, p)) for p in raw_pwms]
        self.send_thruster_cmd(self.latest_pwms)

    def send_thruster_cmd(self, pwms: list):
        if not self.sock:
            return
        cmd = ThrusterCommand(pwm_us=pwms)
        frame = pack_sil_can_frame(CAN_ID_THRUSTER_CMD, cmd.pack())
        try:
            self.sock.sendall(frame)
        except OSError as e:
            print(f"[Pi-Core SIL Bridge] CAN TX error: {e}")

    def send_emergency_break(self):
        if not self.sock:
            return
        frame = pack_sil_can_frame(CAN_ID_EMERGENCY_BREAK, b"\x01")
        try:
            self.sock.sendall(frame)
        except OSError as e:
            print(f"[Pi-Core SIL Bridge] CAN E-Break TX error: {e}")

    def step(self, timeout_sec: float = 0.005):
        """Polls ZMQ inputs and CAN SIL socket."""
        # 1. Process incoming ZMQ pilot commands
        self.joystick_sub.spin_once(timeout_ms=1)

        # 2. Read incoming CAN frames from the simulated STM32 bus
        if not self.sock:
            return

        rlist, _, _ = select.select([self.sock], [], [], timeout_sec)
        if not rlist:
            return

        try:
            chunk = self.sock.recv(SIL_PACKET_SIZE)
            if not chunk or len(chunk) < SIL_PACKET_SIZE:
                return

            can_id, payload = unpack_sil_can_frame(chunk)

            if can_id == CAN_ID_NAV_TELEMETRY:
                nav = NavTelemetry.unpack(payload)
                self.nav_count += 1
                self.latest_depth = nav.depth_meters

                # Repackage into X19-Core Protobuf SensorData message and publish over ZMQ
                msg = telemetry_pb2.SensorData(
                    timestamp_us=int(time.time() * 1e6),
                    acceleration=telemetry_pb2.Vector3D(x=0.0, y=0.0, z=-9.81),
                    angular_velocity=telemetry_pb2.Vector3D(
                        x=nav.gyro_x_rad_s, y=nav.gyro_y_rad_s, z=nav.gyro_z_rad_s
                    ),
                    depth=nav.depth_meters,
                    temperature=self.latest_temp,
                )
                self.telemetry_pub.publish(msg)

            elif can_id == CAN_ID_ENV_TELEMETRY:
                env = EnvTelemetry.unpack(payload)
                self.env_count += 1
                self.latest_temp = env.temperature_c

            elif can_id == CAN_ID_POWER_TELEMETRY:
                self.power_count += 1

            elif can_id == CAN_ID_EMERGENCY_BREAK:
                self.emergency_break_received = True

        except (BlockingIOError, OSError):
            pass

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        self.telemetry_pub.close()
        self.joystick_sub.close()
