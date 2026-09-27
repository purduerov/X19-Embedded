"""Transport, CAN decoding, and shared state for the Embedded SIL dashboard."""

import os
import sys
import time
import socket
import select
import struct
import threading
import subprocess
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Dict, Any

# Path configuration strictly within X19-Embedded
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
BRIDGE_DIR = os.path.join(REPO_ROOT, "tests", "sil_companion_bridge")
if BRIDGE_DIR not in sys.path:
    sys.path.insert(0, BRIDGE_DIR)

# Optional read-only import of compiled Protobuf definitions if present
PROTO_DIR = os.path.abspath(os.path.join(REPO_ROOT, "..", "X19-Core", "src", "protocols", "python"))
if os.path.exists(PROTO_DIR) and PROTO_DIR not in sys.path:
    sys.path.insert(0, PROTO_DIR)

HAVE_PROTOBUF = False
try:
    import telemetry_pb2
    HAVE_PROTOBUF = True
except Exception:
    telemetry_pb2 = None

from sil_protocol import (
    CAN_ID_EMERGENCY_BREAK,
    CAN_ID_EFUSE_FAULT_ALERT,
    CAN_ID_THRUSTER_CMD,
    CAN_ID_SOLENOID_CMD,
    CAN_ID_NAV_TELEMETRY,
    CAN_ID_ENV_TELEMETRY,
    CAN_ID_POWER_TELEMETRY,
    CAN_ID_SIL_OUTPUT_STATUS,
    SIL_PACKET_SIZE,
    SIL_MAGIC_HEADER,
    SIL_MAGIC_HEADER_LEGACY,
    ThrusterCommand,
    SolenoidCommand,
    NavTelemetry,
    EnvTelemetry,
    PowerTelemetry,
    pack_sil_can_frame,
    unpack_sil_can_frame,
    unpack_sil_output_status,
)

SERVER_EXE_PATH = os.environ.get("X19_SIL_SERVER") or next(
    (
        candidate
        for candidate in (
            os.path.join(REPO_ROOT, "build-native", "tests", "sil_bridge_server.exe"),
            os.path.join(REPO_ROOT, "build-native", "tests", "sil_bridge_server"),
            os.path.join(REPO_ROOT, "build", "tests", "sil_bridge_server.exe"),
            os.path.join(REPO_ROOT, "build", "tests", "sil_bridge_server"),
        )
        if os.path.isfile(candidate)
    ),
    os.path.join(REPO_ROOT, "build-native", "tests", "sil_bridge_server.exe"),
)

CAN_ID_MAP = {
    CAN_ID_EMERGENCY_BREAK: ("EMERGENCY_BREAK", "Priority 0: Hardware Cutoff"),
    CAN_ID_EFUSE_FAULT_ALERT: ("EFUSE_FAULT_ALERT", "Priority 0: Power Slab Fault"),
    CAN_ID_THRUSTER_CMD: ("THRUSTER_CMD", "Priority 1: 8x ESC PWMs"),
    CAN_ID_SOLENOID_CMD: ("SOLENOID_CMD", "Priority 1: 10-Ch Solenoids"),
    CAN_ID_NAV_TELEMETRY: ("NAV_TELEMETRY", "Priority 2: 100 Hz IMU + Depth"),
    CAN_ID_ENV_TELEMETRY: ("ENV_TELEMETRY", "Priority 2: 10 Hz Leak & Temp"),
    CAN_ID_POWER_TELEMETRY: ("POWER_TELEMETRY", "Priority 3: 20 Hz Power Slab"),
    CAN_ID_SIL_OUTPUT_STATUS: ("SIL_OUTPUT_STATUS", "SIL-only mocked output snapshot"),
}

@dataclass
class CanPacketRecord:
    timestamp: str
    direction: str  # "Core -> STM32" or "STM32 -> Core"
    can_id: int
    name: str
    dlc: int
    hex_data: str
    decoded_summary: str


def stream_age_label(client: "SilDashboardClient", can_id: int) -> str:
    """Return packet age for a compact stream freshness indicator."""
    received_at = client.last_rx_monotonic.get(can_id)
    if received_at is None:
        return "waiting"
    age = max(0.0, time.monotonic() - received_at)
    threshold = {
        CAN_ID_NAV_TELEMETRY: 0.5,
        CAN_ID_ENV_TELEMETRY: 2.5,
        CAN_ID_POWER_TELEMETRY: 2.5,
        CAN_ID_SIL_OUTPUT_STATUS: 0.5,
    }.get(can_id, 2.0)
    return f"{'STALE' if age > threshold else 'LIVE'} · {age:.2f}s"

def compute_thrust_allocation(surge: float, sway: float, heave: float, yaw: float, pitch: float = 0.0, roll: float = 0.0) -> List[int]:
    """
    Standard Purdue ROV X19 8-Thruster Allocation Matrix:
    Maps 6-DOF Pilot commands (-1.0 to 1.0) into 8-Thruster PWMs (1000 to 2000 us).
    Horizontal thrusters (45-degree vectored configuration):
      T0 (Front-Left):  +surge +sway +yaw
      T1 (Front-Right): +surge -sway -yaw
      T2 (Aft-Left):    +surge -sway +yaw
      T3 (Aft-Right):   +surge +sway -yaw
    Vertical thrusters:
      T4 (Front-Left):  -heave +pitch -roll
      T5 (Front-Right): -heave +pitch +roll
      T6 (Aft-Left):    -heave -pitch -roll
      T7 (Aft-Right):   -heave -pitch +roll
    """
    raw_h = [
        surge + sway + yaw,   # T0
        surge - sway - yaw,   # T1
        surge - sway + yaw,   # T2
        surge + sway - yaw,   # T3
    ]
    raw_v = [
        -heave + pitch - roll,  # T4
        -heave + pitch + roll,  # T5
        -heave - pitch - roll,  # T6
        -heave - pitch + roll,  # T7
    ]

    max_h = max([abs(x) for x in raw_h] + [1.0])
    norm_h = [x / max_h for x in raw_h]

    max_v = max([abs(x) for x in raw_v] + [1.0])
    norm_v = [x / max_v for x in raw_v]

    all_norm = norm_h + norm_v
    pwms = [int(1500 + x * 400) for x in all_norm]
    return [max(1100, min(1900, p)) for p in pwms]

class SilDashboardClient:
    def __init__(self, host: str = "127.0.0.1", port: int = 8765):
        self.host = host
        self.port = port
        self.sock: Optional[socket.socket] = None
        self.server_proc: Optional[subprocess.Popen] = None
        self.connected = False
        self.running = False
        self.lock = threading.Lock()

        # Packet history & C console stream
        self.packet_log = deque(maxlen=250)
        self.c_stdout_log = deque(maxlen=150)
        self._last_logged_payload: Dict[int, bytes] = {}
        self._last_logged_time: Dict[int, float] = {}

        # Live vehicle state
        self.pwms = [1500] * 8
        self.actual_pwms = [1500] * 8
        self.solenoid_mask = 0
        self.actual_solenoid_mask = 0
        self.sim_time_ms = 0
        self.output_status_received_monotonic: Optional[float] = None
        self.last_rx_monotonic: Dict[int, float] = {}
        self.nav_data: Optional[NavTelemetry] = None
        self.env_data: Optional[EnvTelemetry] = None
        self.power_data: Optional[PowerTelemetry] = None
        self.emergency_break_tripped = False
        self.emergency_break_requested = False
        self.frame_count = 0
        self.history_depth: List[float] = []
        self.history_time: List[float] = []

        # End-to-End Pipeline state
        self.pipeline_surface_cmd: Dict[str, Any] = {
            "surge": 0.0, "sway": 0.0, "heave": 0.0,
            "yaw": 0.0, "pitch": 0.0, "roll": 0.0,
            "timestamp_us": int(time.time() * 1e6),
            "raw_hex": "08 00 15 00 00 00 00",
        }
        self.pipeline_core_pwms: List[int] = [1500] * 8
        self.pipeline_core_can_hex: str = "DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05"
        self.pipeline_core_to_surface: Dict[str, Any] = {
            "timestamp_us": int(time.time() * 1e6),
            "depth": 0.0,
            "temp": 22.5,
            "gyro_x": 0.0,
            "gyro_y": 0.0,
            "gyro_z": 0.0,
            "accel_x": 0.0,
            "accel_y": 0.0,
            "accel_z": 9.81,
            "raw_hex": "08 00 1D 00 00 00 00 25 00 00 B4 41",
        }

    def start_server_process(self) -> bool:
        if not os.path.exists(SERVER_EXE_PATH):
            return False
        if self.server_proc is None or self.server_proc.poll() is not None:
            self.server_proc = subprocess.Popen(
                [SERVER_EXE_PATH, "--port", str(self.port)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            stdout_reader = threading.Thread(target=self._read_c_stdout, daemon=True)
            stdout_reader.start()
            time.sleep(0.4)
            return True
        return True

    def _read_c_stdout(self):
        if not self.server_proc or not self.server_proc.stdout:
            return
        for line in iter(self.server_proc.stdout.readline, ''):
            if not line:
                break
            stripped = line.strip()
            if stripped:
                with self.lock:
                    self.c_stdout_log.append(f"[{time.strftime('%H:%M:%S')}] {stripped}")

    def stop_server_process(self):
        self.disconnect()
        if self.server_proc and self.server_proc.poll() is None:
            self.server_proc.terminate()
            try:
                self.server_proc.wait(timeout=1.5)
            except subprocess.TimeoutExpired:
                self.server_proc.kill()
            self.server_proc = None

    def connect(self) -> bool:
        with self.lock:
            if self.connected:
                return True
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect((self.host, self.port))
                s.setblocking(False)
                self.sock = s
                self.connected = True
                self.running = True
                self.worker_thread = threading.Thread(target=self._rx_loop, daemon=True)
                self.worker_thread.start()
                return True
            except (ConnectionRefusedError, OSError):
                self.connected = False
                return False

    def disconnect(self):
        self.running = False
        with self.lock:
            if self.sock:
                try:
                    self.sock.close()
                except OSError:
                    pass
                self.sock = None
            self.connected = False

    def send_surface_pilot_command(self, surge: float, sway: float, heave: float, yaw: float, pitch: float = 0.0, roll: float = 0.0):
        """
        Executes the full Topside Surface -> Core -> STM32 pipeline:
        1. Packs Surface Protobuf JoystickCommand.
        2. Computes Core 8-Thruster Allocation Matrix.
        3. Encodes CAN ID 0x100 and sends to STM32 over SIL bus.
        """
        timestamp_us = int(time.time() * 1e6)
        raw_pb_bytes = b""
        raw_pb_hex = ""

        if HAVE_PROTOBUF and telemetry_pb2:
            try:
                cmd_proto = telemetry_pb2.JoystickCommand(
                    timestamp_us=timestamp_us,
                    forward=float(surge),
                    strafe=float(sway),
                    vertical=float(heave),
                    pitch=float(pitch),
                    roll=float(roll),
                    yaw=float(yaw),
                )
                raw_pb_bytes = cmd_proto.SerializeToString()
                raw_pb_hex = " ".join(f"{b:02X}" for b in raw_pb_bytes)
            except Exception:
                raw_pb_hex = f"Surge={surge:+.2f} Sway={sway:+.2f} Heave={heave:+.2f} Yaw={yaw:+.2f}"
        else:
            raw_pb_hex = f"Surge={surge:+.2f} Sway={sway:+.2f} Heave={heave:+.2f} Yaw={yaw:+.2f}"

        pwms = compute_thrust_allocation(surge, sway, heave, yaw, pitch, roll)
        cmd = ThrusterCommand(pwm_us=pwms)
        can_payload = cmd.pack()
        can_hex = " ".join(f"{b:02X}" for b in can_payload)

        with self.lock:
            self.pipeline_surface_cmd = {
                "surge": surge, "sway": sway, "heave": heave,
                "yaw": yaw, "pitch": pitch, "roll": roll,
                "timestamp_us": timestamp_us,
                "raw_hex": raw_pb_hex or "08 00 15 00 00 00 00",
                "raw_bytes": raw_pb_bytes,
            }
            self.pipeline_core_pwms = list(pwms)
            self.pipeline_core_can_hex = can_hex
            self.pwms = list(pwms)

        self.send_pwms(pwms)

    def send_pwms(self, pwms: List[int]):
        with self.lock:
            self.pwms = list(pwms)
            cmd = ThrusterCommand(pwm_us=self.pwms)
            payload = cmd.pack()
            self.pipeline_core_can_hex = " ".join(f"{b:02X}" for b in payload)

            if not self.connected or not self.sock:
                return

            frame = pack_sil_can_frame(CAN_ID_THRUSTER_CMD, payload)
            try:
                self.sock.sendall(frame)
                self._record_packet("Core -> STM32", CAN_ID_THRUSTER_CMD, payload, f"PWMs: {self.pwms}")
            except OSError:
                self.connected = False

    def send_solenoids(self, mask: int):
        with self.lock:
            self.solenoid_mask = mask & 0x03FF
            if not self.connected or not self.sock:
                return
            cmd = SolenoidCommand(solenoid_mask=self.solenoid_mask)
            payload = cmd.pack()
            frame = pack_sil_can_frame(CAN_ID_SOLENOID_CMD, payload)
            try:
                self.sock.sendall(frame)
                self._record_packet("Core -> STM32", CAN_ID_SOLENOID_CMD, payload, f"Bitmask: 0x{self.solenoid_mask:04X}")
            except OSError:
                self.connected = False

    def trigger_emergency_break(self):
        with self.lock:
            # Magic signature (0xAA, 0x55) required by Node 2 authorization check (PR #46)
            payload = b"\xAA\x55\x01"
            if not self.connected or not self.sock:
                return
            frame = pack_sil_can_frame(CAN_ID_EMERGENCY_BREAK, payload)
            try:
                self.sock.sendall(frame)
                self.emergency_break_requested = True
                self._record_packet("Core -> STM32", CAN_ID_EMERGENCY_BREAK, payload, "EMERGENCY CUTOFF TRIGGERED (Authorized)")
            except OSError:
                self.connected = False

    def _record_packet(self, direction: str, can_id: int, payload: bytes, summary: str):
        now = time.monotonic()
        telemetry_ids = {CAN_ID_NAV_TELEMETRY, CAN_ID_ENV_TELEMETRY, CAN_ID_POWER_TELEMETRY}
        if can_id in telemetry_ids:
            last_payload = self._last_logged_payload.get(can_id)
            last_time = self._last_logged_time.get(can_id, 0.0)
            if last_payload == payload and (now - last_time) < 2.0:
                return
            self._last_logged_payload[can_id] = payload
            self._last_logged_time[can_id] = now

        name = CAN_ID_MAP.get(can_id, ("UNKNOWN", ""))[0]
        rec = CanPacketRecord(
            timestamp=time.strftime("%H:%M:%S") + f".{int(time.time()*1000)%1000:03d}",
            direction=direction,
            can_id=can_id,
            name=name,
            dlc=len(payload),
            hex_data=" ".join(f"{b:02X}" for b in payload),
            decoded_summary=summary,
        )
        self.packet_log.append(rec)

    def _rx_loop(self):
        buf = bytearray()
        while self.running and self.connected and self.sock:
            try:
                rlist, _, _ = select.select([self.sock], [], [], 0.05)
                if not rlist:
                    continue
                chunk = self.sock.recv(SIL_PACKET_SIZE * 4)
                if not chunk:
                    break
                buf.extend(chunk)

                while len(buf) >= SIL_PACKET_SIZE:
                    magic = int.from_bytes(buf[:4], byteorder="little")
                    if magic not in (SIL_MAGIC_HEADER, SIL_MAGIC_HEADER_LEGACY):
                        del buf[:1]
                        continue

                    packet_chunk = bytes(buf[:SIL_PACKET_SIZE])
                    del buf[:SIL_PACKET_SIZE]
                    try:
                        can_id, payload = unpack_sil_can_frame(packet_chunk)
                    except ValueError:
                        continue

                    self.frame_count += 1
                    self.last_rx_monotonic[can_id] = time.monotonic()
                    summary = ""

                    if can_id == CAN_ID_SIL_OUTPUT_STATUS:
                        try:
                            actual_pwms, brake_active, actual_solenoids, sim_time_ms = unpack_sil_output_status(payload)
                        except ValueError:
                            continue
                        self.actual_pwms = actual_pwms
                        self.emergency_break_tripped = brake_active
                        self.emergency_break_requested = self.emergency_break_requested and not brake_active
                        self.actual_solenoid_mask = actual_solenoids
                        self.sim_time_ms = sim_time_ms
                        self.output_status_received_monotonic = time.monotonic()
                        summary = (
                            f"Actual PWM: {actual_pwms}; brake={brake_active}; "
                            f"solenoids=0x{actual_solenoids:03X}; sim={sim_time_ms} ms"
                        )

                    elif can_id == CAN_ID_NAV_TELEMETRY:
                        try:
                            nav = NavTelemetry.unpack(payload)
                        except ValueError:
                            continue
                        self.nav_data = nav
                        summary = f"Depth: {nav.depth_meters:.2f}m, YawRate: {nav.gyro_z_rad_s:.3f}rad/s, Q:({nav.q_w:.2f},{nav.q_x:.2f},{nav.q_y:.2f},{nav.q_z:.2f})"
                        self.history_depth.append(nav.depth_meters)
                        self.history_time.append(time.time())
                        if len(self.history_depth) > 100:
                            self.history_depth.pop(0)
                            self.history_time.pop(0)

                        # Update pipeline Core -> Surface translation
                        raw_bytes = b""
                        raw_hex = ""
                        depth_val = nav.depth_meters
                        temp_val = self.env_data.temperature_c if self.env_data else 22.5
                        gx, gy, gz = nav.gyro_x_rad_s, nav.gyro_y_rad_s, nav.gyro_z_rad_s
                        ts_us = int(time.time() * 1e6)

                        if HAVE_PROTOBUF and telemetry_pb2:
                            try:
                                sdata = telemetry_pb2.SensorData(
                                    timestamp_us=ts_us,
                                    depth=depth_val,
                                    temperature=temp_val,
                                    angular_velocity=telemetry_pb2.Vector3D(x=gx, y=gy, z=gz),
                                    acceleration=telemetry_pb2.Vector3D(x=0.0, y=0.0, z=9.81),
                                )
                                raw_bytes = sdata.SerializeToString()
                                raw_hex = " ".join(f"{b:02X}" for b in raw_bytes)
                            except Exception:
                                pass

                        with self.lock:
                            self.pipeline_core_to_surface = {
                                "timestamp_us": ts_us,
                                "depth": depth_val,
                                "temp": temp_val,
                                "gyro_x": gx,
                                "gyro_y": gy,
                                "gyro_z": gz,
                                "accel_x": 0.0,
                                "accel_y": 0.0,
                                "accel_z": 9.81,
                                "raw_hex": raw_hex or "08 00 1D 00 00 00 00 25 00 00 B4 41",
                                "raw_bytes": raw_bytes,
                            }

                    elif can_id == CAN_ID_ENV_TELEMETRY:
                        try:
                            env = EnvTelemetry.unpack(payload)
                        except ValueError:
                            continue
                        self.env_data = env
                        summary = f"Pres: {env.pressure_hpa:.1f}hPa, Hum: {env.humidity_pct:.1f}%, Leak: 0x{env.leak_flags:02X}"
                    elif can_id == CAN_ID_POWER_TELEMETRY:
                        try:
                            pwr = PowerTelemetry.unpack(payload)
                        except ValueError:
                            continue
                        self.power_data = pwr
                        summary = f"48V Rail: {pwr.tether_voltage_mv/1000:.1f}V @ {pwr.tether_current_ma/1000:.1f}A, Temp: {pwr.pcb_temp_c_tenths/10:.1f}C"

                    elif can_id == CAN_ID_EMERGENCY_BREAK:
                        summary = "EMERGENCY BREAK LATCHED"

                    self._record_packet("STM32 -> Core", can_id, payload, summary)

            except Exception:
                break
        self.connected = False
