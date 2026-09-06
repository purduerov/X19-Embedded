"""
X19 Software-in-the-Loop (SIL) Interactive Testing Dashboard.
Powered by Streamlit.

Enables live hardware-free testing and end-to-end verification:
- Topside Surface Pilot Station: 6-DOF controls (Surge, Sway, Heave, Yaw) mapped to ZeroMQ Protobuf commands.
- End-to-End Message Pipeline Tracer:
    Surface (Protobuf JoystickCommand) -> Core (Thrust Allocation Matrix & CAN Packing)
    -> STM32 CAN Bus (0x100 / 0x110) -> STM32 C Firmware Execution (app.c, TIM6 ramp, BDTR latch)
    -> STM32 Telemetry (0x200, 0x210, 0x300) -> Core Topside Translation (Protobuf SensorData).
- Raw CAN Bus Monitor & Packet Inspector (Tab 6): Live frame stream, hex inspector, and filtering.
- Live C Firmware Console & Code Verification (Tab 7): Real-time stdout from sil_bridge_server.exe.
- Direct Thruster & Solenoid Control (Tab 2).
- Real-time live gauges: 100 Hz Navigation (Tab 3), Environmental & Leak (Tab 4), Power Slab (Tab 5).
"""

import os
import sys
import time
import socket
import select
import threading
import subprocess
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Dict, Any

import streamlit as st

# Path configuration
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
BRIDGE_DIR = os.path.join(REPO_ROOT, "tests", "sil_companion_bridge")
if BRIDGE_DIR not in sys.path:
    sys.path.insert(0, BRIDGE_DIR)

# Attempt to import compiled Protobuf definitions from X19-Core
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
    SIL_PACKET_SIZE,
    ThrusterCommand,
    SolenoidCommand,
    NavTelemetry,
    EnvTelemetry,
    PowerTelemetry,
    pack_sil_can_frame,
    unpack_sil_can_frame,
)

SERVER_EXE_PATH = os.path.join(REPO_ROOT, "build", "tests", "sil_bridge_server.exe")

CAN_ID_MAP = {
    CAN_ID_EMERGENCY_BREAK: ("EMERGENCY_BREAK", "Priority 0: Hardware Cutoff"),
    CAN_ID_EFUSE_FAULT_ALERT: ("EFUSE_FAULT_ALERT", "Priority 0: Power Slab Fault"),
    CAN_ID_THRUSTER_CMD: ("THRUSTER_CMD", "Priority 1: 8x ESC PWMs"),
    CAN_ID_SOLENOID_CMD: ("SOLENOID_CMD", "Priority 1: 10-Ch Solenoids"),
    CAN_ID_NAV_TELEMETRY: ("NAV_TELEMETRY", "Priority 2: 100 Hz IMU + Depth"),
    CAN_ID_ENV_TELEMETRY: ("ENV_TELEMETRY", "Priority 2: 10 Hz Leak & Temp"),
    CAN_ID_POWER_TELEMETRY: ("POWER_TELEMETRY", "Priority 3: 20 Hz Power Slab"),
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

def compute_thrust_allocation(surge: float, sway: float, heave: float, yaw: float, pitch: float = 0.0, roll: float = 0.0) -> List[int]:
    """
    Standard X19 8-Thruster Allocation Matrix:
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

        # Live vehicle state
        self.pwms = [1500] * 8
        self.solenoid_mask = 0
        self.nav_data: Optional[NavTelemetry] = None
        self.env_data: Optional[EnvTelemetry] = None
        self.power_data: Optional[PowerTelemetry] = None
        self.emergency_break_tripped = False
        self.frame_count = 0
        self.history_depth: List[float] = []
        self.history_time: List[float] = []

        # End-to-End Pipeline state
        self.pipeline_surface_cmd: Dict[str, Any] = {
            "surge": 0.0, "sway": 0.0, "heave": 0.0,
            "yaw": 0.0, "pitch": 0.0, "roll": 0.0,
            "timestamp_us": 0, "raw_hex": ""
        }
        self.pipeline_core_pwms: List[int] = [1500] * 8
        self.pipeline_core_can_hex: str = ""
        self.pipeline_core_to_surface: Dict[str, Any] = {
            "depth": 0.0, "temp": 0.0, "raw_hex": ""
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
                raw_pb_hex = raw_pb_bytes.hex()
            except Exception:
                raw_pb_hex = f"surge={surge:.2f}, sway={sway:.2f}, heave={heave:.2f}, yaw={yaw:.2f}"
        else:
            raw_pb_hex = f"Surge={surge:+.2f} Sway={sway:+.2f} Heave={heave:+.2f} Yaw={yaw:+.2f}"

        pwms = compute_thrust_allocation(surge, sway, heave, yaw, pitch, roll)

        with self.lock:
            self.pipeline_surface_cmd = {
                "surge": surge, "sway": sway, "heave": heave,
                "yaw": yaw, "pitch": pitch, "roll": roll,
                "timestamp_us": timestamp_us, "raw_hex": raw_pb_hex
            }
            self.pipeline_core_pwms = pwms

        self.send_pwms(pwms)

    def send_pwms(self, pwms: List[int]):
        with self.lock:
            if not self.connected or not self.sock:
                return
            self.pwms = list(pwms)
            cmd = ThrusterCommand(pwm_us=self.pwms)
            payload = cmd.pack()
            frame = pack_sil_can_frame(CAN_ID_THRUSTER_CMD, payload)
            try:
                self.sock.sendall(frame)
                self.pipeline_core_can_hex = payload.hex()
                self._record_packet("Core -> STM32", CAN_ID_THRUSTER_CMD, payload, f"PWMs: {self.pwms}")
            except OSError:
                self.connected = False

    def send_solenoids(self, mask: int):
        with self.lock:
            if not self.connected or not self.sock:
                return
            self.solenoid_mask = mask & 0x03FF
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
            if not self.connected or not self.sock:
                return
            payload = b"\x01"
            frame = pack_sil_can_frame(CAN_ID_EMERGENCY_BREAK, payload)
            try:
                self.sock.sendall(frame)
                self.emergency_break_tripped = True
                self._record_packet("Core -> STM32", CAN_ID_EMERGENCY_BREAK, payload, "EMERGENCY CUTOFF TRIGGERED")
            except OSError:
                self.connected = False

    def _record_packet(self, direction: str, can_id: int, payload: bytes, summary: str):
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
                    packet_chunk = bytes(buf[:SIL_PACKET_SIZE])
                    del buf[:SIL_PACKET_SIZE]
                    can_id, payload = unpack_sil_can_frame(packet_chunk)

                    self.frame_count += 1
                    summary = ""

                    if can_id == CAN_ID_NAV_TELEMETRY:
                        nav = NavTelemetry.unpack(payload)
                        self.nav_data = nav
                        summary = f"Depth: {nav.depth_meters:.2f}m, YawRate: {nav.gyro_z_rad_s:.3f}rad/s, Q:({nav.q_w:.2f},{nav.q_x:.2f},{nav.q_y:.2f},{nav.q_z:.2f})"
                        self.history_depth.append(nav.depth_meters)
                        self.history_time.append(time.time())
                        if len(self.history_depth) > 100:
                            self.history_depth.pop(0)
                            self.history_time.pop(0)

                        # Update pipeline Core -> Surface translation
                        if HAVE_PROTOBUF and telemetry_pb2:
                            try:
                                sdata = telemetry_pb2.SensorData(
                                    timestamp_us=int(time.time() * 1e6),
                                    depth=nav.depth_meters,
                                    temperature=self.env_data.temperature_c if self.env_data else 22.0,
                                    angular_velocity=telemetry_pb2.Vector3D(x=nav.gyro_x_rad_s, y=nav.gyro_y_rad_s, z=nav.gyro_z_rad_s)
                                )
                                self.pipeline_core_to_surface = {
                                    "depth": nav.depth_meters,
                                    "temp": self.env_data.temperature_c if self.env_data else 22.0,
                                    "raw_hex": sdata.SerializeToString().hex(),
                                }
                            except Exception:
                                pass

                    elif can_id == CAN_ID_ENV_TELEMETRY:
                        env = EnvTelemetry.unpack(payload)
                        self.env_data = env
                        summary = f"Pres: {env.pressure_hpa:.1f}hPa, Hum: {env.humidity_pct:.1f}%, Leak: 0x{env.leak_flags:02X}"
                        if env.leak_flags != 0:
                            self.emergency_break_tripped = True

                    elif can_id == CAN_ID_POWER_TELEMETRY:
                        pwr = PowerTelemetry.unpack(payload)
                        self.power_data = pwr
                        summary = f"48V Rail: {pwr.tether_voltage_mv/1000:.1f}V @ {pwr.tether_current_ma/1000:.1f}A, Temp: {pwr.pcb_temp_c_tenths/10:.1f}C"

                    elif can_id == CAN_ID_EMERGENCY_BREAK:
                        self.emergency_break_tripped = True
                        summary = "EMERGENCY BREAK LATCHED"

                    self._record_packet("STM32 -> Core", can_id, payload, summary)

            except Exception:
                break
        self.connected = False

# Global client cache
@st.cache_resource
def get_sil_client() -> SilDashboardClient:
    return SilDashboardClient()

def main():
    st.set_page_config(
        page_title="X19 ROV - Embedded SIL Testing Station",
        layout="wide",
        initial_sidebar_state="expanded",
    )

    client = get_sil_client()

    # --- SIDEBAR: Master Controls & System State ---
    with st.sidebar:
        st.title("X19 SIL Master Hub")
        st.markdown("**Subsea Node Firmware Simulation**")

        st.subheader("SIL Server State")
        if not client.connected:
            client.start_server_process()
            client.connect()

        col_srv1, col_srv2 = st.columns(2)
        with col_srv1:
            if st.button("Restart Engine", use_container_width=True):
                client.stop_server_process()
                time.sleep(0.3)
                client.start_server_process()
                client.connect()
                st.rerun()
        with col_srv2:
            if st.button("Stop Engine", use_container_width=True):
                client.stop_server_process()
                st.rerun()

        status_color = "green" if client.connected else "red"
        st.markdown(f"**Connection Status:** :{status_color}[{'ONLINE (127.0.0.1:8765)' if client.connected else 'OFFLINE'}]")
        st.metric("Processed CAN Packets", client.frame_count)
        st.metric("Logged CAN Frames", len(client.packet_log))

        st.divider()
        st.subheader("Fault & Safety Injection")
        if st.button("TRIP EMERGENCY BREAK (0x001)", type="primary", use_container_width=True):
            client.trigger_emergency_break()
            st.error("Priority 0 Emergency Break Triggered! Thrusters cut to 1500 us neutral.")

        if client.emergency_break_tripped:
            st.error("VEHICLE STATE: EMERGENCY LATCHED (TIMx_BDTR Active)")
            if st.button("Reset E-Break State", use_container_width=True):
                client.emergency_break_tripped = False
                st.rerun()

        st.divider()
        st.markdown("### Simulated Nodes")
        st.markdown("- **Node 1**: Pi Shield (STM32C5)")
        st.markdown("- **Node 2**: Control Board (STM32C5 + FPU)")
        st.markdown("- **Node 3**: Power Slab (STM32C5)")
        st.markdown("- **Node 4**: USB Camera Hub (PCIe)")

    # --- MAIN DASHBOARD INTERFACE ---
    st.title("Purdue ROV — X19 Embedded SIL Testing Station")
    st.caption("Live hardware-free simulation for propulsion, actuation, active control, and complete communication tracing.")

    tab1, tab2, tab3, tab4, tab5, tab6, tab7 = st.tabs([
        "Surface Pilot & Pipeline Tracer",
        "Direct Thrusters & Solenoids",
        "Navigation & Attitude (100 Hz)",
        "Environmental & Leak (10 Hz)",
        "Power Slab Telemetry (20 Hz)",
        "Raw CAN Bus Monitor & Packet Inspector",
        "Live C Firmware Console & Code Verification"
    ])

    # =========================================================================
    # TAB 1: SURFACE PILOT STATION & END-TO-END PIPELINE TRACER
    # =========================================================================
    with tab1:
        st.subheader("Topside Pilot Station & End-to-End Communication Tracer")
        st.markdown(
            "Control the ROV from the surface and trace every transformation: "
            "**Surface (ZeroMQ Protobuf)** -> **Core (Thrust Matrix & CAN ID 0x100)** -> "
            "**STM32 C Code (`app.c` TIM6 ramp)** -> **CAN Telemetry (0x200)** -> **Core to Surface (Protobuf SensorData)**."
        )

        st.markdown("### 1. Pilot Control Flight Deck")
        col_ctrl1, col_ctrl2 = st.columns([1, 1])

        with col_ctrl1:
            st.markdown("**6-DOF Flight Axes**")
            surge = st.slider("Surge (Forward / Reverse)", -1.0, 1.0, float(client.pipeline_surface_cmd["surge"]), 0.05)
            sway = st.slider("Sway (Strafe Left / Right)", -1.0, 1.0, float(client.pipeline_surface_cmd["sway"]), 0.05)
            heave = st.slider("Heave (Dive / Ascend)", -1.0, 1.0, float(client.pipeline_surface_cmd["heave"]), 0.05)
            yaw = st.slider("Yaw (Turn Left / Right)", -1.0, 1.0, float(client.pipeline_surface_cmd["yaw"]), 0.05)

        with col_ctrl2:
            st.markdown("**Flight Presets**")
            preset_cols = st.columns(3)
            with preset_cols[0]:
                if st.button("All Stop (Hover)", use_container_width=True):
                    client.send_surface_pilot_command(0.0, 0.0, 0.0, 0.0)
                    st.rerun()
            with preset_cols[1]:
                if st.button("Forward (+0.5 Surge)", use_container_width=True):
                    client.send_surface_pilot_command(0.5, 0.0, 0.0, 0.0)
                    st.rerun()
            with preset_cols[2]:
                if st.button("Reverse (-0.5 Surge)", use_container_width=True):
                    client.send_surface_pilot_command(-0.5, 0.0, 0.0, 0.0)
                    st.rerun()

            preset_cols2 = st.columns(3)
            with preset_cols2[0]:
                if st.button("Strafe Right (+0.5 Sway)", use_container_width=True):
                    client.send_surface_pilot_command(0.0, 0.5, 0.0, 0.0)
                    st.rerun()
            with preset_cols2[1]:
                if st.button("Dive (+0.5 Heave)", use_container_width=True):
                    client.send_surface_pilot_command(0.0, 0.0, 0.5, 0.0)
                    st.rerun()
            with preset_cols2[2]:
                if st.button("Yaw Right (+0.5 Yaw)", use_container_width=True):
                    client.send_surface_pilot_command(0.0, 0.0, 0.0, 0.5)
                    st.rerun()

            if (surge != client.pipeline_surface_cmd["surge"] or
                sway != client.pipeline_surface_cmd["sway"] or
                heave != client.pipeline_surface_cmd["heave"] or
                yaw != client.pipeline_surface_cmd["yaw"]):
                client.send_surface_pilot_command(surge, sway, heave, yaw)

        st.divider()
        st.markdown("### 2. Live End-to-End Message Flow Inspector")

        p_col1, p_col2, p_col3 = st.columns(3)

        with p_col1:
            st.markdown("#### [Stage 1] Surface -> Core")
            st.caption("Topside ZeroMQ PUB (`tcp://127.0.0.1:5555`) topic `joystick`")
            st.info(
                f"**Protobuf Message: `JoystickCommand`**\n"
                f"- `forward (surge)`: {client.pipeline_surface_cmd['surge']:+.2f}\n"
                f"- `strafe (sway)`: {client.pipeline_surface_cmd['sway']:+.2f}\n"
                f"- `vertical (heave)`: {client.pipeline_surface_cmd['heave']:+.2f}\n"
                f"- `yaw`: {client.pipeline_surface_cmd['yaw']:+.2f}\n"
                f"- `timestamp_us`: {client.pipeline_surface_cmd['timestamp_us']}"
            )
            st.code(f"Serialized Hex:\n{client.pipeline_surface_cmd['raw_hex'] or '(none)'}", language="text")

        with p_col2:
            st.markdown("#### [Stage 2] Core -> STM32")
            st.caption("CAN FD Arbitration ID `0x100` (`THRUSTER_CMD`)")
            st.info(
                f"**Thrust Allocation Matrix Output:**\n"
                f"- T0..T3 (Horiz): {client.pipeline_core_pwms[0:4]} us\n"
                f"- T4..T7 (Vert):  {client.pipeline_core_pwms[4:8]} us\n"
                f"- DLC: 16 Bytes (8x uint16_t LE)"
            )
            st.code(f"CAN Frame 0x100 Hex:\n{client.pipeline_core_can_hex or '(waiting)'}", language="text")

        with p_col3:
            st.markdown("#### [Stage 3] STM32 C Firmware")
            st.caption("Control Board Node 2 (`Src/app.c`) Native Execution")
            e_status = "LATCHED (E-Break Tripped)" if client.emergency_break_tripped else "ARMED & RUNNING"
            st.success(
                f"**C Firmware Engine Status:**\n"
                f"- Safety State: `{e_status}`\n"
                f"- Slew Limiter: 1 kHz `TIM6` ramping (1000 us/s max)\n"
                f"- Target PWMs: {client.pwms}\n"
                f"- Sensor Sampling: LSM6DSOXTR IMU (100 Hz), MS5837 Depth (100 Hz)"
            )
            st.code(f"Binary Symbol: sil_bridge_server.exe -> app_step_100hz()", language="text")

        p_col4, p_col5 = st.columns(2)
        with p_col4:
            st.markdown("#### [Stage 4] STM32 -> Core Telemetry")
            st.caption("CAN Arbitration IDs `0x200` (Nav), `0x210` (Leak), `0x300` (Power)")
            if client.nav_data:
                st.info(
                    f"**CAN ID 0x200 (33 Bytes):**\n"
                    f"- Depth: `{client.nav_data.depth_meters:.2f} m` | Yaw Rate: `{client.nav_data.gyro_z_rad_s:.3f} rad/s`\n"
                    f"- Quaternions: `({client.nav_data.q_w:.2f}, {client.nav_data.q_x:.2f}, {client.nav_data.q_y:.2f}, {client.nav_data.q_z:.2f})`\n"
                    f"- Status: IMU High Precision Mode ({client.nav_data.imu_status})"
                )
            else:
                st.warning("Awaiting CAN 0x200 Navigation Telemetry stream...")

        with p_col5:
            st.markdown("#### [Stage 5] Core -> Surface Telemetry")
            st.caption("Topside ZeroMQ SUB (`tcp://127.0.0.1:5556`) topic `telemetry`")
            st.info(
                f"**Protobuf Message: `SensorData`**\n"
                f"- Depth: `{client.pipeline_core_to_surface['depth']:.2f} m`\n"
                f"- Temperature: `{client.pipeline_core_to_surface['temp']:.1f} °C`\n"
                f"- Angular Velocity: `Vector3D (x, y, z)`\n"
                f"- Serialization: Protobuf v3 encoded"
            )
            st.code(f"Serialized Hex:\n{client.pipeline_core_to_surface['raw_hex'] or '(none)'}", language="text")

    # =========================================================================
    # TAB 2: DIRECT THRUSTERS & SOLENOIDS
    # =========================================================================
    with tab2:
        st.subheader("8-Channel Thruster ESC Command & PWM Monitor")
        st.caption("Direct pulse widths sent over CAN ID 0x100 -> Ramped via 1 kHz TIM6 slew-rate limiter on Control Board.")

        col_all1, col_all2, col_all3 = st.columns([1, 1, 2])
        with col_all1:
            if st.button("All Stop (1500 us)", use_container_width=True, key="btn_all_stop_tab2"):
                client.send_pwms([1500] * 8)
                st.rerun()
        with col_all2:
            if st.button("All Forward (1650 us)", use_container_width=True, key="btn_all_fwd_tab2"):
                client.send_pwms([1650] * 8)
                st.rerun()
        with col_all3:
            global_slider = st.slider("Master Sync Throttle", 1000, 2000, 1500, step=10, key="sync_throttle_tab2")
            if st.button("Apply Sync Throttle", key="btn_apply_sync_tab2"):
                client.send_pwms([global_slider] * 8)
                st.rerun()

        pwm_cols = st.columns(4)
        new_pwms = list(client.pwms)
        thruster_names = [
            "T0: Front-Left Horiz", "T1: Front-Right Horiz",
            "T2: Aft-Left Horiz",   "T3: Aft-Right Horiz",
            "T4: Front-Left Vert",  "T5: Front-Right Vert",
            "T6: Aft-Left Vert",    "T7: Aft-Right Vert"
        ]

        for i in range(8):
            col = pwm_cols[i % 4]
            with col:
                val = st.slider(
                    f"{thruster_names[i]}",
                    min_value=1000,
                    max_value=2000,
                    value=int(client.pwms[i]),
                    step=5,
                    key=f"thruster_slider_tab2_{i}",
                )
                new_pwms[i] = val
                delta = val - 1500
                st.progress((val - 1000) / 1000.0)
                st.caption(f"Effort: {delta:+d} us ({'Forward' if delta > 0 else 'Reverse' if delta < 0 else 'Neutral'})")

        if new_pwms != client.pwms:
            client.send_pwms(new_pwms)

        st.divider()
        st.subheader("10-Channel Pneumatic Solenoid Drivers (AO3400A)")
        st.caption("5 Double-Acting SMC SY3400-6U1-NA Valves switched via CAN ID 0x110.")

        sol_cols = st.columns(5)
        new_mask = 0
        for valve in range(5):
            with sol_cols[valve]:
                st.markdown(f"**Valve {valve + 1}**")
                chA = st.toggle(f"V{valve+1} Extend (Ch {valve*2})", value=bool(client.solenoid_mask & (1 << (valve*2))), key=f"sol_ext_{valve}")
                chB = st.toggle(f"V{valve+1} Retract (Ch {valve*2+1})", value=bool(client.solenoid_mask & (1 << (valve*2+1))), key=f"sol_ret_{valve}")
                if chA:
                    new_mask |= (1 << (valve * 2))
                if chB:
                    new_mask |= (1 << (valve * 2 + 1))

        if new_mask != client.solenoid_mask:
            client.send_solenoids(new_mask)

    # =========================================================================
    # TAB 3: NAVIGATION & ATTITUDE (100 Hz)
    # =========================================================================
    with tab3:
        st.subheader("100 Hz Navigation Telemetry Stream (CAN ID 0x200)")
        nav = client.nav_data
        if nav:
            col_nav1, col_nav2, col_nav3, col_nav4 = st.columns(4)
            col_nav1.metric("Depth (Hydrostatic)", f"{nav.depth_meters:.2f} m")
            col_nav2.metric("Angular Rate (Yaw)", f"{nav.gyro_z_rad_s:.3f} rad/s")
            col_nav3.metric("IMU Status", f"State {nav.imu_status} (High Precision)" if nav.imu_status == 3 else f"State {nav.imu_status}")
            col_nav4.metric("Attitude Norm", f"{(nav.q_w**2 + nav.q_x**2 + nav.q_y**2 + nav.q_z**2)**0.5:.3f}")

            st.markdown("#### Orientation Quaternions & Gyro Rates")
            q_cols = st.columns(4)
            q_cols[0].metric("Q_w", f"{nav.q_w:.4f}")
            q_cols[1].metric("Q_x", f"{nav.q_x:.4f}")
            q_cols[2].metric("Q_y", f"{nav.q_y:.4f}")
            q_cols[3].metric("Q_z", f"{nav.q_z:.4f}")

            gyro_cols = st.columns(3)
            gyro_cols[0].metric("Gyro X (Roll Rate)", f"{nav.gyro_x_rad_s:.4f} rad/s")
            gyro_cols[1].metric("Gyro Y (Pitch Rate)", f"{nav.gyro_y_rad_s:.4f} rad/s")
            gyro_cols[2].metric("Gyro Z (Yaw Rate)", f"{nav.gyro_z_rad_s:.4f} rad/s")

            if len(client.history_depth) > 1:
                st.line_chart(client.history_depth)
        else:
            st.info("Waiting for Navigation Telemetry stream from Control Board (Node 2)... Ensure the SIL Engine is Online.")

    # =========================================================================
    # TAB 4: ENVIRONMENTAL & LEAK (10 Hz)
    # =========================================================================
    with tab4:
        st.subheader("10 Hz Enclosure Environmental & Leak Telemetry (CAN ID 0x210)")
        env = client.env_data
        if env:
            col_env1, col_env2, col_env3 = st.columns(3)
            col_env1.metric("Internal Pressure (BME280)", f"{env.pressure_hpa:.1f} hPa")
            col_env2.metric("Relative Humidity", f"{env.humidity_pct:.1f} %")
            col_env3.metric("Enclosure Temperature", f"{env.temperature_c:.1f} °C")

            st.markdown("#### Vacuum Leak Decay & Probe Contact Sensors")
            leak_active = (env.leak_flags != 0)
            if leak_active:
                st.error(f"LEAK DETECTED ON NODE 1 (Flags: 0x{env.leak_flags:02X})")
            else:
                st.success("Sealed Enclosure Normal: No Ingress Detected (Floor Probes Dry, Humidity < 80%)")
        else:
            st.info("Waiting for Environmental Telemetry from Pi Shield (Node 1)...")

    # =========================================================================
    # TAB 5: POWER SLAB TELEMETRY (20 Hz)
    # =========================================================================
    with tab5:
        st.subheader("20 Hz Power Distribution & PMBus Telemetry (CAN ID 0x300)")
        pwr = client.power_data
        if pwr:
            col_pwr1, col_pwr2, col_pwr3, col_pwr4 = st.columns(4)
            col_pwr1.metric("48V Tether Voltage", f"{pwr.tether_voltage_mv / 1000.0:.2f} V")
            col_pwr2.metric("48V Tether Current", f"{pwr.tether_current_ma / 1000.0:.2f} A")
            col_pwr3.metric("5.2V Logic Rail", f"{pwr.v5_voltage_mv / 1000.0:.2f} V")
            col_pwr4.metric("PCB Copper Temp", f"{pwr.pcb_temp_c_tenths / 10.0:.1f} °C")

            st.markdown("#### 4x 12V 300W Converter Bricks (VCB4812EBO-300WFR3-N)")
            brick_cols = st.columns(4)
            for b in range(4):
                curr = pwr.v12_current_ma[b] / 1000.0
                watts = curr * 12.0
                brick_cols[b].metric(f"Brick {b+1} (ESCs {b*2},{b*2+1})", f"{curr:.2f} A", f"{watts:.1f} W")
                brick_cols[b].progress(min(1.0, curr / 25.0))
        else:
            st.info("Waiting for Power Telemetry from Power Slab (Node 3)...")

    # =========================================================================
    # TAB 6: RAW CAN BUS MONITOR & PACKET INSPECTOR
    # =========================================================================
    with tab6:
        st.subheader("Raw CAN Bus Monitor & Packet Inspector")
        st.caption("Real-time inspection of all CAN FD arbitration IDs, DLCs, hex payloads, and decoded data flowing across the virtual bus.")

        col_can_filter1, col_can_filter2, col_can_filter3 = st.columns([2, 1, 1])
        with col_can_filter1:
            filter_id = st.selectbox(
                "Filter by CAN ID",
                options=["ALL", "0x001 (EMERGENCY_BREAK)", "0x100 (THRUSTER_CMD)", "0x110 (SOLENOID_CMD)", "0x200 (NAV_TELEMETRY)", "0x210 (ENV_TELEMETRY)", "0x300 (POWER_TELEMETRY)"],
                index=0
            )
        with col_can_filter2:
            st.metric("Total Buffered Frames", len(client.packet_log))
        with col_can_filter3:
            if st.button("Clear Buffer", use_container_width=True):
                client.packet_log.clear()
                st.rerun()

        # Build table of packets
        packets = list(client.packet_log)
        if filter_id != "ALL":
            target_id_str = filter_id.split()[0]
            target_id = int(target_id_str, 16)
            packets = [p for p in packets if p.can_id == target_id]

        if packets:
            table_data = []
            for p in reversed(packets[-50:]):
                table_data.append({
                    "Time": p.timestamp,
                    "Direction": p.direction,
                    "ID": f"0x{p.can_id:03X}",
                    "Name": p.name,
                    "DLC": p.dlc,
                    "Hex Payload": p.hex_data,
                    "Decoded Summary": p.decoded_summary,
                })
            st.dataframe(table_data, use_container_width=True, height=400)
        else:
            st.info("No packets in buffer matching the selected filter.")

    # =========================================================================
    # TAB 7: LIVE C FIRMWARE CONSOLE & CODE VERIFICATION
    # =========================================================================
    with tab7:
        st.subheader("Live C Firmware Console & Native Execution Verification")
        st.markdown(
            "Verify that your **actual C firmware code** is compiling, linking, and executing inside `sil_bridge_server.exe`."
        )

        st.markdown("#### 1. Live C Standard Output (stdout stream from `sil_bridge_server.exe`)")
        c_lines = list(client.c_stdout_log)
        if c_lines:
            st.text_area("C Engine Terminal Output", value="\n".join(c_lines[-40:]), height=300, disabled=True)
        else:
            st.info("No C stdout output yet. Make sure the SIL Engine is running in the sidebar.")

        st.divider()
        st.markdown("#### 2. How to Verify Your C Code is Actually Running")
        st.markdown(
            """
            This Software-in-the-Loop testbench runs **native machine code compiled from your exact C source files**:
            - `nodes/node2_control_board/Core/Src/app.c`
            - `nodes/node1_pi_shield/Core/Src/app.c`
            - `nodes/node3_power_slab/Core/Src/app.c`
            - `shared/src/x19_pwm_ramp.c`, `shared/src/x19_safety.c`, `shared/src/can_interface.c`
            - `drivers/src/lsm6dsoxtr.c`, `drivers/src/ms5837.c`, `drivers/src/bme280.c`

            **To test and prove your own C code modifications:**
            1. Open `nodes/node2_control_board/Core/Src/app.c` in your editor.
            2. Add any custom debug statement, for example:
               ```c
               printf(">>> MY CUSTOM C CODE: Target T0=%u us, Current Depth=%.2f m\\n",
                      g_target_pwms.pwm_us[0], g_depth_dev.depth_m);
               fflush(stdout);
               ```
            3. In a terminal, recompile the C target:
               ```powershell
               cmake --build build --target sil_bridge_server
               ```
            4. In the dashboard sidebar, click **Restart Engine**.
            5. Watch your custom `printf` statements appear live right in the box above!
            """
        )

    # Auto-refresh loop when connected
    if client.connected:
        time.sleep(0.1)
        st.rerun()

if __name__ == "__main__":
    main()
