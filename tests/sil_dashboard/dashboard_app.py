"""
X19 Software-in-the-Loop (SIL) Interactive Testing Dashboard.
Powered by Streamlit.

Enables live hardware-free testing and end-to-end verification strictly within X19-Embedded:
- Topside Surface Pilot Station: 6-DOF controls (Surge, Sway, Heave, Yaw) mapped to Protobuf commands.
- End-to-End Message Pipeline Tracer:
    Downlink: Surface (Protobuf JoystickCommand) -> Core (Thrust Allocation Matrix & CAN Packing)
              -> STM32 CAN Bus (0x100) -> STM32 C Firmware Execution (app.c, TIM6 ramp, BDTR latch)
    Uplink:   STM32 Telemetry (0x200, 0x210, 0x300) -> Core Topside Translation (Protobuf SensorData)
              -> Topside Pilot Heads-Up Display (HUD)
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
import struct
import threading
import subprocess
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Dict, Any

import streamlit as st

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
            self.emergency_break_tripped = True
            if not self.connected or not self.sock:
                return
            frame = pack_sil_can_frame(CAN_ID_EMERGENCY_BREAK, payload)
            try:
                self.sock.sendall(frame)
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
            if st.button("Restart Engine", width="stretch"):
                client.stop_server_process()
                time.sleep(0.3)
                client.start_server_process()
                client.connect()
                st.rerun()
        with col_srv2:
            if st.button("Stop Engine", width="stretch"):
                client.stop_server_process()
                st.rerun()

        status_color = "green" if client.connected else "red"
        st.markdown(f"**Connection Status:** :{status_color}[{'ONLINE (127.0.0.1:8765)' if client.connected else 'OFFLINE'}]")
        st.metric("Processed CAN Packets", client.frame_count)
        st.metric("Logged CAN Frames", len(client.packet_log))

        st.divider()
        st.subheader("Live Telemetry Streaming")
        auto_refresh = st.toggle("Auto-Refresh Telemetry", value=True, help="Periodically re-renders live gauges.")
        refresh_rate = 0.5
        if auto_refresh:
            refresh_rate = st.select_slider("Refresh Interval", options=[0.2, 0.5, 1.0, 2.0], value=0.5, format_func=lambda x: f"{x}s")

        st.divider()
        st.subheader("Fault & Safety Injection")
        if st.button("TRIP EMERGENCY BREAK (0x001)", type="primary", width="stretch"):
            client.trigger_emergency_break()
            st.error("Priority 0 Emergency Break Triggered! Thrusters cut to 1500 us neutral.")

        if client.emergency_break_tripped:
            st.error("VEHICLE STATE: EMERGENCY LATCHED (TIMx_BDTR Active)")
            if st.button("Reset E-Break State", width="stretch"):
                client.emergency_break_tripped = False
                st.rerun()

        st.divider()
        st.markdown("### Subsea Nodes")
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
            "Trace every boundary of the Purdue ROV distributed system in real time: "
            "from **Surface Pilot Gamepad Input** down through the **Pi 5 Companion Engine** "
            "to **STM32 C Microcontroller Timers**, and all the way back up to the **Topside Pilot HUD**."
        )

        with st.expander("System Architecture Overview (Click to expand)", expanded=False):
            st.markdown(
                """
```
DOWNLINK (Control Flow: Surface -> Vehicle):
  [Topside Pilot Station] ──(ZeroMQ PUB: JoystickCommand Protobuf)──> [Pi 5 Companion Engine]
                                                                             │
                                                                 (8-Thruster Allocation)
                                                                             │
                                                                             ▼
  [Physical ESC Timers] <──(TIM6 Slew-Rate Limiter)── [STM32 Control Board] <──(CAN FD 0x100)

UPLINK (Telemetry Flow: Subsea Sensors -> Pilot Screen):
  [IMU & Depth Sensors] ──> [STM32 Control Board] ──(CAN FD 0x200 @ 100 Hz)──> [Pi 5 Companion Engine]
  [Leak & Enclosure]    ──> [STM32 Pi Shield]    ──(CAN FD 0x210 @ 10 Hz) ──>          │
  [48V/12V Power Slab]  ──> [STM32 Power Slab]   ──(CAN FD 0x300 @ 20 Hz) ──> (Telemetry Aggregator)
                                                                                       │
  [Surface Pilot HUD] <──(ZeroMQ SUB: SensorData Protobuf)─────────────────────────────┘
```
                """
            )

        st.markdown("### 1. Topside Pilot Flight Deck")
        col_ctrl1, col_ctrl2 = st.columns([1, 1])

        with col_ctrl1:
            st.markdown("**6-DOF Flight Axes** (Drag to pilot vehicle)")
            surge = st.slider("Surge (Forward / Reverse)", -1.0, 1.0, float(client.pipeline_surface_cmd["surge"]), 0.05, key="slider_surge")
            sway = st.slider("Sway (Strafe Right / Left)", -1.0, 1.0, float(client.pipeline_surface_cmd["sway"]), 0.05, key="slider_sway")
            heave = st.slider("Heave (Dive / Ascend)", -1.0, 1.0, float(client.pipeline_surface_cmd["heave"]), 0.05, key="slider_heave")
            yaw = st.slider("Yaw (Turn Right / Left)", -1.0, 1.0, float(client.pipeline_surface_cmd["yaw"]), 0.05, key="slider_yaw")

        with col_ctrl2:
            st.markdown("**Flight Presets**")
            preset_cols = st.columns(3)
            with preset_cols[0]:
                if st.button("All Stop (Hover)", width="stretch"):
                    client.send_surface_pilot_command(0.0, 0.0, 0.0, 0.0)
                    st.rerun()
            with preset_cols[1]:
                if st.button("Forward (+0.5 Surge)", width="stretch"):
                    client.send_surface_pilot_command(0.5, 0.0, 0.0, 0.0)
                    st.rerun()
            with preset_cols[2]:
                if st.button("Reverse (-0.5 Surge)", width="stretch"):
                    client.send_surface_pilot_command(-0.5, 0.0, 0.0, 0.0)
                    st.rerun()

            preset_cols2 = st.columns(3)
            with preset_cols2[0]:
                if st.button("Strafe Right (+0.5 Sway)", width="stretch"):
                    client.send_surface_pilot_command(0.0, 0.5, 0.0, 0.0)
                    st.rerun()
            with preset_cols2[1]:
                if st.button("Dive (+0.5 Heave)", width="stretch"):
                    client.send_surface_pilot_command(0.0, 0.0, 0.5, 0.0)
                    st.rerun()
            with preset_cols2[2]:
                if st.button("Yaw Right (+0.5 Yaw)", width="stretch"):
                    client.send_surface_pilot_command(0.0, 0.0, 0.0, 0.5)
                    st.rerun()

            if (surge != client.pipeline_surface_cmd["surge"] or
                sway != client.pipeline_surface_cmd["sway"] or
                heave != client.pipeline_surface_cmd["heave"] or
                yaw != client.pipeline_surface_cmd["yaw"]):
                client.send_surface_pilot_command(surge, sway, heave, yaw)

        st.divider()
        st.markdown("### 2. Live End-to-End Message Flow Inspector")

        # ---------------------------------------------------------------------
        # PART A: DOWNLINK COMMAND PATH
        # ---------------------------------------------------------------------
        st.markdown("#### Part A: Downlink Command Path (Pilot Input -> Physical Thrusters)")
        st.caption("How your joystick stick movements are transformed across software boundaries to spin vehicle motors.")

        col_stg1, col_stg2, col_stg3 = st.columns(3)

        # STAGE 1
        with col_stg1:
            st.markdown("##### [Stage 1] Surface -> Core")
            st.caption("Topside ZeroMQ PUB (`tcp://127.0.0.1:5555`) topic `joystick`")
            st.info(
                "**Purpose of Stage 1 (`JoystickCommand`):**\n"
                "Captures the pilot's raw controller input. Instead of sending motor pulse widths over "
                "the tether, the topside pilot interface speaks in vehicle-centric 6-DOF coordinates "
                "(Surge, Sway, Heave, Yaw)."
            )

            ts_us = client.pipeline_surface_cmd.get("timestamp_us", 0)
            st.dataframe(
                [
                    {"Field": "forward (Surge)", "Value": f"{client.pipeline_surface_cmd['surge']:+.2f}", "Description": "Forward/Reverse throttle (-1 to +1)"},
                    {"Field": "strafe (Sway)", "Value": f"{client.pipeline_surface_cmd['sway']:+.2f}", "Description": "Lateral strafe Right/Left"},
                    {"Field": "vertical (Heave)", "Value": f"{client.pipeline_surface_cmd['heave']:+.2f}", "Description": "Vertical Dive/Ascend"},
                    {"Field": "yaw", "Value": f"{client.pipeline_surface_cmd['yaw']:+.2f}", "Description": "Heading rotation Right/Left"},
                    {"Field": "timestamp_us", "Value": f"{ts_us}", "Description": "Microsecond timestamp for latency tracking"},
                ],
                width="stretch",
                hide_index=True
            )
            st.markdown("**Protobuf Wire Serialization (Hex):**")
            st.code(client.pipeline_surface_cmd.get("raw_hex", "08 00 15 00 00 00 00"), language="text")

        # STAGE 2
        with col_stg2:
            st.markdown("##### [Stage 2] Core -> STM32")
            st.caption("CAN FD Arbitration ID `0x100` (`THRUSTER_CMD`, DLC: 16 Bytes)")
            st.info(
                "**Purpose of Stage 2 (`THRUSTER_CMD`):**\n"
                "The Pi 5 receives `JoystickCommand`. It solves the Purdue ROV 8-thruster allocation geometry matrix "
                "(4 vectored horizontal @ 45°, 4 vertical) to convert 6-DOF motion into 8 discrete motor pulse widths."
            )

            pwms = client.pipeline_core_pwms
            thruster_meta = [
                ("T0", "Front-Left Horiz (45°)", "Surge + Sway + Yaw", pwms[0]),
                ("T1", "Front-Right Horiz (45°)", "Surge - Sway - Yaw", pwms[1]),
                ("T2", "Aft-Left Horiz (45°)", "Surge - Sway + Yaw", pwms[2]),
                ("T3", "Aft-Right Horiz (45°)", "Surge + Sway - Yaw", pwms[3]),
                ("T4", "Front-Left Vert", "-Heave + Pitch - Roll", pwms[4]),
                ("T5", "Front-Right Vert", "-Heave + Pitch + Roll", pwms[5]),
                ("T6", "Aft-Left Vert", "-Heave - Pitch - Roll", pwms[6]),
                ("T7", "Aft-Right Vert", "-Heave - Pitch + Roll", pwms[7]),
            ]
            matrix_table = []
            for tid, name, formula, pwm in thruster_meta:
                effort = pwm - 1500
                effort_str = f"{effort:+d} us ({'Fwd' if effort > 0 else 'Rev' if effort < 0 else 'Stop'})"
                hex_le = f"{(pwm & 0xFF):02X} {((pwm >> 8) & 0xFF):02X}"
                matrix_table.append({
                    "Thruster": tid,
                    "Position": name,
                    "Target PWM": f"{pwm} us",
                    "Effort": effort_str,
                    "LE Hex": hex_le,
                })
            st.dataframe(matrix_table, width="stretch", hide_index=True)

            st.markdown("**CAN FD ID 0x100 Payload (8x uint16_t Little-Endian):**")
            st.code(client.pipeline_core_can_hex or "DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05", language="text")

        # STAGE 3
        with col_stg3:
            st.markdown("##### [Stage 3] STM32 C Firmware")
            st.caption("Control Board Node 2 (`Src/app.c`) Native C Execution")
            st.info(
                "**Purpose of Stage 3 (STM32 Control Board Firmware):**\n"
                "The physical STM32 microcontroller receives CAN ID 0x100, enforces safety checks (emergency break, "
                "leak kill switch), and applies a 1 kHz TIM6 slew-rate filter (1000 us/s max ramp rate). "
                "This smooths out abrupt joystick moves so the 12V 300W DC-DC converters don't brown out."
            )

            e_status = "LATCHED: TIMx_BDTR HARDWARE CUTOFF" if client.emergency_break_tripped else "ARMED & RUNNING"
            st.dataframe(
                [
                    {"Parameter": "Safety State", "Status": e_status, "Detail": "Emergency Break / Leak Interlock"},
                    {"Parameter": "Slew Limiter (TIM6)", "Status": "1000 us/s Active", "Detail": "Limits inrush current on 12V rail"},
                    {"Parameter": "Active PWM Output", "Status": f"T0={pwms[0]} us, T1={pwms[1]} us", "Detail": "Loaded into TIM1/TIM8 compare registers"},
                    {"Parameter": "Hardware Output", "Status": "8x Basic ESCs", "Detail": "TIM1_CH1..CH4, TIM8_CH1..CH4 pins"},
                ],
                width="stretch",
                hide_index=True
            )
            st.markdown("**Compiled Machine Code:**")
            st.code("Target: build/tests/sil_bridge_server.exe\nModule: nodes/node2_control_board/Core/Src/app.c", language="text")

        st.divider()

        # ---------------------------------------------------------------------
        # PART B: UPLINK TELEMETRY PATH
        # ---------------------------------------------------------------------
        st.markdown("#### Part B: Uplink Telemetry Path (Vehicle Sensors -> Pilot Heads-Up Display)")
        st.caption("How physical subsea sensor measurements are gathered, packed, and streamed to the pilot's Primary Flight Display (HUD).")

        col_stg4, col_stg5 = st.columns(2)

        # STAGE 4
        with col_stg4:
            st.markdown("##### [Stage 4] STM32 -> Core Telemetry")
            st.caption("CAN Arbitration IDs `0x200` (Nav @ 100 Hz), `0x210` (Env @ 10 Hz), `0x300` (Power @ 20 Hz)")
            st.info(
                "**Purpose of Stage 4 (Subsea Sensor Acquisition):**\n"
                "The subsea microcontrollers read their physical onboard sensors and stream data back onto the vehicle's "
                "internal CAN FD bus. Node 2 streams 100 Hz Navigation (BMI270 IMU + MS5837 Depth), Node 1 streams 10 Hz "
                "Enclosure health (BME280 pressure/temp/humidity and leak probes), and Node 3 streams 20 Hz Power distribution."
            )

            nav = client.nav_data
            env = client.env_data
            pwr = client.power_data

            sensor_rows = [
                {"Node & Board": "Node 2: Control Board", "CAN ID": "0x200 (Nav)", "Sensor": "MS5837 Depth", "Live Value": f"{nav.depth_meters:.2f} m" if nav else "0.00 m"},
                {"Node & Board": "Node 2: Control Board", "CAN ID": "0x200 (Nav)", "Sensor": "LSM6DSOX Gyro", "Live Value": f"Yaw={nav.gyro_z_rad_s:.3f} rad/s" if nav else "0.000 rad/s"},
                {"Node & Board": "Node 2: Control Board", "CAN ID": "0x200 (Nav)", "Sensor": "LSM6DSOX Quaternions", "Live Value": f"({nav.q_w:.2f}, {nav.q_x:.2f}, {nav.q_y:.2f}, {nav.q_z:.2f})" if nav else "(1.00, 0.00, 0.00, 0.00)"},
                {"Node & Board": "Node 1: Pi Shield", "CAN ID": "0x210 (Env)", "Sensor": "BME280 Pressure", "Live Value": f"{env.pressure_hpa:.1f} hPa" if env else "1013.2 hPa"},
                {"Node & Board": "Node 1: Pi Shield", "CAN ID": "0x210 (Env)", "Sensor": "Leak Probes", "Live Value": f"0x{env.leak_flags:02X} ({'INGRESS' if env and env.leak_flags else 'DRY/OK'})" if env else "0x00 (DRY/OK)"},
                {"Node & Board": "Node 3: Power Slab", "CAN ID": "0x300 (Pwr)", "Sensor": "INA228 48V Rail", "Live Value": f"{pwr.tether_voltage_mv/1000:.1f}V @ {pwr.tether_current_ma/1000:.1f}A" if pwr else "48.0V @ 2.2A"},
            ]
            st.dataframe(sensor_rows, width="stretch", hide_index=True)

        # STAGE 5
        with col_stg5:
            st.markdown("##### [Stage 5] Core -> Surface Telemetry")
            st.caption("Topside ZeroMQ SUB (`tcp://127.0.0.1:5556`) topic `telemetry`")
            st.info(
                "**What is this supposed to show me?**\n"
                "Stage 5 is the primary telemetry packet sent from the ROV across the tether to your topside Surface Laptop "
                "over ZeroMQ topic `telemetry`. It aggregates the subsea CAN frames into a clean Protobuf message (`SensorData`) "
                "so the pilot's UI doesn't have to deal with raw CAN bus protocols.\n\n"
                "The pilot software uses this exact message to draw the **Primary Flight Display (PFD) / Heads-Up Display (HUD)**: "
                "the artificial horizon, depth gauge, compass tape, and battery/leak warning indicators on the pilot screen."
            )

            p_srf = client.pipeline_core_to_surface
            st.dataframe(
                [
                    {"Field": "depth", "Decoded Value": f"{p_srf['depth']:.2f} meters", "Role on Pilot Heads-Up Display (HUD)": "Vertical depth tape & auto-depth PID target"},
                    {"Field": "temperature", "Decoded Value": f"{p_srf['temp']:.1f} °C", "Role on Pilot Heads-Up Display (HUD)": "Hull overheating alarm indicator"},
                    {"Field": "angular_velocity", "Decoded Value": f"X:{p_srf['gyro_x']:.2f}, Y:{p_srf['gyro_y']:.2f}, Z:{p_srf['gyro_z']:.2f} rad/s", "Role on Pilot Heads-Up Display (HUD)": "Rate-of-turn indicator & gyro stabilization"},
                    {"Field": "acceleration", "Decoded Value": f"X:{p_srf['accel_x']:.2f}, Y:{p_srf['accel_y']:.2f}, Z:{p_srf['accel_z']:.2f} m/s²", "Role on Pilot Heads-Up Display (HUD)": "Gravity vector, tilt estimator, collision detector"},
                    {"Field": "timestamp_us", "Decoded Value": f"{p_srf['timestamp_us']}", "Role on Pilot Heads-Up Display (HUD)": "Tether latency & packet freshness heartbeat"},
                ],
                width="stretch",
                hide_index=True
            )
            st.markdown("**Protobuf Wire Serialization (Hex):**")
            st.code(p_srf.get("raw_hex", "08 00 1D 00 00 00 00 25 00 00 B4 41"), language="text")

    # =========================================================================
    # TAB 2: DIRECT THRUSTERS & SOLENOIDS
    # =========================================================================
    with tab2:
        st.subheader("8-Channel Thruster ESC Command & PWM Monitor")
        st.caption("Direct pulse widths sent over CAN ID 0x100 -> Ramped via 1 kHz TIM6 slew-rate limiter on Control Board.")

        col_all1, col_all2, col_all3 = st.columns([1, 1, 2])
        with col_all1:
            if st.button("All Stop (1500 us)", width="stretch", key="btn_all_stop_tab2"):
                client.send_pwms([1500] * 8)
                st.rerun()
        with col_all2:
            if st.button("All Forward (1650 us)", width="stretch", key="btn_all_fwd_tab2"):
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
            if st.button("Clear Buffer", width="stretch"):
                client.packet_log.clear()
                st.rerun()

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
            st.dataframe(table_data, width="stretch", height=400)
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
               printf(">>> MY CUSTOM C CODE: Target T0=%u us, Current Depth=%.2f m\n",
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

    # Controlled auto-refresh loop
    if client.connected and auto_refresh:
        time.sleep(refresh_rate)
        st.rerun()

if __name__ == "__main__":
    main()
