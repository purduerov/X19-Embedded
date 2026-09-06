"""
X19 Software-in-the-Loop (SIL) Interactive Testing Dashboard.
Powered by Streamlit.

Enables live hardware-free testing:
- Start/stop the native STM32 SIL Bridge server (Node 1, Node 2, Node 3).
- Real-time 8-Thruster PWM slider control & monitoring.
- 10-Channel Pneumatic Solenoid toggle switches.
- Fault injection: Priority 0 Emergency Break, Enclosure Leaks, PMBus overcurrent trips.
- Real-time live gauges: 100 Hz Navigation (depth, quaternions, gyro), BME280 leak status, 48V/12V power telemetry.
- ZeroMQ bridge pass-through to X19-Core / Surface UI.
"""

import os
import sys
import time
import socket
import select
import threading
import subprocess
from dataclasses import dataclass
from typing import List, Optional

import streamlit as st

# Path configuration
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
BRIDGE_DIR = os.path.join(REPO_ROOT, "tests", "sil_companion_bridge")
if BRIDGE_DIR not in sys.path:
    sys.path.insert(0, BRIDGE_DIR)

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

class SilDashboardClient:
    def __init__(self, host: str = "127.0.0.1", port: int = 8765):
        self.host = host
        self.port = port
        self.sock: Optional[socket.socket] = None
        self.server_proc: Optional[subprocess.Popen] = None
        self.connected = False
        self.running = False
        self.lock = threading.Lock()

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

    def start_server_process(self) -> bool:
        if not os.path.exists(SERVER_EXE_PATH):
            return False
        if self.server_proc is None or self.server_proc.poll() is not None:
            self.server_proc = subprocess.Popen(
                [SERVER_EXE_PATH, "--port", str(self.port)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            time.sleep(0.4)
            return True
        return True

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

    def send_pwms(self, pwms: List[int]):
        with self.lock:
            if not self.connected or not self.sock:
                return
            self.pwms = list(pwms)
            cmd = ThrusterCommand(pwm_us=self.pwms)
            frame = pack_sil_can_frame(CAN_ID_THRUSTER_CMD, cmd.pack())
            try:
                self.sock.sendall(frame)
            except OSError:
                self.connected = False

    def send_solenoids(self, mask: int):
        with self.lock:
            if not self.connected or not self.sock:
                return
            self.solenoid_mask = mask & 0x03FF
            cmd = SolenoidCommand(solenoid_mask=self.solenoid_mask)
            frame = pack_sil_can_frame(CAN_ID_SOLENOID_CMD, cmd.pack())
            try:
                self.sock.sendall(frame)
            except OSError:
                self.connected = False

    def trigger_emergency_break(self):
        with self.lock:
            if not self.connected or not self.sock:
                return
            frame = pack_sil_can_frame(CAN_ID_EMERGENCY_BREAK, b"\x01")
            try:
                self.sock.sendall(frame)
                self.emergency_break_tripped = True
            except OSError:
                self.connected = False

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
                    if can_id == CAN_ID_NAV_TELEMETRY:
                        nav = NavTelemetry.unpack(payload)
                        self.nav_data = nav
                        self.history_depth.append(nav.depth_meters)
                        self.history_time.append(time.time())
                        if len(self.history_depth) > 100:
                            self.history_depth.pop(0)
                            self.history_time.pop(0)

                    elif can_id == CAN_ID_ENV_TELEMETRY:
                        self.env_data = EnvTelemetry.unpack(payload)
                        if self.env_data.leak_flags != 0:
                            self.emergency_break_tripped = True

                    elif can_id == CAN_ID_POWER_TELEMETRY:
                        self.power_data = PowerTelemetry.unpack(payload)

                    elif can_id == CAN_ID_EMERGENCY_BREAK:
                        self.emergency_break_tripped = True
            except Exception:
                break
        self.connected = False

# Global state management inside Streamlit session
@st.cache_resource
def get_sil_client() -> SilDashboardClient:
    return SilDashboardClient()

def main():
    st.set_page_config(
        page_title="X19 ROV - Embedded SIL Testing Dashboard",
        layout="wide",
        initial_sidebar_state="expanded",
    )

    client = get_sil_client()

    # --- SIDEBAR: Master Controls & Hardware Injection ---
    with st.sidebar:
        st.title("X19 SIL Master Hub")
        st.markdown("**Subsea Node Firmware Simulation**")

        st.subheader("SIL Server State")
        # Auto-connect if not already connected
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

        st.divider()
        st.subheader("Fault & Safety Injection")
        if st.button("TRIP EMERGENCY BREAK (0x001)", type="primary", use_container_width=True):
            client.trigger_emergency_break()
            st.error("Priority 0 Emergency Break Triggered! Thrusters cut to 1500 us neutral.")

        if client.emergency_break_tripped:
            st.error("VEHICLE STATE: EMERGENCY LATCHED (MOE Disabled)")
            if st.button("Reset E-Break State", use_container_width=True):
                client.emergency_break_tripped = False
                st.rerun()

        st.divider()
        st.markdown("### Vehicle Subsystems")
        st.markdown("- **Node 1**: Pi Shield (STM32C5)")
        st.markdown("- **Node 2**: Control Board (STM32C5 + FPU)")
        st.markdown("- **Node 3**: Power Slab (STM32C5)")
        st.markdown("- **Node 4**: USB Camera Hub (PCIe)")

    # --- MAIN DASHBOARD INTERFACE ---
    st.title("Purdue ROV — X19 Embedded SIL Testing Station")
    st.caption("Live hardware-in-the-loop firmware simulation for propulsion, actuation, active control, and telemetry.")

    tab1, tab2, tab3, tab4 = st.tabs([
        "Thrusters & Solenoids",
        "Navigation & Attitude (100 Hz)",
        "Environmental & Leak (10 Hz)",
        "Power Slab Telemetry (20 Hz)"
    ])

    # --- TAB 1: 8-THRUSTER PWMs & 10-CH SOLENOIDS ---
    with tab1:
        st.subheader("8-Channel Thruster ESC Command & PWM Monitor")
        st.caption("Target pulse widths sent over CAN ID 0x100 -> Ramped via 1 kHz TIM6 slew-rate limiter on Control Board.")

        col_all1, col_all2, col_all3 = st.columns([1, 1, 2])
        with col_all1:
            if st.button("All Stop (1500 us)", use_container_width=True):
                client.send_pwms([1500] * 8)
                st.rerun()
        with col_all2:
            if st.button("All Forward (1650 us)", use_container_width=True):
                client.send_pwms([1650] * 8)
                st.rerun()
        with col_all3:
            global_slider = st.slider("Master Sync Throttle", 1000, 2000, 1500, step=10)
            if st.button("Apply Sync Throttle"):
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
                    key=f"thruster_slider_{i}",
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
                chA = st.toggle(f"V{valve+1} Extend (Ch {valve*2})", value=bool(client.solenoid_mask & (1 << (valve*2))))
                chB = st.toggle(f"V{valve+1} Retract (Ch {valve*2+1})", value=bool(client.solenoid_mask & (1 << (valve*2+1))))
                if chA:
                    new_mask |= (1 << (valve * 2))
                if chB:
                    new_mask |= (1 << (valve * 2 + 1))

        if new_mask != client.solenoid_mask:
            client.send_solenoids(new_mask)

    # --- TAB 2: NAVIGATION & ATTITUDE (100 Hz) ---
    with tab2:
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
            st.info("Waiting for Navigation Telemetry stream from Control Board (Node 2)... Click 'Start SIL Engine' in the sidebar.")

    # --- TAB 3: ENVIRONMENTAL & LEAK (10 Hz) ---
    with tab3:
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

    # --- TAB 4: POWER SLAB TELEMETRY (20 Hz) ---
    with tab4:
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

    # Auto-refresh loop when connected
    if client.connected:
        time.sleep(0.1)
        st.rerun()

if __name__ == "__main__":
    main()
