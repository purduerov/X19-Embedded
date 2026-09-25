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
from enum import Enum
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

# --- Deadman control constants -------------------------------------------------
# These mirror the firmware in shared/include/rov_parameters.h.  They are the
# only place the host is allowed to encode the PWM envelope or the watchdog
# window; nothing here may widen them.
PWM_MIN_US = 1000          # ROV_PWM_MIN_US
PWM_MAX_US = 2000          # ROV_PWM_MAX_US
NEUTRAL_PWM_US = 1500      # ROV_PWM_STOP_US
NEUTRAL_PWMS = [NEUTRAL_PWM_US] * 8

# The control worker resends the last accepted command every 50 ms so Node 2's
# 100 ms heartbeat watchdog never expires while the operator holds a command.
CONTROL_PERIOD_S = 0.050
HEARTBEAT_TIMEOUT_S = 0.100
CONTROL_JOIN_TIMEOUT_S = 1.0

# Node 2 only accepts a thruster command after this authorized signature check
# (nodes/node2_control_board/Core/Src/app.c).  The exact bytes are load-bearing.
EMERGENCY_BREAK_SIGNATURE = b"\xAA\x55\x01"


class ControlState(str, Enum):
    """
    Explicit dashboard deadman state.

        DISCONNECTED -> STOPPED -> ARMED -> RUNNING
              ^            |          |         |
              +------------+----------+---------+
                           STOP / ESTOP

    * ``DISCONNECTED``: no SIL transport, no frames are ever sent.
    * ``STOPPED``: no command accepted; the last command is neutral.
    * ``ARMED``: transport up, no non-neutral command active.
    * ``RUNNING``: the last accepted command is resent every 50 ms (20 Hz).
    * ``ESTOP``: the authorized emergency frame was sent and outputs are forced
      neutral.  This state LATCHES: only ``connect()`` (a new transport) or
      ``start_server_process()`` (a new engine) clears it.
    """

    DISCONNECTED = "disconnected"
    STOPPED = "stopped"
    ARMED = "armed"
    RUNNING = "running"
    ESTOP = "estop"


def _validate_pwms(pwms: Any) -> List[int]:
    """
    Reject anything that is not exactly eight PWM values inside 1000-2000 us.

    The firmware clamps out-of-range values, but silently clamping a host bug
    would hide it, so the host refuses instead.  Integral floats are accepted
    because Streamlit slider values are not guaranteed to be ``int``.
    """
    if pwms is None or isinstance(pwms, (str, bytes)):
        raise ValueError(f"PWM command must be a sequence of 8 values, got {pwms!r}")
    values = list(pwms)
    if len(values) != 8:
        raise ValueError(f"PWM command requires exactly 8 values, got {len(values)}")
    validated: List[int] = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"PWM values must be numbers, got {value!r}")
        numeric = float(value)
        # is_integer() is False for NaN and both infinities.
        if not numeric.is_integer():
            raise ValueError(f"PWM values must be finite whole microseconds, got {value!r}")
        clamped = int(numeric)
        if clamped < PWM_MIN_US or clamped > PWM_MAX_US:
            raise ValueError(
                f"PWM values must stay within {PWM_MIN_US}-{PWM_MAX_US} us, got {clamped}"
            )
        validated.append(clamped)
    return validated


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
        # Serializes socket writes only. It is deliberately NOT the state lock:
        # holding self.lock across a blocking sendall would stall _rx_loop and
        # blind the dashboard to telemetry exactly when something has gone wrong.
        # Re-entrant because the worker can fault while already inside a write.
        self._send_lock = threading.RLock()

        # 20 Hz deadman control worker
        self.control_state = ControlState.DISCONNECTED
        self._last_command: List[int] = list(NEUTRAL_PWMS)
        self._control_stop = threading.Event()
        self._control_thread: Optional[threading.Thread] = None
        self._estop_latched = False
        self.worker_thread: Optional[threading.Thread] = None

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
            # Restarting the engine is one of the two explicit, operator-driven
            # ways out of a latched ESTOP (the other is a fresh connect()).
            self._estop_latched = False
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
            s = None
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect((self.host, self.port))
                s.setblocking(False)
                self.sock = s
                self.connected = True
                self.running = True
                # A genuinely new transport is the operator explicitly
                # reconnecting, so it clears a latched ESTOP and hands the
                # control path back in a neutral, non-RUNNING state.
                self._estop_latched = False
                self._last_command = list(NEUTRAL_PWMS)
                self._control_stop = threading.Event()
                self._control_thread = None
                self.control_state = ControlState.ARMED
                self.worker_thread = threading.Thread(target=self._rx_loop, daemon=True)
                self.worker_thread.start()
                return True
            except (ConnectionRefusedError, OSError):
                if s is not None:
                    try:
                        s.close()
                    except OSError:
                        pass
                self.sock = None
                self.connected = False
                self.control_state = ControlState.DISCONNECTED
                return False

    def disconnect(self):
        # Stop transmitting BEFORE the socket goes away, so no frame can be
        # written to a closing transport. The neutral frame is deliberately not
        # sent here: the firmware heartbeat watchdog is what guarantees neutral
        # once the transport is gone, and writing to a dead socket cannot.
        self.running = False
        self.stop_control_loop(send_neutral=False)
        with self.lock:
            if self.sock:
                try:
                    self.sock.close()
                except OSError:
                    pass
                self.sock = None
            self.connected = False
            self.control_state = ControlState.DISCONNECTED

    def send_surface_pilot_command(self, surge: float, sway: float, heave: float, yaw: float, pitch: float = 0.0, roll: float = 0.0) -> bool:
        """
        Executes the full Topside Surface -> Core -> STM32 pipeline:
        1. Packs Surface Protobuf JoystickCommand.
        2. Computes Core 8-Thruster Allocation Matrix.
        3. Hands CAN ID 0x100 to the 20 Hz control worker.

        This is a deadman input, not a one-shot: while the operator holds the
        axis the worker resends the allocation at 20 Hz, and any STOP / ESTOP /
        disconnect forces neutral.
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

        return self.send_pwms(pwms)

    # ------------------------------------------------------------------
    # Transport primitives
    # ------------------------------------------------------------------

    def _send_frame(self, direction: str, can_id: int, payload: bytes, summary: str) -> None:
        """
        Push one complete SIL frame down the wire.

        The state lock is never held here: a blocking or contended send must not
        be able to stall _rx_loop. Only _send_lock is taken, so a 73-byte write
        from the UI thread cannot interleave with a worker write and corrupt the
        framing. Raises OSError if the transport is gone.
        """
        sock = self.sock
        if sock is None or not self.connected:
            raise OSError("SIL transport is not connected")
        frame = pack_sil_can_frame(can_id, payload)
        with self._send_lock:
            sock.sendall(frame)
        self._record_packet(direction, can_id, payload, summary)

    def _send_thruster_frame(self, pwms: List[int]) -> None:
        """Transmit one CAN ID 0x100 frame. Raises OSError on transport failure."""
        channels = list(pwms)
        payload = ThrusterCommand(pwm_us=channels).pack()
        self._send_frame("Core -> STM32", CAN_ID_THRUSTER_CMD, payload, f"PWMs: {channels}")

    # ------------------------------------------------------------------
    # Deadman control state machine
    # ------------------------------------------------------------------

    def _transition_stopped(self, send_neutral: bool = True) -> None:
        """
        Single fail-safe transition shared by worker faults, explicit stop, and
        emergency break, so every exit path behaves identically.

        Never downgrades a latched ESTOP: a socket error arriving after an
        emergency break must not make the UI look safe.
        """
        self._control_stop.set()
        if send_neutral and self.connected and self.sock is not None:
            try:
                self._send_thruster_frame(list(NEUTRAL_PWMS))
            except OSError:
                # The firmware watchdog is the backstop once the transport is
                # gone; there is nothing further this process can transmit.
                self.connected = False
        with self.lock:
            self._last_command = list(NEUTRAL_PWMS)
            if self._estop_latched:
                # A latched ESTOP outranks every other transition: a socket
                # error arriving after an emergency break must not make the
                # dashboard look safe again.
                return
            if self.connected and self.sock is not None:
                self.control_state = ControlState.STOPPED
            else:
                self.control_state = ControlState.DISCONNECTED

    def start_control_loop(self) -> bool:
        """
        Start the 20 Hz resend worker, at most once per transport.

        Returns True when a worker is running (including one this call started
        earlier) and False when the transport is down or ESTOP is latched.
        """
        with self.lock:
            if self._estop_latched:
                return False
            if not self.connected or self.sock is None:
                return False
            existing = self._control_thread
            if existing is not None and existing.is_alive():
                return True
            # A fresh event per generation: if a previous worker ever failed to
            # join, clearing the shared event would resurrect its stop signal.
            stop_event = threading.Event()
            self._control_stop = stop_event
            self.control_state = ControlState.RUNNING
            thread = threading.Thread(
                target=self._control_loop,
                args=(stop_event,),
                name="sil-dashboard-control",
                daemon=True,
            )
            self._control_thread = thread
        thread.start()
        return True

    def stop_control_loop(self, send_neutral: bool = True) -> bool:
        """
        Signal the worker, join it with a bounded timeout, and force neutral.

        Idempotent, and never writes a frame once the transport is down.
        Returns False only if a worker refused to stop within the join timeout,
        which is a test failure, not a normal outcome.
        """
        thread = self._control_thread
        self._control_stop.set()
        joined = True
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=CONTROL_JOIN_TIMEOUT_S)
            joined = not thread.is_alive()
        if joined:
            self._control_thread = None
        # _transition_stopped also sets _control_stop and clears the command; it
        # deliberately does not join, so the worker may call it on its own way out.
        self._transition_stopped(send_neutral=send_neutral)
        return joined

    def request_stop(self) -> bool:
        """Operator STOP: cancel the held command and drive the outputs neutral."""
        return self.stop_control_loop(send_neutral=True)

    def _control_loop(self, stop_event: threading.Event) -> None:
        """
        Resend the last accepted command every CONTROL_PERIOD_S until stopped.

        Pacing is a monotonic deadline, not ``Event.wait(0.05)``. A bare wait
        pays the send duration twice on some ticks and not at all on others, so
        the cadence becomes bursty: a run of over-long gaps followed by a
        catch-up tick with a near-zero gap. Measured with a 12 ms send cost, a
        bare wait spans 12-78 ms per gap while this loop holds 46-64 ms. The
        stop event is only ever used to end the wait early.
        """
        next_tick = time.monotonic()
        while not stop_event.is_set():
            with self.lock:
                if self.control_state is not ControlState.RUNNING:
                    return
                pwms = list(self._last_command)

            if not self.connected or self.sock is None:
                # The transport died under us: stop transmitting immediately and
                # let the firmware watchdog force neutral.
                with self.lock:
                    if not self._estop_latched:
                        self.control_state = ControlState.DISCONNECTED
                return

            try:
                self._send_thruster_frame(pwms)
            except OSError:
                # The socket is already broken, so a neutral frame cannot be
                # delivered; retrying would only spin against a dead transport.
                self.connected = False
                self._transition_stopped(send_neutral=False)
                return
            except Exception:
                # A non-transport fault (a bug, not a dead socket) leaves the
                # transport usable, so the neutral frame must really go out.
                self._transition_stopped(send_neutral=True)
                return

            next_tick += CONTROL_PERIOD_S
            remaining = next_tick - time.monotonic()
            if remaining > 0.0:
                if stop_event.wait(remaining):
                    return
            else:
                # A send overran its own period. Resync instead of firing a
                # catch-up burst; the next deadline is a full period out, so the
                # watchdog is fed as fast as the transport allows without
                # emitting a pair of back-to-back command frames.
                next_tick = time.monotonic()

    def send_pwms(self, pwms: List[int]) -> bool:
        """
        Accept a thruster command, transmit it now, and hold it at 20 Hz.

        Compatibility entry point: the Streamlit UI still calls this. It is no
        longer a one-shot; the value becomes the worker's held command until
        request_stop(), an emergency break, a transport error, or disconnect.
        """
        validated = _validate_pwms(pwms)
        with self.lock:
            latched = self._estop_latched
            transport_up = self.connected and self.sock is not None

        if latched:
            # A refused command must not be able to overwrite the held command
            # or the dashboard readback: ESTOP has to look and behave neutral.
            return False

        if not transport_up:
            # No transport, so nothing can be transmitted. Mirror the value for
            # the UI only, and leave the worker's command neutral so a later
            # connect() can never resurrect a stale non-neutral command.
            with self.lock:
                self.pwms = list(validated)
                payload = ThrusterCommand(pwm_us=self.pwms).pack()
                self.pipeline_core_can_hex = " ".join(f"{b:02X}" for b in payload)
                self.control_state = ControlState.DISCONNECTED
            return False

        with self.lock:
            self.pwms = list(validated)
            self._last_command = list(validated)
            payload = ThrusterCommand(pwm_us=self.pwms).pack()
            self.pipeline_core_can_hex = " ".join(f"{b:02X}" for b in payload)

        self.control_state = ControlState.RUNNING
        if not self.start_control_loop():
            return False
        try:
            # Send the operator's command straight away so UI response is not
            # quantized to the next 50 ms tick; the worker sustains it after this.
            self._send_thruster_frame(list(validated))
        except OSError:
            self.connected = False
            self._transition_stopped(send_neutral=False)
            return False
        return True

    def request_emergency_break(self) -> bool:
        """
        Trip the authorized emergency break and latch ESTOP.

        Node 2 only accepts 0xAA 0x55 (PR #46), so those bytes are sent exactly
        as the firmware requires. The state latches until connect() or
        start_server_process(): a latched ESTOP refuses every later command.
        """
        self._estop_latched = True
        # Stop the worker without a neutral frame first: the emergency frame is
        # the higher-priority message and must be the next thing on the wire.
        self.stop_control_loop(send_neutral=False)
        with self.lock:
            self.control_state = ControlState.ESTOP
            self._last_command = list(NEUTRAL_PWMS)
        if not self.connected or self.sock is None:
            return False
        try:
            self._send_frame(
                "Core -> STM32",
                CAN_ID_EMERGENCY_BREAK,
                EMERGENCY_BREAK_SIGNATURE,
                "EMERGENCY CUTOFF TRIGGERED (Authorized)",
            )
            self.emergency_break_requested = True
        except OSError:
            self.connected = False
            return False
        # The board forces neutral the instant it latches; sending the neutral
        # thruster frame as well keeps the dashboard readback and the packet log
        # consistent with what the firmware is doing.
        try:
            self._send_thruster_frame(list(NEUTRAL_PWMS))
        except OSError:
            self.connected = False
            return False
        return True

    def trigger_emergency_break(self) -> bool:
        """
        Backwards-compatible alias for request_emergency_break().

        Kept because the Streamlit UI and existing operator runbooks call this
        name; the authorized 0xAA 0x55 0x01 signature is unchanged.
        """
        return self.request_emergency_break()

    def send_solenoids(self, mask: int) -> bool:
        with self.lock:
            self.solenoid_mask = mask & 0x03FF
            solenoid_mask = self.solenoid_mask
        if not self.connected or self.sock is None:
            return False
        cmd = SolenoidCommand(solenoid_mask=solenoid_mask)
        payload = cmd.pack()
        try:
            self._send_frame(
                "Core -> STM32", CAN_ID_SOLENOID_CMD, payload, f"Bitmask: 0x{solenoid_mask:04X}"
            )
        except OSError:
            self.connected = False
            return False
        return True

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
                    self.last_rx_monotonic[can_id] = time.monotonic()
                    summary = ""

                    if can_id == CAN_ID_SIL_OUTPUT_STATUS:
                        actual_pwms, brake_active, actual_solenoids, sim_time_ms = unpack_sil_output_status(payload)
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
                    elif can_id == CAN_ID_POWER_TELEMETRY:
                        pwr = PowerTelemetry.unpack(payload)
                        self.power_data = pwr
                        summary = f"48V Rail: {pwr.tether_voltage_mv/1000:.1f}V @ {pwr.tether_current_ma/1000:.1f}A, Temp: {pwr.pcb_temp_c_tenths/10:.1f}C"

                    elif can_id == CAN_ID_EMERGENCY_BREAK:
                        summary = "EMERGENCY BREAK LATCHED"

                    self._record_packet("STM32 -> Core", can_id, payload, summary)

            except Exception:
                break
        self.connected = False
