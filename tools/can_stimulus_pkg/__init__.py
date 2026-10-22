"""
CAN stimulus package for X19 ROV.
"""

from __future__ import annotations

import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = TOOLS_DIR.parent
SIL_BRIDGE_DIR = REPO_ROOT / "tests" / "sil_companion_bridge"
SIL_STIMULUS_DIR = REPO_ROOT / "tests" / "sil_stimulus"
SIL_DASHBOARD_DIR = REPO_ROOT / "tests" / "sil_dashboard"
for _path in (SIL_STIMULUS_DIR, SIL_BRIDGE_DIR, SIL_DASHBOARD_DIR):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from sil_protocol import (
    CAN_ID_EFUSE_FAULT_ALERT,
    CAN_ID_EMERGENCY_BREAK,
    CAN_ID_ENV_TELEMETRY,
    CAN_ID_NAV_TELEMETRY,
    CAN_ID_POWER_TELEMETRY,
    CAN_ID_SIL_OUTPUT_STATUS,
    CAN_ID_SOLENOID_CMD,
    CAN_ID_THRUSTER_CMD,
    CAN_ID_TIME_SYNC_MASTER,
    CAN_ID_TIME_SYNC_REQ,
    CAN_ID_TIME_SYNC_RESP,
    CAN_ID_USB_HUB_TELEMETRY,
    SIL_PACKET_SIZE,
    EnvTelemetry,
    NavTelemetry,
    PowerTelemetry,
    SolenoidCommand,
    ThrusterCommand,
    pack_sil_can_frame,
    unpack_sil_can_frame,
    unpack_sil_output_status,
)
from sil_dashboard_client import conflicting_solenoid_valve
from sil_test_support import (
    SERVER_START_TIMEOUT_S,
    SERVER_STEM,
    require_server_executable,
)

from .backends import (
    ForeignEngineError,
    PythonCanBackend,
    SilSocketBackend,
    UartBackend,
    _BaseBackend,
    _optional_module_available,
    _require_optional_module,
    build_backend,
)
from .cli import (
    ACTION_FLAGS,
    build_parser,
    print_report,
    run_selected_action,
    selected_action_flags,
    target_label,
)
from .constants import (
    COMMAND_PERIOD_S,
    DEFAULT_RECV_TIMEOUT_S,
    DEFAULT_SIL_HOST,
    DEFAULT_SIL_PORT,
    EMERGENCY_FRAME,
    EMERGENCY_SIGNATURE,
    EMERGENCY_TIMEOUT_S,
    ESC_ARMING_SIM_MS,
    ESTOP_LATCH_WINDOW_S,
    FIRST_OUTPUT_TIMEOUT_S,
    HEARTBEAT_TIMEOUT_S,
    MAX_SIL_PAYLOAD,
    MIN_ARMING_SAMPLES,
    MIN_TELEMETRY_HZ,
    NEUTRAL_PWMS,
    NEUTRAL_TIMEOUT_S,
    NEUTRAL_US,
    NUM_THRUSTERS,
    POST_ESTOP_PROBE_PWM_US,
    POWER_TELEMETRY_SIM_PERIOD_MS,
    PWM_MAX_US,
    PWM_MIN_US,
    PWM_SLEW_RATE_US_PER_MS,
    RAMP_TIMEOUT_S,
    READ_POLL_S,
    SAFETY_SIGNATURES,
    SERVER_SHUTDOWN_TIMEOUT_S,
    SIM_MS_PER_WALL_S,
    SIM_WALL_SLACK_S,
    SMOKE_ALL_PWM_US,
    SMOKE_SINGLE_PWM_US,
    SMOKE_SOLENOID_MASK,
    SOCKET_TIMEOUT_S,
    SOLENOID_MASK_BITS,
    SOLENOID_MASK_MAX,
    SOLENOID_OBSERVE_TIMEOUT_S,
    SOLENOID_VALVES,
    UNAUTHORIZED_FRAME,
    UNAUTHORIZED_FRAME_WINDOW_S,
)
from .models import (
    CheckResult,
    OutputStatus,
    StimulusReport,
    energised_coils,
    format_result,
)
from .protocol import (
    _raw_frame,
    _validate_channel,
    _validate_pulse_us,
    describe_can_id,
    int_literal,
    validate_can_id,
    validate_payload,
)
from .tester import VehicleStimulusTester

__all__ = [
    "CAN_ID_EFUSE_FAULT_ALERT",
    "CAN_ID_EMERGENCY_BREAK",
    "CAN_ID_ENV_TELEMETRY",
    "CAN_ID_NAV_TELEMETRY",
    "CAN_ID_POWER_TELEMETRY",
    "CAN_ID_SIL_OUTPUT_STATUS",
    "CAN_ID_SOLENOID_CMD",
    "CAN_ID_THRUSTER_CMD",
    "CAN_ID_TIME_SYNC_MASTER",
    "CAN_ID_TIME_SYNC_REQ",
    "CAN_ID_TIME_SYNC_RESP",
    "CAN_ID_USB_HUB_TELEMETRY",
    "ESC_ARMING_SIM_MS",
    "ForeignEngineError",
    "MIN_ARMING_SAMPLES",
    "NEUTRAL_PWMS",
    "NEUTRAL_US",
    "PWM_MAX_US",
    "PWM_MIN_US",
    "SIL_PACKET_SIZE",
    "CheckResult",
    "OutputStatus",
    "PythonCanBackend",
    "SilSocketBackend",
    "StimulusReport",
    "UartBackend",
    "VehicleStimulusTester",
    "build_backend",
    "build_parser",
    "conflicting_solenoid_valve",
    "describe_can_id",
    "energised_coils",
    "format_result",
    "int_literal",
    "pack_sil_can_frame",
    "print_report",
    "run_selected_action",
    "selected_action_flags",
    "unpack_sil_can_frame",
    "unpack_sil_output_status",
    "validate_can_id",
    "validate_payload",
]
