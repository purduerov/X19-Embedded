#!/usr/bin/env python3
"""
Purdue ROV X19 fake-CAN stimulus tool (canonical, single implementation).

This is the one maintained way to inject stimulus at the vehicle CAN boundary
and to read the result back.  It replaces two divergent prototype scripts whose
``--auto`` runs were worse than useless: ``tools/can_stimulus.py`` exited 0
while raising ``PowerTelemetry object has no attribute 'tether_v'`` and
``tools/node2_stimulus.py`` crashed on ``AttributeError: 'Namespace' object has
no attribute 'port'``.  A tool that reports success while its own checks crash
teaches the operator to ignore it, so the following contract is load bearing
and is pinned by ``tests/sil_stimulus/test_stimulus_flows.py``:

* every check returns a ``CheckResult``; nothing returns a bare bool;
* an exception raised inside a check becomes a FAILED ``CheckResult`` carrying
  the exception text (``VehicleStimulusTester.guard``) - it is never allowed to
  propagate as a traceback and never swallowed;
* ``StimulusReport.ok`` is False when any check failed *and* when no check ran;
* ``main()`` returns 0 only when ``report.ok``; there is no unconditional
  ``[PASS]`` line anywhere;
* every wait is bounded.  A missing, malformed, stale, or wrong observation is a
  failed check, never a hang and never a pass.

Wire format
-----------
The SIL packet layout is defined exactly once, in
``tests/sil_companion_bridge/sil_protocol.py``, and is imported here - there are
no fallback packet classes and no re-derived ``struct`` formats.  Framing and
server discovery are reused from ``tests/sil_stimulus/sil_test_support.py``
(Task 1) so the tool and the tests can never disagree about the wire.

``sys.path`` handling
---------------------
``sil_protocol`` and ``sil_test_support`` are not installed packages: they live
in the test tree, which is the single source of truth for the framing.  They are
therefore reached by absolute path, computed from ``__file__`` rather than from
the current working directory, so the module imports identically whether it is
run as ``python tools/can_stimulus.py`` or imported as ``from tools import
can_stimulus`` (which is how the tests use it).  The import is deliberately not
wrapped in ``try/except ImportError``: a missing or moved ``sil_protocol`` is a
hard error that must be visible, not something to paper over with a second,
possibly divergent, definition of the packet format.

Third-party hardware adapters (``PythonCanBackend``, ``UartBackend``) import
their dependency lazily and raise a clear ``RuntimeError`` when it is missing, so
importing this module on a machine without python-can or pyserial is fine.
"""

from __future__ import annotations

import argparse
import contextlib
import importlib.util
import math
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Callable, Dict, List, NamedTuple, Optional, Sequence, Tuple

# --------------------------------------------------------------------------
# Path setup. Must precede the protocol import: the SIL packet format and the
# shared SIL fixtures are test-tree modules, not installed packages.
# --------------------------------------------------------------------------
TOOLS_DIR = Path(__file__).resolve().parent
REPO_ROOT = TOOLS_DIR.parent
SIL_BRIDGE_DIR = REPO_ROOT / "tests" / "sil_companion_bridge"
SIL_STIMULUS_DIR = REPO_ROOT / "tests" / "sil_stimulus"
SIL_DASHBOARD_DIR = REPO_ROOT / "tests" / "sil_dashboard"
for _path in (SIL_STIMULUS_DIR, SIL_BRIDGE_DIR, SIL_DASHBOARD_DIR):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

# CAN IDs, packet sizes, and the output-status decoder come from the one place
# they are defined. Re-exported so callers need only this module.
from sil_protocol import (  # noqa: E402  (path setup must precede the import)
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

from sil_test_support import (  # noqa: E402  (path setup must precede the import)
    SERVER_START_TIMEOUT_S,
    SERVER_STEM,
    recv_frames,
    require_server_executable,  # noqa: F401  (re-exported for callers of this tool)
    server_executable,
)

# The solenoid interlock is a firmware rule with exactly one definition. It lives
# with sil_dashboard_client.send_solenoids, the other code path that packs 0x110,
# so this tool and the dashboard cannot disagree about which masks the board
# applies - which is exactly the drift that let the UI build a mask (0x007) this
# tool refuses. tools/can_stimulus.py -> tests/sil_dashboard/ is a test-tree
# import, reached by absolute path for the same reason the two above are.
from sil_dashboard_client import conflicting_solenoid_valve  # noqa: E402,F401

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
    "main",
    "pack_sil_can_frame",
    "print_report",
    "run_selected_action",
    "selected_action_flags",
    "unpack_sil_can_frame",
    "unpack_sil_output_status",
    "validate_can_id",
    "validate_payload",
]

# --------------------------------------------------------------------------
# Firmware constants.
#
# These are mirrors of C #defines and of two decisions that live in app.c. They
# are named after their firmware source so a check can cite what it relies on;
# the authority is always the C file, never this file.
# --------------------------------------------------------------------------
# shared/include/rov_parameters.h:37  ROV_NUM_THRUSTERS (8)
NUM_THRUSTERS = 8
# shared/include/rov_parameters.h:38  ROV_PWM_STOP_US (1500) - the neutral value
NEUTRAL_US = 1500
# shared/include/rov_parameters.h:39-40  ROV_PWM_MIN_US / ROV_PWM_MAX_US
PWM_MIN_US = 1000
PWM_MAX_US = 2000
NEUTRAL_PWMS: Tuple[int, ...] = (NEUTRAL_US,) * NUM_THRUSTERS
# shared/include/rov_parameters.h:71  ROV_HEARTBEAT_TIMEOUT_MS (100)
HEARTBEAT_TIMEOUT_S = 0.100
# A command is resent at half the watchdog budget, so a single slow tick cannot
# expire the heartbeat. 20 Hz, the same cadence the dashboard worker uses.
COMMAND_PERIOD_S = HEARTBEAT_TIMEOUT_S / 2.0
# nodes/node2_control_board/Core/Src/app.c:24  ESC_ARMING_TIME_MS (3000)
ESC_ARMING_SIM_MS = 3000
# The arming gate must be observed repeatedly while it is closed. One snapshot
# at or after the boundary would leave the intruder list empty and make an
# unwatched window look verified; a real run produces ~150 inside it.
MIN_ARMING_SAMPLES = 3
# The C engine advances 10 ms of virtual time per 10 ms wall tick
# (tests/sil_bridge_server.c:264,427). 0.8 is a deliberately generous lower bound
# on that rate, so a slow host widens a wall-clock budget instead of tripping a
# sim-clock deadline.  It is a BUDGET input only: it is not a measurement, so it
# must never be turned into a number a report claims to have observed.  Rates are
# measured against the engine's own clock (see MIN_TELEMETRY_HZ).
SIM_MS_PER_WALL_S = 0.8
SIM_WALL_SLACK_S = 6.0
# shared/include/rov_parameters.h:53-54  ROV_NUM_SOLENOIDS (5) valves,
# ROV_NUM_SOLENOID_CHANNELS (10) discrete coils - two opposing coils per valve.
SOLENOID_VALVES = 5
SOLENOID_MASK_BITS = 10
SOLENOID_MASK_MAX = (1 << SOLENOID_MASK_BITS) - 1
# nodes/node2_control_board/Core/Src/app.c:112  authorized emergency signature
EMERGENCY_SIGNATURE = b"\xAA\x55"
# The exact authorized emergency payload. Third byte is the leak bitmask, 0x01
# meaning "a leak was asserted" (nodes/node1_pi_shield/Core/Src/app.c:78).
EMERGENCY_FRAME = b"\xAA\x55\x01"
# Same frame with one signature byte wrong. Firmware must ignore it
# (app.c:112 rejects anything that is not 0xAA 0x55).
UNAUTHORIZED_FRAME = b"\xAA\x56\x01"
# eFuse alert authorization signature (node2 app.c:121). There is deliberately no
# automatic eFuse check: tripping that path force-neutrals the board and latches
# a 0x005 alert for the life of the engine, exactly like the emergency break, so
# it belongs in the operator's hands via --raw-send. raw_send() still refuses an
# unsigned 0x005 so a mis-typed frame cannot masquerade as a real alert, and the
# eFuse's *effect* is asserted for real by monitor_node3_power's fault-bit check.
EFUSE_SIGNATURE = b"\xEF\x01"
# Safety-critical IDs the firmware only acts on when the payload carries the
# authorization signature. Enforced at the point of injection.
SAFETY_SIGNATURES = {
    CAN_ID_EMERGENCY_BREAK: EMERGENCY_SIGNATURE,
    CAN_ID_EFUSE_FAULT_ALERT: EFUSE_SIGNATURE,
}
# shared/include/rov_parameters.h:44-45  ROV_PWM_MAX_SLEW_RATE_US_PER_MS (2).
# 1000 us of travel therefore needs 500 ms of sim time.
PWM_SLEW_RATE_US_PER_MS = 2
# sil_protocol.SIL_PACKET_FMT carries a 64-byte data field.
MAX_SIL_PAYLOAD = 64

DEFAULT_SIL_HOST = "127.0.0.1"
DEFAULT_SIL_PORT = 8765  # tests/sil_bridge_server.c:61
DEFAULT_RECV_TIMEOUT_S = 0.5
SOCKET_TIMEOUT_S = 2.0
SERVER_SHUTDOWN_TIMEOUT_S = 2.0

# Every wait below is bounded; the constants are the bounds.
FIRST_OUTPUT_TIMEOUT_S = 3.0
READ_POLL_S = 0.02
# PWM reaches its target through a 2 us/ms slew ramp, so allow 1000 us of travel
# plus generous slack, expressed in wall time.
RAMP_TIMEOUT_S = (PWM_MAX_US - NEUTRAL_US) / PWM_SLEW_RATE_US_PER_MS / 1000.0 / SIM_MS_PER_WALL_S + SIM_WALL_SLACK_S
NEUTRAL_TIMEOUT_S = RAMP_TIMEOUT_S
SOLENOID_OBSERVE_TIMEOUT_S = 1.5
UNAUTHORIZED_FRAME_WINDOW_S = 0.40
ESTOP_LATCH_WINDOW_S = 0.50
EMERGENCY_TIMEOUT_S = 3.0
# Liveness floor for a telemetry stream, in the stream's OWN clock. Node 2 is
# specified for 100 Hz simulated (rov_parameters.h:66) and the SIL server forwards
# it unthrottled, so a fifth of the design rate is already a broken stream. It is
# deliberately a floor on evidence, not a claim that the design rate was met.
MIN_TELEMETRY_HZ = 20.0

# Defaults for the combined smoke run. The emergency break is last because it
# latches for the life of the engine (rov_safety.c:37-40 has no clear path, and
# app.c:116 sets ESC_STATE_DISARMED), so anything after it could only observe a
# permanently disarmed board.
SMOKE_SINGLE_PWM_US = 1700
SMOKE_ALL_PWM_US = 1650
# Valve 0 coil A only. Valid: bits 0 and 1 are the two opposing coils of valve 0
# and exactly one of them is energised, so the interlock at
# shared/src/rov_can_protocol.c:120-126 is satisfied.
SMOKE_SOLENOID_MASK = 0x0001
POST_ESTOP_PROBE_PWM_US = 1800


class OutputStatus(NamedTuple):
    """
    A decoded 0x7FE snapshot of the mocked BSP outputs plus the sim clock.

    Decoded by ``sil_protocol.unpack_sil_output_status``; this is a view over an
    already-decoded tuple, not a second definition of the wire format.
    """

    pwms: Tuple[int, ...]
    brake_active: bool
    solenoids: int
    sim_time_ms: int

    @classmethod
    def decode(cls, payload: bytes) -> "OutputStatus":
        pwms, brake_active, solenoids, sim_time_ms = unpack_sil_output_status(payload)
        return cls(tuple(pwms), brake_active, solenoids, sim_time_ms)

    def describe(self) -> str:
        brake = "BRAKE" if self.brake_active else "brake-off"
        return (
            f"sim={self.sim_time_ms}ms pwms={list(self.pwms)} {brake} "
            f"solenoids=0x{self.solenoids:04X}"
        )


class CheckResult:
    """One named observation with its verdict and the evidence behind it."""

    __slots__ = ("name", "passed", "detail")

    def __init__(self, name: str, passed: bool, detail: str = "") -> None:
        self.name = name
        self.passed = bool(passed)
        self.detail = detail

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"CheckResult(name={self.name!r}, passed={self.passed!r}, detail={self.detail!r})"

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, CheckResult):
            return NotImplemented
        return (self.name, self.passed, self.detail) == (other.name, other.passed, other.detail)


class StimulusReport:
    """
    The aggregate verdict.

    ``ok`` is False when any check failed *and* when no check ran at all: an
    empty report is an unverified vehicle, not a verified one.  This is the
    silent-success hole that let the prototypes exit 0.
    """

    __slots__ = ("results",)

    def __init__(self, results: Sequence[CheckResult]) -> None:
        self.results: List[CheckResult] = list(results)

    @property
    def ok(self) -> bool:
        return bool(self.results) and all(result.passed for result in self.results)

    @property
    def failures(self) -> List[CheckResult]:
        return [result for result in self.results if not result.passed]

    @property
    def passed(self) -> List[CheckResult]:
        return [result for result in self.results if result.passed]

    def format(self, include_checks: bool = True) -> str:
        """
        Render the report. The failure block repeats the failed checks by name
        with their detail on purpose: the per-check lines are chronological and a
        long run buries them, and the operator reading only the tail must still
        see which check failed and why.

        ``include_checks=False`` omits the per-check lines, for a caller that has
        already streamed them as they completed.
        """
        lines = []
        if include_checks:
            for result in self.results:
                lines.append(format_result(result))
        lines.append(
            f"Summary: {len(self.results)} check(s), {len(self.passed)} passed, "
            f"{len(self.failures)} failed"
        )
        if self.failures:
            names = ", ".join(failure.name for failure in self.failures)
            lines.append(f"[FAIL] {len(self.failures)} check(s) failed: {names}")
            for failure in self.failures:
                lines.append(f"[FAIL] {failure.name} failed: {failure.detail}")
        return "\n".join(lines)

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"StimulusReport({self.results!r}, ok={self.ok!r})"


def format_result(result: CheckResult) -> str:
    """One line per check. Used both by the live stream and by the final report."""
    return f"[{'PASS' if result.passed else 'FAIL'}] {result.name}: {result.detail}"


def energised_coils(mask: int) -> List[str]:
    """
    Name every coil a mask energises, e.g. 0x0001 -> ``['valve 0 coil A']``.

    Used to make a solenoid pass message describe the mask that was actually
    sent instead of asserting something generic about it.
    """
    return [
        f"valve {valve} coil {'AB'[coil]}"
        for valve in range(SOLENOID_VALVES)
        for coil in (0, 1)
        if mask & (1 << (2 * valve + coil))
    ]


# --------------------------------------------------------------------------
# Transports
# --------------------------------------------------------------------------


def validate_payload(payload: bytes) -> bytes:
    """Return ``payload`` as bytes, rejecting anything the packet cannot carry."""
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise TypeError(f"CAN payload must be bytes, got {type(payload).__name__}")
    payload = bytes(payload)
    if len(payload) > MAX_SIL_PAYLOAD:
        raise ValueError(
            f"SIL frame payload is {len(payload)} bytes; the packet carries at most "
            f"{MAX_SIL_PAYLOAD} (sil_protocol.SIL_PACKET_FMT)"
        )
    return payload


def validate_can_id(can_id: int) -> int:
    """Return ``can_id`` after checking it is a plausible arbitration id."""
    if not isinstance(can_id, int) or isinstance(can_id, bool):
        raise TypeError(f"CAN id must be an int, got {type(can_id).__name__}")
    if not 0 <= can_id <= 0x1FFFFFFF:
        raise ValueError(f"CAN id 0x{can_id:X} is out of range")
    return can_id


def describe_can_id(can_id: int) -> str:
    """Render an arbitration id the way every message in this tool does."""
    return f"CAN 0x{can_id:03X}"


class _BaseBackend:
    """
    Shared payload validation and capability declaration for every transport.

    ``provides_output_readback`` is False by default on purpose.  0x7FE is a
    SIL-only mock-BSP channel (``sil_protocol.py:22``,
    ``tests/sil_bridge_server.c:62``), so a transport that cannot supply it must
    not be asked to verify an actuation.  A check consults this *before* it
    writes anything, which is what stops a blind actuation leaving the bus.

    A transport that forgets to declare the capability therefore fails safe.
    """

    provides_output_readback = False

    def latest_outputs(self) -> Optional[OutputStatus]:
        raise NotImplementedError

    def set_frame_listener(self, listener):
        raise NotImplementedError

    def send_frame(self, can_id: int, payload: bytes) -> None:
        raise NotImplementedError

    def recv_frame(self, timeout_s: float = DEFAULT_RECV_TIMEOUT_S):
        raise NotImplementedError

    def describe_destination(self, can_id: int) -> str:
        """How a frame addressed to ``can_id`` actually leaves this transport."""
        return describe_can_id(can_id)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()


class ForeignEngineError(ConnectionError):
    """
    A SIL engine this backend did not start is already listening on the port.

    Raised by :meth:`SilSocketBackend.connect` before anything is written.  It is a
    distinct type, not a bare ``ConnectionError``, because "the port was busy" and
    "that engine belongs to somebody else" call for different operator responses:
    the first is a retry, the second is a decision.
    """


class SilSocketBackend(_BaseBackend):
    """
    SIL transport: a TCP connection to ``sil_bridge_server``.

    Responsibilities, all of which the tests pin:

    * resolve the server through ``X19_SIL_SERVER`` and then the build trees
      (``sil_test_support.server_executable``);
    * connect to an already-running server if one is listening, and only launch
      a child when nothing answers *and* auto_start is on *and* the mode is SIL;
    * refuse an engine it did not start, unless ``attach_existing`` says the
      caller meant it (:class:`ForeignEngineError`);
    * refuse a payload over 64 bytes with ``ValueError`` before touching the wire;
    * buffer partial TCP reads across calls (delegated to
      ``sil_test_support.recv_frames``, whose buffer is keyed per socket);
    * decode to ``(can_id, payload)`` pairs;
    * track the latest 0x7FE output-status readback in ``latest_outputs()``;
    * terminate the child in ``close()`` on every path, including exceptions.

    A malformed packet poisons the stream: ``recv_frames`` deliberately leaves the
    offending bytes in place with no resynchronisation scan, so the framing offset
    is no longer known.  The failure is reported as a check failure; recovery
    requires closing and reconnecting, which is exactly what a fresh engine
    connection does.

    This is the only transport that can supply the 0x7FE readback, so it is the
    only one on which an actuation command can be verified.
    """

    provides_output_readback = True

    def __init__(
        self,
        host: str = DEFAULT_SIL_HOST,
        port: int = DEFAULT_SIL_PORT,
        auto_start: bool = True,
        mode: str = "sil",
        attach_existing: bool = False,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.auto_start = bool(auto_start)
        self.mode = mode
        # Deliberate attachment to an engine somebody else started. False by
        # default, and it has to be: see connect() and ForeignEngineError.
        self.attach_existing = bool(attach_existing)
        self._sock: Optional[socket.socket] = None
        self._process: Optional[subprocess.Popen] = None
        self._pending: List[Tuple[int, bytes]] = []
        self._latest: Optional[OutputStatus] = None
        self._listener: Optional[Callable[[int, bytes], None]] = None

    # -- lifecycle ---------------------------------------------------------
    @property
    def connected(self) -> bool:
        return self._sock is not None

    @property
    def started_server(self) -> bool:
        """True when this backend owns a child ``sil_bridge_server`` process."""
        return self._process is not None

    def connect(self, timeout_s: float = SERVER_START_TIMEOUT_S) -> None:
        if self.mode != "sil":
            raise RuntimeError(
                f"the SIL TCP transport only speaks the SIL framing; mode={self.mode!r} must use "
                "--mode can or --mode uart instead"
            )
        if self._sock is not None:
            return
        if self._try_connect(min(1.0, max(0.05, timeout_s))):
            self._require_ownership()
            return
        if not self.auto_start:
            raise ConnectionError(
                f"no SIL bridge server is listening on {self.host}:{self.port} and auto-start is "
                "disabled, so there is nothing to stimulate"
            )
        try:
            self._start_server()
            if not self._try_connect(timeout_s):
                raise ConnectionError(
                    f"SIL bridge server on {self.host}:{self.port} did not accept a connection "
                    f"within {timeout_s:.1f} s"
                )
        except Exception:
            # Never leave a half-started engine behind: main() only calls close()
            # on a backend that was successfully constructed, so the cleanup has
            # to happen here too.
            self.close()
            raise

    def _require_ownership(self) -> None:
        """
        Refuse a connection to an engine this backend did not start.

        Silently reusing whatever answers is the same defect class as writing a
        frame that cannot be read back: a run that reports success about an engine
        it commandeered.  ``--emergency-break`` against somebody else's engine
        drives all eight thrusters to 1800 us and latches *their* emergency brake
        for the life of that engine, and the dashboard auto-starts an engine on
        the default port on every render, so the documented commands reach this
        path while a dashboard is open.

        Ownership is therefore a precondition, checked before anything is written
        for the same reason ``_require_output_readback`` is: a refusal that
        arrives after the actuation is a refusal that is too late.  ``attach_existing``
        is the explicit opt-in, for a caller that really did start the engine
        (Task 5's integration tests, through ``SilServerProcess``) or that means
        to share one on purpose.
        """
        if self.attach_existing or self.started_server:
            return
        sock, self._sock = self._sock, None
        if sock is not None:
            with contextlib.suppress(OSError):
                sock.close()
        raise ForeignEngineError(
            f"refused before sending any frame: something is already listening on "
            f"{self.host}:{self.port} and this run did not start it, so the checks would drive "
            "an engine this tool does not own. --emergency-break alone latches that engine's "
            "brake for the life of it, and the dashboard starts an engine on the default port "
            "on every render, so this is reachable while someone else is using the vehicle. "
            "To use your own engine, pass --port with a port nothing is listening on and the "
            f"tool will launch and reap one. To deliberately attach to the engine on "
            f"{self.host}:{self.port}, pass --attach-existing and select the individual checks "
            "you want; --auto is refused with it, because there is no harmless whole-vehicle "
            "run against somebody else's engine."
        )

    def _try_connect(self, timeout_s: float) -> bool:
        try:
            self._sock = socket.create_connection((self.host, self.port), timeout=timeout_s)
        except OSError:
            self._sock = None
            return False
        self._sock.settimeout(SOCKET_TIMEOUT_S)
        self._pending.clear()
        self._latest = None
        return True

    def _port_is_listening(self, timeout_s: float) -> bool:
        """Probe the port and drop the probe, so exactly one socket survives."""
        try:
            probe = socket.create_connection((self.host, self.port), timeout=timeout_s)
        except OSError:
            return False
        probe.close()
        return True

    def _start_server(self) -> None:
        executable = server_executable()
        if executable is None:
            raise ConnectionError(
                f"no {SERVER_STEM} binary was found to start, so the SIL engine cannot be launched. "
                "Build it with 'cmake -B build-native -G Ninja' then "
                f"'cmake --build build-native --target {SERVER_STEM}', or point X19_SIL_SERVER at an "
                "existing binary."
            )
        self._process = subprocess.Popen(
            [str(executable), "--port", str(self.port), "--cycles", "0"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        deadline = time.monotonic() + SERVER_START_TIMEOUT_S
        while time.monotonic() < deadline:
            if self._process.poll() is not None:
                raise ConnectionError(
                    f"{SERVER_STEM} exited with code {self._process.returncode} before listening on "
                    f"{self.host}:{self.port}; the port is probably already in use"
                )
            if self._port_is_listening(0.1):
                return
        raise ConnectionError(
            f"{SERVER_STEM} did not listen on {self.host}:{self.port} within "
            f"{SERVER_START_TIMEOUT_S:.1f} s"
        )

    def close(self) -> None:
        """Close the socket and terminate the child engine. Safe to call twice."""
        sock, self._sock = self._sock, None
        if sock is not None:
            with contextlib.suppress(OSError):
                sock.close()
        process, self._process = self._process, None
        if process is not None and process.poll() is None:
            with contextlib.suppress(OSError, subprocess.TimeoutExpired):
                process.terminate()
                process.wait(timeout=SERVER_SHUTDOWN_TIMEOUT_S)
            if process.poll() is None:  # pragma: no cover - terminate always wins on CPython
                with contextlib.suppress(OSError, subprocess.TimeoutExpired):
                    process.kill()
                    process.wait(timeout=SERVER_SHUTDOWN_TIMEOUT_S)
        self._pending.clear()

    # -- framing -----------------------------------------------------------
    def set_frame_listener(self, listener):
        """Install a callback invoked for every non-0x7FE frame decoded."""
        previous, self._listener = self._listener, listener
        return previous

    def send_frame(self, can_id: int, payload: bytes) -> None:
        payload = validate_payload(payload)
        can_id = validate_can_id(can_id)
        if self._sock is None:
            raise ConnectionError("the SIL transport is not connected; call connect() first")
        self._sock.sendall(pack_sil_can_frame(can_id, payload))

    def recv_frame(self, timeout_s: float = DEFAULT_RECV_TIMEOUT_S) -> Optional[Tuple[int, bytes]]:
        """
        Return the next decoded ``(can_id, payload)``, or None on timeout.

        0x7FE snapshots are consumed into ``latest_outputs()`` and are not
        returned: that channel is a SIL-only mock-BSP readout, so a caller
        watching outputs and a caller watching telemetry do not have to filter.
        A 0x7FE does not reset the budget, so the call stays bounded by
        ``timeout_s`` however many snapshots it swallows.

        :meth:`drain_frames` obeys the same rule by default, so the two entry
        points cannot drift apart; pass ``include_output_status=True`` there to
        receive the raw stream.
        """
        if self._sock is None:
            raise ConnectionError("the SIL transport is not connected; call connect() first")
        deadline = time.monotonic() + max(0.0, timeout_s)
        while True:
            frame = self._take_pending()
            if frame is None:
                if not self._fill_pending(deadline):
                    return None
                continue
            if self._wanted(frame[0], False):
                return frame

    def drain_frames(
        self, timeout_s: float = DEFAULT_RECV_TIMEOUT_S, include_output_status: bool = False
    ) -> List[Tuple[int, bytes]]:
        """
        Decode every frame available inside one bounded read window.

        Used by the polling loops so a telemetry monitor counts the frames that
        really arrived instead of the ones a one-frame-at-a-time read happened to
        reach.  The pending queue is emptied first so a backlog is never dropped.

        ``include_output_status`` defaults to False, so this returns exactly what
        :meth:`recv_frame` returns: 0x7FE snapshots are recorded into
        ``latest_outputs()`` and withheld.  That is not pedantry - the channel is
        a SIL-only mock-BSP readout and not a vehicle CAN message
        (``sil_protocol.py:22``), and the engine publishes it every 10 ms tick
        (``sil_bridge_server.c:339-359``), roughly ten times more often than any
        telemetry frame, so a caller counting this list would report its counts
        dominated by a mock channel.  Pass ``True`` to receive the raw stream
        including 0x7FE; the decoded readback is available from
        ``latest_outputs()`` either way.
        """
        if self._sock is None:
            raise ConnectionError("the SIL transport is not connected; call connect() first")
        collected: List[Tuple[int, bytes]] = []
        while True:
            frame = self._take_pending()
            if frame is None:
                break
            if self._wanted(frame[0], include_output_status):
                collected.append(frame)
        for can_id, payload in recv_frames(self._sock, max(0.0, timeout_s)):
            self._record(can_id, payload)
            if self._wanted(can_id, include_output_status):
                collected.append((can_id, payload))
        return collected

    # -- drain plumbing, shared so the output-status rule has one home -------
    def _wanted(self, can_id: int, include_output_status: bool) -> bool:
        """
        The one rule for whether a decoded frame is handed back to a caller.

        0x7FE is withheld unless explicitly requested.  ``_record`` has already
        consumed it into ``latest_outputs()`` by this point either way, so
        withholding it costs a caller nothing: both read paths share this
        predicate, which is what stops :meth:`recv_frame` and :meth:`drain_frames`
        from disagreeing about the same channel.
        """
        return include_output_status or can_id != CAN_ID_SIL_OUTPUT_STATUS

    def _take_pending(self) -> Optional[Tuple[int, bytes]]:
        """Pop and record the next already-decoded frame, or None if the queue is empty."""
        if not self._pending:
            return None
        can_id, payload = self._pending.pop(0)
        self._record(can_id, payload)
        return can_id, payload

    def _fill_pending(self, deadline: float) -> bool:
        """
        Move whatever arrived inside the remaining budget into ``_pending``.

        Returns False when the window is over or produced nothing, so the caller
        ends rather than spins.  ``recv_frames`` keeps leftover bytes in a
        per-socket buffer, so a packet split across TCP segments is reassembled
        across calls.
        """
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        drained = recv_frames(self._sock, remaining)
        if not drained:
            return False
        self._pending.extend(drained)
        return True

    def _record(self, can_id: int, payload: bytes) -> None:
        if can_id == CAN_ID_SIL_OUTPUT_STATUS:
            # Raises ValueError on a short payload: a truncated snapshot is a
            # failed check, never a silently zero-filled one.
            self._latest = OutputStatus.decode(payload)
            return
        if self._listener is not None:
            self._listener(can_id, payload)

    def latest_outputs(self) -> Optional[OutputStatus]:
        return self._latest


def _optional_module_available(name: str) -> bool:
    """True when ``name`` can be imported, without importing it."""
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, ValueError):  # pragma: no cover - broken namespace package
        return False


def _require_optional_module(name: str, distribution: str, install_hint: str, mode: str):
    """
    Import a hardware dependency on demand, or explain how to get it.

    Never called at module import time, so ``import can_stimulus`` works on a
    host that has neither python-can nor pyserial, and the operator sees a
    one-line instruction instead of an ImportError traceback.
    """
    try:
        return __import__(name)
    except ImportError as exc:
        raise RuntimeError(
            f"--mode {mode} needs the optional '{distribution}' package, which is not installed. "
            f"Install it with '{install_hint}', or use --mode sil to run against the SIL engine. "
            f"(import error: {exc})"
        ) from exc


class PythonCanBackend(_BaseBackend):
    """
    Adapter for a real CAN FD bus, driven by python-can.

    What this transport is honestly for: **observing a real bus, not driving
    one.**  There is no 0x7FE readback on real hardware - that channel exists
    only in the SIL mock (``sil_protocol.py:22``) - so a command sent here could
    never be verified, and the checks that require a readback refuse before they
    write anything.  That refusal is the point: a thruster or valve command that
    moves something on a real vehicle and cannot be read back is exactly the
    blind actuation this tool must not perform.  The receive-only checks
    (``monitor_node1_env``, ``monitor_node3_power``, ``monitor_node2_depth``,
    ``sniff``) are meaningful here and are the supported use of this mode.

    ``raw_send`` is the one exception and is deliberate: the operator named the
    exact frame and bytes, and the tool is a pipe for it.  Its pass text says so
    and claims no vehicle response.
    """

    def __init__(self, interface: str = "can0", channel: Optional[str] = None) -> None:
        self.interface = interface
        self.channel = channel if channel is not None else interface
        self._bus = None
        self._can = None
        self._pending: List[Tuple[int, bytes]] = []

    def connect(self, timeout_s: float = SOCKET_TIMEOUT_S) -> None:
        if self._bus is not None:
            return
        self._can = _require_optional_module(
            "can", "python-can", "pip install python-can", "can"
        )
        self._bus = self._can.Bus(channel=self.channel, interface=self.interface, fd=True)
        self._pending.clear()

    def set_frame_listener(self, listener):
        previous, self._listener = getattr(self, "_listener", None), listener
        return previous

    def send_frame(self, can_id: int, payload: bytes) -> None:
        payload = validate_payload(payload)
        can_id = validate_can_id(can_id)
        if self._bus is None:
            raise ConnectionError(
                f"the CAN transport is not open on {self.interface}; call connect() first"
            )
        message = self._can.Message(
            arbitration_id=can_id,
            data=payload,
            is_extended_id=can_id > 0x7FF,
            is_fd=True,
        )
        self._bus.send(message)

    def recv_frame(self, timeout_s: float = DEFAULT_RECV_TIMEOUT_S) -> Optional[Tuple[int, bytes]]:
        if self._pending:
            return self._pending.pop(0)
        if self._bus is None:
            raise ConnectionError(
                f"the CAN transport is not open on {self.interface}; call connect() first"
            )
        message = self._bus.recv(timeout=max(0.0, timeout_s))
        if message is None:
            return None
        return int(message.arbitration_id), bytes(message.data)

    def latest_outputs(self) -> Optional[OutputStatus]:
        return None

    def describe_destination(self, can_id: int) -> str:
        return f"{describe_can_id(can_id)} on {self.interface}"

    def close(self) -> None:
        bus, self._bus = self._bus, None
        if bus is not None:
            with contextlib.suppress(Exception):
                bus.shutdown()
        self._pending.clear()


# A UART link carries no CAN arbitration id, so recv_frame reports this sentinel
# rather than inventing one. No check can match a telemetry id against it.
UART_UNDECODED_ID = -1


class UartBackend(_BaseBackend):
    """
    Adapter for a raw serial link, driven by pyserial.

    Same honest contract as :class:`PythonCanBackend`: **observe, do not drive.**
    Writes the payload verbatim and reads whatever bytes arrive; there is no CAN
    framing on a UART, so ``recv_frame`` reports :data:`UART_UNDECODED_ID` and
    ``describe_destination`` says plainly that the arbitration id was not
    addressed by this link.  With no 0x7FE readback and no CAN ids, only the
    frame counter in ``sniff`` means anything here.
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200) -> None:
        self.port = port
        self.baudrate = int(baudrate)
        self._serial = None
        self._pending: List[Tuple[int, bytes]] = []

    def connect(self, timeout_s: float = SOCKET_TIMEOUT_S) -> None:
        if self._serial is not None:
            return
        serial = _require_optional_module("serial", "pyserial", "pip install pyserial", "uart")
        try:
            self._serial = serial.Serial(
                self.port, baudrate=self.baudrate, timeout=max(0.0, timeout_s)
            )
        except Exception as exc:
            raise RuntimeError(f"could not open the UART port {self.port}: {exc}") from exc
        self._pending.clear()

    def set_frame_listener(self, listener):
        previous, self._listener = getattr(self, "_listener", None), listener
        return previous

    def send_frame(self, can_id: int, payload: bytes) -> None:
        payload = validate_payload(payload)
        if self._serial is None:
            raise ConnectionError(f"the UART port {self.port} is not open; call connect() first")
        self._serial.write(payload)

    def recv_frame(self, timeout_s: float = DEFAULT_RECV_TIMEOUT_S) -> Optional[Tuple[int, bytes]]:
        if self._pending:
            return self._pending.pop(0)
        if self._serial is None:
            raise ConnectionError(f"the UART port {self.port} is not open; call connect() first")
        data = self._serial.read(MAX_SIL_PAYLOAD)
        if not data:
            return None
        return UART_UNDECODED_ID, bytes(data)

    def latest_outputs(self) -> Optional[OutputStatus]:
        return None

    def describe_destination(self, can_id: int) -> str:
        return (
            f"the unaddressed byte stream on {self.port} at {self.baudrate} baud - this link "
            f"carries no arbitration id, so CAN 0x{can_id:03X} was NOT addressed by it"
        )

    def close(self) -> None:
        handle, self._serial = self._serial, None
        if handle is not None:
            with contextlib.suppress(Exception):
                handle.close()
        self._pending.clear()


# --------------------------------------------------------------------------
# Checks
# --------------------------------------------------------------------------


def _validate_pulse_us(pulse_us: object) -> int:
    if not isinstance(pulse_us, int) or isinstance(pulse_us, bool):
        raise TypeError(f"PWM pulse width must be an int, got {type(pulse_us).__name__}")
    if not PWM_MIN_US <= pulse_us <= PWM_MAX_US:
        raise ValueError(
            f"PWM {pulse_us} us is outside the firmware envelope "
            f"{PWM_MIN_US}-{PWM_MAX_US} us (rov_parameters.h:39-40)"
        )
    return pulse_us


def _validate_channel(channel: object) -> int:
    if not isinstance(channel, int) or isinstance(channel, bool):
        raise TypeError(f"thruster channel must be an int, got {type(channel).__name__}")
    if not 0 <= channel < NUM_THRUSTERS:
        raise ValueError(
            f"thruster channel {channel} does not exist; the board drives {NUM_THRUSTERS} "
            "channels (rov_parameters.h:37)"
        )
    return channel


class VehicleStimulusTester:
    """
    Named checks that each answer one question about the vehicle.

    Every method returns a ``CheckResult``.  Observation failures (timeout, decode
    error, wrong signature, missing readback) are returned as failures with the
    evidence in ``detail``; they are never raised.  Argument errors *are* raised,
    because a caller bug is not an observation and must not be reported as a
    vehicle failure.

    Two invariants are enforced here rather than left to each check:

    * a check that needs the 0x7FE readback refuses **before it writes anything**
      on a transport that cannot supply one, so a non-SIL mode never puts an
      unverifiable actuation on a bus (:meth:`_require_output_readback`);
    * a check that claims a precondition it did not observe fails instead of
      quietly dropping the claim (:meth:`test_emergency_break`).
    """

    def __init__(self, backend: _BaseBackend, on_result: Optional[Callable[[CheckResult], None]] = None) -> None:
        self.backend = backend
        # Called with each result as it completes, so a long run leaves evidence
        # behind even if the process is killed part way through.
        self.on_result = on_result

    # -- plumbing ----------------------------------------------------------
    def _require_output_readback(self, name: str) -> Optional[CheckResult]:
        """
        Refuse a readback-dependent check on a transport that has no 0x7FE.

        Returns a FAILED ``CheckResult`` to return instead of running, or None to
        proceed.  This is checked before the first ``send_frame`` on purpose: a
        refusal that arrives after the actuation is a refusal that is too late.
        """
        if getattr(self.backend, "provides_output_readback", False):
            return None
        return CheckResult(
            name,
            False,
            f"refused before sending any frame: the {describe_can_id(CAN_ID_SIL_OUTPUT_STATUS)} "
            "output-status readback is a SIL-only mock-BSP channel (sil_protocol.py:22) and this "
            "transport cannot supply it, so no actuation sent over "
            f"{self.backend.describe_destination(0)} could be verified. A thruster or valve "
            "command that moves something and cannot be read back is a blind actuation, so this "
            "check is SIL-only. Use --mode sil, or select the receive-only checks "
            "(--node1-env, --node3-power, --node2-depth, --sniff).",
        )

    def run_plan(self, plan: Sequence[Tuple[str, Callable[[], CheckResult]]]) -> StimulusReport:
        """Run a name/action plan, streaming each result as it completes."""
        results: List[CheckResult] = []
        for name, action in plan:
            result = self.guard(name, action)
            results.append(result)
            if self.on_result is not None:
                self.on_result(result)
        return StimulusReport(results)

    def guard(self, name: str, action: Callable[[], CheckResult]) -> CheckResult:
        """
        Run a check and convert any exception into a FAILED ``CheckResult``.

        This is the whole point of the tool.  The prototype raised
        ``AttributeError`` out of its own check and the process still exited 0, so
        a crash was indistinguishable from success.  Here the exception text is
        preserved in ``detail`` and the verdict is a failure.
        """
        try:
            result = action()
        except Exception as exc:  # noqa: BLE001 - deliberately total
            return CheckResult(name, False, f"check raised {type(exc).__name__}: {exc}")
        if not isinstance(result, CheckResult):
            return CheckResult(
                name, False, f"check returned {type(result).__name__} instead of a CheckResult"
            )
        return result

    def _pump(
        self,
        deadline: float,
        send: Optional[Callable[[], None]] = None,
        on_frame: Optional[Callable[[int, bytes], None]] = None,
        on_output: Optional[Callable[[OutputStatus], None]] = None,
        until: Optional[Callable[[OutputStatus], bool]] = None,
    ) -> List[OutputStatus]:
        """
        Bounded transport loop used by every check.

        Resends the command at :data:`COMMAND_PERIOD_S` so the firmware's 100 ms
        heartbeat stays fed, and reads in :data:`READ_POLL_S` slices so a check
        never blocks longer than its own deadline.  Returns the fresh 0x7FE
        snapshots observed; ``until`` ends the loop as soon as it is satisfied.
        """
        snapshots: List[OutputStatus] = []
        previous_listener = self.backend.set_frame_listener(on_frame)
        next_send_due = time.monotonic()
        try:
            while True:
                now = time.monotonic()
                if now >= deadline:
                    break
                if send is not None and now >= next_send_due:
                    send()
                    next_send_due = now + COMMAND_PERIOD_S
                slice_s = max(0.0, min(READ_POLL_S, deadline - now))
                before = self.backend.latest_outputs()
                if hasattr(self.backend, "drain_frames"):
                    self.backend.drain_frames(slice_s)
                else:
                    # PythonCanBackend and UartBackend have no drain_frames: they
                    # hand back one frame at a time and no 0x7FE readback, so the
                    # readback-dependent checks have already refused. The
                    # receive-only checks still run here, one frame per slice.
                    self.backend.recv_frame(timeout_s=slice_s)
                current = self.backend.latest_outputs()
                if current is not None and current is not before:
                    snapshots.append(current)
                    if on_output is not None:
                        on_output(current)
                    if until is not None and until(current):
                        break
        finally:
            self.backend.set_frame_listener(previous_listener)
        return snapshots

    def _await_output(
        self,
        predicate: Callable[[OutputStatus], bool],
        timeout_s: float,
        send: Optional[Callable[[], None]] = None,
    ) -> Optional[OutputStatus]:
        """Bounded poll for the first fresh snapshot satisfying ``predicate``."""
        snapshots = self._pump(time.monotonic() + max(0.0, timeout_s), send=send, until=predicate)
        if snapshots and predicate(snapshots[-1]):
            return snapshots[-1]
        return None

    def _first_output(self, timeout_s: float) -> Optional[OutputStatus]:
        return self._await_output(lambda snapshot: True, timeout_s)

    def _collect(
        self,
        can_id: int,
        duration_s: float,
        send: Optional[Callable[[], None]] = None,
        on_output: Optional[Callable[[OutputStatus], None]] = None,
    ) -> List[bytes]:
        payloads: List[bytes] = []

        def on_frame(frame_id: int, payload: bytes) -> None:
            if frame_id == can_id:
                payloads.append(payload)

        self._pump(
            time.monotonic() + max(0.0, duration_s),
            send=send,
            on_frame=on_frame,
            on_output=on_output,
        )
        return payloads

    def _no_readback_failure(self, name: str, timeout_s: float) -> CheckResult:
        return CheckResult(
            name,
            False,
            f"no 0x7FE output-status readback within {timeout_s:.1f} s. The SIL engine only "
            "publishes that channel to a connected client (sil_bridge_server.c:339-359), so this "
            "means the engine is not running, the connection was dropped, or the client is not "
            "reading - the request is NOT confirmed.",
        )

    def _release_to_neutral(self, timeout_s: float = NEUTRAL_TIMEOUT_S) -> bool:
        """Stop commanding and require the outputs to come back to neutral."""
        neutral = ThrusterCommand(pwm_us=list(NEUTRAL_PWMS)).pack()

        def send_neutral() -> None:
            self.backend.send_frame(CAN_ID_THRUSTER_CMD, neutral)

        released = self._await_output(
            lambda snapshot: tuple(snapshot.pwms) == NEUTRAL_PWMS, timeout_s, send=send_neutral
        )
        return released is not None

    def _ramp_budget_s(self, pulse_us: int, sim_rate: float, slack_s: float) -> float:
        """
        Wall-clock budget for a slew ramp to ``pulse_us`` and back.

        The 2 us/ms ramp is charged in *simulated* milliseconds
        (``rov_parameters.h:45``) and converted at ``sim_rate``, so a test can
        raise the rate instead of waiting out half a second of virtual time.
        """
        travel_ms = abs(pulse_us - NEUTRAL_US) / PWM_SLEW_RATE_US_PER_MS
        return travel_ms / 1000.0 / max(1e-6, sim_rate) + slack_s

    # -- node 1: Pi shield -------------------------------------------------
    def monitor_node1_env(self, duration_s: float = 3.0) -> CheckResult:
        """
        Watch 0x210 enclosure telemetry from the Pi shield.

        Asserts that at least two frames arrive in the window, that every frame
        decodes as a full 13-byte ``EnvTelemetry`` record, that the values are
        finite and physically plausible, and that no leak is flagged.  The count
        of two is grounded in the server: the first 0x210 is sent immediately on
        connect and the rest at 1 Hz (sil_bridge_server.c:376-382), so a 3.0 s
        window must contain more than one.  A nonzero leak flag in a nominal
        simulation is a genuine failure, because the shield would already have
        broadcast an emergency break (node1 app.c:220-222).
        """
        name = "node1_env"
        payloads = self._collect(CAN_ID_ENV_TELEMETRY, duration_s)
        if not payloads:
            return CheckResult(
                name,
                False,
                f"no 0x210 environment telemetry within {duration_s:.1f} s; the Pi shield streams "
                "it at 10 Hz (node1 app.c:159) and the server forwards the first frame immediately "
                f"then 1 Hz (sil_bridge_server.c:376-382). Received nothing.",
            )
        if len(payloads) < 2:
            return CheckResult(
                name,
                False,
                f"only {len(payloads)} 0x210 frame(s) in {duration_s:.1f} s; the server emits the "
                "first frame on connect and then one per second, so at least 2 are required to "
                "distinguish a live stream from a single stale frame.",
            )
        try:
            records = [EnvTelemetry.unpack(payload) for payload in payloads]
        except ValueError as exc:
            return CheckResult(
                name, False, f"0x210 frame could not be decoded as EnvTelemetry: {exc}"
            )
        for index, record in enumerate(records):
            for field in ("pressure_hpa", "humidity_pct", "temperature_c"):
                if not math.isfinite(getattr(record, field)):
                    return CheckResult(
                        name, False, f"0x210 frame {index} has a non-finite {field}: {record}"
                    )
            if not 500.0 <= record.pressure_hpa <= 1200.0:
                return CheckResult(
                    name,
                    False,
                    f"0x210 frame {index} reports {record.pressure_hpa:.2f} hPa, outside the "
                    "500-1200 hPa range a submerged enclosure can hold",
                )
            if not 0.0 <= record.humidity_pct <= 100.0:
                return CheckResult(
                    name, False, f"0x210 frame {index} reports {record.humidity_pct:.2f} % RH"
                )
            if not -40.0 <= record.temperature_c <= 85.0:
                return CheckResult(
                    name, False, f"0x210 frame {index} reports {record.temperature_c:.2f} C"
                )
            if record.leak_flags != 0:
                return CheckResult(
                    name,
                    False,
                    f"0x210 frame {index} reports leak_flags=0x{record.leak_flags:02X}; a nominal "
                    "enclosure must be dry. A set flag means the shield should already have "
                    "broadcast an emergency break (node1 app.c:220-222).",
                )
        last = records[-1]
        return CheckResult(
            name,
            True,
            f"{len(records)} frame(s) decoded in {duration_s:.1f} s: "
            f"pressure={last.pressure_hpa:.2f} hPa humidity={last.humidity_pct:.1f} % "
            f"temperature={last.temperature_c:.1f} C leak_flags=0x{last.leak_flags:02X}",
        )

    # -- node 2: control board --------------------------------------------
    def test_node2_arming(
        self,
        sim_rate: float = SIM_MS_PER_WALL_S,
        slack_s: float = SIM_WALL_SLACK_S,
        first_output_timeout_s: float = FIRST_OUTPUT_TIMEOUT_S,
    ) -> CheckResult:
        """
        Prove the mandatory ESC arming gate, in both directions.

        ``nodes/node2_control_board/Core/Src/app.c`` refuses every thruster
        command while ``g_esc_state != ESC_STATE_ACTIVE`` (app.c:127-129) and
        holds all eight channels at neutral for the whole
        ``ESC_ARMING_TIME_MS`` window (app.c:192-194), where
        ``ESC_ARMING_TIME_MS`` is 3000 ms of *simulated* time (app.c:24,99-102).

        Asserts: a neutral observation taken while the window is still open; that
        an aggressive command sent during the window is ignored (the gate, not
        an absence of traffic, is what holds neutral); that the same command takes
        effect once the window closes; and that releasing it returns the outputs
        to neutral.

        The neutral run is required to cover the window with
        :data:`MIN_ARMING_SAMPLES` observations *strictly inside* it.  Without
        that floor a single post-3000 snapshot would make the intruder list empty
        and the gate would look verified having never been watched.

        ``sim_rate`` and ``slack_s`` let the tests drive these branches without
        waiting out 3000 ms of virtual time; production callers use the defaults.
        """
        name = "node2_arming"
        refused = self._require_output_readback(name)
        if refused is not None:
            return refused
        first = self._first_output(first_output_timeout_s)
        if first is None:
            return self._no_readback_failure(name, first_output_timeout_s)
        if first.sim_time_ms >= ESC_ARMING_SIM_MS:
            return CheckResult(
                name,
                False,
                f"the arming gate was already open when the first snapshot arrived "
                f"(sim_time_ms={first.sim_time_ms} >= {ESC_ARMING_SIM_MS}), so the mandatory "
                f"{ESC_ARMING_SIM_MS} ms window could not be observed. Restart the engine (or let "
                "this tool launch one) and run the arming check first.",
            )
        if tuple(first.pwms) != NEUTRAL_PWMS:
            return CheckResult(
                name,
                False,
                f"outputs were not neutral before arming: {list(first.pwms)} at "
                f"sim_time_ms={first.sim_time_ms}; app.c:63-67 parks every channel at "
                f"{NEUTRAL_US} us during boot and arming",
            )

        probe_pwms = [SMOKE_SINGLE_PWM_US] * NUM_THRUSTERS
        command = ThrusterCommand(pwm_us=probe_pwms).pack()

        def send_command() -> None:
            self.backend.send_frame(CAN_ID_THRUSTER_CMD, command)

        remaining_sim_ms = ESC_ARMING_SIM_MS - first.sim_time_ms
        window_budget_s = remaining_sim_ms / 1000.0 / max(1e-6, sim_rate) + slack_s
        during = self._pump(
            time.monotonic() + window_budget_s,
            send=send_command,
            until=lambda snapshot: snapshot.sim_time_ms >= ESC_ARMING_SIM_MS,
        )
        if not during or during[-1].sim_time_ms < ESC_ARMING_SIM_MS:
            reached = during[-1].sim_time_ms if during else "no snapshots"
            return CheckResult(
                name,
                False,
                f"the {ESC_ARMING_SIM_MS} ms arming window never elapsed within {window_budget_s:.1f} "
                f"s of wall time (sim clock reached {reached} ms). The engine's virtual clock is "
                "not advancing, so no arming verdict is possible.",
            )
        inside = [snapshot for snapshot in during if snapshot.sim_time_ms < ESC_ARMING_SIM_MS]
        if len(inside) < MIN_ARMING_SAMPLES:
            return CheckResult(
                name,
                False,
                f"only {len(inside)} observation(s) landed strictly inside the {ESC_ARMING_SIM_MS} ms "
                f"window, below the {MIN_ARMING_SAMPLES} required. A single snapshot at or after the "
                "boundary would make the window look observed when it was not, so the gate holding "
                "neutral is unverified.",
            )
        intruders = [
            snapshot for snapshot in inside if tuple(snapshot.pwms) != NEUTRAL_PWMS
        ]
        if intruders:
            return CheckResult(
                name,
                False,
                f"outputs left neutral at sim_time_ms={intruders[0].sim_time_ms} < "
                f"{ESC_ARMING_SIM_MS} while a {SMOKE_SINGLE_PWM_US} us command was being resent; "
                f"app.c:192-194 must hold neutral for the whole arming window. First offender: "
                f"{intruders[0].describe()}",
            )

        opened = self._await_output(
            lambda snapshot: snapshot.sim_time_ms >= ESC_ARMING_SIM_MS
            and any(pulse != NEUTRAL_US for pulse in snapshot.pwms),
            self._ramp_budget_s(SMOKE_SINGLE_PWM_US, sim_rate, slack_s),
            send=send_command,
        )
        if opened is None:
            last = self.backend.latest_outputs()
            seen = "no readback" if last is None else last.describe()
            return CheckResult(
                name,
                False,
                f"the arming window closed at {ESC_ARMING_SIM_MS} ms but the outputs never left "
                f"neutral afterwards, so the gate never opened ({seen}). Either the command is not "
                "reaching node 2 or the board stays disarmed.",
            )
        if not self._release_to_neutral(self._ramp_budget_s(SMOKE_SINGLE_PWM_US, sim_rate, slack_s)):
            return CheckResult(
                name,
                False,
                f"thrust was observed at sim_time_ms={opened.sim_time_ms} but the outputs did not "
                f"return to {NEUTRAL_US} us after the command was released",
            )
        return CheckResult(
            name,
            True,
            f"gate held neutral across {len(inside)} observation(s) inside the first "
            f"{ESC_ARMING_SIM_MS} ms of sim time against a resent {SMOKE_SINGLE_PWM_US} us command, "
            f"then took effect at sim_time_ms={opened.sim_time_ms} ({list(opened.pwms)}) and "
            "released back to neutral",
        )

    def test_node2_pwm(
        self,
        channel: int,
        pulse_us: int,
        duration_s: float = 1.0,
        timeout_s: float = RAMP_TIMEOUT_S,
        sim_rate: float = SIM_MS_PER_WALL_S,
        slack_s: float = SIM_WALL_SLACK_S,
    ) -> CheckResult:
        """
        Drive one thruster channel and require it to hold.

        The interesting part is the *hold*: node 2 drops to neutral when no
        thruster command arrives for 100 ms (``rov_parameters.h:71``,
        ``rov_safety.c:29-35``, ``app.c:182-189``), so a single frame is not
        evidence of anything.  The command is resent at 20 Hz and every snapshot
        inside the observation window must hold the commanded value, and the other
        seven channels must stay at neutral throughout.

        ``sim_rate``/``slack_s``/``timeout_s`` exist so the failure branches are
        testable without waiting out a slew ramp of half a second of virtual time.
        """
        channel = _validate_channel(channel)
        pulse_us = _validate_pulse_us(pulse_us)
        name = f"node2_pwm_ch{channel}"
        refused = self._require_output_readback(name)
        if refused is not None:
            return refused
        pwms = [NEUTRAL_US] * NUM_THRUSTERS
        pwms[channel] = pulse_us
        command = ThrusterCommand(pwm_us=pwms).pack()

        def send_command() -> None:
            self.backend.send_frame(CAN_ID_THRUSTER_CMD, command)

        reached = self._await_output(
            lambda snapshot: snapshot.pwms[channel] == pulse_us, timeout_s, send=send_command
        )
        if reached is None:
            last = self.backend.latest_outputs()
            seen = "no readback" if last is None else last.describe()
            # The generic message, and the facts behind it. Built first so both
            # branches below carry the same evidence; the gate-closed branch then
            # leads with the conclusion, because "the command was never accepted"
            # and "the board cannot accept commands yet" call for different
            # operator actions and must not read the same.
            detail = (
                f"channel {channel} never reached {pulse_us} us within {timeout_s:.1f} s "
                f"({seen}). The 2 us/ms slew ramp (rov_parameters.h:45) needs "
                f"{abs(pulse_us - NEUTRAL_US) // PWM_SLEW_RATE_US_PER_MS} ms of sim time, and the "
                "command is only accepted once the ESC gate is open (app.c:127-129)."
            )
            if last is not None and last.sim_time_ms < ESC_ARMING_SIM_MS:
                # The budget expired while the mandatory arming window was still
                # running, so this check has not examined the PWM path at all.
                # RAMP_TIMEOUT_S is a fixed SIM_WALL_SLACK_S cushion on top of the
                # ramp, and the arming window is 3000 ms of *simulated* time, so a
                # host whose ticks are slower than the default assumption spends
                # its whole budget arming. Saying only "never reached" would send
                # the operator after a broken PWM path that was never exercised.
                return CheckResult(
                    name,
                    False,
                    f"the ESC gate was still closed at sim_time_ms={last.sim_time_ms}, below the "
                    f"mandatory {ESC_ARMING_SIM_MS} ms arming window (app.c:24), so this check has "
                    f"NOT examined the PWM path: the {timeout_s:.1f} s budget ran out before the "
                    "board was ever able to accept a thruster command. This is a too-short "
                    "timeout_s or a slow host, not a broken PWM path - raise timeout_s, or run "
                    f"--node2-arm first and re-run this check. For the record: {detail}",
                )
            return CheckResult(name, False, detail)
        samples = self._pump(time.monotonic() + max(0.0, duration_s), send=send_command)
        if not samples:
            return CheckResult(
                name,
                False,
                f"channel {channel} reached {pulse_us} us but no fresh 0x7FE snapshot arrived during "
                f"the {duration_s:.1f} s observation window, so the hold is unverified",
            )
        drifted = [s for s in samples if s.pwms[channel] != pulse_us]
        if drifted:
            return CheckResult(
                name,
                False,
                f"channel {channel} did not hold {pulse_us} us for the whole {duration_s:.1f} s "
                f"window: {len(drifted)}/{len(samples)} snapshot(s) disagreed, first at "
                f"sim_time_ms={drifted[0].sim_time_ms} showing {list(drifted[0].pwms)}. A 100 ms "
                "watchdog lapse would look exactly like this (app.c:182-189).",
            )
        polluted = [
            index
            for index in range(NUM_THRUSTERS)
            if index != channel and any(s.pwms[index] != NEUTRAL_US for s in samples)
        ]
        if polluted:
            return CheckResult(
                name,
                False,
                f"channel(s) {polluted} moved off {NEUTRAL_US} us while only channel {channel} was "
                "commanded; the board must apply an 8-channel frame per channel",
            )
        if not self._release_to_neutral(self._ramp_budget_s(pulse_us, sim_rate, slack_s)):
            return CheckResult(
                name,
                False,
                f"channel {channel} held {pulse_us} us, but the outputs did not return to "
                f"{NEUTRAL_US} us after the command was released",
            )
        return CheckResult(
            name,
            True,
            f"channel {channel} held {pulse_us} us across {len(samples)} snapshot(s) in "
            f"{duration_s:.1f} s (> the {int(HEARTBEAT_TIMEOUT_S * 1000)} ms watchdog) with the other "
            "7 channels at neutral, then released to neutral",
        )

    def test_node2_all(
        self,
        pulse_us: int,
        duration_s: float = 1.0,
        timeout_s: float = RAMP_TIMEOUT_S,
        sim_rate: float = SIM_MS_PER_WALL_S,
        slack_s: float = SIM_WALL_SLACK_S,
    ) -> CheckResult:
        """
        Drive all eight channels and require the whole vector to hold.

        Same heartbeat argument as :meth:`test_node2_pwm`, applied to the full
        allocation, so a partial application (a dropped frame, a short payload) is
        caught.
        """
        pulse_us = _validate_pulse_us(pulse_us)
        name = "node2_all"
        refused = self._require_output_readback(name)
        if refused is not None:
            return refused
        command = ThrusterCommand(pwm_us=[pulse_us] * NUM_THRUSTERS).pack()

        def send_command() -> None:
            self.backend.send_frame(CAN_ID_THRUSTER_CMD, command)

        target = (pulse_us,) * NUM_THRUSTERS
        reached = self._await_output(
            lambda snapshot: tuple(snapshot.pwms) == target, timeout_s, send=send_command
        )
        if reached is None:
            last = self.backend.latest_outputs()
            seen = "no readback" if last is None else last.describe()
            return CheckResult(
                name,
                False,
                f"the board never reached {pulse_us} us on all {NUM_THRUSTERS} channels within "
                f"{timeout_s:.1f} s ({seen}). The 2 us/ms slew ramp (rov_parameters.h:45) needs "
                f"{abs(pulse_us - NEUTRAL_US) // PWM_SLEW_RATE_US_PER_MS} ms of sim time.",
            )
        samples = self._pump(time.monotonic() + max(0.0, duration_s), send=send_command)
        if not samples:
            return CheckResult(
                name,
                False,
                f"all channels reached {pulse_us} us but no fresh 0x7FE snapshot arrived during the "
                f"{duration_s:.1f} s observation window",
            )
        drifted = [s for s in samples if tuple(s.pwms) != target]
        if drifted:
            first_bad = drifted[0]
            channels = [i for i, v in enumerate(first_bad.pwms) if v != pulse_us]
            return CheckResult(
                name,
                False,
                f"the {pulse_us} us allocation did not hold for the whole {duration_s:.1f} s window: "
                f"{len(drifted)}/{len(samples)} snapshot(s) disagreed, channel(s) {channels} at "
                f"sim_time_ms={first_bad.sim_time_ms} showing {list(first_bad.pwms)}",
            )
        if not self._release_to_neutral(self._ramp_budget_s(pulse_us, sim_rate, slack_s)):
            return CheckResult(
                name,
                False,
                f"all channels held {pulse_us} us, but the outputs did not return to {NEUTRAL_US} us "
                "after the command was released",
            )
        return CheckResult(
            name,
            True,
            f"all {NUM_THRUSTERS} channels held {pulse_us} us across {len(samples)} snapshot(s) in "
            f"{duration_s:.1f} s (> the {int(HEARTBEAT_TIMEOUT_S * 1000)} ms watchdog), then "
            "released to neutral",
        )

    def test_node2_solenoid(
        self, mask: int, timeout_s: float = SOLENOID_OBSERVE_TIMEOUT_S
    ) -> CheckResult:
        """
        Actuate a solenoid mask and require the board to report exactly that mask.

        The mask is validated against the real interlock first, because an invalid
        mask cannot be used as evidence.  ``shared/src/rov_can_protocol.c:118-126``
        masks to ten channels and then rejects any valve with both opposing coils
        energised by zeroing the *whole* mask and returning ``ROV_ERR_INVALID_ARG``;
        ``app.c:150-153`` only calls ``bsp_solenoid_set`` on ``ROV_OK``, so the
        hardware keeps its previous mask.  A tool that sent such a mask and then
        read back a stale or all-zero mask would be reporting the wrong thing
        entirely, so the check refuses it up front and says why.

        A neutral thruster command is resent while polling: once the ESC gate is
        open, a heartbeat lapse calls ``node2_force_neutral()``, which zeroes the
        solenoids (``app.c:49-52,182-189``).
        """
        name = "node2_solenoid"
        if not isinstance(mask, int) or isinstance(mask, bool):
            raise TypeError(f"solenoid mask must be an int, got {type(mask).__name__}")
        if not 0 <= mask <= SOLENOID_MASK_MAX:
            raise ValueError(
                f"solenoid mask 0x{mask:X} is outside the 10-bit field (rov_parameters.h:54)"
            )
        if mask == 0:
            # 0x0000 is the released state the board already holds
            # (app.c:72 zeroes it at init), so commanding it produces no state
            # change to observe: the readback would satisfy any predicate on the
            # first snapshot from a board that never received the frame. It is
            # refused here rather than reported as a successful actuation.
            raise ValueError(
                "solenoid mask 0x0000 is refused: it commands the released state the board "
                "already holds (app.c:72), so it cannot be evidenced by a readback - a de-energised "
                "mask and an ignored frame look identical. Pass a mask that energises at least one "
                f"coil, such as 0x{SMOKE_SOLENOID_MASK:04X} (valve 0 coil A)."
            )
        refused = self._require_output_readback(name)
        if refused is not None:
            return refused
        conflict = conflicting_solenoid_valve(mask)
        if conflict is not None:
            pair_mask = 0x3 << (2 * conflict)
            return CheckResult(
                name,
                False,
                f"mask 0x{mask:04X} energises both opposing coils of valve {conflict} "
                f"(bits {2 * conflict} and {2 * conflict + 1}, pair mask 0x{pair_mask:04X}). "
                "rov_can_unpack_solenoid_cmd (shared/src/rov_can_protocol.c:120-126) zeroes the "
                "whole mask and returns ROV_ERR_INVALID_ARG, and app.c:150-153 then skips "
                "bsp_solenoid_set, so the readback would still show the previous mask and say "
                "nothing about this frame. Re-run with a single-coil mask such as "
                f"0x{SMOKE_SOLENOID_MASK:04X} (valve 0 coil A).",
            )

        coils = energised_coils(mask)
        neutral = ThrusterCommand(pwm_us=list(NEUTRAL_PWMS)).pack()

        def send_neutral() -> None:
            self.backend.send_frame(CAN_ID_THRUSTER_CMD, neutral)

        # Read the pre-command state before writing, so the pass can require an
        # observed transition rather than merely a value that was already true.
        # Bounded, because a missing readback is a failed check, not a hang.
        pre_existing = self._await_output(lambda snapshot: True, min(timeout_s, 0.5))
        if pre_existing is None:
            return self._no_readback_failure(name, min(timeout_s, 0.5))
        previous_mask = pre_existing.solenoids
        if previous_mask == mask:
            return CheckResult(
                name,
                False,
                f"the board already reports solenoid mask 0x{mask:04X} at sim_time_ms="
                f"{pre_existing.sim_time_ms}, before this check sent anything. Commanding the state "
                "the board is already in produces an identical readback, so the frame cannot be "
                "distinguished from one that was ignored. Pick a mask the board is not holding, or "
                "restart the engine to clear the latched mask.",
            )
        self.backend.send_frame(CAN_ID_SOLENOID_CMD, SolenoidCommand(solenoid_mask=mask).pack())
        actuated = self._await_output(
            lambda snapshot: snapshot.solenoids == mask, timeout_s, send=send_neutral
        )
        if actuated is None:
            last = self.backend.latest_outputs()
            seen = "no 0x7FE readback" if last is None else last.describe()
            return CheckResult(
                name,
                False,
                f"0x110 mask 0x{mask:04X} was written but the board never reported it back within "
                f"{timeout_s:.1f} s ({seen}). Either the frame did not reach node 2 or the mask was "
                "rejected; the readback is the only evidence of actuation.",
            )
        self.backend.send_frame(CAN_ID_SOLENOID_CMD, SolenoidCommand(solenoid_mask=0).pack())
        released = self._await_output(
            lambda snapshot: snapshot.solenoids == 0, timeout_s, send=send_neutral
        )
        if released is None:
            return CheckResult(
                name,
                False,
                f"0x110 mask 0x{mask:04X} actuated (readback 0x{actuated.solenoids:04X} at "
                f"sim_time_ms={actuated.sim_time_ms}) but the board did not return to 0x0000 after "
                "being commanded to release",
            )
        return CheckResult(
            name,
            True,
            f"0x110 mask 0x{mask:04X} energised {', '.join(coils)} and the board reported that mask "
            f"back at sim_time_ms={actuated.sim_time_ms}, a change from the pre-command "
            f"0x{previous_mask:04X} observed at sim_time_ms={pre_existing.sim_time_ms}. The mask "
            "energises one coil per valve, so rov_can_unpack_solenoid_cmd accepts it, and it was "
            f"released back to 0x0000 at sim_time_ms={released.sim_time_ms}.",
        )

    def monitor_node2_depth(self, duration_s: float = 2.0) -> CheckResult:
        """
        Watch 0x200 navigation telemetry, which carries the depth reading.

        The server forwards nav telemetry immediately at the board's 100 Hz rate
        (``sil_bridge_server.c:372-375``, ``rov_parameters.h:66``), so a 2.0 s
        window must carry a real stream.  Every frame must decode as a full
        ``NavTelemetry`` record with finite values, and the depth must respect the
        board's own acceptance range: ``app.c:228-234`` rejects anything below
        -0.5 m and clamps a small negative reading to 0.0 m.

        The 100 Hz is a *simulated* time rate, and the floor below is expressed in
        that clock too.  The C engine sleeps 10 ms of wall time per 10 ms tick
        (``sil_bridge_server.c:427``) and Windows ``Sleep`` has roughly 15.6 ms
        granularity, so a host observes about 64 Hz of wall clock for a 100 Hz
        stream; a floor in wall time would be a statement about the host's timer
        as much as about the board.  Every 0x7FE snapshot carries the engine's own
        clock, so the rate is measured against that and the wall figure is reported
        beside it, labelled as wall.
        """
        name = "node2_depth"
        sim_ms: List[int] = []
        payloads = self._collect(
            CAN_ID_NAV_TELEMETRY,
            duration_s,
            on_output=lambda snapshot: sim_ms.append(snapshot.sim_time_ms),
        )
        if not payloads:
            return CheckResult(
                name,
                False,
                f"no 0x200 navigation telemetry within {duration_s:.1f} s; node 2 streams it at "
                "100 Hz (node2 app.c:218-262) and the server forwards it unthrottled "
                "(sil_bridge_server.c:372-375). Received nothing.",
            )
        wall_hz = len(payloads) / max(1e-6, duration_s)
        # Simulated span across the snapshots that bracket the window. A span of
        # zero means a clock that did not move, which is not a rate at all, so the
        # wall figure stands in and the message says which clock it came from.
        sim_span_ms = sim_ms[-1] - sim_ms[0] if len(sim_ms) >= 2 else 0
        sim_hz = len(payloads) / (sim_span_ms / 1000.0) if sim_span_ms > 0 else None
        measured_hz, clock = (sim_hz, "simulated") if sim_hz is not None else (wall_hz, "wall")
        if measured_hz < MIN_TELEMETRY_HZ:
            return CheckResult(
                name,
                False,
                f"0x200 arrived at only {measured_hz:.1f} Hz {clock} ({len(payloads)} frames in "
                f"{duration_s:.1f} s); the board is specified for 100 Hz (rov_parameters.h:66) and "
                f"the server forwards it unthrottled, so anything under {MIN_TELEMETRY_HZ:.0f} Hz "
                "is a broken stream",
            )
        try:
            records = [NavTelemetry.unpack(payload) for payload in payloads]
        except ValueError as exc:
            return CheckResult(
                name, False, f"0x200 frame could not be decoded as NavTelemetry: {exc}"
            )
        for index, record in enumerate(records):
            values = {
                "q_w": record.q_w,
                "q_x": record.q_x,
                "q_y": record.q_y,
                "q_z": record.q_z,
                "gyro_x_rad_s": record.gyro_x_rad_s,
                "gyro_y_rad_s": record.gyro_y_rad_s,
                "gyro_z_rad_s": record.gyro_z_rad_s,
                "depth_meters": record.depth_meters,
            }
            for field, value in values.items():
                if not math.isfinite(value):
                    return CheckResult(
                        name, False, f"0x200 frame {index} has a non-finite {field}: {value}"
                    )
            if record.depth_meters < 0.0:
                return CheckResult(
                    name,
                    False,
                    f"0x200 frame {index} reports depth {record.depth_meters:.3f} m; node 2 clamps a "
                    "small negative reading to 0.0 m and rejects anything below -0.5 m "
                    "(node2 app.c:228-234)",
                )
            if record.depth_meters > 300.0:
                return CheckResult(
                    name,
                    False,
                    f"0x200 frame {index} reports depth {record.depth_meters:.2f} m, deeper than any "
                    "planned dive",
                )
        last = records[-1]
        if sim_hz is None:
            rate_text = (
                f"{len(records)} frame(s) at {wall_hz:.0f} Hz wall in {duration_s:.1f} s; the "
                "simulated rate is UNMEASURED because no 0x7FE output-status snapshot carried the "
                "engine's clock, so the floor above was applied to wall time"
            )
        else:
            rate_text = (
                f"{len(records)} frame(s) at {sim_hz:.0f} Hz simulated ({wall_hz:.0f} Hz wall), "
                f"measured over {sim_span_ms:.0f} ms of engine time, against the 100 Hz design rate"
            )
        return CheckResult(
            name,
            True,
            f"{rate_text}; depth={last.depth_meters:.3f} m imu_status=0x{last.imu_status:02X} "
            f"gyro=({last.gyro_x_rad_s:+.4f},{last.gyro_y_rad_s:+.4f},{last.gyro_z_rad_s:+.4f})",
        )

    # -- node 3: power slab -----------------------------------------------
    def monitor_node3_power(self, duration_s: float = 3.0) -> CheckResult:
        """
        Watch 0x300 power telemetry from the power slab.

        The server sends the first 0x300 immediately on connect and then one per
        second (``sil_bridge_server.c:383-389``), so a 3.0 s window must contain
        more than one frame.  Every frame must decode as a full 20-byte
        ``PowerTelemetry`` record, the rails must sit in their documented
        envelopes, the four 12 V brick currents must be present and sane, and the
        fault bit (``status_flags & 0x0001``, node3 ``app.c:186,197``) must be
        clear.  A set fault bit means the slab latched a fault and broadcast a
        0x005 eFuse alert, which trips the control board's brake (``app.c:121-126``).
        """
        name = "node3_power"
        payloads = self._collect(CAN_ID_POWER_TELEMETRY, duration_s)
        if not payloads:
            return CheckResult(
                name,
                False,
                f"no 0x300 power telemetry within {duration_s:.1f} s; node 3 streams it at 20 Hz "
                "(node3 app.c:78-207) and the server forwards the first frame immediately then 1 Hz "
                f"(sil_bridge_server.c:383-389). Received nothing.",
            )
        if len(payloads) < 2:
            return CheckResult(
                name,
                False,
                f"only {len(payloads)} 0x300 frame(s) in {duration_s:.1f} s; the server emits the "
                "first frame on connect and then one per second, so at least 2 are required to "
                "distinguish a live stream from a single stale frame.",
            )
        try:
            records = [PowerTelemetry.unpack(payload) for payload in payloads]
        except ValueError as exc:
            return CheckResult(
                name, False, f"0x300 frame could not be decoded as PowerTelemetry: {exc}"
            )
        for index, record in enumerate(records):
            if not 40000 <= record.tether_voltage_mv <= 60000:
                return CheckResult(
                    name,
                    False,
                    f"0x300 frame {index} reports a tether of {record.tether_voltage_mv} mV, outside "
                    "the 40-60 V tether envelope (rov_parameters.h:17)",
                )
            if not 4000 <= record.v5_voltage_mv <= 7000:
                return CheckResult(
                    name,
                    False,
                    f"0x300 frame {index} reports a 5 V rail of {record.v5_voltage_mv} mV, outside "
                    "the 4.0-7.0 V logic rail envelope (rov_parameters.h:21)",
                )
            if record.tether_current_ma > 25000:
                return CheckResult(
                    name,
                    False,
                    f"0x300 frame {index} reports a tether current of {record.tether_current_ma} mA, "
                    "above the 25 A limit (rov_parameters.h:18)",
                )
            if len(record.v12_current_ma) != 4:
                return CheckResult(
                    name,
                    False,
                    f"0x300 frame {index} carries {len(record.v12_current_ma)} 12 V brick reading(s); "
                    "4 are required (rov_parameters.h:25)",
                )
            for brick, current_ma in enumerate(record.v12_current_ma, start=1):
                if current_ma > 25000:
                    return CheckResult(
                        name,
                        False,
                        f"0x300 frame {index} 12 V brick {brick} draws {current_ma} mA, above the "
                        "25 A limit (rov_parameters.h:26)",
                    )
            if not -400 <= record.pcb_temp_c_tenths <= 1200:
                return CheckResult(
                    name,
                    False,
                    f"0x300 frame {index} reports a PCB temperature of "
                    f"{record.pcb_temp_c_tenths / 10.0:.1f} C",
                )
            if record.status_flags & 0x0001:
                return CheckResult(
                    name,
                    False,
                    f"0x300 frame {index} has the fault bit set (status_flags=0x"
                    f"{record.status_flags:04X}); node 3 latched a fault and broadcast a 0x005 eFuse "
                    "alert (node3 app.c:185-195), which trips the control board brake "
                    "(node2 app.c:121-126).",
                )
        last = records[-1]
        return CheckResult(
            name,
            True,
            f"{len(records)} frame(s) decoded in {duration_s:.1f} s: tether="
            f"{last.tether_voltage_mv / 1000.0:.1f} V @ {last.tether_current_ma / 1000.0:.2f} A, "
            f"5 V rail={last.v5_voltage_mv / 1000.0:.2f} V @ {last.v5_current_ma / 1000.0:.2f} A, "
            f"12 V bricks={last.v12_current_ma} mA, pcb={last.pcb_temp_c_tenths / 10.0:.1f} C, "
            f"status_flags=0x{last.status_flags:04X}",
        )

    # -- vehicle-wide safety ----------------------------------------------
    def test_emergency_break(
        self,
        timeout_s: float = EMERGENCY_TIMEOUT_S,
        sim_rate: float = SIM_MS_PER_WALL_S,
        slack_s: float = SIM_WALL_SLACK_S,
        unauthorized_window_s: float = UNAUTHORIZED_FRAME_WINDOW_S,
        estop_window_s: float = ESTOP_LATCH_WINDOW_S,
        first_output_timeout_s: float = FIRST_OUTPUT_TIMEOUT_S,
    ) -> CheckResult:
        """
        Trip the emergency break with the authorized frame and require a latch.

        ``nodes/node2_control_board/Core/Src/app.c:110-119`` only reacts to 0x001
        when the payload begins ``0xAA 0x55``; it then trips the hardware brake,
        sets ``ESC_STATE_DISARMED``, and forces neutral.

        The post-ESTOP sub-claim is the whole reason this check is hard, and it is
        worthless without a precondition.  ``app.c:127-129`` refuses thruster
        commands whenever ``g_esc_state != ESC_STATE_ACTIVE`` - not only while the
        break is active - and ``app.c:192-193`` forces neutral for the whole
        arming period.  So "a 1800 us command afterwards did not resume thrust"
        proves the latch *only* if the board was demonstrably ACTIVE and accepting
        thrust immediately before the trip.  Against a board that never went
        ACTIVE the same observation is what a disarmed board always looks like.

        This check therefore arranges its own precondition and fails rather than
        dropping the claim:

        1. the brake is not already latched, so a later trip is attributable to
           this frame;
        2. an *unauthorized* payload (``0xAA 0x56``) does NOT trip it - this is
           what makes the trip meaningful instead of coincidental;
        3. the board is driven to a **non-neutral readback**, which is the proof
           that it was ACTIVE and accepting thrust.  If that cannot be observed
           within its own bounded budget the check FAILS and says the post-ESTOP
           claim is unobservable; it does not report a partial pass;
        4. exactly ``b"\\xAA\\x55\\x01"`` latches the brake with all eight channels
           at neutral;
        5. a full-thrust command sent afterwards does not resume thrust - now a
           statement about the latch, because step 3 established the board would
           otherwise have moved.

        ``sim_rate``, ``slack_s``, ``unauthorized_window_s`` and ``estop_window_s``
        exist so the tests can drive these branches without waiting out 3000 ms of
        virtual time; production callers use the module defaults.
        """
        name = "emergency_break"
        refused = self._require_output_readback(name)
        if refused is not None:
            return refused
        initial = self._first_output(timeout_s)
        if initial is None:
            return self._no_readback_failure(name, timeout_s)
        if initial.brake_active:
            return CheckResult(
                name,
                False,
                f"the emergency brake was already tripped at sim_time_ms={initial.sim_time_ms} "
                "before this check sent anything, so a trip could not be attributed to the "
                "authorized 0xAA 0x55 signature. Restart the engine to clear the latch and re-run.",
            )

        self.backend.send_frame(CAN_ID_EMERGENCY_BREAK, UNAUTHORIZED_FRAME)
        false_trips: List[OutputStatus] = []
        self._pump(
            time.monotonic() + unauthorized_window_s,
            on_output=lambda snapshot: false_trips.append(snapshot)
            if snapshot.brake_active
            else None,
        )
        if false_trips:
            return CheckResult(
                name,
                False,
                f"an unauthorized 0xAA 0x56 frame tripped the brake at sim_time_ms="
                f"{false_trips[0].sim_time_ms}; app.c:112 requires the 0xAA 0x55 signature, so the "
                "authorization check is not holding.",
            )

        armed = self._arm_before_trip(sim_rate, slack_s, first_output_timeout_s)
        if not isinstance(armed, OutputStatus):
            return armed
        if not self._release_to_neutral(self._ramp_budget_s(POST_ESTOP_PROBE_PWM_US, sim_rate, slack_s)):
            return CheckResult(
                name,
                False,
                f"the board accepted thrust ({list(armed.pwms)} at sim_time_ms={armed.sim_time_ms}) "
                f"but did not return to {NEUTRAL_US} us before the emergency frame, so the trip "
                "would be observed from an unknown starting state",
            )

        self.backend.send_frame(CAN_ID_EMERGENCY_BREAK, EMERGENCY_FRAME)
        latched = self._await_output(lambda snapshot: snapshot.brake_active, timeout_s)
        if latched is None:
            last = self.backend.latest_outputs()
            seen = "no readback" if last is None else last.describe()
            return CheckResult(
                name,
                False,
                f"the authorized frame {EMERGENCY_FRAME.hex(' ').upper()} did not latch the "
                f"emergency brake within {timeout_s:.1f} s ({seen}). app.c:110-119 should have "
                "tripped bsp_emergency_brake_trip and forced neutral.",
            )
        if tuple(latched.pwms) != NEUTRAL_PWMS:
            return CheckResult(
                name,
                False,
                f"the brake latched at sim_time_ms={latched.sim_time_ms} but the channels are not "
                f"all at {NEUTRAL_US} us: {list(latched.pwms)}. app.c:118 calls "
                "node2_force_neutral(), so every channel must read neutral.",
            )
        resumed: List[OutputStatus] = []
        probe = ThrusterCommand(pwm_us=[POST_ESTOP_PROBE_PWM_US] * NUM_THRUSTERS).pack()
        self._pump(
            time.monotonic() + estop_window_s,
            send=lambda: self.backend.send_frame(CAN_ID_THRUSTER_CMD, probe),
            on_output=lambda snapshot: resumed.append(snapshot)
            if tuple(snapshot.pwms) != NEUTRAL_PWMS
            else None,
        )
        if resumed:
            return CheckResult(
                name,
                False,
                f"a {POST_ESTOP_PROBE_PWM_US} us command resumed thrust at sim_time_ms="
                f"{resumed[0].sim_time_ms} ({list(resumed[0].pwms)}); app.c:116 sets "
                "ESC_STATE_DISARMED and app.c:129 refuses commands while the break is active, so "
                "the latch is not holding.",
            )
        return CheckResult(
            name,
            True,
            f"{EMERGENCY_FRAME.hex(' ').upper()} latched the brake at sim_time_ms="
            f"{latched.sim_time_ms} with all channels at {NEUTRAL_US} us. The unauthorized "
            f"{UNAUTHORIZED_FRAME.hex(' ').upper()} frame was correctly ignored. The latch claim is "
            f"earned: the board was observed at {list(armed.pwms)} (ESC ACTIVE, accepting thrust) "
            f"at sim_time_ms={armed.sim_time_ms}, immediately before the trip, and a "
            f"{POST_ESTOP_PROBE_PWM_US} us command afterwards did not resume thrust.",
        )

    def _arm_before_trip(
        self, sim_rate: float, slack_s: float, first_output_timeout_s: float
    ) -> object:
        """
        Drive the board to a non-neutral readback, or explain why that is impossible.

        This is the precondition the post-ESTOP sub-claim needs, and this check
        establishes it for itself rather than assuming it.  Returns the
        ``OutputStatus`` that proves the board was ACTIVE, or a FAILED
        ``CheckResult`` - never a silent skip, because a skipped precondition
        would turn the latch claim into an unearned one.
        """
        name = "emergency_break_precondition"
        probe = ThrusterCommand(pwm_us=[POST_ESTOP_PROBE_PWM_US] * NUM_THRUSTERS).pack()
        first = self._first_output(first_output_timeout_s)
        if first is None:
            return self._no_readback_failure(name, first_output_timeout_s)
        if first.brake_active:
            return CheckResult(
                name,
                False,
                f"the brake is tripped at sim_time_ms={first.sim_time_ms}, so the board is "
                "permanently DISARMED (app.c:116) and can never accept thrust again. Restart the "
                "engine to clear the latch.",
            )
        remaining_sim_ms = max(0, ESC_ARMING_SIM_MS - first.sim_time_ms)
        # Budget covers the rest of the arming window plus the slew ramp, both
        # converted from virtual time to wall time.
        budget_s = (
            remaining_sim_ms / 1000.0
            + abs(POST_ESTOP_PROBE_PWM_US - NEUTRAL_US) / PWM_SLEW_RATE_US_PER_MS
        ) / max(1e-6, sim_rate) + slack_s
        armed = self._await_output(
            lambda snapshot: any(pulse != NEUTRAL_US for pulse in snapshot.pwms),
            budget_s,
            send=lambda: self.backend.send_frame(CAN_ID_THRUSTER_CMD, probe),
        )
        if armed is None:
            last = self.backend.latest_outputs()
            seen = "no 0x7FE readback" if last is None else last.describe()
            return CheckResult(
                name,
                False,
                f"the board never accepted a {POST_ESTOP_PROBE_PWM_US} us command within "
                f"{budget_s:.1f} s of wall time, starting from sim_time_ms={first.sim_time_ms} "
                f"({seen}). The post-ESTOP latch claim is therefore UNOBSERVABLE against a board "
                "that never reached ESC_STATE_ACTIVE: app.c:127-129 refuses thruster commands "
                "whenever the ESC state is not ACTIVE and app.c:192-193 holds neutral throughout "
                "arming, so a 1800 us frame after the trip proves nothing here. This check is "
                "reported as FAILED rather than as a partial pass.",
            )
        return armed

    # -- ad hoc ------------------------------------------------------------
    def raw_send(self, can_id: int, payload: bytes) -> CheckResult:
        """
        Write one operator-supplied raw frame and report only that it was written.

        No readback is claimed: asserting "the frame went out" is a statement
        about the transport, so that is exactly what is asserted, and the text
        names the real destination - a UART port, which carries no arbitration id,
        is described as such rather than being reported as "CAN 0xNNN".

        ``raw_send`` is the one place a non-SIL transport may still write.  That
        is deliberate: the operator typed the exact id and bytes, and the tool is a
        pipe for them.  Every *check* refuses to actuate without a readback; this
        does not second-guess an explicit frame, it just refuses to misdescribe it.

        The two safety-critical IDs are gated on their authorization signature,
        because the firmware silently ignores an unsigned frame
        (``node2 app.c:112`` and ``:121``).  Writing one and reporting success
        would be the same defect as a check that reports a pass it did not earn.
        """
        name = f"raw_send_0x{can_id:03X}" if isinstance(can_id, int) and can_id >= 0 else f"raw_send_{can_id}"
        payload = validate_payload(payload)
        can_id = validate_can_id(can_id)
        shown = payload.hex(" ").upper() or "<empty>"
        required = SAFETY_SIGNATURES.get(can_id)
        if required is not None and not payload.startswith(required):
            return CheckResult(
                name,
                False,
                f"{describe_can_id(can_id)} is only acted on when the payload starts with the "
                f"authorization signature {required.hex(' ').upper()} (node2 app.c:112/:121), so "
                f"the {len(payload)}-byte payload {shown} would have been ignored. Nothing was "
                "written; add the signature or pick a different CAN id.",
            )
        self.backend.send_frame(can_id, payload)
        return CheckResult(
            name,
            True,
            f"wrote {len(payload)} byte(s) {shown} to {self.backend.describe_destination(can_id)} "
            "with no transport error; this asserts the write only, not any vehicle response",
        )

    def sniff(self, duration_s: float = 1.0) -> CheckResult:
        """Decode and print every frame seen in the window; pass iff there was one."""
        name = "sniff"
        counts: Dict[int, int] = {}
        printed: Dict[int, int] = {}

        def on_frame(can_id: int, payload: bytes) -> None:
            counts[can_id] = counts.get(can_id, 0) + 1
            if printed.get(can_id, 0) < 5:
                printed[can_id] = printed.get(can_id, 0) + 1
                print(f"  [0x{can_id:03X}] {len(payload):2d} B {payload[:16].hex(' ').upper()}")

        self._pump(time.monotonic() + max(0.0, duration_s), on_frame=on_frame)
        if not counts:
            return CheckResult(
                name,
                False,
                f"no frames decoded in {duration_s:.1f} s; a live SIL engine publishes 0x7FE at "
                "100 Hz plus telemetry (sil_bridge_server.c:339-412), so silence means the engine "
                "is not running or the connection is dead",
            )
        summary = ", ".join(f"0x{can_id:03X} x{count}" for can_id, count in sorted(counts.items()))
        return CheckResult(
            name, True, f"decoded {sum(counts.values())} frame(s) in {duration_s:.1f} s: {summary}"
        )

    # -- aggregate ---------------------------------------------------------
    def run_full_smoke(
        self,
        pwm_us: int = SMOKE_SINGLE_PWM_US,
        all_pwm_us: int = SMOKE_ALL_PWM_US,
        solenoid_mask: int = SMOKE_SOLENOID_MASK,
    ) -> StimulusReport:
        """
        Run the whole vehicle in one report, in the only order that is meaningful.

        * The arming gate is first, and nothing before it may send a thruster
          command.  The 3000 ms window (``app.c:24``) is measured on the engine's
          virtual clock, so it closes whether or not anyone is watching; once it
          closes the gate can no longer be observed and the check would be
          reporting on something it never saw.
        * The emergency break is last.  It latches for the life of the engine
          (``rov_safety.c`` has no clear path and ``app.c:116`` sets
          ``ESC_STATE_DISARMED``), so every check after it would only be observing
          a permanently disarmed board.
        """
        sequence: List[Tuple[str, Callable[[], CheckResult]]] = [
            ("node2_arming", self.test_node2_arming),
            ("node1_env", self.monitor_node1_env),
            ("node3_power", self.monitor_node3_power),
            ("node2_depth", self.monitor_node2_depth),
            ("node2_pwm", lambda: self.test_node2_pwm(0, pwm_us)),
            ("node2_all", lambda: self.test_node2_all(all_pwm_us)),
            ("node2_solenoid", lambda: self.test_node2_solenoid(solenoid_mask)),
            ("emergency_break", self.test_emergency_break),
        ]
        return self.run_plan(sequence)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

ACTION_FLAGS = (
    "--node1-env",
    "--node2-arm",
    "--node2-thruster",
    "--node2-all",
    "--node2-depth",
    "--node2-solenoid",
    "--node3-power",
    "--emergency-break",
    "--raw-send",
    "--sniff",
)


def _raw_frame(text: str) -> Tuple[int, bytes]:
    """Parse ``0x100:AABB`` into ``(can_id, payload)``."""
    if ":" not in text:
        raise argparse.ArgumentTypeError(
            f"--raw-send expects CAN_ID:HEXBYTES (for example 0x001:AA5501), got {text!r}"
        )
    id_text, _, hex_text = text.partition(":")
    try:
        can_id = int(id_text, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid CAN id {id_text!r}: {exc}") from exc
    try:
        payload = bytes.fromhex(hex_text.replace(" ", ""))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid hex payload {hex_text!r}: {exc}") from exc
    if len(payload) > MAX_SIL_PAYLOAD:
        raise argparse.ArgumentTypeError(
            f"payload is {len(payload)} bytes; a SIL frame carries at most {MAX_SIL_PAYLOAD}"
        )
    return can_id, payload


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="can_stimulus.py",
        description=(
            "Inject stimulus at the X19 CAN boundary and verify the result. "
            "Exits nonzero unless every requested check passed."
        ),
    )
    parser.add_argument(
        "--mode",
        choices=("sil", "can", "uart"),
        default="sil",
        help=(
            "sil drives the native SIL engine over TCP and is the only mode that can verify an "
            "actuation, because the CAN 0x7FE output-status readback exists only there. can and "
            "uart are RECEIVE-ONLY: they observe a real link, and every check that would need to "
            "actuate a thruster or a valve refuses before sending a frame. Both need optional "
            "hardware packages (python-can, pyserial)."
        ),
    )
    parser.add_argument("--host", default=DEFAULT_SIL_HOST, help="SIL engine host")
    parser.add_argument("--port", type=int, default=DEFAULT_SIL_PORT, help="SIL engine TCP port")
    parser.add_argument("--interface", default="can0", help="python-can interface for --mode can")
    parser.add_argument("--uart-port", default="/dev/ttyUSB0", help="serial port for --mode uart")
    parser.add_argument("--baudrate", type=int, default=115200, help="baud rate for --mode uart")
    parser.add_argument(
        "--attach-existing",
        action="store_true",
        help=(
            "deliberately drive the engine that is already listening on the port instead of "
            "refusing to touch an engine this run did not start. The default is to refuse, "
            "because every check here actuates and the emergency break latches the brake of "
            "whatever engine it reaches, for the life of that engine. Cannot be combined with "
            "--auto."
        ),
    )

    parser.add_argument("--node1-env", action="store_true", help="monitor 0x210 enclosure telemetry")
    parser.add_argument("--node2-arm", action="store_true", help="verify the 3000 ms ESC arming gate")
    parser.add_argument(
        "--node2-thruster",
        nargs=2,
        type=int,
        metavar=("CHANNEL", "PULSE_US"),
        help="drive one thruster channel and require it to hold",
    )
    parser.add_argument(
        "--node2-all", type=int, metavar="PULSE_US", help="drive all eight thruster channels"
    )
    parser.add_argument(
        "--node2-depth", type=float, metavar="SECONDS", help="monitor 0x200 depth telemetry"
    )
    parser.add_argument(
        "--node2-solenoid", type=int, metavar="MASK", help="actuate a solenoid mask (one coil per valve)"
    )
    parser.add_argument("--node3-power", action="store_true", help="monitor 0x300 power telemetry")
    parser.add_argument("--emergency-break", action="store_true", help="trip and verify the emergency brake")
    parser.add_argument(
        "--raw-send",
        action="append",
        type=_raw_frame,
        metavar="ID:HEX",
        help="write one raw frame, for example 0x100:D405DC05DC05DC05DC05DC05DC05DC05",
    )
    parser.add_argument(
        "--sniff", type=float, metavar="SECONDS", help="decode and print every frame for a while"
    )
    parser.add_argument(
        "--auto", action="store_true", help="run the whole vehicle smoke sequence"
    )
    return parser


def build_backend(args: argparse.Namespace) -> _BaseBackend:
    if args.mode == "sil":
        return SilSocketBackend(
            args.host,
            args.port,
            auto_start=True,
            mode="sil",
            attach_existing=bool(getattr(args, "attach_existing", False)),
        )
    if args.mode == "can":
        return PythonCanBackend(interface=args.interface)
    if args.mode == "uart":
        return UartBackend(port=args.uart_port, baudrate=args.baudrate)
    raise ValueError(f"unknown mode {args.mode!r}")


def target_label(args: argparse.Namespace) -> str:
    """Human-readable description of what this invocation talks to."""
    if args.mode == "sil":
        return f"{args.host}:{args.port}"
    if args.mode == "can":
        return f"interface={args.interface}"
    return f"{args.uart_port} at {args.baudrate} baud"


def run_selected_action(tester: VehicleStimulusTester, args: argparse.Namespace) -> StimulusReport:
    """
    Map every supplied flag to exactly one check and return them all.

    Each dispatch goes through ``tester.guard``, so a check that raises becomes a
    failed entry in the report rather than an exception escaping the CLI - the
    prototype's ``AttributeError: 'Namespace' object has no attribute 'port'``
    must not be able to reach a caller as anything but a reported failure.
    """
    plan: List[Tuple[str, Callable[[], CheckResult]]] = []
    if getattr(args, "node1_env", False):
        plan.append(("node1_env", lambda: tester.monitor_node1_env()))
    if getattr(args, "node2_arm", False):
        plan.append(("node2_arming", tester.test_node2_arming))
    if getattr(args, "node2_thruster", None) is not None:
        channel, pulse_us = args.node2_thruster
        plan.append(
            ("node2_pwm", lambda: tester.test_node2_pwm(int(channel), int(pulse_us)))
        )
    if getattr(args, "node2_all", None) is not None:
        plan.append(("node2_all", lambda: tester.test_node2_all(int(args.node2_all))))
    if getattr(args, "node2_depth", None) is not None:
        plan.append(("node2_depth", lambda: tester.monitor_node2_depth(float(args.node2_depth))))
    if getattr(args, "node2_solenoid", None) is not None:
        plan.append(
            ("node2_solenoid", lambda: tester.test_node2_solenoid(int(args.node2_solenoid)))
        )
    if getattr(args, "node3_power", False):
        plan.append(("node3_power", lambda: tester.monitor_node3_power()))
    if getattr(args, "emergency_break", False):
        plan.append(("emergency_break", tester.test_emergency_break))
    for index, (can_id, payload) in enumerate(getattr(args, "raw_send", None) or []):
        plan.append(
            (f"raw_send_0x{can_id:03X}_{index}", lambda cid=can_id, data=payload: tester.raw_send(cid, data))
        )
    if getattr(args, "sniff", None) is not None:
        plan.append(("sniff", lambda: tester.sniff(float(args.sniff))))
    return tester.run_plan(plan)


def print_report(report: StimulusReport, include_checks: bool = True) -> None:
    """
    The one way the operator-facing report text is produced.

    ``include_checks=False`` is for a caller that has already streamed the
    per-check lines as they completed, which is what ``main`` does; it still
    prints the ``Summary:`` line and the whole failure block.
    """
    print(report.format(include_checks=include_checks))


def selected_action_flags(args: argparse.Namespace) -> List[str]:
    """
    Which action flags were supplied.

    A flag counts as selected unless it is absent or a bare ``False``.  Truthiness
    is the wrong test: ``--sniff 0``, ``--node2-depth 0`` and ``--node2-all 0`` are
    real requests, and reporting them as "no action was selected" would hide the
    operator's intent behind a usage error.  ``run_selected_action`` already uses
    ``is not None``, and the two must agree.
    """
    selected = []
    for flag in ACTION_FLAGS:
        value = getattr(args, flag.lstrip("-").replace("-", "_"), None)
        if value is None or value is False:
            continue
        selected.append(flag)
    return selected


def main(argv: Optional[Sequence[str]] = None, backend_factory=None) -> int:
    """
    Canonical CLI entry point. Returns 0 only when every requested check passed.

    Four independent layers guarantee a crashed or timed-out check can never look
    like a success: each check converts its own observation failures into failed
    results, ``tester.guard`` converts anything that still escapes into a failed
    result, and the ``except`` clauses here turn a failure to reach the transport,
    or to run the sequence at all, into a nonzero exit.  On top of that, every
    result is streamed as it completes, so a run that is killed part way through
    still leaves per-check evidence behind.

    ``backend_factory`` is resolved from the module global on every call rather
    than captured as a default argument, so a test can substitute a transport.
    """
    backend_factory = backend_factory or build_backend
    args = build_parser().parse_args(argv)
    selected = selected_action_flags(args)
    if args.auto and selected:
        # --auto runs the whole sequence; a flag alongside it would be dropped in
        # silence, which is how an operator ends up believing a check ran.
        print(
            f"[WARN] --auto runs the full sequence, so these flags are ignored: "
            f"{', '.join(selected)}. Drop --auto to run only the flags you listed."
        )
        selected = []
    if not args.auto and not selected:
        print(
            "[FAIL] no action was selected, so nothing was verified. Pass --auto for the whole "
            f"vehicle, or one of: {', '.join(ACTION_FLAGS)}."
        )
        return 2
    if args.auto and args.mode != "sil":
        print(
            f"[FAIL] --auto needs the {describe_can_id(CAN_ID_SIL_OUTPUT_STATUS)} output-status "
            f"readback, which is a SIL-only channel, so it cannot run in --mode {args.mode}. "
            "--auto drives thrusters and a pneumatic valve; running it where the result cannot be "
            "read back would be a blind actuation. Use --mode sil, or select the receive-only "
            "checks individually."
        )
        return 2
    if args.auto and args.attach_existing:
        # Refused in the pre-flight, before a transport is even built, for the same
        # reason the ownership check is: --auto drives all eight thrusters, actuates
        # a valve, and latches the brake, so there is no harmless version of it
        # against an engine this run did not start. The operator names the checks
        # they want instead, and the refusal names the way out.
        print(
            "[FAIL] refused: --auto cannot be combined with --attach-existing. --auto drives all "
            "eight thrusters, actuates a valve, and latches the emergency brake, so running it "
            "against an engine this run did not start commandeers somebody else's vehicle. Drop "
            "--auto and name the checks you want to run against it."
        )
        return 2

    target = target_label(args)
    print(
        f"X19 CAN stimulus: mode={args.mode} target={target} "
        f"({'full smoke' if args.auto else 'selected checks'})"
    )
    backend: Optional[_BaseBackend] = None
    try:
        backend = backend_factory(args)
        backend.connect()
    except ForeignEngineError as exc:
        # Its own branch, not the generic one below: the generic wording ("could
        # not open the transport") describes a fault, and this is a decision. A
        # refusal dressed as a connection failure is how the precondition becomes
        # invisible to the operator reading the log.
        print(f"[FAIL] refused: {exc}")
        if backend is not None:
            with contextlib.suppress(Exception):
                backend.close()
        return 2
    except Exception as exc:  # noqa: BLE001 - a failed connect is a failed run
        print(
            f"[FAIL] could not open the {args.mode} transport at {target}: "
            f"{type(exc).__name__}: {exc}"
        )
        if backend is not None:
            with contextlib.suppress(Exception):
                backend.close()
        return 1

    # Streamed as each check completes: a 30 s run must not print nothing until
    # the end, or a killed process leaves no evidence of what had already run.
    tester = VehicleStimulusTester(backend, on_result=lambda result: print(format_result(result)))
    try:
        if args.auto:
            report = tester.run_full_smoke()
        else:
            report = run_selected_action(tester, args)
    except Exception as exc:  # noqa: BLE001 - never report a crash as success
        print(f"[FAIL] the stimulus run aborted: {type(exc).__name__}: {exc}")
        return 1
    finally:
        with contextlib.suppress(Exception):
            backend.close()

    # One implementation of the operator-facing text, not a second copy of it:
    # every per-check line was already streamed above, so this is exactly
    # format(include_checks=False). Spelling the summary out again here is how
    # the copy the operator reads and the one the tests exercise drift apart.
    print_report(report, include_checks=False)
    return 0 if report.ok else 1


if __name__ == "__main__":
    sys.exit(main())
