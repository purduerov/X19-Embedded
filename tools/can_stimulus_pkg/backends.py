"""
Backend implementations for can_stimulus (SIL socket, python-can, UART).
"""

from __future__ import annotations

import argparse
import contextlib
import importlib.util
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Callable, List, Optional, Tuple

TOOLS_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = TOOLS_DIR.parent
SIL_BRIDGE_DIR = REPO_ROOT / "tests" / "sil_companion_bridge"
SIL_STIMULUS_DIR = REPO_ROOT / "tests" / "sil_stimulus"
SIL_DASHBOARD_DIR = REPO_ROOT / "tests" / "sil_dashboard"
for _path in (SIL_STIMULUS_DIR, SIL_BRIDGE_DIR, SIL_DASHBOARD_DIR):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from sil_protocol import CAN_ID_SIL_OUTPUT_STATUS, pack_sil_can_frame
from sil_test_support import (
    SERVER_SHUTDOWN_TIMEOUT_S,
    SERVER_START_TIMEOUT_S,
    SERVER_STEM,
    recv_frames,
    server_executable,
)

from .constants import (
    DEFAULT_RECV_TIMEOUT_S,
    DEFAULT_SIL_HOST,
    DEFAULT_SIL_PORT,
    MAX_SIL_PAYLOAD,
    SOCKET_TIMEOUT_S,
)
from .models import OutputStatus
from .protocol import describe_can_id, validate_can_id, validate_payload


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
            "tool will launch and reap one. To deliberately attach to the engine on "
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
