"""
Reusable fixtures for driving the native X19 SIL bridge server from Python tests.

The SIL wire format is defined exactly once, in
``tests/sil_companion_bridge/sil_protocol.py``.  This module only adds process
and socket plumbing on top of it; it never redefines packet formats.

Usage from the repository root::

    python -m unittest tests.sil_stimulus.test_protocol_roundtrip -v
"""

from __future__ import annotations

import os
import socket
import subprocess
import sys
import time
import unittest
import weakref
from pathlib import Path
from typing import List, Optional, Tuple

# tests/sil_stimulus/sil_test_support.py -> tests/sil_stimulus -> tests -> repo root
REPO_ROOT = Path(__file__).resolve().parents[2]
COMPANION_BRIDGE_DIR = REPO_ROOT / "tests" / "sil_companion_bridge"
if str(COMPANION_BRIDGE_DIR) not in sys.path:
    sys.path.insert(0, str(COMPANION_BRIDGE_DIR))

from sil_protocol import (  # noqa: E402  (path setup must precede the import)
    CAN_ID_EMERGENCY_BREAK,
    CAN_ID_NAV_TELEMETRY,
    CAN_ID_SIL_OUTPUT_STATUS,
    CAN_ID_SOLENOID_CMD,
    CAN_ID_THRUSTER_CMD,
    SIL_PACKET_SIZE,
    SolenoidCommand,
    ThrusterCommand,
    pack_sil_can_frame,
    unpack_sil_can_frame,
    unpack_sil_output_status,
)

SERVER_STEM = "sil_bridge_server"
SERVER_EXE = Path(
    os.environ.get(
        "X19_SIL_SERVER",
        str(REPO_ROOT / "build-native" / "tests" / f"{SERVER_STEM}.exe"),
    )
)
SERVER_START_TIMEOUT_S = 5.0
SERVER_SHUTDOWN_TIMEOUT_S = 2.0

_BUILD_DIR_CANDIDATES = ("build-native", "build", "build-test", "build-host")

# socket.socket is a C type without a __dict__, so read-ahead buffers live in a
# weak map keyed by socket identity and disappear when the socket is closed.
_RX_BUFFERS: "weakref.WeakKeyDictionary[socket.socket, bytearray]" = weakref.WeakKeyDictionary()

__all__ = [
    "CAN_ID_EMERGENCY_BREAK",
    "CAN_ID_NAV_TELEMETRY",
    "CAN_ID_SIL_OUTPUT_STATUS",
    "CAN_ID_SOLENOID_CMD",
    "CAN_ID_THRUSTER_CMD",
    "REPO_ROOT",
    "SERVER_EXE",
    "SERVER_STEM",
    "SIL_PACKET_SIZE",
    "SilServerProcess",
    "SolenoidCommand",
    "ThrusterCommand",
    "free_tcp_port",
    "pack_sil_can_frame",
    "recv_frames",
    "require_server_executable",
    "send_raw_frame",
    "server_executable",
    "unpack_sil_can_frame",
    "unpack_sil_output_status",
]


def server_executable() -> Optional[Path]:
    """
    Locate the native ``sil_bridge_server`` binary.

    ``X19_SIL_SERVER`` wins when set.  Returns ``None`` when the binary has not
    been built so callers can skip instead of failing spuriously.
    """
    configured = os.environ.get("X19_SIL_SERVER")
    if configured:
        candidate = Path(configured)
        return candidate if candidate.is_file() else None

    for build_dir in _BUILD_DIR_CANDIDATES:
        for name in (f"{SERVER_STEM}.exe", SERVER_STEM):
            candidate = REPO_ROOT / build_dir / "tests" / name
            if candidate.is_file():
                return candidate
    return None


def require_server_executable() -> Path:
    """Return the server binary or raise ``unittest.SkipTest`` with build instructions."""
    resolved = server_executable()
    if resolved is None:
        raise unittest.SkipTest(
            "Native sil_bridge_server was not found, so server-dependent SIL tests are skipped. "
            "Build it with 'cmake -B build-native -G Ninja' then "
            "'cmake --build build-native --target sil_bridge_server', or point X19_SIL_SERVER "
            f"at an existing binary. Searched {REPO_ROOT / '<build dir>' / 'tests'} and the default "
            f"path {SERVER_EXE}."
        )
    return resolved


def free_tcp_port() -> int:
    """Reserve an ephemeral loopback port and release it for immediate reuse."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return int(probe.getsockname()[1])


def send_raw_frame(sock: socket.socket, can_id: int, payload: bytes) -> None:
    """Wrap ``payload`` in a SIL frame and push the whole packet down the wire."""
    sock.sendall(pack_sil_can_frame(can_id, payload))


def _rx_buffer(sock: socket.socket) -> bytearray:
    """Per-socket read-ahead buffer so partial TCP reads survive across calls."""
    buffer = _RX_BUFFERS.get(sock)
    if buffer is None:
        buffer = bytearray()
        _RX_BUFFERS[sock] = buffer
    return buffer


def recv_frames(sock: socket.socket, timeout: float) -> List[Tuple[int, bytes]]:
    """
    Drain-window read: collect every complete SIL frame available for up to
    ``timeout`` seconds.

    This is not a read-one-frame primitive.  It keeps reading until the window
    closes, so a burst is returned in one call and an idle period costs the full
    ``timeout``.  Return value semantics:

    * Frames arrive in order, and only whole ``SIL_PACKET_SIZE`` packets are
      returned; a trailing partial packet is retained, not emitted.
    * An exhausted window with nothing complete yields ``[]`` so callers can
      poll without exception handling.
    * A closed peer ends the window early and returns whatever was collected.

    Cross-call buffering: leftover bytes are kept per-socket in
    ``_RX_BUFFERS`` (a :class:`weakref.WeakKeyDictionary`), so a frame straddling
    two calls is reassembled.  The buffer must not be per-call; that would drop
    the partial tail and desynchronise every later frame boundary.

    Buffer-poisoning policy: a malformed packet (bad magic, or a length field
    above 64) raises ``ValueError`` from :func:`unpack_sil_can_frame` and the
    offending bytes are deliberately *left in place* with no resynchronisation
    scan.  The stream is therefore unrecoverable at that point: callers must
    close and reopen the socket rather than keep reading, because the framing
    offset is no longer known.  There is no attempt to skip to the next magic.
    """
    buffer = _rx_buffer(sock)
    frames: List[Tuple[int, bytes]] = []
    deadline = time.monotonic() + max(0.0, timeout)

    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break
        sock.settimeout(remaining)
        try:
            chunk = sock.recv(SIL_PACKET_SIZE * 4)
        except (socket.timeout, TimeoutError):
            break
        if not chunk:
            break

        buffer.extend(chunk)
        while len(buffer) >= SIL_PACKET_SIZE:
            frames.append(unpack_sil_can_frame(bytes(buffer[:SIL_PACKET_SIZE])))
            del buffer[:SIL_PACKET_SIZE]

    return frames


class SilServerProcess:
    """
    Context-managed lifecycle for the native ``sil_bridge_server``.

    Launched with the flags the C server actually supports (see
    ``tests/sil_bridge_server.c``): ``--port`` and ``--cycles`` (``0`` means run
    indefinitely).  ``stop()`` is always safe to call, so a failing test can
    never leak a simulator process.
    """

    def __init__(self, executable: Path, port: Optional[int] = None) -> None:
        self.executable = Path(executable)
        self.port = int(port) if port is not None else free_tcp_port()
        self.process: Optional[subprocess.Popen] = None

    @property
    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def start(self) -> None:
        if self.is_running:
            raise RuntimeError(f"SIL server is already running on port {self.port}")

        self.process = subprocess.Popen(
            [str(self.executable), "--port", str(self.port), "--cycles", "0"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )

        deadline = time.monotonic() + SERVER_START_TIMEOUT_S
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError(
                    f"SIL server exited with code {self.process.returncode} before listening "
                    f"on port {self.port}"
                )
            try:
                probe = socket.create_connection(("127.0.0.1", self.port), timeout=0.1)
            except OSError:
                time.sleep(0.02)
            else:
                probe.close()
                return

        self.stop()
        raise RuntimeError(f"SIL server did not listen on port {self.port}")

    def connect_socket(self, timeout: float = 3.0) -> socket.socket:
        """Open a client connection to the running server (caller closes it)."""
        if not self.is_running:
            raise RuntimeError("SIL server is not running; call start() first")
        return socket.create_connection(("127.0.0.1", self.port), timeout=timeout)

    def stop(self) -> None:
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=SERVER_SHUTDOWN_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=SERVER_SHUTDOWN_TIMEOUT_S)
        self.process = None

    def __enter__(self) -> "SilServerProcess":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()
