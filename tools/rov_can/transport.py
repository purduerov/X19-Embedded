"""CAN transports behind one interface.

SocketCanTransport  - real bus via Linux SocketCAN (can0), CAN FD with BRS.
SilTcpTransport     - host SIL bridge over TCP (73-byte ROVC framing).
"""

from __future__ import annotations

import socket
import sys
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Optional, Tuple

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SIL_DIR = _REPO_ROOT / "tests" / "sil_companion_bridge"

MAX_FD_PAYLOAD = 64
DEFAULT_SIL_PORT = 5555

Frame = Tuple[int, bytes]


class CanTransport(ABC):
    """Send/receive CAN frames; usable as a context manager."""

    @abstractmethod
    def send_message(self, can_id: int, payload: bytes) -> None: ...

    @abstractmethod
    def receive_message(self, timeout_s: float = 1.0) -> Optional[Frame]:
        """Return (can_id, payload), or None on timeout. Never blocks past timeout_s."""

    @abstractmethod
    def close(self) -> None: ...

    def __enter__(self) -> "CanTransport":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def _check(can_id: int, payload: bytes) -> bytes:
    if not 0 <= can_id <= 0x7FF:
        raise ValueError(f"CAN id 0x{can_id:X} outside 11-bit range")
    payload = bytes(payload)
    if len(payload) > MAX_FD_PAYLOAD:
        raise ValueError(f"payload {len(payload)} B exceeds CAN FD max {MAX_FD_PAYLOAD} B")
    return payload


class SocketCanTransport(CanTransport):
    """Linux SocketCAN. Fails loudly (not silently) anywhere SocketCAN is absent."""

    def __init__(self, interface: str = "can0") -> None:
        if sys.platform != "linux":
            raise RuntimeError(
                f"SocketCAN needs Linux (running on {sys.platform}). "
                "Run this on the Raspberry Pi, or use --sil HOST[:PORT] for the simulator."
            )
        try:
            import can
        except ImportError as exc:
            raise RuntimeError("python-can is not installed: pip install python-can") from exc
        self._can = can
        try:
            self._bus = can.Bus(channel=interface, interface="socketcan", fd=True)
        except Exception as exc:
            raise RuntimeError(
                f"could not open {interface}: {exc}. Bring it up with: "
                f"sudo ip link set {interface} up type can bitrate 1000000 dbitrate 5000000 fd on"
            ) from exc
        self.interface = interface

    def send_message(self, can_id: int, payload: bytes) -> None:
        payload = _check(can_id, payload)
        self._bus.send(self._can.Message(arbitration_id=can_id, data=payload, is_extended_id=False, is_fd=True))

    def receive_message(self, timeout_s: float = 1.0) -> Optional[Frame]:
        msg = self._bus.recv(timeout=max(0.0, timeout_s))
        return None if msg is None else (int(msg.arbitration_id), bytes(msg.data))

    def close(self) -> None:
        try:
            self._bus.shutdown()
        except Exception:
            pass


class SilTcpTransport(CanTransport):
    """TCP link to the host SIL bridge (sil_bridge_server)."""

    def __init__(self, host: str = "127.0.0.1", port: int = DEFAULT_SIL_PORT, connect_timeout_s: float = 2.0) -> None:
        if str(_SIL_DIR) not in sys.path:
            sys.path.insert(0, str(_SIL_DIR))
        import sil_protocol

        self._proto = sil_protocol
        self._size = sil_protocol.SIL_PACKET_SIZE
        self._buf = b""
        try:
            self._sock = socket.create_connection((host, port), timeout=connect_timeout_s)
        except OSError as exc:
            raise RuntimeError(f"no SIL bridge listening on {host}:{port} ({exc})") from exc

    def send_message(self, can_id: int, payload: bytes) -> None:
        payload = _check(can_id, payload)
        self._sock.sendall(self._proto.pack_sil_can_frame(can_id, payload))

    def receive_message(self, timeout_s: float = 1.0) -> Optional[Frame]:
        self._sock.settimeout(max(0.0, timeout_s))
        while len(self._buf) < self._size:
            try:
                chunk = self._sock.recv(self._size - len(self._buf))
            except (socket.timeout, BlockingIOError):
                return None
            if not chunk:
                raise ConnectionError("SIL bridge closed the connection")
            self._buf += chunk
        frame, self._buf = self._buf[: self._size], self._buf[self._size :]
        return self._proto.unpack_sil_can_frame(frame)

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass


def open_transport(interface: str = "can0", sil: Optional[str] = None) -> CanTransport:
    """`sil` is 'HOST' or 'HOST:PORT' to use the simulator; otherwise SocketCAN."""
    if sil:
        host, _, port = sil.partition(":")
        return SilTcpTransport(host or "127.0.0.1", int(port) if port else DEFAULT_SIL_PORT)
    return SocketCanTransport(interface)
