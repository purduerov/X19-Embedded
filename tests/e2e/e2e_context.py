"""End-to-End Test Harness & Shared Fixtures for X19-Embedded.

Provides test helpers for CLI execution, temporary scratch files, mock boards,
and SIL CAN TCP bridge communication.
"""

from __future__ import annotations

import contextlib
import os
import shutil
import socket
import stat
import struct
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from typing import Generator, Optional, Tuple

REPO_ROOT = Path(__file__).resolve().parent.parent.parent

# SIL CAN packet constants matching tests/sil_bridge_server.c
SIL_MAGIC_HEADER = 0x524F5643        # "ROVC" in ASCII
SIL_MAGIC_HEADER_LEGACY = 0x58313943 # "X19C" in ASCII
SIL_CAN_PACKET_FORMAT = "<IIB64s"    # magic(4), id(4), len(1), data(64) = 73 bytes
SIL_CAN_PACKET_SIZE = struct.calcsize(SIL_CAN_PACKET_FORMAT)

# Standard protocol CAN identifiers
CAN_ID_EMERGENCY_BREAK = 0x001
CAN_ID_EFUSE_FAULT_ALERT = 0x005
CAN_ID_TIME_SYNC_MASTER = 0x010
CAN_ID_TIME_SYNC_REQ = 0x011
CAN_ID_TIME_SYNC_RESP = 0x012
CAN_ID_THRUSTER_CMD = 0x100
CAN_ID_SOLENOID_CMD = 0x110
CAN_ID_NAV_TELEMETRY = 0x200
CAN_ID_ENV_TELEMETRY = 0x210
CAN_ID_POWER_TELEMETRY = 0x300
CAN_ID_USB_HUB_TELEMETRY = 0x310
CAN_ID_OUTPUT_STATUS = 0x7FE

# Valid scratch C file template implementing app_main()
VALID_SCRATCH_C = """#include <stdint.h>
#include <stdbool.h>

void app_main(void) {
    volatile uint32_t counter = 0;
    for (int i = 0; i < 10; ++i) {
        counter += (uint32_t)i;
    }
}
"""

SYNTAX_ERROR_C = """#include <stdint.h>
void app_main(void) {
    this is completely invalid C code syntax missing semicolon
}
"""

MISSING_APP_MAIN_C = """#include <stdint.h>
int helper_function(void) {
    return 42;
}
"""

PREPROCESSOR_ERROR_C = """#include <stdint.h>
#error "Deliberate preprocessor compilation failure"
void app_main(void) {}
"""


def pack_sil_can_frame(arb_id: int, data: bytes, magic: int = SIL_MAGIC_HEADER) -> bytes:
    """Pack an arbitration ID and payload into a 73-byte SIL CAN packet."""
    length = len(data)
    if length > 64:
        raise ValueError(f"CAN FD payload exceeds 64 bytes: {length}")
    padded_data = data.ljust(64, b"\x00")
    return struct.pack(SIL_CAN_PACKET_FORMAT, magic, arb_id, length, padded_data)


def unpack_sil_can_frame(buffer: bytes) -> Tuple[int, int, int, bytes]:
    """Unpack a 73-byte SIL CAN packet into (magic, arb_id, length, data)."""
    if len(buffer) < SIL_CAN_PACKET_SIZE:
        raise ValueError(f"Buffer too short for SIL frame: {len(buffer)} < {SIL_CAN_PACKET_SIZE}")
    magic, arb_id, length, raw_data = struct.unpack(SIL_CAN_PACKET_FORMAT, buffer[:SIL_CAN_PACKET_SIZE])
    return magic, arb_id, length, raw_data[:length]


def run_rov_cli(args: list[str], timeout: float = 60.0) -> subprocess.CompletedProcess:
    """Execute python rov.py with the given arguments."""
    cmd = [sys.executable, str(REPO_ROOT / "rov.py")] + args
    env = os.environ.copy()
    msys_bin = Path(r"C:\msys64\ucrt64\bin")
    if msys_bin.is_dir() and str(msys_bin) not in env.get("PATH", ""):
        env["PATH"] = f"{msys_bin};{env.get('PATH', '')}"

    return subprocess.run(
        cmd,
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        timeout=timeout,
        env=env,
    )


@contextlib.contextmanager
def temp_scratch_file(
    content: str = VALID_SCRATCH_C,
    prefix: str = "test_scratch_",
    suffix: str = ".c",
    directory: Optional[Path] = None,
) -> Generator[Path, None, None]:
    """Create a temporary C scratch file and automatically unlink on exit."""
    target_dir = directory or REPO_ROOT
    fd, path_str = tempfile.mkstemp(prefix=prefix, suffix=suffix, dir=target_dir)
    os.close(fd)
    path = Path(path_str)
    try:
        path.write_text(content, encoding="utf-8")
        yield path
    finally:
        if path.is_file():
            for _ in range(10):
                try:
                    path.unlink()
                    break
                except OSError:
                    time.sleep(0.1)


def robust_rmtree(path: Path | str, retries: int = 30, delay: float = 0.15) -> bool:
    """Robustly remove a directory tree, retrying on Windows file locks and permission errors."""
    target = Path(path)
    if not target.exists():
        return True

    def _remove_readonly(func, file_path, exc_info=None):
        try:
            os.chmod(file_path, stat.S_IWRITE)
            func(file_path)
        except Exception:
            pass

    for _ in range(retries):
        if not target.exists():
            return True
        try:
            if target.is_dir():
                if sys.version_info >= (3, 12):
                    shutil.rmtree(target, onexc=lambda func, p, exc: _remove_readonly(func, p, exc))
                else:
                    shutil.rmtree(target, onerror=_remove_readonly)
            else:
                target.unlink(missing_ok=True)
            if not target.exists():
                return True
        except OSError:
            pass
        if target.is_dir():
            try:
                os.rmdir(target)
                return True
            except OSError:
                pass
        time.sleep(delay)

    if target.exists():
        if os.name == "nt":
            try:
                subprocess.run(["cmd", "/c", "rmdir", "/s", "/q", str(target.resolve())], capture_output=True, timeout=5)
            except Exception:
                pass
        if target.exists():
            for _ in range(5):
                try:
                    shutil.rmtree(target, ignore_errors=True)
                    if not target.exists():
                        break
                except Exception:
                    pass
                time.sleep(delay)
    return not target.exists()


@contextlib.contextmanager
def temp_board(
    board_name: str = "test_board_tmp",
    mcu: str = "STM32F411RET6",
    family: str = "STM32F4",
) -> Generator[Path, None, None]:
    """Create a temporary board folder with an .ioc file inside boards/ and clean up on exit."""
    board_dir = REPO_ROOT / "boards" / board_name
    board_dir.mkdir(parents=True, exist_ok=True)
    ioc_file = board_dir / f"{board_name}.ioc"
    ioc_content = f"""# MicroXplorer Configuration
Mcu.CPN={mcu}
Mcu.Family={family}
Mcu.Name={mcu}
Mcu.Package=LQFP64
"""
    ioc_file.write_text(ioc_content, encoding="utf-8")
    try:
        yield board_dir
    finally:
        # Give Windows processes and Ninja a moment to release file handles before teardown
        time.sleep(0.15)
        if board_dir.exists():
            robust_rmtree(board_dir, retries=30, delay=0.15)
        build_root = REPO_ROOT / "build"
        if build_root.is_dir():
            for preset_dir in build_root.iterdir():
                if preset_dir.is_dir():
                    cand = preset_dir / "boards" / board_name
                    if cand.exists():
                        robust_rmtree(cand, retries=30, delay=0.15)
                    for lock_name in (".ninja_lock", ".ninja_log.restat"):
                        lock_file = preset_dir / lock_name
                        if lock_file.exists():
                            for _ in range(5):
                                try:
                                    lock_file.unlink()
                                    break
                                except OSError:
                                    time.sleep(0.1)
        time.sleep(0.1)


class MockSilTcpServer:
    """Lightweight in-process TCP server emulating the SIL CAN bridge for unit tests."""

    def __init__(self, host: str = "127.0.0.1", port: int = 0):
        self.host = host
        self.port = port
        self.server_socket: Optional[socket.socket] = None
        self.actual_port: int = port
        self.received_frames: list[Tuple[int, bytes]] = []
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self._client_socket: Optional[socket.socket] = None

    def start(self) -> int:
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.actual_port = self.server_socket.getsockname()[1]
        self.server_socket.listen(5)
        self.server_socket.settimeout(0.5)
        self.running = True
        self.thread = threading.Thread(target=self._serve_loop, daemon=True)
        self.thread.start()
        return self.actual_port

    def _serve_loop(self) -> None:
        while self.running:
            try:
                client_sock, _ = self.server_socket.accept()
                client_sock.settimeout(0.5)
                self._client_socket = client_sock
                self._handle_client(client_sock)
            except (socket.timeout, OSError):
                continue

    def _handle_client(self, sock: socket.socket) -> None:
        buf = bytearray()
        while self.running:
            try:
                data = sock.recv(1024)
                if not data:
                    break
                buf.extend(data)
                while len(buf) >= SIL_CAN_PACKET_SIZE:
                    packet_bytes = bytes(buf[:SIL_CAN_PACKET_SIZE])
                    del buf[:SIL_CAN_PACKET_SIZE]
                    try:
                        magic, arb_id, length, payload = unpack_sil_can_frame(packet_bytes)
                        self.received_frames.append((arb_id, payload))
                    except Exception:
                        pass
            except (socket.timeout, OSError):
                continue
        sock.close()

    def wait_for_client(self, timeout: float = 2.0) -> bool:
        """Wait until a client has connected and been accepted."""
        t_end = time.time() + timeout
        while time.time() < t_end:
            if self._client_socket is not None:
                return True
            time.sleep(0.01)
        return False

    def send_frame_to_client(self, arb_id: int, data: bytes) -> bool:
        if not self._client_socket:
            return False
        try:
            packet = pack_sil_can_frame(arb_id, data)
            self._client_socket.sendall(packet)
            return True
        except OSError:
            return False

    def stop(self) -> None:
        self.running = False
        if self._client_socket:
            try:
                self._client_socket.close()
            except OSError:
                pass
        if self.server_socket:
            try:
                self.server_socket.close()
            except OSError:
                pass
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
