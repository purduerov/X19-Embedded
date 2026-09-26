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
import signal
import socket
import subprocess
import sys
import tempfile
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
    "ServerBackedTestCase",
    "SilServerProcess",
    "SolenoidCommand",
    "ThrusterCommand",
    "describe_exit",
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


def describe_exit(returncode: Optional[int]) -> str:
    """
    Turn a child's ``Popen.returncode`` into the sentence a human needs.

    A negative return code is POSIX for "killed by signal -N", and naming the
    signal is the whole point: ``returncode == -13`` on the Linux runner is
    SIGPIPE, and without the name a dead simulator is indistinguishable from a
    refused connection.  ``Popen`` cannot tell you the name itself; this can,
    because ``signal.Signals`` is the authority on the numbering.
    """
    if returncode is None:
        return "still running (not yet reaped)"
    if returncode >= 0:
        return f"exited normally with code {returncode}"
    number = -returncode
    try:
        name = signal.Signals(number).name
    except ValueError:  # pragma: no cover - only for a signal this build lacks
        return f"killed by signal {number} (unnamed on this platform)"
    return f"killed by signal {number} ({name})"


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

    The child's stdout and stderr are **captured**, not discarded.  They used to
    go to ``DEVNULL``, which meant a failing server left no evidence at all: the
    "Client disconnected, waiting for reconnection..." line, the bind failure,
    and the crash message were all produced and all thrown away, so a dead
    simulator and a refused connection reported identically.  ``server_output()``
    and ``failure_report()`` exist to close that gap.

    The capture target is a temporary **file**, not a pipe, and that is
    deliberate.  ``DEVNULL`` was presumably chosen to avoid the classic
    pipe-fill deadlock, where a child that writes more than the pipe buffer
    blocks forever because nobody is draining it.  A pipe would reintroduce
    exactly that hazard unless a reader thread were added, and a reader thread
    is a second thing that can be wrong.  A regular file has no capacity limit,
    so the server can print as much as it likes and never blocks; the tests only
    ever read the file, and only on failure.
    """

    #: How much of the captured output ``failure_report`` includes.  The server
    #: prints a line per 200 cycles plus every accepted command, so a long test
    #: can accumulate thousands of lines; the tail is where a disconnect, a
    #: bind failure, or a crash always is.
    REPORT_TAIL_LINES = 40

    def __init__(self, executable: Path, port: Optional[int] = None) -> None:
        self.executable = Path(executable)
        self.port = int(port) if port is not None else free_tcp_port()
        self.process: Optional[subprocess.Popen] = None
        self._log_path: Optional[Path] = None
        self._log_handle = None
        #: Read from disk and kept when the file is removed, so diagnostics
        #: survive ``stop()`` and the fixture's own teardown cannot destroy the
        #: evidence before the failure message is built.
        self._retained_output = ""

    @property
    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def start(self) -> None:
        if self.is_running:
            raise RuntimeError(f"SIL server is already running on port {self.port}")

        handle, path = tempfile.mkstemp(
            prefix="sil_bridge_server-", suffix=".log", text=True
        )
        self._log_path = Path(path)
        self._log_handle = os.fdopen(handle, "w", encoding="utf-8", errors="replace")
        self._retained_output = ""

        try:
            self.process = subprocess.Popen(
                [str(self.executable), "--port", str(self.port), "--cycles", "0"],
                stdout=self._log_handle,
                stderr=subprocess.STDOUT,
            )
        except OSError:
            # The fixture must not leave a temp file behind when the binary
            # cannot even be exec'd; the caller's SkipTest/require path has
            # already vetted the executable, so this is a genuine surprise.
            self._close_log()
            raise

        deadline = time.monotonic() + SERVER_START_TIMEOUT_S
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError(
                    f"SIL server {describe_exit(self.process.returncode)} before listening "
                    f"on port {self.port}. Its own output was:\n{self.server_output()}"
                )
            try:
                probe = socket.create_connection(("127.0.0.1", self.port), timeout=0.1)
            except OSError:
                time.sleep(0.02)
            else:
                probe.close()
                return

        self.stop()
        raise RuntimeError(
            f"SIL server did not listen on port {self.port}. Its own output was:\n"
            f"{self.server_output()}"
        )

    def server_output(self) -> str:
        """
        Everything the server has written to stdout/stderr so far.

        Safe to call at any point in the fixture's life, including after
        ``stop()``: the file is read before it is removed, and the content is
        retained so teardown cannot erase the evidence.
        """
        if self._log_path is not None and self._log_path.exists():
            try:
                # Read a snapshot the child may still be appending to.  The
                # child's own stdio buffer is flushed by the fflush() calls in
                # sil_bridge_server.c, so anything it meant to say about the
                # connection is already here.
                return self._log_path.read_text(encoding="utf-8", errors="replace")
            except OSError:  # pragma: no cover - the file vanished under us
                pass
        return self._retained_output

    def failure_report(self) -> str:
        """
        A self-diagnosing block for an assertion message.

        The three facts it reports are deliberately distinct, because a single
        "connect() returned False" cannot tell them apart:

        * whether the **process** is still alive (``Popen.poll()`` re-reads the
          process table, so this is an OS fact, not a cached flag);
        * how it **exited**, by name -- a signal-terminated child is the whole
          diagnosis in one token;
        * whether the **port** still has an owner, probed by attempting to bind
          it.  Binding is used rather than connecting on purpose: the SIL server
          is single-client and *replaces* its current connection on every
          accept, so a connect-probe would silently steal the test's own
          socket and turn a diagnostic into a second, self-inflicted failure.

        The probe binds with the default options, i.e. without ``SO_REUSEADDR``.
        That is what makes it meaningful: a socket actively ``listen()``-ing on
        the port cannot be bound a second time on either platform.
        """
        process = self.process
        returncode = process.poll() if process is not None else None
        alive = process is not None and returncode is None
        output = self.server_output()

        lines = [
            "  sil_bridge_server diagnostics:",
            f"    executable            : {self.executable}",
            f"    port                  : {self.port}",
            f"    process alive         : {'yes' if alive else 'NO'}",
            f"    process exit          : {describe_exit(returncode)}",
            f"    port still bound      : {'yes' if self.port_is_bound() else 'no'}",
        ]
        if process is not None and returncode is not None:
            lines.append(
                "    >> the engine process is GONE, so this is not a connect race: nothing is "
                "listening, which is why the client's connect() was refused. Read the exit line "
                "above and the server's own output below."
            )
        elif process is not None:
            lines.append(
                "    >> the engine process is ALIVE and still owns the port, so a refused "
                "connect() came from the client or the socket layer, not from a dead simulator."
            )

        if not output.strip():
            lines.append("    server stdout/stderr  : (empty - the engine printed nothing)")
        else:
            captured = output.splitlines()
            kept = captured[-self.REPORT_TAIL_LINES :]
            omitted = len(captured) - len(kept)
            lines.append(
                f"    server stdout/stderr  : {len(captured)} line(s)"
                + (f", showing the last {len(kept)}" if omitted else "")
            )
            lines.append("    --- begin server output ---")
            lines.extend(f"    {line}" for line in kept)
            lines.append("    --- end server output ---")
        return "\n".join(lines)

    def port_is_bound(self) -> bool:
        """
        True when some socket still owns ``self.port``.

        Diagnostic only.  See ``failure_report`` for why this binds instead of
        connecting, and for why ``SO_REUSEADDR`` is left off: without it a second
        bind onto a listening port fails on Windows and on Linux alike, which is
        the answer this method is after.
        """
        probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            probe.bind(("127.0.0.1", self.port))
        except OSError:
            return True
        else:
            return False
        finally:
            probe.close()

    def connect_socket(self, timeout: float = 3.0) -> socket.socket:
        """Open a client connection to the running server (caller closes it)."""
        if not self.is_running:
            raise RuntimeError("SIL server is not running; call start() first")
        return socket.create_connection(("127.0.0.1", self.port), timeout=timeout)

    def _close_log(self) -> None:
        """Read the log into memory, then remove the temp file."""
        if self._log_path is not None and self._log_path.exists():
            try:
                self._retained_output = self._log_path.read_text(
                    encoding="utf-8", errors="replace"
                )
            except OSError:  # pragma: no cover - the file vanished under us
                pass
        if self._log_handle is not None:
            try:
                self._log_handle.close()
            except OSError:  # pragma: no cover
                pass
            self._log_handle = None
        if self._log_path is not None:
            try:
                self._log_path.unlink()
            except OSError:  # pragma: no cover - already gone
                pass
            self._log_path = None

    def stop(self) -> None:
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=SERVER_SHUTDOWN_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=SERVER_SHUTDOWN_TIMEOUT_S)
        self.process = None
        self._close_log()

    def __enter__(self) -> "SilServerProcess":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()


class _FailureAnnotatingMethod:
    """
    Callable wrapper that appends the engine's diagnostics to a failure.

    Deliberately a callable *object* rather than a ``def`` wrapper or a
    ``functools.partial``.  ``unittest.TestLoader.loadTestsFromName`` resolves a
    dotted test id (``module.Class.test_name``) by fetching the attribute off the
    class, instantiating, and then checking
    ``isinstance(getattr(instance, name), types.FunctionType)``
    (unittest/loader.py:187).  An instance attribute that *is* a function makes
    that check fall through, after which the loader calls the unbound class
    function with no arguments and dies with
    ``TypeError: missing 1 required positional argument: 'self'``.  Any object
    with ``__call__`` keeps ``python -m unittest module.Class.test_x`` working,
    which is exactly how both originally-failing tests are re-run in isolation.
    """

    def __init__(self, method, diagnostics) -> None:
        self._method = method
        self._diagnostics = diagnostics

    def __call__(self):
        try:
            return self._method()
        except AssertionError as exc:
            context = self._diagnostics()
            # Skip when the assertion already quoted the report.  A test is
            # allowed to inline ``failure_report()`` in its own message so the
            # text survives even if this base class is ever dropped, and
            # appending it a second time would print the same seven lines twice
            # and bury whatever the assertion actually had to say.
            if context and context.splitlines()[0] not in str(exc):
                raise AssertionError(f"{exc}\n{context}") from None
            raise


class ServerBackedTestCase(unittest.TestCase):
    """
    Base for test classes that talk to a real ``sil_bridge_server``.

    Every assertion failure raised by the test method is re-raised with the
    engine's own diagnostics appended.  This exists because of a real CI
    failure: ``test_disconnect_stops_transmission_and_the_watchdog_drops_to_
    neutral`` reported ``AssertionError: False is not true`` from
    ``SilDashboardClient.connect()`` on the Linux runner and nothing else, and
    the fixture was discarding the child's stdout, so "the engine died" and "the
    reconnect was refused" were the same message.  Appending the evidence to
    *every* failure in the class, rather than to a hand-picked few, is
    deliberate: the point is that the next failure answers its own question
    without anyone having to remember which assertion to annotate.

    ``subTest`` failures are not annotated -- unittest records those inside the
    subtest's own outcome rather than propagating them to the test method.

    The wrapper is installed in ``__init__`` because ``TestCase.run`` resolves
    ``self._testMethodName`` *before* it calls ``setUp()``: an instance
    attribute set from ``setUp`` would never be the callable that gets run.
    """

    #: Replaced by the subclass's ``setUp``. ``None`` is fine and means "no
    #: fixture-owned engine", in which case the client's own captured child
    #: output is used instead (see ``_server_diagnostics``).
    server: Optional[SilServerProcess] = None

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        method = getattr(self, self._testMethodName)
        if isinstance(method, _FailureAnnotatingMethod):
            return
        setattr(
            self,
            self._testMethodName,
            _FailureAnnotatingMethod(method, self._server_diagnostics),
        )

    def _server_diagnostics(self) -> str:
        """
        The engine's evidence, or ``""`` when there is no engine to report on.

        Two sources, because two classes own their engine differently:
        ``SilServerProcess`` captures the child's output to a file, while
        ``SilDashboardClient.start_server_process`` tees it into
        ``c_stdout_log``. Both are consulted, so the client-owned-engine class
        gets the same treatment as the fixture-owned one.
        """
        if self.server is not None:
            return self.server.failure_report()
        client = getattr(self, "client", None)
        log = getattr(client, "c_stdout_log", None)
        if not log:
            return ""
        return (
            "  sil_bridge_server diagnostics (captured by the client):\n"
            + "\n".join(f"    {line}" for line in list(log)[-40:])
        )
