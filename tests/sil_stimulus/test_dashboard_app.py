"""
Behavioural tests for the Streamlit SIL dashboard's control wiring.

The dashboard is the operator's control surface for simulated thrusters, so the
wiring between a button press and the deadman state machine is safety-critical
code, not glue.  ``SilDashboardClient`` (Task 2) resends a held command at 20 Hz
and treats an all-neutral command as a *release*: it goes to ``ARMED`` and
stops the worker, because a continuously refreshed neutral heartbeat would stop
the firmware watchdog from ever zeroing the solenoid mask
(``node2_force_neutral()`` calls ``bsp_solenoid_set(0)`` and runs on
``heartbeat_lost`` in ``nodes/node2_control_board/Core/Src/app.c``).

That is why the two "All Stop" buttons must call ``request_stop()`` rather than
sending a neutral command, and why the emergency button must call
``request_emergency_break()``.  These tests drive the button handlers that
``dashboard_app.main()`` calls and assert on the state machine and on the frames
that actually reach the wire, so an accidental reversion to the old one-shot UI
fails here.

Three layers, cheapest first:

* ``TestDashboardAppModule``      -- the handlers are importable module-level
  functions, so the safety wiring stays testable at all.
* ``TestDashboardHandlerWiring``  -- each handler calls exactly the one client
  method it is allowed to call, checked against a recording stub.
* ``TestDashboardHandlersAgainstRealClient`` -- the real
  ``SilDashboardClient`` over a ``socket.socketpair``, so "the worker stopped"
  means no further frames were written, not that a return value said so.
* ``TestDashboardHandlersAgainstServer`` -- the same handlers against the real
  native ``sil_bridge_server``, skipped when it has not been built.

Run from the repository root::

    python -m unittest tests.sil_stimulus.test_dashboard_app -v
"""

from __future__ import annotations

import inspect
import socket
import sys
import threading
import time
import unittest
from typing import List, Tuple

try:
    from .sil_test_support import (
        CAN_ID_EMERGENCY_BREAK,
        CAN_ID_THRUSTER_CMD,
        REPO_ROOT,
        SIL_PACKET_SIZE,
        SilServerProcess,
        ThrusterCommand,
        require_server_executable,
        unpack_sil_can_frame,
    )
except ImportError:  # pragma: no cover - direct execution fallback
    from sil_test_support import (  # type: ignore[no-redef]
        CAN_ID_EMERGENCY_BREAK,
        CAN_ID_THRUSTER_CMD,
        REPO_ROOT,
        SIL_PACKET_SIZE,
        SilServerProcess,
        ThrusterCommand,
        require_server_executable,
        unpack_sil_can_frame,
    )

# tests/sil_stimulus/test_dashboard_app.py -> tests/sil_stimulus -> tests
DASHBOARD_DIR = REPO_ROOT / "tests" / "sil_dashboard"
if str(DASHBOARD_DIR) not in sys.path:
    sys.path.insert(0, str(DASHBOARD_DIR))

from sil_dashboard_client import (  # noqa: E402  (path setup must precede it)
    CONTROL_PERIOD_S,
    HEARTBEAT_TIMEOUT_S,
    NEUTRAL_PWM_US,
    ControlState,
    SilDashboardClient,
    compute_thrust_allocation,
)

# Importing the dashboard module pulls in streamlit.  A missing streamlit must
# skip the whole module rather than error at import time, or a skipped suite
# would be indistinguishable from a passing one.
dashboard_app = None
_DASHBOARD_IMPORT_ERROR: "BaseException | None" = None
try:
    import dashboard_app  # type: ignore[no-redef]  # noqa: F811
except Exception as exc:  # noqa: BLE001 - classified immediately below
    _DASHBOARD_IMPORT_ERROR = exc


def setUpModule() -> None:
    if _DASHBOARD_IMPORT_ERROR is None:
        return
    missing = getattr(_DASHBOARD_IMPORT_ERROR, "name", "") or ""
    if missing.split(".")[0] == "streamlit" or "streamlit" in str(_DASHBOARD_IMPORT_ERROR):
        raise unittest.SkipTest(
            "streamlit is not installed, so tests/sil_dashboard/dashboard_app.py "
            "cannot be imported. This suite is SKIPPED, not passing. Install it "
            "with 'pip install streamlit' and re-run."
        )
    raise _DASHBOARD_IMPORT_ERROR


NEUTRAL_PWMS = [NEUTRAL_PWM_US] * 8
# Node 2 refuses thruster commands until ESC_ARMING_TIME_MS of virtual time has
# elapsed (nodes/node2_control_board/Core/Src/app.c).
ESC_ARMING_SIM_MS = 3000
SIM_MS_PER_WALL_S = 0.8  # generous lower bound; the C engine sleeps 10 ms per tick
# Long enough to span several control periods *and* the firmware heartbeat
# window, so "the command path went quiet" cannot be a timing accident.
QUIET_WINDOW_S = HEARTBEAT_TIMEOUT_S * 3
# How long to let the 20 Hz worker prove it is running.  Generous on purpose:
# the assertion that matters is the *absence* of frames afterwards, so waiting
# longer here only costs time, while waiting too briefly would fail the
# precondition for the wrong reason.
WORKER_SPINUP_S = CONTROL_PERIOD_S * 8
WORKER_SPINUP_FRAMES = 4
# Five double-acting valves, one coil energized each.  Not 0x3FF: rov_can_unpack
# _solenoid_cmd rejects a mask that energizes both coils of any pair and zeroes
# the *whole* mask when it does, so 0x3FF never reaches the outputs.
ENERGIZED_SOLENOID_MASK = 0x0155


class _RecordingClient:
    """
    Minimal stand-in that records every control call the UI makes.

    It deliberately implements only the client surface a button handler is
    allowed to touch.  Any other attribute access raises ``AttributeError``, so
    a handler that reaches for something unexpected fails loudly instead of
    quietly working against a permissive mock.
    """

    def __init__(self, control_state: ControlState = ControlState.ARMED,
                 connected: bool = True) -> None:
        self.connected = connected
        self.control_state = control_state
        self.pwms: List[int] = list(NEUTRAL_PWMS)
        self.solenoid_mask = 0
        self.pipeline_surface_cmd = {
            "surge": 0.0, "sway": 0.0, "heave": 0.0,
            "yaw": 0.0, "pitch": 0.0, "roll": 0.0,
            "timestamp_us": 0, "raw_hex": "",
        }
        self.calls: List[Tuple[str, tuple]] = []
        self.started_server = False
        self.connected_after_start = False

    # -- control path ---------------------------------------------------
    def _record(self, name: str, *args) -> bool:
        self.calls.append((name, args))
        return True

    def request_stop(self) -> bool:
        # Deliberately leaves ``pwms`` alone, exactly as the real client does:
        # only the worker's held command goes neutral.  The UI depends on this.
        # An All Stop that published a new commanded target would be undone by
        # the per-slider diff on the very next Streamlit rerun.
        return self._record("request_stop")

    def request_emergency_break(self) -> bool:
        self._record("request_emergency_break")
        self.control_state = ControlState.ESTOP
        return True

    def trigger_emergency_break(self) -> bool:
        self._record("trigger_emergency_break")
        self.control_state = ControlState.ESTOP
        return True

    def send_pwms(self, pwms) -> bool:
        self._record("send_pwms", list(pwms))
        # The real client publishes the accepted target for the UI to diff
        # against, so a faithful stub has to as well.
        self.pwms = list(pwms)
        return True

    def send_surface_pilot_command(self, surge, sway, heave, yaw, pitch=0.0, roll=0.0) -> bool:
        self._record("send_surface_pilot_command", surge, sway, heave, yaw)
        self.pipeline_surface_cmd.update(
            {"surge": surge, "sway": sway, "heave": heave, "yaw": yaw}
        )
        self.pwms = compute_thrust_allocation(surge, sway, heave, yaw, pitch, roll)
        return True

    def send_solenoids(self, mask) -> bool:
        self._record("send_solenoids", mask)
        self.solenoid_mask = mask
        return True

    # -- transport ------------------------------------------------------
    def start_server_process(self) -> bool:
        self.calls.append(("start_server_process", ()))
        self.started_server = True
        return True

    def stop_server_process(self) -> None:
        self.calls.append(("stop_server_process", ()))
        self.connected = False

    def connect(self) -> bool:
        self.calls.append(("connect", ()))
        self.connected = True
        self.connected_after_start = True
        return True

    def disconnect(self) -> None:
        self.calls.append(("disconnect", ()))
        self.connected = False

    # -- assertions -----------------------------------------------------
    def send_calls(self) -> List[Tuple[str, tuple]]:
        """Every call that puts a command on the wire, in order."""
        return [
            (name, args) for name, args in self.calls
            if name in ("send_pwms", "send_surface_pilot_command", "send_solenoids")
        ]


class _WireRecorder:
    """Parses every SIL frame the client writes, in wire order, on a reader thread."""

    def __init__(self, sock: socket.socket) -> None:
        self._sock = sock
        self._buffer = bytearray()
        self._lock = threading.Lock()
        self._frames: List[Tuple[int, bytes]] = []
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="fake-sil-board", daemon=True
        )
        self._thread.start()

    def _run(self) -> None:
        self._sock.settimeout(0.05)
        while not self._stop.is_set():
            try:
                chunk = self._sock.recv(SIL_PACKET_SIZE * 8)
            except (socket.timeout, TimeoutError):
                continue
            except OSError:
                return
            if not chunk:
                return
            with self._lock:
                self._buffer.extend(chunk)
                while len(self._buffer) >= SIL_PACKET_SIZE:
                    packet = bytes(self._buffer[:SIL_PACKET_SIZE])
                    del self._buffer[:SIL_PACKET_SIZE]
                    self._frames.append(unpack_sil_can_frame(packet))

    def frames(self) -> List[Tuple[int, bytes]]:
        with self._lock:
            return list(self._frames)

    def count(self) -> int:
        with self._lock:
            return len(self._frames)

    def settle(self) -> int:
        """
        Return the frame count once anything already on the wire has been parsed.

        The reader thread is asynchronous, so a count taken immediately after a
        synchronous ``sendall`` can lag by a frame.  Any baseline for a
        "nothing more was sent" assertion has to be taken after this, or the
        assertion measures parser latency rather than transmission.
        """
        time.sleep(2 * CONTROL_PERIOD_S)
        return self.count()

    def thruster_pwms(self) -> List[List[int]]:
        return [
            ThrusterCommand.unpack(payload).pwm_us
            for can_id, payload in self.frames() if can_id == CAN_ID_THRUSTER_CMD
        ]

    def wait_for_count(self, minimum: int, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.count() >= minimum:
                return True
            time.sleep(0.005)
        return self.count() >= minimum

    def quiet_for(self, duration_s: float) -> bool:
        before = self.count()
        time.sleep(duration_s)
        return self.count() == before

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            self._sock.close()
        except OSError:
            pass


class _FakeSilTransport:
    """
    A transport-free ``SilDashboardClient`` that still writes real frames.

    ``socket.socketpair`` returns two connected loopback sockets, so the client's
    ``_send_frame`` performs a genuine ``sendall`` and this recorder parses what
    actually went out.  An assertion about "the command path went quiet" is
    therefore an assertion about the wire, not about a boolean return value.

    The client's reader thread is intentionally not started: this recorder plays
    the part of the native board, so ``actual_pwms`` never moves and every
    assertion stays a statement about the *host*.  Firmware behaviour is covered
    by ``TestDashboardHandlersAgainstServer``.

    The socket is left blocking.  Production calls ``setblocking(False)``, and a
    non-blocking 73-byte write would raise ``BlockingIOError`` -- an ``OSError``
    subclass the client treats as a transport fault.  The recorder drains
    continuously, so the send buffer never fills and the blocking mode only
    removes that one confounder.
    """

    def __init__(self, client: SilDashboardClient) -> None:
        client_side, board_side = socket.socketpair()
        self.client_sock = client_side
        self.recorder = _WireRecorder(board_side)
        self.attach(client)

    def attach(self, client: SilDashboardClient) -> None:
        """Put the client into exactly the state ``connect()`` leaves behind."""
        client.sock = self.client_sock
        client.connected = True
        client.running = True
        client.control_state = ControlState.ARMED
        client._control_stop = threading.Event()
        client._control_thread = None
        client._last_command = list(NEUTRAL_PWMS)

    def close(self, client: SilDashboardClient) -> None:
        client.connected = False
        client.running = False
        client._control_stop.set()
        thread = client._control_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        if client.sock is not None:
            try:
                client.sock.close()
            except OSError:
                pass
            client.sock = None
        self.recorder.close()


class TestDashboardAppModule(unittest.TestCase):
    """The control surface has to be reachable without a Streamlit runtime."""

    def test_dashboard_app_imports_and_exposes_main(self):
        self.assertTrue(callable(dashboard_app.main))

    def test_control_handlers_are_module_level_functions(self):
        """
        Invariant: every safety-relevant button handler is a module-level
        function that ``main()`` calls.

        Inline handlers cannot be exercised without booting Streamlit, so
        inlining them back would silently delete the coverage of the stop,
        emergency, and reconnect wiring.  This is the test that makes those
        tests possible to keep.
        """
        handlers = (
            "all_stop",
            "trip_emergency_break",
            "auto_refresh_due",
            "pilot_axes_changed",
            "thruster_targets_changed",
            "control_state_presentation",
            "ensure_transport",
        )
        for name in handlers:
            with self.subTest(handler=name):
                handler = getattr(dashboard_app, name, None)
                self.assertIsNotNone(
                    handler,
                    f"dashboard_app.{name} must be a module-level function so the "
                    f"safety wiring stays testable",
                )
                self.assertTrue(
                    inspect.isfunction(handler),
                    f"dashboard_app.{name} must be a plain function, not a "
                    f"Streamlit-decorated callable",
                )

    def test_main_does_not_reach_into_the_client_for_control_actions(self):
        """
        Invariant: the only control actions in ``main()`` go through the module
        handlers, so there is exactly one place that can call ``request_stop``
        or ``request_emergency_break``.
        """
        source = inspect.getsource(dashboard_app.main)
        for forbidden in ("request_stop", "request_emergency_break", "trigger_emergency_break"):
            with self.subTest(call=forbidden):
                self.assertNotIn(
                    forbidden, source,
                    f"main() must delegate {forbidden} to a module-level handler",
                )
        for handler in ("all_stop", "trip_emergency_break", "auto_refresh_due",
                        "pilot_axes_changed", "thruster_targets_changed",
                        "control_state_presentation", "ensure_transport"):
            with self.subTest(handler=handler):
                self.assertIn(f"{handler}(", source, f"main() must use {handler}()")


class TestDashboardHandlerWiring(unittest.TestCase):
    """
    What each handler is allowed to do, checked against a recording stub.

    These run with no native binary and no transport: they pin the *call*
    contract, which is the thing that regressed.
    """

    def test_all_stop_calls_request_stop_and_sends_no_command(self):
        """
        Invariant: "All Stop" is the explicit STOP transition, not a command.
        """
        client = _RecordingClient(control_state=ControlState.RUNNING)
        dashboard_app.all_stop(client)
        self.assertEqual(client.calls, [("request_stop", ())])
        self.assertEqual(client.send_calls(), [])

    def test_all_stop_never_touches_the_transport(self):
        """
        Invariant: All Stop is not a reconnect, a restart, or a disconnect.
        """
        client = _RecordingClient(control_state=ControlState.RUNNING)
        dashboard_app.all_stop(client)
        for forbidden in ("connect", "start_server_process", "stop_server_process",
                          "disconnect", "start_control_loop", "stop_control_loop"):
            with self.subTest(call=forbidden):
                self.assertNotIn(forbidden, [name for name, _ in client.calls])

    def test_emergency_button_calls_the_canonical_request_emergency_break(self):
        """
        Invariant: the emergency button uses ``request_emergency_break()``, the
        canonical Task 2 API, and not the legacy alias.
        """
        client = _RecordingClient(control_state=ControlState.RUNNING)
        dashboard_app.trip_emergency_break(client)
        self.assertEqual(client.calls, [("request_emergency_break", ())])
        self.assertIs(client.control_state, ControlState.ESTOP)

    def test_pilot_all_stop_is_not_silently_reversed_by_the_next_rerun(self):
        """
        Invariant: pressing "All Stop (Hover)" is not undone by the rerun that
        follows it.

        Streamlit keeps widget state across a rerun, so the axis sliders still
        read the operator's last value.  An All Stop that republished the axes
        as all-zero would make the very next rerun see a difference and re-send
        the pre-stop command, silently re-arming thrust.  ``request_stop()``
        leaves ``pipeline_surface_cmd`` alone, so the diff is empty.
        """
        client = _RecordingClient(control_state=ControlState.RUNNING)
        # The operator drags Surge to 0.5; the pilot tab sends it.
        client.send_surface_pilot_command(0.5, 0.0, 0.0, 0.0)
        calls_before_stop = len(client.calls)

        dashboard_app.all_stop(client)
        # Next rerun: the sliders still hold 0.5.
        if dashboard_app.pilot_axes_changed(client, 0.5, 0.0, 0.0, 0.0):
            client.send_surface_pilot_command(0.5, 0.0, 0.0, 0.0)
        self.assertEqual(
            client.calls[calls_before_stop:],
            [("request_stop", ())],
            "The rerun after All Stop re-sent the pilot axis; thrust would be "
            "re-armed without any operator action",
        )

    def test_thruster_all_stop_is_not_silently_reversed_by_the_next_rerun(self):
        """
        Invariant: pressing "All Stop (1500 us)" is not undone by the rerun
        that follows it.

        The per-thruster sliders keep the operator's last value across a rerun.
        An All Stop that rewrote the client's commanded target to neutral would
        make the next rerun see a difference and re-send the pre-stop PWMs.
        """
        client = _RecordingClient(control_state=ControlState.RUNNING)
        client.send_pwms([1650] * 8)
        calls_before_stop = len(client.calls)

        dashboard_app.all_stop(client)
        if dashboard_app.thruster_targets_changed(client, [1650] * 8):
            client.send_pwms([1650] * 8)
        self.assertEqual(
            client.calls[calls_before_stop:],
            [("request_stop", ())],
            "The rerun after All Stop re-sent the thruster targets; thrust would "
            "be re-armed without any operator action",
        )

    def test_pilot_axes_changed_is_false_when_the_axes_match(self):
        client = _RecordingClient()
        client.send_surface_pilot_command(0.25, -0.5, 0.5, 0.25)
        self.assertFalse(dashboard_app.pilot_axes_changed(client, 0.25, -0.5, 0.5, 0.25))
        self.assertTrue(dashboard_app.pilot_axes_changed(client, 0.25, -0.5, 0.5, 0.0))
        self.assertTrue(dashboard_app.pilot_axes_changed(client, 0.26, -0.5, 0.5, 0.25))

    def test_thruster_targets_changed_is_false_when_the_targets_match(self):
        client = _RecordingClient()
        client.send_pwms([1650] * 8)
        self.assertFalse(dashboard_app.thruster_targets_changed(client, [1650] * 8))
        self.assertTrue(dashboard_app.thruster_targets_changed(client, [1600] * 8))

    def test_auto_refresh_loop_sends_no_second_command(self):
        """
        Invariant: the render loop never transmits.

        The 20 Hz control worker owns the command cadence.  If the refresh path
        also sent, every Streamlit rerun would be an extra command frame at
        whatever rate the operator's browser happened to poll.
        """
        client = _RecordingClient(control_state=ControlState.RUNNING)
        for _ in range(5):
            dashboard_app.auto_refresh_due(client, True, 0.5)
        self.assertEqual(client.send_calls(), [])
        self.assertEqual(client.calls, [])

    def test_auto_refresh_due_only_fires_on_a_connected_transport(self):
        connected = _RecordingClient(control_state=ControlState.RUNNING, connected=True)
        self.assertTrue(dashboard_app.auto_refresh_due(connected, True, 0.5))
        offline = _RecordingClient(control_state=ControlState.DISCONNECTED, connected=False)
        self.assertFalse(dashboard_app.auto_refresh_due(offline, True, 0.5))
        # An operator who turned refresh off must not get a rerun loop.
        self.assertFalse(dashboard_app.auto_refresh_due(connected, False, 0.5))

    def test_control_state_presentation_separates_safe_from_live_states(self):
        """
        Invariant: the operator can tell, at a glance, whether the control path
        is transmitting.

        ``ARMED`` and ``STOPPED`` are both safe but mean different things, and
        ``RUNNING`` and ``ESTOP`` are the two states that must never be
        mistaken for either.  All five need distinct labels, and the two
        safe-idle states must not be presented the same as the live ones.
        """
        presentations = {
            state: dashboard_app.control_state_presentation(
                _RecordingClient(control_state=state)
            )
            for state in ControlState
        }
        for state, (label, color, help_text) in presentations.items():
            with self.subTest(state=state):
                self.assertTrue(label.strip(), "every state needs a label")
                self.assertTrue(color.strip(), "every state needs a color")
                self.assertTrue(help_text.strip(), "every state needs an explanation")

        labels = [label for label, _, _ in presentations.values()]
        self.assertEqual(
            len(set(labels)), len(labels),
            f"all five control states need distinguishable labels, got {labels}",
        )
        self.assertNotEqual(
            presentations[ControlState.ESTOP][1],
            presentations[ControlState.STOPPED][1],
            "ESTOP must not be color-coded like a safe idle state",
        )
        self.assertNotEqual(
            presentations[ControlState.RUNNING][1],
            presentations[ControlState.ARMED][1],
            "RUNNING must not be color-coded like a safe idle state",
        )

    def test_latched_estop_blocks_automatic_reconnect(self):
        """
        Invariant: a latched E-stop cannot be cleared by a transport hiccup.

        ``SilDashboardClient.connect()`` clears the E-stop latch.  If the UI
        reconnected automatically whenever the transport looked down, a reader
        error would silently disarm a tripped emergency break with no operator
        action.  The reconnect therefore requires an explicit button press.
        """
        client = _RecordingClient(control_state=ControlState.ESTOP, connected=False)
        self.assertFalse(dashboard_app.ensure_transport(client))
        self.assertEqual(
            client.calls, [],
            "A latched E-stop must not be cleared by an automatic reconnect",
        )
        self.assertFalse(client.started_server)
        self.assertFalse(client.connected_after_start)

    def test_automatic_reconnect_still_happens_without_a_latched_estop(self):
        """
        Invariant: the interlock above must not break ordinary operation.
        """
        for state in (ControlState.DISCONNECTED, ControlState.STOPPED, ControlState.ARMED):
            with self.subTest(state=state):
                client = _RecordingClient(control_state=state, connected=False)
                self.assertTrue(dashboard_app.ensure_transport(client))
                self.assertEqual(
                    [name for name, _ in client.calls],
                    ["start_server_process", "connect"],
                )

    def test_ensure_transport_leaves_a_live_connection_alone(self):
        client = _RecordingClient(control_state=ControlState.RUNNING, connected=True)
        self.assertTrue(dashboard_app.ensure_transport(client))
        self.assertEqual(client.calls, [])


class TestDashboardHandlersAgainstRealClient(unittest.TestCase):
    """
    The real ``SilDashboardClient`` over a real socket, asserting on the wire.

    No native binary needed: these assert what the *host* transmits, which is
    exactly the question the UI wiring raises.
    """

    def setUp(self) -> None:
        self.client = SilDashboardClient(port=1)
        self.addCleanup(self.client.disconnect)
        self.transport = _FakeSilTransport(self.client)
        self.recorder = self.transport.recorder
        self.addCleanup(self.transport.close, self.client)

    def assertWorkerStopped(self) -> None:
        thread = self.client._control_thread
        self.assertTrue(
            thread is None or not thread.is_alive(),
            "The 20 Hz control worker is still running after an All Stop",
        )

    def test_all_stop_stops_the_worker_and_silences_the_command_path(self):
        """
        Invariant: "All Stop" ends the 20 Hz cadence, not just the current
        value.  The firmware watchdog can only lapse, and the solenoids can only
        be released, if the command path goes quiet.
        """
        self.assertTrue(self.client.send_pwms([1650] * 8))
        self.assertIs(self.client.control_state, ControlState.RUNNING)
        self.assertTrue(
            self.recorder.wait_for_count(WORKER_SPINUP_FRAMES, WORKER_SPINUP_S),
            "The 20 Hz worker must be transmitting before the stop",
        )

        dashboard_app.all_stop(self.client)

        self.assertIs(self.client.control_state, ControlState.STOPPED)
        self.assertWorkerStopped()
        self.assertEqual(self.client._last_command, NEUTRAL_PWMS)
        baseline = self.recorder.settle()
        self.assertTrue(
            self.recorder.quiet_for(QUIET_WINDOW_S),
            "Frames were still transmitted after All Stop; the firmware "
            "watchdog would never lapse and the solenoids would stay energized",
        )
        self.assertEqual(self.recorder.count(), baseline)
        sent = self.recorder.thruster_pwms()
        self.assertTrue(sent, "no thruster frame reached the wire at all")
        self.assertEqual(
            sent[-1], NEUTRAL_PWMS,
            "The last frame after All Stop must be neutral",
        )
        for values in sent:
            self.assertTrue(
                all(1000 <= value <= 2000 for value in values),
                f"a frame left the PWM envelope: {values}",
            )

    def test_all_stop_from_the_pilot_hover_button_releases_the_deadman(self):
        """
        Invariant: the pilot tab's "All Stop (Hover)" is a real stop too, not a
        one-shot neutral command.
        """
        self.assertTrue(self.client.send_surface_pilot_command(0.5, 0.0, 0.0, 0.0))
        self.assertIs(self.client.control_state, ControlState.RUNNING)
        self.assertTrue(self.recorder.wait_for_count(WORKER_SPINUP_FRAMES, WORKER_SPINUP_S))

        dashboard_app.all_stop(self.client)

        self.assertIs(self.client.control_state, ControlState.STOPPED)
        self.assertWorkerStopped()
        baseline = self.recorder.settle()
        self.assertTrue(self.recorder.quiet_for(QUIET_WINDOW_S))
        self.assertEqual(self.recorder.count(), baseline)
        self.assertEqual(self.recorder.thruster_pwms()[-1], NEUTRAL_PWMS)

    def test_neither_all_stop_button_is_reversed_by_the_next_rerun(self):
        """
        Invariant: the rerun after an All Stop transmits nothing, on both tabs.

        This is the one-shot UI's worst failure mode: the All Stop republished
        the client's commanded target, so the per-slider diff on the following
        rerun saw a change and re-sent the pre-stop command.
        """
        for tab, pending in (
            ("pilot", (0.5, 0.0, 0.0, 0.0)),
            ("thruster", ([1650] * 8,)),
        ):
            with self.subTest(tab=tab):
                self._reset_to_armed()
                if tab == "pilot":
                    self.assertTrue(self.client.send_surface_pilot_command(*pending))
                else:
                    self.assertTrue(self.client.send_pwms(*pending))
                self.assertIs(self.client.control_state, ControlState.RUNNING)
                self.assertTrue(
                    self.recorder.wait_for_count(WORKER_SPINUP_FRAMES, WORKER_SPINUP_S)
                )

                dashboard_app.all_stop(self.client)
                after_stop = self.recorder.settle()

                # Streamlit rerun: the widgets still hold the operator's values.
                if tab == "pilot":
                    changed = dashboard_app.pilot_axes_changed(self.client, *pending)
                    if changed:
                        self.client.send_surface_pilot_command(*pending)
                else:
                    changed = dashboard_app.thruster_targets_changed(
                        self.client, list(pending[0])
                    )
                    if changed:
                        self.client.send_pwms(*pending)

                self.assertFalse(
                    changed,
                    f"The {tab} tab would re-send the pre-stop command on the "
                    f"next rerun, re-arming thrust with no operator action",
                )
                time.sleep(2 * CONTROL_PERIOD_S)
                self.assertEqual(
                    self.recorder.count(), after_stop,
                    f"A frame was transmitted after the {tab} tab's All Stop",
                )

    def _reset_to_armed(self) -> None:
        """
        Return the shared fixture to a clean armed transport between subtests.

        Done by hand rather than via ``disconnect()``/``connect()``: a real
        reconnect would dial a port that is not listening, and ``connect()``
        clearing the E-stop latch is exactly the behaviour under test
        elsewhere, so it must not be used as test setup here.
        """
        self.client._control_stop.set()
        thread = self.client._control_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self.client._control_thread = None
        self.client._estop_latched = False
        self.transport.attach(self.client)
        self.client.pwms = list(NEUTRAL_PWMS)
        self.client.pipeline_surface_cmd.update(
            {"surge": 0.0, "sway": 0.0, "heave": 0.0, "yaw": 0.0}
        )
        self.assertIs(self.client.control_state, ControlState.ARMED)
        time.sleep(2 * CONTROL_PERIOD_S)

    def test_emergency_break_sends_the_authorized_frame_and_latches(self):
        """
        Invariant: the emergency button puts the authorized 0xAA 0x55 frame on
        the wire, latches ESTOP, and refuses every later command.
        """
        self.assertTrue(self.client.send_pwms([1650] * 8))
        self.assertIs(self.client.control_state, ControlState.RUNNING)
        self.assertTrue(self.recorder.wait_for_count(WORKER_SPINUP_FRAMES, WORKER_SPINUP_S))

        self.assertTrue(dashboard_app.trip_emergency_break(self.client))

        self.assertIs(self.client.control_state, ControlState.ESTOP)
        self.assertWorkerStopped()
        emergency_frames = [
            payload for can_id, payload in self.recorder.frames()
            if can_id == CAN_ID_EMERGENCY_BREAK
        ]
        self.assertTrue(emergency_frames, "no emergency frame reached the wire")
        self.assertTrue(
            all(payload[:2] == b"\xAA\x55" for payload in emergency_frames),
            f"Node 2 only accepts 0xAA 0x55, got "
            f"{[payload[:3].hex(' ') for payload in emergency_frames]}",
        )
        self.assertEqual(emergency_frames[0][:3], b"\xAA\x55\x01")
        after_trip = self.recorder.settle()
        self.assertEqual(self.recorder.thruster_pwms()[-1], NEUTRAL_PWMS)
        self.assertTrue(
            self.recorder.quiet_for(QUIET_WINDOW_S),
            "The command path kept transmitting after the emergency break",
        )
        self.assertEqual(self.recorder.count(), after_trip)
        # Latched: a latched ESTOP refuses every later command outright.
        self.assertFalse(self.client.send_pwms([1650] * 8))
        self.assertFalse(self.client.start_control_loop())
        self.assertEqual(self.recorder.count(), after_trip)
        self.assertIs(self.client.control_state, ControlState.ESTOP)

    def test_emergency_alias_and_canonical_entry_point_are_the_same_action(self):
        """
        Invariant: ``trigger_emergency_break`` is a true alias of
        ``request_emergency_break``, so the UI's choice of name is behaviour
        neutral.  This is the evidence behind calling the canonical one.
        """
        for entry_point in (
            "request_emergency_break", "trigger_emergency_break",
        ):
            with self.subTest(entry_point=entry_point):
                self._reset_to_armed()
                self.assertTrue(self.client.send_pwms([1650] * 8))
                before = len(self.recorder.frames())

                self.assertTrue(getattr(self.client, entry_point)())

                self.assertIs(self.client.control_state, ControlState.ESTOP)
                emergency = [
                    payload for can_id, payload in self.recorder.frames()[before:]
                    if can_id == CAN_ID_EMERGENCY_BREAK
                ]
                self.assertTrue(emergency, f"{entry_point} sent no emergency frame")
                self.assertEqual(emergency[0][:3], b"\xAA\x55\x01")
                baseline = self.recorder.settle()
                self.assertEqual(
                    self.recorder.thruster_pwms()[-1], NEUTRAL_PWMS,
                    f"{entry_point} did not leave the outputs neutral",
                )
                self.assertTrue(
                    self.recorder.quiet_for(QUIET_WINDOW_S),
                    f"{entry_point} left the command path transmitting",
                )
                self.assertEqual(self.recorder.count(), baseline)

    def test_control_state_readout_tracks_the_real_machine(self):
        """
        Invariant: the sidebar readout follows the state machine rather than a
        second, independent notion of safety.
        """
        self.assertIs(self.client.control_state, ControlState.ARMED)
        observed = [
            (self.client.control_state, dashboard_app.control_state_presentation(self.client)[0])
        ]
        self.assertTrue(self.client.send_pwms([1650] * 8))
        observed.append(
            (self.client.control_state, dashboard_app.control_state_presentation(self.client)[0])
        )
        dashboard_app.all_stop(self.client)
        observed.append(
            (self.client.control_state, dashboard_app.control_state_presentation(self.client)[0])
        )
        dashboard_app.trip_emergency_break(self.client)
        observed.append(
            (self.client.control_state, dashboard_app.control_state_presentation(self.client)[0])
        )

        self.assertEqual(
            [state for state, _ in observed],
            [ControlState.ARMED, ControlState.RUNNING, ControlState.STOPPED, ControlState.ESTOP],
            "the handler sequence did not drive the expected state machine",
        )
        self.assertEqual(
            len({label for _, label in observed}), 4,
            f"the readout collapsed two of {observed} into the same label",
        )


class TestDashboardHandlersAgainstServer(unittest.TestCase):
    """
    The same handlers driving the real native ``sil_bridge_server``.

    Skipped when the binary has not been built; a skip is honest here and a
    silent pass would not be.
    """

    @classmethod
    def setUpClass(cls) -> None:
        cls.server_exe = require_server_executable()

    def setUp(self) -> None:
        self.server = SilServerProcess(self.server_exe)
        self.server.start()
        self.addCleanup(self.server.stop)

        self.client = SilDashboardClient(port=self.server.port)
        self.assertTrue(self.client.connect(), "Dashboard client must reach the SIL server")
        self.addCleanup(self.client.stop_server_process)

    def _wait_for(self, predicate, timeout_s: float, what: str) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.02)
        self.fail(
            f"Timed out after {timeout_s:.1f} s waiting for {what}; "
            f"control_state={self.client.control_state}, "
            f"actual_pwms={self.client.actual_pwms}, "
            f"actual_solenoid_mask=0x{self.client.actual_solenoid_mask:03X}, "
            f"sim_time_ms={self.client.sim_time_ms}"
        )

    def _wait_until_esc_armed(self) -> None:
        self._wait_for(
            lambda: self.client.sim_time_ms >= ESC_ARMING_SIM_MS,
            timeout_s=ESC_ARMING_SIM_MS / 1000.0 / SIM_MS_PER_WALL_S + 6.0,
            what="the control board to finish ESC arming",
        )

    def _thruster_frames_sent(self) -> int:
        return sum(
            1 for record in self.client.packet_log
            if record.can_id == CAN_ID_THRUSTER_CMD and record.direction == "Core -> STM32"
        )

    def test_all_stop_button_returns_the_board_to_neutral(self):
        """
        Invariant: the All Stop button leaves the native board at neutral.
        """
        self._wait_until_esc_armed()
        self.assertTrue(self.client.send_pwms([1650] * 8))
        self._wait_for(
            lambda: min(self.client.actual_pwms) > NEUTRAL_PWM_US,
            timeout_s=2.0, what="the outputs to leave neutral",
        )

        dashboard_app.all_stop(self.client)

        self.assertIs(self.client.control_state, ControlState.STOPPED)
        self._wait_for(
            lambda: self.client.actual_pwms == NEUTRAL_PWMS,
            timeout_s=1.0, what="the board outputs to return to neutral",
        )
        sent_at_stop = self._thruster_frames_sent()
        time.sleep(QUIET_WINDOW_S)
        self.assertEqual(
            self._thruster_frames_sent(), sent_at_stop,
            "The dashboard kept commanding after All Stop; the firmware "
            "watchdog could never lapse",
        )

    def test_all_stop_button_lets_the_firmware_watchdog_release_the_solenoids(self):
        """
        Invariant: All Stop ends the heartbeat, and the *firmware* -- not the
        host -- is what zeroes the solenoid mask.

        ``rov_safety_feed_heartbeat`` only runs on an accepted thruster command
        (app.c:133), and ``node2_force_neutral()`` calls ``bsp_solenoid_set(0)``
        on ``heartbeat_lost`` (app.c:183-189).  A stop that kept the 20 Hz
        worker alive would keep the solenoids energized behind a dashboard that
        says the vehicle is stopped.
        """
        self._wait_until_esc_armed()
        # The heartbeat must exist *before* the solenoids are energized:
        # rov_safety_is_heartbeat_lost() returns true until the first thruster
        # command establishes a deadline, and node2_force_neutral() zeroes the
        # solenoid mask on every step while that is the case (app.c:183-189).
        self.assertTrue(self.client.send_pwms([1650] * 8))
        self._wait_for(
            lambda: min(self.client.actual_pwms) > NEUTRAL_PWM_US,
            timeout_s=2.0, what="the outputs to leave neutral",
        )
        self.assertTrue(self.client.send_solenoids(ENERGIZED_SOLENOID_MASK))
        self._wait_for(
            lambda: self.client.actual_solenoid_mask == ENERGIZED_SOLENOID_MASK,
            timeout_s=1.0, what="the solenoid mask to be energized",
        )
        # A non-neutral command keeps the heartbeat fresh, so nothing clears them.
        time.sleep(3 * HEARTBEAT_TIMEOUT_S)
        self.assertEqual(
            self.client.actual_solenoid_mask, ENERGIZED_SOLENOID_MASK,
            "Precondition failed: the solenoids released while a command was "
            "still being refreshed, so this test would prove nothing",
        )

        dashboard_app.all_stop(self.client)

        self.assertIs(self.client.control_state, ControlState.STOPPED)
        self._wait_for(
            lambda: self.client.actual_solenoid_mask == 0,
            timeout_s=1.0,
            what="the firmware watchdog to zero the solenoid mask",
        )
        self._wait_for(
            lambda: self.client.actual_pwms == NEUTRAL_PWMS,
            timeout_s=1.0, what="the board outputs to return to neutral",
        )

    def test_emergency_button_trips_the_native_board(self):
        """
        Invariant: the emergency button's frame really latches the firmware
        emergency break, not just the dashboard's own state.
        """
        self._wait_until_esc_armed()
        self.assertTrue(dashboard_app.trip_emergency_break(self.client))
        self._wait_for(
            lambda: self.client.emergency_break_tripped,
            timeout_s=2.0, what="the board to latch the emergency break",
        )
        self.assertIs(self.client.control_state, ControlState.ESTOP)
        self.assertTrue(
            any("AA 55 01" in record.hex_data for record in self.client.packet_log),
            "The authorized 0xAA 0x55 0x01 signature is not in the packet log",
        )
        self._wait_for(
            lambda: self.client.actual_pwms == NEUTRAL_PWMS,
            timeout_s=1.0, what="the board outputs to be forced neutral",
        )
        sent_at_trip = self._thruster_frames_sent()
        time.sleep(QUIET_WINDOW_S)
        self.assertEqual(self._thruster_frames_sent(), sent_at_trip)
        self.assertEqual(
            self.client.actual_solenoid_mask, 0,
            "A latched firmware emergency break must release every solenoid",
        )


class _StreamlitFakeClient(_RecordingClient):
    """
    Client stand-in that the *real* Streamlit script can be driven against.

    ``dashboard_app`` resolves the client through ``st.cache_resource``, so
    replacing ``SilDashboardClient`` on the already-imported module is enough for
    the script's ``from sil_dashboard_client import SilDashboardClient`` to bind
    this class.  That is what makes it possible to click the real buttons and
    assert on what they did.

    Two deliberate deviations from a live client:

    * ``connect()`` does *not* bring the transport up.  ``main()`` auto-connects
      at the top and re-renders on a loop at the bottom, so a fake that really
      connected would rerun forever.  Tests that need a live transport call
      ``engage()``, and only after the refresh toggle is off.
    * ``connected`` counts reads.  A single render reads it a handful of times;
      a count that runs away means the script is in a rerun loop, so the guard
      reports the transport as down.  The loop breaks and the test fails loudly
      instead of hanging the suite.
    """

    MAX_CONNECTED_READS = 25

    def __init__(self) -> None:
        self._connected = False
        self._connected_reads = 0
        super().__init__(connected=False)
        # The read-only telemetry the dashboard renders.  All of it is empty, so
        # a render exercises every display branch without a SIL engine.
        self.sim_time_ms = 0
        self.frame_count = 0
        self.packet_log: List = []
        self.c_stdout_log: List = []
        self.actual_pwms = list(NEUTRAL_PWMS)
        self.actual_solenoid_mask = 0
        self.nav_data = None
        self.env_data = None
        self.power_data = None
        self.history_depth: List[float] = []
        self.emergency_break_tripped = False
        self.emergency_break_requested = False
        self.last_rx_monotonic: dict = {}
        self.pipeline_core_pwms = list(NEUTRAL_PWMS)
        self.pipeline_core_can_hex = ""
        self.pipeline_core_to_surface = {
            "depth": 0.0, "temp": 0.0, "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0,
            "accel_x": 0.0, "accel_y": 0.0, "accel_z": 9.81, "timestamp_us": 0,
            "raw_hex": "",
        }

    @property
    def connected(self) -> bool:  # type: ignore[override]
        if self._connected:
            self._connected_reads += 1
            if self._connected_reads > self.MAX_CONNECTED_READS:
                return False
        return self._connected

    @connected.setter
    def connected(self, value: bool) -> None:
        self._connected = bool(value)
        if not value:
            self._connected_reads = 0

    def connect(self) -> bool:
        self.calls.append(("connect", ()))
        self.connected_after_start = True
        return True  # deliberately does not set ``connected``

    def reset(self) -> None:
        self.calls.clear()
        self.connected = False
        self.connected_after_start = False
        self.started_server = False
        self.pwms = list(NEUTRAL_PWMS)
        self.solenoid_mask = 0
        self.control_state = ControlState.ARMED
        self.pipeline_surface_cmd.update(
            {"surge": 0.0, "sway": 0.0, "heave": 0.0, "yaw": 0.0}
        )

    def engage(self, control_state: ControlState) -> None:
        """Report a live transport, as an established session would."""
        self._connected_reads = 0
        self._connected = True
        self.control_state = control_state


#: ``st.cache_resource`` memoizes the client for the whole process, so every test
#: in this class has to share one instance or the second test would be asserting
#: against the first test's leftover client.
_STREAM_CLIENT = _StreamlitFakeClient()


def _app_test():
    """Return Streamlit's AppTest, or skip when the harness is unavailable."""
    try:
        from streamlit.testing.v1 import AppTest
    except Exception as exc:  # noqa: BLE001 - any import failure means "cannot run"
        raise unittest.SkipTest(
            f"streamlit.testing.v1 is unavailable ({exc}); the button-binding "
            f"tests are SKIPPED, not passing."
        )
    return AppTest


class TestDashboardUiThroughStreamlit(unittest.TestCase):
    """
    The real script, rendered by Streamlit, with the real buttons clicked.

    This is the only layer that proves a *button* is bound to a *handler*.
    Everything else proves what the handlers do.
    """

    @classmethod
    def setUpClass(cls) -> None:
        cls.AppTest = _app_test()
        cls.dashboard_script = DASHBOARD_DIR / "dashboard_app.py"
        if not cls.dashboard_script.is_file():
            raise unittest.SkipTest(f"{cls.dashboard_script} is missing")

    def setUp(self) -> None:
        _STREAM_CLIENT.reset()
        import sil_dashboard_client

        self._original_client_class = sil_dashboard_client.SilDashboardClient
        sil_dashboard_client.SilDashboardClient = lambda *a, **k: _STREAM_CLIENT
        self.addCleanup(
            setattr,
            sil_dashboard_client,
            "SilDashboardClient",
            self._original_client_class,
        )
        self.client = _STREAM_CLIENT
        self.at = self.AppTest.from_file(
            str(self.dashboard_script), default_timeout=30
        )

    def _run(self, tree=None):
        tree = self.at if tree is None else tree
        tree.run()
        self.assertEqual(
            [
                f"{type(element).__name__}: {getattr(element, 'value', element)}"
                for element in tree.exception
            ],
            [],
            "dashboard_app.main() raised while Streamlit rendered it",
        )
        return tree

    def _button(self, tree, label: str):
        matches = [b for b in tree.button if b.label == label]
        self.assertEqual(
            len(matches), 1,
            f"expected exactly one '{label}' button, found {[b.label for b in tree.button]}",
        )
        return matches[0]

    def test_the_dashboard_renders_without_a_streamlit_exception(self):
        """
        Invariant: the edited script still executes end to end under Streamlit.
        """
        self._run()
        labels = [b.label for b in self.at.button]
        for expected in (
            "All Stop (Hover)",
            "All Stop (1500 us)",
            "TRIP EMERGENCY BREAK (0x001)",
            "Restart Engine",
        ):
            with self.subTest(button=expected):
                self.assertIn(expected, labels)

    def test_the_sidebar_renders_the_control_state_metric(self):
        """
        Invariant: the deadman state is actually on screen, not just in the
        client.  A missing metric is a failed lookup, not a silent pass.
        """
        self._run()
        metric = next(
            (m for m in self.at.sidebar.metric if m.label == "Control State"), None
        )
        self.assertIsNotNone(
            metric, "the sidebar must render a 'Control State' metric"
        )
        self.assertEqual(metric.value, self.client.control_state.value)
        rendered = " ".join(c.value for c in self.at.sidebar.markdown)
        self.assertIn(
            "Control State", rendered,
            "the colour-coded control-state line is missing from the sidebar",
        )

    def _assert_single_stop(self, label: str) -> None:
        """
        Exactly one explicit stop, and not one command on the wire.

        The transport chatter (``start_server_process`` / ``connect``) is
        deliberately ignored: the click's ``st.rerun()`` re-renders the sidebar
        with the transport still down, which is a property of the fake, not of
        the button.
        """
        stops = [c for c in self.client.calls if c[0] == "request_stop"]
        self.assertEqual(
            stops, [("request_stop", ())],
            f"clicking '{label}' must issue exactly one explicit stop, "
            f"got calls {self.client.calls}",
        )
        self.assertEqual(
            self.client.send_calls(), [],
            f"clicking '{label}' must not put a command on the wire",
        )

    def test_all_stop_hover_button_calls_request_stop(self):
        """
        Invariant: the pilot tab's "All Stop (Hover)" button is bound to the
        deadman stop, not to a neutral command.
        """
        self._run()
        self.client.calls.clear()
        self._button(self.at, "All Stop (Hover)").click()
        self._run()
        self._assert_single_stop("All Stop (Hover)")

    def test_all_stop_thruster_button_calls_request_stop(self):
        """
        Invariant: the thruster tab's "All Stop (1500 us)" button is bound to
        the deadman stop, not to a neutral command.
        """
        self._run()
        self.client.calls.clear()
        self._button(self.at, "All Stop (1500 us)").click()
        self._run()
        self._assert_single_stop("All Stop (1500 us)")

    def _disable_auto_refresh(self) -> None:
        """
        Turn the render loop off before reporting a live transport.

        ``main()`` ends with ``sleep; st.rerun()`` whenever the transport is up
        and refresh is enabled, which would rerun the script forever.  The toggle
        is a real widget, so this goes through Streamlit like the operator would.
        """
        toggles = [t for t in self.at.sidebar.toggle if t.label.startswith("Auto-Refresh")]
        self.assertEqual(len(toggles), 1, "the auto-refresh toggle is missing from the sidebar")
        toggles[0].set_value(False)
        self._run()

    def test_emergency_button_calls_request_emergency_break(self):
        """
        Invariant: the emergency button is bound to the canonical emergency
        entry point, and it is clickable once the transport is up.
        """
        self._run()
        self._disable_auto_refresh()
        self.client.engage(ControlState.RUNNING)
        self._run()
        self.client.calls.clear()
        self._button(self.at, "TRIP EMERGENCY BREAK (0x001)").click()
        self._run()
        trips = [c for c in self.client.calls if c[0] == "request_emergency_break"]
        self.assertEqual(
            trips, [("request_emergency_break", ())],
            "clicking the emergency button must trip the break and nothing "
            f"else, got calls {self.client.calls}",
        )
        self.assertEqual(
            self.client.send_calls(), [],
            "the emergency button must not put a thruster command on the wire",
        )
        self.assertIs(self.client.control_state, ControlState.ESTOP)

    def test_emergency_button_is_disabled_without_a_transport(self):
        """
        Invariant: with no SIL engine there is nothing to trip, so the button
        must not be clickable.
        """
        self._run()
        button = self._button(self.at, "TRIP EMERGENCY BREAK (0x001)")
        self.assertTrue(
            button.disabled,
            "the emergency button must be disabled while disconnected",
        )


if __name__ == "__main__":  # pragma: no cover - convenience entry point
    unittest.main(verbosity=2)
