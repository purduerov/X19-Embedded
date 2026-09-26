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
import re
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

#: Every module-level function in dashboard_app that main() must delegate to.
#: If one of these is inlined back into main() the safety wiring becomes
#: untestable, so both directions are asserted.
HANDLER_NAMES = (
    "all_stop",
    "trip_emergency_break",
    "stop_engine",
    "restart_engine",
    "auto_refresh_due",
    "pilot_axes_changed",
    "thruster_targets_changed",
    "control_state_presentation",
    "ensure_transport",
)

#: The only conditions under which main() is allowed to transmit.  Anything else
#: -- notably the unattended auto-refresh loop -- must not put a frame on the
#: wire.  ``!=`` covers the inline solenoid diff, which is a comparison against
#: the client's current mask rather than a named predicate.
_LEGITIMATE_SEND_CONDITIONS = (
    "st.button",
    "changed(",
    "!=",
)


def _enclosing_condition(lines, index: int) -> str:
    """
    Return the nearest enclosing block condition for the line at ``index``.

    Walks backwards for the first line that is less indented than the target and
    opens a block, which is the condition the target line is executed under.  An
    ``if``, ``elif`` and ``else`` all open blocks, and all three must be reported:
    a send hidden under an ``elif`` (or under a bare ``else``) is exactly as
    unattended as one hidden under a plain ``if``.  A send with no enclosing block
    yields ``""``, which no legitimate condition matches.
    """
    target_indent = len(lines[index]) - len(lines[index].lstrip())
    openers = ("if ", "elif ", "else:", "else ", "for ", "while ", "with ")
    for offset in range(index - 1, -1, -1):
        line = lines[offset]
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        indent = len(line) - len(line.lstrip())
        if indent < target_indent and stripped.startswith(openers):
            return stripped
    return ""



class _RecordingClient:
    """
    Minimal stand-in that records every control call the UI makes.

    It deliberately implements only the client surface a button handler is
    allowed to touch.  Any other attribute access raises ``AttributeError``, so
    a handler that reaches for something unexpected fails loudly instead of
    quietly working against a permissive mock.
    """

    def __init__(self, control_state: ControlState = ControlState.ARMED,
                 connected: bool = True, estop_latched: bool = False) -> None:
        self.connected = connected
        self.control_state = control_state
        self.estop_latched = estop_latched
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

    def start_control_loop(self) -> bool:
        return self._record("start_control_loop")

    def stop_control_loop(self, send_neutral: bool = True) -> bool:
        return self._record("stop_control_loop", send_neutral)

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
        handlers = HANDLER_NAMES
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
        for handler in HANDLER_NAMES:
            with self.subTest(handler=handler):
                self.assertIn(f"{handler}(", source, f"main() must use {handler}()")

    def test_every_transmission_in_main_is_a_button_or_a_diff_branch(self):
        """
        Invariant: ``main()`` transmits only in response to an operator action.

        The auto-refresh loop runs unattended on every browser poll.  A single
        ``client.send_pwms(...)`` there would refresh the firmware heartbeat
        forever, which is precisely what stops Node 2 releasing the solenoids --
        and it would do so while the dashboard reports the vehicle is stopped.
        The loop must therefore be unable to transmit, and the only legitimate
        transmission sites are a button press or a per-tab slider diff.

        Checked structurally: every line naming a sender must sit inside a block
        whose condition is a widget action or a diff predicate.  A bare send in
        the refresh block has no such condition and fails here.
        """
        lines = inspect.getsource(dashboard_app.main).splitlines()
        senders = ("send_pwms", "send_surface_pilot_command", "send_solenoids")
        for index, line in enumerate(lines):
            if not any(sender in line for sender in senders):
                continue
            condition = _enclosing_condition(lines, index)
            with self.subTest(line=line.strip()):
                self.assertTrue(
                    any(token in condition for token in _LEGITIMATE_SEND_CONDITIONS),
                    f"{line.strip()!r} transmits, but its enclosing condition is "
                    f"{condition.strip()!r}. A transmission is only allowed under "
                    f"a button press or a slider diff.",
                )

    def test_both_all_stop_buttons_call_the_same_handler(self):
        """
        Invariant: the two All Stop buttons cannot drift apart.

        They are the operator's only way to stop simulated thrust, and they live
        in different tabs.  Each must reach ``all_stop()`` directly, so a future
        edit cannot leave one of them sending a command.
        """
        for label in ("All Stop (Hover)", "All Stop (1500 us)"):
            with self.subTest(button=label):
                self._assert_button_calls_handler(label, "all_stop(client)")

    def test_the_emergency_button_calls_its_handler(self):
        """
        Invariant: the E-stop button stays bound to ``trip_emergency_break``.

        The AppTest layer also clicks this button, but AppTest is a Streamlit
        internal surface that silently skips on an upgrade.  This structural twin
        means a rename of the button, or of the call inside it, cannot quietly
        unbind the emergency break from the deadman API.
        """
        self._assert_button_calls_handler(
            "TRIP EMERGENCY BREAK (0x001)", "trip_emergency_break(client)"
        )

    def _assert_button_calls_handler(self, label: str, call: str) -> None:
        """
        Assert the body of ``st.button(label)`` calls ``call``.

        The body is extracted by indentation, not by scanning forward to a
        ``st.rerun()``: the emergency button's block has no rerun, and a
        text-to-rerun split would sweep in unrelated code from later tabs and make
        the assertion vacuous.
        """
        lines = inspect.getsource(dashboard_app.main).splitlines()
        found = [i for i, line in enumerate(lines) if f'st.button("{label}"' in line]
        self.assertEqual(
            len(found), 1,
            f"expected exactly one '{label}' button, found {len(found)}",
        )
        start = found[0]
        indent = len(lines[start]) - len(lines[start].lstrip())
        body = []
        for line in lines[start + 1:]:
            stripped = line.strip()
            if stripped and not stripped.startswith("#"):
                if len(line) - len(line.lstrip()) <= indent:
                    break
            body.append(line)
        self.assertIn(
            call, "\n".join(body),
            f"the '{label}' button must call {call} directly; its body was "
            f"{chr(10).join(body)!r}",
        )


    def test_main_publishes_a_baseline_for_each_slider_tab(self):
        """
        Invariant: both slider tabs record their own baseline every render.

        Without the trailing ``publish_tab_baseline`` a tab would never register
        its own value, and its predicate would treat the operator's first slider
        move as unchanged -- a dead control surface.
        """
        source = inspect.getsource(dashboard_app.main)
        identifiers = re.findall(
            r"publish_tab_baseline\(client,\s*([A-Za-z_][A-Za-z_0-9]*),", source
        )
        self.assertEqual(
            len(identifiers), 2,
            "each of the two slider tabs must publish exactly one baseline per "
            f"render, found {identifiers}",
        )
        # Resolved through the module rather than matched as text, so renaming a
        # constant does not break this while still pinning that main() uses the
        # two distinct tab identities the predicates read.
        self.assertEqual(
            {getattr(dashboard_app, name, None) for name in identifiers},
            {dashboard_app.TAB_PILOT, dashboard_app.TAB_THRUSTER},
        )


class TestPresetDisplayTarget(unittest.TestCase):
    """
    Invariant: after a preset the panel shows the command that is being held.

    The pilot presets publish an axis command and hold it at 20 Hz
    (``test_the_pilot_presets_hold_their_command_too``), but they never moved the
    four sliders.  Streamlit seeds a keyed widget once and then owns it, so every
    later render showed 0.0 on the axis panel while four horizontal thrusters were
    held at full forward: the primary control surface read safe, and the diff that
    would have corrected it is one-directional by design and can only stay quiet.

    The fix is the one ``request_stop``'s docstring already names: separate the
    DISPLAY TARGET from the DIFF BASELINE, and have the preset write the display
    target it just commanded.  The veto, the latch gate, and the per-tab baselines
    are untouched - ``client.pwms`` stays the operator's commanded-value record,
    so nothing here can re-arm an All Stop.
    """

    PILOT_KEYS = dashboard_app.PILOT_SLIDER_KEYS

    def test_a_pilot_preset_publishes_the_axes_it_commanded(self):
        client = _RecordingClient(control_state=ControlState.RUNNING)
        state: dict = {}
        self.assertTrue(dashboard_app.pilot_preset(client, 0.5, 0.0, 0.0, 0.0, state=state))
        self.assertEqual(
            [state[key] for key in self.PILOT_KEYS],
            [0.5, 0.0, 0.0, 0.0],
            "the panel must display the command the preset just published",
        )
        self.assertEqual(
            client.calls,
            [("send_surface_pilot_command", (0.5, 0.0, 0.0, 0.0))],
            "the preset must still be the one command on the wire, and only one",
        )

    def test_a_thruster_preset_publishes_the_targets_it_commanded(self):
        client = _RecordingClient(control_state=ControlState.RUNNING)
        state: dict = {}
        self.assertTrue(dashboard_app.thruster_preset(client, [1650] * 8, state=state))
        self.assertEqual(
            [state[dashboard_app.THRUSTER_SLIDER_KEY(i)] for i in range(8)],
            [1650] * 8,
            "the thruster panel must display the command the preset just published",
        )
        self.assertEqual(client.calls, [("send_pwms", ([1650] * 8,))])

    def test_the_displayed_axes_still_do_not_re_send(self):
        """
        The veto is the reason this fix is safe, so it is asserted after it.

        The panel now reads 0.5 and the held command is 0.5, so the tab's diff sees
        a value that already matches the command.  One-directional suppression
        cannot cause a send; this proves the other half, that it stays quiet.
        """
        client = _RecordingClient(control_state=ControlState.RUNNING)
        _render_pilot_tab(client, 0.0, 0.0, 0.0, 0.0)  # first render: seeds
        state: dict = {}
        dashboard_app.pilot_preset(client, 0.5, 0.0, 0.0, 0.0, state=state)
        calls_before = len(client.calls)
        displayed = [state[key] for key in self.PILOT_KEYS]
        for render in range(3):
            with self.subTest(render=render):
                self.assertFalse(
                    _render_pilot_tab(client, *displayed),
                    "the panel echoing the held command must not re-send it",
                )
        self.assertEqual(client.calls[calls_before:], [])
        self.assertEqual(client.pwms, compute_thrust_allocation(0.5, 0.0, 0.0, 0.0))

    def test_the_display_target_write_happens_before_the_widget_is_created(self):
        """
        Streamlit refuses a session-state write after the widget exists.

        ``st.session_state`` raises ``StreamlitWidgetAlreadyInstantiatedError`` if
        the key's widget was already created in the same run, so each preset block
        has to be rendered before the sliders it moves.  That is why the pilot
        column is filled before the slider column: the layout is unchanged, the
        order of the two ``with`` blocks is load bearing, and nothing in Python
        says so if it is reversed.
        """
        source = inspect.getsource(dashboard_app.main)
        for handler, widget in (
            ("pilot_preset(client", "key=PILOT_SLIDER_KEYS[0]"),
            ("thruster_preset(client", "key=THRUSTER_SLIDER_KEY(i)"),
        ):
            with self.subTest(handler=handler):
                self.assertLess(
                    source.index(handler),
                    source.index(widget),
                    f"{handler} must run before its widget is created, or the "
                    "display-target write raises instead of taking effect",
                )

    def test_a_refused_preset_does_not_lie_about_what_it_displayed(self):
        """
        A command the client refused must not be shown as commanded.

        ``send_pwms``/``send_surface_pilot_command`` return False when the E-stop
        is latched or the transport is down.  Writing the display target anyway
        would put a target on screen that the board was never told, which is the
        same stale-display defect pointing the other way.
        """

        class RefusingClient(_RecordingClient):
            def send_surface_pilot_command(self, surge, sway, heave, yaw, pitch=0.0, roll=0.0):
                self.calls.append(("send_surface_pilot_command", (surge, sway, heave, yaw)))
                return False

        client = RefusingClient(control_state=ControlState.ESTOP, estop_latched=True)
        state: dict = {}
        self.assertFalse(dashboard_app.pilot_preset(client, 0.5, 0.0, 0.0, 0.0, state=state))
        self.assertEqual(state, {}, "a refused command must leave the panel alone")


def _render_pilot_tab(client, surge, sway, heave, yaw) -> bool:
    """
    Do exactly what ``main()``'s pilot tab does on one render.  True if it sent.

    Mirrors the tab's block line for line, so a test that drives this is driving
    the real control flow rather than an approximation of it.
    """
    axes = (surge, sway, heave, yaw)
    sent = False
    if dashboard_app.pilot_axes_changed(client, *axes):
        client.send_surface_pilot_command(*axes)
        sent = True
    dashboard_app.publish_tab_baseline(client, dashboard_app.TAB_PILOT, axes)
    return sent


def _render_thruster_tab(client, pwms) -> bool:
    """Do exactly what ``main()``'s thruster tab does on one render."""
    targets = list(pwms)
    sent = False
    if dashboard_app.thruster_targets_changed(client, targets):
        client.send_pwms(targets)
        sent = True
    dashboard_app.publish_tab_baseline(client, dashboard_app.TAB_THRUSTER, targets)
    return sent


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
        the pre-stop command, silently re-arming thrust.  The tab's own baseline
        is untouched by the stop, so the diff is empty.
        """
        client = _RecordingClient(control_state=ControlState.RUNNING)
        # The operator drags Surge to 0.5; the pilot tab sends it.
        _render_pilot_tab(client, 0.0, 0.0, 0.0, 0.0)
        self.assertTrue(_render_pilot_tab(client, 0.5, 0.0, 0.0, 0.0))
        calls_before_stop = len(client.calls)

        dashboard_app.all_stop(client)
        # Next rerun: the sliders still hold 0.5.
        self.assertFalse(
            _render_pilot_tab(client, 0.5, 0.0, 0.0, 0.0),
            "The rerun after All Stop re-sent the pilot axis; thrust would be "
            "re-armed without any operator action",
        )
        self.assertEqual(
            client.calls[calls_before_stop:],
            [("request_stop", ())],
            "the pilot tab must transmit nothing after an All Stop",
        )

    def test_thruster_all_stop_is_not_silently_reversed_by_the_next_rerun(self):
        """
        Invariant: pressing "All Stop (1500 us)" is not undone by the rerun
        that follows it.

        The per-thruster sliders keep the operator's last value across a rerun.
        An All Stop that rewrote the tab's baseline to neutral would make the
        next rerun see a difference and re-send the pre-stop PWMs.
        """
        client = _RecordingClient(control_state=ControlState.RUNNING)
        _render_thruster_tab(client, NEUTRAL_PWMS)
        self.assertTrue(_render_thruster_tab(client, [1650] * 8))
        calls_before_stop = len(client.calls)

        dashboard_app.all_stop(client)
        self.assertFalse(
            _render_thruster_tab(client, [1650] * 8),
            "The rerun after All Stop re-sent the thruster targets; thrust would "
            "be re-armed without any operator action",
        )
        self.assertEqual(
            client.calls[calls_before_stop:],
            [("request_stop", ())],
            "the thruster tab must transmit nothing after an All Stop",
        )

    def test_the_pilot_tab_sends_only_when_the_operator_moves_an_axis(self):
        client = _RecordingClient()
        # First render of a session: the tab adopts what the sliders show and
        # transmits nothing, because the operator has not asked for anything.
        self.assertFalse(_render_pilot_tab(client, 0.0, 0.0, 0.0, 0.0))
        self.assertEqual(client.send_calls(), [])
        # An operator move sends exactly once, then renders go quiet.
        self.assertTrue(_render_pilot_tab(client, 0.25, -0.5, 0.5, 0.25))
        self.assertEqual(len(client.send_calls()), 1)
        self.assertFalse(_render_pilot_tab(client, 0.25, -0.5, 0.5, 0.25))
        self.assertEqual(len(client.send_calls()), 1)
        # Any single axis is enough to trigger a send.
        for axes in ((0.0, -0.5, 0.5, 0.25), (0.25, 0.0, 0.5, 0.25),
                     (0.25, -0.5, 0.0, 0.25), (0.25, -0.5, 0.5, 0.0),
                     (0.26, -0.5, 0.5, 0.25)):
            with self.subTest(axes=axes):
                self.assertTrue(_render_pilot_tab(client, *axes))
                self.assertFalse(_render_pilot_tab(client, *axes))


    def test_the_thruster_tab_sends_only_when_the_operator_moves_a_slider(self):
        client = _RecordingClient()
        self.assertFalse(_render_thruster_tab(client, NEUTRAL_PWMS))
        self.assertEqual(client.send_calls(), [])
        self.assertTrue(_render_thruster_tab(client, [1650] * 8))
        self.assertEqual(client.send_calls(), [("send_pwms", ([1650] * 8,))])
        self.assertFalse(_render_thruster_tab(client, [1650] * 8))
        self.assertTrue(_render_thruster_tab(client, [1600] * 8))
        self.assertFalse(_render_thruster_tab(client, [1600] * 8))
        # A single channel is enough.
        self.assertTrue(_render_thruster_tab(client, [1600] * 7 + [1610]))
        self.assertFalse(_render_thruster_tab(client, [1600] * 7 + [1610]))


    def test_a_pilot_command_does_not_rearm_the_thruster_tabs_target(self):
        """
        Invariant: using one control tab cannot re-arm the other tab's command.

        Streamlit renders every tab on every run, so both diffs run whether or
        not the operator touched that tab.  ``send_surface_pilot_command()`` ends
        in ``send_pwms()``, which writes the shared ``client.pwms``; a thruster
        diff reading that shared field sees the pilot's allocation as a slider
        move and re-sends the thruster tab's stale target at 20 Hz, with no
        operator action on the thruster tab.
        """
        client = _RecordingClient(control_state=ControlState.RUNNING)
        # The operator sets full forward on the thruster tab: the session's first
        # render seeds the baseline, the second carries the operator's move.
        _render_thruster_tab(client, NEUTRAL_PWMS)
        self.assertTrue(_render_thruster_tab(client, [1650] * 8))
        # The operator moves to the pilot tab and asks for half surge.
        _render_pilot_tab(client, 0.0, 0.0, 0.0, 0.0)
        self.assertTrue(_render_pilot_tab(client, 0.5, 0.0, 0.0, 0.0))

        # Every following automatic render, with the thruster sliders untouched.
        for render in range(3):
            with self.subTest(render=render):
                self.assertFalse(
                    _render_thruster_tab(client, [1650] * 8),
                    "the thruster tab re-sent its stale target because the pilot "
                    "tab published a command; the vehicle would hold full "
                    "forward on all eight thrusters with no operator action",
                )
        self.assertEqual(
            [call for call in client.send_calls() if call[0] == "send_pwms"],
            [("send_pwms", ([1650] * 8,))],
            "only the operator's own thruster command may be sent",
        )

    def test_a_thruster_command_does_not_cancel_pilot_input(self):
        """
        Invariant: the mirror direction.  A thruster command must not make the
        pilot tab re-send or re-publish anything on the next render.

        This also pins the precondition of the All Stop guarantee across tabs:
        after an All Stop on either tab, the *other* tab's next render is silent.
        """
        client = _RecordingClient(control_state=ControlState.RUNNING)
        _render_pilot_tab(client, 0.0, 0.0, 0.0, 0.0)
        self.assertTrue(_render_pilot_tab(client, 0.5, 0.0, 0.0, 0.0))
        # The operator reaches for the thruster tab directly.
        _render_thruster_tab(client, NEUTRAL_PWMS)
        self.assertTrue(_render_thruster_tab(client, [1700] * 8))

        for render in range(3):
            with self.subTest(render=render):
                self.assertFalse(
                    _render_pilot_tab(client, 0.5, 0.0, 0.0, 0.0),
                    "the pilot tab re-sent because the thruster tab published a "
                    "command, so the two tabs fight over the vehicle",
                )

        # An All Stop on the thruster tab must not be undone by the pilot tab.
        dashboard_app.all_stop(client)
        for render in range(2):
            with self.subTest(after_stop=render):
                self.assertFalse(_render_pilot_tab(client, 0.5, 0.0, 0.0, 0.0))
                self.assertFalse(_render_thruster_tab(client, [1700] * 8))

    def test_stop_engine_does_not_clear_a_latched_estop(self):
        """
        Invariant: shutting the engine down is not a way to disarm the E-stop.
        """
        client = _RecordingClient(control_state=ControlState.ESTOP,
                                  connected=False, estop_latched=True)
        dashboard_app.stop_engine(client)
        self.assertIn("stop_server_process", [name for name, _ in client.calls])
        self.assertNotIn("start_server_process", [name for name, _ in client.calls])
        self.assertNotIn("connect", [name for name, _ in client.calls])

    def test_restart_engine_is_the_explicit_path_that_clears_the_latch(self):
        """
        Invariant: restarting the engine is the only dashboard action that
        starts a new engine, and it is the documented way out of a latch.
        """
        client = _RecordingClient(control_state=ControlState.ESTOP,
                                  connected=False, estop_latched=True)
        self.assertTrue(dashboard_app.restart_engine(client, settle_s=0.0))
        self.assertEqual(
            [name for name, _ in client.calls],
            ["stop_server_process", "start_server_process", "connect"],
        )

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
        colors = [color for _, color, _ in presentations.values()]
        self.assertEqual(
            len(set(colors)), len(colors),
            f"all five control states need distinguishable colours, got {colors}; "
            f"a shared colour would hide the difference between two states",
        )

        # The wording has to describe what the state does, not merely exist.
        # Swapping two help strings fails here.
        expected_meaning = {
            # A running worker is what keeps the board out of its failsafe.
            ControlState.RUNNING: ("refresh", "20 hz", "transmitting"),
            # Armed is the state whose whole point is that the watchdog lapses.
            ControlState.ARMED: ("watchdog", "lapse", "not refreshed"),
            # Stopped is an explicit stop, and nothing is being transmitted.
            ControlState.STOPPED: ("explicit", "no command", "transmitted"),
            # A latched break refuses every command until the engine restarts.
            ControlState.ESTOP: ("latched", "refused", "restarted"),
            # Disconnected means the control path is dead, not merely idle.
            ControlState.DISCONNECTED: ("no sil transport", "nothing is transmitted", "dead"),
        }
        for state, keywords in expected_meaning.items():
            help_text = presentations[state][2].lower()
            for keyword in keywords:
                with self.subTest(state=state, keyword=keyword):
                    self.assertIn(
                        keyword, help_text,
                        f"the {state.name} help text must say {keyword!r}; it "
                        f"reads {presentations[state][2]!r}",
                    )

    def test_an_unrecognised_control_state_is_presented_as_unsafe(self):
        """
        Invariant: a state this dashboard does not know about is shown as
        unknown and unsafe, never silently rendered as a safe idle state.
        """
        exotic = _RecordingClient(control_state=ControlState.ARMED)
        exotic.control_state = "some_future_state"
        label, color, help_text = dashboard_app.control_state_presentation(exotic)
        self.assertEqual(label, "UNKNOWN")
        self.assertEqual(color, "red")
        self.assertIn("unsafe", help_text.lower())

    def test_latched_estop_blocks_automatic_reconnect(self):
        """
        Invariant: a latched E-stop cannot be cleared by a transport hiccup.

        ``SilDashboardClient.connect()`` clears the E-stop latch.  If the UI
        reconnected automatically whenever the transport looked down, a reader
        error would silently disarm a tripped emergency break with no operator
        action.  The reconnect therefore requires an explicit button press.
        """
        client = _RecordingClient(control_state=ControlState.ESTOP, connected=False,
                                  estop_latched=True)
        self.assertFalse(dashboard_app.ensure_transport(client))
        self.assertEqual(
            client.calls, [],
            "A latched E-stop must not be cleared by an automatic reconnect",
        )
        self.assertFalse(client.started_server)
        self.assertFalse(client.connected_after_start)

    def test_a_fresh_session_seeds_its_baseline_from_the_widget_source(self):
        """
        Invariant: a tab that has published nothing treats its seeded sliders as
        "nothing to send", and adopts the seeded value as its baseline.

        Streamlit seeds the sliders from the client's own current command, so on a
        session's first render the widget values and the seed source agree by
        construction and the diff is empty.
        """
        for tab in (dashboard_app.TAB_PILOT, dashboard_app.TAB_THRUSTER):
            with self.subTest(tab=tab):
                client = _RecordingClient()
                # The client's command is not neutral: a preset or another tab
                # moved it, and a fresh session's sliders are seeded from it.
                if tab == dashboard_app.TAB_THRUSTER:
                    client.send_pwms([1650] * 8)
                    seeded = [1650] * 8
                else:
                    client.send_surface_pilot_command(0.5, 0.0, 0.0, 0.0)
                    seeded = (0.5, 0.0, 0.0, 0.0)
                self.assertIsNone(
                    dashboard_app.tab_baseline(client, tab),
                    "a fresh client must have no baseline for this tab yet",
                )
                sent = (
                    _render_pilot_tab(client, *seeded)
                    if tab == dashboard_app.TAB_PILOT
                    else _render_thruster_tab(client, seeded)
                )
                self.assertFalse(sent, "a first render must not transmit")
                self.assertEqual(
                    dashboard_app.tab_baseline(client, tab),
                    dashboard_app.tab_seed_value(client, tab),
                    "the first baseline must be the value the widgets were "
                    "seeded from",
                )
                self.assertEqual(
                    tuple(seeded), tuple(dashboard_app.tab_seed_value(client, tab)),
                    "the seeded widget values and the seed source must agree, or "
                    "the first render could not be a no-op by construction",
                )

    def test_a_first_render_never_records_widget_values_as_the_baseline(self):
        """
        Invariant: a tab's first baseline is the seed source, even if that
        render's widget values disagree with it.

        A first render adopts ``tab_seed_value`` precisely because Streamlit seeds
        the widgets from the same place.  If the two ever drift -- a new slider
        default, a rounding change, a tab that reads a different field -- then
        recording the widget values would silently adopt the operator's control
        position as "the command", and the difference between the control and the
        command would never be sent again.  Seeding from the command side keeps the
        discrepancy visible on the next render.
        """
        client = _RecordingClient(control_state=ControlState.RUNNING)
        _render_thruster_tab(client, NEUTRAL_PWMS)          # first render: seeds
        self.assertTrue(_render_thruster_tab(client, [1700] * 8))  # full forward
        dashboard_app.forget_tab_baselines(client)  # a new session's first render
        self.assertIsNone(dashboard_app.tab_baseline(client, dashboard_app.TAB_THRUSTER))

        # The sliders disagree with the command: a no-op for now, because there is
        # no baseline to compare against yet.
        self.assertFalse(_render_thruster_tab(client, [1200] * 8))
        self.assertEqual(
            dashboard_app.tab_baseline(client, dashboard_app.TAB_THRUSTER),
            tuple([1700] * 8),
            "the first baseline must be the command, not the widget value",
        )
        # The next render sees control 1200 against command 1700 and sends it.
        self.assertTrue(
            _render_thruster_tab(client, [1200] * 8),
            "the difference between a control and the command was swallowed",
        )
        self.assertEqual(client.pwms, [1200] * 8)

    def test_a_second_session_cannot_rearm_the_previous_ones_command(self):
        """
        Invariant: a second session sharing the cached client sends nothing on its
        first render, for either tab.

        ``st.cache_resource`` hands every session the same client, so the new
        session inherits the old session's baseline while its own sliders are
        seeded from the client's current command.  Diffing those two values is an
        operator action nobody took, and the real client turns it into a RUNNING
        state with a live 20 Hz worker.
        """
        for tab in (dashboard_app.TAB_PILOT, dashboard_app.TAB_THRUSTER):
            with self.subTest(tab=tab):
                client = _RecordingClient(control_state=ControlState.RUNNING)
                if tab == dashboard_app.TAB_THRUSTER:
                    _render_thruster_tab(client, NEUTRAL_PWMS)
                    self.assertTrue(_render_thruster_tab(client, [1650] * 8))
                    # The other tab publishes, moving the command out from under
                    # the baseline, exactly as the pilot tab does in production.
                    client.send_surface_pilot_command(0.5, 0.0, 0.0, 0.0)
                    dashboard_app.all_stop(client)
                    seeded = [int(v) for v in client.pwms]
                    self.assertNotEqual(seeded, [1650] * 8)
                    calls_before = len(client.calls)
                    self.assertFalse(
                        _render_thruster_tab(client, seeded),
                        "the new session re-sent the previous session's slider "
                        "value, re-arming thrust with no operator action",
                    )
                else:
                    _render_pilot_tab(client, 0.0, 0.0, 0.0, 0.0)
                    self.assertTrue(_render_pilot_tab(client, 0.5, 0.0, 0.0, 0.0))
                    client.send_pwms([1650] * 8)
                    dashboard_app.all_stop(client)
                    seeded = (
                        client.pipeline_surface_cmd["surge"],
                        client.pipeline_surface_cmd["sway"],
                        client.pipeline_surface_cmd["heave"],
                        client.pipeline_surface_cmd["yaw"],
                    )
                    calls_before = len(client.calls)
                    self.assertFalse(
                        _render_pilot_tab(client, *seeded),
                        "the new session re-sent the previous session's axes",
                    )
                self.assertEqual(
                    client.calls[calls_before:],
                    [],
                    f"the '{tab}' tab must transmit nothing on a new session's "
                    f"first render",
                )

    def test_a_second_session_cannot_rearm_a_pilot_preset_command(self):
        """
        Invariant: the pilot tab's own cross-session guard, exposed by a preset.

        A flight preset publishes axes without moving the sliders, so the tab's
        baseline keeps the *pre-preset* axis while ``pipeline_surface_cmd`` -- the
        source a new session's axes are seeded from -- holds the preset's axes.  A
        new session therefore sees its own seeded axes differ from the inherited
        baseline.  "The axes already show the command" is what makes that a
        no-op, and it is the only thing that does: without it the new session
        re-sends and the vehicle re-arms on its first render.
        """
        client = _RecordingClient(control_state=ControlState.RUNNING)
        _render_pilot_tab(client, 0.0, 0.0, 0.0, 0.0)      # first render: seeds
        # "Forward (+0.5 Surge)": publishes axes, leaves the sliders where they are.
        client.send_surface_pilot_command(0.5, 0.0, 0.0, 0.0)
        dashboard_app.all_stop(client)
        self.assertEqual(
            dashboard_app.tab_baseline(client, dashboard_app.TAB_PILOT),
            (0.0, 0.0, 0.0, 0.0),
            "precondition: the preset moved the command, not the baseline",
        )

        # A new session's axes are seeded from the command, not from the baseline.
        seeded = (
            client.pipeline_surface_cmd["surge"],
            client.pipeline_surface_cmd["sway"],
            client.pipeline_surface_cmd["heave"],
            client.pipeline_surface_cmd["yaw"],
        )
        self.assertEqual(seeded, (0.5, 0.0, 0.0, 0.0))
        calls_before = len(client.calls)
        for render in range(3):
            with self.subTest(render=render):
                self.assertFalse(
                    _render_pilot_tab(client, *seeded),
                    f"render {render} of a new session re-sent the pilot preset "
                    f"with no operator action, re-arming thrust",
                )
        self.assertEqual(client.calls[calls_before:], [])
        self.assertEqual(
            client.pwms, compute_thrust_allocation(0.5, 0.0, 0.0, 0.0),
            "nothing may have re-published a command; the stop must still hold",
        )

    def test_preset_buttons_hold_their_command_until_an_axis_moves(self):
        """
        Invariant, and an operator-facing behaviour change: a preset button's
        command is *held*, not cancelled by the next render.

        The presets ("All Forward", "Apply Sync Throttle", the six flight presets)
        publish a command without moving any slider.  Because they do not touch the
        diff baseline, the following render finds the sliders unchanged and sends
        nothing -- so the preset now holds until the operator stops the vehicle or
        moves a control.  Previously each preset was silently cancelled by its own
        next render, which made those buttons look broken.
        """
        client = _RecordingClient(control_state=ControlState.RUNNING)
        _render_thruster_tab(client, NEUTRAL_PWMS)

        # The operator presses "All Forward (1650 us)": a raw send, no widget move.
        client.send_pwms([1650] * 8)
        for render in range(3):
            with self.subTest(render=render):
                self.assertFalse(
                    _render_thruster_tab(client, NEUTRAL_PWMS),
                    "the preset command was cancelled by a later render; the "
                    "operator asked for sustained forward thrust",
                )
        self.assertEqual(client.pwms, [1650] * 8)

        # Moving a control is still honoured, and still wins over the preset.
        self.assertTrue(_render_thruster_tab(client, [1550] * 8))
        self.assertEqual(client.pwms, [1550] * 8)
        self.assertFalse(_render_thruster_tab(client, [1550] * 8))
        # Dragging back to neutral stops the vehicle.
        self.assertTrue(_render_thruster_tab(client, NEUTRAL_PWMS))
        self.assertEqual(client.pwms, NEUTRAL_PWMS)

    def test_the_pilot_presets_hold_their_command_too(self):
        client = _RecordingClient(control_state=ControlState.RUNNING)
        _render_pilot_tab(client, 0.0, 0.0, 0.0, 0.0)
        # "Forward (+0.5 Surge)": a raw send with the axes left where they are.
        client.send_surface_pilot_command(0.5, 0.0, 0.0, 0.0)
        for render in range(3):
            with self.subTest(render=render):
                self.assertFalse(
                    _render_pilot_tab(client, 0.0, 0.0, 0.0, 0.0),
                    "the flight preset was cancelled by a later render",
                )
        # Moving an axis is honoured and supersedes the preset.
        self.assertTrue(_render_pilot_tab(client, 0.25, 0.0, 0.0, 0.0))
        self.assertFalse(_render_pilot_tab(client, 0.25, 0.0, 0.0, 0.0))
        # Centring the stick from there is honoured, and stops the vehicle.
        self.assertTrue(_render_pilot_tab(client, 0.0, 0.0, 0.0, 0.0))
        self.assertFalse(_render_pilot_tab(client, 0.0, 0.0, 0.0, 0.0))

    def test_the_latch_gate_does_not_depend_on_the_displayed_state(self):

        """
        Invariant: the interlock keys off the latch itself, not off
        ``control_state``.

        ``disconnect()`` overwrites ``control_state`` with ``DISCONNECTED`` while
        leaving the latch set, so a gate that read the state would let "Stop
        Engine" -- whose intent is to shut the engine *down* -- clear a tripped
        emergency break on the very next render.
        """
        for state in ControlState:
            with self.subTest(state=state):
                client = _RecordingClient(control_state=state, connected=False,
                                          estop_latched=True)
                self.assertFalse(
                    dashboard_app.ensure_transport(client),
                    f"a latched E-stop must block the reconnect whatever the "
                    f"displayed state is (it was {state.name})",
                )
                self.assertEqual(client.calls, [])

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

    def test_a_pilot_command_does_not_rearm_the_thruster_tabs_stale_target(self):
        """
        Invariant: the C-1 hazard, on the wire, with the real client.

        The operator sets full forward on the thruster tab, then asks for half
        surge on the pilot tab.  ``send_surface_pilot_command`` ends in
        ``send_pwms``, so the shared ``client.pwms`` becomes the pilot's
        allocation.  If the thruster tab's diff read that shared field, the next
        render would treat the pilot's allocation as a slider move and re-send the
        thruster tab's stale target -- no operator action on that tab, at 20 Hz,
        vertical bank included.  The refreshed heartbeat would then stop the
        firmware watchdog from ever releasing the solenoids.

        Both directions are driven, because the tabs share one vehicle.
        """
        self._reset_to_armed()
        # The operator sets full forward on the thruster tab: the session's first
        # render seeds the baseline, the second carries the operator's move.
        _render_thruster_tab(self.client, NEUTRAL_PWMS)
        self.assertTrue(_render_thruster_tab(self.client, [1650] * 8))
        self.assertIs(self.client.control_state, ControlState.RUNNING)
        self.assertTrue(
            self.recorder.wait_for_count(WORKER_SPINUP_FRAMES, WORKER_SPINUP_S)
        )

        # The operator moves to the pilot tab and asks for half surge.
        _render_pilot_tab(self.client, 0.0, 0.0, 0.0, 0.0)
        self.assertTrue(_render_pilot_tab(self.client, 0.5, 0.0, 0.0, 0.0))
        expected_held = compute_thrust_allocation(0.5, 0.0, 0.0, 0.0)
        self.assertEqual(self.client._last_command, expected_held)

        # Every following automatic render, with the thruster sliders untouched.
        for render in range(3):
            with self.subTest(direction="pilot_then_thruster", render=render):
                self.assertFalse(
                    _render_thruster_tab(self.client, [1650] * 8),
                    "the thruster tab re-sent its stale target because the pilot "
                    "tab published a command",
                )
                self.assertEqual(
                    self.client._last_command, expected_held,
                    "the held command was replaced by the other tab's target",
                )
                self.assertFalse(
                    _render_pilot_tab(self.client, 0.5, 0.0, 0.0, 0.0),
                    "the pilot tab re-sent because the thruster tab published a "
                    "command; the two tabs must not fight over the vehicle",
                )
                self.assertEqual(self.client._last_command, expected_held)

        # And the command path is still sending exactly the pilot's command.
        time.sleep(2 * CONTROL_PERIOD_S)
        self.assertEqual(self.recorder.thruster_pwms()[-1], expected_held)

    def test_a_second_session_cannot_rearm_a_stopped_vehicle(self):
        """
        Invariant: the cross-session re-arm, end to end on the wire.

        ``st.cache_resource`` gives every browser session the same client.  Session
        A drives the thrusters, then the pilot, then presses All Stop; the stop
        deliberately leaves ``client.pwms`` holding the pre-stop value.  Session B
        then opens the dashboard: its sliders are seeded from that stale value while
        its inherited baseline still holds session A's slider value, so the diff
        sees a difference nobody caused -- and the real client answers it by setting
        RUNNING and starting the 20 Hz worker, with the heartbeat refreshed so the
        firmware watchdog can no longer release the solenoids.

        Session B must transmit nothing, and the worker must stay stopped.
        """
        self._reset_to_armed()
        # --- session A -------------------------------------------------------
        _render_thruster_tab(self.client, NEUTRAL_PWMS)
        self.assertTrue(_render_thruster_tab(self.client, [1650] * 8))
        self.assertIs(self.client.control_state, ControlState.RUNNING)
        _render_pilot_tab(self.client, 0.0, 0.0, 0.0, 0.0)
        self.assertTrue(_render_pilot_tab(self.client, 0.5, 0.0, 0.0, 0.0))
        self.assertTrue(
            self.recorder.wait_for_count(WORKER_SPINUP_FRAMES, WORKER_SPINUP_S)
        )

        dashboard_app.all_stop(self.client)
        self.assertIs(self.client.control_state, ControlState.STOPPED)
        self.assertWorkerStopped()
        after_stop = self.recorder.settle()
        stale_pwms = list(self.client.pwms)
        self.assertNotEqual(
            stale_pwms, NEUTRAL_PWMS,
            "precondition: request_stop() leaves the commanded-value record "
            "stale, which is what session B's sliders get seeded from",
        )

        # --- session B: a fresh browser, the same cached client --------------
        # No widget state, so Streamlit seeds the sliders from the client.
        session_b_thruster = [int(value) for value in self.client.pwms]
        session_b_pilot = (
            self.client.pipeline_surface_cmd["surge"],
            self.client.pipeline_surface_cmd["sway"],
            self.client.pipeline_surface_cmd["heave"],
            self.client.pipeline_surface_cmd["yaw"],
        )
        self.assertNotEqual(session_b_thruster, [1650] * 8)
        self.assertNotEqual(session_b_thruster, NEUTRAL_PWMS)

        for render in range(3):
            with self.subTest(render=render):
                self.assertFalse(
                    _render_thruster_tab(self.client, session_b_thruster),
                    f"render {render} of a new session re-armed the thruster "
                    f"command with no operator action",
                )
                self.assertFalse(
                    _render_pilot_tab(self.client, *session_b_pilot),
                    f"render {render} of a new session re-armed the pilot "
                    f"command with no operator action",
                )
                self.assertIs(self.client.control_state, ControlState.STOPPED)
                self.assertWorkerStopped()

        time.sleep(2 * CONTROL_PERIOD_S)
        self.assertEqual(
            self.recorder.count(), after_stop,
            "a new session put frames on the wire with no operator action",
        )
        self.assertTrue(
            self.recorder.quiet_for(QUIET_WINDOW_S),
            "the command path came back to life on its own",
        )
        self.assertEqual(self.client._last_command, NEUTRAL_PWMS)

    def test_a_second_session_cannot_rearm_after_a_preset_command(self):
        """
        Invariant: the same cross-session case with a preset's command in flight.

        The preset's command is the current one, so a fresh session's sliders are
        seeded from it.  The session must not fight the preset by re-sending its
        own inherited slider value, and the preset's held command must survive.
        """
        self._reset_to_armed()
        _render_thruster_tab(self.client, NEUTRAL_PWMS)
        # "All Forward (1650 us)": a raw send that leaves the sliders at 1500.
        self.assertTrue(self.client.send_pwms([1650] * 8))
        self.assertIs(self.client.control_state, ControlState.RUNNING)
        self.assertTrue(
            self.recorder.wait_for_count(WORKER_SPINUP_FRAMES, WORKER_SPINUP_S)
        )

        # A new session's sliders are seeded from the preset's command.
        session_b_thruster = [int(value) for value in self.client.pwms]
        self.assertEqual(session_b_thruster, [1650] * 8)
        for render in range(2):
            with self.subTest(render=render):
                self.assertFalse(_render_thruster_tab(self.client, session_b_thruster))
        self.assertEqual(
            self.client._last_command, [1650] * 8,
            "a new session must not disturb a command the operator asked for",
        )

    def test_a_second_session_cannot_rearm_a_pilot_preset_command(self):
        """
        Invariant: the pilot preset's cross-session case, on the wire.

        A flight preset publishes axes without moving the sliders, so the pilot
        tab's baseline keeps the pre-preset axis while ``pipeline_surface_cmd`` --
        the source a new session's axes are seeded from -- holds the preset's
        axes.  The new session sees its own axes differ from the inherited
        baseline, and must treat that as nothing to send.
        """
        self._reset_to_armed()
        _render_pilot_tab(self.client, 0.0, 0.0, 0.0, 0.0)
        # "Forward (+0.5 Surge)": a raw send with the axes left at zero.
        self.assertTrue(self.client.send_surface_pilot_command(0.5, 0.0, 0.0, 0.0))
        self.assertIs(self.client.control_state, ControlState.RUNNING)
        self.assertTrue(
            self.recorder.wait_for_count(WORKER_SPINUP_FRAMES, WORKER_SPINUP_S)
        )
        dashboard_app.all_stop(self.client)
        self.assertIs(self.client.control_state, ControlState.STOPPED)
        self.assertWorkerStopped()
        after_stop = self.recorder.settle()

        seeded = (
            self.client.pipeline_surface_cmd["surge"],
            self.client.pipeline_surface_cmd["sway"],
            self.client.pipeline_surface_cmd["heave"],
            self.client.pipeline_surface_cmd["yaw"],
        )
        self.assertEqual(seeded, (0.5, 0.0, 0.0, 0.0))
        self.assertEqual(
            dashboard_app.tab_baseline(self.client, dashboard_app.TAB_PILOT),
            (0.0, 0.0, 0.0, 0.0),
            "precondition: the preset moved the command, not the baseline",
        )

        for render in range(3):
            with self.subTest(render=render):
                self.assertFalse(_render_pilot_tab(self.client, *seeded))
                self.assertIs(self.client.control_state, ControlState.STOPPED)
                self.assertWorkerStopped()
        self.assertTrue(
            self.recorder.quiet_for(QUIET_WINDOW_S),
            "a new session brought the command path back to life on its own",
        )
        self.assertEqual(self.recorder.count(), after_stop)

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
                    _render_pilot_tab(self.client, 0.0, 0.0, 0.0, 0.0)
                    self.assertTrue(_render_pilot_tab(self.client, *pending))
                else:
                    _render_thruster_tab(self.client, NEUTRAL_PWMS)
                    self.assertTrue(_render_thruster_tab(self.client, *pending))
                self.assertIs(self.client.control_state, ControlState.RUNNING)
                self.assertTrue(
                    self.recorder.wait_for_count(WORKER_SPINUP_FRAMES, WORKER_SPINUP_S)
                )

                dashboard_app.all_stop(self.client)
                after_stop = self.recorder.settle()

                # Every following render: the operator touched nothing.
                for render in range(2):
                    with self.subTest(render=render):
                        if tab == "pilot":
                            sent = _render_pilot_tab(self.client, *pending)
                        else:
                            sent = _render_thruster_tab(self.client, *pending)
                        self.assertFalse(
                            sent,
                            f"the {tab} tab re-sent the pre-stop command on a "
                            f"later render, re-arming thrust with no operator "
                            f"action",
                        )
                time.sleep(2 * CONTROL_PERIOD_S)
                self.assertEqual(
                    self.recorder.count(), after_stop,
                    f"A frame was transmitted after the {tab} tab's All Stop",
                )

    def test_neither_tab_rearms_the_others_command_after_a_stop(self):
        """
        Invariant: an All Stop on one tab is not undone by the *other* tab's
        diff on a later render.

        Streamlit renders every tab every run, so both diffs run regardless of
        which tab the operator is looking at.  Both tabs are given real operator
        input first, because a tab whose baseline was never established has
        nothing to re-send and would make this test vacuous.
        """
        self._reset_to_armed()
        _render_thruster_tab(self.client, NEUTRAL_PWMS)
        self.assertTrue(_render_thruster_tab(self.client, [1650] * 8))
        _render_pilot_tab(self.client, 0.0, 0.0, 0.0, 0.0)
        self.assertTrue(_render_pilot_tab(self.client, 0.5, 0.0, 0.0, 0.0))
        self.assertIs(self.client.control_state, ControlState.RUNNING)
        self.assertTrue(
            self.recorder.wait_for_count(WORKER_SPINUP_FRAMES, WORKER_SPINUP_S)
        )

        dashboard_app.all_stop(self.client)
        after_stop = self.recorder.settle()

        for render in range(3):
            with self.subTest(render=render):
                self.assertFalse(_render_pilot_tab(self.client, 0.5, 0.0, 0.0, 0.0))
                self.assertFalse(_render_thruster_tab(self.client, [1650] * 8))
        self.assertIs(self.client.control_state, ControlState.STOPPED)
        self.assertTrue(
            self.recorder.quiet_for(QUIET_WINDOW_S),
            "a cross-tab render re-armed the command path after an All Stop",
        )
        self.assertEqual(self.recorder.thruster_pwms()[-1], NEUTRAL_PWMS)

    def test_stop_engine_does_not_restart_a_latched_emergency_break(self):
        """
        Invariant: the real client's latch survives "Stop Engine" and the next
        automatic render.

        This is the I-1 hole: ``stop_server_process()`` calls ``disconnect()``,
        which overwrites ``control_state`` with ``DISCONNECTED`` while leaving the
        latch set.  A gate that read ``control_state`` would therefore start a
        new engine on the next render, and ``start_server_process()`` clears the
        latch -- disarming a tripped emergency break as the side effect of an
        operator asking for the engine to be shut *down*.

        The engine calls are recorded rather than delegated, so the test stays
        hermetic and cannot actually launch the native simulator.
        """
        engine_calls = []
        self.client.start_server_process = lambda: (
            engine_calls.append("start_server_process"), True
        )[1]
        self.client.connect = lambda: (engine_calls.append("connect"), True)[1]

        self.assertTrue(self.client.request_emergency_break())
        self.assertTrue(self.client.estop_latched)
        self.assertIs(self.client.control_state, ControlState.ESTOP)

        # The operator clicks "Stop Engine".
        dashboard_app.stop_engine(self.client)
        self.assertFalse(self.client.connected)
        self.assertTrue(
            self.client.estop_latched,
            "stop_server_process() must not clear the latch",
        )
        self.assertIsNot(
            self.client.control_state, ControlState.ESTOP,
            "precondition: disconnect() overwrites the state, which is exactly "
            "why the gate cannot read it",
        )

        # The next automatic render must not bring the engine back.
        self.assertFalse(
            dashboard_app.ensure_transport(self.client),
            "ensure_transport restarted the engine while an E-stop was latched",
        )
        self.assertEqual(
            engine_calls, [],
            f"a latched E-stop must not be cleared by an automatic reconnect; "
            f"got {engine_calls}",
        )
        self.assertTrue(self.client.estop_latched, "the latch must still be set")

    def test_restart_engine_clears_the_latch_for_real(self):
        """
        Invariant: the operator's documented escape hatch works.

        ``restart_engine`` is the one dashboard action allowed to start a new
        engine, and ``start_server_process()`` is what clears the latch.  Only
        the engine start is recorded, so no simulator is launched, but the latch
        clear itself is the real client's own behaviour.
        """
        self.assertTrue(self.client.request_emergency_break())
        self.assertTrue(self.client.estop_latched)

        def _fake_start() -> bool:
            # Mirror the real start_server_process(), which clears the latch.
            self.client._estop_latched = False
            return True

        self.client.start_server_process = _fake_start
        dashboard_app.restart_engine(self.client, settle_s=0.0)
        self.assertFalse(
            self.client.estop_latched,
            "Restart Engine is the documented way out of a latched E-stop",
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
        dashboard_app.forget_tab_baselines(self.client)
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
                # settle() first: the recorder parses on its own thread, so a
                # frame written synchronously a moment ago may not be parsed yet.
                self.recorder.settle()
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
        # The per-tab diff baselines live in dashboard_app keyed by client, and
        # this fake is a process-wide singleton behind st.cache_resource, so they
        # have to be cleared explicitly or one test would inherit another's.
        dashboard_app.forget_tab_baselines(self)
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

        Asserts the metric's value *and* the colour-coded line carrying the
        presentation's own label and colour, so the two cannot drift apart and
        the check cannot be satisfied by the metric's title alone.
        """
        self._run()
        metric = next(
            (m for m in self.at.sidebar.metric if m.label == "Control State"), None
        )
        self.assertIsNotNone(
            metric, "the sidebar must render a 'Control State' metric"
        )
        self.assertEqual(metric.value, self.client.control_state.value)
        label, color, _ = dashboard_app.control_state_presentation(self.client)
        rendered = " ".join(c.value for c in self.at.sidebar.markdown)
        self.assertIn(
            f":{color}[{label}]", rendered,
            "the sidebar must render the colour-coded control-state line for "
            f"{label} in {color}; rendered markdown was {rendered!r}",
        )

    def test_main_does_not_transmit_with_a_live_transport(self):
        """
        Invariant: a render with a live SIL transport transmits nothing.

        This is the end-to-end version of the refresh-loop guarantee.  The
        transport is engaged and the auto-refresh toggle left on, so ``main()``
        reaches its trailing ``sleep; st.rerun()`` block -- the one piece of
        ``main()`` that runs with nobody watching.  Whatever that block does, it
        must not put a command frame on the wire: a 20 Hz neutral resend would
        refresh the firmware heartbeat forever and hold the solenoids energized
        behind a dashboard that reports the vehicle is stopped.

        ``MAX_CONNECTED_READS`` in the fake bounds the rerun loop this provokes.
        """
        self._run()
        self.client.calls.clear()
        self.client.engage(ControlState.RUNNING)
        self._run()
        self.assertEqual(
            self.client.send_calls(), [],
            f"a render with a live transport transmitted {self.client.send_calls()}; "
            f"only an operator action may put a command on the wire",
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

    def test_a_pilot_preset_leaves_the_slider_showing_the_held_command(self):
        """
        The end-to-end version of the stale-display defect, on the real script.

        A preset holds ``compute_thrust_allocation(0.5, 0, 0, 0)`` at 20 Hz
        indefinitely, and Streamlit seeds a keyed widget once. Before the fix the
        rendered slider stayed at 0.0 for as long as that command was held, so the
        tab's own baseline was all zero and the panel read safe while four
        horizontal thrusters were commanded forward.
        """
        self._run()
        self._disable_auto_refresh()
        self.client.engage(ControlState.RUNNING)
        self._run()
        self.client.calls.clear()
        self._button(self.at, "Forward (+0.5 Surge)").click()
        self._run()

        self.assertEqual(
            self.client.send_calls(),
            [("send_surface_pilot_command", (0.5, 0.0, 0.0, 0.0))],
            "the preset must be the only command the click produces",
        )
        self.assertEqual(
            self.client.pwms,
            compute_thrust_allocation(0.5, 0.0, 0.0, 0.0),
            "precondition: the command is held at 20 Hz, not one-shot",
        )
        surge = self.at.slider(key=dashboard_app.PILOT_SLIDER_KEYS[0])
        self.assertEqual(
            surge.value,
            0.5,
            "the panel must show the command being held; it read 0.0 while four "
            "thrusters were held forward",
        )
        for key in dashboard_app.PILOT_SLIDER_KEYS[1:]:
            with self.subTest(key=key):
                self.assertEqual(self.at.slider(key=key).value, 0.0)

    def test_a_thruster_preset_leaves_the_sliders_showing_the_held_command(self):
        self._run()
        self._disable_auto_refresh()
        self.client.engage(ControlState.RUNNING)
        self._run()
        self.client.calls.clear()
        self._button(self.at, "All Forward (1650 us)").click()
        self._run()

        self.assertEqual(self.client.send_calls(), [("send_pwms", ([1650] * 8,))])
        for channel in range(8):
            key = dashboard_app.THRUSTER_SLIDER_KEY(channel)
            with self.subTest(channel=channel):
                self.assertEqual(
                    self.at.slider(key=key).value,
                    1650,
                    f"{key} must show the held command, not the pre-preset position",
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
