"""
Purdue ROV Software-in-the-Loop (SIL) Interactive Testing Dashboard.
Powered by Streamlit.

Enables live hardware-free testing and end-to-end verification strictly within X19-Embedded:
- SIL pilot controls: 6-DOF inputs mapped to simulated CAN commands.
- End-to-End Message Pipeline Tracer:
    Downlink: Dashboard controls -> SIL CAN injection -> native C application logic
    Uplink:   Native C telemetry -> SIL CAN frames -> dashboard packet decoder and gauges
- Raw CAN Bus Monitor & Packet Inspector (Tab 6): Live frame stream, hex inspector, and filtering.
- Live C Firmware Console & Code Verification (Tab 7): Real-time stdout from sil_bridge_server.exe.
- Direct Thruster & Solenoid Control (Tab 2).
- Real-time live gauges: 100 Hz Navigation (Tab 3), Environmental & Leak (Tab 4), Power Slab (Tab 5).
"""

import os
import sys
import time
import weakref

import streamlit as st

dashboard_dir = os.path.dirname(os.path.abspath(__file__))
if dashboard_dir not in sys.path:
    sys.path.insert(0, dashboard_dir)

from sil_dashboard_client import (
    CAN_ID_NAV_TELEMETRY,
    CAN_ID_ENV_TELEMETRY,
    CAN_ID_POWER_TELEMETRY,
    CAN_ID_SIL_OUTPUT_STATUS,
    ControlState,
    SilDashboardClient,
    stream_age_label,
)

# --- Control-surface handlers ------------------------------------------------
# Every action that can put a command on the wire, or decide that the command
# path is live, lives in a module-level function rather than inline in main().
# main() is Streamlit code: it cannot be imported and called without a running
# Streamlit server, so inlining these would make the safety wiring untestable.
# tests/sil_stimulus/test_dashboard_app.py drives them directly.
#
# None of these helpers hold a Streamlit reference and none of them may transmit
# on their own except through the client: the 20 Hz control worker owns the
# command cadence, and the UI only decides *what* is held.

# Colour, label, and operator-facing explanation for each deadman state.
# RUNNING and ESTOP are visually separated from the two safe idle states
# (ARMED, STOPPED) because they are the states in which the vehicle can move.
_CONTROL_STATE_PRESENTATION = {
    ControlState.RUNNING: (
        "RUNNING",
        "green",
        "20 Hz control worker is transmitting. Node 2's heartbeat is being "
        "refreshed, so the board is out of its failsafe.",
    ),
    ControlState.ARMED: (
        "ARMED",
        "blue",
        "Transport up, no command held. Outputs are neutral and the heartbeat "
        "is deliberately not refreshed, so the firmware watchdog can lapse and "
        "release the solenoids.",
    ),
    ControlState.STOPPED: (
        "STOPPED",
        "orange",
        "Explicit operator stop. Outputs are neutral and no command is being "
        "transmitted.",
    ),
    ControlState.ESTOP: (
        "ESTOP",
        "red",
        "Emergency break latched. Every command is refused until the SIL engine "
        "is explicitly restarted.",
    ),
    ControlState.DISCONNECTED: (
        "DISCONNECTED",
        "gray",
        "No SIL transport. Nothing is transmitted and the control path is dead.",
    ),
}

_UNKNOWN_STATE_PRESENTATION = (
    "UNKNOWN",
    "red",
    "The client reported a control state this dashboard does not recognise. "
    "Treat the control path as unsafe.",
)


def all_stop(client) -> bool:
    """
    Operator STOP: cancel the held command and drive the outputs neutral.

    This must be ``request_stop()`` and not a neutral PWM command.  A neutral
    command goes to ARMED and stops the worker, which is safe, but it leaves the
    operator in the same state as merely releasing the stick.  ``request_stop()``
    is also the transition that keeps the dashboard's slider diffs quiet: it
    leaves the client's record of the operator's commanded values alone, so the
    next render finds nothing to re-send.
    """
    return bool(client.request_stop())


def stop_engine(client) -> None:
    """
    Operator "Stop Engine": shut the SIL engine down.

    Intentionally does *not* clear a latched E-stop -- see :func:`ensure_transport`.
    Shutting the engine down is the operator asking for less, never for the
    emergency break to be disarmed.
    """
    client.stop_server_process()


def restart_engine(client, settle_s: float = 0.3) -> bool:
    """
    Operator "Restart Engine": the explicit action that clears a latched E-stop.

    ``start_server_process()`` is one of exactly two ways out of a latched ESTOP
    (the other is a fresh transport via ``connect()``), and the dashboard offers
    this as the only one.  ``settle_s`` is the pause that lets the old engine
    release the port; it is a parameter so tests need not wait for it.
    """
    client.stop_server_process()
    time.sleep(settle_s)
    started = client.start_server_process()
    connected = client.connect()
    return bool(started and connected)


def trip_emergency_break(client) -> bool:
    """
    Trip the authorized emergency break and latch ESTOP.

    Uses the canonical ``request_emergency_break()``.  ``trigger_emergency_break``
    is a zero-argument alias for the same method in sil_dashboard_client, so
    either name sends the identical 0xAA 0x55 0x01 frame and latches the same
    way; the canonical name is preferred because the alias exists only for
    backwards compatibility.
    """
    return bool(client.request_emergency_break())


def auto_refresh_due(client, auto_refresh: bool, refresh_rate: float) -> bool:
    """
    True when the render loop should sleep and re-run.

    Deliberately transmits nothing.  The 20 Hz control worker owns the command
    cadence; a second sender here would emit an extra command frame at whatever
    rate the operator's browser happened to poll.
    """
    return bool(auto_refresh) and bool(client.connected)


# --- Per-tab diff baselines -------------------------------------------------
# Streamlit renders every tab on every run, and each control tab diffs its
# slider widgets against a baseline to decide "did the operator move something?".
#
# The baseline must be owned by the tab that published it.  A shared client field
# cannot do that job: ``send_surface_pilot_command()`` ends in ``send_pwms()``,
# which writes the shared ``client.pwms``, so the thruster tab's diff would read
# the pilot's allocation as a slider move and re-send the thruster tab's stale
# target -- with no operator action on that tab, at 20 Hz, vertical bank
# included.  Symmetrically, a thruster command must not make the pilot tab
# re-send.  One baseline per tab closes both directions.
#
# One baseline per *client* is not enough on its own: ``st.cache_resource`` hands
# every browser session the same client, so a second session inherits the first
# session's baseline while its own sliders are seeded from the client's current
# command.  Comparing those two values re-arms thrust on the new session's very
# first render.  :func:`publish_tab_baseline` therefore seeds a tab's first
# baseline from :func:`tab_seed_value` -- the same source its widgets seed from --
# and the predicates additionally treat "the sliders already show the command" as
# nothing to send.  The baseline is deliberately NOT keyed on the browser session:
# ``st.session_state`` is not reliably readable outside a script run, and these
# predicates must stay callable (and testable) without a Streamlit runtime.
#
# Keyed weakly by client so a replaced client cannot leak baselines, and held in
# the UI rather than the client because these are dashboard-owned records of
# what the operator last dialled in, not protocol state.

_TAB_BASELINES = weakref.WeakKeyDictionary()

#: Stable identifiers for the two slider-diff baselines.
TAB_PILOT = "pilot"
TAB_THRUSTER = "thruster"


def tab_baseline(client, tab: str):
    """
    Return the last value *this tab* published, or None if it never has.
    """
    return _TAB_BASELINES.get(client, {}).get(tab)


def tab_seed_value(client, tab: str):
    """
    The value a tab's sliders are seeded from on a render with no widget state.

    Must stay identical to the ``value=`` / ``float(...)`` source of the matching
    ``st.slider`` in main(), because that equality is what makes a fresh session's
    first render a no-op.  Pilot sliders read ``client.pipeline_surface_cmd``;
    thruster sliders read ``client.pwms``.
    """
    if tab == TAB_PILOT:
        command = client.pipeline_surface_cmd
        return (command["surge"], command["sway"], command["heave"], command["yaw"])
    if tab == TAB_THRUSTER:
        return tuple(int(value) for value in client.pwms)
    raise ValueError(f"unknown control tab: {tab!r}")


def publish_tab_baseline(client, tab: str, values) -> None:
    """
    Record what a tab is now showing, so its next diff finds no change.

    Called once per tab per render, after the send decision: it records the
    transmitted value when a send happened and the unchanged widget value when it
    did not, which are the same thing in both cases.

    The FIRST publish for a client adopts :func:`tab_seed_value` instead of the
    widget value -- the same source the sliders were seeded from -- so a fresh
    session's baseline and its seeded slider values agree by construction.  Without
    that, a second session sharing the cached client would hold the *previous*
    session's widget values as its baseline while its own sliders were seeded from
    the client's current command, see a difference that no operator caused, and
    re-arm thrust on its first render.
    """
    stored = _TAB_BASELINES.get(client, {}).get(tab)
    _TAB_BASELINES.setdefault(client, {})[tab] = (
        tuple(values) if stored is not None else tab_seed_value(client, tab)
    )


def forget_tab_baselines(client) -> None:
    """Drop a client's per-tab baselines, e.g. when a session's controls reset."""
    _TAB_BASELINES.pop(client, None)


def pilot_axes_changed(client, surge: float, sway: float, heave: float, yaw: float) -> bool:
    """
    True when the 6-DOF sliders differ from the last axes *this tab* published
    *and* differ from the command the client is already holding.

    Both conditions are needed.  The first is the operator's intent, and reading
    only its own baseline is what keeps the two tabs from fighting.  The second
    covers a session whose baseline predates the current command: a fresh session's
    sliders are seeded from the client's command, so "the sliders already show the
    command" means there is nothing to send, however stale the baseline is.

    Read-only.  It never reads a field the thruster tab writes.
    """
    axes = (surge, sway, heave, yaw)
    baseline = tab_baseline(client, TAB_PILOT)
    if baseline is None:
        return False
    if axes == tuple(baseline):
        return False
    return axes != tab_seed_value(client, TAB_PILOT)


def thruster_targets_changed(client, pwms) -> bool:
    """
    True when the 8 per-channel sliders differ from the last targets *this tab*
    published *and* differ from the command the client is already holding.

    Deliberately does not treat "differs from ``client.pwms``" as the whole test:
    every sender writes that field, including the pilot tab, and
    ``request_stop()`` deliberately leaves it stale.  Comparing a fresh session's
    seeded sliders against a previous session's baseline -- or against a stale
    ``pwms`` -- is exactly the cross-session re-arm this replaces.
    """
    targets = tuple(pwms)
    baseline = tab_baseline(client, TAB_THRUSTER)
    if baseline is None:
        return False
    if targets == tuple(baseline):
        return False
    return targets != tab_seed_value(client, TAB_THRUSTER)



def control_state_presentation(client):
    """
    Return ``(label, color, help_text)`` for the client's current control state.

    The operator has to be able to tell at a glance whether the control path is
    transmitting, because ARMED and STOPPED are both safe but mean different
    things and neither is what RUNNING or ESTOP is.
    """
    return _CONTROL_STATE_PRESENTATION.get(
        client.control_state, _UNKNOWN_STATE_PRESENTATION
    )


def ensure_transport(client) -> bool:
    """
    Bring up the SIL engine and connect, unless an E-stop is latched.

    Gates on ``client.estop_latched`` and NOT on ``client.control_state``:
    ``disconnect()`` overwrites the state with ``DISCONNECTED`` while leaving the
    latch set, so a state check lets "Stop Engine" -- whose whole intent is to
    shut the engine *down* -- clear a tripped emergency break on the very next
    render, because ``start_server_process()`` and ``connect()`` both clear the
    latch.

    Clearing it therefore requires the explicit "Restart Engine" button, which
    calls :func:`restart_engine`.
    """
    if client.connected:
        return True
    if client.estop_latched:
        return False
    client.start_server_process()
    return bool(client.connect())

# Global client cache
@st.cache_resource
def get_sil_client() -> SilDashboardClient:
    return SilDashboardClient()

def main():
    st.set_page_config(
        page_title="Purdue ROV - Embedded SIL Testing Station",
        layout="wide",
        initial_sidebar_state="expanded",
    )

    client = get_sil_client()

    # --- SIDEBAR: Master Controls & System State ---
    with st.sidebar:
        st.title("ROV Embedded SIL Master Hub")
        st.markdown("**Subsea Node Firmware Simulation**")

        st.subheader("SIL Server State")
        # A latched E-stop deliberately blocks the automatic reconnect.
        if not ensure_transport(client):
            if client.estop_latched:
                st.error(
                    "Emergency break latched on the dashboard. Automatic reconnect "
                    "is disabled so neither a transport hiccup nor Stop Engine can "
                    "clear the latch. Click **Restart Engine** to start a new SIL "
                    "engine and re-arm the control path."
                )
            else:
                st.error(
                    "SIL engine unreachable on 127.0.0.1:8765. Use **Restart "
                    "Engine** in this panel to start it."
                )

        col_srv1, col_srv2 = st.columns(2)
        with col_srv1:
            if st.button("Restart Engine", width="stretch"):
                if restart_engine(client):
                    st.rerun()
                else:
                    # No rerun: a Streamlit error does not survive one, and the
                    # operator needs to read why the engine did not come back.
                    st.error(
                        "Restart Engine failed: the SIL engine did not start, or "
                        "did not accept a connection on 127.0.0.1:8765. Check that "
                        "sil_bridge_server is built and runnable, then try again."
                    )
        with col_srv2:
            if st.button("Stop Engine", width="stretch"):
                stop_engine(client)
                st.rerun()

        status_color = "green" if client.connected else "red"
        st.markdown(f"**Connection Status:** :{status_color}[{'ONLINE (127.0.0.1:8765)' if client.connected else 'OFFLINE'}]")

        # Deadman readout. The two safe idle states (ARMED, STOPPED) are shown
        # differently from each other and from the two states in which the
        # vehicle can move (RUNNING, ESTOP), so the operator never has to guess
        # whether the control path is live.
        state_label, state_color, state_help = control_state_presentation(client)
        st.metric("Control State", client.control_state.value)
        st.markdown(f"**Control State:** :{state_color}[{state_label}]")
        st.caption(state_help)

        st.metric("SIL Virtual Time", f"{client.sim_time_ms / 1000.0:.2f} s")
        frame_cols = st.columns(2)
        frame_cols[0].metric("Received Frames", client.frame_count)
        frame_cols[1].metric("Buffered Records", len(client.packet_log))
        st.caption(f"Mock output snapshot: {stream_age_label(client, CAN_ID_SIL_OUTPUT_STATUS)}")
        st.caption(
            "Stream age — "
            f"NAV {stream_age_label(client, CAN_ID_NAV_TELEMETRY)} · "
            f"ENV {stream_age_label(client, CAN_ID_ENV_TELEMETRY)} · "
            f"PWR {stream_age_label(client, CAN_ID_POWER_TELEMETRY)}"
        )

        st.divider()
        st.subheader("Live Telemetry Streaming")
        auto_refresh = st.toggle("Auto-Refresh Telemetry", value=True, help="Periodically re-renders live gauges.")
        refresh_rate = 0.5
        if auto_refresh:
            refresh_rate = st.select_slider("Refresh Interval", options=[0.2, 0.5, 1.0, 2.0], value=0.5, format_func=lambda x: f"{x}s")

        st.divider()
        st.subheader("Fault & Safety Injection")
        if st.button("TRIP EMERGENCY BREAK (0x001)", type="primary", width="stretch", disabled=not client.connected):
            trip_emergency_break(client)
            st.warning("Emergency-break command sent. Waiting for the simulated board readback to confirm the latch.")

        if client.emergency_break_tripped:
            st.error("SIL board readback: emergency brake latched")
            st.caption("Restart the SIL engine to clear its latch. This display cannot reset target hardware.")
        elif client.emergency_break_requested:
            st.warning("Emergency break sent; waiting for the board output snapshot.")

        st.divider()
        st.markdown("### Subsea Nodes")
        st.markdown("- **Node 1**: Pi Shield (STM32C5)")
        st.markdown("- **Node 2**: Control Board (STM32C5 + FPU)")
        st.markdown("- **Node 3**: Power Slab (STM32C5)")
        st.markdown("- **Node 4**: USB Camera Hub (PCIe)")

    # --- MAIN DASHBOARD INTERFACE ---
    st.title("Purdue ROV — Embedded SIL Testing Station")
    st.caption("Native C application logic with mocked peripherals. Readbacks are simulated outputs, not hardware measurements.")

    tab1, tab2, tab3, tab4, tab5, tab6, tab7 = st.tabs([
        "Surface Pilot & Pipeline Tracer",
        "Direct Thrusters & Solenoids",
        "Navigation & Attitude (100 Hz)",
        "Environmental & Leak (10 Hz)",
        "Power Slab Telemetry (20 Hz)",
        "Raw CAN Bus Monitor & Packet Inspector",
        "Live C Firmware Console & Code Verification"
    ])

    # =========================================================================
    # TAB 1: SURFACE PILOT STATION & END-TO-END PIPELINE TRACER
    # =========================================================================
    with tab1:
        st.subheader("SIL Pilot Controls & Native C Pipeline")
        st.markdown(
            "Inspect the dashboard's local command adapter, simulated CAN traffic, native node application responses, "
            "and mock BSP output readback. Core and Surface processes are covered by the separate bridge test."
        )

        with st.expander("System Architecture Overview (Click to expand)", expanded=False):
            st.markdown(
                """
```
DOWNLINK (dashboard controls to native application):
  [Dashboard axes] -> [Local PWM mapper] -> [simulated CAN 0x100] -> [Native Node 2 app]
                                                              -> [Mock BSP output snapshot]

UPLINK (mock sensors to dashboard views):
  [Mock IMU/depth] -> [Native Node 2] -> simulated CAN 0x200 -> [Dashboard]
  [Mock environment] -> [Native Node 1] -> simulated CAN 0x210 -> [Dashboard]
  [Mock power] -> [Native Node 3] -> simulated CAN 0x300 -> [Dashboard]
```
                """
            )

        st.markdown("### 1. Topside Pilot Flight Deck")
        col_ctrl1, col_ctrl2 = st.columns([1, 1])

        with col_ctrl1:
            st.markdown("**6-DOF Flight Axes** (Drag to pilot vehicle)")
            surge = st.slider("Surge (Forward / Reverse)", -1.0, 1.0, float(client.pipeline_surface_cmd["surge"]), 0.05, key="slider_surge")
            sway = st.slider("Sway (Strafe Right / Left)", -1.0, 1.0, float(client.pipeline_surface_cmd["sway"]), 0.05, key="slider_sway")
            heave = st.slider("Heave (Dive / Ascend)", -1.0, 1.0, float(client.pipeline_surface_cmd["heave"]), 0.05, key="slider_heave")
            yaw = st.slider("Yaw (Turn Right / Left)", -1.0, 1.0, float(client.pipeline_surface_cmd["yaw"]), 0.05, key="slider_yaw")

        with col_ctrl2:
            st.markdown("**Flight Presets**")
            preset_cols = st.columns(3)
            with preset_cols[0]:
                if st.button("All Stop (Hover)", width="stretch"):
                    all_stop(client)
                    st.rerun()
            with preset_cols[1]:
                if st.button("Forward (+0.5 Surge)", width="stretch"):
                    client.send_surface_pilot_command(0.5, 0.0, 0.0, 0.0)
                    st.rerun()
            with preset_cols[2]:
                if st.button("Reverse (-0.5 Surge)", width="stretch"):
                    client.send_surface_pilot_command(-0.5, 0.0, 0.0, 0.0)
                    st.rerun()

            preset_cols2 = st.columns(3)
            with preset_cols2[0]:
                if st.button("Strafe Right (+0.5 Sway)", width="stretch"):
                    client.send_surface_pilot_command(0.0, 0.5, 0.0, 0.0)
                    st.rerun()
            with preset_cols2[1]:
                if st.button("Dive (+0.5 Heave)", width="stretch"):
                    client.send_surface_pilot_command(0.0, 0.0, 0.5, 0.0)
                    st.rerun()
            with preset_cols2[2]:
                if st.button("Yaw Right (+0.5 Yaw)", width="stretch"):
                    client.send_surface_pilot_command(0.0, 0.0, 0.0, 0.5)
                    st.rerun()

            # Diff against the pilot tab's OWN baseline, then record what the tab
            # is showing.  The thruster tab's sliders keep their own baseline, so
            # a command published by either tab cannot be read as a slider move
            # on the other.
            pilot_axes = (surge, sway, heave, yaw)
            if pilot_axes_changed(client, *pilot_axes):
                client.send_surface_pilot_command(*pilot_axes)
            publish_tab_baseline(client, TAB_PILOT, pilot_axes)

        st.divider()
        st.markdown("### 2. Live End-to-End Message Flow Inspector")

        # ---------------------------------------------------------------------
        # PART A: DOWNLINK COMMAND PATH
        # ---------------------------------------------------------------------
        st.markdown("#### Part A: Downlink Command Path (Dashboard -> Native Node 2)")
        st.caption("The local dashboard adapter creates the CAN command; this view does not drive physical motors.")

        col_stg1, col_stg2, col_stg3 = st.columns(3)

        # STAGE 1
        with col_stg1:
            st.markdown("##### [Stage 1] Dashboard axes -> SIL adapter")
            st.caption("Dashboard axis controls (local SIL adapter; bypasses the Core process)")
            st.info(
                "**Purpose of Stage 1 (`JoystickCommand`):**\n"
                "The dashboard serializes pilot axes using the shared Protobuf schema for inspection, then its local "
                "SIL adapter converts those axes to simulated CAN commands. The Core ZMQ process is not in this path."
            )

            ts_us = client.pipeline_surface_cmd.get("timestamp_us", 0)
            st.dataframe(
                [
                    {"Field": "forward (Surge)", "Value": f"{client.pipeline_surface_cmd['surge']:+.2f}", "Description": "Forward/Reverse throttle (-1 to +1)"},
                    {"Field": "strafe (Sway)", "Value": f"{client.pipeline_surface_cmd['sway']:+.2f}", "Description": "Lateral strafe Right/Left"},
                    {"Field": "vertical (Heave)", "Value": f"{client.pipeline_surface_cmd['heave']:+.2f}", "Description": "Vertical Dive/Ascend"},
                    {"Field": "yaw", "Value": f"{client.pipeline_surface_cmd['yaw']:+.2f}", "Description": "Heading rotation Right/Left"},
                    {"Field": "timestamp_us", "Value": f"{ts_us}", "Description": "Microsecond timestamp for latency tracking"},
                ],
                width="stretch",
                hide_index=True
            )
            st.markdown("**Protobuf Wire Serialization (Hex):**")
            st.code(client.pipeline_surface_cmd.get("raw_hex", "08 00 15 00 00 00 00"), language="text")

        # STAGE 2
        with col_stg2:
            st.markdown("##### [Stage 2] SIL mapper -> simulated CAN")
            st.caption("CAN FD Arbitration ID `0x100` (`THRUSTER_CMD`, DLC: 16 Bytes)")
            st.info(
                "**SIL adapter:** this dashboard maps pilot axes to PWM targets and injects CAN ID 0x100 into the "
                "Embedded simulator. It does not run the X19-Core application or a physical Pi."
            )

            pwms = client.pipeline_core_pwms
            thruster_meta = [
                ("T0", "Front-Left Horiz (45°)", "Surge + Sway + Yaw", pwms[0]),
                ("T1", "Front-Right Horiz (45°)", "Surge - Sway - Yaw", pwms[1]),
                ("T2", "Aft-Left Horiz (45°)", "Surge - Sway + Yaw", pwms[2]),
                ("T3", "Aft-Right Horiz (45°)", "Surge + Sway - Yaw", pwms[3]),
                ("T4", "Front-Left Vert", "-Heave + Pitch - Roll", pwms[4]),
                ("T5", "Front-Right Vert", "-Heave + Pitch + Roll", pwms[5]),
                ("T6", "Aft-Left Vert", "-Heave - Pitch - Roll", pwms[6]),
                ("T7", "Aft-Right Vert", "-Heave - Pitch + Roll", pwms[7]),
            ]
            matrix_table = []
            for tid, name, formula, pwm in thruster_meta:
                effort = pwm - 1500
                effort_str = f"{effort:+d} us ({'Fwd' if effort > 0 else 'Rev' if effort < 0 else 'Stop'})"
                hex_le = f"{(pwm & 0xFF):02X} {((pwm >> 8) & 0xFF):02X}"
                matrix_table.append({
                    "Thruster": tid,
                    "Position": name,
                    "Target PWM": f"{pwm} us",
                    "Effort": effort_str,
                    "LE Hex": hex_le,
                })
            st.dataframe(matrix_table, width="stretch", hide_index=True)

            st.markdown("**CAN FD ID 0x100 Payload (8x uint16_t Little-Endian):**")
            st.code(client.pipeline_core_can_hex or "DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05", language="text")

        # STAGE 3
        with col_stg3:
            st.markdown("##### [Stage 3] Native Node 2 C application")
            st.caption("Control Board Node 2 (`Src/app.c`) Native C Execution")
            st.info(
                "**Native Control Board application:** receives CAN ID 0x100, applies application-level checks, and "
                "updates mocked PWM state. The host SIL does not emulate STM32 timers, BDTR hardware, or rail behavior."
            )

            e_status = "SIL brake latch observed" if client.emergency_break_tripped else "No brake latch observed"
            st.dataframe(
                [
                    {"Parameter": "Safety State", "Status": e_status, "Detail": "Emergency Break / Leak Interlock"},
                    {"Parameter": "PWM target", "Status": f"T0={pwms[0]} us, T1={pwms[1]} us", "Detail": "Dashboard-generated command"},
                    {"Parameter": "Mock PWM readback", "Status": f"T0={client.actual_pwms[0]} us, T1={client.actual_pwms[1]} us", "Detail": "Latest SIL output snapshot"},
                    {"Parameter": "Output snapshot", "Status": stream_age_label(client, CAN_ID_SIL_OUTPUT_STATUS), "Detail": f"Virtual time {client.sim_time_ms} ms"},
                    {"Parameter": "Build type", "Status": "Native host SIL", "Detail": "No STM32 HAL or physical I/O"},
                ],
                width="stretch",
                hide_index=True
            )
            st.markdown("**Compiled Machine Code:**")
            st.code("Target: build/tests/sil_bridge_server.exe\nModule: nodes/node2_control_board/Core/Src/app.c", language="text")

        st.divider()

        # ---------------------------------------------------------------------
        # PART B: UPLINK TELEMETRY PATH
        # ---------------------------------------------------------------------
        st.markdown("#### Part B: Uplink Telemetry Path (Mock Sensors -> SIL Views)")
        st.caption("Values come from injected mock sensors and native C telemetry; they are not hardware measurements.")

        col_stg4, col_stg5 = st.columns(2)

        # STAGE 4
        with col_stg4:
            st.markdown("##### [Stage 4] Native nodes -> simulated CAN telemetry")
            st.caption("CAN Arbitration IDs `0x200` (Nav @ 100 Hz), `0x210` (Env @ 10 Hz), `0x300` (Power @ 20 Hz)")
            st.info(
                "**Purpose of Stage 4 (Subsea Sensor Acquisition):**\n"
                "Native C node applications read injected mock sensor values and publish telemetry on the simulated CAN bus. Node 2 streams 100 Hz Navigation (BMI270 IMU + MS5837 Depth), Node 1 streams 10 Hz "
                "Enclosure health (BME280 pressure/temp/humidity and leak probes), and Node 3 streams 20 Hz Power distribution."
            )

            nav = client.nav_data
            env = client.env_data
            pwr = client.power_data

            sensor_rows = [
                {"Node & Board": "Node 2: Control Board", "CAN ID": "0x200 (Nav)", "Sensor": "MS5837 Depth", "Live Value": f"{nav.depth_meters:.2f} m" if nav else "No sample"},
                {"Node & Board": "Node 2: Control Board", "CAN ID": "0x200 (Nav)", "Sensor": "IMU Gyro Z", "Live Value": f"{nav.gyro_z_rad_s:.3f} rad/s" if nav else "No sample"},
                {"Node & Board": "Node 2: Control Board", "CAN ID": "0x200 (Nav)", "Sensor": "IMU Quaternion", "Live Value": f"({nav.q_w:.2f}, {nav.q_x:.2f}, {nav.q_y:.2f}, {nav.q_z:.2f})" if nav else "No sample"},
                {"Node & Board": "Node 1: Pi Shield", "CAN ID": "0x210 (Env)", "Sensor": "BME280 Pressure", "Live Value": f"{env.pressure_hpa:.1f} hPa" if env else "No sample"},
                {"Node & Board": "Node 1: Pi Shield", "CAN ID": "0x210 (Env)", "Sensor": "Leak Probes", "Live Value": f"0x{env.leak_flags:02X} ({'INGRESS' if env and env.leak_flags else 'DRY/OK'})" if env else "No sample"},
                {"Node & Board": "Node 3: Power Slab", "CAN ID": "0x300 (Pwr)", "Sensor": "Tether Rail", "Live Value": f"{pwr.tether_voltage_mv/1000:.1f} V @ {pwr.tether_current_ma/1000:.1f} A" if pwr else "No sample"},
            ]
            st.dataframe(sensor_rows, width="stretch", hide_index=True)

        # STAGE 5
        with col_stg5:
            st.markdown("##### [Stage 5] Surface Protobuf translation preview")
            st.caption("Core/Surface integration contract: ZeroMQ topic `telemetry` on port 5555 (not exercised by this direct-CAN dashboard path).")
            st.info(
                "**What is this supposed to show me?**\n"
                "The Core/Surface repositories define a `SensorData` Protobuf over the `telemetry` topic. This dashboard "
                "shows the SIL-side translation preview; its direct CAN connection does not route packets through the Core "
                "or Surface processes. The separate bridge integration test exercises that cross-repository path."
            )

            p_srf = client.pipeline_core_to_surface
            st.dataframe(
                [
                    {"Field": "depth", "Decoded Value": f"{p_srf['depth']:.2f} meters", "Role on Pilot Heads-Up Display (HUD)": "Vertical depth tape & auto-depth PID target"},
                    {"Field": "temperature", "Decoded Value": f"{p_srf['temp']:.1f} °C", "Role on Pilot Heads-Up Display (HUD)": "Hull overheating alarm indicator"},
                    {"Field": "angular_velocity", "Decoded Value": f"X:{p_srf['gyro_x']:.2f}, Y:{p_srf['gyro_y']:.2f}, Z:{p_srf['gyro_z']:.2f} rad/s", "Role on Pilot Heads-Up Display (HUD)": "Rate-of-turn indicator & gyro stabilization"},
                    {"Field": "acceleration", "Decoded Value": f"X:{p_srf['accel_x']:.2f}, Y:{p_srf['accel_y']:.2f}, Z:{p_srf['accel_z']:.2f} m/s²", "Role on Pilot Heads-Up Display (HUD)": "Fixed SIL placeholder; not measured IMU data"},
                    {"Field": "timestamp_us", "Decoded Value": f"{p_srf['timestamp_us']}", "Role on Pilot Heads-Up Display (HUD)": "Tether latency & packet freshness heartbeat"},
                ],
                width="stretch",
                hide_index=True
            )
            st.markdown("**Protobuf Wire Serialization (Hex):**")
            st.code(p_srf.get("raw_hex", "08 00 1D 00 00 00 00 25 00 00 B4 41"), language="text")

    # =========================================================================
    # TAB 2: DIRECT THRUSTERS & SOLENOIDS
    # =========================================================================
    with tab2:
        st.subheader("8-Channel Thruster ESC Command & PWM Monitor")
        st.caption("Controls send target PWM over simulated CAN; actual values below come from the mock BSP output snapshot.")
        st.markdown("#### Native Control Board output readback")
        st.caption(f"{stream_age_label(client, CAN_ID_SIL_OUTPUT_STATUS)} · virtual time {client.sim_time_ms} ms")
        output_cols = st.columns(8)
        for channel, column in enumerate(output_cols):
            column.metric(
                f"T{channel} actual",
                f"{client.actual_pwms[channel]} µs",
                delta=f"{client.actual_pwms[channel] - client.pwms[channel]:+d} µs vs target",
            )

        col_all1, col_all2, col_all3 = st.columns([1, 1, 2])
        with col_all1:
            if st.button("All Stop (1500 us)", width="stretch", key="btn_all_stop_tab2"):
                all_stop(client)
                st.rerun()
        with col_all2:
            if st.button("All Forward (1650 us)", width="stretch", key="btn_all_fwd_tab2"):
                client.send_pwms([1650] * 8)
                st.rerun()
        with col_all3:
            global_slider = st.slider("Master Sync Throttle", 1000, 2000, 1500, step=10, key="sync_throttle_tab2")
            if st.button("Apply Sync Throttle", key="btn_apply_sync_tab2"):
                client.send_pwms([global_slider] * 8)
                st.rerun()

        pwm_cols = st.columns(4)
        new_pwms = list(client.pwms)
        thruster_names = [
            "T0: Front-Left Horiz", "T1: Front-Right Horiz",
            "T2: Aft-Left Horiz",   "T3: Aft-Right Horiz",
            "T4: Front-Left Vert",  "T5: Front-Right Vert",
            "T6: Aft-Left Vert",    "T7: Aft-Right Vert"
        ]

        for i in range(8):
            col = pwm_cols[i % 4]
            with col:
                val = st.slider(
                    f"{thruster_names[i]}",
                    min_value=1000,
                    max_value=2000,
                    value=int(client.pwms[i]),
                    step=5,
                    key=f"thruster_slider_tab2_{i}",
                )
                new_pwms[i] = val
                delta = val - 1500
                st.progress((val - 1000) / 1000.0)
                st.caption(f"Effort: {delta:+d} us ({'Forward' if delta > 0 else 'Reverse' if delta < 0 else 'Neutral'})")

        if thruster_targets_changed(client, new_pwms):
            client.send_pwms(new_pwms)
        publish_tab_baseline(client, TAB_THRUSTER, new_pwms)

        st.divider()
        st.subheader("10-Channel Pneumatic Solenoid Drivers (AO3400A)")
        st.caption("5 Double-Acting SMC SY3400-6U1-NA Valves switched via CAN ID 0x110.")

        sol_cols = st.columns(5)
        new_mask = 0
        for valve in range(5):
            with sol_cols[valve]:
                st.markdown(f"**Valve {valve + 1}**")
                chA = st.toggle(f"V{valve+1} Extend (Ch {valve*2})", value=bool(client.solenoid_mask & (1 << (valve*2))), key=f"sol_ext_{valve}")
                chB = st.toggle(f"V{valve+1} Retract (Ch {valve*2+1})", value=bool(client.solenoid_mask & (1 << (valve*2+1))), key=f"sol_ret_{valve}")
                if chA:
                    new_mask |= (1 << (valve * 2))
                if chB:
                    new_mask |= (1 << (valve * 2 + 1))

        if new_mask != client.solenoid_mask:
            client.send_solenoids(new_mask)
        st.caption(
            f"Command target 0x{client.solenoid_mask:03X} · mock readback 0x{client.actual_solenoid_mask:03X} · "
            f"{stream_age_label(client, CAN_ID_SIL_OUTPUT_STATUS)}"
        )

    # =========================================================================
    # TAB 3: NAVIGATION & ATTITUDE (100 Hz)
    # =========================================================================
    with tab3:
        st.subheader("100 Hz Navigation Telemetry Stream (CAN ID 0x200)")
        st.caption(f"Stream freshness: {stream_age_label(client, CAN_ID_NAV_TELEMETRY)}")
        nav = client.nav_data
        if nav:
            col_nav1, col_nav2, col_nav3, col_nav4 = st.columns(4)
            col_nav1.metric("Depth (Hydrostatic)", f"{nav.depth_meters:.2f} m")
            col_nav2.metric("Angular Rate (Yaw)", f"{nav.gyro_z_rad_s:.3f} rad/s")
            col_nav3.metric("IMU Status", f"State {nav.imu_status} (High Precision)" if nav.imu_status == 3 else f"State {nav.imu_status}")
            col_nav4.metric("Attitude Norm", f"{(nav.q_w**2 + nav.q_x**2 + nav.q_y**2 + nav.q_z**2)**0.5:.3f}")

            st.markdown("#### Orientation Quaternions & Gyro Rates")
            q_cols = st.columns(4)
            q_cols[0].metric("Q_w", f"{nav.q_w:.4f}")
            q_cols[1].metric("Q_x", f"{nav.q_x:.4f}")
            q_cols[2].metric("Q_y", f"{nav.q_y:.4f}")
            q_cols[3].metric("Q_z", f"{nav.q_z:.4f}")

            gyro_cols = st.columns(3)
            gyro_cols[0].metric("Gyro X (Roll Rate)", f"{nav.gyro_x_rad_s:.4f} rad/s")
            gyro_cols[1].metric("Gyro Y (Pitch Rate)", f"{nav.gyro_y_rad_s:.4f} rad/s")
            gyro_cols[2].metric("Gyro Z (Yaw Rate)", f"{nav.gyro_z_rad_s:.4f} rad/s")

            if len(client.history_depth) > 1:
                st.line_chart(client.history_depth)
        else:
            st.info("Waiting for Navigation Telemetry stream from Control Board (Node 2)... Ensure the SIL Engine is Online.")

    # =========================================================================
    # TAB 4: ENVIRONMENTAL & LEAK (10 Hz)
    # =========================================================================
    with tab4:
        st.subheader("10 Hz Enclosure Environmental & Leak Telemetry (CAN ID 0x210)")
        st.caption(f"Stream freshness: {stream_age_label(client, CAN_ID_ENV_TELEMETRY)}")
        env = client.env_data
        if env:
            col_env1, col_env2, col_env3 = st.columns(3)
            col_env1.metric("Internal Pressure (BME280)", f"{env.pressure_hpa:.1f} hPa")
            col_env2.metric("Relative Humidity", f"{env.humidity_pct:.1f} %")
            col_env3.metric("Enclosure Temperature", f"{env.temperature_c:.1f} °C")

            st.markdown("#### Vacuum Leak Decay & Probe Contact Sensors")
            leak_active = (env.leak_flags != 0)
            if leak_active:
                st.error(f"LEAK DETECTED ON NODE 1 (Flags: 0x{env.leak_flags:02X})")
            else:
                st.success("Sealed Enclosure Normal: No Ingress Detected (Floor Probes Dry, Humidity < 80%)")
        else:
            st.info("Waiting for Environmental Telemetry from Pi Shield (Node 1)...")

    # =========================================================================
    # TAB 5: POWER SLAB TELEMETRY (20 Hz)
    # =========================================================================
    with tab5:
        st.subheader("20 Hz Power Distribution & PMBus Telemetry (CAN ID 0x300)")
        st.caption(f"Stream freshness: {stream_age_label(client, CAN_ID_POWER_TELEMETRY)}")
        pwr = client.power_data
        if pwr:
            col_pwr1, col_pwr2, col_pwr3, col_pwr4, col_pwr5, col_pwr6 = st.columns(6)
            col_pwr1.metric("48V Tether Voltage", f"{pwr.tether_voltage_mv / 1000.0:.2f} V")
            col_pwr2.metric("48V Tether Current", f"{pwr.tether_current_ma / 1000.0:.2f} A")
            col_pwr3.metric("5.2V Logic Rail", f"{pwr.v5_voltage_mv / 1000.0:.2f} V")
            col_pwr4.metric("5.2V Load", f"{pwr.v5_current_ma / 1000.0:.2f} A")
            col_pwr5.metric("PCB Copper Temp", f"{pwr.pcb_temp_c_tenths / 10.0:.1f} °C")
            col_pwr6.metric("Power Status Flags", f"0x{pwr.status_flags:04X}")

            st.markdown("#### 4x 12V 300W Converter Bricks (VCB4812EBO-300WFR3-N)")
            brick_cols = st.columns(4)
            for b in range(4):
                curr = pwr.v12_current_ma[b] / 1000.0
                watts = curr * 12.0
                brick_cols[b].metric(f"Brick {b+1} (ESCs {b*2},{b*2+1})", f"{curr:.2f} A", f"{watts:.1f} W")
                brick_cols[b].progress(min(1.0, curr / 25.0))
        else:
            st.info("Waiting for Power Telemetry from Power Slab (Node 3)...")

    # =========================================================================
    # TAB 6: RAW CAN BUS MONITOR & PACKET INSPECTOR
    # =========================================================================
    with tab6:
        st.subheader("Raw CAN Bus Monitor & Packet Inspector")
        st.caption("Real-time inspection of all CAN FD arbitration IDs, DLCs, hex payloads, and decoded data flowing across the virtual bus.")

        col_can_filter1, col_can_filter2, col_can_filter3 = st.columns([2, 1, 1])
        with col_can_filter1:
            filter_id = st.selectbox(
                "Filter by CAN ID",
                options=["ALL", "0x001 (EMERGENCY_BREAK)", "0x100 (THRUSTER_CMD)", "0x110 (SOLENOID_CMD)", "0x200 (NAV_TELEMETRY)", "0x210 (ENV_TELEMETRY)", "0x300 (POWER_TELEMETRY)"],
                index=0
            )
        with col_can_filter2:
            st.metric("Total Buffered Frames", len(client.packet_log))
        with col_can_filter3:
            if st.button("Clear Buffer", width="stretch"):
                client.packet_log.clear()
                st.rerun()

        packets = list(client.packet_log)
        if filter_id != "ALL":
            target_id_str = filter_id.split()[0]
            target_id = int(target_id_str, 16)
            packets = [p for p in packets if p.can_id == target_id]

        if packets:
            table_data = []
            for p in reversed(packets[-50:]):
                table_data.append({
                    "Time": p.timestamp,
                    "Direction": p.direction,
                    "ID": f"0x{p.can_id:03X}",
                    "Name": p.name,
                    "DLC": p.dlc,
                    "Hex Payload": p.hex_data,
                    "Decoded Summary": p.decoded_summary,
                })
            st.dataframe(table_data, width="stretch", height=400)
        else:
            st.info("No packets in buffer matching the selected filter.")

    # =========================================================================
    # TAB 7: LIVE C FIRMWARE CONSOLE & CODE VERIFICATION
    # =========================================================================
    with tab7:
        st.subheader("Live C Firmware Console & Native Execution Verification")
        st.markdown(
            "Verify that your **actual C firmware code** is compiling, linking, and executing inside `sil_bridge_server.exe`."
        )

        st.markdown("#### 1. Live C Standard Output (stdout stream from `sil_bridge_server.exe`)")
        c_lines = list(client.c_stdout_log)
        if c_lines:
            st.text_area("C Engine Terminal Output", value="\n".join(c_lines[-40:]), height=300, disabled=True)
        else:
            st.info("No C stdout output yet. Make sure the SIL Engine is running in the sidebar.")

        st.divider()
        st.markdown("#### 2. How to Verify Your C Code is Actually Running")
        st.markdown(
            """
            This host SIL runs **native machine code compiled from the listed C application sources**, with mocked BSP, CAN, and sensors:
            - `nodes/node2_control_board/Core/Src/app.c`
            - `nodes/node1_pi_shield/Core/Src/app.c`
            - `nodes/node3_power_slab/Core/Src/app.c`
            - `shared/src/rov_pwm_ramp.c`, `shared/src/rov_safety.c`, `shared/src/can_interface.c`
            - `drivers/src/lsm6dsoxtr.c`, `drivers/src/ms5837.c`, `drivers/src/bme280.c`

            **To rebuild the native C SIL after changing application code:**
            1. Open `nodes/node2_control_board/Core/Src/app.c` in your editor.
            2. Add any custom debug statement, for example:
               ```c
               printf(">>> MY CUSTOM C CODE: Target T0=%u us, Current Depth=%.2f m\n",
                      g_target_pwms.pwm_us[0], g_depth_dev.depth_m);
               fflush(stdout);
               ```
            3. In a terminal, recompile the C target:
               ```powershell
               cmake --build build-native --target sil_bridge_server
               ```
            4. In the dashboard sidebar, click **Restart Engine**.
            5. Watch your custom `printf` statements appear live right in the box above!
            """
        )

    # Controlled auto-refresh loop. This sends nothing: the 20 Hz control worker
    # owns the command cadence, and a second sender here would emit an extra
    # command frame at whatever rate the operator's browser happened to poll.
    if auto_refresh_due(client, auto_refresh, refresh_rate):
        time.sleep(refresh_rate)
        st.rerun()

if __name__ == "__main__":
    main()
