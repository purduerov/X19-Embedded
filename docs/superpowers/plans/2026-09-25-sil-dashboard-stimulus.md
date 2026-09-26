# SIL Dashboard and Fake-CAN Stimulus Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the host SIL dashboard sustain safe 20 Hz control and provide one maintained, failure-reporting fake-CAN stimulus tool for Nodes 1, 2, and 3.

**Architecture:** A canonical `tools/can_stimulus.py` owns the SIL TCP backend, optional hardware adapters, protocol checks, node smoke tests, and CLI exit status. `tools/node2_stimulus.py` becomes a compatibility wrapper. `SilDashboardClient` owns a 20 Hz control worker with an explicit deadman state machine, while Python integration tests exercise both the dashboard client and stimulus tool against the native `sil_bridge_server`.

**Tech Stack:** Python 3.10+ standard library, existing `tests/sil_companion_bridge/sil_protocol.py`, native C11 SIL server, `unittest`, Streamlit dashboard, GitHub Actions.

**Spec:** `docs/superpowers/specs/2026-09-25-sil-dashboard-stimulus-design.md`

## Global Constraints

- Keep target STM32C542 startup, linker, pin, BSP, FDCAN, and hardware sensor integration out of this host-first change.
- Preserve existing CAN IDs: emergency `0x001`, eFuse `0x005`, thrusters `0x100`, solenoids `0x110`, navigation `0x200`, environment `0x210`, power `0x300`, SIL output status `0x7FE`.
- Emergency frames must contain `0xAA 0x55`; eFuse alerts must contain `0xEF 0x01`.
- PWM values must remain within 1000–2000 us; neutral is 1500 us; the control worker period is exactly 50 ms (20 Hz).
- Any missing, malformed, stale, or incorrect required observation must produce a failed check and a nonzero process status.
- Do not duplicate SIL packet structures or backend logic between the canonical tool and the Node 2 wrapper.
- Use `X19_SIL_SERVER` when the SIL server is not at `build-native/tests/sil_bridge_server[.exe]`.
- Keep the existing native CTest suite authoritative; do not replace it with Python-only coverage.
- Do not add emojis to source, tests, documentation, or commits.

## Review Focus

- A one-shot or stale dashboard command must not leave PWM above neutral after the 100 ms heartbeat window.
- A socket disconnect, worker exception, or transport error must stop command transmission and force neutral where the transport remains usable.
- Unauthorized emergency or eFuse frames must be ignored by the receiving application, and stimulus checks must not treat a rejected frame as a successful actuation test.
- Missing telemetry, decode errors, and timeouts must fail the owning check instead of being logged as warnings while the process exits successfully.
- Two tests or two dashboard sessions must not share a port or leave a `sil_bridge_server` process running after teardown.

---

## File Map

### Create

- `tests/sil_stimulus/__init__.py` — test package marker.
- `tests/sil_stimulus/sil_test_support.py` — free-port allocation, native server lifecycle, and framed-packet helpers.
- `tests/sil_stimulus/test_protocol_roundtrip.py` — pure protocol and output-status framing tests.
- `tests/sil_stimulus/test_dashboard_control.py` — dashboard worker and deadman integration tests.
- `tests/sil_stimulus/test_stimulus_flows.py` — Node 1/2/3 stimulus integration tests against the native SIL server.
- `tests/sil_stimulus/test_stimulus_cli.py` — CLI result aggregation, failure status, and Node 2 wrapper tests.
- `tools/can_stimulus.py` — canonical vehicle-wide fake-CAN/hardware stimulus implementation and CLI.
- `tools/node2_stimulus.py` — legacy Node 2 flag translation and delegation wrapper.

### Modify

- `tests/sil_dashboard/sil_dashboard_client.py` — control state machine, 20 Hz worker, neutral/estop transitions, and UART/SIL send serialization.
- `tests/sil_dashboard/dashboard_app.py` — use the client state machine and expose explicit Stop/All-Stop behavior.
- `.github/workflows/build_and_lint.yml` — install Python test dependencies and run the Python SIL/dashboard checks.
- `docs/sil_simulation_guide.md` — document the 20 Hz deadman, canonical tool, wrapper, and per-node commands.
- `README.md` — link the SIL stimulus and dashboard verification commands.

### Retain unchanged

- `tests/sil_companion_bridge/sil_protocol.py` remains the single Python protocol definition.
- `tests/sil_bridge_server.c`, native node `app.c` files, and existing CTest targets remain the execution authority.

---

### Task 1: Add reusable SIL server test support

**Files:**
- Create: `tests/sil_stimulus/__init__.py`
- Create: `tests/sil_stimulus/sil_test_support.py`
- Test: `tests/sil_stimulus/test_protocol_roundtrip.py`

**Interfaces:**
- Produces `free_tcp_port() -> int`.
- Produces `SilServerProcess(executable: Path, port: int | None = None)` with `start()`, `stop()`, and context-manager methods.
- Produces `send_raw_frame(sock: socket.socket, can_id: int, payload: bytes) -> None` and `recv_frames(sock: socket.socket, timeout: float) -> list[tuple[int, bytes]]`.
- Consumes `pack_sil_can_frame`, `unpack_sil_can_frame`, and `unpack_sil_output_status` from `sil_protocol`.

- [ ] **Step 1: Write the failing framing tests.**

```python
REPO_ROOT = Path(__file__).resolve().parents[2]
SERVER_EXE = Path(os.environ.get(
    "X19_SIL_SERVER",
    str(REPO_ROOT / "build-native" / "tests" / "sil_bridge_server.exe"),
))

class TestSilProtocolRoundtrip(unittest.TestCase):
    def test_command_frame_roundtrip(self):
        payload = ThrusterCommand(pwm_us=[1500, 1600, 1400, 1500, 1500, 1500, 1500, 1500]).pack()
        can_id, decoded = unpack_sil_can_frame(pack_sil_can_frame(CAN_ID_THRUSTER_CMD, payload))
        self.assertEqual(can_id, CAN_ID_THRUSTER_CMD)
        self.assertEqual(ThrusterCommand.unpack(decoded).pwm_us, [1500, 1600, 1400, 1500, 1500, 1500, 1500, 1500])

    def test_output_status_roundtrip(self):
        payload = struct.pack("<8H", 1500, 1600, 1400, 1500, 1500, 1500, 1500, 1500) + struct.pack("<BHI", 1, 0x0123, 4321)
        pwms, brake, solenoids, sim_ms = unpack_sil_output_status(payload)
        self.assertEqual(pwms, [1500, 1600, 1400, 1500, 1500, 1500, 1500, 1500])
        self.assertTrue(brake)
        self.assertEqual(solenoids, 0x0123)
        self.assertEqual(sim_ms, 4321)
```

- [ ] **Step 2: Run the focused test and verify the expected import/setup failure.**

Run from the repository root:

```powershell
python -m unittest tests.sil_stimulus.test_protocol_roundtrip -v
```

Expected: FAIL until the package and test support module exist.

- [ ] **Step 3: Implement the test support module.**

Use a subprocess lifecycle that always terminates the server:

```python
class SilServerProcess:
    def __init__(self, executable: Path, port: int | None = None):
        self.executable = Path(executable)
        self.port = port or free_tcp_port()
        self.process: subprocess.Popen[bytes] | None = None

    def start(self) -> None:
        self.process = subprocess.Popen(
            [str(self.executable), "--port", str(self.port), "--cycles", "0"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            try:
                probe = socket.create_connection(("127.0.0.1", self.port), timeout=0.1)
                probe.close()
                return
            except OSError:
                time.sleep(0.02)
        self.stop()
        raise RuntimeError(f"SIL server did not listen on port {self.port}")

    def stop(self) -> None:
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2.0)
        self.process = None

    def __enter__(self) -> "SilServerProcess":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()
```

`recv_frames` must buffer partial TCP reads and return only complete `SIL_PACKET_SIZE` frames, raising `ValueError` for an invalid magic/length after the caller’s timeout expires.

- [ ] **Step 4: Run the focused test and verify it passes.**

```powershell
python -m unittest tests.sil_stimulus.test_protocol_roundtrip -v
```

Expected: all protocol tests PASS.

- [ ] **Step 5: Commit the test support.**

```powershell
git add tests/sil_stimulus
git commit -m "test: add reusable SIL server fixtures"
```

---

### Task 2: Implement the dashboard 20 Hz deadman controller

**Files:**
- Modify: `tests/sil_dashboard/sil_dashboard_client.py`
- Test: `tests/sil_stimulus/test_dashboard_control.py`

**Interfaces:**
- Produces `ControlState` with `DISCONNECTED`, `STOPPED`, `ARMED`, `RUNNING`, and `ESTOP`.
- Produces `SilDashboardClient.control_state -> ControlState`.
- Produces `start_control_loop()`, `stop_control_loop(send_neutral: bool = True)`, `request_stop()`, and `request_emergency_break()`.
- Keeps `send_surface_pilot_command(...)` and `send_pwms(...)` as compatibility entry points, but makes them update the worker state instead of sending only once.

- [ ] **Step 1: Write a failing sustained-control test.**

```python
class TestDashboardControl(unittest.TestCase):
    def test_command_is_resent_until_stop(self):
        with SilServerProcess(SERVER_EXE) as server:
            client = SilDashboardClient(port=server.port)
            self.assertTrue(client.connect())
            self.addCleanup(client.stop_server_process)
            client.send_pwms([1650] * 8)
            self.assertTrue(client.start_control_loop())
            time.sleep(0.45)
            self.assertGreater(max(client.actual_pwms), 1500)
            client.request_stop()
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline and any(p != 1500 for p in client.actual_pwms):
                time.sleep(0.02)
            self.assertEqual(client.actual_pwms, [1500] * 8)
```

- [ ] **Step 2: Run the focused test and verify it fails because sustained control does not exist.**

```powershell
python -m unittest tests.sil_stimulus.test_dashboard_control -v
```

Expected: FAIL with a missing control-loop method or neutral output after the heartbeat timeout.

- [ ] **Step 3: Add the state and worker fields.**

```python
class ControlState(str, Enum):
    DISCONNECTED = "disconnected"
    STOPPED = "stopped"
    ARMED = "armed"
    RUNNING = "running"
    ESTOP = "estop"
```

Initialize `self.control_state = ControlState.DISCONNECTED`, `self._control_wakeup = threading.Event()`, `self._control_stop = threading.Event()`, `self._control_thread: Optional[threading.Thread] = None`, and `self._last_command = [1500] * 8`.

`connect()` transitions to `ARMED`; `disconnect()` calls `stop_control_loop(send_neutral=False)` and transitions to `DISCONNECTED`.

- [ ] **Step 4: Implement the 50 ms worker.**

```python
def _control_loop(self) -> None:
    while not self._control_stop.wait(0.0):
        try:
            with self.lock:
                if self.control_state is not ControlState.RUNNING:
                    return
                pwms = list(self._last_command)
            self._send_thruster_frame(pwms)
            self._control_wakeup.wait(0.05)
            self._control_wakeup.clear()
        except Exception:
            self._transition_stopped(send_neutral=True)
            return
```

`start_control_loop()` starts exactly one thread, `stop_control_loop()` signals and joins it, and `request_stop()` sets `[1500] * 8`, sends one neutral frame when connected, and transitions to `STOPPED`.

Define the shared transition helper so worker errors and UI actions behave identically:

```python
def _transition_stopped(self, send_neutral: bool = True) -> None:
    self._control_stop.set()
    if send_neutral and self.connected and self.sock:
        self._send_thruster_frame([1500] * 8)
    self._last_command = [1500] * 8
    self.control_state = ControlState.STOPPED
```

`start_control_loop()` returns `True` when a worker is running or already running, and `False` when the client is not connected. `stop_control_loop()` is idempotent and never sends a frame after the transport is disconnected.

Refactor `send_pwms()` so it validates eight values in 1000–2000, updates `_last_command`, sets `RUNNING`, and starts the worker. It must not hold `self.lock` while calling `sendall`.

- [ ] **Step 5: Implement emergency and neutral transitions.**

`trigger_emergency_break()` must set `ESTOP`, stop the worker, send the authorized `0xAA 0x55 0x01` frame, send `[1500] * 8`, and keep the state latched until `start_server_process()`/`connect()` is called again. Any `OSError` in the worker must call the same neutral transition.

- [ ] **Step 6: Run the focused dashboard tests.**

```powershell
python -m unittest tests.sil_stimulus.test_dashboard_control -v
```

Expected: sustained-command, stop-neutral, disconnect, and emergency tests PASS.

- [ ] **Step 7: Commit the dashboard controller.**

```powershell
git add tests/sil_dashboard/sil_dashboard_client.py tests/sil_stimulus/test_dashboard_control.py
git commit -m "fix: add 20Hz dashboard deadman control"
```

---

### Task 3: Update the dashboard UI to use the control state machine

**Files:**
- Modify: `tests/sil_dashboard/dashboard_app.py`
- Test: `tests/sil_stimulus/test_dashboard_app.py`

**Interfaces:**
- Consumes `client.control_state`, `client.request_stop()`, and `client.request_emergency_break()` from Task 2.
- Produces a visible control-state indicator and an explicit All-Stop action that calls `request_stop()`.

- [ ] **Step 1: Write the import/UI-surface test.**

```python
class TestDashboardAppSurface(unittest.TestCase):
    def test_dashboard_module_imports(self):
        import dashboard_app
        self.assertTrue(hasattr(dashboard_app, "main"))

    def test_all_stop_uses_deadman_api(self):
        source = Path(dashboard_app.__file__).read_text(encoding="utf-8")
        self.assertIn("request_stop", source)
        self.assertIn("control_state", source)
```

- [ ] **Step 2: Run the test and verify the current one-shot UI fails it.**

```powershell
python -m unittest tests.sil_stimulus.test_dashboard_app -v
```

Expected: FAIL until the UI uses the new client methods.

- [ ] **Step 3: Replace one-shot UI actions.**

The All-Stop buttons in the sidebar and thruster tab must call `client.request_stop()`. Emergency buttons must call `client.request_emergency_break()`. The pilot and thruster controls may continue calling `send_surface_pilot_command()`/`send_pwms()` because Task 2 makes those methods start and update the worker.

Add a sidebar metric:

```python
st.metric("Control State", client.control_state.value)
```

Do not send a second command from the Streamlit auto-refresh loop; the worker owns the 20 Hz cadence.

- [ ] **Step 4: Run the UI-surface and client tests.**

```powershell
python -m unittest tests.sil_stimulus.test_dashboard_app tests.sil_stimulus.test_dashboard_control -v
```

Expected: all dashboard tests PASS.

- [ ] **Step 5: Commit the UI integration.**

```powershell
git add tests/sil_dashboard/dashboard_app.py tests/sil_stimulus/test_dashboard_app.py
git commit -m "fix: wire dashboard controls to deadman state"
```

---

### Task 4: Create the canonical fake-CAN stimulus tool

**Files:**
- Create: `tools/can_stimulus.py`
- Test: `tests/sil_stimulus/test_stimulus_flows.py`

**Interfaces:**
- Produces `SilSocketBackend(host: str, port: int, auto_start: bool = True)` with `connect()`, `send_frame()`, `recv_frame()`, `latest_outputs()`, and `close()`.
- Produces `CheckResult(name: str, passed: bool, detail: str)` and `StimulusReport(results: list[CheckResult])` with an `ok` property.
- Produces `VehicleStimulusTester(backend).run_full_smoke() -> StimulusReport`.
- Produces `run_selected_action(tester: VehicleStimulusTester, args: argparse.Namespace) -> StimulusReport` for one CLI action or a combined selection.
- Produces `main(argv: list[str] | None = None) -> int` for the canonical CLI.

- [ ] **Step 1: Write failing protocol/backend tests.**

```python
class TestStimulusBackend(unittest.TestCase):
    def test_backend_rejects_oversized_payload(self):
        backend = SilSocketBackend("127.0.0.1", 1, auto_start=False)
        with self.assertRaises(ValueError):
            backend.send_frame(CAN_ID_THRUSTER_CMD, b"x" * 65)

    def test_report_fails_when_a_check_fails(self):
        report = StimulusReport([CheckResult("node3", False, "missing frame")])
        self.assertFalse(report.ok)
```

- [ ] **Step 2: Run the focused tests and verify the import failure.**

```powershell
python -m unittest tests.sil_stimulus.test_stimulus_flows -v
```

Expected: FAIL because `tools.can_stimulus` does not exist in the merged branch.

- [ ] **Step 3: Implement canonical protocol loading and backends.**

Add `tools` to `sys.path`, then import all protocol names from `sil_protocol`. Do not define fallback packet classes. `SilSocketBackend` must:

- search `X19_SIL_SERVER`, `build-native`, and `build` for the server executable;
- auto-start the server only in SIL mode;
- validate payload length `<= 64`;
- buffer partial TCP frames;
- return decoded `(can_id, payload)` pairs;
- track the latest `CAN_ID_SIL_OUTPUT_STATUS` readback;
- terminate its child process in `close()`.

Implement `PythonCanBackend` and `UartBackend` as optional adapters that raise a clear `RuntimeError` when their dependency is absent.

- [ ] **Step 4: Implement named checks with real field names.**

Use the actual `sil_protocol.PowerTelemetry` fields: `tether_voltage_mv`, `tether_current_ma`, `v5_voltage_mv`, `v5_current_ma`, `v12_current_ma`, `pcb_temp_c_tenths`, and `status_flags`.

`VehicleStimulusTester` must provide:

```python
def monitor_node1_env(self, duration_s: float = 3.0) -> CheckResult
def test_node2_arming(self) -> CheckResult
def test_node2_pwm(self, channel: int, pulse_us: int, duration_s: float = 1.0) -> CheckResult
def test_node2_all(self, pulse_us: int, duration_s: float = 1.0) -> CheckResult
def test_node2_solenoid(self, mask: int) -> CheckResult
def monitor_node2_depth(self, duration_s: float = 2.0) -> CheckResult
def monitor_node3_power(self, duration_s: float = 3.0) -> CheckResult
def test_emergency_break(self) -> CheckResult
def run_full_smoke(self) -> StimulusReport
```

Every method must return a failed `CheckResult` on timeout, decode error, wrong signature, or missing output readback. The arming check must require a neutral observation before 3 seconds and a non-neutral observation after arming. The emergency check must send `b"\xAA\x55\x01"` and require `brake_active == True` plus all-neutral PWM readback.

- [ ] **Step 5: Add CLI parsing and nonzero status.**

Support `--mode sil|can|uart`, `--host`, `--port`, `--node1-env`, `--node2-thruster`, `--node2-all`, `--node2-depth`, `--node2-arm`, `--node2-solenoid`, `--node3-power`, `--emergency-break`, `--raw-send`, `--sniff`, and `--auto`. `run_selected_action()` must map each supplied flag to exactly one tester method and return a report containing every requested check:

```python
def run_selected_action(tester, args):
    checks = []
    if args.node1_env:
        checks.append(tester.monitor_node1_env())
    if args.node2_arm:
        checks.append(tester.test_node2_arming())
    if args.node2_thruster:
        checks.append(tester.test_node2_pwm(args.node2_thruster[0], args.node2_thruster[1]))
    if args.node2_all is not None:
        checks.append(tester.test_node2_all(args.node2_all))
    if args.node2_depth is not None:
        checks.append(tester.monitor_node2_depth(args.node2_depth))
    if args.node2_solenoid is not None:
        checks.append(tester.test_node2_solenoid(args.node2_solenoid))
    if args.node3_power:
        checks.append(tester.monitor_node3_power())
    if args.emergency_break:
        checks.append(tester.test_emergency_break())
    return StimulusReport(checks)
```

`main()` must return `0` only when `report.ok` is true:

```python
report = tester.run_full_smoke() if args.auto else run_selected_action(tester, args)
return 0 if report.ok else 1
```

Do not print an unconditional `[PASS]` line; the CLI summary must list failed checks and the process must exit nonzero.

- [ ] **Step 6: Run the focused backend tests.**

```powershell
python -m unittest tests.sil_stimulus.test_stimulus_flows -v
```

Expected: backend, framing, and report-status tests PASS.

- [ ] **Step 7: Commit the canonical tool.**

```powershell
git add tools/can_stimulus.py tests/sil_stimulus/test_stimulus_flows.py
git commit -m "feat: add unified vehicle CAN stimulus tool"
```

---

### Task 5: Add real per-node SIL integration checks

**Files:**
- Modify: `tests/sil_stimulus/test_stimulus_flows.py`

**Interfaces:**
- Consumes `SilServerProcess` and `VehicleStimulusTester` from Tasks 1 and 4.
- Produces one independent integration test per node, each with its own free port and server process.

- [ ] **Step 1: Add the Node 1 test.**

```python
def test_node1_environment_telemetry(self):
    with SilServerProcess(SERVER_EXE) as server:
        backend = SilSocketBackend("127.0.0.1", server.port, auto_start=False)
        backend.connect()
        try:
            result = VehicleStimulusTester(backend).monitor_node1_env(2.0)
        finally:
            backend.close()
        self.assertTrue(result.passed, result.detail)
```

- [ ] **Step 2: Add the Node 2 arming/PWM and solenoid/emergency tests.**

Use a fresh `SilServerProcess` for each test. The arming test must observe the neutral interval and transition; the PWM test must resend the command at least every 50 ms and compare `CAN_ID_SIL_OUTPUT_STATUS`; the solenoid test must use a valid single-coil mask and verify the readback; the emergency test must verify brake and neutral state.

- [ ] **Step 3: Add the Node 3 telemetry test.**

Require at least one `CAN_ID_POWER_TELEMETRY` frame, decode it with `PowerTelemetry.unpack`, and assert finite/in-range fields rather than treating a decode exception as a warning.

- [ ] **Step 4: Run the per-node integration tests.**

```powershell
$env:X19_SIL_SERVER = (Resolve-Path "build-native/tests/sil_bridge_server.exe").Path
python -m unittest tests.sil_stimulus.test_stimulus_flows -v
Remove-Item Env:X19_SIL_SERVER
```

Expected: Node 1, Node 2, and Node 3 integration tests PASS, with no leftover server process.

- [ ] **Step 5: Commit the per-node coverage.**

```powershell
git add tests/sil_stimulus/test_stimulus_flows.py
git commit -m "test: cover each node through fake CAN stimulus"
```

---

### Task 6: Add the Node 2 compatibility wrapper and CLI tests

**Files:**
- Create: `tools/node2_stimulus.py`
- Test: `tests/sil_stimulus/test_stimulus_cli.py`

**Interfaces:**
- Produces `translate_legacy_args(argv: list[str]) -> list[str]` in `node2_stimulus.py`.
- Produces `main(argv: list[str] | None = None) -> int` in both tools.
- Consumes the canonical `tools.can_stimulus.main()`.

- [ ] **Step 1: Write failing wrapper tests.**

```python
class TestStimulusCli(unittest.TestCase):
    def test_legacy_flags_translate(self):
        from node2_stimulus import translate_legacy_args
        self.assertEqual(
            translate_legacy_args(["--test-thruster", "0", "1600"]),
            ["--node2-thruster", "0", "1600"],
        )
        self.assertEqual(translate_legacy_args(["--test-emergency"]), ["--emergency-break"])

    def test_failed_report_returns_nonzero(self):
        from can_stimulus import CheckResult, StimulusReport
        self.assertEqual(1 if not StimulusReport([CheckResult("x", False, "no data")]).ok else 0, 1)
```

- [ ] **Step 2: Run the focused tests and verify the wrapper is absent.**

```powershell
python -m unittest tests.sil_stimulus.test_stimulus_cli -v
```

Expected: FAIL with an import error for `node2_stimulus`.

- [ ] **Step 3: Implement the wrapper.**

Translate the legacy mappings:

```python
LEGACY_FLAGS = {
    "--test-thruster": "--node2-thruster",
    "--test-all": "--node2-all",
    "--test-arm": "--node2-arm",
    "--test-depth": "--node2-depth",
    "--test-solenoid": "--node2-solenoid",
    "--test-emergency": "--emergency-break",
}
```

Preserve argument values, add the `tools` directory to `sys.path`, call the canonical `main`, and return its integer status. Do not duplicate backend or protocol code.

- [ ] **Step 4: Add a subprocess help/exit-status test.**

Run `python tools/node2_stimulus.py --help` and assert exit code 0. Run an invalid node action and assert exit code 2 or 1, never 0.

- [ ] **Step 5: Run the CLI tests and commit.**

```powershell
python -m unittest tests.sil_stimulus.test_stimulus_cli -v
git add tools/node2_stimulus.py tests/sil_stimulus/test_stimulus_cli.py
git commit -m "feat: preserve Node 2 stimulus CLI compatibility"
```

---

### Task 7: Add CI and documentation for the complete host flow

**Files:**
- Modify: `.github/workflows/build_and_lint.yml`
- Modify: `docs/sil_simulation_guide.md`
- Modify: `README.md`

**Interfaces:**
- CI consumes the `build-native/tests/sil_bridge_server` output through `X19_SIL_SERVER`.
- Documentation exposes the canonical and wrapper commands with explicit expected pass/fail behavior.

- [ ] **Step 1: Add the CI Python dependency/test step.**

After the native build, install `pyzmq protobuf streamlit` and run:

```yaml
- name: Run Python SIL and Dashboard Checks
  run: |
    python -m pip install --upgrade pyzmq protobuf streamlit
    python -m py_compile tests/sil_dashboard/*.py tests/sil_companion_bridge/*.py tools/can_stimulus.py tools/node2_stimulus.py
    X19_SIL_SERVER="$PWD/build-native/tests/sil_bridge_server" \
      python -m unittest discover -s tests/sil_stimulus -p 'test_*.py' -v
```

The step must be blocking for the host test surface; target readiness remains its existing `continue-on-error` step.

- [ ] **Step 2: Document the control contract.**

Add commands and expected behavior:

```powershell
python tests/sil_dashboard/run_dashboard.py
python tools/can_stimulus.py --mode sil --auto
python tools/node2_stimulus.py --mode sil --test-depth 2
```

State that the dashboard worker sends at 20 Hz, All-Stop/ESTOP force neutral, the canonical tool is `can_stimulus.py`, and the Node 2 file is a compatibility wrapper. State that nonzero exit means a required SIL observation failed.

- [ ] **Step 3: Link the commands from the root README.**

Add a short SIL verification subsection pointing to the simulation guide and explicitly separating host SIL results from target readiness.

- [ ] **Step 4: Run the documented commands locally.**

```powershell
cmake --build build-native --target sil_bridge_server
$env:X19_SIL_SERVER = (Resolve-Path "build-native/tests/sil_bridge_server.exe").Path
python -m unittest discover -s tests/sil_stimulus -p "test_*.py" -v
python -m py_compile tests/sil_dashboard/*.py tools/can_stimulus.py tools/node2_stimulus.py
Remove-Item Env:X19_SIL_SERVER
```

Expected: all Python SIL tests PASS and all syntax checks exit 0.

- [ ] **Step 5: Commit CI and documentation.**

```powershell
git add .github/workflows/build_and_lint.yml docs/sil_simulation_guide.md README.md
git commit -m "ci: verify dashboard and per-node SIL stimulus"
```

---

## Final Verification

Run from a clean worktree:

```powershell
cmake -B build-native -G Ninja
cmake --build build-native
ctest --test-dir build-native -E "^target_" --output-on-failure
$env:X19_SIL_SERVER = (Resolve-Path "build-native/tests/sil_bridge_server.exe").Path
python -m unittest discover -s tests/sil_stimulus -p "test_*.py" -v
python tests/sil_companion_bridge/test_full_system_sil.py
python -m py_compile tests/sil_dashboard/*.py tests/sil_companion_bridge/*.py tools/can_stimulus.py tools/node2_stimulus.py
Remove-Item Env:X19_SIL_SERVER
```

Expected results:

- Native CTest: 25/25 pass.
- Python SIL stimulus tests: all pass.
- Existing companion SIL test: 4/4 pass.
- Dashboard and stimulus syntax checks: exit 0.
- Target readiness remains separately documented and non-blocking; it is not counted as host SIL success.
