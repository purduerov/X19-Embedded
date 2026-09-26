# Software-in-the-Loop (SIL) Architecture & Co-Simulation Guide

> **Purdue ROV — X19 Embedded Software-in-the-Loop (SIL) Framework**  
> *Seamless co-simulation of STM32 subsea node firmware and Raspberry Pi 5 ZeroMQ/Protobuf companion software without physical hardware.*

---

## 1. Overview

The X19 SIL framework enables testing **100% of subsea STM32 microcontroller firmware state machines**, CAN FD protocol serialization, and companion computer communications on host machines (Windows, macOS, Linux/WSL2, and CI).

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                             HOST MACHINE (PC / CI)                          │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  [ PILOT STATION (Simulated / Surface UI) ]                                 │
│    - Publishes JoystickCommand over ZeroMQ (tcp://127.0.0.1:5556)          │
│    - Subscribes to SensorData over ZeroMQ (tcp://127.0.0.1:5555)           │
│                                 │                                           │
│                                 ▼                                           │
│  [ PI 5 CORE CAN <-> ZMQ GATEWAY (X19-Core Codebase) ]                     │
│    - Python `src.python.messaging.Publisher` & `Subscriber`                │
│    - Converts Joystick Protobuf -> 8-Thruster CAN Frame (0x100)            │
│    - Unpacks Navigation Telemetry (0x200) -> Protobuf SensorData           │
│                                 │                                           │
│                         TCP Framing Socket                                  │
│                          (127.0.0.1:8765)                                   │
│                                 ▼                                           │
│  [ STM32 MULTI-NODE SIL ENGINE (sil_bridge_server) ]                         │
│    - Discrete 100 Hz Virtual / Wall-Clock Stepping                          │
│    - In-Memory CAN FD Bus Router (`mock_can.c`)                             │
│    - Synthetic Sensor Injection (`mock_sensors.c`)                          │
│                                                                             │
│    ┌────────────────────┬────────────────────┬────────────────────┐        │
│    ▼                    ▼                    ▼                    ▼        │
│  NODE 1 (Pi Shield)   NODE 2 (Control)     NODE 3 (Power Slab)   CAN FD    │
│  - BME280 leak mon    - 8x ESC PWM ramping - 5x PMBus converters Bus Route │
│  - 0x001 E-Break      - 10x Solenoid GPIOs - 0x005 eFuse Alert    History   │
│  - 0x210 Env Stream   - 0x200 Nav (100 Hz) - 0x300 Power Stream   & Drops   │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Components

1. **`sil_bridge_server` (C Native Executable)**:
   - Compiles Node 1 (`nodes/node1_pi_shield/Core/Src/app.c`), Node 2 (`nodes/node2_control_board/Core/Src/app.c`), and Node 3 (`nodes/node3_power_slab/Core/Src/app.c`) with mock BSP and CAN drivers.
   - Listens on TCP `127.0.0.1:8765`.
   - Steps all node state machines concurrently at 100 Hz.
   - Forwards frames between the virtual CAN bus and the TCP client.

2. **`PiCoreSilBridge` (`tests/sil_companion_bridge/pi_core_sil_bridge.py`)**:
   - Imports directly from `X19-Core`:
     - `src.python.messaging.Publisher`
     - `src.python.messaging.Subscriber`
     - `src.protocols.python.telemetry_pb2`
   - Handles the bridge between topside ZeroMQ messages and CAN FD packets.

3. **`test_full_system_sil.py`**:
   - Automated end-to-end regression test:
     - Boots `sil_bridge_server`.
     - Connects `PiCoreSilBridge`.
     - Publishes pilot joystick movements.
     - Verifies thruster PWM ramping on the STM32 Control Board.
     - Receives navigation telemetry from the STM32 depth sensor and IMU.
     - Tests emergency break thruster cutoffs.

---

## 3. How to Run the Tests

### Fast CTest Unit & Multi-Node Suite
```powershell
cd X19-Embedded
cmake --build build --target test_multi_node_bus sil_bridge_server
ctest --test-dir build --output-on-failure
```

### Full Multi-Process SIL Test with X19-Core ZMQ
```powershell
cd X19-Embedded
python tests/sil_companion_bridge/test_full_system_sil.py
```

### Interactive Testing Dashboard (Streamlit GUI)
To interactively test thrusters, toggle pneumatic solenoids, inject fault conditions (e-breaks, leaks), and inspect live 100 Hz navigation, environmental, and power telemetry via browser UI:
```powershell
cd X19-Embedded
python tests/sil_dashboard/run_dashboard.py
```
Or run directly via Streamlit:
```powershell
cd X19-Embedded
streamlit run tests/sil_dashboard/dashboard_app.py
```
Dashboard features:
- **One-Click Server Management**: Start/stop the native STM32 SIL simulation binary from the sidebar.
- **8-Thruster Live Controls**: Real-time PWM sliders with automatic slew-rate ramping and individual effort visualization.
- **10-Channel Pneumatics**: Toggle switches for all 5 double-acting SMC solenoid valves.
- **Emergency Break Injection**: Priority 0 `0x001` trigger to test instant motor shutdown and hardware latch.
- **Live Graphs & Telemetry**: 100 Hz depth and quaternion attitude tracking, BME280 leak detection, and 4x PMBus converter metrics.

### Per-Node and Whole-Vehicle Stimulus Checks

[`tools/can_stimulus.py`](../tools/can_stimulus.py) is the **canonical** stimulus tool. It injects stimulus at the CAN boundary and verifies the result against the board's own `0x7FE` output-status readback, which exists only inside the SIL engine. When nothing is listening on the target port it launches an engine of its own and reaps it on exit.

**When something *is* already listening, the tool refuses** and exits 2, before a single frame is written:

```
[FAIL] refused: refused before sending any frame: something is already listening on
127.0.0.1:8765 and this run did not start it, so the checks would drive an engine
this tool does not own. ...
```

That is not caution for its own sake. `--emergency-break` drives all eight thrusters to 1800 us and latches the emergency brake of whatever engine it reaches, for the life of that engine, and the dashboard starts an engine on port 8765 on every render — so the commands below, run while a dashboard is open, would otherwise commandeer a live operator's vehicle and report `[PASS]` while doing it. Two ways forward:

- **Use your own engine.** Pass `--port` with a port nothing is listening on; the tool launches one, reaps it, and reports the result honestly.
- **Deliberately attach.** Pass `--attach-existing` and name the individual checks you want. `--auto` is refused alongside it, because there is no harmless whole-vehicle run against somebody else's engine.

```powershell
# Build the native engine the checks drive
cmake -B build-native -G Ninja
cmake --build build-native --target sil_bridge_server

# Whole-vehicle smoke sequence: all eight checks, in the only order that is meaningful
python tools/can_stimulus.py --mode sil --auto

# One check at a time (--help lists the rest)
python tools/can_stimulus.py --mode sil --node2-arm
```

`--auto` runs the arming gate first, because nothing may send a thruster command before it and the 3000 ms window cannot be observed retroactively. It runs the emergency break **last**, because that latch holds for the life of the engine and every check after it would only be observing a permanently disarmed board.

| Flag | Asserts |
|---|---|
| `--node1-env` | `0x210` enclosure telemetry decodes, with leak flags clear |
| `--node2-arm` | outputs held neutral for the first 3000 ms of sim time against a resent 1700 us command, then took effect and released |
| `--node2-thruster N US` | channel `N` held `US` past the 100 ms watchdog with the other seven neutral, then released |
| `--node2-all US` | all eight channels held `US` past the watchdog, then released |
| `--node2-solenoid MASK` | the mask energises one coil per valve, the board reports that mask back, then releases to `0x0000` |
| `--node2-depth S` | `0x200` depth telemetry decodes at the design rate |
| `--node3-power` | `0x300` power telemetry decodes, with status flags clear |
| `--emergency-break` | the authorized `AA 55 01` frame latches the brake, an unauthorized `AA 56 01` is ignored, and a later 1800 us command does not resume thrust |
| `--raw-send ID:HEX` | the write reached the transport; asserts nothing about the vehicle |
| `--sniff S` | decodes every non-`0x7FE` frame for `S` seconds |

### Node 2 Legacy Flags

[`tools/node2_stimulus.py`](../tools/node2_stimulus.py) is a **compatibility wrapper**, not a second implementation. It translates the legacy Node 2 flag names and then delegates every check, every verdict, and the exit status to `can_stimulus.py`. New work belongs in the canonical tool.

```powershell
# --test-depth 2 is translated to --node2-depth 2, then delegated
python tools/node2_stimulus.py --mode sil --test-depth 2

# The wrapper's help prints the whole legacy-to-canonical table
python tools/node2_stimulus.py --help
```

A legacy invocation also prints a one-line `[COMPAT]` notice on stderr naming the canonical flag and whether it actuates. Three legacy flags (`--bitrate`, `--data-bitrate`, `--channel`) have no canonical counterpart and are refused with exit 2 rather than silently dropped.

---

## 4. 6-DOF Hydrodynamic & Electrical Plant Simulation

The SIL engine includes a physics plant model ([`tests/mocks/mock_physics.c`](../tests/mocks/mock_physics.c) / [`tests/mocks/mock_physics.h`](../tests/mocks/mock_physics.h)) that simulates closed-loop ROV dynamics:

- **6-DOF Rigid-Body Dynamics**: Linear velocities ($u, v, w$), world coordinates ($x, y, z$), orientation quaternions ($q_w, q_x, q_y, q_z$), and body angular rates ($p, q, r$).
- **Hydrodynamic Forces**:
  - Restoring forces from vehicle mass (18.5 kg) and net positive buoyancy (+2.1 N upward).
  - Non-linear quadratic and linear hydrodynamic drag in surge, sway, and heave.
  - Thruster force allocation matrix for 4 horizontal vectored and 4 vertical thrusters based on T200 PWM-thrust curves.
- **Electrical & Thermal Plant**:
  - Thruster electrical current consumption modeling based on commanded PWM.
  - Current distribution across the 4x 12V 300W DC-DC converter bricks.
  - Converter temperature modeling and 25A eFuse overcurrent trip protection.
  - Automatic injection into synthetic hardware sensor mocks (`MS5837` hydrostatic pressure and `TPS25990`/`INA226`/`INA237` electrical monitors).

---

## 5. Headless Fast-Forward Mode

For automated continuous integration, long-term stability runs, and rapid batch simulation, `sil_bridge_server` supports running without real-time delays or TCP sockets:

```powershell
# Run 10,000 cycles (100 seconds of virtual simulation time) at maximum CPU speed
./build/tests/sil_bridge_server --fast-forward 10000
```

---

## 6. Comprehensive CTest Suite (25 Test Suites)

The SIL test suite comprises 25 suites testing hardware drivers, communications, safety invariants, plant physics, and multi-node integration:

| # | Test Target | Description |
|---|---|---|
| 1 | `test_can_protocol` | CAN FD frame serialization, unpacking, and validation |
| 2 | `test_pwm_ramp` | 1 kHz slew-rate limiter and curve shaping |
| 3 | `test_safety` | Watchdog timeouts and emergency break triggers |
| 4 | `test_i2c_recovery` | 9-clock bit-bang I2C bus clear routine |
| 5 | `test_timesync` | Distributed vehicle clock synchronization and time slew |
| 6 | `test_bsp` | Board support package time, PWM bounds, and emergency brake |
| 7–16 | `test_driver_*` | Unit tests for BME280, MS5837, INA226, INA237, TCAN1044, TPS25990, LSM6DSOXTR, BMI270, TMP1075, PMBus brick |
| 17 | `test_mock_physics` | Closed-loop 6-DOF hydrodynamic physics and closed-loop depth PID hold |
| 18 | `test_node1_pi_shield` | Pi Shield enclosure pressure, humidity, and leak detection |
| 19 | `test_node2_control_board` | Control board thruster slew rate, solenoid actuation, and 100 Hz nav stream |
| 20 | `test_node3_power_slab` | Power slab PMBus telemetry, current monitoring, and 20 Hz power stream |
| 21 | `test_multi_node_bus` | Full virtual CAN FD bus integration across all 3 nodes and Pi Core |
| 22 | `test_sil_safety` | Tether watchdog SLA, emergency break virtual-time latency, and pneumatics |
| 23 | `test_sil_power` | Full thruster load current modeling, eFuse protection, and telemetry continuity |
| 24 | `test_sil_fuzz` | 1,000 randomized malformed CAN FD frames and Bus-Off fault recovery |
| 25 | `test_sil_burnin` | 100,000-cycle (16.7 min virtual time) continuous stability and ramp alternating |

---

## 7. Control Contract, Exit Status, and Test Discovery

### 7.1 The dashboard deadman

The dashboard control worker ([`tests/sil_dashboard/sil_dashboard_client.py`](../tests/sil_dashboard/sil_dashboard_client.py)) resends the last accepted thruster command every **50 ms (20 Hz)**. The firmware heartbeat window is **100 ms** (`ROV_HEARTBEAT_TIMEOUT_MS` in [`shared/include/rov_parameters.h`](../shared/include/rov_parameters.h)), so a stalled worker, a dropped socket, or a closed browser tab takes the board below neutral **within 100 ms** — two missed resends.

| Operator action | Control state | 20 Hz worker |
| :--- | :--- | :--- |
| Non-neutral thruster or pilot command | `RUNNING` | running; the heartbeat stays refreshed |
| All-neutral command, `send_pwms([1500] * 8)` | `ARMED` | **stopped** |
| **All Stop** button | `STOPPED` | stopped, after one confirming neutral frame |
| **E-Stop** button | `ESTOP`, latched until reconnect | stopped; the authorized `0x001` `AA 55 01` frame is the next thing on the wire |

**Holding neutral does not keep the link alive, and that is deliberate.** The firmware watchdog is what actually releases the pneumatics: `node2_force_neutral()` puts `bsp_solenoid_set(0)` inside itself ([`nodes/node2_control_board/Core/Src/app.c`](../nodes/node2_control_board/Core/Src/app.c) lines 49-52) and is called when `heartbeat_lost` goes true (lines 187-189). An operator who expected "hold neutral" to keep the heartbeat alive would be wrong in the dangerous direction: a continuously refreshed neutral heartbeat prevents the watchdog from ever firing, so the solenoid mask would stay energized indefinitely behind a dashboard that still claims to be running. So an all-neutral command lands in `ARMED` with **no** worker, the heartbeat lapses within 100 ms, and Node 2 zeroes the mask itself. The dashboard also gives each browser tab its own command baseline rather than reading a shared one, so opening a second tab cannot silently re-arm a held thrust.

### 7.2 Exit status of the stimulus tools

`tools/can_stimulus.py`, and therefore `tools/node2_stimulus.py`, exits:

| Status | Meaning |
| --- | --- |
| `0` | every selected check **observed** what it required |
| `1` | at least one check failed, or the engine could not be reached |
| `2` | usage error: no action selected, an unusable value, a refused legacy flag, or a refusal to actuate an engine this run did not start (see 3) |

A nonzero exit means a required SIL observation was **missing, malformed, stale, or incorrect**. It never means a crash was swallowed: each check turns its own observation failures into a failed result, a guard converts anything that still escapes into a failed result, and the CLI turns a failure to reach the transport into a nonzero status. The tool cannot report success while a check did not actually run. Every result is also streamed as it completes, so a run killed part way through still leaves per-check evidence in its output.

`--mode can` and `--mode uart` are receive-only. The `0x7FE` output-status readback is a SIL-only mock-BSP channel, so on real hardware the checks that would need to actuate refuse **before** sending a frame, rather than actuating blind and failing afterwards.

### 7.3 The emergency check actuates

**`--emergency-break` drives all eight thrusters to 1800 us and ramps them back** before it sends the authorized `AA 55 01` frame, because a latch claim is only worth something if the board was demonstrably accepting thrust beforehand. It then latches the brake for the life of that engine. Measured cost on a developer machine: about 1.5 s against an engine that is already armed, and about 7 s when the tool has to launch an engine of its own and wait out the 3000 ms arming window first. The legacy `--test-emergency` does the same thing. Know this before running either.

### 7.4 The arming check needs a fresh engine

`--node2-arm` (and `--test-arm`) proves the mandatory 3000 ms ESC arming gate held neutral while a 1700 us command was resent, then took effect once the gate closed. The window is measured on the engine's virtual clock and cannot be observed retroactively, so against an engine already past 3000 ms the check **legitimately fails**:

```
[FAIL] node2_arming: the arming gate was already open when the first snapshot arrived
(sim_time_ms=NNNN >= 3000), so the mandatory 3000 ms window could not be observed.
Restart the engine (or let this tool launch one) and run the arming check first.
```

Run it first, against a fresh engine. That is why `--auto` puts it first, why the tool refuses to run against an engine it did not start (3), and why every per-node integration test starts an engine of its own. `NNNN` is whatever the engine's virtual clock had already reached; a real capture on a developer machine read `4090`.

### 7.4a A check can fail for a reason that is not the vehicle

Every check measures on the **engine's virtual clock**, not on wall time. The engine advances simulated time by 10 ms per 10 ms of real sleep, so on a loaded or virtualised host the simulation can run several times slower than real time. A check that needs N telemetry frames therefore has to wait N frame-periods of *simulated* time, and the tools compute that budget from the clock the frames actually carry rather than from how long the wall has been.

This matters when a check **fails**: a frame-count shortfall on a host running at a fraction of real time says the host was too slow, not that the Power Slab or the Pi Shield is unhealthy. Read the failure text — it names which clock the window was measured on. Before blaming a node for a telemetry-count failure, re-run the same command on an idle machine and compare.

The converse is the property the tooling exists to guarantee: no check can pass by inferring a rate from a constant. Where a rate is reported, it is derived from the `sim_time_ms` deltas in the engine's own output-status snapshots, and a transport that produces no snapshots reports `UNMEASURED` rather than a guess.

### 7.5 Pointing the tools at a build tree: `X19_SIL_SERVER`

Both the stimulus tools and the SIL test suites look for the native engine in `build-native/tests/`, then `build/`, `build-test/`, and `build-host/`. Set `X19_SIL_SERVER` to select a binary explicitly whenever it lives anywhere else:

```powershell
$env:X19_SIL_SERVER = (Resolve-Path "build-native/tests/sil_bridge_server.exe").Path
# ... run the checks ...
Remove-Item Env:X19_SIL_SERVER
```

The tools read the same variable when they launch an engine of their own, so one setting covers both the checks and the suites.

### 7.6 Running the Python SIL suites

```powershell
# The native engine first: the server-backed tests drive it
cmake -B build-native -G Ninja
cmake --build build-native

# Dashboard control, stimulus CLI, protocol round-trip, and per-node integration
$env:X19_SIL_SERVER = (Resolve-Path "build-native/tests/sil_bridge_server.exe").Path
python -m unittest discover -s tests/sil_stimulus -p "test_*.py" -v

# The X19-Core ZeroMQ/Protobuf companion bridge, from the multi-repository workspace
python tests/sil_companion_bridge/test_full_system_sil.py

# Syntax gate for everything the suites import or shell out to. PowerShell does
# not expand a glob for a native command, so expand it explicitly.
python -m py_compile (Get-ChildItem tests/sil_dashboard/*.py, tests/sil_companion_bridge/*.py).FullName tools/can_stimulus.py tools/node2_stimulus.py

Remove-Item Env:X19_SIL_SERVER
```

The same gate under bash, which is what CI runs, can use the glob directly:

```bash
python -m py_compile tests/sil_dashboard/*.py tests/sil_companion_bridge/*.py tools/can_stimulus.py tools/node2_stimulus.py
```

The `tests/sil_stimulus/` suites import only the Python standard library, this repository's own modules, and `streamlit` (which `tests/sil_dashboard/dashboard_app.py` imports at module scope). `streamlit` is therefore required, not optional: `test_dashboard_app.py` raises `SkipTest` from `setUpModule` when `dashboard_app.py` cannot be imported, which costs all 55 of its tests at once. Install it before running the discovery:

```powershell
python -m pip install --upgrade "streamlit>=1.40,<2"
```

That bound is the single source of truth for CI: the workflow step holds it in one variable, installs from it, and then greps this guide for the same string, so the two cannot drift apart silently. If you change one, change both or the step fails. Use `--upgrade` locally too, or a Streamlit already installed outside the range will simply be kept.

The 26 tests in `test_dashboard_control.py` cover the 20 Hz deadman in the client itself and need no Streamlit at all, so a missing install degrades to 229 passing plus 55 skipped rather than to a total loss.

CI runs Python 3.14, because that is the interpreter this suite was verified on: the passing result, the timing, the companion result, and the log shape all come from 3.14. The pin is a choice, not a necessity — the suite was also run end to end on 3.11.15 and passed there too, so the code is not version-fragile.

The counts below are the size of the suite **at the moment each row was measured**. The suite has grown since the 3.11 row was taken, so treat that row as evidence that the *code* runs on 3.11, not as the current count. The current count is 336, verified on 3.14.

| Interpreter | Streamlit | Result |
| :--- | :--- | :--- |
| 3.14.x (the newest 3.14 release; CI pins `'3.14'`) | 1.64.0 | `Ran 336 tests in 217.312s` — `OK`, `skipped=0`, exit 0 |
| 3.14.0 | 1.64.0 | `Ran 284 tests in 213.870s` — `OK`, exit 0 |
| 3.11.15 | 1.64.0 | `Ran 284 tests in 212.034s` — `OK`, exit 0 |

If you add tests, re-run the discovery on every row you want to keep quoting and update the counts. A stale count in this table is worse than no table: it is the kind of second, quietly-drifting description of coverage that the rest of this section exists to prevent.

CI pins `3.14`, not `3.14.0`: `actions/setup-python` resolves a partial version to the newest release in that series, so a runner is on whatever 3.14.x is current. The second row is a 3.14.0 measurement, and the exact patch release a runner lands on is the one thing in this table CI does not pin — which is why the count and skip assertions below exist rather than a version assertion.

If you change the pin, re-run the discovery on the new version before trusting it. A version error that raises is loud and self-correcting; the dangerous direction is one that turns into a skip, which is why the CI step asserts on the executed-test count and on the skip count rather than on the exit code.

`tests/sil_companion_bridge/` additionally needs `pyzmq`, `protobuf`, and sibling `X19-Core` and `X19-Surface` checkouts, so run it from the multi-repository workspace rather than from this repository alone.

Continuous integration runs the same discovery with `streamlit` installed, and that step is **blocking**: it is the only automated path over these suites, and it runs on every push and pull request. The suite takes roughly 3.5 minutes on a developer machine, most of it the unavoidable 3000 ms simulated arming gate that several tests must pay against a fresh engine of their own. That cost is acceptable here — the same job already cross-compiles every node and runs CTest — and no subset is carved out, because a maintained allowlist would silently drop checks from CI, which is the failure class this flow exists to prevent.

### 7.7 A pass and a skip are not the same thing

Every server-backed test skips cleanly, per test, when the native engine cannot be found — and **`python -m unittest` still exits 0 on a skip**, so an all-skipped run is indistinguishable from green by exit code alone. A skip means "this was not verified here", never "this passed".

To tell a real pass from a silent skip:

- Run with `-v`. Each skip prints its reason, so the log names exactly what was not verified.
- Read the trailing summary. A bare `OK` means everything ran; `OK (skipped=N)` means N checks were not verified.
- Check that the count is non-trivial. A real run is 336 `ok` lines; a run where every line says `skipped` is a skip, not a pass.
- Remember that one missing dependency can cost a whole module. `test_dashboard_app.py` skips all 55 of its tests at once when `streamlit` is absent, and reports that as a single skip, so `OK (skipped=1)` does not mean one test was skipped.
- Confirm the engine was found. Point `X19_SIL_SERVER` at the binary (7.5) instead of relying on the default search, so a renamed or relocated build tree cannot silently convert the suite into skips.

Continuous integration applies the same rule, and enforces it rather than only reporting it. The step prints its `ok` and `skipped` counts and then asserts on both: it fails when fewer than **250** tests reported `ok`, and it fails on **any** skip at all. The floor catches the catastrophic cases (a total skip reads `ok=0`; a whole-module skip reads `ok=229`); the zero-skip rule catches the partial cases no floor can, such as a future release renaming `streamlit.testing.v1`, where only the 7 AppTest tests skip and the count still reads a comfortable 277.

### 7.8 Host SIL results are not target readiness

These suites verify the **host** simulation: application logic, CAN protocol serialization, the deadman contract, and the dashboard and stimulus tooling, all against mocked peripherals. They do not build or validate STM32 startup code, vendor HAL integration, peripheral timing, pin configuration, FDCAN, or electrical behavior.

Target readiness is a separate, currently **red** CTest group:

```powershell
ctest --test-dir build-native -R "^target_" --output-on-failure
```

Five of the six fail today, and each fails for its own reason, so read the one you are chasing rather than assuming a shared cause:

| Test | Fails because |
| :--- | :--- |
| `target_startup_node1` | `HAL_Init`, `SystemClock_Config`, `MX_GPIO_Init`, `MX_FDCAN1_Init`, and `MX_I2C1_Init` never run before `app_main` (5 failures) |
| `target_startup_node3` | the same five startup calls (5 failures) |
| `target_bsp_node1` | no CAN controller in `bsp_init`, leak probes 0 and 1 not configured as pulled-up inputs, emergency cutoff not configured as an output (4 failures). **No solenoid assertion at all** — this is the Pi Shield, which has none. |
| `target_bsp_node2` | no CAN controller, solenoid outputs not initialized as GPIO outputs, **nine of the ten** solenoid bits do not drive their own output, the emergency brake does not assert the hardware cutoff and does not expose the latch (13 failures) |
| `target_bsp_node3` | no CAN controller, logic voltage not sourced from INA237 data, PCB temperature not sourced from TMP1075 data, converter enable pins not initialized as outputs, ideal-diode status pin not initialized as a pulled-up input (5 failures). **Neither solenoids nor the emergency brake** — this is the Power Slab, which has neither. |

`target_startup_node2` passes, and it is not the same kind of check as its two siblings. Nodes 1 and 3 are compiled contracts: `tests/CMakeLists.txt` builds `hardware/test_target_startup.c` against the node's real `main.c` under a strict fake HAL. Node 2's is a **text scan** — `cmake -P tests/hardware/check_node2_startup.cmake` reads `nodes/node2_control_board/Core/Src/main.c` and asserts, by `string(FIND)`, that eight calls appear in order (`HAL_Init`, `SystemClock_Config`, `MX_GPIO_Init`, `MX_FDCAN1_Init`, `MX_I2C1_Init`, `MX_TIM1_Init`, `MX_TIM8_Init`, `app_main`). It passes because node 2's `main.c` is fully CubeMX-generated at 553 lines, while the `main.c` of nodes 1 and 3 are 26-line hand-written stubs that call `app_main()` and nothing else. So node 2 passing is weaker evidence than a compiled pass, not a different flavour of the same evidence.

In CI that whole step is deliberately `continue-on-error`. A green host SIL run is not evidence that any of the failing contracts work, and **no physical hardware has been validated**. The remaining blockers are tracked in [`target-integration-blockers.md`](target-integration-blockers.md).
