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
