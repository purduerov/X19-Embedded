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
