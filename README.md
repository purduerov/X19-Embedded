# ROV-Embedded Firmware Repository

> **Purdue ROV — Modular Subsea Microcontroller Firmware Platform (Configured for X19 Subsea Vehicle)**  
> *Standardized across 100% of nodes on STM32C542CCT6 (Cortex-M33 @ 144 MHz with single-precision FPU, 2x FDCAN) running CAN FD @ 1 Mbps / 5 Mbps*

---

## 1. Subsystem Architecture Overview

| Node Name | Directory | Target MCU | Primary Responsibilities |
| :--- | :--- | :--- | :--- |
| **Node 1: Pi Shield** | `nodes/node1_pi_shield/` | STM32C542CCT6 (48-Pin) | BME280 leak & vacuum testing, INA237/226 5V monitor, GPIO leak traces, 10 Hz Leak stream (`0x210`), autonomous E-Stop (`0x001`). |
| **Node 2: Control Board** | `nodes/node2_control_board/` | STM32C542CCT6 (48-Pin) | 8x Thruster PWM (`TIM1`/`TIM8`), 1 kHz ramping (`TIM6`), 10-ch SMC solenoids, BMI270/LSM6DSOXTR IMU (SPI), MS5837 Depth (I2C), 100 Hz Nav stream (`0x200`). |
| **Node 3: Power Slab** | `nodes/node3_power_slab/` | STM32C542CCT6 (48-Pin) | PMBus I2C to 5 converter bricks (4x 12V 300W + 1x 5.2V 50W), PCB copper thermal ADC, LM74700 ideal diode status, 20 Hz Power stream (`0x300`), 0x005 eFuse Fault Alert. |
| **PCIe USB Camera Hub** | N/A (Hardware Only) | Renesas UPD720201 + 4x USB2512 | Pure hardware PCIe-to-USB 3.0/2.0 hub powering 8x ExploreHD cameras. No standalone microcontroller or CAN interface. |
| **CAN Bootloader** | `bootloader/` | STM32C542CCT6 (Sector 0) | High-speed underwater firmware updating over CAN FD (5 Mbps data phase). |
| **Testing & R&D Sandbox** | `nodes/testing_and_rnd/` | STM32C542 (Dev/Nucleo) | Standalone STM32 + Raspberry Pi bench testing, bidirectional CAN FD ping-pong, and latency verification. |

---

## 2. Software-in-the-Loop (SIL) Host Testing (Zero-Hardware Simulation)

Developers can compile and execute the shared application logic, protocol code, and host-testable drivers natively on Linux, Windows, or macOS. SIL substitutes mock CAN, BSP, and sensor interfaces for the target peripherals. It does not build or validate STM32 startup code, vendor HAL integration, peripheral timing, or electrical behavior; use the cross-compile and hardware bench checks for those layers.

### Running the Test Suite Locally

```powershell
# Configure host build tree
cmake -B build -G Ninja

# Build all mock libraries, drivers, node applications, and unit tests
cmake --build build

# Run the host-side logic, protocol, and driver regression suite
ctest --test-dir build -E "^target_" --output-on-failure

# Run target startup and board-I/O readiness checks (expected to fail until implemented)
ctest --test-dir build -R "^target_" --output-on-failure
```

The `target_*` acceptance checks compile each node's real `main.c` and `bsp.c` against a strict fake HAL. They check startup peripheral initialization, GPIO setup, CAN bring-up, actuator outputs, and live power-sensor values. These checks are intentionally red while target firmware integration is incomplete; their assertion output identifies the missing contract. The 25 host-side CTest executables exercise application logic with mocked hardware and are not evidence that target firmware is ready.

The host SIL now rejects solenoid commands that energize both coils of a double-acting valve and verifies that an E-stop clears pneumatics and prevents later commands from re-energizing them. The deterministic frame fuzzer also checks that oversized frames are rejected and records accepted input volume.

### SIL Limits and Next Coverage

- The CAN mock models bounded receive FIFOs, broadcast, bus-off, TX failure, and frame drops. It does not model bit timing, arbitration latency, ACK errors, retransmission, or hardware error confinement; add these before using SIL to make bus timing claims.
- Mock physics and sensor injection check application responses to chosen values. They do not model hull leakage, real pressure transients, electrical brownouts, EMI, or analog sensor noise. Add calibrated plant scenarios as hardware data becomes available.
- Fake-HAL target checks verify that firmware requests expected startup and I/O operations. They do not emulate STM32 registers, Cube HAL state, interrupts, or pin electrical behavior; bench tests remain necessary.
- Add command sequence/property tests for heartbeat expiry while pneumatics are active, CAN RX FIFO saturation, sensor disconnect/stale samples, timer wraparound, and recovery after brownout. Record expected safety behavior for each scenario before treating it as a pass criterion.

The Python Pi Core bridge test is separate from CTest. Run it after building the host bridge server and installing the Core Python dependencies:

```powershell
python tests/sil_companion_bridge/test_full_system_sil.py
```

When the embedded repository is opened from a separate worktree, the bridge test locates the sibling repositories automatically. Set `X19_WORKSPACE_ROOT` to the multi-repository workspace root, or set `X19_CORE_DIR` and `X19_SURFACE_DIR` explicitly if the repositories use another layout.

### Python SIL Verification in CI

The `build_and_lint.yml` workflow runs the Python SIL surface as a **blocking** step. After the native build it points `X19_SIL_SERVER` at the freshly built `sil_bridge_server` and runs the dashboard, stimulus-CLI, protocol round-trip, and per-node integration suites under `tests/sil_stimulus/`, preceded by a syntax gate over the dashboard, companion-bridge, and stimulus sources. `streamlit` is the only third-party package those suites need. The companion-bridge test needs `pyzmq`, `protobuf`, and the sibling `X19-Core`/`X19-Surface` checkouts, so it is executed from the multi-repository workspace instead.

Every server-backed test skips cleanly when the native engine is missing, and `python -m unittest` still exits 0 on a skip, so the CI step does not trust the exit code alone: it counts the results and fails when fewer than 250 tests reported `ok` or when anything skipped at all.

Host SIL results and target readiness are different things. A green run above is evidence about application logic, CAN protocol serialization, the 20 Hz deadman contract, and the dashboard and stimulus tooling, all against mocked peripherals. It is not evidence about STM32 startup, vendor HAL integration, peripheral timing, pin configuration, FDCAN, or electrical behavior, and no physical hardware has been validated. Target readiness is the separate `continue-on-error` step, where five of the six acceptance contracts fail today, each for its own reason: nodes 1 and 3 for the missing CubeMX startup and peripheral-init calls, node 1's BSP for its leak-probe inputs and emergency-cutoff output, node 2's for the solenoid outputs and the emergency-brake cutoff and latch, and node 3's for INA237 and TMP1075 sourcing, the converter enable pins, and the ideal-diode status pin.

[`docs/sil_simulation_guide.md`](docs/sil_simulation_guide.md) documents the working commands, the deadman and arming semantics, the exit-status contract, and how to tell a real pass from a silent skip.

Target-only startup, pin, FDCAN, and hardware sensor integration blockers are tracked in [`docs/target-integration-blockers.md`](docs/target-integration-blockers.md).

### Simulation Architecture (`tests/mocks/`)

1. **Virtual Multi-Node CAN FD Bus (`mock_can.h` / `mock_can.c`)**:
   - In-memory CAN FD broadcast matrix connecting Node 1, Node 2, Node 3, and Pi Core simultaneously.
   - Per-node hardware RX FIFO queues with priority arbitration.
   - Fault injection: supports simulating Bus-Off states, mailbox exhaustion, dropped frames, and packet corruption.
2. **Virtual Board Support Package (`mock_bsp.h` / `mock_bsp.c`)**:
   - Discrete virtual time clock (`mock_bsp_advance_time_ms`) for deterministic stepping without sleep delays.
   - 8-channel ESC PWM monitoring (`mock_bsp_get_pwm_us`) to verify 1 kHz slew-rate ramping and failsafe neutrality.
   - 10-channel pneumatic solenoid bitmask monitoring (`mock_bsp_get_solenoid_mask`).
   - Floor leak probe contact injection (`mock_bsp_set_leak_probe`).
   - Hardware Emergency Brake trip tracking (`mock_bsp_is_emergency_brake_tripped`).
3. **Synthetic Sensor & Driver Injection (`mock_sensors.h` / `mock_sensors.c`)**:
   - Inject synthetic physics into Bosch BME280, TE MS5837-30BA, TI INA226/237, ST/Bosch 6-axis IMUs, and TI TPS25990 PMBus bricks.
   - Verifies driver conversions (hydrostatic depth formula, power calculations, quaternion normalization) against exact mathematical baselines.

### Test Coverage Summary (25 CTest Executables)

| Test Executable | Target Layer | Verification Scope |
| :--- | :--- | :--- |
| `test_can_protocol` | Shared Core | Zero-copy byte serialization and bit fidelity for all CAN arbitration IDs (`0x100`, `0x110`, `0x200`, `0x210`, `0x300`). |
| `test_pwm_ramp` | Shared Math | 1 kHz slew-rate limiter ($2\,\mu\text{s/ms}$), parametric cubic exponential curve ($a=0.65$), bounds clamping. |
| `test_safety` | Shared Safety | Watchdog timer expiration, heartbeat loss tracking ($>100\,\text{ms}$), emergency break latch. |
| `test_i2c_recovery` | Shared Recovery | Automated 9-clock bus-clearing sequence for unsticking hung I2C slave devices. |
| `test_timesync` | Shared Time Sync | Master, request, response processing, and clock-offset behavior. |
| `test_bsp` | Board Support Package | Mock clock, PWM, solenoid, leak probe, and emergency brake behavior. |
| `test_driver_bme280` | Driver Layer | Bosch BME280 enclosure pressure, humidity, and temperature acquisition with NULL guards. |
| `test_driver_ms5837` | Driver Layer | Hydrostatic depth conversion from absolute millibar pressure ($h = \Delta P / (\rho g)$). |
| `test_driver_ina226` | Driver Layer | Bus voltage, shunt current, and wattage calculation ($P = V \times I$). |
| `test_driver_tcan1044` | Driver Layer | TI TCAN1044 high-speed CAN FD transceiver state machine and standby mode control. |
| `test_driver_tps25990` | Driver Layer | TI TPS25990 PMBus converter brick telemetry and status reporting. |
| `test_driver_lsm6dsoxtr`| Driver Layer | 6-axis IMU angular rate acquisition and Madgwick quaternion normalization. |
| `test_driver_bmi270` | Driver Layer | BMI270 initialization and inertial data conversion through mocked sensor registers. |
| `test_driver_ina237` | Driver Layer | INA237 voltage, current, power, and status conversions. |
| `test_driver_tmp1075` | Driver Layer | TMP1075 temperature-register conversion and edge handling. |
| `test_driver_pmbus_brick` | Driver Layer | PMBus brick command, telemetry, and status decoding. |
| `test_mock_physics` | SIL Plant Model | Buoyancy, motion, orientation, thruster load, and sensor synchronization. |
| `test_node1_pi_shield` | Node 1 App | Sealed enclosure vacuum decay, humidity spike ($>80\%$), floor leak probe contact, 10 Hz telemetry, instant `0x001` E-Stop broadcast. |
| `test_node2_control_board`| Node 2 App | 8x PWM slew ramping, 100 ms heartbeat timeout failsafe, `0x001` emergency break cutoff, 10-ch solenoids, 100 Hz nav telemetry stream. |
| `test_node3_power_slab` | Node 3 App | 5-brick PMBus telemetry, 48V tether monitoring, 25A overcurrent trip, 85 C overtemp protection, `0x005` eFuse Fault Alert broadcast. |
| `test_multi_node_bus` | Full Vehicle Stack | Concurrent multi-node simulation (Pi Core + Node 1 + Node 2 + Node 3) verifying pilot control, depth changes, leak event, and instant distributed cutoff. |
| `test_sil_safety` | SIL Safety | Heartbeat watchdog shutdown, emergency-break latency, and solenoid interlocks across nodes. |
| `test_sil_power` | SIL Power | eFuse overcurrent response and power telemetry cadence. |
| `test_sil_fuzz` | SIL Fault Injection | Deterministic malformed-frame input and Control Board bus-off failsafe/recovery. |
| `test_sil_burnin` | SIL Stability | 100,000 neutral cycles and 10,000 alternating command/ramp cycles. |

These 25 native executables are registered with CTest. The Python bridge integration test runs separately from CTest, from the multi-repository workspace rather than from this repository's CI, and can be run locally with `python tests/sil_companion_bridge/test_full_system_sil.py`. It covers ZMQ/Protobuf, simulated pilot commands, telemetry, emergency stop, and solenoid output through the host bridge server.

---

## 3. Communication, Safety & Hardware Abstraction Contracts

- **Master Parameters**: All physical bounds, vehicle power caps (1200W tether, 12.5A thruster cap), timing intervals, and CAN bitrates are strictly defined in [`shared/include/rov_parameters.h`](shared/include/rov_parameters.h).
- **Packet Serialization**: Standard packet packing and unpacking routines are in [`shared/include/rov_can_protocol.h`](shared/include/rov_can_protocol.h).
- **Safety State Machine**: Watchdog tracking and emergency break routines are in [`shared/include/rov_safety.h`](shared/include/rov_safety.h).
- **Unified Hardware Abstraction Layer (HAL Wrapper / BSP)**:
  - Application code (`app.c`, control loops, state machines) **must NOT** invoke low-level vendor HAL functions (`HAL_CAN_...`, `HAL_FDCAN_...`, `HAL_GPIO_...`) directly.
  - All communication must use the shared transport interface defined in [`shared/include/can_interface.h`](shared/include/can_interface.h) (`can_send()`, `can_receive()`, `can_init()`).
  - All board-level utilities must use [`shared/include/bsp.h`](shared/include/bsp.h) (`time_get_ms()`, `delay_ms()`, `bsp_pwm_set_us()`, `bsp_solenoid_set()`, `bsp_leak_probe_read()`, `bsp_emergency_brake_trip()`).
  - STM32CubeMX generated `main.c` must ONLY contain a call to `app_main()` placed strictly inside `/* USER CODE BEGIN 2 */` / `/* USER CODE END 2 */`.

---

## 4. Contributing & Pull Request Rules

- **Zero-Vendor-HAL in Application Code**: PRs introducing direct `HAL_CAN_...` / `HAL_FDCAN_...` / `HAL_GPIO_...` calls inside node application code will be rejected during code review; use `can_interface.h` and `bsp.h`.
- **CI/CD Enforced**: All Pull Requests to `master` must pass automated cross-compilation with zero warnings (`-Wall -Wextra -Werror -Wpedantic`) and formatting checks.
- **Code Style**: Format code before submitting via `clang-format -i` (`.clang-format`).
- **Review Requirement**: All PRs must follow the [PR Template](.github/pull_request_template.md) and receive sign-off from Lead Engineer (Aman).
