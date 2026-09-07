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

Developers can compile and execute 100% of the vehicle's firmware logic natively on their host machines (Linux, Windows, macOS) without needing any physical microcontrollers, sensors, or CAN transceivers.

### Running the Test Suite Locally

```powershell
# Configure host build tree
cmake -B build -G Ninja

# Build all mock libraries, drivers, node applications, and unit tests
cmake --build build

# Execute all 14 test suites (<1 second total runtime)
ctest --test-dir build --output-on-failure
```

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

### Test Coverage Summary (14 Test Suites)

| Test Executable | Target Layer | Verification Scope |
| :--- | :--- | :--- |
| `test_can_protocol` | Shared Core | Zero-copy byte serialization and bit fidelity for all CAN arbitration IDs (`0x100`, `0x110`, `0x200`, `0x210`, `0x300`). |
| `test_pwm_ramp` | Shared Math | 1 kHz slew-rate limiter ($2\,\mu\text{s/ms}$), parametric cubic exponential curve ($a=0.65$), bounds clamping. |
| `test_safety` | Shared Safety | Watchdog timer expiration, heartbeat loss tracking ($>100\,\text{ms}$), emergency break latch. |
| `test_i2c_recovery` | Shared Recovery | Automated 9-clock bus-clearing sequence for unsticking hung I2C slave devices. |
| `test_driver_bme280` | Driver Layer | Bosch BME280 enclosure pressure, humidity, and temperature acquisition with NULL guards. |
| `test_driver_ms5837` | Driver Layer | Hydrostatic depth conversion from absolute millibar pressure ($h = \Delta P / (\rho g)$). |
| `test_driver_ina226` | Driver Layer | Bus voltage, shunt current, and wattage calculation ($P = V \times I$). |
| `test_driver_tcan1044` | Driver Layer | TI TCAN1044 high-speed CAN FD transceiver state machine and standby mode control. |
| `test_driver_tps25990` | Driver Layer | TI TPS25990 PMBus converter brick telemetry and status reporting. |
| `test_driver_lsm6dsoxtr`| Driver Layer | 6-axis IMU angular rate acquisition and Madgwick quaternion normalization. |
| `test_node1_pi_shield` | Node 1 App | Sealed enclosure vacuum decay, humidity spike ($>80\%$), floor leak probe contact, 10 Hz telemetry, instant `0x001` E-Stop broadcast. |
| `test_node2_control_board`| Node 2 App | 8x PWM slew ramping, 100 ms heartbeat timeout failsafe, `0x001` emergency break cutoff, 10-ch solenoids, 100 Hz nav telemetry stream. |
| `test_node3_power_slab` | Node 3 App | 5-brick PMBus telemetry, 48V tether monitoring, 25A overcurrent trip, 85 C overtemp protection, `0x005` eFuse Fault Alert broadcast. |
| `test_multi_node_bus` | Full Vehicle Stack | Concurrent multi-node simulation (Pi Core + Node 1 + Node 2 + Node 3) verifying pilot control, depth changes, leak event, and instant distributed cutoff. |

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
