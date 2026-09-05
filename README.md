# X19-Embedded Firmware Repository

> **Purdue ROV — 2026-2027 Subsea Microcontroller Firmware Architecture**  
> *Standardized across 100% of nodes on STM32G4 (Cortex-M4 @ 170 MHz with FPU) running CAN FD @ 1 Mbps / 5 Mbps*

---

## 1. Subsystem Architecture Overview

| Node Name | Directory | Target MCU | Primary Responsibilities |
| :--- | :--- | :--- | :--- |
| **Node 1: Pi Shield** | `nodes/node1_pi_shield/` | STM32G431CB (48-Pin) | BME280 leak & vacuum testing, INA226 5V monitor, GPIO leak traces, 10 Hz Leak stream (`0x210`). |
| **Node 2: Control Board** | `nodes/node2_control_board/` | STM32G431CB (48-Pin) | 8x Thruster PWM (`TIM1`/`TIM8`), 1 kHz ramping (`TIM6`), 10-ch SMC solenoids, LSM6DSOXTR IMU (SPI), MS5837 Depth (I2C), 100 Hz Nav stream (`0x200`). |
| **Node 3: Power Slab** | `nodes/node3_power_slab/` | STM32G431CB (48-Pin) | PMBus I2C to 5 converter bricks (4x 12V 300W + 1x 5.2V 50W), PCB copper thermal ADC, LM74700 ideal diode status, 20 Hz Power stream (`0x300`). |
| **Node 4: USB Camera Hub** | `nodes/node4_usb_hub/` | STM32G431CB (48-Pin) | Per-port VBUS voltage/current monitoring, remote camera power cycling via GPIO, 5 Hz USB Hub stream (`0x310`). |
| **CAN Bootloader** | `bootloader/` | STM32G431CB (Sector 0) | High-speed underwater firmware updating over CAN FD (5 Mbps data phase). |
| **Testing & R&D Sandbox** | `nodes/testing_and_rnd/` | STM32G431 (Nucleo/Custom) | Standalone STM32 + Raspberry Pi bench testing, bidirectional CAN FD ping-pong, and latency verification. |

---

## 2. Developer Quickstart & Onboarding

### Prerequisites
1. **VS Code** with the **STM32Cube for VS Code Extension** (`STMicroelectronics.stm32-for-vscode`).
2. **ARM GNU Toolchain** (`arm-none-eabi-gcc`) & **CMake** (3.22+) / **Ninja**.

### Getting Started in 3 Steps
1. **Open Workspace**:
   - In VS Code: **File $\rightarrow$ Open Workspace from File...** $\rightarrow$ select `X19-Embedded.code-workspace`.
2. **Pick Your GitHub Issue**:
   - Go to [GitHub Project Board #7](https://github.com/orgs/purduerov/projects/7) and find your assigned ticket.
   - Create your feature branch: `git checkout -b dev/<feature-name>`.
3. **Build & Flash**:
   - In the VS Code left sidebar, click the **STM32Cube** tab.
   - Select your node $\rightarrow$ Click **Build** (`F7`) or **Flash/Debug** (`F5`).

---

## 3. Communication, Safety & Hardware Abstraction Contracts

- **Master Parameters**: All physical bounds, vehicle power caps (1200W tether, 12.5A thruster cap), timing intervals, and CAN bitrates are strictly defined in [`shared/include/x19_parameters.h`](shared/include/x19_parameters.h).
- **Packet Serialization**: Standard packet packing and unpacking routines are in [`shared/include/x19_can_protocol.h`](shared/include/x19_can_protocol.h).
- **Safety State Machine**: Watchdog tracking and emergency break routines are in [`shared/include/x19_safety.h`](shared/include/x19_safety.h).
- **Unified CAN Hardware Abstraction (HAL Wrapper / BSP)**:
  - Application code (`main.c`, control loops, state machines) **must NOT** invoke low-level vendor HAL functions (`HAL_CAN_...` / `HAL_FDCAN_...`) directly.
  - All communication must use the shared transport interface defined in [`shared/include/can_interface.h`](shared/include/can_interface.h) (`can_send()`, `can_receive()`, `can_init()`).
  - Underlying hardware drivers (`can_g4.c` for G4 FDCAN and `can_f4.c` for F4 bxCAN) encapsulate mailbox handling, FIFO release, and auto-bus-off recovery cleanly away from application logic.

---

## 4. Contributing & Pull Request Rules

- **No Raw HAL in Applications**: PRs introducing direct `HAL_CAN_...` / `HAL_FDCAN_...` calls inside node application directories will be rejected during code review; use `can_interface.h`.
- **CI/CD Enforced**: All Pull Requests to `master` must pass automated cross-compilation with zero warnings (`-Wall -Wextra -Werror`) and style linting.
- **Code Style**: Format code before submitting via `Shift + Alt + F` (`.clang-format`).
- **Review Requirement**: All PRs must follow the [PR Template](.github/pull_request_template.md) and receive sign-off from Lead Engineer (Aman).

---

## 5. Fall 2026 Milestone Deadlines & GitHub Tracking

| Milestone | Target Deadline | Primary Scope & Deliverables | Key Issues |
| :--- | :--- | :--- | :--- |
| **Milestone 1: Core Protocol & Driver Baseline** | **September 16, 2026** | Lead CAN FD Sanity Test (2x Nucleo + Pi 5), shared headers, CAN FD serialization, math library, CI/CD cross-compilation. | [#11](https://github.com/purduerov/X19-Embedded/issues/11), [#12](https://github.com/purduerov/X19-Embedded/issues/12), [#13](https://github.com/purduerov/X19-Embedded/issues/13), [#22](https://github.com/purduerov/X19-Embedded/issues/22), [#29](https://github.com/purduerov/X19-Embedded/issues/29) |
| **Milestone 2: Subsystem Application Firmware** | **October 24, 2026** | Node 1 (Pi Shield), Node 2 (Control Board), Node 3 (Power Slab), and Node 4 (USB Hub) firmware; non-blocking depth state machine; atomic double buffering; PMBus drivers. | [#2](https://github.com/purduerov/X19-Embedded/issues/2), [#3](https://github.com/purduerov/X19-Embedded/issues/3), [#4](https://github.com/purduerov/X19-Embedded/issues/4), [#5](https://github.com/purduerov/X19-Embedded/issues/5), [#16](https://github.com/purduerov/X19-Embedded/issues/16), [#17](https://github.com/purduerov/X19-Embedded/issues/17), [#18](https://github.com/purduerov/X19-Embedded/issues/18), [#19](https://github.com/purduerov/X19-Embedded/issues/19), [#20](https://github.com/purduerov/X19-Embedded/issues/20), [#21](https://github.com/purduerov/X19-Embedded/issues/21) |
| **Milestone 3: CAN Bootloader & Flashing Suite** | **November 20, 2026** | STM32G4 Sector 0 CAN FD custom bootloader firmware, host Python flasher utility (`can_flash.py`), RAM mailbox DFU jump. | [#14](https://github.com/purduerov/X19-Embedded/issues/14), [#15](https://github.com/purduerov/X19-Embedded/issues/15) |
| **Milestone 4: Full-Stack Dry Bench Integration** | **December 12, 2026** | Physical vehicle stack integration inside 4-inch enclosure, 8 thruster spin under load, 5 pneumatic solenoids, 30-min thermal soak, wet leak cutoff validation. | [#10](https://github.com/purduerov/X19-Embedded/issues/10), [#30](https://github.com/purduerov/X19-Embedded/issues/30) |

