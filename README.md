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

## 3. Communication & Safety Protocol Contracts

- **Master Parameters**: All physical bounds, vehicle power caps (1200W tether, 12.5A thruster cap), timing intervals, and CAN bitrates are strictly defined in [`shared/include/x19_parameters.h`](shared/include/x19_parameters.h).
- **Packet Serialization**: Standard packet packing and unpacking routines are in [`shared/include/x19_can_protocol.h`](shared/include/x19_can_protocol.h).
- **Safety State Machine**: Watchdog tracking and emergency break routines are in [`shared/include/x19_safety.h`](shared/include/x19_safety.h).

---

## 4. Contributing & Pull Request Rules

- **CI/CD Enforced**: All Pull Requests to `master` must pass automated cross-compilation with zero warnings (`-Wall -Wextra -Werror`) and style linting.
- **Code Style**: Format code before submitting via `Shift + Alt + F` (`.clang-format`).
- **Review Requirement**: All PRs must follow the [PR Template](.github/pull_request_template.md) and receive sign-off from Lead Engineer (Aman).
