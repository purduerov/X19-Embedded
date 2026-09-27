# X19 Embedded Subsystem Guidelines

The `Embedded/X19-Embedded/` repository contains the C firmware, shared protocols, safety monitors, drivers, and host Software-in-the-Loop (SIL) test suite for the X19-ROV vehicle network.

## Hardware & Architecture Overview

- **Standardized Vehicle Target MCU**: STM32C542CCT6 (48-pin LQFP48, Cortex-M33 @ 144 MHz with single-precision hardware FPU, Arm TrustZone, 256 KB SRAM, 1 MB Flash, dual native FDCAN).
  - Node 1: Pi Shield (RPi 5 HAT, leak detection, chamber environment)
  - Node 2: Control Board (Thruster PWM, SMC pneumatics, BMI270 IMU, MS5837 depth)
  - Node 3: Power Slab (48V to 12V/5V distribution, INA237 current monitoring, PMBus, TPS25990 eFuses)
- **Bench Evaluation & Testing Boards**:
  - **STM32F446RE (NUCLEO-F446RE / Testing & R&D Sandbox)**: Located under `nodes/testing_and_rnd/`. Standalone sandbox for physical CAN bus ping-pong with the Raspberry Pi, latency profiling, and bench testing.
  - **STM32G474RE (NUCLEO-G474RE Nucleo-64)**: Standalone bench project under `nodes/node2_control_board/` for Node 2 firmware bench testing.
  - **STM32F411RE (NUCLEO-F411RE)**: Supported for bench peripheral testing via generic ST-Link debug and flash configurations.
- **Physical Bus**: CAN FD (1.0 Mbps nominal / 5.0 Mbps data phase with Bit Rate Switch enabled) via TI TCAN1044 transceivers.

## Toolchain & Prerequisites

Run the automated diagnostic tool to verify all required compilers, build tools, and flash utilities:

```powershell
python tools/check_prereqs.py
```

### Required Tools
- **CMake**: >= 3.22 (System install on PATH)
- **Ninja**: >= 1.10 (System install on PATH)
- **Host Compiler**: GCC, Clang, or MSVC (C11 support for host SIL tests)
- **ARM Cross-Compiler**: GNU Arm Embedded Toolchain 13.x+ (`arm-none-eabi-gcc` on PATH)
  - Windows: `winget install Arm.GnuArmEmbeddedToolchain`
  - Linux: `sudo apt install gcc-arm-none-eabi`
  - macOS: `brew install --cask gcc-arm-embedded`
- **Flashing Utility**: STM32CubeCLT or STM32CubeProgrammer (`STM32_Programmer_CLI`)
  - Official ST Command Line Toolset (bundles programmer and debug tools): [STM32CubeCLT on st.com](https://www.st.com/en/development-tools/stm32cubeclt.html)
  - Or standalone programmer: [STM32CubeProgrammer on st.com](https://www.st.com/en/development-tools/stm32cubeprog.html)

## VS Code Development Environment

### Recommended Extensions
- `ms-vscode.cmake-tools` (CMake Tools integration)
- `ms-vscode.cpptools` (C/C++ IntelliSense)
- `marus25.cortex-debug` (Hardware debugging via ST-Link/J-Link)
- `twxs.cmake` (CMake syntax highlighting)

> [!WARNING]
> Do NOT install the official `stmicroelectronics.stm32-vscode-extension`. It misidentifies internal CMake fragment directories as standalone projects and overrides build directories.

### Workspace Setup
Open `Embedded/X19-Embedded/` as the root workspace folder, or open `X19-Embedded.code-workspace`. Configuration is driven cleanly by `CMakePresets.json`.

## Build, Test, and Verification Workflows

### 1. Host Software-in-the-Loop (SIL) Simulation
Builds and runs the entire vehicle logic, virtual bus, and 25 unit/integration test suites natively without requiring physical hardware:

- **VS Code**: Press `Ctrl+Shift+B` -> Select `Build: Host SIL (Debug)`. Run tests via task `Test: Host SIL (Debug)`.
- **CLI (from `Embedded/X19-Embedded/`)**:
  ```powershell
  cmake --preset sil-debug
  cmake --build --preset sil-debug
  ctest --preset sil-debug
  ```

### 2. STM32C542 Vehicle Nodes Cross-Compilation
Compiles Node 1 (Pi Shield) and Node 3 (Power Slab) using `arm-none-eabi-gcc`:

- **VS Code**: Select preset `arm-c542-debug` from the CMake Tools status bar, or run task `Build: ARM Nodes (C542)`. Output binaries are placed into `build/arm-c542-debug/`.
- **CLI (from `Embedded/X19-Embedded/`)**:
  ```powershell
  cmake --preset arm-c542-debug
  cmake --build --preset arm-c542-debug
  ```

### 3. Node 2 Nucleo Bench Target (STM32G474)
Compiles the standalone Nucleo bench project:

- **VS Code**: Run task `Build: Node 2 Nucleo G474 (Debug)`.
- **CLI (from `Embedded/X19-Embedded/nodes/node2_control_board`)**:
  ```powershell
  cmake --preset Debug
  cmake --build --preset Debug
  ```

### 4. Declarative Developer CLI (`rov` & `rov.toml`)
Provides a declarative, PlatformIO-like developer workflow across heterogeneous development boards (NUCLEO-F411, NUCLEO-G474) and production STM32C542 targets:

- **Configuration File**: [`rov.toml`](rov.toml) defines active nodes, default dev boards, and hardware mappings.
- **Unified Commands (run from `Embedded/X19-Embedded/` via `.\rov` or `python tools/rov.py`)**:
  ```powershell
  # Build target binary (defaults to active node/board in rov.toml)
  .\rov build
  .\rov build -n pi_shield -b f411
  .\rov build -n control_board -b f411
  .\rov build -n pi_shield -b stm32c5

  # Build, flash with hardware reset, auto-detect ST-Link COM port, and open live serial monitor:
  .\rov run
  .\rov run -n pi_shield -b f411

  # Auto-detect connected ST-Link COM port and open serial monitor:
  .\rov monitor

  # List connected ST-Link hardware probes and COM ports:
  .\rov devices

  # Build and run all 25 Host SIL CTest test suites:
  .\rov test
  .\rov test -R "safety"
  ```
- **VS Code One-Key Execution**: Press `Ctrl+Shift+B` to trigger task `ROV: Run (Build, Flash, & Monitor)`.

## Flashing Targets

### ST-Link SWD Flashing
Connect an ST-LINK programmer to the target's SWD header (SWDIO, SWCLK, GND, 3V3) or plug in a Nucleo USB cable:
- **VS Code**: Run task:
  - `Flash: Node 1 Pi Shield (ST-Link SWD)`
  - `Flash: Node 2 Nucleo G474 Bench (ST-Link)`
  - `Flash: Node 3 Power Slab (ST-Link SWD)`
  - `Flash: Testing & R&D F446 Dev Board (ST-Link)`
  - `Flash: Nucleo F411RE Dev Board (ST-Link)`
  - `Flash: Any Firmware File (choose path and port)`

### USB DFU Flashing
Connect the board's USB-C device port to the PC with BOOT0 pulled high:
- **VS Code**: Run task:
  - `Flash: Node 1 Pi Shield (STM32C542 USB DFU)`
  - `Flash: Node 3 Power Slab (STM32C542 USB DFU)`

### Remote CAN Bootloader Flashing
Flash nodes remotely over the vehicle CAN bus using the companion computer:
```bash
python bootloader/tools/can_flash.py --channel can0 --node-id 0x01 firmware.bin
```

## Debugging

Debug configurations are pre-wired in `.vscode/launch.json`:

1. **Physical Hardware Debugging (`marus25.cortex-debug`)**:
   - `Debug: Testing & R&D F446 (dev board, ST-Link)`
   - `Debug: Nucleo F411RE (dev board, ST-Link)`
   - `Debug: Node 2 Nucleo G474 (dev board, ST-Link)`
   - `Debug: Node 1 Pi Shield (ST-Link SWD)`
   - `Debug: Node 3 Power Slab (ST-Link SWD)`
   - `Debug: Any STM32 Target (dev board / generic ST-Link)`
   - Requires ST-Link probe or Nucleo board connected. Automatically builds target before launching, connects GDB, and halts at `main`.
2. **Host SIL Debugging (`cppdbg`)**:
   - `Debug: Host SIL Test (Node 2 Control Board)`
   - `Debug: Host SIL Bridge Server`
   - Attaches host GDB to `sil_bridge_server.exe` for stepping through simulation loops.

## Critical Safety & Architectural Rules

- **CubeMX Trampoline**: STM32CubeMX generated `main.c` must ONLY contain a call to `app_main()` placed strictly inside `/* USER CODE BEGIN 2 */`. Application logic lives exclusively in `Core/Src/app.c`.
- **Zero ST HAL in Application Code**: Application routines must NEVER call `HAL_*` functions directly. Use the hardware abstractions in `shared/include/bsp.h` and `shared/include/can_interface.h`.
- **Fail-Safe Invariants**:
  - Thruster neutral PWM is strictly 1500 us.
  - Heartbeat loss (>250ms), water leak, or E-stop command immediately disarms all actuator outputs.
  - No `HAL_Delay()` inside polling routines or timer callbacks.
  - No dynamic memory allocation (`malloc`, `free`) in vehicle firmware.
- **Clang-Format**: All C/C++ code must be formatted with `.clang-format` before committing.
- **No Emojis**: Do not use emojis in commit messages, code comments, or documentation.
