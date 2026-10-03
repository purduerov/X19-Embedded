# VS Code embedded development workflow

This workflow coordinates host Software-in-the-Loop (SIL) simulation, ST-Link development bench hardware bring-up, and target vehicle flashing across the X19 embedded monorepo. CMake and `rov.toml` remain the source of truth for all builds. Bench hardware development is prioritized around ST-Link dev boards (Nucleo G474, F411), with USB DFU and SWD provided for custom vehicle PCB bring-up.

## Install once (Prerequisites & Toolchain Setup)

Install CMake 3.22 or newer, Ninja, a native host compiler (for SIL simulation), the GNU Arm Embedded toolchain (`arm-none-eabi-gcc`), and STM32CubeProgrammer:

- **macOS (Apple Silicon & Intel)**:
  ```bash
  brew install cmake ninja arm-none-eabi-gcc
  pip3 install -r requirements.txt
  # Install STM32CubeProgrammer from ST: https://www.st.com/en/development-tools/stm32cubeprog.html
  # (If Gatekeeper blocks installer: sudo xattr -cr ~/Downloads/SetupSTM32CubeProgrammer.app)
  ```
- **Windows (PowerShell)**:
  ```powershell
  choco install cmake ninja gcc-arm-embedded
  pip install -r requirements.txt
  # Install STM32CubeProgrammer from ST (default path: C:\Program Files\STMicroelectronics\...)
  ```
- **Linux (Ubuntu / Debian / WSL2)**:
  ```bash
  sudo apt update && sudo apt install -y cmake ninja-build gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential python3-pip
  pip3 install -r requirements.txt
  sudo usermod -a -G dialout $USER
  ```

In VS Code install the recommended extensions: **STM32CubeIDE for Visual Studio Code**, **CMake Tools**, and **C/C++**. The flash tasks and `rov` CLI automatically search standard installation paths for `STM32_Programmer_CLI`.

Open `X19-Embedded.code-workspace`, not an individual `Core` folder. The workspace keeps all nodes and shared libraries visible together.

## Host SIL: build and run tests

Use **Terminal > Run Task > Build: Host SIL (Debug)** or press `Ctrl+Shift+B`. This configures and builds the host simulation without an STM32 or debug probe. Run native CTest with **Test: Host SIL (Debug)**.

Equivalent commands from this directory:

```powershell
cmake --preset sil-debug
cmake --build --preset sil-debug
ctest --preset sil-debug
```

`sil-release` provides the corresponding release configuration in a separate build directory.

Host SIL targets compile and link natively out-of-the-box. Running `ctest --preset sil-debug` executes the 25 host test suites while excluding unfinished `target_*` acceptance contracts until physical target bring-up is completed.

## Development bench workflow (ST-Link & Nucleo dev boards - Recommended active workflow)

While custom vehicle PCB schematics are undergoing revision (see [`board_findings.md`](board_findings.md)), physical hardware validation is conducted on ST Nucleo development boards over **ST-Link SWD**.

### Declarative multi-target CLI (`rov.toml` & `rov.py`)

The embedded workspace includes a unified declarative CLI (`tools/rov.py`, accessible from the repo root as `python rov.py` or `./rov` / `.\rov`). It automatically correlates connected ST-Link hardware probe serial numbers to their corresponding CDC Virtual COM Ports, providing an automated build, flash, and live monitor cycle:

```bash
# Build active node and board configured in rov.toml:
python rov.py build

# Build a specific node for a bench development board:
python rov.py build -n pi_shield -b f411
python rov.py build -n control_board -b g474

# Build, flash via ST-Link SWD, and auto-open live serial monitor:
python rov.py run

# Rapid Prototyping / Developer Sandbox (Single-File Testing):
# Test code in sandbox/sandbox.c or pass any custom C file with app_main():
python rov.py sandbox                      # Run default sandbox/sandbox.c on default board (f411)
python rov.py sandbox -b f411              # Run default sandbox on NUCLEO-F411RE
python rov.py sandbox -b g474              # Run default sandbox on NUCLEO-G474RE
python rov.py sandbox -b host              # Run sandbox natively in Host SIL simulator (no hardware)
python rov.py sandbox -f my_experiment.c -b f411  # Run custom scratch C file on F411

# Hardware I2C Diagnostic Bus Scanner (Builds, flashes, and streams live ASCII table):
python rov.py scan -b f411                 # Zero-code I2C bus scanner on NUCLEO-F411RE (Pins PB8/PB9)
python rov.py scan -b g474                 # Zero-code I2C bus scanner on NUCLEO-G474RE (Pins PA15/PB7)

# Auto-detect connected ST-Link COM port and stream serial output at 115200 baud:
python rov.py monitor

# Enumerate connected ST-Link probes and serial COM ports:
python rov.py devices

# Run host SIL simulation test suite:
python rov.py test

# Optional: Install as an editable package to use 'rov' directly in any shell:
pip install -e .
rov sandbox -b f411
```

### Monorepo Node & Board Target Mapping

| Node Name (`-n`) | Target Board (`-b`) | Underlying CMake Target | Generated ELF Binary Path | Primary Source Files |
| :--- | :--- | :--- | :--- | :--- |
| **`pi_shield`** (Node 1) | `f411` | `bench_node1_pi_shield.elf` | `build/arm-c542-debug/nodes/testing_and_rnd/Projects/bench_node1_pi_shield.elf` | `nodes/node1_pi_shield/Core/Src/app.c`, `testing_and_rnd/Src/bsp.c` |
| **`pi_shield`** (Node 1) | `stm32c5` | `node1_pi_shield.elf` | `build/arm-c542-debug/nodes/node1_pi_shield/node1_pi_shield.elf` | `nodes/node1_pi_shield/Core/Src/app.c`, `bsp.c`, `main.c` |
| **`control_board`** (Node 2) | `g474` | `node2_control_board.elf` | `nodes/node2_control_board/build/Debug/node2_control_board.elf` | `nodes/node2_control_board/Core/Src/app.c`, `bsp.c`, `main.c` |
| **`control_board`** (Node 2) | `f411` | `bench_node2_control_board.elf` | `build/arm-c542-debug/nodes/testing_and_rnd/Projects/bench_node2_control_board.elf` | `nodes/node2_control_board/Core/Src/app.c`, `testing_and_rnd/Src/bsp.c` |
| **`control_board`** (Node 2) | `stm32c5` | `node2_control_board.elf` | `build/arm-c542-debug/nodes/node2_control_board/node2_control_board.elf` | `nodes/node2_control_board/Core/Src/app.c`, `bsp.c`, `main.c` |
| **`power_slab`** (Node 3) | `f411` | `bench_node3_power_slab.elf` | `build/arm-c542-debug/nodes/testing_and_rnd/Projects/bench_node3_power_slab.elf` | `nodes/node3_power_slab/Core/Src/app.c`, `power_sequence.c` |
| **`power_slab`** (Node 3) | `stm32c5` | `node3_power_slab.elf` | `build/arm-c542-debug/nodes/node3_power_slab/node3_power_slab.elf` | `nodes/node3_power_slab/Core/Src/app.c`, `power_sequence.c`, `bsp.c` |
| **`rnd`** (I2C Scan) | `f411` / `f446` | `testing_and_rnd.elf` | `build/arm-c542-debug/nodes/testing_and_rnd/Projects/testing_and_rnd.elf` | `nodes/testing_and_rnd/Src/app.c` (`bsp_i2c_scan`), `bsp.c` |
| **`rnd`** (I2C Scan) | `g474` | `node2_i2c_scanner.elf` | `nodes/node2_control_board/build/Debug/node2_i2c_scanner.elf` | `nodes/node2_control_board/Core/Src/diagnostic_i2c_scan.c` |
| **`sandbox`** | `f411` | `bench_sandbox.elf` | `build/arm-c542-debug/nodes/testing_and_rnd/Projects/bench_sandbox.elf` | `sandbox/sandbox.c` (or `-f <file>`), `testing_and_rnd/Src/bsp.c` |
| **`sandbox`** | `g474` | `sandbox.elf` | `nodes/node2_control_board/build/Debug/sandbox.elf` | `sandbox/sandbox.c` (or `-f <file>`), `node2_control_board/Core/Src/bsp.c` |
| **`sandbox`** | `host` | `sil_sandbox` | `build/sil-debug/tests/sil_sandbox.exe` | `sandbox/sandbox.c` (or `-f <file>`), `tests/sil_sandbox_main.c` |

### VS Code bench tasks

The `.vscode/tasks.json` configuration wires these workflows directly into the editor:
- **Default Build Task (`Ctrl+Shift+B`)**: Runs `ROV: Run (Build, Flash, & Monitor)`.
- **Node 2 Nucleo G474 Bench**:
  - `Build: Node 2 Nucleo G474 (Debug)`: Compiles the bench firmware in `nodes/node2_control_board/` using preset `Debug`.
  - `Flash: Node 2 Nucleo G474 Bench (ST-Link)`: Flashes the bench ELF using the Nucleo's onboard ST-Link over SWD.
- **Testing & R&D Sandbox F446 Dev Board**:
  - `Build: Testing & R&D F446 Dev Board`: Cross-compiles `nodes/testing_and_rnd` for bench testing.

---

## Custom vehicle PCB flashing (USB DFU & ST-Link SWD - Vehicle integration)

Production vehicle nodes standardize on the **STM32C542CCT6** (LQFP48). Vehicle firmware targets can be programmed either via their SWD debug header or through the STM32 factory system-memory USB DFU bootloader once physical boards route USB data lines.

### Flashing tasks in VS Code

For vehicle targets, choose **Terminal > Run Task** and select one of:
- **ST-Link SWD**:
  - `Flash: Node 1 Pi Shield (ST-Link SWD)`
  - `Flash: Node 2 Vehicle Control Board (ST-Link SWD)`
  - `Flash: Node 3 Power Slab (ST-Link SWD)`
- **USB DFU (Factory System Bootloader)**:
  - `Flash: Node 1 Pi Shield (STM32C542 USB DFU)`
  - `Flash: Node 2 Vehicle Control Board (STM32C542 USB DFU)`
  - `Flash: Node 3 Power Slab (STM32C542 USB DFU)`

Each task invokes `tools/flash_firmware.py`, connecting via `SWD` or `USB1`, programming, verifying, and resetting the target MCU.

### Hardware caveats & per-node DFU status

As documented in [`board_findings.md`](board_findings.md), USB DFU is an aspirational vehicle-level target and cannot run on current board revisions:

| Vehicle Node | Target MCU | USB DFU Status | Hardware Reality Check |
|---|---|---|---|
| **Node 1 Pi Shield** | STM32C542 | ROM supports DFU if PA11/PA12 wired | Pending C542 pinout definition (#138). Missing generated startup/HAL. |
| **Node 2 Control Board** | STM32C542 | Blocked on hardware schematic | The Control Board schematic explicitly labels its USB-C connector as unused (data lines unconnected). Must use ST-Link dev board or SWD header. |
| **Node 3 Power Slab** | STM32C542 | Pin collision on hardware schematic | Pins PA11/PA12 (USB D+/D-) are assigned to I2C1 in `.ioc`. Pending pinout review. |


## Why this workflow

CMake and `rov.toml` remain the single source of truth for all builds across host SIL simulation and cross-compiled ARM hardware targets. Desktop STM32CubeIDE remains an optional fallback.

References:

- [STM32CubeIDE for VS Code extension](https://marketplace.visualstudio.com/items?itemName=stmicroelectronics.stm32-vscode-extension)
- [ST VS Code extension commands and project discovery](https://dev.st.com/stm32cube-docs/stm32cubeide-vscode/latest/en/docs/markup/workspace_and_extension_workflow/extension_commands.html)
- [ST multi-project workspaces](https://dev.st.com/stm32cube-docs/stm32cubeide-vscode/latest/en/docs/markup/workspace_and_extension_workflow/multi_project_workspaces.html)
- [STM32CubeProgrammer GUI programming workflow](https://dev.st.com/stm32cube-docs/prog/2.23.0/en/docs/markup/CubeProg_UserManual/Memory_programming_and_erasing.html)
- [STM32CubeProgrammer CLI and USB DFU connection](https://dev.st.com/stm32cube-docs/prog/2.23.0/en/docs/markup/CubeProg_Command_Lines.html)
- [STM32C5 reference manual](https://www.st.com/resource/en/reference_manual/rm0522-stm32c5-series-armbased-32bit-mcus-stmicroelectronics.pdf)
- [AN2606: STM32 system-memory boot mode, including STM32C531/532/542](https://www.st.com/resource/en/application_note/an2606-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf)
