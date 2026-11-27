# X19 Embedded Subsystem Guidelines

The `Embedded/X19-Embedded/` repository contains the C11 firmware, shared protocols, safety monitors, drivers, and host Software-in-the-Loop (SIL) suite for the X19-ROV vehicle network.

## Hardware & Layer Architecture

- **Vehicle Target MCU**: `STM32C542CCT6` (LQFP-48, Cortex-M33 @ 144 MHz, dual FDCAN, 1 MB Flash / 256 KB SRAM).
  - Node 1 (`nodes/node1_pi_shield`): RPi 5 HAT, leak detection, chamber environment.
  - Node 2 (`nodes/node2_control_board`): Thruster PWM, SMC pneumatics, BMI270 IMU, MS5837 depth.
  - Node 3 (`nodes/node3_power_slab`): 48V->12V/5V distribution, INA237, PMBus, TPS25990 eFuses.
- **Bench Targets**: `NUCLEO-F446RE` (`nodes/testing_and_rnd/`), `NUCLEO-G474RE` (`nodes/node2_control_board/`), `NUCLEO-F411RE`.
- **Bus**: CAN FD (1.0 Mbps nominal / 5.0 Mbps data, BRS enabled) via TI TCAN1044.
- **One-Way Dependency Graph**: `app (nodes/*/Core/Src/app.c)` -> `services` -> `drivers` -> `shared`. Platform code (`boards/*/platform`, `bsp.c`, `can_f4.c`) may include `shared` and vendor HAL, never `drivers`/`services`/`app`.

## Core Developer Commands (`python rov.py ...`)

- `python rov.py check`: Verify CMake, Ninja, `arm-none-eabi-gcc`, `STM32_Programmer_CLI`, and git hooks.
- `python rov.py lint [--format-check] [--json]`: Enforce layer boundaries, firmware invariants, hygiene, and formatting.
- `python rov.py test [--node N] [--suite S] [-R REGEX] [-k CASE] [--compact]`: Build & run Host SIL CTest suites (`build-native/`).
- `python rov.py build [target] [-b f411|g474|stm32c5|host]` / `flash` / `run` / `monitor` / `devices`.
- `python rov.py can sniff [--sil localhost:8765 | -i can0]` / `estop` / `pwm <us>`.
- `python rov.py format [--check]`: Run `clang-format`.
- `python rov.py memory <status|search|add|distill|prune>`: Query and maintain institutional memory.

## Critical Safety, Assumptions & Architectural Rules

- **Always State Assumptions Explicitly**: Before acting or alongside every answer, explicitly state any assumptions made (target board/MCU, pinout, build preset, protocol/unit semantics, or ambiguous requirements). Never silently guess hardware or pin assignments; check `docs/board_findings.md` and `docs/lessons.md` first.
- **Zero ST HAL in Application Code**: Never call `HAL_*` / `__HAL_*` / `LL_*` outside platform code; use `shared/include/bsp.h` and `shared/include/can_interface.h`. (Mechanically enforced by `rov.py lint`.)
- **CubeMX Trampoline**: Generated `nodes/*/Core/Src/main.c` must ONLY call `app_main()` inside `/* USER CODE BEGIN 2 */`.
- **Fail-Safe Invariants**: Thruster neutral PWM is strictly `1500 us`; heartbeat loss (>250 ms), water leak, or E-stop immediately disarms all outputs.
- **No Dynamic Allocation or Blocking Delays**: Never use `malloc`/`calloc`/`realloc`/`free` or `HAL_Delay()` in non-platform firmware.
- **Formatting & Hygiene**: Format C/C++ with `.clang-format`; never use emojis in code, comments, or docs; never leave root `.ioc` or `*(1)*` duplicate files.

## Knowledge & On-Demand Skills (Progressive Disclosure)

- **Documentation Index**: [docs/INDEX.md](docs/INDEX.md) (source-of-truth map, hardware errata, specs, and plans).
- **Public Header Symbol Map**: [docs/REPO_MAP.md](docs/REPO_MAP.md) (auto-generated via `python rov.py map`; check before opening multiple headers).
- **Lessons & Decision Log**: [docs/lessons.md](docs/lessons.md) (check with `python rov.py memory search <topic>`).
- **Repo Skills** (`.agents/skills/`):
  - `x19-firmware-workflow`: CMake presets, flashing (ST-Link / DFU / CAN bootloader), and VS Code debugging.
  - `x19-cubemx-and-layers`: Safe `.ioc` regeneration (`rov.py generate`), trampoline, and layer boundaries.
  - `x19-sil-and-i2c-drivers`: Injected `rov_i2c_bus_t` drivers, SIL bridge stimulus, and CTest filtering.
  - `x19-knowledge-capture`: End-of-task reflection, `rov.py memory add/distill/prune`, and invariant promotion.
