---
name: x19-firmware-workflow
description: >-
  Build, flash, and debug X19-Embedded firmware across Host SIL, STM32C542 vehicle nodes,
  and Nucleo bench boards (F411RE, F446RE, G474RE). Use when configuring CMake presets,
  flashing via ST-Link SWD / USB DFU / CAN bootloader, or using VS Code debug tasks.
---

# X19 Firmware Build, Flash & Debug Workflow

## 1. Host Software-in-the-Loop (SIL)

- **CLI**: `python rov.py test` (or `cmake --preset sil-debug && cmake --build --preset sil-debug && ctest --preset sil-debug`).
- **Targeted runs**: `python rov.py test --suite safety`, `python rov.py test --node control_board`, `python rov.py test -R can -k estop --compact`.
- **VS Code**: `Ctrl+Shift+B` -> `Build: Host SIL (Debug)` / `Test: Host SIL (Debug)`.

## 2. STM32C542 Vehicle Nodes Cross-Compilation

- **CLI**: `cmake --preset arm-c542-debug && cmake --build --preset arm-c542-debug` (outputs in `build/arm-c542-debug/`).
- **Note**: Check `docs/target-integration-blockers.md` before enabling `X19_BUILD_NODE1_ROOT` or `X19_BUILD_NODE3_ROOT`.

## 3. Node 2 Nucleo Bench Target (STM32G474)

- **CLI (standalone)**: Run `cmake --preset Debug && cmake --build --preset Debug` inside `nodes/node2_control_board/`.
- **Unified CLI**: `python rov.py build control_board -b g474` or `python rov.py run control_board -b g474`.

## 4. Flashing & Debugging

- **ST-Link SWD**: `python rov.py flash <target> -b <f411|g474|stm32c5>` or VS Code tasks (`Flash: Node 1/2/3 ...`).
- **USB DFU (STM32C542 with BOOT0 high)**: VS Code tasks `Flash: Node 1 Pi Shield (STM32C542 USB DFU)` / `Flash: Node 3 Power Slab (STM32C542 USB DFU)`.
- **Remote CAN Bootloader**: `python bootloader/tools/can_flash.py --channel can0 --node-id 0x01 firmware.bin`.
- **Debugging**: Use `.vscode/launch.json` (`marus25.cortex-debug` for ST-Link hardware, `cppdbg` for `sil_bridge_server.exe`). Never install `stmicroelectronics.stm32-vscode-extension`.
