# VS Code embedded development workflow

This guide keeps the repository's shared CMake libraries and multi-folder VS Code workspace. VS Code organizes the folders; CMake remains the source of truth for how SIL and firmware targets build.

## Install once

Install CMake 3.22 or newer, Ninja, and the native compiler. For embedded builds install the GNU Arm Embedded toolchain. In VS Code install the recommended extensions, especially **STM32CubeIDE for Visual Studio Code** by ST and **CMake Tools**. The ST extension's bundle manager can install its supported tools. ST-LINK USB drivers must also be installed on Windows.

Open `X19-Embedded.code-workspace`, not an individual `Core` folder. The repository workspace keeps the node folders and shared libraries visible together.

## Host SIL: build and run tests

Use **Terminal > Run Task > Build: Host SIL (Debug)** or press `Ctrl+Shift+B`. This configures and builds the host simulation without requiring an STM32 or debug probe. To run the native CTest suite, choose **Test: Host SIL (Debug)**.

CMake preset equivalents from this directory:

```powershell
cmake --preset sil-debug
cmake --build --preset sil-debug
ctest --preset sil-debug
```

`sil-release` provides the corresponding release configuration. Each configuration has its own `build/` directory so changing presets does not reuse a stale CMake cache.

**Known clean-checkout build blocker:** on Windows, cmake --build --preset sil-debug currently fails at link time for the MS5837-dependent host targets with unresolved BSP I2C/time symbols (sp_i2c_write, sp_i2c_read, 	ime_get_ms, delay_ms). The preset and VS Code task make the canonical build reproducible, but a successful full SIL build requires fixing that pre-existing mock/link integration issue.

## Node 2 Nucleo G474 bench target

The only node with a complete generated target build tree in this checkout is a **STM32G474RE Nucleo bench project**. It is useful for bench work only; it is not firmware for the custom X19 Node 2 board, whose `.ioc` and architecture docs specify STM32C542.

Install `arm-none-eabi-gcc` and Ninja, connect the Nucleo board to the PC's USB port, then select **Run and Debug > Debug: Node 2 Nucleo G474 (bench only)**. `F5` runs the build task before programming/debugging through the Nucleo's onboard ST-LINK. You can also build from **Terminal > Run Task > Build: Node 2 Nucleo G474 (Debug)**.

For an external ST-LINK, connect its USB side to the computer and its SWD signals (SWDIO, SWCLK, GND, and preferably NRST) to the target board's SWD header. The board's USB-C connector is not a substitute for SWD/debug. The Control Board schematic even labels its USB-C path as not used for programming. Use the board's documented SWD header and verify target voltage and pinout before connecting.

## Current readiness by X19 node

| Node | MCU in `.ioc` | Target status in this checkout |
| --- | --- | --- |
| Node 1 Pi Shield | STM32C542 (STM32C5) | No complete Cube-generated target sources, startup, linker script, or target CMake build yet. SIL only. |
| Node 2 Control Board | STM32G474RE in checked-in generated project; STM32C542 in vehicle architecture | Generated G4 Nucleo bench target only. Do not flash it to the custom C5 board. C5 target integration remains needed. |
| Node 3 Power Slab | STM32C542 (STM32C5) | No complete Cube-generated target sources, startup, linker script, or target CMake build yet. SIL only. |

The prior VS Code configurations implied all three custom nodes were G4 targets and wrote binaries at `0x08004000`. They did not match the checked-in C5 `.ioc` files or the Node 2 G474 linker script, whose flash origin is `0x08000000`. This workflow removes those unsafe/stale launch and flash entries instead of presenting a non-buildable image as deployable.

To enable F5 flashing for all three custom boards, each needs a real STM32C542 CubeMX2-generated target (startup, HAL2/device sources, linker script, peripheral initialization), a per-node CMake target linked to `rov_shared` and `rov_drivers`, and a matching SWD launch configuration. Keep these target projects under their current node folders so they continue to link the shared libraries.

## Why this workflow

ST's current VS Code extension supports CMake, project discovery, CMake target selection, ST-LINK debugging, and multi-folder workspaces. There is no need to move shared libraries into separate IDE projects or abandon the `.code-workspace`. Desktop STM32CubeIDE remains an optional fallback for features such as advanced tracing; it is not required for this source layout.

References:

- [STM32CubeIDE for VS Code extension](https://marketplace.visualstudio.com/items?itemName=stmicroelectronics.stm32-vscode-extension)
- [ST VS Code extension commands and project discovery](https://dev.st.com/stm32cube-docs/stm32cubeide-vscode/latest/en/docs/markup/workspace_and_extension_workflow/extension_commands.html)
- [ST multi-project workspaces](https://dev.st.com/stm32cube-docs/stm32cubeide-vscode/latest/en/docs/markup/workspace_and_extension_workflow/multi_project_workspaces.html)
- [ST VS Code debugging and launch configuration](https://dev.st.com/stm32cube-docs/stm32cubeide-vscode/latest/en/docs/markup/development/debug.html)
- [ST comparison of desktop CubeIDE and its VS Code extension](https://www.st.com/content/st_com/en/stm32cubeide.html)
- [STM32CubeProgrammer GUI programming workflow](https://dev.st.com/stm32cube-docs/prog/2.23.0/en/docs/markup/CubeProg_UserManual/Memory_programming_and_erasing.html)
