# VS Code embedded development workflow

This workflow keeps the shared CMake libraries and multi-folder VS Code workspace. CMake remains the source of truth for host simulation and firmware targets. USB DFU flashing is exposed as one task per vehicle node.

## Install once

Install CMake 3.22 or newer, Ninja, a native compiler, the GNU Arm Embedded toolchain, and STM32CubeProgrammer. In VS Code install the recommended extensions, especially **STM32CubeIDE for Visual Studio Code** by ST and **CMake Tools**. The flash tasks expect STM32CubeProgrammer at its default Windows install path under `Program Files`; update `.vscode/tasks.json` if installed elsewhere. Install the STM32 DFU driver if Windows does not recognize the MCU in DFU mode.

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

**Known clean-checkout build blocker:** on Windows, `cmake --build --preset sil-debug` currently fails at link time for MS5837-dependent host targets because BSP I2C/time symbols are unresolved. The preset and task make the command reproducible, but the existing mock/link integration needs repair for a successful full SIL build.

## Flash a vehicle node over USB-C

The STM32C542 factory system-memory bootloader supports USB DFU. When the board routes USB D+ and D− to the MCU USB pins, enter system-memory boot mode, connect the board's USB-C device port to the PC, then choose **Terminal > Run Task** and select one of:

- **Flash: Node 1 Pi Shield (STM32C542 USB DFU)**
- **Flash: Node 2 Vehicle Control Board (STM32C542 USB DFU)**
- **Flash: Node 3 Power Slab (STM32C542 USB DFU)**

Each task prompts for that node's ELF or Intel HEX image, connects through `USB1`, programs, verifies, then resets the MCU. In STM32CubeProgrammer GUI, the equivalent flow is: select **USB**, refresh and select the DFU device, open the matching node ELF/HEX, then click **Download**.

For STM32C5, BOOT0 high selects the factory system bootloader, subject to the BOOT_SEL option-byte configuration; BOOT0 low selects user flash. The board needs a way to assert BOOT0 and reset, such as straps/buttons. USB-C must be wired as a USB device, with proper Type-C CC Rd resistors, protection, and power arrangement. The C542 USB DFU pins are PA11 (USB_DM) and PA12 (USB_DP); check the selected package and board routing before relying on the connector. A USB-C connector by itself does not provide DFU.

### Per-node readiness

| Vehicle node | MCU | USB DFU support | Build/image status in this repository |
| --- | --- | --- | --- |
| Node 1 Pi Shield | STM32C542 | Factory ROM supports USB DFU if USB D−/D+ reach PA11/PA12 and BOOT0/reset are accessible. | `.ioc` exists; no complete generated target, startup, linker script, or deployable image yet. |
| Node 2 Control Board | STM32C542 per vehicle architecture | Same C542 DFU workflow if the custom board routes USB and exposes BOOT0/reset. | Checked-in generated project is STM32G474 Nucleo bench firmware, not vehicle firmware. Do not flash its image to the C542 vehicle board. The C542 target project/image is missing. |
| Node 3 Power Slab | STM32C542 | Same C542 DFU workflow if the board routes USB and exposes BOOT0/reset. | `.ioc` exists; no complete generated target, startup, linker script, or deployable image yet. |

The flash tasks are ready to program compatible images, but this checkout does not yet produce deployable vehicle images for all three nodes. They prompt for an image and never substitute the Node 2 G474 bench ELF. Confirm the selected image belongs to the connected node before programming. The checked-in board designs do not establish that each vehicle board has USB-C data routed to its MCU; the Control Board schematic explicitly labels its USB-C connector as unused. Node 3's `.ioc` also assigns PA11/PA12 to I2C1, so its board routing and connected peripherals need review before those pins can serve USB DFU.

## Node 2 Nucleo G474 bench target

The checked-in generated project is a **STM32G474RE Nucleo bench project**, useful only for that bench target. It is not firmware for the custom X19 Node 2 board, whose vehicle architecture specifies STM32C542.

Install `arm-none-eabi-gcc` and Ninja, connect the Nucleo to the PC, then choose **Terminal > Run Task > Build: Node 2 Nucleo G474 (Debug)**. The Nucleo's onboard ST-LINK can program that bench board. This is separate from the C542 USB DFU tasks for vehicle nodes.

## Why this workflow

ST's current VS Code extension supports CMake, project discovery, CMake target selection, ST-LINK support, and multi-folder workspaces. Shared libraries do not require moving to desktop CubeIDE. Desktop STM32CubeIDE remains an optional fallback.

References:

- [STM32CubeIDE for VS Code extension](https://marketplace.visualstudio.com/items?itemName=stmicroelectronics.stm32-vscode-extension)
- [ST VS Code extension commands and project discovery](https://dev.st.com/stm32cube-docs/stm32cubeide-vscode/latest/en/docs/markup/workspace_and_extension_workflow/extension_commands.html)
- [ST multi-project workspaces](https://dev.st.com/stm32cube-docs/stm32cubeide-vscode/latest/en/docs/markup/workspace_and_extension_workflow/multi_project_workspaces.html)
- [STM32CubeProgrammer GUI programming workflow](https://dev.st.com/stm32cube-docs/prog/2.23.0/en/docs/markup/CubeProg_UserManual/Memory_programming_and_erasing.html)
- [STM32CubeProgrammer CLI and USB DFU connection](https://dev.st.com/stm32cube-docs/prog/2.23.0/en/docs/markup/CubeProg_Command_Lines.html)
- [STM32C5 reference manual](https://www.st.com/resource/en/reference_manual/rm0522-stm32c5-series-armbased-32bit-mcus-stmicroelectronics.pdf)
- [AN2606: STM32 system-memory boot mode, including STM32C531/532/542](https://www.st.com/resource/en/application_note/an2606-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf)
