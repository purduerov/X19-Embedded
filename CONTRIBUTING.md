# X19 Embedded Firmware Contribution Guidelines

Welcome to the Purdue ROV **X19-Embedded** team. Follow these standards to maintain safe, reliable subsea vehicle operation.

---

## 1. Development Workflow

1. **Pick an Issue**: Select your assigned issue from [GitHub Project Board #7](https://github.com/orgs/purduerov/projects/7).
2. **Branch Naming**:
   - `dev/<feature-name>` (e.g. `dev/imu-spi-driver`, `dev/pmbus-brick-telemetry`)
3. **Open Workspace**: Always open `X19-Embedded.code-workspace` in VS Code.
4. **Compile Frequently**: Press `Ctrl + Shift + B` or use the STM32Cube extension tab to verify clean builds.
5. **Format Code**: Press `Shift + Alt + F` to auto-format using `.clang-format`.
6. **Submit PR**: Open a Pull Request against `master`. All PRs require:
   - Green CI/CD build passing.
   - Lead Engineer (Aman) review and sign-off.

---

## 2. Safety & Embedded Coding Standards

- **Zero Warnings**: Code must compile cleanly with `-Wall -Wextra -Wpedantic -Werror`.
- **No Blocking Delays**: Never use `HAL_Delay()` inside telemetry loops, sensor poll loops, or timer callbacks. Use non-blocking millisecond tick comparisons (`HAL_GetTick()`) or hardware timer interrupts.
- **Fail-Safe Motor Neutral**: Thruster outputs must initialize to stopped neutral (`1500 us`) and revert to neutral immediately if the heartbeat watchdog expires or leak is detected.
- **Memory Safety**: No dynamic memory allocation (`malloc`, `free`) in embedded flight firmware. Use static allocations and compile-time arrays.
- **I2C Bus Recovery**: All I2C peripherals must implement bus timeout handling and automated 9-clock bus clear recovery via `x19_i2c_recover_bus()`.
