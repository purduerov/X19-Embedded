# ROV Embedded Firmware Contribution Guidelines

Welcome to the Purdue ROV **Embedded Firmware** team (configured for the X19 subsea vehicle). Follow these standards to maintain safe, reliable subsea vehicle operation.

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
- **I2C Bus Recovery**: All I2C peripherals must implement bus timeout handling and automated 9-clock bus clear recovery via `rov_i2c_recover_bus()`.

---

## 3. Issue Format

All defect and audit findings use one fixed structure so they can be triaged by label. Open issues with the **Firmware finding** form (`.github/ISSUE_TEMPLATE/firmware_finding.yml`) or copy the structure below.

- **Title**: `[Priority] area: short summary` (for example `[High] drivers/bme280: t_fine is uint32_t`).
- **Body sections**, in order: Severity, Status (`NEW` or `KNOWN / already documented`), Area, Location (`file:line`), Evidence (quote the code), Impact, Suggested fix.
- **Labels**: `bug`, plus exactly one `priority:*`, one `scope:*` (`firmware` or `integration`), and the owning `track:*` where one applies.
- **Milestone**: Milestone 2 (Subsystem Application Firmware), Milestone 3 (CAN Bootloader & Flashing Suite), or Milestone 4 (Full-Stack Dry Bench Integration).

Before filing, check `docs/target-integration-blockers.md` and `docs/board_findings.md`. Gaps already recorded there should cite the document in the Status field rather than opening a duplicate.
