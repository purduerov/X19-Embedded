## Summary of Changes
<!-- Provide a clear, concise description of what this PR introduces or fixes -->

## Related Issues & Track
- Resolves #<!-- insert issue number -->
- **Track**: `track:lead-aman` | `track:dev1-nav-actuation` | `track:dev2-power-slab` | `track:dev3-pi-shield`

## Target Node(s) Affected
- [ ] `shared/` (Core Protocol & Types)
- [ ] `drivers/` (Sensor / Transceiver Drivers)
- [ ] `nodes/node1_pi_shield` (Pi Shield)
- [ ] `nodes/node2_control_board` (Control Board)
- [ ] `nodes/node3_power_slab` (Power Slab)
- [ ] `nodes/node4_usb_hub` (USB Camera Hub)
- [ ] `bootloader/` (CAN Bootloader Suite)

## Pre-Merge Safety & Quality Checklist
- [ ] **Clean Compilation**: Compiles with zero errors and zero warnings under `-Wall -Wextra -Werror`.
- [ ] **Formatting**: Code formatted with `clang-format` (`Shift + Alt + F` in VS Code).
- [ ] **Non-Blocking Execution**: No blocking `HAL_Delay()` calls inside real-time control loops or ISRs.
- [ ] **CAN Protocol Compliance**: CAN FD message IDs, DLC lengths, and bit timings strictly match `x19_parameters.h`.
- [ ] **Physical Bounds**: Thruster PWM commands remain within 1000–2000 us (1500 us neutral stop).
- [ ] **Hardware Tested**: Tested on physical ST-Link / development bench (if applicable).
