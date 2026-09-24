# Target Integration Blockers

This branch fixes host-testable application, protocol, driver, SIL, and tooling
failures. It does not claim that the custom STM32C542 node images are ready to
flash. The following target-only gaps remain intentionally unresolved because
the required generated hardware integration is not present in this checkout.

## Missing generated startup and board configuration

`nodes/node1_pi_shield/Core/Src/main.c` and
`nodes/node3_power_slab/Core/Src/main.c` currently hand off directly to
`app_main()`. They do not contain the generated `HAL_Init()`,
`SystemClock_Config()`, `MX_GPIO_Init()`, `MX_FDCAN1_Init()`, or `MX_I2C1_Init()`
sequence, and the corresponding startup assembly and linker scripts are also
absent. Those files must be regenerated or supplied by the board owners before
a target image can be built or accepted.

The Node 1 and Node 3 BSPs also depend on generated pin macros that are absent
from their checked-in `main.h` files. This change does not guess those pin
assignments. The target readiness tests are expected to remain red until the
real CubeMX configuration is restored.

## CAN and FDCAN ownership

The shared weak CAN fallback now fails closed. A target must provide strong
`can_init`, `can_send`, `can_send_emergency`, and `can_receive` implementations
backed by the selected FDCAN instance, filters, FIFO configuration, and error
handling. The Node 2 generated CubeMX configuration still requires a hardware
review of its FDCAN frame format and bit timing before it is changed; its
configuration is generated code and is not treated as verified by the host
suite.

## Sensor integration

The BME280, BMI270/LSM6DSOXTR, INA226/INA237, MS5837, PMBus, and TMP1075 paths
now reject unavailable samples rather than returning success with stale or
zero data. The checked-in implementations still rely on mock sensor hooks;
real I2C/SPI/PMBus transactions and their fault recovery must be implemented in
the target BSP/driver integration.

## HIL watchdog observation

The vehicle CAN protocol currently has navigation, environment, and power
telemetry but no actuator/PWM readback frame. The HIL runner therefore reports
the watchdog test as unverifiable and exits nonzero instead of claiming a pass.
Add a hardware-observable output-status frame before using the HIL runner as a
release gate.

## Verification boundary

The host build and non-target CTest suite pass. The target readiness checks are
intentionally not treated as passing evidence, and the ARM cross-toolchain
build requires the repository's `arm-none-eabi` toolchain and generated target
files.
