# Target Integration Blockers

This branch fixes host-testable application, protocol, driver, SIL, and tooling
failures. It does not claim that the custom STM32C542 node images are ready to
flash. The following target-only gaps remain intentionally unresolved because
the required generated hardware integration is not present in this checkout.

## Missing generated startup and board configuration

`nodes/node1_pi_shield/Core/Src/main.c` and
`nodes/node3_power_slab/Core/Src/main.c` call `HAL_Init()`,
`SystemClock_Config()`, `MX_GPIO_Init()`, `MX_FDCAN1_Init()`, and `MX_I2C1_Init()`,
but none of those definitions are present in this checkout, and the corresponding
startup assembly and linker scripts are also absent. The root cross build
therefore does not build the STM32C542 images by default; set
`X19_BUILD_NODE1_ROOT` or `X19_BUILD_NODE3_ROOT` to `ON` once the generated files
land. Those files must be regenerated or supplied by the board owners before a
target image can be built or accepted.

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

The BME280, INA226, INA237, and TMP1075 drivers read the injected I2C bus
(`rov_i2c_bus_t`) and convert raw registers to engineering units per datasheet;
their host tests drive the real driver against a device model in the mock
register file, and no driver carries a value hook. INA226 validates its
manufacturer and die ID; TMP1075N and INA237 have no usable ID register and
validate arguments only. On target, each driver still needs a board
`bsp_i2c_bus()` implementation and, for INA237, a configured I2C peripheral.
The BMI270/LSM6DSOXTR (SPI), MS5837 (command-addressed), and PMBus paths remain
on their existing hooks and are out of scope here.

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
