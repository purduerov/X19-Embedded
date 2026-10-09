# Design Spec: Board-Agnostic Hardware Targets and .ioc Architecture

- **Date**: 2026-10-03
- **Status**: Draft / In Review
- **Scope**: `Embedded/X19-Embedded/`
- **Track**: Embedded Architecture & Build System

---

## 1. Problem Statement & Motivation

Currently, STM32CubeMX `.ioc` project definition files and generated HAL trees are co-located inside vehicle node folders:
- `nodes/testing_and_rnd/testing_and_rnd.ioc` (Actually the NUCLEO-F411 / F446 dev board).
- `nodes/node2_control_board/node2_control_board.ioc` (Actually the NUCLEO-G474RE dev board).
- `nodes/node1_pi_shield/node1_pi_shield.ioc` (STM32C542 production vehicle MCU).
- `nodes/node3_power_slab/node3_power_slab.ioc` (STM32C542 production vehicle MCU).

### Consequences:
1. **Hardware / Application Coupling**: Application developers working on Node 1 (Pi Shield) or Node 3 (Power Slab) on physical bench hardware had to build inside `nodes/testing_and_rnd/Projects/CMakeLists.txt` because Node 1's directory has a C5 `.ioc`, not an F4 `.ioc`.
2. **CubeMX Re-generation Confusion**: Modifying pinout or clock configuration for a dev board required editing files scattered across node directories.
3. **Redundant `.ioc` Configurations**: Multiple nodes duplicating board setups or relying on complex cross-node CMake include paths.

---

## 2. Core Architectural Principle: Hardware Platforms vs Application Nodes

We strictly decouple **Physical Hardware Targets** from **Subsea Application Firmware**:

- **`boards/<board>/` (Hardware Platforms)**:
  - Owns the physical silicon configuration (`.ioc` file).
  - Owns STM32CubeMX code generation (`Drivers/`, CMSIS, `startup_stm32....s`, `cmake/stm32cubemx/`).
  - Owns the hardware entry point (`Core/Src/main.c`) which configures clocks, GPIOs, and interrupts before calling `app_main()`.
  - Owns the board-specific Board Support Package (`bsp.c`) that bridges vendor ST HAL calls to the standardized `shared/include/bsp.h` and `shared/include/can_interface.h`.
  - Zero vehicle logic (no thruster equations, no depth math, no vacuum state machines).

- **`nodes/<node>/` (Pure Application Logic)**:
  - 100% portable C11 code (`Core/Src/app.c`, `Core/Inc/app.h`).
  - Zero ST HAL calls, zero vendor register access, zero raw register twiddling.
  - Interacts with hardware solely through clean, modular domain APIs without `bsp_` prefixes:
    - Sensors: `env_get_temperature()`, `env_get_humidity()`, `env_get_pressure()`, `env_read_all()`
    - Safety: `leak_probe_is_wet(probe_id)`, `safety_emergency_trip()`, `safety_is_tripped()`
    - Actuators: `pwm_set_pulse_us(channel, us)`, `solenoid_set_mask(mask)`
    - Communication: `can_send(id, data, len)`, `can_receive(&id, data, &len)`
    - Timing: `time_get_ms()`, `delay_ms(ms)`

### 2.1 High-Level Domain Application APIs (No "BSP" Prefix Baggage)

Application code in `app.c` reads cleanly and intuitively:

```c
void node1_app_step(void) {
    float temp_c = 0.0f;
    float press_hpa = 0.0f;
    float hum_pct = 0.0f;

    /* Read environmental sensors */
    if (env_read_all(&temp_c, &press_hpa, &hum_pct) == ROV_OK) {
        vacuum_state_machine_update(press_hpa, hum_pct);
    }

    /* Check floor leak probes */
    if (leak_probe_is_wet(0) || leak_probe_is_wet(1)) {
        safety_emergency_trip();
        can_send_emergency(ROV_CAN_ID_EMERGENCY_BREAK, NULL, 0);
    }

    /* Broadcast 10 Hz environmental telemetry over CAN */
    if (time_get_ms() - last_telemetry_time >= 100) {
        last_telemetry_time = time_get_ms();
        rov_can_env_telemetry_t payload = {
            .temperature_c = temp_c,
            .pressure_hpa = press_hpa,
            .humidity_pct = hum_pct,
        };
        can_send(ROV_CAN_ID_ENV_TELEMETRY, (uint8_t *)&payload, sizeof(payload));
    }
}
```

### 2.2 Eliminating the "God File" (`bsp.c` -> Focused Platform Modules)

Rather than dumping CAN, I2C, bit-banging, UART, timers, GPIO, PWM, and PMBus into a single 500-line monolithic `bsp.c`, each board platform is split into focused, single-responsibility modules:

- `platform_time.c`: SysTick timers (`time_get_ms()`, `time_get_us()`, `delay_ms()`).
- `platform_i2c.c`: I2C peripheral init and bus transactions (`i2c_write()`, `i2c_read()`, `i2c_mem_read()`).
- `platform_can.c`: CAN/FDCAN hardware mailboxes, filters, and ISR callbacks (`can_init()`, `can_send()`).
- `platform_gpio.c`: Indicator LEDs, leak probe pins, emergency cutoff lines (`led_toggle()`, `leak_probe_is_wet()`).
- `platform_pwm.c`: Timer PWM channels for ESCs (`pwm_set_pulse_us()`).
- `services/env_service.c`: High-level sensor service linking the pure driver (`bme280.c`) with `platform_i2c.c` to expose `env_read_all()`, `env_get_temp()`, `env_get_pressure()`, `env_get_humidity()`.

---

## 3. Directory Layout Specification

```text
Embedded/X19-Embedded/
├── boards/                                   # Physical Hardware Targets (Owns .ioc & HAL)
│   ├── f411_nucleo/                          # ST NUCLEO-F411RE (Dev Bench)
│   │   ├── f411_nucleo.ioc                   # CubeMX definition (clocks, pins)
│   │   ├── cmake/stm32cubemx/                # Generated HAL CMake configuration
│   │   ├── Core/Src/main.c                   # Hardware init trampoline -> app_main()
│   │   ├── platform/                         # Focused Platform Modules (NO God File)
│   │   │   ├── platform_time.c               # time_get_ms, delay_ms
│   │   │   ├── platform_i2c.c                # I2C driver
│   │   │   ├── platform_can.c                # CAN driver (can_send, can_receive)
│   │   │   ├── platform_gpio.c               # LED, leak probe lines
│   │   │   └── platform_pwm.c                # ESC PWM channels
│   │   └── STM32F411xE_FLASH.ld              # Linker script
│   │
│   ├── g474_nucleo/                          # ST NUCLEO-G474RE (Control Bench)
│   │   ├── g474_nucleo.ioc                   # CubeMX definition (clocks, pins)
│   │   ├── cmake/stm32cubemx/                # Generated HAL CMake configuration
│   │   ├── Core/Src/main.c                   # Hardware init trampoline -> app_main()
│   │   ├── platform/
│   │   │   ├── platform_time.c
│   │   │   ├── platform_i2c.c
│   │   │   ├── platform_can.c                # FDCAN
│   │   │   ├── platform_gpio.c
│   │   │   └── platform_pwm.c
│   │   └── STM32G474RETx_FLASH.ld            # Linker script
│   │
│   └── c542_vehicle/                         # Subsea Vehicle Custom PCB (STM32C542)
│       ├── c542_vehicle.ioc                  # CubeMX definition (clocks, pins)
│       ├── Core/Src/main.c                   # Hardware init trampoline -> app_main()
│       └── platform/
│           ├── platform_time.c
│           ├── platform_i2c.c
│           ├── platform_can.c
│           └── platform_gpio.c
│
├── services/                                 # High-Level Subsystem Services
│   ├── env_service.c                         # env_read_all, env_get_temp, env_get_pressure
│   ├── safety_service.c                      # safety_emergency_trip, leak_probe_is_wet
│   └── actuator_service.c                    # pwm_set_pulse_us, solenoid_set_mask
│
├── nodes/                                    # Subsea Application Logic (Zero HAL)
│   ├── node1_pi_shield/                      # Node 1 Application
│   │   ├── Core/Src/app.c                    # BME280 leak, 10 Hz CAN telemetry
│   │   └── Core/Inc/app.h
│   ├── node2_control_board/                  # Node 2 Application
│   │   ├── Core/Src/app.c                    # 8x PWM, solenoids, IMU, depth
│   │   └── Core/Inc/app.h
│   ├── node3_power_slab/                     # Node 3 Application
│   │   ├── Core/Src/app.c                    # PMBus power monitoring
│   │   ├── Core/Src/power_sequence.c         # Staggered converter sequencing
│   │   └── Core/Inc/app.h
│   └── sandbox/                              # Developer Scratchpad
│       ├── sandbox.c
│       └── app.c
│
├── shared/                                   # Shared CAN protocol & types
├── drivers/                                  # Pure sensor drivers (BME280, MS5837, INA237)
├── tests/                                    # Host SIL CTest test suites (25 suites)
└── tools/                                    # rov CLI toolchain
```

---

## 4. CMake Build System Design

Each target executable is composed cleanly:

$$\text{Executable}(\text{Node } N, \text{Board } B) = \text{boards}/B\text{ (HAL, main.c, bsp.c)} + \text{nodes}/N\text{ (app.c)} + \text{shared} + \text{drivers}$$

### Target Naming Standard:
- `f411_node1_pi_shield.elf`
- `f411_node2_control_board.elf`
- `f411_node3_power_slab.elf`
- `f411_sandbox.elf`
- `g474_node2_control_board.elf`
- `c542_node1_pi_shield.elf`
- `c542_node2_control_board.elf`
- `c542_node3_power_slab.elf`

For backwards compatibility with existing VS Code tasks and debug configurations, legacy aliases (`testing_and_rnd.elf`, `bench_node1_pi_shield.elf`) will be maintained.

---

## 5. `rov.toml` and CLI Mapping

The declarative `rov.toml` naturally maps to this model:

```toml
[boards.f411]
display_name = "ST NUCLEO-F411RE (Dev Bench)"
mcu = "STM32F411RET6"
board_dir = "boards/f411_nucleo"
preset = "arm-c542-debug"

[boards.g474]
display_name = "ST NUCLEO-G474RE (Control Bench)"
mcu = "STM32G474RET6"
board_dir = "boards/g474_nucleo"
preset = "Debug"

[nodes.pi_shield]
app_source = "nodes/node1_pi_shield/Core/Src/app.c"
target_f411 = "f411_node1_pi_shield.elf"
target_c542 = "c542_node1_pi_shield.elf"
```

Invoking:
```bash
python rov.py build -n pi_shield -b f411
python rov.py run -n sandbox -b f411
python rov.py run -n control_board -b g474
```
builds the exact pair seamlessly.

---

## 6. Verification & Safety Contract

1. **Clean Host SIL**: Running `ctest --test-dir build-native -E "^target_" --output-on-failure` must pass 25/25 suites (100%).
2. **Clean ARM Cross-Compilation**: All bench targets must compile with zero errors and zero warnings under strict `-Wall -Wextra -Wpedantic -Werror`.
3. **No Flashing Regressions**: Auto-detecting ST-Link probes and serial monitoring via `python rov.py run` must continue to work without manual reconfiguration.
4. **Local Isolation**: All changes remain local until explicitly approved by the user.
