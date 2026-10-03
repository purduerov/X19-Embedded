# Board-Agnostic Hardware Targets and .ioc Architecture Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Decouple STM32CubeMX `.ioc` hardware files from application nodes by introducing a dedicated `boards/` hardware layer, modularizing the monolithic `bsp.c` into single-responsibility platform files, and introducing high-level domain services (`services/`) so `app.c` uses clean, expressive APIs without `bsp_` prefixes.

**Architecture:** 
1. **Hardware Platforms (`boards/<board>/`)**: Owns physical silicon `.ioc`, CubeMX-generated HAL, hardware `main.c`, and modular `platform/` files (`platform_time.c`, `platform_i2c.c`, `platform_can.c`, `platform_gpio.c`, `platform_pwm.c`).
2. **Services (`services/`)**: Hardware-agnostic domain services (`env_service.c`, `safety_service.c`, `actuator_service.c`) that combine drivers with platform bus I/O.
3. **Application Nodes (`nodes/<node>/`)**: Pure C11 logic (`app.c`) that interacts strictly with high-level domain APIs (`env_read_all()`, `leak_probe_is_wet()`, `can_send()`).

**Tech Stack:** C11, STM32CubeMX, CMake (Ninja), arm-none-eabi-gcc, CTest, Python 3 (`rov.py`).

**Spec:** `docs/superpowers/specs/2026-10-03-board-agnostic-ioc-architecture-design.md`

---

## Global Constraints

- C Standard: C11 strictly enforced (`c_std_11`).
- Compiler Flags: `-Wall -Wextra -Wpedantic -Wshadow -Wdouble-promotion -Werror -O2`.
- Formatting: 100% compliant with `clang-format` (`.clang-format`).
- Memory: Zero dynamic heap allocation (`malloc`/`free`) in flight firmware and services.
- Timing: Zero blocking `HAL_Delay()` calls in control loops or timer callbacks.
- Fail-Safe Invariants: Emergency brake trips, leak detection flags, and neutral PWM remains 1500 us.
- Local Isolation: All changes must remain local on the working branch without pushing to remote until approved.

---

## Review Focus

1. **Host SIL Compatibility**: Host simulation targets in `build-native` must compile cleanly without referencing ARM-specific HAL headers.
2. **Backwards Compatibility**: Existing `rov.py` commands (`python rov.py build -n rnd -b f411`, `python rov.py build -n pi_shield -b f411`) and binary outputs must continue to work without breaking.
3. **No God Files**: No single platform file may exceed 200 lines or combine unrelated peripherals (e.g. CAN and I2C must be separated).
4. **Clean API Usability**: Application logic in `app.c` must not contain raw register addresses or `bsp_` prefixes.
5. **CubeMX Safety**: `main.c` across all boards must strictly contain only peripheral setup and hand off execution to `app_main()` inside `/* USER CODE BEGIN 2 */`.

---

## Task Breakdown

### Task 1: Define Domain Service Interfaces (`services/include/`)

**Files:**
- Create: `services/include/env_service.h`
- Create: `services/include/safety_service.h`
- Create: `services/include/actuator_service.h`
- Create: `tests/test_services.c`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces:
  - `rov_status_t env_read_all(float *temp_c, float *press_hpa, float *hum_pct);`
  - `float env_get_temperature(void);`
  - `float env_get_pressure(void);`
  - `float env_get_humidity(void);`
  - `bool leak_probe_is_wet(uint8_t probe_idx);`
  - `void safety_emergency_trip(void);`
  - `bool safety_is_tripped(void);`
  - `void pwm_set_pulse_us(uint8_t channel, uint16_t pulse_us);`
  - `uint16_t pwm_get_pulse_us(uint8_t channel);`
  - `void solenoid_set_mask(uint16_t mask);`

- [ ] **Step 1: Write header definitions in `services/include/`**
  - Create `env_service.h`, `safety_service.h`, and `actuator_service.h` with clean Doxygen documentation and include guards.
- [ ] **Step 2: Write unit test `tests/test_services.c`**
  - Add test cases checking reading mock values and verifying safety trip state transitions.
- [ ] **Step 3: Register test in `tests/CMakeLists.txt`**
  - Add `test_services` executable and register via `add_test(NAME test_services COMMAND test_services)`.
- [ ] **Step 4: Verify test fails link before implementation**
  - Run: `cmake --build build-native --target test_services`
  - Expected: Link failure on missing service function symbols.
- [ ] **Step 5: Commit interface definitions**
  ```bash
  git add services/include/ tests/test_services.c tests/CMakeLists.txt
  git commit -m "feat(services): define domain service interfaces and unit tests"
  ```

---

### Task 2: Implement Domain Services (`services/src/`)

**Files:**
- Create: `services/src/env_service.c`
- Create: `services/src/safety_service.c`
- Create: `services/src/actuator_service.c`
- Create: `services/CMakeLists.txt`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `drivers/include/bme280.h`, `shared/include/bsp.h`, `shared/include/can_interface.h`
- Produces: `librov_services.a` (CMake target `rov_services`)

- [ ] **Step 1: Implement `env_service.c`**
  - Wraps BME280 initialization and measurement reads, caching latest temperature, pressure, and humidity in engineering units.
- [ ] **Step 2: Implement `safety_service.c`**
  - Wraps leak probe reading and latched emergency stop cutoff with immediate CAN alert broadcast.
- [ ] **Step 3: Implement `actuator_service.c`**
  - Enforces 1000–2000 us physical clamp (1500 us neutral) and delegates to platform PWM.
- [ ] **Step 4: Wire `services/CMakeLists.txt` and root `CMakeLists.txt`**
  - Add `add_subdirectory(services)` and link `rov_services` to native SIL and embedded targets.
- [ ] **Step 5: Run tests and verify 100% pass**
  - Run: `ctest --test-dir build-native -R "^test_services$" --output-on-failure`
  - Expected: PASS.
- [ ] **Step 6: Commit services implementation**
  ```bash
  git add services/ CMakeLists.txt
  git commit -m "feat(services): implement hardware-agnostic domain services layer"
  ```

---

### Task 3: Refactor Node 1 (`nodes/node1_pi_shield/Core/Src/app.c`) to Use Domain Services

**Files:**
- Modify: `nodes/node1_pi_shield/Core/Src/app.c`
- Modify: `nodes/node1_pi_shield/Core/Inc/app.h`
- Modify: `nodes/node1_pi_shield/CMakeLists.txt`
- Test: `tests/test_node1_pi_shield.c`

**Interfaces:**
- Consumes: `services/include/env_service.h`, `services/include/safety_service.h`, `shared/include/can_interface.h`

- [ ] **Step 1: Replace raw sensor calls in `node1_pi_shield/Core/Src/app.c`**
  - Use `env_read_all()`, `leak_probe_is_wet()`, and `can_send()` directly.
- [ ] **Step 2: Link `rov_services` in `nodes/node1_pi_shield/CMakeLists.txt`**
- [ ] **Step 3: Run Node 1 SIL test**
  - Run: `ctest --test-dir build-native -R "^test_node1_pi_shield$" --output-on-failure`
  - Expected: PASS.
- [ ] **Step 4: Commit Node 1 refactor**
  ```bash
  git add nodes/node1_pi_shield/
  git commit -m "refactor(node1): use clean domain services in application loop"
  ```

---

### Task 4: Modularize NUCLEO-F411 Platform into `boards/f411_nucleo/`

**Files:**
- Create directory: `boards/f411_nucleo/`
- Move: `nodes/testing_and_rnd/testing_and_rnd.ioc` -> `boards/f411_nucleo/f411_nucleo.ioc`
- Move: `nodes/testing_and_rnd/Projects/cmake/stm32cubemx/` -> `boards/f411_nucleo/cmake/stm32cubemx/`
- Create: `boards/f411_nucleo/platform/platform_time.c` (time_get_ms, time_get_us, delay_ms)
- Create: `boards/f411_nucleo/platform/platform_i2c.c` (i2c_read, i2c_write, i2c_mem_read, i2c_mem_write)
- Create: `boards/f411_nucleo/platform/platform_can.c` (bxCAN can_init, can_send, can_receive)
- Create: `boards/f411_nucleo/platform/platform_gpio.c` (led_toggle, leak_probe lines)
- Create: `boards/f411_nucleo/platform/platform_pwm.c` (pwm_set_pulse_us)
- Move & Adapt: `nodes/testing_and_rnd/Src/main.c` -> `boards/f411_nucleo/Core/Src/main.c`
- Create: `boards/f411_nucleo/CMakeLists.txt`

- [ ] **Step 1: Create `boards/f411_nucleo/` directory tree**
- [ ] **Step 2: Extract focused platform modules from `bsp.c`**
  - Create `platform_time.c`, `platform_i2c.c`, `platform_can.c`, `platform_gpio.c`, `platform_pwm.c` (each strictly < 100 lines).
- [ ] **Step 3: Relocate F411 `.ioc` and CubeMX CMake tree into `boards/f411_nucleo/`**
- [ ] **Step 4: Write `boards/f411_nucleo/CMakeLists.txt` defining platform library `f411_platform`**
- [ ] **Step 5: Verify clean compilation of `f411_platform` library**
  - Run: `cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake`
  - Expected: Clean configuration.
- [ ] **Step 6: Commit F411 board modularization**
  ```bash
  git add boards/f411_nucleo/
  git commit -m "feat(boards): introduce boards/f411_nucleo with modular platform drivers"
  ```

---

### Task 5: Modularize NUCLEO-G474 Platform into `boards/g474_nucleo/`

**Files:**
- Create directory: `boards/g474_nucleo/`
- Move: `nodes/node2_control_board/node2_control_board.ioc` -> `boards/g474_nucleo/g474_nucleo.ioc`
- Move: `nodes/node2_control_board/cmake/stm32cubemx/` -> `boards/g474_nucleo/cmake/stm32cubemx/`
- Create: `boards/g474_nucleo/platform/platform_time.c`
- Create: `boards/g474_nucleo/platform/platform_i2c.c`
- Create: `boards/g474_nucleo/platform/platform_can.c` (FDCAN)
- Create: `boards/g474_nucleo/platform/platform_gpio.c`
- Create: `boards/g474_nucleo/platform/platform_pwm.c`
- Create: `boards/g474_nucleo/CMakeLists.txt`

- [ ] **Step 1: Create `boards/g474_nucleo/` directory tree**
- [ ] **Step 2: Relocate G474 `.ioc` and CubeMX CMake files**
- [ ] **Step 3: Implement modular G474 platform files**
- [ ] **Step 4: Verify G474 CMake configuration**
- [ ] **Step 5: Commit G474 board modularization**
  ```bash
  git add boards/g474_nucleo/
  git commit -m "feat(boards): introduce boards/g474_nucleo with modular platform drivers"
  ```

---

### Task 6: Update Root CMake & Build Target Composition

**Files:**
- Modify: `CMakeLists.txt`
- Modify: `nodes/testing_and_rnd/Projects/CMakeLists.txt` (maintain backwards compatibility)

- [ ] **Step 1: Wire `add_subdirectory(boards)` in root `CMakeLists.txt`**
- [ ] **Step 2: Define dynamic composition targets**
  - `f411_node1_pi_shield.elf`
  - `f411_node2_control_board.elf`
  - `f411_node3_power_slab.elf`
  - `f411_sandbox.elf`
- [ ] **Step 3: Retain backwards-compatible aliases**
  - `testing_and_rnd.elf` -> points to `f411_sandbox.elf`
  - `bench_node1_pi_shield.elf` -> points to `f411_node1_pi_shield.elf`
- [ ] **Step 4: Build targets using ARM cross-compiler**
  - Run: `python rov.py build -n rnd -b f411`
  - Run: `python rov.py build -n pi_shield -b f411`
  - Expected: Zero compilation errors, zero warnings.
- [ ] **Step 5: Commit CMake composition**
  ```bash
  git add CMakeLists.txt
  git commit -m "build(cmake): compose board platforms and application nodes dynamically"
  ```

---

### Task 7: Update `rov.toml` and CLI Mapping

**Files:**
- Modify: `rov.toml`
- Modify: `tools/rov.py` (if path resolution needs updates)

- [ ] **Step 1: Update board definitions in `rov.toml`**
  - Point board directories to `boards/f411_nucleo` and `boards/g474_nucleo`.
- [ ] **Step 2: Test CLI commands**
  - Run: `python rov.py build -n rnd -b f411`
  - Run: `python rov.py devices`
- [ ] **Step 3: Run full host SIL regression test**
  - Run: `ctest --test-dir build-native -E "^target_" --output-on-failure`
  - Expected: 25/25 suites pass (100%).
- [ ] **Step 4: Run `clang-format` on all modified files**
- [ ] **Step 5: Commit configuration and verification**
  ```bash
  git add rov.toml tools/
  git commit -m "chore(config): update rov.toml for board-agnostic targets and verify build"
  ```
