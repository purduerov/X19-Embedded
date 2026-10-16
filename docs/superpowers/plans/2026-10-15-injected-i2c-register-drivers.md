# Injected I2C Bus: Remaining Register-Addressed Drivers Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move the three remaining register-addressed I2C drivers (TMP1075, INA237, INA226) onto the injected `rov_i2c_bus_t`, replacing their `mock_sensors_get_*` value hooks with real datasheet register I/O and a host device model, so each driver runs unchanged on target firmware and host SIL.

**Architecture:** Same shape as the BME280 pilot. A driver holds a `const rov_i2c_bus_t *` and issues raw `write`/`read` through the inline `rov_i2c_mem_*` helpers. It converts bytes to engineering units exactly per datasheet and stores nothing else. The host side gets one device model per chip that writes datasheet-correct words into the `mock_bsp.c` register file, so SIL tests exercise the real driver instead of a value hook. `app.c` keeps calling the driver through the bus it already obtains (`bsp_i2c_bus()`).

**Tech Stack:** C11, CMake 3.22, `assert`-style host tests, `tests/mocks/mock_bsp.c` register file, STM32 HAL BSP adapters.

**Spec:** `docs/superpowers/specs/2026-10-15-injected-i2c-bus-driver-architecture-design.md` (Migration order step 5)

**Predecessor plan:** `docs/superpowers/plans/2026-10-15-injected-i2c-bus-foundation.md` (Tasks 1-4: bus contract, host bus, BME280 pilot, target/bench adapters)

## The register-width problem this plan must solve first

The BME280 is byte-addressed: its registers are bytes at consecutive addresses (`0x88`, `0x89`, ...), so the mock's byte-addressed register file models it directly. TMP1075, INA237, and INA226 are not. They are **16-bit-register devices with 1-byte-spaced register addresses**: VSHUNT is register `0x04`, VBUS is register `0x05`, DIETEMP is register `0x06`, and each register is a 16-bit word read MSB-first. A driver sends the register address `R` once and reads two bytes.

The mock's `mock_bus_read()` returns `g_mock_i2c_regs[addr][ptr + i]` for `i` in `0..len-1`, i.e. it advances the pointer per **byte**. A two-byte read at register `0x05` would therefore return the bytes at offsets `0x05` and `0x06`, which is exactly where a byte-addressed model would also put register `0x06`. Consecutive 16-bit registers cannot coexist in that file: writing VBUS and then DIETEMP would corrupt VBUS's low byte.

So Task 1 adds 16-bit word registers to the mock bus (a second, word-indexed file plus a per-address flag), and the three device models write words instead of byte pairs. Everything else follows the BME280 pilot.

## Global Constraints

- Build warnings are errors: `-Wall -Wextra -Wpedantic -Wshadow -Wdouble-promotion -Werror`. Every new file must compile clean. `-Wdouble-promotion` means every float literal in a float expression carries the `f` suffix.
- Drivers must not call ST HAL, `bsp_*`, or `mock_*`. They only touch `dev->bus`. No `malloc`/`free`. No `HAL_Delay()`.
- A driver is bytes-to-units and nothing else: no history, no thresholds, no policy, no software filtering. Chip-internal configuration registers (the INA237 ADC mode) are datasheet work and stay in the driver.
- `app.c` must not name a chip, driver, or bus beyond the calls it already makes. This plan changes `ina237_init()`'s signature, so the one existing caller in `nodes/node1_pi_shield/Core/Src/app.c` is updated in place; no other app logic changes.
- Device words are big-endian (MSB first) on all three parts.
- Do not use emojis anywhere.
- Host build environment: prepend `C:\msys64\ucrt64\bin` to `PATH` before any `cmake`/`ctest`/`ninja` invocation, or `cc1.exe` fails to load its DLLs.
- Host test commands, from `Embedded/X19-Embedded/`:

  ```powershell
  $env:PATH = "C:\msys64\ucrt64\bin;" + $env:PATH
  cmake -B build-native -G Ninja
  cmake --build build-native
  ctest --test-dir build-native -E "^target_" --output-on-failure
  ```

- Cross build, from `Embedded/X19-Embedded/` (must stay green; the drivers compile into the target libraries):

  ```powershell
  cmake --preset arm-c542-debug
  cmake --build --preset arm-c542-debug
  ```

- The two drift guards stay green with no CMake test-target changes: `tests/test_registry_consistency.py` and `tests/test_suite_manifest.py`. This plan changes the *bodies* of existing `test_*` targets and adds mock source files to the `rov_mocks` object library; it adds no new CTest target, so `docs/sil_simulation_guide.md` keeps the same 36 rows.
- `target_*` acceptance tests are expected-red contracts; do not attempt to make them pass. Do not weaken them either.

## Datasheet facts this plan depends on

| Part | Register | Address | LSB / encoding |
| :--- | :--- | :--- | :--- |
| TMP1075 | Temperature | `0x00` | 12-bit two's complement, left-justified in 16 bits (bits 3:0 read 0), `0.0625 C` per count |
| TMP1075 | Device ID | `0x0F` | `0x7500`; **only on non-N orderables**. The board part is `TMP1075NDRLR` (N), so the driver must not probe it |
| INA226 | Shunt Voltage | `0x01` | signed 16-bit, `2.5 uV` per count |
| INA226 | Bus Voltage | `0x02` | unsigned 16-bit, `1.25 mV` per count |
| INA226 | Manufacturer ID | `0xFE` | `0x5449` |
| INA226 | Die ID | `0xFF` | `0x2260` or `0x2261` |
| INA226 | Config (reset) | `0x00` | reset `0x4127`: MODE = `111` = continuous shunt+bus, so no config write is needed |
| INA237 | Shunt Voltage | `0x04` | signed 16-bit, `5 uV` per count at the default `ADCRANGE = 0` |
| INA237 | Bus Voltage | `0x05` | unsigned 16-bit, `3.125 mV` per count |
| INA237 | Die Temp | `0x06` | signed 12-bit, left-justified in 16 bits, `0.125 C` per count |
| INA237 | ADC Config | `0x01` | reset `0xFB68` = continuous, 1052 us on all three fields, 1-sample average; the driver writes `0xFB6A` to select 16-sample averaging |
| INA237 | Device ID | `0x3F` | **not available on INA237** (present on INA238 only), so the driver must not probe it |

Sources: TI TMP1075, INA226, INA237 datasheets; Linux `drivers/hwmon/ina238.c` (covers INA237: `INA238_DEVICE_ID` is annotated "not available on INA237"; INA237 `temp_resolution = 12`; shunt LSB `5 uV` at default gain). The 1-byte-address / 2-byte-value I2C convention for these parts is confirmed by the Adafruit INA226/INA237 read/write helpers and by the datasheet register maps (registers at `0x00`, `0x01`, `0x02`, ... are distinct 16-bit registers, so the device advances its address pointer per register, not per byte).

## Review Focus

- A `NULL` bus, a `NULL` dev, or a non-positive shunt must fail loudly (`ROV_ERR_INVALID_ARG`) rather than store a pointer that faults on the first read, or divide by zero.
- A transport error must propagate out of every read unchanged, and must leave the caller's previous reading intact (no half-updated struct).
- The TMP1075 conversion must sign-extend a negative temperature correctly. Sign-extending the 16-bit word *before* dropping the four pad bits is the whole trick; doing it after gives +255.9 C for -0.0625 C.
- The INA237 die-temperature field is 12-bit left-justified like TMP1075; the INA237 shunt/bus fields are full 16-bit.
- `ina237_init()` must write `ADC_CONFIG` to select 16-sample averaging (the reset `0xFB68` is already continuous at 1052 us, so the write is not what leaves shutdown). `ina226_init()` must not write `CONFIG`, because the power-on default is already continuous.
- Only INA226 gets a chip-ID probe (it has manufacturer and die ID registers). TMP1075N and INA237 have no usable ID register, so their `init()` validates arguments only. This asymmetry is deliberate and documented at each `init()`.
- A word-addressed device must not alias a neighbouring register: the mock's word file is indexed by register address, and the byte file is untouched, so a two-byte read at register `R` returns only register `R`.
- The mock and the target must present the same register semantics (one register address, two big-endian bytes), or a driver that passes on host fails on hardware.

---

### Task 1: 16-bit word registers in the mock I2C bus

**Files:**
- Modify: `tests/mocks/mock_bsp.h`
- Modify: `tests/mocks/mock_bsp.c`
- Modify: `tests/test_mock_i2c_bus.c`
- Modify: `docs/sil_simulation_guide.md` (row 12 description)

**Interfaces:**
- Produces (host only): `mock_bsp_i2c_set_word(uint8_t addr, uint8_t reg, uint16_t value)` and `mock_bsp_i2c_get_word(uint8_t addr, uint8_t reg)`. Calling `mock_bsp_i2c_set_word()` also marks the address as word-addressed, which switches `mock_bus_read()`/`mock_bus_write()` for that address to the word file.
- Consumes: the existing `mock_bsp_i2c_set_regs()` byte path, which stays byte-addressed for BME280.

- [ ] **Step 1: Write the failing test**

In `tests/test_mock_i2c_bus.c`, add this function before `main()`:

```c
static void test_word_register_round_trip(void) {
    const rov_i2c_bus_t *bus = bsp_i2c_bus();
    uint8_t readback[2] = {0u, 0u};
    uint8_t writeback[2] = {0xABu, 0xCDu};

    mock_bsp_reset();
    mock_bsp_i2c_set_word(0x40u, 0x05u, 0x1234u);

    /* A two-byte read at register 0x05 returns the word, MSB first. */
    assert(rov_i2c_mem_read(bus, 0x40u, 0x05u, readback, 2U) == ROV_OK);
    assert(readback[0] == 0x12u);
    assert(readback[1] == 0x34u);

    /*
     * The word store must not touch the byte register at the same index, or
     * register 0x05 (VBUS) and register 0x06 (DIETEMP) would alias.
     */
    assert(mock_bsp_i2c_get_reg(0x40u, 0x05u) == 0x00u);
    assert(mock_bsp_i2c_get_reg(0x40u, 0x06u) == 0x00u);

    /* A two-byte write through the bus updates the word file. */
    assert(rov_i2c_mem_write(bus, 0x40u, 0x05u, writeback, 2U) == ROV_OK);
    assert(mock_bsp_i2c_get_word(0x40u, 0x05u) == 0xABCDu);

    /* A byte-addressed address is unaffected by the word path. */
    mock_bsp_i2c_set_regs(0x76u, 0x10u, writeback, 2U);
    assert(mock_bsp_i2c_get_reg(0x76u, 0x10u) == 0xABu);
    assert(rov_i2c_mem_read(bus, 0x76u, 0x10u, readback, 2U) == ROV_OK);
    assert(readback[0] == 0xABu);
    assert(readback[1] == 0xCDu);
}
```

and call it from `main()`:

```c
    test_word_register_round_trip();
```

- [ ] **Step 2: Run the test and confirm it fails to build**

```powershell
$env:PATH = "C:\msys64\ucrt64\bin;" + $env:PATH
cmake --build build-native --target test_mock_i2c_bus
```

Expected: compile error, `mock_bsp_i2c_set_word` / `mock_bsp_i2c_get_word` undefined. RED.

- [ ] **Step 3: Declare the word accessors**

In `tests/mocks/mock_bsp.h`, add after the `mock_bsp_i2c_set_regs` declaration:

```c
/**
 * @brief Set a 16-bit register and mark the device word-addressed.
 *
 * A byte-addressed device (BME280) stores register R at byte offset R. A
 * word-addressed device (INA226, INA237, TMP1075) stores the 16-bit register R
 * as one word, and a driver reads two big-endian bytes from register R. The mock
 * keeps those words in a separate file so register R cannot alias register R+1,
 * which the byte file would otherwise do because it advances one byte per step.
 *
 * @param addr 7-bit I2C device address.
 * @param reg Register address.
 * @param value 16-bit register value.
 */
void mock_bsp_i2c_set_word(uint8_t addr, uint8_t reg, uint16_t value);

/**
 * @brief Read a 16-bit register from the mock word file.
 *
 * @param addr 7-bit I2C device address.
 * @param reg Register address.
 * @return Stored word, or 0 for an unknown address.
 */
uint16_t mock_bsp_i2c_get_word(uint8_t addr, uint8_t reg);
```

- [ ] **Step 4: Add the word file and switch the bus on the flag**

In `tests/mocks/mock_bsp.c`, add the two arrays next to the existing register file:

```c
static uint8_t g_mock_i2c_regs[MOCK_I2C_NUM_ADDRS][MOCK_I2C_NUM_REGS];
static uint16_t g_mock_i2c_words[MOCK_I2C_NUM_ADDRS][MOCK_I2C_NUM_REGS];
static bool g_mock_i2c_word_addressed[MOCK_I2C_NUM_ADDRS];
static uint8_t g_mock_i2c_ptr[MOCK_I2C_NUM_ADDRS];
```

In `mock_bsp_reset()`, next to the existing `memset` calls, add:

```c
    memset(g_mock_i2c_words, 0, sizeof(g_mock_i2c_words));
    memset(g_mock_i2c_word_addressed, 0, sizeof(g_mock_i2c_word_addressed));
```

Add the accessors (for example next to `mock_bsp_i2c_set_regs`):

```c
void mock_bsp_i2c_set_word(uint8_t addr, uint8_t reg, uint16_t value) {
    if (addr < MOCK_I2C_NUM_ADDRS) {
        g_mock_i2c_word_addressed[addr] = true;
        g_mock_i2c_words[addr][reg] = value;
    }
}

uint16_t mock_bsp_i2c_get_word(uint8_t addr, uint8_t reg) {
    if (addr < MOCK_I2C_NUM_ADDRS) {
        return g_mock_i2c_words[addr][reg];
    }

    return 0u;
}
```

Replace `mock_bus_write()` with a version that branches on the flag:

```c
static rov_status_t mock_bus_write(void *ctx, uint8_t addr, const uint8_t *data, uint16_t len) {
    (void)ctx;

    if (addr >= MOCK_I2C_NUM_ADDRS || data == NULL || len == 0U) {
        return ROV_ERR_INVALID_ARG;
    }

    if (g_mock_i2c_fault != ROV_OK) {
        return g_mock_i2c_fault;
    }

    /* The first byte is the register pointer; the rest are payload bytes. */
    g_mock_i2c_ptr[addr] = data[0];

    if (g_mock_i2c_word_addressed[addr]) {
        /*
         * The payload is a sequence of big-endian 16-bit words, one per
         * register. A pointer-only write (len == 1) has no payload.
         */
        for (uint16_t i = 1U; (uint16_t)(i + 1U) < len; i += 2U) {
            uint8_t reg = (uint8_t)(g_mock_i2c_ptr[addr] + (uint8_t)((i - 1U) / 2U));

            g_mock_i2c_words[addr][reg] = (uint16_t)(((uint16_t)data[i] << 8) | (uint16_t)data[i + 1U]);
        }

        return ROV_OK;
    }

    for (uint16_t i = 1U; i < len; i++) {
        uint8_t reg = (uint8_t)(g_mock_i2c_ptr[addr] + (uint8_t)(i - 1U));

        g_mock_i2c_regs[addr][reg] = data[i];
    }

    return ROV_OK;
}
```

Replace `mock_bus_read()` likewise:

```c
static rov_status_t mock_bus_read(void *ctx, uint8_t addr, uint8_t *data, uint16_t len) {
    (void)ctx;

    if (addr >= MOCK_I2C_NUM_ADDRS || data == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    if (g_mock_i2c_fault != ROV_OK) {
        return g_mock_i2c_fault;
    }

    if (g_mock_i2c_word_addressed[addr]) {
        /* Two bytes per register, MSB first. */
        for (uint16_t i = 0U; i < len; i++) {
            uint8_t reg = (uint8_t)(g_mock_i2c_ptr[addr] + (uint8_t)(i / 2U));
            uint16_t word = g_mock_i2c_words[addr][reg];

            data[i] = ((i % 2U) == 0U) ? (uint8_t)(word >> 8) : (uint8_t)(word & 0xFFu);
        }

        return ROV_OK;
    }

    /*
     * The register index is a uint8_t and the register file is 256 wide, so the
     * running pointer wraps within the same address space the driver sees.
     */
    for (uint16_t i = 0U; i < len; i++) {
        data[i] = g_mock_i2c_regs[addr][(uint8_t)(g_mock_i2c_ptr[addr] + (uint8_t)i)];
    }

    return ROV_OK;
}
```

- [ ] **Step 5: Run the test and the full non-target suite**

```powershell
$env:PATH = "C:\msys64\ucrt64\bin;" + $env:PATH
cmake --build build-native
ctest --test-dir build-native -E "^target_" --output-on-failure
```

Expected: `test_mock_i2c_bus` passes, including the new word round trip; all 36 non-target tests green (the byte path is unchanged, so `test_driver_bme280` and `test_node1_pi_shield` must still pass).

- [ ] **Step 6: Update the guide row**

In `docs/sil_simulation_guide.md`, change row 12:

```
| 12 | `test_mock_i2c_bus` | Mock BSP I2C bus, byte and 16-bit word registers, fault injection, and presence probe |
```

- [ ] **Step 7: Commit**

```powershell
git add tests/mocks/mock_bsp.h tests/mocks/mock_bsp.c tests/test_mock_i2c_bus.c docs/sil_simulation_guide.md
git commit -m "test(mocks): add 16-bit word registers to the mock I2C bus"
```

---

### Task 2: TMP1075 real driver and device model

**Files:**
- Modify: `drivers/include/tmp1075.h`
- Modify: `drivers/src/tmp1075.c`
- Create: `tests/mocks/mock_tmp1075.h`
- Create: `tests/mocks/mock_tmp1075.c`
- Modify: `tests/CMakeLists.txt` (add `mocks/mock_tmp1075.c` to the `rov_mocks` object library)
- Modify: `tests/test_driver_tmp1075.c`
- Modify: `tests/mocks/mock_sensors.h` (drop the `tmp1075.h` include and the two prototypes)
- Modify: `tests/mocks/mock_sensors.c` (drop the `g_mock_tmp1075` struct, its reset lines, and the two functions)
- Modify: `docs/sil_simulation_guide.md` (row 21 description)

**Interfaces:**
- Consumes: `rov_i2c_bus_t`, `rov_i2c_mem_read()` from `shared/include/rov_i2c.h`; `ROV_OK`, `ROV_ERROR`, `ROV_ERR_INVALID_ARG` from `shared/include/rov_types.h`; `mock_bsp_i2c_set_word()` from Task 1.
- Produces: `tmp1075_init(tmp1075_dev_t *dev, const rov_i2c_bus_t *bus, uint8_t i2c_addr)` and `tmp1075_read_temperature(tmp1075_dev_t *dev)`, both returning `rov_status_t`. `tmp1075_dev_t` gains a `const rov_i2c_bus_t *bus;` first field.
- Produces (host only): `mock_tmp1075_set_temperature(uint8_t addr, float temp_c)`.

- [ ] **Step 1: Write the failing test**

Replace the body of `tests/test_driver_tmp1075.c` with:

```c
/**
 * @file test_driver_tmp1075.c
 * @brief Unit tests for TI TMP1075 Temperature Sensor Driver.
 * @organization Purdue ROV
 */

#include "tmp1075.h"
#include "mocks/mock_bsp.h"
#include "mocks/mock_tmp1075.h"
#include <assert.h>
#include <stdio.h>

void test_tmp1075_driver(void) {
    tmp1075_dev_t dev;
    const rov_i2c_bus_t *bus = bsp_i2c_bus();

    mock_bsp_reset();

    /* Negative test: NULL pointers. */
    assert(tmp1075_init(NULL, bus, 0x48) == ROV_ERR_INVALID_ARG);
    assert(tmp1075_init(&dev, NULL, 0x48) == ROV_ERR_INVALID_ARG);
    assert(tmp1075_read_temperature(NULL) == ROV_ERR_INVALID_ARG);

    /* Valid initialization stores the transport and address. */
    assert(tmp1075_init(&dev, bus, 0x48) == ROV_OK);
    assert(dev.i2c_addr == 0x48);

    /* Positive temperature round-trips through the datasheet conversion. */
    mock_tmp1075_set_temperature(0x48, 42.5f);
    assert(tmp1075_read_temperature(&dev) == ROV_OK);
    assert(dev.temperature_c > 42.4f && dev.temperature_c < 42.6f);

    /*
     * Negative temperature: the 12-bit two's complement field must sign-extend.
     * 12-bit two's complement of -10.0 C is 0xD80; left-justified that is
     * 0xD800, which is a negative int16_t. Dropping the pad bits after the
     * sign-extension gives -10 C, not +245 C.
     */
    mock_tmp1075_set_temperature(0x48, -10.0f);
    assert(tmp1075_read_temperature(&dev) == ROV_OK);
    assert(dev.temperature_c > -10.1f && dev.temperature_c < -9.9f);

    /* A transport fault propagates and leaves the previous reading intact. */
    mock_tmp1075_set_temperature(0x48, 25.0f);
    assert(tmp1075_read_temperature(&dev) == ROV_OK);
    float held = dev.temperature_c;

    mock_bsp_i2c_set_fault(ROV_TIMEOUT);
    assert(tmp1075_read_temperature(&dev) == ROV_TIMEOUT);
    assert(dev.temperature_c == held);
    mock_bsp_i2c_set_fault(ROV_OK);

    printf("[PASS] test_tmp1075_driver\n");
}

int main(void) {
    printf("Running TMP1075 Driver Unit Tests...\n");
    test_tmp1075_driver();
    printf("All TMP1075 Driver Tests Passed Successfully!\n");
    return 0;
}
```

- [ ] **Step 2: Run the test and confirm it fails to build**

```powershell
$env:PATH = "C:\msys64\ucrt64\bin;" + $env:PATH
cmake --build build-native --target test_driver_tmp1075
```

Expected: compile error. `tmp1075_init` takes three arguments now but the header still declares two, and `mocks/mock_tmp1075.h` does not exist yet. RED.

- [ ] **Step 3: Change the driver header**

Replace `drivers/include/tmp1075.h` with:

```c
/**
 * @file tmp1075.h
 * @brief TI TMP1075NDRLR PCB Copper Digital Temperature Sensor I2C Driver.
 * @organization Purdue ROV
 */

#ifndef TMP1075_H
#define TMP1075_H

#include "rov_i2c.h"
#include "rov_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const rov_i2c_bus_t *bus; /* injected transport; set by tmp1075_init() */
    uint8_t i2c_addr;
    float temperature_c;
} tmp1075_dev_t;

/**
 * @brief Bind a TMP1075 at @p i2c_addr to @p bus.
 *
 * The board part is the TMP1075N orderable, which has no device-ID register
 * (the 0x0F ID register exists only on non-N parts), so init cannot verify the
 * chip and validates its arguments only. Presence is proven by a successful
 * temperature read.
 */
rov_status_t tmp1075_init(tmp1075_dev_t *dev, const rov_i2c_bus_t *bus, uint8_t i2c_addr);

rov_status_t tmp1075_read_temperature(tmp1075_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* TMP1075_H */
```

- [ ] **Step 4: Implement the driver**

Replace `drivers/src/tmp1075.c` with:

```c
/**
 * @file tmp1075.c
 * @brief TI TMP1075 Temperature Sensor Driver Implementation.
 * @organization Purdue ROV
 */

#include "tmp1075.h"

#include <string.h>

/* TMP1075 register pointer values. */
#define TMP1075_REG_TEMPERATURE 0x00u

/* 12-bit two's complement, left-justified in 16 bits, 0.0625 C per count. */
#define TMP1075_TEMP_LSB_C 0.0625f

rov_status_t tmp1075_init(tmp1075_dev_t *dev, const rov_i2c_bus_t *bus, uint8_t i2c_addr) {
    if (dev == NULL || bus == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(tmp1075_dev_t));
    dev->bus = bus;
    dev->i2c_addr = i2c_addr;

    return ROV_OK;
}

rov_status_t tmp1075_read_temperature(tmp1075_dev_t *dev) {
    uint8_t raw[2] = {0u, 0u};
    int16_t raw16;
    int16_t t12;

    if (dev == NULL || dev->bus == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    rov_status_t status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, TMP1075_REG_TEMPERATURE, raw, sizeof(raw));

    if (status != ROV_OK) {
        return status;
    }

    /*
     * The temperature register is the 12-bit two's complement reading in bits
     * 15:4, with bits 3:0 reading zero. Sign-extend the 16-bit word first, then
     * drop the four pad bits, so a negative reading stays negative.
     */
    raw16 = (int16_t)(((uint16_t)raw[0] << 8) | (uint16_t)raw[1]);
    t12 = (int16_t)(raw16 >> 4);
    dev->temperature_c = (float)t12 * TMP1075_TEMP_LSB_C;

    return ROV_OK;
}
```

- [ ] **Step 5: Add the host device model**

Create `tests/mocks/mock_tmp1075.h`:

```c
/**
 * @file mock_tmp1075.h
 * @brief TMP1075 device model for the host mock I2C register file.
 * @organization Purdue ROV
 *
 * Places a TMP1075 at a chosen temperature by writing the raw register word a
 * real part would have produced. The conversion is a single linear scale, so
 * the inverse is exact arithmetic rather than the BME280 model's bisection.
 */

#ifndef MOCK_TMP1075_H
#define MOCK_TMP1075_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Write the temperature register of the TMP1075 at @p addr.
 * @param addr 7-bit I2C address, 0x48..0x4F.
 * @param temp_c Target temperature in degrees Celsius.
 */
void mock_tmp1075_set_temperature(uint8_t addr, float temp_c);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_TMP1075_H */
```

Create `tests/mocks/mock_tmp1075.c`:

```c
/**
 * @file mock_tmp1075.c
 * @brief TMP1075 device model for the host mock I2C register file.
 * @organization Purdue ROV
 */

#include "mock_tmp1075.h"

#include "mock_bsp.h"

/* Register map, matching drivers/src/tmp1075.c. */
#define TMP1075_REG_TEMPERATURE 0x00u

/* 12-bit two's complement, left-justified, 0.0625 C per count. */
#define TMP1075_TEMP_LSB_C 0.0625f

void mock_tmp1075_set_temperature(uint8_t addr, float temp_c) {
    int16_t counts;

    if (temp_c >= 0.0f) {
        counts = (int16_t)((temp_c / TMP1075_TEMP_LSB_C) + 0.5f);
    } else {
        counts = (int16_t)((temp_c / TMP1075_TEMP_LSB_C) - 0.5f);
    }

    /* Left-justify the 12-bit reading into the top nibble-aligned position. */
    mock_bsp_i2c_set_word(addr, TMP1075_REG_TEMPERATURE, (uint16_t)((uint16_t)counts << 4));
}
```

- [ ] **Step 6: Register the model in the mock object library**

In `tests/CMakeLists.txt`, add the new source to `rov_mocks`:

```cmake
add_library(rov_mocks OBJECT
    mocks/mock_can.c
    mocks/mock_bsp.c
    mocks/mock_bme280.c
    mocks/mock_sensors.c
    mocks/mock_tmp1075.c
    mocks/mock_physics.c
    harness/sil_clock.c
)
```

- [ ] **Step 7: Delete the TMP1075 value hook**

In `tests/mocks/mock_sensors.h`: remove `#include "tmp1075.h"` and the block

```c
/**
 * @brief Inject TMP1075 PCB temperature.
 */
void mock_sensors_set_tmp1075(float temp_c);
bool mock_sensors_get_tmp1075(float *temp_c);
```

In `tests/mocks/mock_sensors.c`: remove the struct

```c
static struct {
    float temp_c;
    bool valid;
} g_mock_tmp1075;
```

remove the two reset lines

```c
    g_mock_tmp1075.temp_c = 25.0f;
    g_mock_tmp1075.valid = true;
```

and remove both functions

```c
void mock_sensors_set_tmp1075(float temp_c) {
    g_mock_tmp1075.temp_c = temp_c;
    g_mock_tmp1075.valid = true;
}

bool mock_sensors_get_tmp1075(float *temp_c) {
    if (!g_mock_tmp1075.valid)
        return false;
    if (temp_c)
        *temp_c = g_mock_tmp1075.temp_c;
    return true;
}
```

- [ ] **Step 8: Run the test and the full non-target suite**

```powershell
$env:PATH = "C:\msys64\ucrt64\bin;" + $env:PATH
cmake --build build-native
ctest --test-dir build-native -E "^target_" --output-on-failure
```

Expected: `test_driver_tmp1075` passes; 36/36 non-target tests green. If a `target_*` build fails to link because `tests/hardware/fakes/fake_hal.c` still defines `mock_sensors_get_tmp1075`, that is expected and is resolved in Task 5; confirm the failure is confined to a `target_*` build, which is excluded here.

- [ ] **Step 9: Update the guide row**

In `docs/sil_simulation_guide.md`, change row 21:

```
| 21 | `test_driver_tmp1075` | TMP1075 temperature register read, sign extension, and fault propagation |
```

- [ ] **Step 10: Commit**

```powershell
git add drivers/include/tmp1075.h drivers/src/tmp1075.c tests/mocks/mock_tmp1075.h tests/mocks/mock_tmp1075.c tests/CMakeLists.txt tests/test_driver_tmp1075.c tests/mocks/mock_sensors.h tests/mocks/mock_sensors.c docs/sil_simulation_guide.md
git commit -m "refactor(drivers): read TMP1075 over the injected I2C bus"
```

---

### Task 3: INA237 real driver, device model, and node 1 ripple

**Files:**
- Modify: `drivers/include/ina237.h`
- Modify: `drivers/src/ina237.c`
- Create: `tests/mocks/mock_ina237.h`
- Create: `tests/mocks/mock_ina237.c`
- Modify: `tests/CMakeLists.txt` (add `mocks/mock_ina237.c` to `rov_mocks`)
- Modify: `tests/test_driver_ina237.c`
- Modify: `nodes/node1_pi_shield/Core/Src/app.c` (one call site)
- Modify: `tests/test_node1_pi_shield.c` (replace the value hook with the device model)
- Modify: `tests/mocks/mock_physics.c` (write the INA237 register image instead of the value hook)
- Modify: `docs/sil_simulation_guide.md` (row 20 description)

**Interfaces:**
- Consumes: `rov_i2c_bus_t`, `rov_i2c_mem_read()`, `rov_i2c_mem_write()`; `mock_bsp_i2c_set_word()` from Task 1.
- Produces: `ina237_init(ina237_dev_t *dev, const rov_i2c_bus_t *bus, uint8_t i2c_addr, float shunt_resistor_ohms)` and `ina237_read_power(ina237_dev_t *dev)`. `ina237_dev_t` gains a `const rov_i2c_bus_t *bus;` first field.
- Produces (host only): `mock_ina237_set_reading(uint8_t addr, float bus_voltage_v, float current_a, float shunt_resistor_ohms)` and `mock_ina237_set_die_temperature(uint8_t addr, float temp_c)`.
- Note: the weak `mock_sensors_get_ina226` symbol is still defined in `drivers/src/ina226.c` and still consumed by INA226, so it stays in `tests/mocks/mock_sensors.c` until Task 4. Only INA237's own copy of the weak definition is removed here.

- [ ] **Step 1: Write the failing test**

Replace the body of `tests/test_driver_ina237.c` with:

```c
/**
 * @file test_driver_ina237.c
 * @brief Unit tests for TI INA237 High-Precision Power Monitor Driver.
 * @organization Purdue ROV
 */

#include "ina237.h"
#include "mocks/mock_bsp.h"
#include "mocks/mock_ina237.h"
#include <assert.h>
#include <stdio.h>

#define INA237_TEST_ADDR 0x40u
#define INA237_TEST_SHUNT_OHMS 0.001f

void test_ina237_driver(void) {
    ina237_dev_t dev;
    const rov_i2c_bus_t *bus = bsp_i2c_bus();

    mock_bsp_reset();

    /* Negative test: NULL pointers and a non-positive shunt. */
    assert(ina237_init(NULL, bus, INA237_TEST_ADDR, INA237_TEST_SHUNT_OHMS) == ROV_ERR_INVALID_ARG);
    assert(ina237_init(&dev, NULL, INA237_TEST_ADDR, INA237_TEST_SHUNT_OHMS) == ROV_ERR_INVALID_ARG);
    assert(ina237_init(&dev, bus, INA237_TEST_ADDR, 0.0f) == ROV_ERR_INVALID_ARG);
    assert(ina237_init(&dev, bus, INA237_TEST_ADDR, -0.01f) == ROV_ERR_INVALID_ARG);
    assert(ina237_read_power(NULL) == ROV_ERR_INVALID_ARG);

    /* Valid initialization stores the transport, address, and shunt. */
    assert(ina237_init(&dev, bus, INA237_TEST_ADDR, INA237_TEST_SHUNT_OHMS) == ROV_OK);
    assert(dev.i2c_addr == INA237_TEST_ADDR);
    assert(dev.shunt_resistor_ohms > 0.0009f && dev.shunt_resistor_ohms < 0.0011f);

    /*
     * Datasheet round trip at the default ADCRANGE = 0:
     *   VBUS   = 5.2 V  -> 5.2 / 3.125 mV  = 1664 counts
     *   VSHUNT = 3.5 A * 1 mOhm = 3.5 mV -> 3.5 mV / 5 uV = 700 counts
     */
    mock_ina237_set_reading(INA237_TEST_ADDR, 5.2f, 3.5f, INA237_TEST_SHUNT_OHMS);
    mock_ina237_set_die_temperature(INA237_TEST_ADDR, 25.0f);

    assert(ina237_read_power(&dev) == ROV_OK);
    assert(dev.bus_voltage_v > 5.19f && dev.bus_voltage_v < 5.21f);
    assert(dev.shunt_current_a > 3.49f && dev.shunt_current_a < 3.51f);
    assert(dev.power_w > 18.19f && dev.power_w < 18.21f);
    assert(dev.die_temp_c > 24.9f && dev.die_temp_c < 25.1f);

    /* A transport fault propagates and leaves the previous reading intact. */
    float held_voltage = dev.bus_voltage_v;
    mock_bsp_i2c_set_fault(ROV_TIMEOUT);
    assert(ina237_read_power(&dev) == ROV_TIMEOUT);
    assert(dev.bus_voltage_v == held_voltage);
    mock_bsp_i2c_set_fault(ROV_OK);

    printf("[PASS] test_ina237_driver\n");
}

int main(void) {
    printf("Running INA237 Driver Unit Tests...\n");
    test_ina237_driver();
    printf("All INA237 Driver Tests Passed Successfully!\n");
    return 0;
}
```

- [ ] **Step 2: Run the test and confirm it fails to build**

```powershell
$env:PATH = "C:\msys64\ucrt64\bin;" + $env:PATH
cmake --build build-native --target test_driver_ina237
```

Expected: compile error on the four-argument `ina237_init` and the missing `mocks/mock_ina237.h`. RED.

- [ ] **Step 3: Change the driver header**

Replace `drivers/include/ina237.h` with:

```c
/**
 * @file ina237.h
 * @brief TI INA237AIDGSR High-Precision Power Monitor I2C Driver.
 * @organization Purdue ROV
 */

#ifndef INA237_H
#define INA237_H

#include "rov_i2c.h"
#include "rov_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const rov_i2c_bus_t *bus; /* injected transport; set by ina237_init() */
    uint8_t i2c_addr;
    float shunt_resistor_ohms;
    float bus_voltage_v;
    float shunt_current_a;
    float power_w;
    float die_temp_c;
} ina237_dev_t;

/**
 * @brief Bind an INA237 at @p i2c_addr to @p bus and configure its ADC.
 *
 * The INA237 has no device-ID register (the 0x3F ID register exists on the
 * INA238 only), so init cannot verify the chip and validates its arguments
 * only. It writes ADC_CONFIG to select 16-sample averaging; the power-on
 * reset (0xFB68) is already continuous at 1052 us, averaging one sample.
 */
rov_status_t ina237_init(ina237_dev_t *dev, const rov_i2c_bus_t *bus, uint8_t i2c_addr, float shunt_resistor_ohms);

rov_status_t ina237_read_power(ina237_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* INA237_H */
```

- [ ] **Step 4: Implement the driver**

Replace `drivers/src/ina237.c` with:

```c
/**
 * @file ina237.c
 * @brief TI INA237AIDGSR Power Monitor Driver Implementation.
 * @organization Purdue ROV
 */

#include "ina237.h"

#include <string.h>

/* INA237 register pointer values. */
#define INA237_REG_ADC_CONFIG 0x01u
#define INA237_REG_VSHUNT     0x04u
#define INA237_REG_VBUS       0x05u
#define INA237_REG_DIE_TEMP   0x06u

/*
 * ADC_CONFIG: continuous bus + shunt + temperature (MODE = 1111), 1052 us
 * conversion time on all three fields, 16-sample average.
 *
 * The power-on reset is 0xFB68: the same continuous 1052 us settings but with
 * 1-sample averaging. This write keeps those settings and raises the average
 * to 16 samples for a lower-noise reading; it is not what leaves shutdown.
 */
#define INA237_ADC_CONFIG_CONTINUOUS 0xFB6Au

/* Default ADCRANGE = 0 conversion factors. */
#define INA237_SHUNT_VOLTAGE_LSB_V 0.000005f /* 5 uV per count */
#define INA237_BUS_VOLTAGE_LSB_V   0.003125f /* 3.125 mV per count */
#define INA237_DIE_TEMP_LSB_C      0.125f    /* 0.125 C per count */

static rov_status_t ina237_configure_adc(ina237_dev_t *dev) {
    const uint8_t config[2] = {(uint8_t)(INA237_ADC_CONFIG_CONTINUOUS >> 8),
                               (uint8_t)(INA237_ADC_CONFIG_CONTINUOUS & 0xFFu)};

    return rov_i2c_mem_write(dev->bus, dev->i2c_addr, INA237_REG_ADC_CONFIG, config, sizeof(config));
}

rov_status_t ina237_init(ina237_dev_t *dev, const rov_i2c_bus_t *bus, uint8_t i2c_addr, float shunt_resistor_ohms) {
    if (dev == NULL || bus == NULL || shunt_resistor_ohms <= 0.0f) {
        return ROV_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(ina237_dev_t));
    dev->bus = bus;
    dev->i2c_addr = i2c_addr;
    dev->shunt_resistor_ohms = shunt_resistor_ohms;

    return ina237_configure_adc(dev);
}

rov_status_t ina237_read_power(ina237_dev_t *dev) {
    uint8_t vbus_raw[2] = {0u, 0u};
    uint8_t vshunt_raw[2] = {0u, 0u};
    uint8_t temp_raw[2] = {0u, 0u};
    uint16_t vbus;
    int16_t vshunt;
    int16_t temp;

    if (dev == NULL || dev->bus == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    rov_status_t status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, INA237_REG_VBUS, vbus_raw, sizeof(vbus_raw));

    if (status != ROV_OK) {
        return status;
    }

    status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, INA237_REG_VSHUNT, vshunt_raw, sizeof(vshunt_raw));

    if (status != ROV_OK) {
        return status;
    }

    status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, INA237_REG_DIE_TEMP, temp_raw, sizeof(temp_raw));

    if (status != ROV_OK) {
        return status;
    }

    vbus = (uint16_t)(((uint16_t)vbus_raw[0] << 8) | (uint16_t)vbus_raw[1]);
    vshunt = (int16_t)(((uint16_t)vshunt_raw[0] << 8) | (uint16_t)vshunt_raw[1]);
    temp = (int16_t)(((uint16_t)temp_raw[0] << 8) | (uint16_t)temp_raw[1]);

    dev->bus_voltage_v = (float)vbus * INA237_BUS_VOLTAGE_LSB_V;
    dev->shunt_current_a = ((float)vshunt * INA237_SHUNT_VOLTAGE_LSB_V) / dev->shunt_resistor_ohms;
    dev->power_w = dev->bus_voltage_v * dev->shunt_current_a;

    /* Die temperature is 12-bit two's complement, left-justified in 16 bits. */
    dev->die_temp_c = (float)(int16_t)(temp >> 4) * INA237_DIE_TEMP_LSB_C;

    return ROV_OK;
}
```

- [ ] **Step 5: Add the host device model**

Create `tests/mocks/mock_ina237.h`:

```c
/**
 * @file mock_ina237.h
 * @brief INA237 device model for the host mock I2C register file.
 * @organization Purdue ROV
 *
 * Places an INA237 at chosen bus voltage / shunt current / die temperature by
 * writing the raw register words a real part would have produced. The
 * conversion is linear, so the inverse is exact arithmetic.
 */

#ifndef MOCK_INA237_H
#define MOCK_INA237_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Write the VBUS and VSHUNT registers of the INA237 at @p addr.
 * @param addr 7-bit I2C address, 0x40..0x4F.
 * @param bus_voltage_v Target bus voltage in volts.
 * @param current_a Target shunt current in amperes.
 * @param shunt_resistor_ohms Shunt value the driver under test is configured with.
 */
void mock_ina237_set_reading(uint8_t addr, float bus_voltage_v, float current_a, float shunt_resistor_ohms);

/**
 * @brief Write the DIETEMP register of the INA237 at @p addr.
 * @param addr 7-bit I2C address.
 * @param temp_c Target die temperature in degrees Celsius.
 */
void mock_ina237_set_die_temperature(uint8_t addr, float temp_c);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_INA237_H */
```

Create `tests/mocks/mock_ina237.c`:

```c
/**
 * @file mock_ina237.c
 * @brief INA237 device model for the host mock I2C register file.
 * @organization Purdue ROV
 */

#include "mock_ina237.h"

#include "mock_bsp.h"

/* Register map, matching drivers/src/ina237.c. */
#define INA237_REG_VSHUNT   0x04u
#define INA237_REG_VBUS     0x05u
#define INA237_REG_DIE_TEMP 0x06u

/* Default ADCRANGE = 0 conversion factors. */
#define INA237_SHUNT_VOLTAGE_LSB_V 0.000005f /* 5 uV per count */
#define INA237_BUS_VOLTAGE_LSB_V   0.003125f /* 3.125 mV per count */
#define INA237_DIE_TEMP_LSB_C      0.125f    /* 0.125 C per count */

static int16_t round_to_i16(float value) {
    if (value >= 0.0f) {
        return (int16_t)(value + 0.5f);
    }

    return (int16_t)(value - 0.5f);
}

void mock_ina237_set_reading(uint8_t addr, float bus_voltage_v, float current_a, float shunt_resistor_ohms) {
    uint16_t vbus_counts = (uint16_t)round_to_i16(bus_voltage_v / INA237_BUS_VOLTAGE_LSB_V);
    int16_t vshunt_counts = round_to_i16((current_a * shunt_resistor_ohms) / INA237_SHUNT_VOLTAGE_LSB_V);

    mock_bsp_i2c_set_word(addr, INA237_REG_VBUS, vbus_counts);
    mock_bsp_i2c_set_word(addr, INA237_REG_VSHUNT, (uint16_t)vshunt_counts);
}

void mock_ina237_set_die_temperature(uint8_t addr, float temp_c) {
    int16_t counts = round_to_i16(temp_c / INA237_DIE_TEMP_LSB_C);

    /* Die temperature is 12-bit, left-justified in the 16-bit register. */
    mock_bsp_i2c_set_word(addr, INA237_REG_DIE_TEMP, (uint16_t)((uint16_t)counts << 4));
}
```

- [ ] **Step 6: Register the model in the mock object library**

In `tests/CMakeLists.txt`, add `mocks/mock_ina237.c` to the `rov_mocks` source list.

- [ ] **Step 7: Update the node 1 call site**

In `nodes/node1_pi_shield/Core/Src/app.c`, change the init call to pass the bus:

```c
    bme280_init(&g_bme280_dev, bsp_i2c_bus());
    ina237_init(&g_ina237_dev, bsp_i2c_bus(), 0x40, 0.001f);
```

- [ ] **Step 8: Replace the node 1 SIL value hook with the device model**

In `tests/test_node1_pi_shield.c`, add the include near the other mock includes:

```c
#include "mocks/mock_ina237.h"
```

and in `test_node1_nominal_telemetry()`, replace

```c
    mock_sensors_set_ina226(5.21f, 1.35f);
```

with

```c
    mock_ina237_set_reading(0x40u, 5.21f, 1.35f, 0.001f);
```

- [ ] **Step 9: Replace the physics value hook with the INA237 register image**

In `tests/mocks/mock_physics.c`, add `#include "mock_ina237.h"` and replace

```c
        /* INA226 12V bus monitor & INA237 48V tether monitor */
        float total_12v_cur = g_phys.brick_currents_a[0] + g_phys.brick_currents_a[1] + g_phys.brick_currents_a[2] +
                              g_phys.brick_currents_a[3];
        mock_sensors_set_ina226(g_phys.brick_voltages_v[0], total_12v_cur);
```

with

```c
        /* INA237 48V tether monitor (node 1 reads it at 0x40 with a 1 mOhm shunt) */
        float total_12v_cur = g_phys.brick_currents_a[0] + g_phys.brick_currents_a[1] + g_phys.brick_currents_a[2] +
                              g_phys.brick_currents_a[3];
        mock_ina237_set_reading(0x40u, g_phys.brick_voltages_v[0], total_12v_cur, 0.001f);
```

(The INA226 half of this block is moved in Task 4; leave the `mock_sensors_set_ina226` call out for now because INA226 still owns that symbol and its own model does not exist until Task 4. Confirm `mock_sensors_set_ina226` is still declared and defined after this edit; it is, until Task 4.)

- [ ] **Step 10: Run the test and the full non-target suite**

```powershell
$env:PATH = "C:\msys64\ucrt64\bin;" + $env:PATH
cmake --build build-native
ctest --test-dir build-native -E "^target_" --output-on-failure
```

Expected: `test_driver_ina237`, `test_node1_pi_shield`, `test_mock_physics`, `test_multi_node_bus`, `test_sil_*`, and `test_services` all pass; 36/36 non-target tests green.

- [ ] **Step 11: Update the guide row**

In `docs/sil_simulation_guide.md`, change row 20:

```
| 20 | `test_driver_ina237` | INA237 bus/shunt/die-temperature register reads, conversion, and fault propagation |
```

- [ ] **Step 12: Commit**

```powershell
git add drivers/include/ina237.h drivers/src/ina237.c tests/mocks/mock_ina237.h tests/mocks/mock_ina237.c tests/CMakeLists.txt tests/test_driver_ina237.c nodes/node1_pi_shield/Core/Src/app.c tests/test_node1_pi_shield.c tests/mocks/mock_physics.c docs/sil_simulation_guide.md
git commit -m "refactor(drivers): read INA237 over the injected I2C bus"
```

---

### Task 4: INA226 real driver, device model, and value-hook removal

**Files:**
- Modify: `drivers/include/ina226.h`
- Modify: `drivers/src/ina226.c`
- Create: `tests/mocks/mock_ina226.h`
- Create: `tests/mocks/mock_ina226.c`
- Modify: `tests/CMakeLists.txt` (add `mocks/mock_ina226.c` to `rov_mocks`)
- Modify: `tests/test_driver_ina226.c`
- Modify: `tests/mocks/mock_physics.c` (add the INA226 register image)
- Modify: `tests/mocks/mock_sensors.h` (drop the `ina226.h` / `ina237.h` includes and the two prototypes)
- Modify: `tests/mocks/mock_sensors.c` (drop the `g_mock_ina226` struct, its reset lines, and the two functions)
- Modify: `docs/sil_simulation_guide.md` (row 15 description)

**Interfaces:**
- Consumes: `rov_i2c_bus_t`, `rov_i2c_mem_read()`; `mock_bsp_i2c_set_word()` from Task 1.
- Produces: `ina226_init(ina226_dev_t *dev, const rov_i2c_bus_t *bus, uint8_t i2c_addr, float shunt_resistor_ohms)` and `ina226_read_power(ina226_dev_t *dev)`. `ina226_dev_t` gains a `const rov_i2c_bus_t *bus;` first field.
- Produces (host only): `mock_ina226_present(uint8_t addr)` and `mock_ina226_set_reading(uint8_t addr, float bus_voltage_v, float current_a, float shunt_resistor_ohms)`.
- After this task, no source references `mock_sensors_get_ina226` except `tests/hardware/fakes/fake_hal.c`, which Task 5 removes.

- [ ] **Step 1: Write the failing test**

Replace the body of `tests/test_driver_ina226.c` with:

```c
/**
 * @file test_driver_ina226.c
 * @brief Unit tests for TI INA226 Power Monitor Driver.
 * @organization Purdue ROV
 */

#include "ina226.h"
#include "mocks/mock_bsp.h"
#include "mocks/mock_ina226.h"
#include <assert.h>
#include <stdio.h>

#define INA226_TEST_ADDR 0x40u
#define INA226_TEST_SHUNT_OHMS 0.002f

void test_ina226_driver(void) {
    ina226_dev_t dev;
    const rov_i2c_bus_t *bus = bsp_i2c_bus();

    mock_bsp_reset();

    /* Negative test: NULL pointers and a non-positive shunt. */
    assert(ina226_init(NULL, bus, INA226_TEST_ADDR, INA226_TEST_SHUNT_OHMS) == ROV_ERR_INVALID_ARG);
    assert(ina226_init(&dev, NULL, INA226_TEST_ADDR, INA226_TEST_SHUNT_OHMS) == ROV_ERR_INVALID_ARG);
    assert(ina226_init(&dev, bus, INA226_TEST_ADDR, 0.0f) == ROV_ERR_INVALID_ARG);
    assert(ina226_init(&dev, bus, INA226_TEST_ADDR, -0.05f) == ROV_ERR_INVALID_ARG);
    assert(ina226_read_power(NULL) == ROV_ERR_INVALID_ARG);

    /* Init probes the die ID; with an empty register file it must fail. */
    assert(ina226_init(&dev, bus, INA226_TEST_ADDR, INA226_TEST_SHUNT_OHMS) != ROV_OK);

    /* Place the part, then init succeeds and stores the transport. */
    mock_ina226_present(INA226_TEST_ADDR);
    assert(ina226_init(&dev, bus, INA226_TEST_ADDR, INA226_TEST_SHUNT_OHMS) == ROV_OK);
    assert(dev.i2c_addr == INA226_TEST_ADDR);

    /*
     * Datasheet round trip:
     *   VBUS   = 5.2 V            -> 5.2 / 1.25 mV = 4160 counts
     *   VSHUNT = 2.5 A * 2 mOhm   = 5 mV -> 5 mV / 2.5 uV = 2000 counts
     */
    mock_ina226_set_reading(INA226_TEST_ADDR, 5.2f, 2.5f, INA226_TEST_SHUNT_OHMS);

    assert(ina226_read_power(&dev) == ROV_OK);
    assert(dev.voltage_v > 5.19f && dev.voltage_v < 5.21f);
    assert(dev.current_a > 2.49f && dev.current_a < 2.51f);
    assert(dev.power_w > 12.99f && dev.power_w < 13.01f);

    /* A transport fault propagates and leaves the previous reading intact. */
    float held_voltage = dev.voltage_v;
    mock_bsp_i2c_set_fault(ROV_TIMEOUT);
    assert(ina226_read_power(&dev) == ROV_TIMEOUT);
    assert(dev.voltage_v == held_voltage);
    mock_bsp_i2c_set_fault(ROV_OK);

    printf("[PASS] test_ina226_driver\n");
}

int main(void) {
    printf("Running INA226 Driver Unit Tests...\n");
    test_ina226_driver();
    printf("All INA226 Driver Tests Passed Successfully!\n");
    return 0;
}
```

- [ ] **Step 2: Run the test and confirm it fails to build**

```powershell
$env:PATH = "C:\msys64\ucrt64\bin;" + $env:PATH
cmake --build build-native --target test_driver_ina226
```

Expected: compile error on the four-argument `ina226_init` and the missing `mocks/mock_ina226.h`. RED.

- [ ] **Step 3: Change the driver header**

Replace `drivers/include/ina226.h` with:

```c
/**
 * @file ina226.h
 * @brief TI INA226 Power & Shunt Current Monitor I2C Driver.
 * @organization Purdue ROV
 */

#ifndef INA226_H
#define INA226_H

#include "rov_i2c.h"
#include "rov_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const rov_i2c_bus_t *bus; /* injected transport; set by ina226_init() */
    uint8_t i2c_addr;
    float shunt_resistor_ohms;
    float voltage_v;
    float current_a;
    float power_w;
} ina226_dev_t;

/**
 * @brief Bind an INA226 at @p i2c_addr to @p bus.
 *
 * Unlike the INA237, the INA226 carries a manufacturer ID (0xFE = 0x5449) and a
 * die ID (0xFF = 0x2260 or 0x2261), so init probes both and rejects a part that
 * does not answer as an INA226. The power-on CONFIG default is already
 * continuous shunt + bus conversion, so init writes no configuration.
 */
rov_status_t ina226_init(ina226_dev_t *dev, const rov_i2c_bus_t *bus, uint8_t i2c_addr, float shunt_resistor_ohms);

rov_status_t ina226_read_power(ina226_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* INA226_H */
```

- [ ] **Step 4: Implement the driver**

Replace `drivers/src/ina226.c` with:

```c
/**
 * @file ina226.c
 * @brief TI INA226 Power Monitor Driver Implementation.
 * @organization Purdue ROV
 */

#include "ina226.h"

#include <string.h>

/* INA226 register pointer values. */
#define INA226_REG_SHUNT_VOLTAGE   0x01u
#define INA226_REG_BUS_VOLTAGE     0x02u
#define INA226_REG_MANUFACTURER_ID 0xFEu
#define INA226_REG_DIE_ID          0xFFu

#define INA226_MANUFACTURER_ID 0x5449u
#define INA226_DIE_ID_A        0x2260u
#define INA226_DIE_ID_B        0x2261u

/* Fixed conversion factors. */
#define INA226_SHUNT_VOLTAGE_LSB_V 0.0000025f /* 2.5 uV per count */
#define INA226_BUS_VOLTAGE_LSB_V   0.00125f   /* 1.25 mV per count */

static uint16_t read_u16_be(const uint8_t *p) {
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static rov_status_t ina226_check_id(ina226_dev_t *dev) {
    uint8_t manufacturer_raw[2] = {0u, 0u};
    uint8_t die_raw[2] = {0u, 0u};
    uint16_t die_id;

    rov_status_t status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, INA226_REG_MANUFACTURER_ID, manufacturer_raw,
                                           sizeof(manufacturer_raw));

    if (status != ROV_OK) {
        return status;
    }

    status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, INA226_REG_DIE_ID, die_raw, sizeof(die_raw));

    if (status != ROV_OK) {
        return status;
    }

    if (read_u16_be(manufacturer_raw) != INA226_MANUFACTURER_ID) {
        return ROV_ERROR;
    }

    die_id = read_u16_be(die_raw);

    if (die_id != INA226_DIE_ID_A && die_id != INA226_DIE_ID_B) {
        return ROV_ERROR;
    }

    return ROV_OK;
}

rov_status_t ina226_init(ina226_dev_t *dev, const rov_i2c_bus_t *bus, uint8_t i2c_addr, float shunt_resistor_ohms) {
    if (dev == NULL || bus == NULL || shunt_resistor_ohms <= 0.0f) {
        return ROV_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(ina226_dev_t));
    dev->bus = bus;
    dev->i2c_addr = i2c_addr;
    dev->shunt_resistor_ohms = shunt_resistor_ohms;

    return ina226_check_id(dev);
}

rov_status_t ina226_read_power(ina226_dev_t *dev) {
    uint8_t vbus_raw[2] = {0u, 0u};
    uint8_t vshunt_raw[2] = {0u, 0u};
    uint16_t vbus;
    int16_t vshunt;

    if (dev == NULL || dev->bus == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    rov_status_t status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, INA226_REG_BUS_VOLTAGE, vbus_raw, sizeof(vbus_raw));

    if (status != ROV_OK) {
        return status;
    }

    status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, INA226_REG_SHUNT_VOLTAGE, vshunt_raw, sizeof(vshunt_raw));

    if (status != ROV_OK) {
        return status;
    }

    vbus = read_u16_be(vbus_raw);
    vshunt = (int16_t)read_u16_be(vshunt_raw);

    dev->voltage_v = (float)vbus * INA226_BUS_VOLTAGE_LSB_V;
    dev->current_a = ((float)vshunt * INA226_SHUNT_VOLTAGE_LSB_V) / dev->shunt_resistor_ohms;
    dev->power_w = dev->voltage_v * dev->current_a;

    return ROV_OK;
}
```

- [ ] **Step 5: Add the host device model**

Create `tests/mocks/mock_ina226.h`:

```c
/**
 * @file mock_ina226.h
 * @brief INA226 device model for the host mock I2C register file.
 * @organization Purdue ROV
 */

#ifndef MOCK_INA226_H
#define MOCK_INA226_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Write the manufacturer and die ID registers so ina226_init() succeeds.
 * @param addr 7-bit I2C address, 0x40..0x4F.
 */
void mock_ina226_present(uint8_t addr);

/**
 * @brief Write the VBUS and VSHUNT registers of the INA226 at @p addr.
 * @param addr 7-bit I2C address.
 * @param bus_voltage_v Target bus voltage in volts.
 * @param current_a Target shunt current in amperes.
 * @param shunt_resistor_ohms Shunt value the driver under test is configured with.
 */
void mock_ina226_set_reading(uint8_t addr, float bus_voltage_v, float current_a, float shunt_resistor_ohms);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_INA226_H */
```

Create `tests/mocks/mock_ina226.c`:

```c
/**
 * @file mock_ina226.c
 * @brief INA226 device model for the host mock I2C register file.
 * @organization Purdue ROV
 */

#include "mock_ina226.h"

#include "mock_bsp.h"

/* Register map, matching drivers/src/ina226.c. */
#define INA226_REG_SHUNT_VOLTAGE   0x01u
#define INA226_REG_BUS_VOLTAGE     0x02u
#define INA226_REG_MANUFACTURER_ID 0xFEu
#define INA226_REG_DIE_ID          0xFFu

#define INA226_MANUFACTURER_ID 0x5449u
#define INA226_DIE_ID          0x2260u

#define INA226_SHUNT_VOLTAGE_LSB_V 0.0000025f /* 2.5 uV per count */
#define INA226_BUS_VOLTAGE_LSB_V   0.00125f   /* 1.25 mV per count */

static int16_t round_to_i16(float value) {
    if (value >= 0.0f) {
        return (int16_t)(value + 0.5f);
    }

    return (int16_t)(value - 0.5f);
}

void mock_ina226_present(uint8_t addr) {
    mock_bsp_i2c_set_word(addr, INA226_REG_MANUFACTURER_ID, INA226_MANUFACTURER_ID);
    mock_bsp_i2c_set_word(addr, INA226_REG_DIE_ID, INA226_DIE_ID);
}

void mock_ina226_set_reading(uint8_t addr, float bus_voltage_v, float current_a, float shunt_resistor_ohms) {
    uint16_t vbus_counts = (uint16_t)round_to_i16(bus_voltage_v / INA226_BUS_VOLTAGE_LSB_V);
    int16_t vshunt_counts = round_to_i16((current_a * shunt_resistor_ohms) / INA226_SHUNT_VOLTAGE_LSB_V);

    mock_bsp_i2c_set_word(addr, INA226_REG_BUS_VOLTAGE, vbus_counts);
    mock_bsp_i2c_set_word(addr, INA226_REG_SHUNT_VOLTAGE, (uint16_t)vshunt_counts);
}
```

- [ ] **Step 6: Register the model in the mock object library**

In `tests/CMakeLists.txt`, add `mocks/mock_ina226.c` to the `rov_mocks` source list.

- [ ] **Step 7: Feed the physics sync through the INA226 device model**

In `tests/mocks/mock_physics.c`, add `#include "mock_ina226.h"` and, in the block edited in Task 3, add the INA226 line:

```c
        /* INA226 12V bus monitor & INA237 48V tether monitor */
        float total_12v_cur = g_phys.brick_currents_a[0] + g_phys.brick_currents_a[1] + g_phys.brick_currents_a[2] +
                              g_phys.brick_currents_a[3];
        mock_ina226_set_reading(0x41u, g_phys.brick_voltages_v[0], total_12v_cur, 0.002f);
        mock_ina237_set_reading(0x40u, g_phys.brick_voltages_v[0], total_12v_cur, 0.001f);
```

(INA226 has no node consumer in SIL; the 0x41 address and 2 mOhm shunt are the model's own choice, kept distinct from node 1's INA237 at 0x40.)

- [ ] **Step 8: Delete the INA226 value hook**

In `tests/mocks/mock_sensors.h`: remove `#include "ina226.h"`, `#include "ina237.h"`, and the block

```c
/**
 * @brief Inject INA226 / INA237 bus voltage and current.
 */
void mock_sensors_set_ina226(float voltage_v, float current_a);
bool mock_sensors_get_ina226(float *voltage_v, float *current_a);
```

In `tests/mocks/mock_sensors.c`: remove the struct

```c
static struct {
    float voltage_v;
    float current_a;
    bool valid;
} g_mock_ina226;
```

remove the three reset lines

```c
    g_mock_ina226.voltage_v = 5.2f;
    g_mock_ina226.current_a = 1.5f;
    g_mock_ina226.valid = true;
```

and remove both functions

```c
void mock_sensors_set_ina226(float voltage_v, float current_a) {
    g_mock_ina226.voltage_v = voltage_v;
    g_mock_ina226.current_a = current_a;
    g_mock_ina226.valid = true;
}

bool mock_sensors_get_ina226(float *voltage_v, float *current_a) {
    if (!g_mock_ina226.valid)
        return false;
    if (voltage_v)
        *voltage_v = g_mock_ina226.voltage_v;
    if (current_a)
        *current_a = g_mock_ina226.current_a;
    return true;
}
```

Also confirm the weak definitions and calls in `drivers/src/ina226.c` (replaced in Step 4) and `drivers/src/ina237.c` (replaced in Task 3 Step 4) are gone:

```powershell
git grep -n "mock_sensors_get_ina226" -- drivers tests
```

Expected: only `tests/hardware/fakes/fake_hal.c` remains (removed in Task 5).

- [ ] **Step 9: Run the test and the full non-target suite**

```powershell
$env:PATH = "C:\msys64\ucrt64\bin;" + $env:PATH
cmake --build build-native
ctest --test-dir build-native -E "^target_" --output-on-failure
```

Expected: 36/36 non-target tests pass.

- [ ] **Step 10: Update the guide row**

In `docs/sil_simulation_guide.md`, change row 15:

```
| 15 | `test_driver_ina226` | INA226 die-ID probe, bus/shunt register reads, conversion, and fault propagation |
```

- [ ] **Step 11: Commit**

```powershell
git add drivers/include/ina226.h drivers/src/ina226.c tests/mocks/mock_ina226.h tests/mocks/mock_ina226.c tests/CMakeLists.txt tests/test_driver_ina226.c tests/mocks/mock_physics.c tests/mocks/mock_sensors.h tests/mocks/mock_sensors.c docs/sil_simulation_guide.md
git commit -m "refactor(drivers): read INA226 over the injected I2C bus"
```

---

### Task 5: Retire the dead target-side value hooks

**Files:**
- Modify: `tests/hardware/fakes/fake_hal.c`
- Modify: `tests/hardware/fakes/main.h`
- Modify: `tests/hardware/test_target_bsp_node3.c`
- Modify: `docs/target-integration-blockers.md`

**Interfaces:**
- Consumes: nothing new.
- Produces: nothing new. This task only removes symbols that Tasks 2-4 made dead.

Rationale: after Tasks 2-4 no driver calls `mock_sensors_get_ina226` or `mock_sensors_get_tmp1075`, so the strong definitions in `fake_hal.c` are dead code that would mislead a future reader into thinking a live value path exists. The setters they read (`fake_hal_set_ina237`, `fake_hal_set_tmp1075`) write to file-static floats with no other consumer. The `target_bsp_node3` acceptance checks are preserved unchanged; they still fail on the two constants node 3 returns today.

- [ ] **Step 1: Confirm the hooks are unreferenced**

```powershell
git grep -n "mock_sensors_get_ina226\|mock_sensors_get_tmp1075\|fake_hal_set_ina237\|fake_hal_set_tmp1075"
```

Expected: references only in `tests/hardware/fakes/fake_hal.c`, `tests/hardware/fakes/main.h`, and `tests/hardware/test_target_bsp_node3.c`. If any other file appears, stop and re-check Tasks 2-4.

- [ ] **Step 2: Remove the dead state and hooks from the fake HAL**

In `tests/hardware/fakes/fake_hal.c`, remove the file-static declarations

```c
static float s_ina_voltage_v;
static float s_ina_current_a;
static float s_tmp1075_temperature_c;
static bool s_ina_available;
static bool s_tmp1075_available;
```

remove the matching lines from `fake_hal_reset()`:

```c
    s_ina_voltage_v = 0.0f;
    s_ina_current_a = 0.0f;
    s_tmp1075_temperature_c = 0.0f;
    s_ina_available = false;
    s_tmp1075_available = false;
```

and remove the four functions:

```c
void fake_hal_set_ina237(float voltage_v, float current_a, bool available) {
    s_ina_voltage_v = voltage_v;
    s_ina_current_a = current_a;
    s_ina_available = available;
}

void fake_hal_set_tmp1075(float temperature_c, bool available) {
    s_tmp1075_temperature_c = temperature_c;
    s_tmp1075_available = available;
}

bool mock_sensors_get_ina226(float *voltage_v, float *current_a) {
    if (!s_ina_available || voltage_v == NULL || current_a == NULL) {
        return false;
    }
    *voltage_v = s_ina_voltage_v;
    *current_a = s_ina_current_a;
    return true;
}

bool mock_sensors_get_tmp1075(float *temperature_c) {
    if (!s_tmp1075_available || temperature_c == NULL) {
        return false;
    }
    *temperature_c = s_tmp1075_temperature_c;
    return true;
}
```

- [ ] **Step 3: Drop the declarations from the fake header**

In `tests/hardware/fakes/main.h`, remove:

```c
void fake_hal_set_ina237(float voltage_v, float current_a, bool available);
void fake_hal_set_tmp1075(float temperature_c, bool available);
```

- [ ] **Step 4: Drop the staging calls from the node 3 acceptance test**

In `tests/hardware/test_target_bsp_node3.c`, remove the two staging calls:

```c
    fake_hal_set_ina237(5.07f, 1.25f, true);
    fake_hal_set_tmp1075(37.5f, true);
```

Leave every `CHECK` unchanged. Add a comment above the remaining checks explaining the current state:

```c
    /*
     * Node 3 sources the logic voltage and PCB temperature from INA237 and
     * TMP1075. Both drivers now read the injected I2C bus, but node 3's BSP has
     * no bus (no MX_I2C*_Init and no C542 HAL in this checkout), so it still
     * returns nominal constants and these two checks stay red. When node 3's
     * bus lands, stage the two readings through the I2C register file.
     */
```

- [ ] **Step 5: Run the target contract and confirm the failure set is unchanged**

```powershell
$env:PATH = "C:\msys64\ucrt64\bin;" + $env:PATH
cmake --build build-native --target test_target_bsp_node3
ctest --test-dir build-native -R "^target_bsp_node3$" --output-on-failure
```

Expected: the executable builds and links; the test fails on the same two checks (logic voltage and PCB temperature) and nothing else. The `target_*` failure is the expected-red contract, not a regression.

- [ ] **Step 6: Update the blockers document**

In `docs/target-integration-blockers.md`, add a "Sensor integration" section:

```markdown
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
```

- [ ] **Step 7: Run the whole suite and the cross build**

```powershell
$env:PATH = "C:\msys64\ucrt64\bin;" + $env:PATH
cmake --build build-native
ctest --test-dir build-native -E "^target_" --output-on-failure
```

Expected: 36/36 non-target tests pass.

```powershell
cmake --preset arm-c542-debug
cmake --build --preset arm-c542-debug
```

Expected: exit 0. The drivers compile into the target libraries; no new link errors.

- [ ] **Step 8: Confirm formatting is clean**

```powershell
git diff --name-only HEAD~5 -- "*.c" "*.h" | ForEach-Object { & "C:\msys64\ucrt64\bin\clang-format.exe" --dry-run --Werror $_ }
```

Expected: no output. If a file is flagged, run `clang-format -i` on it and re-run the suite.

- [ ] **Step 9: Commit**

```powershell
git add tests/hardware/fakes/fake_hal.c tests/hardware/fakes/main.h tests/hardware/test_target_bsp_node3.c docs/target-integration-blockers.md
git commit -m "test(target): retire the dead INA/TMP1075 value hooks"
```

---

## Follow-on plans (not in this plan)

The spec's remaining migration steps become their own plans, in this order:

1. **MS5837 and the raw transport** — remove its `mock_sensors_get_ms5837` weak hook, teach the mock bus the command-addressed transaction, and absorb the SIL ripple (`test_driver_ms5837`, `test_services`, the node2 depth path).
2. **Service facades and boundary fixes** — services own their driver and drop the duplicated caches; `app.c` stops naming drivers; device functions (`bsp_get_pcb_temperature_c`, `bsp_get_logic_voltage_mv`, `bsp_pmbus_read_word`) leave `bsp.h`; fail-loud board macros; single safety-state source; delete `x19_*` forwarders.
3. **Bench harness** — `bench/bench_<sensor>.c` per sensor and `rov bench -n`.

## Self-Review

- **Spec coverage:** spec "Migration order" step 5 is Tasks 2-4 (TMP1075, INA237, INA226). Task 1 is a prerequisite the spec did not anticipate: the mock bus could not represent a 16-bit-register device, so the host half of the three-homes contract was not yet satisfiable. Task 5 retires the dead hooks. Steps 6-13 are deferred to the follow-on plans above.
- **Placeholder scan:** no `TBD`/`TODO`/"handle edge cases". Every code step shows full content, including the datasheet constants and the sign-extension, left-justification, and word-register handling that are the easy places to get wrong.
- **Type consistency:** `tmp1075_init(dev, bus, addr)`, `ina237_init(dev, bus, addr, shunt)`, and `ina226_init(dev, bus, addr, shunt)` match the BME280 pilot's `init(dev, bus)` ordering convention (transport immediately after the device). All three device models write through `mock_bsp_i2c_set_word()`, matching the mock's word-addressed read path added in Task 1. `bsp_i2c_bus()` returns `const rov_i2c_bus_t *` everywhere, matching the foundation plan.
- **Register-width correctness:** the word file is separate from the byte file, so register `0x05` and register `0x06` do not alias, and `test_mock_i2c_bus` asserts exactly that. INA226's ID registers at `0xFE`/`0xFF` are addressable because the word file is register-indexed (a stride into the 256-byte file would have overflowed at `0xFE`).
- **Datasheet accuracy:** constants are taken from the TI datasheets and the Linux `ina238.c` driver (see the table above). Only INA226 gets an ID probe, because only INA226 has usable ID registers; TMP1075N and INA237 explicitly do not, and each `init()` documents why.
- **Review Focus coverage:** NULL bus/dev/shunt -> each driver task's Step 1 test; error propagation with an intact reading -> each driver task's Step 1 fault block; TMP1075 sign extension -> Task 2 Step 1 negative case; INA237 die-temp left-justification -> Task 3 Step 1 die-temp case; INA237 must write ADC_CONFIG -> Task 3 Step 4 `ina237_configure_adc()`; INA226 must not write CONFIG -> Task 4 Step 3 header note; no register aliasing -> Task 1 Step 1 test; mock/target parity -> the same one-address-two-bytes semantics on both sides.
