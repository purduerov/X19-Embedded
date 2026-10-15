# Injected I2C Bus Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Introduce the `rov_i2c_bus_t` transport contract and move the BME280 driver onto an injected bus, so one driver source runs on target firmware and host tests without reaching for the global BSP.

**Architecture:** A driver holds a `const rov_i2c_bus_t *` and issues raw `write`/`read` (plus inline `mem_*` helpers). The target BSP and the host mock each supply a bus implementation; the driver is unchanged between them. This is the standard Linux/Zephyr split (`bme280.c` core + `bme280-i2c.c` transport).

**Tech Stack:** C11, CMake 3.22, Unity-free `assert`-style host tests, `tests/mocks/mock_bsp.c` register file, STM32 HAL BSP adapters.

**Spec:** `docs/superpowers/specs/2026-10-15-injected-i2c-bus-driver-architecture-design.md`

## Global Constraints

- Build warnings are errors: `-Wall -Wextra -Wpedantic -Wshadow -Wdouble-promotion -Werror`. Every new file must compile clean.
- Application/control code must not call ST HAL; only BSP/driver-integration code may. Drivers must not call ST HAL either — they call the injected bus.
- No `malloc`/`free` in flight firmware. No `HAL_Delay()` in polling paths.
- Do not hand-edit generated CubeMX files (`main.c`, `*_it.c`, `*_msp.c`).
- Do not use emojis anywhere.
- Keep the fail-safe invariant: PWM neutral `1500 us`, heartbeat/leak/E-stop clears actuators.
- BME280 is the pilot. SPI parts (`bmi270`, `lsm6dsoxtr`), PMBus, `tcan1044`, DShot, INA226/237, TMP1075, and MS5837 are **out of scope** for this plan.
- Host tests run from the repo root of `Embedded/X19-Embedded/` via the native build:
  `cmake -B build-native -G Ninja` then `cmake --build build-native` then
  `ctest --test-dir build-native -E "^target_" --output-on-failure`.

## Review Focus

- A `NULL` bus passed to `bme280_init()` must fail loudly (`ROV_ERR_INVALID_ARG`) rather than store a null pointer that faults on the first read.
- A bus `write`/`read` that returns an error must propagate out of `bme280_read_all()` unchanged, and must leave the caller's previous reading intact (no half-updated telemetry).
- `rov_i2c_mem_write` with an over-long payload (a caller bug) must return `ROV_ERR_INVALID_ARG`, not overrun its scratch buffer.
- The mock bus and the target bus must present the same register-pointer semantics, or a driver that passes on host will fail on hardware.
- `bme280_init()` keeps its 0x76-then-0x77 fallback and its chip-id check on both buses.

---

### Task 1: The bus contract header

**Files:**
- Create: `shared/include/rov_i2c.h`
- Create: `tests/test_rov_i2c.c`
- Modify: `tests/CMakeLists.txt` (add a new `x19_add_node_test` block near the other `SUITE shared` tests, around line 97)

**Interfaces:**
- Consumes: `rov_status_t` and the `ROV_OK` / `ROV_ERR_INVALID_ARG` values from `shared/include/rov_types.h`.
- Produces: `rov_i2c_bus_t` (fields `ctx`, `write`, `read`, `probe`) and the inline helpers
  `rov_i2c_mem_read(const rov_i2c_bus_t*, uint8_t addr, uint8_t reg, uint8_t *dst, uint16_t len)`
  and
  `rov_i2c_mem_write(const rov_i2c_bus_t*, uint8_t addr, uint8_t reg, const uint8_t *src, uint16_t len)`,
  both returning `rov_status_t`.

- [ ] **Step 1: Write the failing test**

Create `tests/test_rov_i2c.c`:

```c
/**
 * @file test_rov_i2c.c
 * @brief Unit tests for the injected I2C bus contract helpers.
 * @organization Purdue ROV
 */

#include "rov_i2c.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint8_t regs[256];
    uint8_t ptr;
    int write_calls;
    int read_calls;
    rov_status_t write_status;
} fake_bus_t;

static rov_status_t fake_write(void *ctx, uint8_t addr, const uint8_t *data, uint16_t len) {
    fake_bus_t *b = (fake_bus_t *)ctx;
    (void)addr;

    if (len == 0U || data == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    b->write_calls++;

    if (b->write_status != ROV_OK) {
        return b->write_status;
    }

    b->ptr = data[0];

    for (uint16_t i = 1U; i < len; i++) {
        b->regs[(uint8_t)(b->ptr + (i - 1U))] = data[i];
    }

    return ROV_OK;
}

static rov_status_t fake_read(void *ctx, uint8_t addr, uint8_t *data, uint16_t len) {
    fake_bus_t *b = (fake_bus_t *)ctx;
    (void)addr;

    b->read_calls++;

    for (uint16_t i = 0U; i < len; i++) {
        data[i] = b->regs[(uint8_t)(b->ptr + i)];
    }

    return ROV_OK;
}

static bool fake_probe(void *ctx, uint8_t addr) {
    (void)ctx;
    (void)addr;
    return true;
}

static void test_mem_read_issues_pointer_write_then_read(void) {
    fake_bus_t b;
    uint8_t out[2] = {0};
    rov_i2c_bus_t bus = {&b, fake_write, fake_read, fake_probe};

    memset(&b, 0, sizeof(b));
    b.regs[0x10] = 0xABu;
    b.regs[0x11] = 0xCDu;

    assert(rov_i2c_mem_read(&bus, 0x76u, 0x10u, out, 2U) == ROV_OK);
    assert(out[0] == 0xABu);
    assert(out[1] == 0xCDu);
    assert(b.write_calls == 1);
    assert(b.read_calls == 1);
}

static void test_mem_write_sends_pointer_then_data(void) {
    fake_bus_t b;
    uint8_t value = 0x57u;
    rov_i2c_bus_t bus = {&b, fake_write, fake_read, fake_probe};

    memset(&b, 0, sizeof(b));

    assert(rov_i2c_mem_write(&bus, 0x76u, 0xF4u, &value, 1U) == ROV_OK);
    assert(b.regs[0xF4] == 0x57u);
    assert(b.write_calls == 1);
}

static void test_mem_write_rejects_over_long_payload(void) {
    fake_bus_t b;
    uint8_t value[16] = {0};
    rov_i2c_bus_t bus = {&b, fake_write, fake_read, fake_probe};

    memset(&b, 0, sizeof(b));

    assert(rov_i2c_mem_write(&bus, 0x76u, 0x00u, value, sizeof(value)) == ROV_ERR_INVALID_ARG);
    assert(b.write_calls == 0);
}

static void test_null_bus_is_rejected(void) {
    uint8_t out[1] = {0};
    assert(rov_i2c_mem_read(NULL, 0x76u, 0x00u, out, 1U) == ROV_ERR_INVALID_ARG);
    assert(rov_i2c_mem_write(NULL, 0x76u, 0x00u, out, 1U) == ROV_ERR_INVALID_ARG);
}

static void test_write_error_propagates_before_read(void) {
    fake_bus_t b;
    uint8_t out[1] = {0};
    rov_i2c_bus_t bus = {&b, fake_write, fake_read, fake_probe};

    memset(&b, 0, sizeof(b));
    b.write_status = ROV_TIMEOUT;

    assert(rov_i2c_mem_read(&bus, 0x76u, 0x10u, out, 1U) == ROV_TIMEOUT);
    assert(b.read_calls == 0);
}

int main(void) {
    printf("Running injected I2C bus contract tests...\n");

    test_mem_read_issues_pointer_write_then_read();
    test_mem_write_sends_pointer_then_data();
    test_mem_write_rejects_over_long_payload();
    test_null_bus_is_rejected();
    test_write_error_propagates_before_read();

    printf("[PASS] test_rov_i2c\n");
    return 0;
}
```

- [ ] **Step 2: Register the test so it builds**

In `tests/CMakeLists.txt`, immediately after the `test_i2c_recovery` block (around line 97), add:

```cmake
x19_add_node_test(NAME test_rov_i2c SUITE shared
    SOURCES test_rov_i2c.c
    LIBS rov_shared
)
```

- [ ] **Step 3: Run it to confirm it fails**

Run:
```powershell
cmake -B build-native -G Ninja
cmake --build build-native --target test_rov_i2c
```
Expected: FAIL to compile with `fatal error: rov_i2c.h: No such file or directory`.

- [ ] **Step 4: Write the header**

Create `shared/include/rov_i2c.h`:

```c
/**
 * @file rov_i2c.h
 * @brief Injected I2C bus contract shared by drivers, target BSP, and host mocks.
 * @organization Purdue ROV
 *
 * A driver never talks to the board BSP directly. It holds a `rov_i2c_bus_t`
 * and issues raw byte transfers, so the same driver source runs on STM32 target
 * firmware (over HAL) and on the host (over a register-file mock).
 *
 * `mem_*` is a convenience over the raw pair: write the register pointer, then
 * read or write the payload. Command-addressed parts (e.g. MS5837) use the raw
 * pair directly, because a register pointer means nothing to them.
 */

#ifndef ROV_I2C_H
#define ROV_I2C_H

#include "rov_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct rov_i2c_bus rov_i2c_bus_t;

struct rov_i2c_bus {
    void *ctx;
    rov_status_t (*write)(void *ctx, uint8_t addr, const uint8_t *data, uint16_t len);
    rov_status_t (*read)(void *ctx, uint8_t addr, uint8_t *data, uint16_t len);
    bool (*probe)(void *ctx, uint8_t addr);
};

/**
 * @brief Write a register address, then read len bytes from the device.
 * @return ROV_OK on success, the transport error otherwise, or
 *         ROV_ERR_INVALID_ARG for a null bus/destination.
 */
static inline rov_status_t rov_i2c_mem_read(const rov_i2c_bus_t *bus, uint8_t addr, uint8_t reg, uint8_t *dst,
                                            uint16_t len) {
    rov_status_t status;

    if (bus == NULL || bus->write == NULL || bus->read == NULL || dst == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    status = bus->write(bus->ctx, addr, &reg, 1U);
    if (status != ROV_OK) {
        return status;
    }

    return bus->read(bus->ctx, addr, dst, len);
}

/**
 * @brief Write a register address followed by len data bytes.
 * @return ROV_OK on success, the transport error otherwise, or
 *         ROV_ERR_INVALID_ARG for a null bus/source or a payload over 8 bytes.
 */
static inline rov_status_t rov_i2c_mem_write(const rov_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                                             const uint8_t *src, uint16_t len) {
    uint8_t buffer[1U + 8U];

    if (bus == NULL || bus->write == NULL || src == NULL || len == 0U || len > 8U) {
        return ROV_ERR_INVALID_ARG;
    }

    buffer[0] = reg;

    for (uint16_t i = 0U; i < len; i++) {
        buffer[1U + i] = src[i];
    }

    return bus->write(bus->ctx, addr, buffer, (uint16_t)(len + 1U));
}

#ifdef __cplusplus
}
#endif

#endif /* ROV_I2C_H */
```

- [ ] **Step 5: Run the test to confirm it passes**

Run:
```powershell
cmake --build build-native --target test_rov_i2c
ctest --test-dir build-native -R "^test_rov_i2c$" --output-on-failure
```
Expected: PASS, printing `[PASS] test_rov_i2c`.

- [ ] **Step 6: Commit**

```powershell
git add shared/include/rov_i2c.h tests/test_rov_i2c.c tests/CMakeLists.txt
git commit -m "feat(shared): add injected I2C bus contract and helper tests"
```
Note: LF/CRLF warnings from Git on this repo are expected and harmless.

---

### Task 2: Host transport in the mock BSP

The mock BSP already keeps a 128x256 register file and already implements
`bsp_i2c_mem_read/write` against it. This task adds the bus surface on top of the
same file, plus fault and presence hooks, so a driver under test can be pointed
at a bus that fails on demand.

**Files:**
- Modify: `shared/include/bsp.h` (add the `rov_i2c.h` include, near line 18, and the `bsp_i2c_bus` prototype after `bsp_i2c_mem_write`, near line 202)
- Modify: `tests/mocks/mock_bsp.c` (add bus implementation + hooks; extend `mock_bsp_reset`)
- Modify: `tests/mocks/mock_bsp.h` (declare the new hooks)
- Create: `tests/test_mock_i2c_bus.c`
- Modify: `tests/CMakeLists.txt` (register the test)

**Interfaces:**
- Consumes: `rov_i2c_bus_t` and `rov_i2c_mem_read` / `rov_i2c_mem_write` from Task 1; the existing `mock_bsp_i2c_set_reg`, `mock_bsp_i2c_set_regs`, `mock_bsp_i2c_get_reg`.
- Produces:
  - `const rov_i2c_bus_t *bsp_i2c_bus(void);`
  - `void mock_bsp_i2c_set_fault(rov_status_t status);` (ROV_OK clears it)
  - `void mock_bsp_i2c_set_present(uint8_t addr, bool present);`

- [ ] **Step 1: Write the failing test**

Create `tests/test_mock_i2c_bus.c`:

```c
/**
 * @file test_mock_i2c_bus.c
 * @brief Tests for the host-side injected I2C bus over the mock register file.
 * @organization Purdue ROV
 */

#include "mock_bsp.h"
#include "rov_i2c.h"
#include <assert.h>
#include <stdio.h>

static void test_mem_read_and_write_round_trip(void) {
    const rov_i2c_bus_t *bus = bsp_i2c_bus();
    uint8_t value = 0x5Au;
    uint8_t readback = 0u;

    mock_bsp_reset();

    assert(rov_i2c_mem_write(bus, 0x29u, 0x02u, &value, 1U) == ROV_OK);
    assert(rov_i2c_mem_read(bus, 0x29u, 0x02u, &readback, 1U) == ROV_OK);
    assert(readback == 0x5Au);
    assert(mock_bsp_i2c_get_reg(0x29u, 0x02u) == 0x5Au);
}

static void test_multi_byte_read_spans_registers(void) {
    const rov_i2c_bus_t *bus = bsp_i2c_bus();
    uint8_t seed[3] = {0x11u, 0x22u, 0x33u};
    uint8_t out[3] = {0u};

    mock_bsp_reset();
    mock_bsp_i2c_set_regs(0x76u, 0x10u, seed, sizeof(seed));

    assert(rov_i2c_mem_read(bus, 0x76u, 0x10u, out, 3U) == ROV_OK);
    assert(out[0] == 0x11u);
    assert(out[1] == 0x22u);
    assert(out[2] == 0x33u);
}

static void test_fault_is_returned_and_cleared_on_reset(void) {
    const rov_i2c_bus_t *bus = bsp_i2c_bus();
    uint8_t out[1] = {0u};

    mock_bsp_reset();
    mock_bsp_i2c_set_fault(ROV_TIMEOUT);

    assert(rov_i2c_mem_read(bus, 0x76u, 0x10u, out, 1U) == ROV_TIMEOUT);

    mock_bsp_reset();
    assert(rov_i2c_mem_read(bus, 0x76u, 0x10u, out, 1U) == ROV_OK);
}

static void test_probe_reports_presence(void) {
    const rov_i2c_bus_t *bus = bsp_i2c_bus();

    mock_bsp_reset();
    assert(bus->probe(bus->ctx, 0x76u) == false);

    mock_bsp_i2c_set_present(0x76u, true);
    assert(bus->probe(bus->ctx, 0x76u) == true);
}

int main(void) {
    printf("Running mock I2C bus tests...\n");

    test_mem_read_and_write_round_trip();
    test_multi_byte_read_spans_registers();
    test_fault_is_returned_and_cleared_on_reset();
    test_probe_reports_presence();

    printf("[PASS] test_mock_i2c_bus\n");
    return 0;
}
```

- [ ] **Step 2: Register the test**

In `tests/CMakeLists.txt`, after the `test_bsp` block (around line 108), add:

```cmake
x19_add_node_test(NAME test_mock_i2c_bus SUITE bsp
    SOURCES test_mock_i2c_bus.c
    LIBS rov_mocks
)
```

- [ ] **Step 3: Run it to confirm it fails**

Run:
```powershell
cmake -B build-native -G Ninja
cmake --build build-native --target test_mock_i2c_bus
```
Expected: FAIL to compile with `'bsp_i2c_bus' undeclared`.

- [ ] **Step 4: Declare the bus in the BSP contract**

In `shared/include/bsp.h`, change the include block (around line 18) to also pull in the bus contract:

```c
#include "rov_types.h"
#include "rov_i2c.h"
#include <stdbool.h>
#include <stdint.h>
```

Then, after the `bsp_i2c_mem_write` prototype (around line 202), add:

```c
/**
 * @brief Get the board's I2C transport as an injectable bus.
 *
 * Drivers take this as an argument instead of calling bsp_i2c_* directly, so the
 * same driver source builds against the target HAL or a host mock.
 *
 * @return Pointer to a statically allocated bus describing this board's I2C.
 */
const rov_i2c_bus_t *bsp_i2c_bus(void);
```

- [ ] **Step 5: Implement the host bus in the mock BSP**

In `tests/mocks/mock_bsp.c`, extend the state block near line 18 with a register pointer, a fault status, and a presence table:

```c
static uint8_t g_mock_i2c_ptr[MOCK_I2C_NUM_ADDRS];
static rov_status_t g_mock_i2c_fault = ROV_OK;
static bool g_mock_i2c_present[MOCK_I2C_NUM_ADDRS];
```

In `mock_bsp_reset()` (near line 38, next to the existing `memset(g_mock_i2c_regs, ...)`), clear the new state:

```c
    memset(g_mock_i2c_ptr, 0, sizeof(g_mock_i2c_ptr));
    memset(g_mock_i2c_present, 0, sizeof(g_mock_i2c_present));
    g_mock_i2c_fault = ROV_OK;
```

Add the hooks next to `mock_bsp_i2c_get_reg` (around line 132):

```c
void mock_bsp_i2c_set_fault(rov_status_t status) {
    g_mock_i2c_fault = status;
}

void mock_bsp_i2c_set_present(uint8_t addr, bool present) {
    if (addr < MOCK_I2C_NUM_ADDRS) {
        g_mock_i2c_present[addr] = present;
    }
}
```

Add the bus implementation and `bsp_i2c_bus()` near the other BSP functions (after `bsp_i2c_mem_write`, around line 308):

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

    for (uint16_t i = 1U; i < len; i++) {
        uint8_t reg = (uint8_t)(g_mock_i2c_ptr[addr] + (uint8_t)(i - 1U));

        g_mock_i2c_regs[addr][reg] = data[i];
    }

    return ROV_OK;
}

static rov_status_t mock_bus_read(void *ctx, uint8_t addr, uint8_t *data, uint16_t len) {
    (void)ctx;

    if (addr >= MOCK_I2C_NUM_ADDRS || data == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    if (g_mock_i2c_fault != ROV_OK) {
        return g_mock_i2c_fault;
    }

    for (uint16_t i = 0U; i < len; i++) {
        data[i] = g_mock_i2c_regs[addr][(uint8_t)(g_mock_i2c_ptr[addr] + (uint8_t)i)];
    }

    return ROV_OK;
}

static bool mock_bus_probe(void *ctx, uint8_t addr) {
    (void)ctx;

    if (addr >= MOCK_I2C_NUM_ADDRS) {
        return false;
    }

    return g_mock_i2c_present[addr];
}

const rov_i2c_bus_t *bsp_i2c_bus(void) {
    static const rov_i2c_bus_t bus = {NULL, mock_bus_write, mock_bus_read, mock_bus_probe};

    return &bus;
}
```

- [ ] **Step 6: Declare the hooks in the mock header**

In `tests/mocks/mock_bsp.h`, after `mock_bsp_i2c_get_reg` (around line 125), add:

```c
/**
 * @brief Force every mock I2C transfer to fail with this status until reset.
 * @param status Error to return; ROV_OK clears the fault.
 */
void mock_bsp_i2c_set_fault(rov_status_t status);

/**
 * @brief Mark whether a device answers at this address for probe().
 * @param addr 7-bit I2C address.
 * @param present true if the bus should report the device present.
 */
void mock_bsp_i2c_set_present(uint8_t addr, bool present);
```

- [ ] **Step 7: Run the test to confirm it passes**

Run:
```powershell
cmake --build build-native --target test_mock_i2c_bus
ctest --test-dir build-native -R "^test_mock_i2c_bus$" --output-on-failure
```
Expected: PASS, printing `[PASS] test_mock_i2c_bus`.

- [ ] **Step 8: Confirm nothing else regressed**

Run:
```powershell
cmake --build build-native
ctest --test-dir build-native -E "^target_" --output-on-failure
```
Expected: the same pass/fail set as before this task (no new failures).

- [ ] **Step 9: Commit**

```powershell
git add shared/include/bsp.h tests/mocks/mock_bsp.c tests/mocks/mock_bsp.h tests/test_mock_i2c_bus.c tests/CMakeLists.txt
git commit -m "feat(mocks): expose host I2C bus with fault and presence hooks"
```

---

### Task 3: BME280 through the injected bus

BME280 is register-addressed and already mocked at the bus level, so this is a
mechanical swap with no change in host behaviour. It has no sensor-value hook.

**Files:**
- Modify: `drivers/include/bme280.h:42-55` (add the bus field, include the contract, change the init signature)
- Modify: `drivers/src/bme280.c` (include swap; 4 `bsp_i2c_mem_read` call sites; 3 `bsp_i2c_mem_write` call sites; init guard)
- Modify: `services/src/env_service.c:19`
- Modify: `nodes/node1_pi_shield/Core/Src/app.c:183`
- Modify: `nodes/testing_and_rnd/Src/app.c:94`
- Modify: `tests/test_driver_bme280.c` (every `bme280_init(&dev)` / `bme280_init(NULL)` call, plus a new fault test)

**Interfaces:**
- Consumes: `rov_i2c_bus_t`, `rov_i2c_mem_read`, `rov_i2c_mem_write`, and `bsp_i2c_bus()` from Tasks 1-2.
- Produces: `rov_status_t bme280_init(bme280_dev_t *dev, const rov_i2c_bus_t *bus);` and a
  `bme280_dev_t` that carries `const rov_i2c_bus_t *bus;`. `bme280_read_all(bme280_dev_t *dev)` is unchanged.

- [ ] **Step 1: Write the failing tests**

In `tests/test_driver_bme280.c`, first add a fault-injection case after `test_power_on_sentinel_is_rejected` (around line 414):

```c
/*
 * A bus error must surface from read_all() and must not overwrite the reading
 * the caller already holds.
 */
static void test_bus_error_propagates_and_leaves_reading_intact(void) {
    bme280_dev_t dev;
    float before;

    setup_bme280();
    setup_measurement();

    assert(bme280_init(&dev, bsp_i2c_bus()) == ROV_OK);
    assert(bme280_read_all(&dev) == ROV_OK);

    before = dev.pressure_hpa;

    mock_bsp_i2c_set_fault(ROV_TIMEOUT);
    assert(bme280_read_all(&dev) == ROV_TIMEOUT);
    assert(dev.pressure_hpa == before);

    mock_bsp_i2c_set_fault(ROV_OK);
}
```

Then register it inside `test_bme280_driver()` (around line 472), after the sentinel case:

```c
    test_bus_error_propagates_and_leaves_reading_intact();
```

Also add to `test_null_pointer_guards` (around line 172):

```c
    assert(bme280_init(&dev, NULL) == ROV_ERR_INVALID_ARG);
```
(declare `bme280_dev_t dev;` at the top of that function first).

- [ ] **Step 2: Run it to confirm it fails**

Run:
```powershell
cmake --build build-native --target test_driver_bme280
```
Expected: FAIL to compile — `bme280_init` takes 1 argument, `bsp_i2c_bus` unknown, `mock_bsp_i2c_set_fault` unknown. (`test_driver_bme280.c` already includes `mocks/mock_bsp.h`.)

- [ ] **Step 3: Change the header**

In `drivers/include/bme280.h`, change the include block (around line 12) to:

```c
#include "rov_i2c.h"
#include "rov_types.h"
#include <stdint.h>
```

Add the bus field to `bme280_dev_t`, and change the init prototype (around lines 42-55):

```c
typedef struct {
    const rov_i2c_bus_t *bus; /* injected transport; set by bme280_init() */
    uint8_t i2c_addr;         // 0x76 or 0x77
    bme280_calib_t calib;
    uint32_t t_fine; // temperature compensation produces an intermediate value

    float pressure_hpa;
    float humidity_pct;
    float temperature_c;

    bool initialized;
} bme280_dev_t;

rov_status_t bme280_init(bme280_dev_t *dev, const rov_i2c_bus_t *bus);
rov_status_t bme280_read_all(bme280_dev_t *dev);
```

- [ ] **Step 4: Change the driver**

In `drivers/src/bme280.c`:

Replace the includes (lines 7-8):

```c
#include "bme280.h"

#include <stdbool.h>
#include <string.h>
```

Replace the four `bsp_i2c_mem_read(dev->i2c_addr, ...)` calls with `rov_i2c_mem_read(dev->bus, dev->i2c_addr, ...)`:
- `bme280_check_id` (line 33)
- `bme280_read_calibration` (lines 59 and 94)
- `bme280_read_all` (line 358)

Replace the three `bsp_i2c_mem_write(dev->i2c_addr, ...)` calls in `bme280_configure` (lines 133, 146, 163) with `rov_i2c_mem_write(dev->bus, dev->i2c_addr, ...)`.

In `bme280_init` (line 172), after the `dev == NULL` guard and before `memset`, add the bus guard, and store the bus after the memset:

```c
rov_status_t bme280_init(bme280_dev_t *dev, const rov_i2c_bus_t *bus) {
    rov_status_t status;

    if (dev == NULL || bus == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));

    dev->bus = bus;
```

(The rest of `bme280_init` is unchanged; `memset` runs before the bus is stored, so the assignment must come after it.)

- [ ] **Step 5: Update the three production callers**

`services/src/env_service.c:19`:

```c
    rov_status_t status = bme280_init(&g_bme_dev, bsp_i2c_bus());
```

`nodes/node1_pi_shield/Core/Src/app.c:183`:

```c
    bme280_init(&g_bme280_dev, bsp_i2c_bus());
```

`nodes/testing_and_rnd/Src/app.c:94`:

```c
    rov_status_t status = bme280_init(&bme280, bsp_i2c_bus());
```

- [ ] **Step 6: Update the existing test call sites**

In `tests/test_driver_bme280.c`, replace every `bme280_init(&dev)` with `bme280_init(&dev, bsp_i2c_bus())` and `bme280_init(NULL)` with `bme280_init(NULL, bsp_i2c_bus())` (11 sites; `grep -n "bme280_init" tests/test_driver_bme280.c` to confirm).

- [ ] **Step 7: Run the driver test to confirm it passes**

Run:
```powershell
cmake --build build-native --target test_driver_bme280
ctest --test-dir build-native -R "^test_driver_bme280$" --output-on-failure
```
Expected: PASS, printing `[PASS] test_bme280_driver`.

- [ ] **Step 8: Run the affected SIL tests**

The node1 app, the services test, and the multi-node/SIL suites all compile `env_service.c` and `node1` `app.c`, so they must still pass against the mock bus.

Run:
```powershell
cmake --build build-native
ctest --test-dir build-native -R "test_node1_pi_shield|test_services|test_multi_node_bus|test_sil_safety|test_sil_power|test_sil_fuzz|test_sil_burnin|test_sil_bridge_logical" --output-on-failure
```
Expected: all PASS.

- [ ] **Step 9: Commit**

```powershell
git add drivers/include/bme280.h drivers/src/bme280.c services/src/env_service.c nodes/node1_pi_shield/Core/Src/app.c nodes/testing_and_rnd/Src/app.c tests/test_driver_bme280.c
git commit -m "refactor(drivers): inject I2C bus into BME280"
```

---

### Task 4: Node1 target transport and full verification

The node1 firmware links `bsp.c`, which still has no `bsp_i2c_bus()`, so the
cross-compiled node1 target will not link until this lands. The dev-board
`platform_i2c.c` gets the same adapter when the bench work arrives (out of scope
here), because nothing in this plan references it yet.

**Files:**
- Modify: `nodes/node1_pi_shield/Core/Src/bsp.c` (add the adapter, after `bsp_i2c_mem_write`)

**Interfaces:**
- Consumes: `rov_i2c_bus_t` from Task 1 and the existing `bsp_i2c_write` / `bsp_i2c_read` / `bsp_i2c_probe` in the same file.
- Produces: `const rov_i2c_bus_t *bsp_i2c_bus(void);` for the node1 target.

- [ ] **Step 1: Add the adapter**

At the end of `nodes/node1_pi_shield/Core/Src/bsp.c` (after the `bsp_i2c_mem_write` definition), add:

```c
static rov_status_t bsp_bus_write(void *ctx, uint8_t addr, const uint8_t *data, uint16_t len) {
    (void)ctx;
    return bsp_i2c_write(addr, data, len) ? ROV_OK : ROV_ERROR;
}

static rov_status_t bsp_bus_read(void *ctx, uint8_t addr, uint8_t *data, uint16_t len) {
    (void)ctx;
    return bsp_i2c_read(addr, data, len) ? ROV_OK : ROV_ERROR;
}

static bool bsp_bus_probe(void *ctx, uint8_t addr) {
    (void)ctx;
    return bsp_i2c_probe(addr);
}

const rov_i2c_bus_t *bsp_i2c_bus(void) {
    static const rov_i2c_bus_t bus = {NULL, bsp_bus_write, bsp_bus_read, bsp_bus_probe};

    return &bus;
}
```

- [ ] **Step 2: Build the target-readiness contract for node1**

Run:
```powershell
cmake --build build-native --target test_target_bsp_node1
```
Expected: builds clean. (`test_target_bsp_node1` compiles `nodes/node1_pi_shield/Core/Src/bsp.c` against the fake HAL, so it is the check that the adapter compiles for the target.)

- [ ] **Step 3: Cross-compile the node1 target firmware**

Run:
```powershell
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
cmake --build build
```
Expected: `f411_node1_pi_shield.elf` (or the configured node1 target) links with no undefined `bsp_i2c_bus`.

- [ ] **Step 4: Run the whole host suite**

Run:
```powershell
cmake --build build-native
ctest --test-dir build-native -E "^target_" --output-on-failure
```
Expected: no new failures versus the pre-plan baseline.

- [ ] **Step 5: Commit**

```powershell
git add nodes/node1_pi_shield/Core/Src/bsp.c
git commit -m "feat(node1): expose injected I2C bus for the target build"
```

---

## Follow-on plans (not in this plan)

The spec's remaining migration steps become their own plans, in this order:

1. **Remaining register-addressed drivers** (INA226, INA237, TMP1075) plus the
   dev-board `platform_i2c.c` adapter, on the Task 1-4 pattern.
2. **MS5837 and the raw transport** — remove its `mock_sensors_get_ms5837` weak
   hook, teach the mock bus the command-addressed transaction, and absorb the SIL
   ripple (`test_driver_ms5837`, `test_services`, the node2 depth path).
3. **Service facades and boundary fixes** — services own their driver and drop
   the duplicated caches; `app.c` stops naming drivers; device functions leave
   `bsp.h`; fail-loud board macros; single safety-state source; delete `x19_*`
   forwarders.
4. **Bench harness** — `bench/bench_<sensor>.c` per sensor and `rov bench -n`.

## Self-Review

- **Spec coverage:** spec "Migration order" steps 1-4 are Tasks 1-4. Steps 5-13
  are explicitly deferred to the follow-on plans above. The bus contract matches
  the corrected spec (raw `write`/`read`/`probe` plus inline `mem_*`).
- **Placeholder scan:** no `TBD`/`TODO`/"handle edge cases". Every code step shows
  full content.
- **Type consistency:** `rov_i2c_bus_t` fields (`ctx`, `write`, `read`, `probe`)
  are identical across the header, the mock implementation, and the node1
  adapter. `bme280_init(dev, bus)` is used consistently in all call sites and
  tests. `bsp_i2c_bus()` returns `const rov_i2c_bus_t *` everywhere.
- **Review Focus coverage:** NULL bus -> Task 3 Step 1 (`bme280_init(&dev, NULL)`);
  error propagation + intact reading -> Task 3 Step 1 fault test; over-long payload
  -> Task 1 Step 1; mock/target parity -> Task 4 (same pointer semantics as the
  mock's `mock_bus_write/read`); 0x76/0x77 fallback + chip-id -> existing
  `test_init` and `test_wrong_chip_id`, still run in Task 3 Step 7.
