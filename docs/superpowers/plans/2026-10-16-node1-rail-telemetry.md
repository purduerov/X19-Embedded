# Node 1 Rail Telemetry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make node 1 publish its 5V rail voltage and current on CAN 0x210, with the INA237 owned by `power_service` instead of `app.c`.

**Architecture:** `power_service` gains `power_service_init()` / `power_read_rail()` that acquire `bsp_i2c_bus()` and own a single `ina237_dev_t`, mirroring `env_service`. `app.c` drops the driver header and the raw device and reads the rail through the service. `rov_env_telemetry_t` grows from 13 to 17 bytes with two appended `uint16_t` fields, preserving existing offsets; every C and Python consumer of the frame is updated in the same change.

**Tech Stack:** C11 (host-native + ARM cross), CMake/CTest, Python 3 `unittest`, `struct`, `ast`.

**Spec:** `docs/superpowers/specs/2026-10-16-node1-rail-telemetry-design.md`

## Global Constraints

- No emojis in code, comments, commits, or docs.
- All C/C++ is formatted with `Embedded/X19-Embedded/.clang-format`; CI builds with strict warnings as errors (`-Wall -Wextra -Wpedantic -Wshadow -Wdouble-promotion -Werror`).
- Preserve CAN arbitration IDs and existing struct field offsets; never reorder or renumber existing fields.
- Layer rule: `app.c` speaks to services only (no driver header, no bus name); drivers call only the injected bus, never `bsp_*`/HAL.
- Fail-safe invariants: PWM neutral is `1500 us`, leak/E-stop clears actuators, no `HAL_Delay()` in polling, no `malloc`/`free` in flight firmware.
- Host build environment: prepend `C:\msys64\ucrt64\bin` to `PATH` before any CMake/CTest command.
- Work from `Embedded/X19-Embedded/`. Commit locally only; do not push.

## Review Focus

- A failed or absent INA237 read must publish rail `0`, never a stale or garbage value (`app.c` must gate on the service status).
- A short 0x210 frame (13 bytes, old layout, or a truncated SIL frame) must be rejected by the C unpacker and skipped by the Python client, not silently truncated.
- `power_read_rail()` called before a successful init must lazy-init, and on a persistent fault must return the error and leave the caller's outputs untouched.
- Rail values must saturate, not wrap: the INA237 can read up to ~164 A at 0.001 ohm, and a `uint16_t` milliampere cast would otherwise report a small number for a huge fault current.
- The Python drift guard between `tools/can_sniffer.py` and `sil_protocol.py` must keep passing after both struct formats change.

---

### Task 1: `power_service` owns the rail INA237

**Files:**
- Modify: `services/include/power_service.h`
- Modify: `services/src/power_service.c`
- Test: `tests/test_services.c`

**Interfaces:**
- Consumes: `ina237_init(ina237_dev_t *, const rov_i2c_bus_t *, uint8_t, float)` and `ina237_read_power(ina237_dev_t *)` from `drivers/include/ina237.h`; `bsp_i2c_bus()` from `shared/include/bsp.h`.
- Produces:
  - `rov_status_t power_service_init(void);`
  - `rov_status_t power_read_rail(float *bus_voltage_v, float *current_a);`

- [ ] **Step 1: Write the failing test**

In `tests/test_services.c`, add the include next to the existing mock includes:

```c
#include "mock_bme280.h"
#include "mock_ina237.h"
```

Add this function after `test_env_service()`:

```c
/*
 * The service owns the rail INA237: power_service_init() binds it to the mock
 * bus and power_read_rail() returns engineering units. Going through the
 * register image means a wrong LSB, a wrong register or a wrong shunt all
 * surface here instead of being masked by a value-level hook.
 */
static void test_power_service_rail(void) {
    float voltage_v = 0.0f;
    float current_a = 0.0f;

    mock_bsp_reset();
    mock_ina237_set_reading(0x40u, 5.2f, 1.5f, 0.001f);

    assert(power_service_init() == ROV_OK);
    assert(power_read_rail(&voltage_v, &current_a) == ROV_OK);
    assert(fabsf(voltage_v - 5.2f) < 0.01f);
    assert(fabsf(current_a - 1.5f) < 0.01f);

    /* A bus fault fails the read and leaves the caller's outputs untouched. */
    mock_bsp_reset();
    mock_bsp_i2c_set_fault(ROV_TIMEOUT);
    voltage_v = 7.0f;
    current_a = 7.0f;
    assert(power_read_rail(&voltage_v, &current_a) == ROV_TIMEOUT);
    assert(voltage_v == 7.0f);
    assert(current_a == 7.0f);

    mock_bsp_reset();
    printf("  [PASS] Power service rail ownership and fault handling\n");
}
```

Register it in `main()` after `test_power_service();`:

```c
    test_power_service();
    test_power_service_rail();
    test_env_service();
```

- [ ] **Step 2: Run the test to verify it fails**

Run:
```powershell
$env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"
cmake --build build-native --target test_services
ctest --test-dir build-native -R "^test_services$" --output-on-failure
```
Expected: FAIL to compile/link with `undefined reference to 'power_service_init'` (and `power_read_rail`).

- [ ] **Step 3: Implement the service**

In `services/include/power_service.h`, add `#include "rov_types.h"` after the `stdint.h` include, and declare the two functions before the closing `extern "C"`:

```c
#include <stdbool.h>
#include <stdint.h>

#include "rov_types.h"
```

```c
/**
 * @brief Initialize the power service and its 5V rail monitor (INA237).
 *
 * Acquires the board I2C bus and configures the rail monitor. Safe to call
 * more than once; the last result is retained.
 *
 * @return ROV_OK on success, error code otherwise.
 */
rov_status_t power_service_init(void);

/**
 * @brief Read the 5V rail voltage and current in engineering units.
 * @param bus_voltage_v Pointer to store bus voltage in volts (may be NULL).
 * @param current_a Pointer to store shunt current in amperes (may be NULL).
 * @return ROV_OK if readings are valid, error code otherwise. On error the
 *         caller's outputs are left untouched.
 */
rov_status_t power_read_rail(float *bus_voltage_v, float *current_a);
```

In `services/src/power_service.c`, replace the top of the file (through the include block) with:

```c
/**
 * @file power_service.c
 * @brief Implementation of power conversion and monitoring domain service.
 * @author Purdue ROV Embedded Team
 */

#include "power_service.h"
#include "bsp.h"
#include "ina237.h"
#include <string.h>

/*
 * Node 1 Pi Shield 5V rail monitor. The service owns the device config so the
 * application never names a driver, an address, or a bus. Per-board rail
 * configuration is deferred to the node 3 power sub-project.
 */
#define POWER_RAIL_INA237_ADDR (0x40u)
#define POWER_RAIL_SHUNT_OHMS  (0.001f)

static ina237_dev_t g_rail_dev;
static bool g_rail_initialized = false;

rov_status_t power_service_init(void) {
    memset(&g_rail_dev, 0, sizeof(g_rail_dev));

    rov_status_t status =
        ina237_init(&g_rail_dev, bsp_i2c_bus(), POWER_RAIL_INA237_ADDR, POWER_RAIL_SHUNT_OHMS);
    g_rail_initialized = (status == ROV_OK);
    return status;
}

rov_status_t power_read_rail(float *bus_voltage_v, float *current_a) {
    if (!g_rail_initialized) {
        rov_status_t init_status = power_service_init();
        if (init_status != ROV_OK) {
            return init_status;
        }
    }

    rov_status_t status = ina237_read_power(&g_rail_dev);
    if (status == ROV_OK) {
        if (bus_voltage_v != NULL) {
            *bus_voltage_v = g_rail_dev.bus_voltage_v;
        }
        if (current_a != NULL) {
            *current_a = g_rail_dev.shunt_current_a;
        }
    }

    return status;
}
```

Leave the four existing node3 pass-through functions (`power_bricks_disable_all`, `power_get_pcb_temperature_c`, `power_lm74700_is_ok`, `power_get_logic_voltage_mv`) unchanged below this block.

- [ ] **Step 4: Run the test to verify it passes**

Run:
```powershell
$env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"
cmake --build build-native --target test_services
ctest --test-dir build-native -R "^test_services$" --output-on-failure
```
Expected: PASS, output includes `[PASS] Power service rail ownership and fault handling`.

- [ ] **Step 5: Format and commit**

Run:
```powershell
clang-format --dry-run --Werror services/include/power_service.h services/src/power_service.c tests/test_services.c
git add services/include/power_service.h services/src/power_service.c tests/test_services.c
git commit -m "feat(services): power_service owns the node1 rail INA237"
```

---

### Task 2: 0x210 carries the rail; node1 app routes through the service

**Files:**
- Modify: `shared/include/rov_types.h:127-135`
- Modify: `nodes/node1_pi_shield/Core/Src/app.c`
- Test: `tests/test_can_protocol.c:130-151`, `tests/test_can_protocol.c:205-209`, `tests/test_node1_pi_shield.c`

**Interfaces:**
- Consumes: `power_service_init()` / `power_read_rail()` from Task 1.
- Produces: `rov_env_telemetry_t` fields `uint16_t rail_voltage_mv` and `uint16_t rail_current_ma` (offsets 13 and 15; `sizeof` is now 17).

- [ ] **Step 1: Grow the wire struct**

In `shared/include/rov_types.h`, replace the env struct (lines 127-135) with:

```c
/**
 * @brief 10 Hz Environmental & Leak Telemetry from Pi Shield (Packed, 17 bytes).
 */
typedef struct __attribute__((packed)) {
    float pressure_hpa;       /**< Enclosure internal pressure in hPa (BME280) */
    float humidity_pct;       /**< Enclosure relative humidity in % (BME280) */
    float temperature_c;      /**< Enclosure internal temperature in deg C (BME280) */
    uint8_t leak_flags;       /**< Bit 0: BME280 trigger, Bit 1: Floor leak trace 1, Bit 2: Floor leak trace 2 */
    uint16_t rail_voltage_mv; /**< Node 1 Pi 5V rail voltage in mV (INA237); 0 when invalid */
    uint16_t rail_current_ma; /**< Node 1 Pi 5V rail current in mA (INA237); 0 when invalid */
} rov_env_telemetry_t;
```

- [ ] **Step 2: Write the failing tests**

In `tests/test_can_protocol.c`, `test_env_telemetry_pack_unpack()`:

```c
    rov_env_telemetry_t original = {.pressure_hpa = 1013.25f,
                                    .humidity_pct = 45.5f,
                                    .temperature_c = 22.1f,
                                    .leak_flags = 5,
                                    .rail_voltage_mv = 5210u,
                                    .rail_current_ma = 1350u};
```

and after the existing field assertions add:

```c
    assert(decoded.leak_flags == original.leak_flags);
    assert(decoded.rail_voltage_mv == original.rail_voltage_mv);
    assert(decoded.rail_current_ma == original.rail_current_ma);
```

In the same file's negative test (around line 205), after `assert(rov_can_unpack_env_telemetry(buffer, 5, &env) == ROV_ERR_INVALID_ARG);` add:

```c
    /* A 13-byte frame is the old layout and must be rejected, not truncated. */
    assert(rov_can_unpack_env_telemetry(buffer, 13, &env) == ROV_ERR_INVALID_ARG);
```

In `tests/test_node1_pi_shield.c`, inside `test_node1_nominal_telemetry()` after `assert(env.leak_flags == 0);` add:

```c
    /*
     * The INA237 at 0x40 was placed at 5.21 V / 1.35 A. 5.21 V quantizes to
     * 1667 bus counts at 3.125 mV each -> 5209 mV; 1.35 A through 0.001 ohm is
     * exactly 270 shunt counts at 5 uV each -> 1350 mA.
     */
    assert(env.rail_voltage_mv >= 5208u && env.rail_voltage_mv <= 5210u);
    assert(env.rail_current_ma == 1350u);
```

Add this new test before `int main(void)`:

```c
void test_node1_rail_zero_when_absent(void) {
    setup();

    set_bme280_reading(1013.25f, 35.0f, 24.0f);
    /* No mock_ina237_set_reading(): the mock register file reads as zero. */

    node1_app_init();

    mock_bsp_advance_time_ms(100);
    node1_app_step();

    uint32_t tx_id = 0;
    uint8_t data[64];
    uint8_t len = 0;

    assert(mock_can_get_last_tx(&tx_id, data, &len));
    assert(tx_id == ROV_CAN_ID_ENV_TELEMETRY);

    rov_env_telemetry_t env;

    assert(rov_can_unpack_env_telemetry(data, len, &env) == ROV_OK);
    assert(env.rail_voltage_mv == 0u);
    assert(env.rail_current_ma == 0u);

    printf("[PASS] test_node1_rail_zero_when_absent\n");
}
```

Add this second new test before `int main(void)`:

```c
void test_node1_rail_saturates(void) {
    setup();

    set_bme280_reading(1013.25f, 35.0f, 24.0f);
    /* 100 A through 0.001 ohm is 100000 mA, which would wrap a uint16_t. */
    mock_ina237_set_reading(0x40u, 5.0f, 100.0f, 0.001f);

    node1_app_init();

    mock_bsp_advance_time_ms(100);
    node1_app_step();

    uint32_t tx_id = 0;
    uint8_t data[64];
    uint8_t len = 0;

    assert(mock_can_get_last_tx(&tx_id, data, &len));
    assert(tx_id == ROV_CAN_ID_ENV_TELEMETRY);

    rov_env_telemetry_t env;

    assert(rov_can_unpack_env_telemetry(data, len, &env) == ROV_OK);
    assert(env.rail_voltage_mv == 5000u);
    assert(env.rail_current_ma == 65535u);

    printf("[PASS] test_node1_rail_saturates\n");
}
```

Register both in `main()` after `test_node1_nominal_telemetry();`:

```c
    test_node1_nominal_telemetry();
    test_node1_rail_zero_when_absent();
    test_node1_rail_saturates();
```

- [ ] **Step 3: Run the tests to verify they fail**

Run:
```powershell
$env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"
cmake --build build-native --target test_can_protocol test_node1_pi_shield
ctest --test-dir build-native -R "^(test_can_protocol|test_node1_pi_shield)$" --output-on-failure
```
Expected: both FAIL. `test_can_protocol` fails the rail roundtrip (`decoded.rail_voltage_mv` is 0, not 5210) and/or the 13-byte rejection; `test_node1_pi_shield` fails the nominal rail assertion and the saturation assertion (both read 0 because the app does not publish rail yet). The zero-when-absent case already passes for the wrong reason (the mock reads zeros).

- [ ] **Step 4: Route the app through the service**

In `nodes/node1_pi_shield/Core/Src/app.c`:

Replace the include line `#include "ina237.h"` with `#include "power_service.h"`.

Delete the two static declarations:

```c
static ina237_dev_t g_ina237_dev;
```
```c
static bool g_ina237_valid = false;
```

Add a saturation helper after the remaining static declarations (before `node1_get_floor_leak_bits()`):

```c
/*
 * Convert a non-negative engineering value to an unsigned 16-bit field,
 * saturating instead of wrapping. NaN and negatives map to 0.
 */
static uint16_t node1_saturate_u16(float value) {
    if (!(value > 0.0f)) {
        return 0u;
    }

    if (value >= 65535.0f) {
        return 65535u;
    }

    return (uint16_t)value;
}
```

In `node1_app_init()`, replace:

```c
    env_service_init();
    ina237_init(&g_ina237_dev, bsp_i2c_bus(), 0x40, 0.001f);
    g_bme280_valid = false;
    g_ina237_valid = false;
```

with:

```c
    env_service_init();
    power_service_init();
    g_bme280_valid = false;
```

Later in `node1_app_init()`, after `g_env_telemetry.leak_flags = 0;`, add the rail defaults so every field is explicitly initialised:

```c
    g_env_telemetry.leak_flags = 0;
    g_env_telemetry.rail_voltage_mv = 0;
    g_env_telemetry.rail_current_ma = 0;
```

In `node1_app_step()`, replace:

```c
        float temp_c = 0.0f;
        float press_hpa = 0.0f;
        float hum_pct = 0.0f;
```
with:
```c
        float temp_c = 0.0f;
        float press_hpa = 0.0f;
        float hum_pct = 0.0f;
        float rail_v = 0.0f;
        float rail_i = 0.0f;
```

Replace:

```c
        g_bme280_valid = (env_read_all(&temp_c, &press_hpa, &hum_pct) == ROV_OK) && isfinite(press_hpa) &&
                         isfinite(hum_pct) && isfinite(temp_c);
        g_ina237_valid = (ina237_read_power(&g_ina237_dev) == ROV_OK) && isfinite(g_ina237_dev.bus_voltage_v) &&
                         isfinite(g_ina237_dev.shunt_current_a);
```

with:

```c
        g_bme280_valid = (env_read_all(&temp_c, &press_hpa, &hum_pct) == ROV_OK) && isfinite(press_hpa) &&
                         isfinite(hum_pct) && isfinite(temp_c);
        bool rail_valid =
            (power_read_rail(&rail_v, &rail_i) == ROV_OK) && isfinite(rail_v) && isfinite(rail_i);
```

Replace:

```c
        g_env_telemetry.leak_flags = leak_bits;
```

with:

```c
        g_env_telemetry.leak_flags = leak_bits;

        g_env_telemetry.rail_voltage_mv = rail_valid ? node1_saturate_u16(rail_v * 1000.0f) : 0u;

        g_env_telemetry.rail_current_ma = rail_valid ? node1_saturate_u16(rail_i * 1000.0f) : 0u;
```

- [ ] **Step 5: Run the tests to verify they pass**

Run:
```powershell
$env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"
cmake --build build-native --target test_can_protocol test_node1_pi_shield
ctest --test-dir build-native -R "^(test_can_protocol|test_node1_pi_shield)$" --output-on-failure
```
Expected: PASS, including `[PASS] test_node1_rail_zero_when_absent` and `[PASS] test_node1_rail_saturates`.

- [ ] **Step 6: Format and commit**

Run:
```powershell
clang-format --dry-run --Werror shared/include/rov_types.h nodes/node1_pi_shield/Core/Src/app.c tests/test_can_protocol.c tests/test_node1_pi_shield.c
git add shared/include/rov_types.h nodes/node1_pi_shield/Core/Src/app.c tests/test_can_protocol.c tests/test_node1_pi_shield.c
git commit -m "feat(node1): publish the 5V rail on CAN 0x210"
```

---

### Task 3: Update every 0x210 consumer to 17 bytes

**Files:**
- Modify: `tests/sil_bridge_server.c:440`
- Modify: `tests/sil_companion_bridge/sil_protocol.py:41,169-181`
- Modify: `tests/sil_stimulus/test_stimulus_flows.py:114-121,1582-1596,3289,3546,3567-3568`
- Modify: `tests/sil_stimulus/test_dashboard_app.py:2137-2150,2277`
- Modify: `tools/can_sniffer.py:43,125-131`
- Modify: `tools/can_stimulus.py:1216,1281-1283`

**Interfaces:**
- Consumes: the 17-byte `rov_env_telemetry_t` from Task 2.
- Produces: `sil_protocol.EnvTelemetry` with `rail_voltage_mv` / `rail_current_ma` fields and a 17-byte minimum; `sil_protocol._STRUCT_ENV` (`"<3fBHH"`) mirrored by `can_sniffer._STRUCT_ENV` (`"<fffBHH"`).

- [ ] **Step 1: Confirm the SIL suites fail on the new length**

Run:
```powershell
$env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"
cmake --build build-native --target sil_bridge_server
python -m unittest tests.sil_stimulus.test_stimulus_flows.TestStimulusVehicleCheckFailures -v
```
Expected: FAIL with an assertion like `0x210 frame 0 is 17 B; ... is 13 B`.

- [ ] **Step 2: Fix the bridge length literal**

In `tests/sil_bridge_server.c`, change the 0x210 branch condition:

```c
                } else if (rx_id == ROV_CAN_ID_ENV_TELEMETRY) {
                    /* Enclosure telemetry: instant if leak, otherwise 1 Hz (initial frame instant) */
                    if ((rx_len >= sizeof(rov_env_telemetry_t) && rx_data[12] != 0) || !s_env_initial_sent ||
                        (now_ms - s_last_tx_env >= 1000)) {
```

- [ ] **Step 3: Update `sil_protocol.py`**

Replace line 41:

```python
_STRUCT_ENV = struct.Struct("<3fBHH")
```

Replace the `EnvTelemetry` class:

```python
@dataclass
class EnvTelemetry:
    pressure_hpa: float
    humidity_pct: float
    temperature_c: float
    leak_flags: int
    rail_voltage_mv: int
    rail_current_ma: int

    @classmethod
    def unpack(cls, data: bytes) -> "EnvTelemetry":
        if len(data) < 17:
            raise ValueError(f"EnvTelemetry requires at least 17 bytes, got {len(data)}")
        p, h, t, flags, rail_mv, rail_ma = _STRUCT_ENV.unpack(data[:17])
        return cls(
            pressure_hpa=p,
            humidity_pct=h,
            temperature_c=t,
            leak_flags=flags,
            rail_voltage_mv=rail_mv,
            rail_current_ma=rail_ma,
        )
```

- [ ] **Step 4: Update `test_stimulus_flows.py`**

Replace the fixture comment line 116 and the builder:

```python
#   env   "<3fBHH" pressure_hpa, humidity_pct, temperature_c, leak_flags, rail mV, rail mA (17 B)
```

```python
def env_payload(
    pressure_hpa=1013.25,
    humidity_pct=35.0,
    temperature_c=24.0,
    leak_flags=0,
    rail_voltage_mv=5200,
    rail_current_ma=1350,
):
    return struct.pack(
        "<3fBHH", pressure_hpa, humidity_pct, temperature_c, leak_flags, rail_voltage_mv, rail_current_ma
    )
```

In `TestCanSnifferAgreesWithSilProtocol`, replace the comment and mapping entry:

```python
    # can_sniffer and sil_protocol now use the same struct name for the 0x210
    # layout; the formats still differ as text ("<fffBHH" vs "<3fBHH") and the
    # comparison is by layout, which is exactly the kind of difference a text
    # comparison would either reject or paper over.
    STRUCT_NAMES = {
        "_STRUCT_TIME_SYNC_MASTER": "_STRUCT_TIME_SYNC_MASTER",
        "_STRUCT_TIME_SYNC_REQ": "_STRUCT_TIME_SYNC_REQ",
        "_STRUCT_TIME_SYNC_RESP": "_STRUCT_TIME_SYNC_RESP",
        "_STRUCT_8H": "_STRUCT_8H",
        "_STRUCT_H": "_STRUCT_H",
        "_STRUCT_NAV_TS": "_STRUCT_NAV_TS",
        "_STRUCT_NAV_LEGACY": "_STRUCT_NAV_LEGACY",
        "_STRUCT_ENV": "_STRUCT_ENV",
        "_STRUCT_POWER": "_STRUCT_POWER",
    }
```

Replace the constant at line 3289:

```python
SIL_ENV_PAYLOAD_LEN = 17  # struct "<3fBHH" (sil_protocol._STRUCT_ENV)
```

Update the docstring/message text in the server-backed env test:
- line 3546: `that each is a full 13-byte record,` -> `that each is a full 17-byte record,`
- line 3567-3568: `f"0x210 frame {index} is {len(payload)} B; the packed rov_env_telemetry_t "` `"is 13 B and a short record cannot be a valid reading",` -> `"is 17 B and a short record cannot be a valid reading",`

- [ ] **Step 5: Update `test_dashboard_app.py`**

Replace the docstring lines 2137-2140 and the payload:

```python
    The workhorse frame is ``CAN_ID_ENV_TELEMETRY``, whose payload is
    ``struct.Struct("<3fBHH")`` -- three floats, a leak-flags byte and two rail
    words, so a known value is trivial to build and ``client.env_data.leak_flags``
    is an exact observable rather than a float comparison.
```

```python
    #: The env payload the real server sends, as ``<3fBHH``.
    ENV_LEAK_FLAGS = 0x03
    ENV_PAYLOAD = struct.pack("<3fBHH", 1013.25, 42.0, 26.5, ENV_LEAK_FLAGS, 5200, 1350)
```

Update the comment at line 2277:

```python
        ``EnvTelemetry.unpack`` requires 17 bytes, so 16 is undecodable while
```

- [ ] **Step 6: Update `tools/can_sniffer.py`**

Replace line 43:

```python
_STRUCT_ENV              = struct.Struct("<fffBHH")
```

Replace the 0x210 branch (lines 125-131):

```python
    elif msg_id == CAN_ID_ENV_TELEMETRY:
        if len(data) == 17:
            press, hum, temp, leak, rail_mv, rail_ma = _STRUCT_ENV.unpack(data[:17])
            leak_str = "LEAK ALERT!" if leak != 0 else "OK"
            print(f"[{ts}] [CAN 0x210] Env: P={press:6.1f}hPa | Hum={hum:4.1f}% | Temp={temp:4.1f}C | Rail={rail_mv/1000.0:4.2f}V @ {rail_ma}mA | Status={leak_str}")
        else:
            print(f"[{ts}] [CAN 0x210] WARNING: Malformed Env Telemetry packet (len={len(data)}, expected 17)")
```

- [ ] **Step 7: Update `tools/can_stimulus.py`**

Replace the docstring phrase at line 1216:

```python
        decodes as a full 17-byte ``EnvTelemetry`` record, that the values are
```

Replace the summary `CheckResult` detail (lines 1281-1283):

```python
            f"{len(records)} frame(s) decoded in {duration_s:.1f} s: "
            f"pressure={last.pressure_hpa:.2f} hPa humidity={last.humidity_pct:.1f} % "
            f"temperature={last.temperature_c:.1f} C leak_flags=0x{last.leak_flags:02X} "
            f"rail={last.rail_voltage_mv} mV @ {last.rail_current_ma} mA",
```

- [ ] **Step 8: Run the Python suites to verify they pass**

Run:
```powershell
$env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"
cmake --build build-native --target sil_bridge_server
python -m unittest tests.sil_stimulus.test_stimulus_flows tests.sil_stimulus.test_dashboard_app -v
```
Expected: PASS (server-backed cases self-skip if the binary is missing, but it was just built). The drift guard `TestCanSnifferAgreesWithSilProtocol` must pass.

- [ ] **Step 9: Commit**

```powershell
git add tests/sil_bridge_server.c tests/sil_companion_bridge/sil_protocol.py tests/sil_stimulus/test_stimulus_flows.py tests/sil_stimulus/test_dashboard_app.py tools/can_sniffer.py tools/can_stimulus.py
git commit -m "test(sil): track the 17-byte 0x210 env frame"
```

---

### Task 4: Full verification and doc sync

**Files:**
- Modify: `System_Software/Docs/x19_architecture/x19_firmware_tasks_and_deadlines.md` (outside this repo; edit in place, no commit here)

- [ ] **Step 1: Native test suite**

Run:
```powershell
$env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"
cmake --build build-native
ctest --test-dir build-native -E "^target_" --output-on-failure
```
Expected: all pass (36 or more tests). The `target_*` acceptance contracts stay expected-red and are excluded.

- [ ] **Step 2: ARM cross build**

Run:
```powershell
cmake --build --preset arm-c542-debug
```
Expected: exit 0.

- [ ] **Step 3: Format gate on every changed C file**

Run:
```powershell
clang-format --dry-run --Werror services/include/power_service.h services/src/power_service.c nodes/node1_pi_shield/Core/Src/app.c shared/include/rov_types.h tests/test_services.c tests/test_can_protocol.c tests/test_node1_pi_shield.c tests/sil_bridge_server.c
```
Expected: exit 0 (no output).

- [ ] **Step 4: Full SIL companion test (if the environment is set up)**

Run:
```powershell
$env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"
cmake --build build-native --target sil_bridge_server
python tests/sil_companion_bridge/test_full_system_sil.py
```
Expected: PASS. If the generated Python bindings or adjacent Core/Surface dependencies are absent, note the skip and move on; the Task 3 suites are the primary SIL gate.

- [ ] **Step 5: Sync the architecture doc**

In `System_Software/Docs/x19_architecture/x19_firmware_tasks_and_deadlines.md`, change the 0x210 line:

```
  - `rov_can_pack_env_telemetry()` / `rov_can_unpack_env_telemetry()` (`0x210`: pressure, humidity, temp, leak flag, 13 bytes).
```

to:

```
  - `rov_can_pack_env_telemetry()` / `rov_can_unpack_env_telemetry()` (`0x210`: pressure, humidity, temp, leak flag, 5V rail mV/mA, 17 bytes).
```

This file lives outside `Embedded/X19-Embedded/`; edit it in place and do not commit it in this repository.

- [ ] **Step 6: Confirm the working tree**

Run:
```powershell
git status --short
git log --oneline -4
```
Expected: clean tree (aside from the out-of-repo doc) and four commits for this feature: the spec, Task 1, Task 2, Task 3.
