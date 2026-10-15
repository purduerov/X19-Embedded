# Injected I2C Bus for Sensor Drivers - Design

- Date: 2026-10-15
- Owner: Purdue ROV Embedded Team
- Status: Draft for review

## Goal

One I2C sensor-driver source that runs unchanged in three homes - STM32 target
firmware, host SIL/unit tests, and a Nucleo bench harness - by injecting the bus
instead of calling `bsp_i2c_mem_read()` directly.

This follows the industry-standard split (Linux `bmp280-core.c` + `bmp280-i2c.c`;
Zephyr `bme280.c` + `bme280_i2c.c`; Bosch's own `bme280_dev.read/write/delay_ms`
function pointers). We are not inventing a scheme; we are matching it.

## Success criteria

- A driver compiles and runs in all three homes with no `#ifdef` selecting the bus.
- Host tests exercise the **real** driver against a **bus-level** mock.
- Adding a new I2C sensor is: write `<chip>.c`/`<chip>.h`, link, add a bench descriptor.
- No vendor HAL call and no `bsp_*` call inside any driver.
- New surface stays small and enumerated: one contract header (`rov_i2c.h`), one bus
  mock, one HAL bus shim, and the `bench/` folder.

## Current state (why this change)

- Drivers call the global `bsp_i2c_mem_read()` directly, so a driver only builds
  where that BSP exists. Testing one sensor requires linking the whole BSP.
- Bench tests are throwaway standalone `app_main`s (`sandbox/test_ms5837.c`,
  `i2c_scanner.c`); `sandbox.c` bypasses both drivers and `bsp.h` and talks to
  `stm32g4xx_hal.h` directly.
- Host tests mock the **driver** (`tests/mocks/mock_bme280.c`), so the real driver
  code path never runs on the host.
- `drivers/src/lsm6dsoxtr.c` embeds a Madgwick filter and calls a `weak`
  `mock_sensors_get_imu()` from production code - the driver depends on the mock.

## Design

### The contract (`shared/include/rov_i2c.h`)

One tiny vtable, header-only helpers:

```c
typedef struct rov_i2c_bus rov_i2c_bus_t;

struct rov_i2c_bus {
    void *ctx;
    rov_status_t (*mem_read)(void *ctx, uint8_t addr, uint8_t reg, uint8_t *dst, uint16_t len);
    rov_status_t (*mem_write)(void *ctx, uint8_t addr, uint8_t reg, const uint8_t *src, uint16_t len);
    bool         (*probe)(void *ctx, uint8_t addr);
};

rov_status_t rov_i2c_read_u8(const rov_i2c_bus_t *bus, uint8_t addr, uint8_t reg, uint8_t *v);
rov_status_t rov_i2c_read_u16le(const rov_i2c_bus_t *bus, uint8_t addr, uint8_t reg, uint16_t *v);
rov_status_t rov_i2c_write_u8(const rov_i2c_bus_t *bus, uint8_t addr, uint8_t reg, uint8_t v);
```

Helpers are `static inline` in the header, or a tiny `rov_i2c.c` if they grow.

### Driver shape

Each I2C driver stores the bus and routes all register access through it:

```c
typedef struct {
    const rov_i2c_bus_t *bus;   /* injected; replaces the bsp_i2c_* dependency */
    uint8_t i2c_addr;
    /* ... existing fields ... */
} bme280_dev_t;

rov_status_t bme280_init(bme280_dev_t *dev, const rov_i2c_bus_t *bus);
rov_status_t bme280_read_all(bme280_dev_t *dev);
```

Internal diff is mechanical: `bsp_i2c_mem_read(addr, R, d, n)` becomes
`dev->bus->mem_read(dev->bus->ctx, addr, R, d, n)`.

### Driver boundary rules

1. Driver = bytes to engineering units, exactly per the chip datasheet. Stateless
   and deterministic. No history, no thresholds, no policy.
2. Configuring the chip's own internal hardware filter (e.g. BME280 IIR register)
   is datasheet work and stays in the driver. Software filtering does not.
3. Drivers contain no `bsp_*` or vendor HAL calls.
4. Drivers contain no reference to any `mock_*` symbol.

Test for scope: needs a history of samples -> filter layer; needs a vehicle
parameter or a decision -> service/app; pure per-sample datasheet transform -> driver.

### The three places

Three sensor-driver homes, three builds (the target firmware and the Nucleo bench
share one transport implementation):

| Home / build | Transport implementation | Links |
| :--- | :--- | :--- |
| STM32 target firmware | `i2c_bus_hal.c` (wraps `bsp_i2c_mem_*`) | `drivers` lib |
| Nucleo bench | `i2c_bus_hal.c` (same as target) | `drivers` lib |
| Host SIL / unit tests | `tests/mocks/i2c_bus_mock.c` (register image + fault flags) | `drivers` lib |

App code names only `bsp_i2c_bus()` (added to the node BSP) and cannot tell which
transport it received.

### Bench harness (`bench/`)

- Nucleo dev board only (F411 / G474). No host/USB transport, no Pi path.
- `bench_main.c`: generic runner - init the HAL bus, init one sensor, read + print at N Hz.
- `bench_sensors.c` / `.h`: descriptor table `{name, init(dev,bus), read(dev), print(dev)}`.
- One firmware, many sensors: replaces writing a bespoke `app_main` per sensor.

### Non-goals (deliberately excluded, YAGNI)

- No `sensors/` wrapper tree. No filters exist except Madgwick; add a filter module
  only when a second consumer appears.
- No `transports/` tree. Each transport lives in its only home (mock in `tests/mocks/`,
  HAL in the node BSP).
- No host-PC / USB-to-I2C bench transport. The injected contract makes it a one-file
  addition later if it is ever wanted, so deferring costs nothing architecturally.
- No `_template/` folder and no per-driver CMake targets. One `drivers` static
  library; the linker drops unreferenced objects.
- Keep `drivers/include` + `drivers/src` flat.
- SPI drivers (`bmi270`, `lsm6dsoxtr`), PMBus, `tcan1044`, and DShot are out of
  scope for the transport migration and are left as-is.

## Migration order

1. Add `shared/include/rov_i2c.h` and the helpers.
2. Add `tests/mocks/i2c_bus_mock.c`; convert one driver (MS5837) and its test as the
   proof, including a fault-injection case.
3. Convert remaining I2C drivers: BME280, INA226, INA237, TMP1075.
4. Add `bsp_i2c_bus()` to each node BSP; update the three `app.c` init call sites.
5. Driver hygiene (independent of transport): remove the `weak mock_*` hook from
   `lsm6dsoxtr.c` and move Madgwick out of that driver to its consumer.
6. Add `bench/` as a Nucleo dev-board build target (F411 / G474).
7. Delete the `tests/mocks/mock_<sensor>.c` driver-level mocks.

## Testing

- Per-driver host test: load a fixed register image captured from the datasheet,
  run the real driver, assert exact engineering units.
- Fault injection via the bus mock (NACK / timeout / short read / corrupt byte):
  assert the driver returns `ROV_ERROR` and leaves prior output untouched (no stale
  zero surfaced).
- Bench run: the same `bench_main.c` flashed to the Nucleo, readings checked over serial.
- CI: host unit tests remain blocking; the target acceptance gate stays as-is.

## Risks and open questions

- `lsm6dsoxtr_read_raw()` is currently a pure mock passthrough and ignores hardware.
  Step 5 only removes the leak; making that path real is separate SPI work and is
  not claimed here.
- The bench harness is Nucleo-only, so a real-sensor check always requires flashing a
  dev board. Accepted: it keeps the surface at one transport and one new folder.
