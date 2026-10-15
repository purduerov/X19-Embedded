# Node Architecture: Boundaries, Injected I2C Bus, and Isolated Testing

- Date: 2026-10-15
- Owner: Purdue ROV Embedded Team
- Status: Draft for review

## Goal

Establish one node architecture with clean layer boundaries, then make the code
match it. Two outcomes:

1. A sensor driver runs unchanged in three homes - STM32 target firmware, host
   tests, and a Nucleo bench - by injecting the I2C bus instead of calling
   `bsp_i2c_mem_read()` directly.
2. Every layer has one job, no layer reaches past another, and any single sensor
   or function can be tested in isolation without touching the rest of the system.

This follows the industry-standard split (Linux `bmp280-core.c` + `bmp280-i2c.c`;
Zephyr `bme280.c` + `bme280_i2c.c`; Bosch's own `bme280_dev.read/write/delay_ms`
function pointers). We are matching it, not inventing it.

## Current state (why)

- Drivers call the global `bsp_i2c_mem_read()` directly, so a driver only builds
  where that BSP exists. Testing one sensor requires linking the whole BSP.
- Host tests mock the **driver** (`tests/mocks/mock_bme280.c`), so the real driver
  code path never runs on the host.
- `bsp.h` mixes board IO with device functions (`bsp_pmbus_read_word`,
  `bsp_get_pcb_temperature_c`, `bsp_get_logic_voltage_mv`).
- `env_service`/`power_service` forward to one driver **and** duplicate its state.
- `app.c` reaches past services straight into drivers, inconsistently.
- Three sources of "is the vehicle safe?" (`app.c`, `safety_service`, `bsp`).
- `bsp.c` wraps real actions in `#if defined(...)` fallbacks that silently return
  `false`/`0`, so a missing board macro makes the emergency brake a no-op that
  still "succeeds". The board macros exist today only in
  `tests/hardware/fakes/main.h`, not in the real node headers.
- Dual header namespaces (`rov_*.h` and `x19_*.h` forwarders).

## The established architecture

Each layer gets exactly one job:

| Layer | One job | May call |
| :--- | :--- | :--- |
| `app.c` | policy + scheduling: loop, cadence, thresholds, decisions, telemetry | services, bsp timing |
| `services/` | sensor facades (own a driver, expose units) and invariant guards | drivers, bsp |
| `drivers/` | one chip: bytes to engineering units | the injected bus only |
| `bsp` (+ `boards/*/platform`) | board hardware: pins, clocks, peripherals, timing | HAL |
| `shared/` | contracts: protocol, params, safety types, bus | pure |
| generated (`main.c`, `*_it.c`, `*_msp.c`) | CubeMX startup/interrupt/MSP | HAL |
| `sandbox/` | free-for-all leaf (throwaway experiments) | anything |

Rules:

1. Nothing calls upward (`app -> services -> drivers -> bus -> bsp -> HAL`).
2. Drivers never call `bsp_*` or HAL; only the injected bus.
3. BSP contains no device/register logic.
4. One owner per piece of state: one safety-state source; no second sensor cache.
5. `app.c` speaks to services only; it never includes a driver header or names a bus.

## The bus contract (`shared/include/rov_i2c.h`)

The primitive is a raw byte transfer, because two of the parts are
command-addressed rather than register-addressed (the MS5837 takes a command byte
and answers with a word; a register-pointer read is meaningless to it). `mem_*` is
a convenience built on the raw pair.

```c
typedef struct rov_i2c_bus rov_i2c_bus_t;

struct rov_i2c_bus {
    void *ctx;
    rov_status_t (*write)(void *ctx, uint8_t addr, const uint8_t *data, uint16_t len);
    rov_status_t (*read)(void *ctx, uint8_t addr, uint8_t *data, uint16_t len);
    bool         (*probe)(void *ctx, uint8_t addr);
};

/* write one register address, then read len bytes */
static inline rov_status_t rov_i2c_mem_read(const rov_i2c_bus_t *bus, uint8_t addr,
                                            uint8_t reg, uint8_t *dst, uint16_t len);
/* write register address followed by len data bytes */
static inline rov_status_t rov_i2c_mem_write(const rov_i2c_bus_t *bus, uint8_t addr,
                                             uint8_t reg, const uint8_t *src, uint16_t len);
```

Both helpers are `static inline` with a NULL/`len` guard returning
`ROV_ERR_INVALID_ARG`. This mirrors the BSP it replaces: `bsp_i2c_write`/
`bsp_i2c_read` (raw, returning `bool`) and `bsp_i2c_mem_read`/`mem_write`
(register-addressed, returning `rov_status_t`).

Timing is not part of this contract. Drivers keep using `time_get_ms()` and
`delay_ms()`; host tests supply them, so a driver still needs no board to build.

## Driver shape and boundary rules

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
`rov_i2c_mem_read(dev->bus, addr, R, d, n)`. A command-addressed part (MS5837)
instead drives the raw pair: `dev->bus->write(...)` then `dev->bus->read(...)`.

Rules:

1. Driver = bytes to engineering units, exactly per datasheet. Stateless,
   deterministic. No history, no thresholds, no policy.
2. Configuring the chip's own hardware filter (e.g. BME280 IIR register) is
   datasheet work and stays in the driver. Software filtering does not.
3. No `bsp_*`, no vendor HAL, and no `mock_*` reference inside a driver.

Scope test: needs history -> filter layer; needs a vehicle parameter or a decision
-> service/app; pure per-sample datasheet transform -> driver.

## The three homes of a driver

| Home / build | Transport implementation | Links |
| :--- | :--- | :--- |
| STM32 target firmware | `bsp_i2c_bus()` wrapping `bsp_i2c_mem_*` | `rov_drivers` |
| Nucleo bench | same HAL transport | `rov_drivers` |
| Host SIL / unit tests | `bsp_i2c_bus()` in `tests/mocks/mock_bsp.c`, wrapping the register file the mocks already keep, plus fault flags | `rov_drivers` |

Services acquire the bus themselves (`bsp_i2c_bus()`), so `app.c` never mentions
a bus or a transport.

The host transport is cheap because most of it already exists: `mock_bsp.c`
keeps a 128x256 register file and already implements `bsp_i2c_mem_read/write`
against it, and register-addressed parts already run through it. A driver moving
to the injected bus behaves identically on the host. Command-addressed parts are
the exception: `mock_bsp.c`'s raw `bsp_i2c_read` currently returns zeros, so the
MS5837 has been leaning on a value-level mock hook inside the driver. Closing
that gap is its own task.

## Overlaps and oversteps to fix

| # | Problem | Fix |
| :--- | :--- | :--- |
| 1 | `bsp.h` exposes device fns | move into drivers; services wrap them |
| 2 | `env_service`/`power_service` duplicate driver state | keep the services; they own the driver, delete the duplicate cache |
| 3 | `app.c` reaches into drivers | forbidden; route every sensor through a service |
| 4 | three safety-state copies | single source of truth in `rov_safety` / `safety_service` |
| 5 | silent `#if defined` fallbacks | fail loud: `#error` when a required board macro is missing |
| 6 | dual `rov_*` / `x19_*` headers | delete the `x19_*` forwarders |

## Isolated testing: composable towers

The layers are separate static libraries, so any test can link a tower of exactly
the height it needs. Nothing else is compiled into that target, so one broken
sensor cannot break another target or the node firmwares.

**Mode 1 - Host test, through the tower, no hardware** (runs in CI):

```
tests/test_env_service.c
   links: env_service + bme280 + mock_bsp (bsp_i2c_bus) + Unity
   loads a register image, calls the real service + driver, asserts exact units.
```

**Mode 2 - Bench, through the tower, real sensor on a Nucleo**:

```c
/* bench/bench_env.c */
#include "env_service.h"
#include "bsp.h"
#include <stdio.h>

void app_main(void) {
    bsp_init();
    env_service_init(bsp_i2c_bus());
    while (1) {
        float t, p, h;
        if (env_read_all(&t, &p, &h) == ROV_OK)
            printf("BME280  t=%.2fC  p=%.1fhPa  h=%.1f%%\r\n", t, p, h);
        else
            printf("BME280 read failed\r\n");
        delay_ms(500);
    }
}
```

Built with the existing helper (no new machinery):

```cmake
add_f411_node_executable(f411_bench_env.elf bench/bench_env.c)
```

Each bench is a separate `.elf` containing only `app -> env_service -> bme280 ->
bus -> HAL`. It compiles no node `app.c` and no other sensor. The linker drops
unreferenced objects because the libraries are static.

Why a real sensor still matters: a host test with fake bytes proves the **math**;
a bench with a real chip proves the **wiring and the chip**. Neither substitutes
for the other.

## Non-goals (deliberately excluded, YAGNI)

- No `sensors/` wrapper tree; services are the facade.
- No `transports/` tree; each transport lives in its only home.
- No host-PC / USB-to-I2C / Pi bench transport. Nucleo dev board only.
- No `_template/` folder, no per-driver CMake targets, no `bench_sensors` table.
  One bench source per sensor; one `add_<board>_node_executable` line each.
- Keep `drivers/include` + `drivers/src` flat.
- SPI drivers (`bmi270`, `lsm6dsoxtr`), PMBus, `tcan1044`, and DShot are out of
  scope for the transport migration and left as-is.

## Migration order

1. Add `shared/include/rov_i2c.h` (bus + inline `mem_*` helpers).
2. Add `bsp_i2c_bus()` to `tests/mocks/mock_bsp.c` over the existing register
   file, with fault flags; add the missing raw `bsp_i2c_read` register-pointer
   behaviour.
3. Convert BME280 to the injected bus (`bme280_init(dev, bus)`), update its
   callers and `tests/test_driver_bme280.c`. BME280 is the pilot because it is
   already register-addressed and already mocked at the bus level, so nothing
   in SIL moves.
4. Add `bsp_i2c_bus()` to the node BSPs and `boards/*/platform` so target builds
   link.
5. Convert the remaining register-addressed I2C drivers: INA226, INA237,
   TMP1075.
6. Convert MS5837 (command-addressed): remove its `mock_sensors_get_ms5837` weak
   hook and teach the mock bus the raw transaction, including the SIL ripple.
7. Add services as facades to the tower (own the driver, delete duplicate
   caches).
8. Move device functions out of `bsp.h` into drivers; make services wrap them.
9. Fail-loud board macros (`#error` on a missing required macro).
10. Single safety-state source.
11. Delete `x19_*` forwarders.
12. Add `bench/bench_<sensor>.c` per sensor plus the CMake target; add the
    `rov bench -n <sensor>` CLI command.
13. Delete the `tests/mocks/mock_<sensor>.c` driver-level mocks.

## Testing

- Per-service host test: register image from the datasheet, real service +
  driver, assert exact engineering units.
- Fault injection via the bus mock (NACK / timeout / short read / corrupt byte):
  assert the driver/service returns an error and leaves prior output untouched.
- Bench: each per-sensor tower flashed to a Nucleo, readings checked over serial.
- Compile-error check: a build missing a required board macro must fail.
- CI: host unit tests remain blocking; the target acceptance gate stays as-is.

## Risks and open questions

- `lsm6dsoxtr_read_raw()` is currently a pure mock passthrough and ignores
  hardware; step 3 does not cover it (SPI, out of scope). Removing its `weak
  mock_*` hook is separate driver hygiene.
- Fail-loud will break the F411/G474/Nucleo bench builds until their board macros
  are supplied. That is the intent, but it makes board bring-up a prerequisite.
- Steps 8-11 are boundary cleanups independent of the bus; they can land before or
  after the bus work, but `app.c` must not be touched by the bus change and the
  boundary change at the same time.
