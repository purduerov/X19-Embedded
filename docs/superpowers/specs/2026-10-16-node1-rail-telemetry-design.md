# Node 1 Rail Telemetry: INA237 Through the Power Service onto CAN 0x210

- Date: 2026-10-16
- Owner: Purdue ROV Embedded Team
- Status: Draft for review
- Parent: `2026-10-15-injected-i2c-bus-driver-architecture-design.md` (migration steps 7-8)

## Goal

Node 1's INA237 5V rail monitor reads a sensor and throws the result away.
`nodes/node1_pi_shield/Core/Src/app.c` owns `g_ina237_dev`, calls
`ina237_read_power()`, stores `g_ina237_valid`, and never publishes or reads it.
`rov_env_telemetry_t` has no rail field.

Make node 1 publish its 5V rail voltage and current, with the INA237 owned by a
service and the values carried on the existing 0x210 environment frame.

Outcomes:

1. `app.c` no longer includes a driver header or names the bus; the INA237 lives
   in `power_service`.
2. 0x210 grows from 13 to 17 bytes and carries `rail_voltage_mv` +
   `rail_current_ma`.
3. Every consumer of 0x210 (C and Python) is updated in the same change.

## Current state (why)

- `nodes/node1_pi_shield/Core/Src/app.c:26,182,215-216` owns
  `ina237_dev_t g_ina237_dev`, calls
  `ina237_init(&g_ina237_dev, bsp_i2c_bus(), 0x40, 0.001f)` and
  `ina237_read_power()`; `g_ina237_valid` is written and never read.
- `services/src/power_service.c` is a pure pass-through to node3-flavoured
  `bsp_*` device functions; it owns no driver.
- `shared/include/rov_types.h:130-135`: `rov_env_telemetry_t` is
  `{float, float, float, uint8_t}` = 13 bytes, `memcpy`'d straight to the wire.
- The 0x210 layout is mirrored in C (`tests/test_can_protocol.c`,
  `tests/test_node1_pi_shield.c`, `tests/sil_bridge_server.c`) and Python
  (`tests/sil_companion_bridge/sil_protocol.py`, `pi_core_sil_bridge.py`,
  `tests/sil_dashboard/dashboard_app.py`, `sil_dashboard_client.py`,
  `tests/sil_stimulus/test_stimulus_flows.py`, `test_dashboard_app.py`,
  `tools/can_sniffer.py`, `tools/can_stimulus.py`).

## Design

### 1. Service layer (`power_service`)

Extend `power_service` to own the rail INA237, mirroring `env_service`'s pattern
(a service acquires its own bus and owns its device):

```c
rov_status_t power_service_init(void);                                /* acquire bus, init INA237 */
rov_status_t power_read_rail(float *bus_voltage_v, float *current_a); /* engineering units */
```

- `power_service_init()` does `memset(&g_rail_dev, 0, ...)`, then
  `ina237_init(&g_rail_dev, bsp_i2c_bus(), POWER_RAIL_INA237_ADDR,
  POWER_RAIL_SHUNT_OHMS)`, and records success in a `static bool`.
- `power_read_rail()` lazy-inits on first use (like `env_read_all`), calls
  `ina237_read_power()`, and copies `bus_voltage_v` / `shunt_current_a` into the
  caller's pointers (NULL-tolerant). On error it returns the status and leaves
  the outputs untouched.
- Rail config is owned by the service: `POWER_RAIL_INA237_ADDR` = `0x40u`,
  `POWER_RAIL_SHUNT_OHMS` = `0.001f`, documented as node 1's Pi 5V rail.
  Generalizing per board is deferred to the node3 sub-project.
- The existing node3 pass-through functions stay untouched; the node3
  sub-project reworks them.

### 2. Node 1 app

- Drop `#include "ina237.h"`, `static ina237_dev_t g_ina237_dev`, and
  `static bool g_ina237_valid`.
- Add `#include "power_service.h"`.
- `node1_app_init()`: replace `ina237_init(...)` with `power_service_init()`.
- 10 Hz loop:

```c
float rail_v = 0.0f;
float rail_i = 0.0f;
bool rail_valid = (power_read_rail(&rail_v, &rail_i) == ROV_OK) &&
                  isfinite(rail_v) && isfinite(rail_i);
g_env_telemetry.rail_voltage_mv = rail_valid ? (uint16_t)(rail_v * 1000.0f) : 0u;
g_env_telemetry.rail_current_ma = rail_valid ? (uint16_t)(rail_i * 1000.0f) : 0u;
```

### 3. Wire format (0x210, 13 -> 17 bytes)

Append to `rov_env_telemetry_t` (existing offsets preserved; struct is packed):

```c
uint16_t rail_voltage_mv; /**< Node 1 Pi 5V rail voltage in mV (INA237); 0 when invalid */
uint16_t rail_current_ma; /**< Node 1 Pi 5V rail current in mA (INA237); 0 when invalid */
```

`rov_can_pack_env_telemetry` / `rov_can_unpack_env_telemetry` need no change
(they use `sizeof`). `tests/sil_bridge_server.c:440` reads the leak byte at
offset 12, which is unchanged; update its `13` literal to
`sizeof(rov_env_telemetry_t)` for clarity.

### 4. Mirrors to update

- C: `shared/include/rov_types.h` (struct + `13 bytes` doc), 
  `tests/test_can_protocol.c` (17-byte roundtrip incl. rail fields),
  `tests/test_node1_pi_shield.c` (rail assertions), `tests/test_services.c`
  (rail test).
- Python: `tests/sil_companion_bridge/sil_protocol.py` (`<3fB` -> `<3fBHH`,
  `EnvTelemetry` fields + 17-byte minimum),
  `tests/sil_stimulus/test_stimulus_flows.py` (`env_payload()` builder,
  `SIL_ENV_PAYLOAD_LEN`), `tests/sil_stimulus/test_dashboard_app.py`
  (`ENV_PAYLOAD`), `tools/can_sniffer.py` (`_STRUCT_ENV`),
  `tools/can_stimulus.py` (decode / validation).
- Decode-only sites (`pi_core_sil_bridge.py`, `sil_dashboard_client.py`,
  `dashboard_app.py`) need no structural change; optionally surface the rail.
- Docs: `System_Software/Docs/x19_architecture/x19_firmware_tasks_and_deadlines.md`
  (`13 bytes` -> 17), outside this repo.

### 5. SIL note

The host SIL shares one mock I2C bus across nodes, and
`tests/mocks/mock_physics.c:363` drives address 0x40 as node 3's 12V brick.
Node 1's INA237 is also 0x40, so in SIL node 1's rail decodes to the plant's
brick voltage/current, not a 5V rail. That is a mock-fidelity artifact, not a
firmware defect. The SIL stimulus test therefore asserts the frame is 17 bytes
and that the rail fields decode; exact rail values are asserted only in the unit
tests, where `mock_ina237_set_reading()` places a known reading.

## Testing

- `tests/test_services.c`: `power_service_init()` + `power_read_rail()` against
  `mock_ina237_set_reading(0x40, 5.2, 1.5, 0.001)`; assert exact units within one
  LSB. Fault case: `mock_bsp_i2c_set_fault(ROV_TIMEOUT)` -> error, outputs
  untouched.
- `tests/test_node1_pi_shield.c`: nominal test asserts `env.rail_voltage_mv`
  within one LSB of the 5.21 V placed at 0x40 and `env.rail_current_ma` == 1350;
  a case with no INA237 reading placed asserts both fields are 0.
- `tests/test_can_protocol.c`: 17-byte roundtrip including the rail fields.
- SIL Python suites updated to the 17-byte payload.

## Non-goals

- Node 3 power facade (services own drivers, device functions leave `bsp.h`,
  `pmbus_brick` on the injected bus) - the next sub-project.
- A new CAN ID; a per-board rail config mechanism; the fail-loud board-macro
  step.

## Risks and open questions

- Appending to 0x210 is a wire change: an old 13-byte frame no longer unpacks
  (new minimum 17). The fleet is reflashed together; Python readers that slice
  `[:13]` keep working on a 17-byte frame.
- Rail config lives in `power_service.c` (node 1's 0x40 / 0.001). When node 3's
  logic rail lands this becomes a per-board parameter.
- The `mock_physics.c` address collision (0x40) noted above.

## Migration order

1. Struct + service (`rov_types.h`, `power_service.h/.c`).
2. Node 1 app routes through the service.
3. C tests (`test_services`, `test_node1_pi_shield`, `test_can_protocol`).
4. Python mirrors (protocol, stimulus, dashboard, tools).
5. Verify: native `ctest`, `arm-c542-debug` build, `clang-format`, SIL Python
   suites.
