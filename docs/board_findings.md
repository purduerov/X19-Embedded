# Hardware Reality Check — 2026-09-28

Audit of the three KiCad boards in `KiCad/Boards/` against the pin assignments
the firmware assumes. This document exists because the answer is not what the
issue tracker implies, and because getting it wrong produces firmware that
compiles, passes its tests, and drives nothing.

## Summary

**No board in this project has a layout.** All three `.kicad_pcb` files are
empty stubs with zero footprints placed. Every board is schematic-only.

The pin assignments that firmware and the `target_bsp_*` acceptance tests rely
on do not match any board. Three mutually contradictory sources exist:

| Source | MCU | Says leak probes are | Says emergency cutoff is |
|---|---|---|---|
| `docs/pinout_mappings.md` (was "authoritative") | STM32G431CB | PA4, PA5 | PA6 |
| `tests/hardware/fakes/main.h` (drives the tests) | unspecified | PA4, PA5 | **PC10** |
| `X19-Pi-Shield-Board.kicad_sch` (ground truth) | STM32C542CCT6 | **no such net** | **no such net** |

The schematics are the only ground truth, and they say the functions are not
present.

## The MCU

All three boards use an **STM32C542CCT6** (Cortex-M33, 144 MHz, LQFP-48).

Relevant to the fake header: this part has **no PC0–PC12 pins** and **no
PF0/PF1**. `PC13` is its only PC pin. Therefore `GPIOC PIN_10`, which the
`target_bsp_node1` and `target_bsp_node2` contracts require, refers to a pin
that cannot be bonded out. No amount of firmware work can satisfy it on real
silicon.

## Per-board findings

### X19-Pi-Shield-Board (Node 1)

- MCU `IC1`, STM32C542CCT6, SamacSys symbol, `QFP50P900X900X160-48N` footprint.
- **No leak-probe net exists.** PA4 and PA5 are unconnected.
- **No emergency-cutoff net exists.**
- I2C is on **PA8 / PA12**, which are not I2C1 pins on this part. (The doc
  claims BME280 I2C is on PB0/PB1.)
- CAN is not FDCAN here; it is an external MCP2518 over UART on PB12/PB13,
  plus TCAN1044 transceivers.
- `SWCLK` is wired to `VSS_2` (pin 35), a ground pin. `PA14`, the real SWCLK
  pin, has no wire.
- `GPIO_ALERT` is wired to `VREF+` (pin 9).
- `.kicad_pcb` holds 6 footprints: 3 mounting holes, 2 Molex, 1 Pi header. No
  MCU placed.

### X19-Control-Board (Node 2)

- MCU `U1`, STM32C542CCT6, `board_control_manual_lib:STM32C542CCT6`.
- **No `SOL_0..SOL_9` nets.** The ten solenoid low-side N-MOSFETs `Q1..Q10` have
  gates netted **`GATE1`..`GATE10`**, on PA0-PA3, PA7, PB0-PB2, PB10, and VCAP.
  Zero overlap with the fake header's `PB0..PB9`.
- **`GATE10` lands on `VCAP` (pin 22).** `VCAP` is the internal LDO output that
  requires a ~4.7 uF decoupling capacitor to ground. It is instead wired to a
  solenoid gate, and has no decoupling capacitor anywhere. Board-damage class.
- **`PWM_B1` lands on `PH2-BOOT0` (pin 44)**, a boot strap pin, with no strap
  resistor.
- The board's own annotation reads "PWM (Bitbang GPIO)" and "why do gpio
  bitbang for PWM? Cant we just use the STM's timer channels?" So the hardware
  was deliberately drawn for bit-banged PWM, which contradicts the HAL
  `TIM1`/`TIM8` BSP the contract tests require.
- MCU CAN TX/RX are **crossed** relative to the TCAN1044 transceiver. The
  transceiver `STB` pin is floating; the schematic's own note reads
  "<-Tie to Ground".
- `VSS` pin 23 is completely unconnected. `VREF+` / `VREF-` unconnected.
- **No status or heartbeat LED on any GPIO.** The three LEDs are 12 V / 5.2 V /
  3.3 V rail indicators.
- `.kicad_pcb` has zero footprints, duplicate UUIDs, and zero-value UUIDs, so
  it will likely not open cleanly.

### X19-Power-Slab-Board (Node 3)

- MCU `U5`, STM32C542CCT6, DigiKey snapshot symbol.
- **The only board with clean, correct signal assignments:**
  - **I2C1 on PB6 (SCL) / PB7 (SDA)** — correct for this part.
  - **FDCAN1 on PA11 (RX) / PA12 (TX)** — correct for this part.
  - SWD on PA13 / PA14 — correct.
- **No brick-enable net exists.** The four Murata E48SC12030 converters have
  their `ON/OFF` pin hard-tied to `VIN(-)`. As drawn, all four 12 V converters
  are permanently held off and are not under MCU control.
- **No LM74700 exists in the design.** The ideal-diode part referenced by the
  node 3 BSP contract is absent; the closest part is a PKU5511 current-sense
  amplifier.
- `.kicad_pcb` has zero footprints.

## Consequences for the BSP

The `target_bsp_node1/2/3` contract tests are useful and worth keeping. They
specify real behaviour precisely: active-LOW leak probes with an
invalid-index fail-safe, deasserted-at-boot emergency cutoff, PWM neutral at
1500 us, brake trip clearing all PWM and solenoids, and a `can_init()` call from
every `bsp_init()`.

What they validate is **BSP logic**, because `tests/hardware/fakes/main.h`
supplies the pins. Making them pass means writing code that calls the HAL in
the right order. It cannot mean the code drives real hardware, because the pins
it drives do not exist on the boards.

This is the recurring failure pattern in this repo: correct-looking code, green
tests, absent behaviour. Making these contracts green would hide the real
blocker rather than fix it.

## What is actually unblocked

Very little firmware work depends on pin assignment:

- **Calling `can_init()` from every `bsp_init()`.** All three contracts require
  it and nodes 1 and 3 do not call it. FDCAN1 is correctly routed on the Power
  Slab, so this is real.
- **Startup ordering** — `HAL_Init` → `SystemClock_Config` → `MX_GPIO_Init` →
  `MX_FDCAN1_Init` → `MX_I2C1_Init` → `app_main`, plus `MX_TIM1_Init` /
  `MX_TIM8_Init` for node 2. This is pin-independent and already enforced by
  `check_node2_startup.cmake` and `test_target_startup.c`.

Everything else is downstream of hardware that does not yet exist.

## Real critical path

Ordered by what unblocks the most downstream work:

1. Decide the solenoid strategy on the Control Board. It is drawn for
   bit-banged GPIO, and `GATE10` on `VCAP` will damage the part. Fix this
   before layout.
2. Route the Power Slab bricks' `ON/OFF` to the MCU. They are hard-tied off.
3. Fix `GATE10` off `VCAP` and `PWM_B1` off `PH2-BOOT0` on the Control Board.
4. Uncross Control Board CAN; tie the transceiver `STB` low; land `VSS` pin 23
   and the `VREF` pins.
5. Decide whether the Pi Shield gets leak probes and an emergency cutoff at
   all, and route them. Fix `SWCLK` off `VSS_2` and `GPIO_ALERT` off `VREF+`.
6. Produce layouts for all three boards.
7. Only then: generate CubeMX projects from real netlists, replace
   `fakes/main.h` with genuine generated headers, and re-point the contracts at
   them.
8. Only then: bench-verify with a scope.

## How to keep the contracts honest

- `tests/hardware/fakes/main.h` now carries a header comment stating that its
  pins are guesses and that a green contract is a statement about logic, never
  about silicon.
- `docs/pinout_mappings.md` is marked SUPERSEDED with the specific errors
  listed.
- The `target_*` tests keep `continue-on-error: true`. A green suite is not
  evidence about hardware, and the `target_*` split exists to keep that
  distinction visible.

## Method and confidence

Netlists were derived by hand from the KiCad S-expression text, since no
`.net` or netlist export exists anywhere in the workspace. Connectivity is by
`(global_label ...)`; both designs are hierarchical with no sheet pins, so
sheet structure does not affect net names.

The Control Board derivation was validated by reproducing datasheet-correct pin
numbers for NRST (7), VREF-/VREF+ (8/9), VDD/VSS (24/23, 36/35, 48/47),
PA13/PA14 (34/37), and PH0/PH1 (5/6). Confidence is high on pin numbers for
that board, and high on the structural findings for all three: the missing
nets, the hard-tied brick `ON/OFF`, the absent LM74700, and the empty `.kicad_pcb`
files are absences and hard straps, not inferences from ambiguous geometry.

Residual uncertainty is in the two conflicting net lists, where a label may
have been missed. That does not affect any conclusion above, because each rests
on either an absence or an unambiguous strap.
