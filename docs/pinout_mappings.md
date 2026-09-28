# X19 Hardware Pinout & GPIO Allocation — SUPERSEDED, DO NOT USE FOR FIRMWARE

> ## STOP: this document is wrong and was never validated against hardware
>
> **It must not be used to write a BSP, a CubeMX configuration, or a pin
> definition.** It previously claimed to be the "Authoritative Silicon Mapping".
> It is not. It was checked against the KiCad schematics on 2026-09-28 and does
> not describe any board in this project.
>
> ### It documents the wrong microcontroller
>
> Every table below is written for an **STM32G431CB**. All three boards
> (`X19-Pi-Shield-Board`, `X19-Control-Board`, `X19-Power-Slab-Board`) use an
> **STM32C542CCT6**, which has a different pinout and a different peripheral set.
>
> The two parts are not interchangeable. The STM32C542CCT6 LQFP-48 **has no
> PC0-PC12 pins at all** (`PC13` is its only PC pin) and **no PF0/PF1**. This
> document references `PC6`, `PC7`, `PC8`, `PC9`, `PC0`, `PF0` and `PF1` as
> though they were available. They are not.
>
> ### Its pin numbers are internally inconsistent
>
> It places `PC14-OSC32_IN` at pin 8 and `PC15-OSC32_OUT` at pin 9, which is the
> STM32G431 numbering. On the STM32C542, pin 8 is `VREF-` and pin 9 is `VREF+`.
> It also lists `PB8-BOOT0` at pin 44 and `PB9` at pin 45, while the STM32C542
> has `PH2-BOOT0` at pin 44 and `PB9` at pin 46. The tables are a mixture of at
> least two different parts' numbering.
>
> ### Its net names do not exist on the boards
>
> | This document claims | Actual schematic |
> |---|---|
> | `LEAK_TRACE_FLOOR1` = PA4, `LEAK_TRACE_FLOOR2` = PA5 (Node 1) | No leak-probe net exists on the Pi Shield. PA4 and PA5 are unconnected. |
> | `EMERGENCY_CUTOFF_OUT` = PA6 (Node 1) | No emergency-cutoff net exists on any board. On the Control Board, PA6 is `MISO`. |
> | `SDA_BME280` / `SCL_BME280` on PB0/PB1 (Node 1) | Pi Shield I2C is on **PA8 / PA12**, which are not I2C1 pins on this part. |
> | `SOL_VALVE_1A..5B` on PB12-PB15, PC13-PC15, PF0/PF1, PC0 (Node 2) | The board has ten MOSFET gate nets named **`GATE1`..`GATE10`**, on PA0-PA3, PA7, PB0-PB2, PB10 and **VCAP**. Zero overlap. |
> | `SAFETY_BREAK_IN` = PB0 as `TIM1_BKIN` (Node 2) | No such net. PB0 drives `GATE6`, a solenoid coil. |
>
> ### It contradicts the firmware acceptance tests
>
> The `target_bsp` contract tests validate against
> `tests/hardware/fakes/main.h`, a hand-written fake header, which specifies a
> **third** set of answers again (`EMERGENCY_CUTOFF` on `GPIOC PIN_10`,
> `LEAK_PROBE0/1` on `PA4/PA5`, `SOL_0..SOL_9` on `PB0..PB9`).
> `GPIOC PIN_10` cannot exist on an STM32C542CCT6.
>
> ### The boards are schematic-only
>
> No board in `KiCad/Boards/` has a layout. All three `.kicad_pcb` files are
> empty stubs with zero footprints placed, so no netlist-versus-firmware check
> has ever been possible. The Control Board file additionally has duplicate and
> zero-value UUIDs and will likely not open cleanly.
>
> ### Known hardware defects found while checking
>
> These are in the schematics, not in this document, and are board-damage or
> non-functional class:
>
> - **Control Board: `GATE10` lands on `VCAP` (pin 22).** `VCAP` is the internal
>   LDO output that requires a ~4.7 uF decoupling capacitor to ground. It is
>   instead wired to a solenoid MOSFET gate, and has no decoupling capacitor.
> - **Control Board: `PWM_B1` lands on `PH2-BOOT0` (pin 44)**, a boot strap pin,
>   with no strap resistor.
> - **Control Board: MCU CAN TX/RX are crossed** relative to the TCAN1044
>   transceiver, and the transceiver `STB` pin is left floating (the schematic's
>   own note reads "<-Tie to Ground").
> - **Control Board: `VSS` pin 23 is completely unconnected**, and `VREF+` /
>   `VREF-` are unconnected.
> - **Power Slab: the four Murata E48SC12030 bricks have their `ON/OFF` pin
>   hard-tied to `VIN(-)`.** They are not routed to the MCU, so as drawn all four
>   12 V converters are permanently held off. There is no brick-enable net.
> - **Power Slab: no LM74700 exists in the design.** Issue #86's diode is not on
>   the board; the closest part is a PKU5511 current-sense amplifier.
> - **Pi Shield: `SWCLK` is wired to `VSS_2` (pin 35)**, a ground pin, and
>   `GPIO_ALERT` is wired to `VREF+` (pin 9). `PA14`, the real SWCLK pin, has no
>   wire.
> - **Control Board: no status or heartbeat LED is connected to any GPIO.** The
>   three LEDs on the board are 12 V / 5.2 V / 3.3 V rail indicators.
>
> ### What is actually correct
>
> Two mappings on the Power Slab are clean and match the STM32C542 pinout:
> **I2C1 on PB6 (SCL) / PB7 (SDA)**, and **FDCAN1 on PA11 (RX) / PA12 (TX)**,
> with SWD on PA13/PA14. Those are the only signal assignments found that agree
> with both the schematic and the silicon.
>
> ---
>
> ## Original (superseded) content follows, retained for history only
>
> The tables below are preserved verbatim so the earlier mistakes can be traced.
> Treat every row as **incorrect**.

---

## 1. Pinout Overview (LQFP48)

All nodes share the exact same core pins for power, reset, crystal oscillator, and debug interfaces:

| Pin # | Pin Name | Core Function | Description |
| :--- | :--- | :--- | :--- |
| **7** | `NRST` | System Reset | Active-low hardware reset with 100nF decoupling cap. |
| **8** | `PC14-OSC32_IN` | HSE / OSC | Optional 24 MHz HSE crystal oscillator input. |
| **9** | `PC15-OSC32_OUT`| HSE / OSC | Optional 24 MHz HSE crystal oscillator output. |
| **23** | `VSS` | Digital Ground | Star-ground plane connection. |
| **24** | `VDD` | Digital 3.3V Power | 3.3V power rail with local 100nF ceramic bypass capacitor. |
| **34** | `PA13` | `SWDIO` | Serial Wire Debug Data (SWD header). |
| **37** | `PA14` | `SWCLK` | Serial Wire Debug Clock (SWD header). |
| **44** | `PB8-BOOT0` | `FDCAN1_RX` / `BOOT0` | Shared CAN FD Receive / Hardware Bootloader pin. |
| **45** | `PB9` | `FDCAN1_TX` | CAN FD Transmit to TI TCAN1044 transceiver. |
| **47** | `VSS` | Digital Ground | Star-ground plane connection. |
| **48** | `VDD` | Digital 3.3V Power | 3.3V power rail with local 100nF ceramic bypass capacitor. |

---

## 2. Node 1: Pi Shield Pinout (`STM32G431CB`)

Mounted directly to the Raspberry Pi 5 40-pin GPIO header.

| Pin # | Pin Name | Peripheral | Net Name / Target | Function |
| :--- | :--- | :--- | :--- | :--- |
| **19** | `PB0` | `I2C1_SDA` | `SDA_BME280` | Bosch BME280 Vacuum / Leak I2C Data. |
| **20** | `PB1` | `I2C1_SCL` | `SCL_BME280` | Bosch BME280 Vacuum / Leak I2C Clock. |
| **29** | `PB10` | `I2C2_SCL` | `SCL_INA5V` | TI INA226 5V Power Monitor I2C Clock. |
| **30** | `PB11` | `I2C2_SDA` | `SDA_INA5V` | TI INA226 5V Power Monitor I2C Data. |
| **14** | `PA4` | `GPIO_Input` | `LEAK_TRACE_FLOOR1` | Blue Robotics internal floor leak trace 1. |
| **15** | `PA5` | `GPIO_Input` | `LEAK_TRACE_FLOOR2` | Blue Robotics internal floor leak trace 2. |
| **16** | `PA6` | `GPIO_Output` | `EMERGENCY_CUTOFF_OUT` | Hardware emergency break output signal. |
| **44** | `PB8` | `FDCAN1_RX` | `CAN_RX` | TCAN1044 RX line (`can0`). |
| **45** | `PB9` | `FDCAN1_TX` | `CAN_TX` | TCAN1044 TX line (`can0`). |

---

## 3. Node 2: Control Board Pinout (`STM32G431CB`)

Central propulsion, tool actuation, and navigation sensor board.

| Pin # | Pin Name | Peripheral | Net Name / Target | Function |
| :--- | :--- | :--- | :--- | :--- |
| **10** | `PA0` | `TIM1_CH1` | `PWM_THRUSTER_1` | Blue Robotics BESC30-R3 ESC 1 PWM. |
| **11** | `PA1` | `TIM1_CH2` | `PWM_THRUSTER_2` | Blue Robotics BESC30-R3 ESC 2 PWM. |
| **12** | `PA2` | `TIM1_CH3` | `PWM_THRUSTER_3` | Blue Robotics BESC30-R3 ESC 3 PWM. |
| **13** | `PA3` | `TIM1_CH4` | `PWM_THRUSTER_4` | Blue Robotics BESC30-R3 ESC 4 PWM. |
| **38** | `PC6` | `TIM8_CH1` | `PWM_THRUSTER_5` | Blue Robotics BESC30-R3 ESC 5 PWM. |
| **39** | `PC7` | `TIM8_CH2` | `PWM_THRUSTER_6` | Blue Robotics BESC30-R3 ESC 6 PWM. |
| **40** | `PC8` | `TIM8_CH3` | `PWM_THRUSTER_7` | Blue Robotics BESC30-R3 ESC 7 PWM. |
| **41** | `PC9` | `TIM8_CH4` | `PWM_THRUSTER_8` | Blue Robotics BESC30-R3 ESC 8 PWM. |
| **14** | `PA4` | `GPIO_Output` | `SPI1_CS_IMU` | ST LSM6DSOXTR IMU Chip Select. |
| **15** | `PA5` | `SPI1_SCK` | `SPI1_SCK_IMU` | ST LSM6DSOXTR IMU SPI Clock. |
| **16** | `PA6` | `SPI1_MISO`| `SPI1_MISO_IMU` | ST LSM6DSOXTR IMU SPI Data In. |
| **17** | `PA7` | `SPI1_MOSI`| `SPI1_MOSI_IMU` | ST LSM6DSOXTR IMU SPI Data Out. |
| **42** | `PA9` | `I2C1_SCL` | `SCL_DEPTH_12V` | TE MS5837 Depth & 12V Power Monitor I2C Clock. |
| **43** | `PA10`| `I2C1_SDA` | `SDA_DEPTH_12V` | TE MS5837 Depth & 12V Power Monitor I2C Data. |
| **25** | `PB12`| `GPIO_Output` | `SOL_VALVE_1A` | SMC SY3400-6U1-NA Valve 1 Solenoid A (12V). |
| **26** | `PB13`| `GPIO_Output` | `SOL_VALVE_1B` | SMC SY3400-6U1-NA Valve 1 Solenoid B (12V). |
| **27** | `PB14`| `GPIO_Output` | `SOL_VALVE_2A` | SMC SY3400-6U1-NA Valve 2 Solenoid A (12V). |
| **28** | `PB15`| `GPIO_Output` | `SOL_VALVE_2B` | SMC SY3400-6U1-NA Valve 2 Solenoid B (12V). |
| **1**  | `PC13`| `GPIO_Output` | `SOL_VALVE_3A` | SMC SY3400-6U1-NA Valve 3 Solenoid A (12V). |
| **2**  | `PC14`| `GPIO_Output` | `SOL_VALVE_3B` | SMC SY3400-6U1-NA Valve 3 Solenoid B (12V). |
| **3**  | `PC15`| `GPIO_Output` | `SOL_VALVE_4A` | SMC SY3400-6U1-NA Valve 4 Solenoid A (12V). |
| **4**  | `PF0` | `GPIO_Output` | `SOL_VALVE_4B` | SMC SY3400-6U1-NA Valve 4 Solenoid B (12V). |
| **5**  | `PF1` | `GPIO_Output` | `SOL_VALVE_5A` | SMC SY3400-6U1-NA Valve 5 Solenoid A (12V). |
| **6**  | `PC0` | `GPIO_Output` | `SOL_VALVE_5B` | SMC SY3400-6U1-NA Valve 5 Solenoid B (12V). |
| **18** | `PB0` | `TIM1_BKIN` | `SAFETY_BREAK_IN` | Hardware Safety Break (Emergency Cutoff). |
| **44** | `PB8` | `FDCAN1_RX` | `CAN_RX` | TCAN1044 RX line. |
| **45** | `PB9` | `FDCAN1_TX` | `CAN_TX` | TCAN1044 TX line. |

---

## 4. Node 3: Power Slab Pinout (`STM32G431CB`)

Central power regulation, 4x 12V 300W bricks, and PMBus telemetry.

| Pin # | Pin Name | Peripheral | Net Name / Target | Function |
| :--- | :--- | :--- | :--- | :--- |
| **42** | `PA9` | `I2C1_SCL` | `PMBUS_SCL` | PMBus Clock to 5 converter bricks. |
| **43** | `PA10`| `I2C1_SDA` | `PMBUS_SDA` | PMBus Data to 5 converter bricks. |
| **10** | `PA0` | `ADC1_IN1` | `TEMP_SENSE_COPPER1` | PCB Copper Temperature Sensor 1 (NTC). |
| **11** | `PA1` | `ADC1_IN2` | `TEMP_SENSE_COPPER2` | PCB Copper Temperature Sensor 2 (NTC). |
| **14** | `PA4` | `GPIO_Input` | `LM74700_FAULT` | TI LM74700-Q1 Ideal Diode Fault Status. |
| **44** | `PB8` | `FDCAN1_RX` | `CAN_RX` | TCAN1044 RX line. |
| **45** | `PB9` | `FDCAN1_TX` | `CAN_TX` | TCAN1044 TX line. |

---

## 5. Node 4: USB Camera Hub Pinout (`STM32G431CB`)

PCIe USB 3.0 Host Controller & 8x exploreHD Camera Hub.

| Pin # | Pin Name | Peripheral | Net Name / Target | Function |
| :--- | :--- | :--- | :--- | :--- |
| **42** | `PA9` | `I2C1_SCL` | `VBUS_MON_SCL` | I2C Power Monitor Clock for camera VBUS. |
| **43** | `PA10`| `I2C1_SDA` | `VBUS_MON_SDA` | I2C Power Monitor Data for camera VBUS. |
| **10-17**| `PA0-PA7`| `GPIO_Output`| `CAM_PWR_EN_1..8` | Per-port 5.2V VBUS MOSFET gate controls. |
| **44** | `PB8` | `FDCAN1_RX` | `CAN_RX` | TCAN1044 RX line. |
| **45** | `PB9` | `FDCAN1_TX` | `CAN_TX` | TCAN1044 TX line. |
