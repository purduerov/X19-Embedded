# X19 STM32G431CB (48-Pin LQFP48) Hardware Pinout & GPIO Allocation Master

> **Authoritative Silicon Mapping for Purdue ROV 2026-2027**  
> *Standardized across 100% of subsea microcontroller nodes.*

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
