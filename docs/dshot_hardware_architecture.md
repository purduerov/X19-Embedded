# STM32G4 DShot Hardware & DMA Driver Architecture

This document specifies the hardware architecture, register configuration, and timing parameters for driving Blue Robotics T200 thrusters via Bluejay ESCs on STM32G4 microcontrollers (NUCLEO-G474RE and production vehicle targets).

All configurations and parameters are grounded directly in the STM32G4 Reference Manual (RM0440), STM32 HAL Drivers, and BLHeli_S / Bluejay / Betaflight flight controller specifications.

---

## 1. Single-Channel TIM1_CH1 PWM DMA Streaming

### DMAMUX & DMA Request Mapping
- **Timer Channel**: TIM1 Channel 1 on `PA8` (`GPIO_AF6_TIM1`).
- **DMAMUX Request**: `DMA_REQUEST_TIM1_CH1` (Request ID `42U`).
  - *Note*: Single-channel `HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, ...)` binds internally to `TIM_DMA_ID_CC1` and requires `DMA_REQUEST_TIM1_CH1`. Counter overflow (`DMA_REQUEST_TIM1_UP`) is used for multi-channel DMA burst writes.
- **DMA Instance**: `DMA1_Channel1`.
- **Direction**: `DMA_MEMORY_TO_PERIPH`.
- **Alignment**:
  - `PeriphDataAlignment`: `DMA_PDATAALIGN_HALFWORD` (16-bit write to `TIM1->CCR1`).
  - `MemDataAlignment`: `DMA_MDATAALIGN_HALFWORD` (16-bit reads from RAM buffer array).
- **Increments**: `PeriphInc = DMA_PINC_DISABLE`, `MemInc = DMA_MINC_ENABLE`.
- **Mode**: `DMA_NORMAL` (per-frame one-shot burst).

### Timer Clocks & Tick Math @ 170 MHz SYSCLK (`PSC = 0`)

| Parameter | DShot300 (300 kHz) | DShot150 (150 kHz) | Unit |
| :--- | :--- | :--- | :--- |
| Bit Period ($T_{\text{bit}}$) | 3.333 | 6.667 | $\mu\text{s}$ |
| Counter Auto-Reload (`ARR`) | **`566`** (567 ticks) | **`1132`** (1133 ticks) | ticks |
| Bit 0 Pulse Width ($T_{0H}$, 37.5%) | **`213`** (1.253 $\mu\text{s}$) | **`425`** (2.500 $\mu\text{s}$) | ticks |
| Bit 1 Pulse Width ($T_{1H}$, 75.0%) | **`425`** (2.500 $\mu\text{s}$) | **`850`** (5.000 $\mu\text{s}$) | ticks |
| Frame Duration (16 bits) | 53.33 | 106.67 | $\mu\text{s}$ |
| Frame Rate | 1000 | 1000 | Hz |

### Frame Buffer & Zero-Glitch Reset Parking
- **Buffer Length**: 18 halfwords (16 DShot bits + 2 trailing zero reset entries).
- **Trailing Entries**: Entries `[16]` and `[17]` are set to `0`. When the DMA transfers `0` into `CCR1`, the timer output immediately clamps to 0% duty cycle (LOW).
- **Transfer Complete Callback (`HAL_TIM_PWM_PulseFinishedCallback`)**:
  ```c
  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
  TIM1->CCR1 = 0;
  ```
  Calling `HAL_TIM_PWM_Stop_DMA` disables the timer and MOE, holding the output firmly at its configured idle state (`TIM_OCIDLESTATE_RESET` = LOW) throughout the >30 $\mu\text{s}$ inter-frame interval.

---

## 2. Multi-Thruster Synchronization (TIM DMA Burst Mode)

To control 4 thrusters simultaneously on TIM1 (`PA8`..`PA11`) or TIM8 (`PC6`..`PC9`) using a single DMA channel:

### DMA Burst Write API
```c
HAL_TIM_DMABurst_Write(&htim1,
                       TIM_DMABASE_CCR1,
                       TIM_DMA_UPDATE,
                       (uint32_t *)pBurstBuffer,
                       TIM_DMABURSTLENGTH_4TRANSFERS);
```

- **Base Register**: `TIM_DMABASE_CCR1` (starts at `CCR1` offset).
- **Trigger**: `TIM_DMA_UPDATE` (`TIM_DMA_ID_UPDATE`), triggered by counter overflow.
- **DMAMUX Request**: `DMA_REQUEST_TIM1_UP` (Request ID `41U`).
- **Burst Length**: `TIM_DMABURSTLENGTH_4TRANSFERS` (writes 4 consecutive registers: `CCR1`, `CCR2`, `CCR3`, `CCR4` per update event).

### Interleaved Buffer Memory Layout
For 16 data bits + 1 trailing zero bit across 4 channels, allocate 68 entries ($17 \times 4$):
```c
uint16_t burst_buffer[68] = {
    /* Bit 15: Ch1, Ch2, Ch3, Ch4 */
    ch1_b15, ch2_b15, ch3_b15, ch4_b15,
    /* Bit 14: Ch1, Ch2, Ch3, Ch4 */
    ch1_b14, ch2_b14, ch3_b14, ch4_b14,
    /* ... */
    /* Bit 0:  Ch1, Ch2, Ch3, Ch4 */
    ch1_b0,  ch2_b0,  ch3_b0,  ch4_b0,
    /* Trailing reset: 0, 0, 0, 0 */
    0, 0, 0, 0
};
```

---

## 3. 8-Thruster Master-Slave Synchronization (TIM1 + TIM8)

To drive all 8 vehicle thrusters (4 on TIM1, 4 on TIM8) with sub-nanosecond phase alignment:

### TIM1 (Master) Configuration
```c
TIM_MasterConfigTypeDef sMasterConfig = {0};
sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
```

### TIM8 (Slave) Configuration
```c
TIM_SlaveConfigTypeDef sSlaveConfig = {0};
sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
sSlaveConfig.InputTrigger = TIM_TS_ITR0; /* TIM1 TRGO -> TIM8 ITR0 */
HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig);
```

### Execution Order
1. Arm TIM8 (Slave) via DMA Burst Write (`HAL_TIM_DMABurst_Write(&htim8, ...)`). TIM8 waits halted on hardware trigger line `ITR0`.
2. Start TIM1 (Master) via DMA Burst Write (`HAL_TIM_DMABurst_Write(&htim1, ...)`).
3. On the first counter update, `TIM1_TRGO` triggers `TIM8` in hardware simultaneously within 1 clock cycle (5.88 ns @ 170 MHz), eliminating all phase skew across the 8 motors.

---

## 4. Bidirectional DShot Telemetry Reception (Single-Wire)

### Hardware Routing Without External Jumpers
- Set `PA8` GPIO mode to **`GPIO_MODE_AF_OD` with `GPIO_PULLUP`** (Open-Drain).
- Configure **TIM1 Channel 1** for PWM output generation.
- Configure **TIM1 Channel 2** in **Indirect Input Capture Mode** mapped to internal signal `TI1` (PA8).
- Connect `DMA_REQUEST_TIM1_CH2` in `DMA_PERIPH_TO_MEMORY` mode.
- In `HAL_TIM_PWM_PulseFinishedCallback`:
  1. Stop PWM output driver (`HAL_TIM_PWM_Stop_DMA`).
  2. Because PA8 is in Open-Drain mode with Pull-Up, driving `CCR1 = 0` releases the line passively HIGH (~30 $\mu\text{s}$ line idle state).
  3. Arm `HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_2, pTelemBuf, 21)` to capture the 21-bit inverted GCR stream returned by Bluejay.

### 5b/4b GCR Decoding Table
| GCR Symbol (5b Hex) | Decoded Nibble (4b Hex) | GCR Symbol (5b Hex) | Decoded Nibble (4b Hex) |
| :--- | :--- | :--- | :--- |
| `0x19` | `0x0` | `0x1A` | `0x8` |
| `0x1B` | `0x1` | `0x09` | `0x9` |
| `0x12` | `0x2` | `0x0A` | `0xA` |
| `0x13` | `0x3` | `0x0B` | `0xB` |
| `0x1D` | `0x4` | `0x0E` | `0xC` |
| `0x15` | `0x5` | `0x0F` | `0xD` |
| `0x16` | `0x6` | `0x14` | `0xE` |
| `0x17` | `0x7` | `0x18` | `0xF` |

### Blue Robotics T200 eRPM to Mechanical RPM Math
- **T200 Thruster**: 14 magnetic poles = **7 pole pairs**.
- **Raw Value**: $\text{raw} = (\text{exponent} \ll 9) \mid \text{mantissa}$.
- **eRPM Period**: $T_{\text{eRPM}} = (\text{mantissa} \ll \text{exponent}) \quad [\mu\text{s}]$.
- **Electrical RPM**: $\text{eRPM} = \frac{60,000,000}{T_{\text{eRPM}}}$.
- **Mechanical Motor RPM**:
  $$\text{Motor RPM} = \frac{\text{eRPM}}{\text{Pole Pairs}} = \frac{60,000,000}{7 \times T_{\text{eRPM}}} = \mathbf{\frac{8,571,428.57}{T_{\text{eRPM}}}}$$

---

## 5. Hardware Emergency Break Shutdown (`TIM1_BKIN`)

In safety-critical marine operations, hardware break inputs protect the vehicle against runaways:

- **Break Pin**: `PA6` or `PB12` (`TIM1_BKIN`).
- **Hardware Action**: When an active fault signal (water leak sensor, hardware E-stop switch) triggers `BKIN`, the timer hardware **clears `BDTR.MOE` within nanoseconds**, bypassing the CPU entirely.
- **Fail-Safe Invariant**: All PWM outputs (`CH1`..`CH4`) are immediately clamped to the inactive off-state (`TIM_OSSI_ENABLE` / `TIM_OSSR_ENABLE`). The outputs remain safely clamped even if firmware enters a HardFault or locked interrupt.

### Re-Arming Sequence
After the physical fault signal is cleared:
1. `__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);`
2. `__HAL_TIM_MOE_ENABLE(&htim1);`

---

## 6. Non-Blocking Command Injection State Machine

DShot special commands (values `1..47`, e.g. Beep 1..5, Direction Normal/Reverse, 3D Mode, Save Settings):
- **Rule 1**: Must be repeated for **10 consecutive frames**.
- **Rule 2**: The **Telemetry Bit** (bit 4) must be set to **`1`** on all 10 frames.

```
                  ┌──────────────────────────────┐
                  │   STATE_STREAMING_THROTTLE   │
                  │  (Normal 1 kHz PID Loop)     │
                  └──────────────┬───────────────┘
                                 │ Command Requested
                                 ▼
                  ┌──────────────────────────────┐
                  │    STATE_COMMAND_INJECT      │
                  │  • Load cmd_repeat_count = 10│
                  │  • Pack frame (Telemetry=1)  │
                  └──────────────┬───────────────┘
                                 │
                                 ▼
                  ┌──────────────────────────────┐
                  │    STATE_COMMAND_SENDING     │
                  │  • Stream 1 frame / 1 ms     │
                  │  • Decrement repeat_count    │
                  └──────────────┬───────────────┘
                                 │ repeat_count == 0
                                 ▼
                  ┌──────────────────────────────┐
                  │ Return to STREAMING_THROTTLE │
                  └──────────────────────────────┘
```
This pattern allows arbitrary commands (beeps, configuration, direction changes) to be dispatched asynchronously without blocking or disrupting the 1000 Hz thruster control stream.

---

## 7. 3D / Bidirectional Motor Mode (Forward & Reverse)

In ROV thruster operations requiring instantaneous bi-directional thrust without mechanical gearboxes, Bluejay ESC firmware is configured in **Forward/Reverse (3D mode)** (via `tools/esc_passthrough` and [esc-configurator.com](https://esc-configurator.com)).

### Bluejay 3D Throttle Range Mapping (Verified via `Bluejay.asm` Line 817)
Bluejay implements the standard 3D bidirectional framing where the 11-bit throttle space is bifurcated:

```assembly
; Check for bidirectional operation (0=stop, 96-2095->fwd, 2096-4095->rev)
```

| DShot Packet Value | Throttle Region | Motor Physical Behavior |
| :--- | :--- | :--- |
| **`0`** | Disarmed / Motor Stop | Output FETs off; required for arming and true coast stop. |
| **`1` .. `47`** | DShot Special Commands | Beeps, beacon, telemetry toggles. |
| **`48`** | Direction A Zero-Idle | Minimum idle floor (0% throttle) in Direction A. |
| **`49` .. `1046`** | Direction A Active Range | Proportional thrust scaling from 0% up to 100%. |
| **`1047`** | Direction A Maximum | **100% Full Power Thrust (Direction A / Forward)**. |
| **`1048`** | Direction B Zero-Idle | Minimum idle floor (0% throttle) in Direction B. |
| **`1049` .. `2046`** | Direction B Active Range | Proportional thrust scaling from 0% up to 100%. |
| **`2047`** | Direction B Maximum | **100% Full Power Thrust (Direction B / Reverse)**. |

### Critical 3D Configuration & Command Rules

#### 1. Arming Signal (`0`)
- In Bluejay (`BLHeli_S.asm` lines 3911–3913), the arming routine specifically inspects that the pulse length equals `0`.
- Firmware must stream **`0`** at 1000 Hz for at least **1.5 to 2.0 seconds** during initialization to trigger the rising arm chime.
- Streaming `1048` during arming will NOT arm the ESC (it will sound only a single detection beep).

#### 2. Prohibition of Direction Commands (Command 20 / 21)
- In 3D Bidirectional Mode, firmware must **never send Command 20 or Command 21**.
- Sending Command 20 forces the ESC out of bidirectional mode and overrides internal 3D direction branching.

#### 3. Recommended ESC Parameters for T200 Low-Kv Marine Thrusters
- **Motor Timing**: **`15° (Medium)`** (30° High causes retarding counter-torque and startup stutter on low-Kv motors).
- **Demag Compensation**: **`Off`** (prevents false power cuts on high-inductance marine motor coils).
- **Minimum Startup Power (Boost)**: **`1125`** (provides maximum open-loop breakaway torque).
- **RPM Power Protection (Rampup)**: **`Off`** (prevents low-speed torque throttling).
- **Brake on Stop**: **`Unchecked`** (prevents destructive regenerative inductive voltage spikes into DC power rails).

---

## 8. Extended DShot Telemetry (EDT) Sensor Scaling

Bluejay Extended DShot Telemetry (EDT) alternates eRPM period frames with auxiliary sensor frames tagged in the upper nibble:

| Metric | Bit Width | Unit | Conversion Formula |
| :--- | :--- | :--- | :--- |
| **Voltage** | 12 bits | Centivolts (cV) | $V = \frac{\text{raw}}{100} \quad [\text{V}]$ |
| **Current** | 12 bits | Centiamperes (cA) | $I = \frac{\text{raw}}{100} \quad [\text{A}]$ |
| **Temperature** | 12 bits | Celsius (°C) | $T = \text{raw} \quad [^\circ\text{C}]$ |
| **Stress / Demag** | 12 bits | Dimensionless / % | $\text{Stress} = \frac{\text{raw}}{40.95} \quad [\%]$ |

---

## 9. Hardware-Periodic 1 kHz DMA Triggering via TIM6 & DMAMUX

To achieve continuous 1 kHz DShot frame streaming with zero CPU overhead:

1. **TIM6 Base Timer**: Configured to produce a `TIM6_UP` update trigger every 1.0 ms (1 kHz).
2. **DMAMUX Request Generator (`DMAMUX1_REQ_GEN`)**:
   - `SignalID = HAL_DMAMUX1_REQ_GEN_TIM6_UP`
   - `Polarity = HAL_DMAMUX_REQ_GEN_RISING`
   - `RequestNumber = 1`
   - Configured via `HAL_DMAEx_ConfigMuxRequestGenerator(&hdma_tim1_ch1, &sRequestGenConfig)`.
3. **Execution**: The hardware DMAMUX Request Generator autonomous triggers the DMA transfer on every TIM6 tick. Application code simply updates throttle values in RAM buffer without software delay loops or interrupt servicing.

---

## 10. Bench Verification, Physical ESC Tuning & Slew-Rate Dynamics

### Hardware Bench Test Configuration
- **Host MCU**: NUCLEO-G474RE (STM32G474RET6, Cortex-M4 @ 170 MHz).
- **Physical Output Pin**: `PA8` (TIM1 Channel 1 via `GPIO_AF6_TIM1`).
- **DMA Request**: `DMA1_Channel1` mapped to `DMA_REQUEST_TIM1_CH1` (Request ID 42U).
- **ESC / Firmware**: BLHeli_S hardware flashed with Bluejay ESC firmware (Forward-Only mode).
- **Thruster**: Blue Robotics T200 sensorless brushless DC motor.
- **Power Supply**: 12.0 V bench power supply.

### Verified Physical Findings

#### 1. Push-Pull GPIO Configuration (`GPIO_MODE_AF_PP`)
- The signal pin must always be configured as `GPIO_MODE_AF_PP` with an internal pull-down (`GPIO_PULLDOWN`).
- Open-drain mode (`GPIO_MODE_AF_OD`) combined with the internal ~40 kΩ pull-up resistor produces an RC rise time exceeding 4.0 µs. This exceeds the total DShot bit duration (3.33 µs for DShot300), attenuating all pulses to near-zero amplitude and preventing the ESC from receiving any frames.

#### 2. Power Supply Current Limit & Brownout Power-Cycling
- When the bench power supply was set to a 6 A limit, high throttle acceleration triggered instantaneous current spikes exceeding 6 A.
- This caused the 12 V rail to sag below the ESC reset threshold (~6 V). The ESC power-cycled, emitted reboot beeps, re-armed, spun briefly, and tripped again.
- Setting the bench supply current limit to 9 A eliminated power-rail brownout, enabling full throttle profiling.

#### 3. Steady-State Current vs Dynamic Inrush
- In air (unloaded thruster):
  - Throttle `350` (~15%): draws **0.10 A**.
  - Throttle `1000` (~50%): draws **0.30 A**.
  - Throttle `2047` (100% full speed): draws **0.60 A**.
- Because steady-state current is minimal in air, failure to run at high throttle is never steady-state current overload—it is driven entirely by dynamic acceleration slew rate ($d\omega/dt$).

#### 4. Sensorless Commutation Desync ($d\omega/dt$ Slew Rate)
- The T200 thruster is a sensorless BLDC motor relying on back-EMF (BEMF) zero-crossing detection on the un-driven stator winding during PWM off-time.
- Demanding instantaneous full throttle (`2047`) or jumping throttle over short 1-second intervals accelerates the stator's rotating magnetic field faster than the mechanical rotor and magnet assembly can physically accelerate against rotor inertia.
- When the rotor lags behind the electrical field, the BEMF zero-crossing window is missed. The ESC loses commutation lock ("desync"), resulting in violent stuttering, high acoustic harshness, and stalling.
- By contrast, stepping through a graduated ladder (`1000 -> 1200 -> 1400 -> 1600 -> 1700 -> 1800 -> 1850 -> 1900 -> 1950 -> 2000 -> 2047`) holding 2.5 seconds at each plateau allows the rotor to lock phase perfectly, achieving stable 100% speed (`2047`) at 0.60 A with zero jitter.
- **Production Rule**: All vehicle thruster setpoints must pass through a slew-rate limiter (maximum acceleration: 500–800 DShot throttle units per second) to prevent commutation desync in both air and water.

#### 5. Prohibition of Blocking UART I/O During Acceleration
- Any `printf` call over UART at 115200 baud blocks the CPU for ~87 µs per character (~1.8 ms for a 20-character line).
- Blocking UART output inside an active ramp loop starves the 1 kHz DShot stream, inserting multi-millisecond gaps in the frame cadence.
- Frame dropouts during active motor acceleration disrupt the ESC's internal digital throttle filter. Firmware must never execute blocking serial transmissions inside active ramp loops.

#### 6. Zero-Glitch DMA Parking
- The DMA transmit buffer consists of 17 halfwords: 16 DShot bit timings followed by one trailing `0`.
- In `HAL_TIM_PWM_PulseFinishedCallback()`, the timer channel DMA is halted via `HAL_TIM_PWM_Stop_DMA()`, and `TIM1->CCR1` is clamped to `0`.
- This ensures the output line remains in the low idle state throughout the inter-frame interval.

---

## 11. ESC Configuration & 4-Way Serial Passthrough

To configure Bluejay / BLHeli ESC parameters (such as enabling 3D Bidirectional Mode, reversing motor direction, or adjusting PWM frequency) directly from a browser via [esc-configurator.com](https://esc-configurator.com):

### Authoritative Project & Binaries
- **Source Location**: [`tools/esc_passthrough/Core/Src/esc_passthrough.c`](file:///C:/Users/aman/Documents/Engineering/ROV/Embedded/X19-Embedded/tools/esc_passthrough/Core/Src/esc_passthrough.c)
- **Precompiled Binary**: [`tools/esc_passthrough/Debug/esc_passthrough.elf`](file:///C:/Users/aman/Documents/Engineering/ROV/Embedded/X19-Embedded/tools/esc_passthrough/Debug/esc_passthrough.elf)
- **STM32CubeMX Configuration**: `tools/esc_passthrough/esc_passthrough.ioc`

### Flashing Procedure
To flash the passthrough firmware to the NUCLEO-G474RE bench board without attaching a serial monitor:
```powershell
& "C:\ST\STM32CubeCLT_1.22.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.EXE" -c port=SWD sn=0025002B3235511337333439 mode=UR reset=HWrst -d "tools\esc_passthrough\Debug\esc_passthrough.elf" -v -rst
```

### Usage Instructions
1. Ensure the ESC is powered by the 12.0 V bench supply.
2. In Chrome or Edge, open [esc-configurator.com](https://esc-configurator.com).
3. Click **Connect** and select the ST-Link Virtual COM Port (e.g. `COM6`).
4. Click **Read Settings**.
5. Make required parameter modifications (e.g., set Motor Direction to **3D / Bidirectional**), then click **Save / Flash**.
6. Disconnect from the browser tool before re-flashing operational firmware.

### Architecture Notes & Troubleshooting
- **Protocol Emulation**: Implements Betaflight MSP V1 (`MSP_SET_PASSTHROUGH` function `245`) over LPUART1 (ST-Link VCP) bridging to the BLHeli 4-way 19200 baud 1-wire UART on `PA8`.
- **Zero-Banner Binary Transport**: Must never print ASCII strings or banners over the VCP link. WebSerial apps expect strict binary framing; ASCII banners cause communication timeouts and COM port sharing errors.
- **Historical Note**: Prior prototype implementations (`sandbox/esc_passthrough_4way.c` and `sandbox/esc_passthrough.c`) were deleted because they contained blocking delay loops and ASCII logging that corrupted binary 4-way framing. `tools/esc_passthrough/` is the sole verified passthrough implementation.



