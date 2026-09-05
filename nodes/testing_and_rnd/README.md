# Testing & R&D Embedded Project

> **Standalone Bench Testing & R&D Sandbox for STM32 + Raspberry Pi CAN Verification**  
> Target: STM32F446RE (NUCLEO-F446RE) <---> Raspberry Pi (SocketCAN `can0`)

---

## 1. Project Purpose

This project provides an isolated, low-overhead environment to test and debug:
1. Physical CAN bus connectivity, clock stability, and transceiver wiring between an STM32F446RE board and a Raspberry Pi.
2. Bidirectional packet exchange:
   - **Pi $\rightarrow$ STM32**: Command frame (`0x100`, 8 bytes).
   - **STM32 $\rightarrow$ Pi**: Telemetry stream (`0x123`, 8 bytes).
3. Roundtrip latency benchmarks and filter validation.

---

## 2. Hardware Wiring Checklist

| STM32 Board (NUCLEO-F446RE) | CAN Transceiver (e.g. TCAN1044 / SN65HVD230) | Raspberry Pi CAN Transceiver | Pi Header Pin |
| :--- | :--- | :--- | :--- |
| `PB9` (CAN1_TX) | `TXD` | - | - |
| `PB8` (CAN1_RX) | `RXD` | - | - |
| `3.3V` / `5.0V` | `VCC` / `VIO` | `VCC` / `VIO` | Pin 1 (3.3V) / Pin 2 (5V) |
| `GND` | `GND` | `GND` (Shared Ground) | Pin 6 (GND) |
| - | `CAN_H` | `CAN_H` | - |
| - | `CAN_L` | `CAN_L` | - |

> **Bus Termination**: Place a $120\,\Omega$ resistor across `CAN_H` and `CAN_L` at each end of the bus (or enable the on-board termination jumper if present).

---

## 3. Building & Flashing

### Via VS Code
1. Open [`X19-Embedded.code-workspace`](../../X19-Embedded.code-workspace).
2. Select **Testing & R&D Project** from the folder list.
3. In the STM32Cube extension tab, build `testing_and_rnd.elf` and flash via ST-Link (`F5`).

### Via Command Line
```powershell
cd X19-Embedded
cmake -B build
cmake --build build --target testing_and_rnd.elf
```

---

## 4. Raspberry Pi Side Verification

### 1. Bring up CAN FD on the Pi
```bash
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
ip -details -statistics link show can0
```

### 2. Run the Ping-Pong & Latency Test
```bash
python3 nodes/testing_and_rnd/pi_can_test.py --interface can0 --mode ping
```

### 3. Live Sniffer
```bash
python3 nodes/testing_and_rnd/pi_can_test.py --interface can0 --mode sniff
```
