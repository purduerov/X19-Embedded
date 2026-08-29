# X19 Remote CAN Bootloader Suite

> **Purdue ROV — High-Speed Underwater CAN FD Bootloading & Firmware Flashing System**  
> *Operates at 1 Mbps Arbitration / 5 Mbps Data Phase over CAN ID `0x700`/`0x701`*

---

## 1. Overview & Architecture

The **X19 CAN Bootloader** allows the Raspberry Pi 5 (`X19-Core`) or surface computer to reflash firmware binaries into any subsea STM32G4 microcontroller without unsealing the aluminum enclosure tube.

```
┌────────────────────────────────────────────────────────────────────────┐
│             STM32G4 FLASH MEMORY PARTITIONING (128 KB)                 │
├────────────────────────────────┬───────────────────────────────────────┤
│ Sector 0 (0x08000000 - 16 KB)  │ Sector 1 to End (0x08004000 - 112 KB) │
│ Custom CAN FD Bootloader       │ Application Firmware (Main ROV Logic) │
│ - FDCAN1 @ 1 Mbps / 5 Mbps     │ - Vector Table Offset:                │
│ - Chunk CRC-16 Verification    │   SCB->VTOR = 0x08004000              │
└────────────────────────────────┴───────────────────────────────────────┘
```

---

## 2. Flashing Workflow via Python Tool

```bash
# Flash Control Board firmware over SocketCAN
python bootloader/tools/can_flash.py --interface can0 --target control_board --bin build/nodes/node2_control_board/node2_control_board.bin

# Flash Power Slab firmware
python bootloader/tools/can_flash.py --interface can0 --target power_slab --bin build/nodes/node3_power_slab/node3_power_slab.bin

# Flash Pi Shield firmware
python bootloader/tools/can_flash.py --interface can0 --target pi_shield --bin build/nodes/node1_pi_shield/node1_pi_shield.bin
```
