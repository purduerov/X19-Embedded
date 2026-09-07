#!/usr/bin/env python3
"""
X19 Vehicle CAN FD Telemetry Sniffer & Live Dashboard.
Decodes 100 Hz Nav, 10 Hz Leak, 20 Hz Power, and Thruster PWM streams.
Purdue ROV 2026-2027.
"""

import sys
import time
import struct
import argparse

try:
    import can
except ImportError:
    print("python-can is not installed. Run 'pip install python-can'")
    sys.exit(1)

CAN_ID_EMERGENCY_BREAK   = 0x001
CAN_ID_EFUSE_FAULT_ALERT = 0x005
CAN_ID_THRUSTER_CMD      = 0x100
CAN_ID_SOLENOID_CMD      = 0x110
CAN_ID_NAV_TELEMETRY     = 0x200
CAN_ID_ENV_TELEMETRY     = 0x210
CAN_ID_POWER_TELEMETRY   = 0x300
CAN_ID_USB_HUB_TELEMETRY = 0x310

def decode_msg(msg: can.Message):
    ts = time.strftime("%H:%M:%S", time.localtime(msg.timestamp))
    msg_id = msg.arbitration_id
    data = msg.data

    if msg_id == CAN_ID_EMERGENCY_BREAK:
        if len(data) >= 2 and data[0] == 0xAA and data[1] == 0x55:
            print(f"[{ts}] [CAN 0x001] *** EMERGENCY BREAK / LEAK SHUTDOWN ACTIVE ***")
        else:
            print(f"[{ts}] [CAN 0x001] WARNING: Malformed EMERGENCY BREAK packet (len={len(data)})")

    elif msg_id == CAN_ID_EFUSE_FAULT_ALERT:
        if len(data) >= 1:
            fault_code = data[0]
            print(f"[{ts}] [CAN 0x005] *** POWER SLAB EFUSE FAULT ALERT: Code 0x{fault_code:02X} ***")
        else:
            print(f"[{ts}] [CAN 0x005] WARNING: Malformed EFUSE FAULT ALERT packet (len={len(data)})")

    elif msg_id == CAN_ID_THRUSTER_CMD:
        if len(data) == 16:
            pwms = struct.unpack("<8H", data[:16])
            pwm_str = " ".join([f"T{i+1}:{pwms[i]}us" for i in range(8)])
            print(f"[{ts}] [CAN 0x100] Thruster PWMs -> {pwm_str}")
        else:
            print(f"[{ts}] [CAN 0x100] WARNING: Malformed Thruster CMD packet (len={len(data)}, expected 16)")

    elif msg_id == CAN_ID_SOLENOID_CMD:
        if len(data) == 2:
            mask = struct.unpack("<H", data[:2])[0]
            active = [f"V{i+1}" for i in range(10) if (mask & (1 << i))]
            print(f"[{ts}] [CAN 0x110] Solenoids -> Active: {active if active else 'None'}")
        else:
            print(f"[{ts}] [CAN 0x110] WARNING: Malformed Solenoid CMD packet (len={len(data)}, expected 2)")

    elif msg_id == CAN_ID_NAV_TELEMETRY:
        if len(data) == 33:
            qw, qx, qy, qz, gx, gy, gz, depth, status = struct.unpack("<ffffffffB", data[:33])
            print(f"[{ts}] [CAN 0x200] Nav: Depth={depth:5.2f}m | Gyro=({gx:+.2f}, {gy:+.2f}, {gz:+.2f}) | Quat=({qw:.2f}, {qx:.2f}, {qy:.2f}, {qz:.2f}) | Cal={status}")
        else:
            print(f"[{ts}] [CAN 0x200] WARNING: Malformed Nav Telemetry packet (len={len(data)}, expected 33)")

    elif msg_id == CAN_ID_ENV_TELEMETRY:
        if len(data) == 13:
            press, hum, temp, leak = struct.unpack("<fffB", data[:13])
            leak_str = "LEAK ALERT!" if leak != 0 else "OK"
            print(f"[{ts}] [CAN 0x210] Env: P={press:6.1f}hPa | Hum={hum:4.1f}% | Temp={temp:4.1f}C | Status={leak_str}")
        else:
            print(f"[{ts}] [CAN 0x210] WARNING: Malformed Env Telemetry packet (len={len(data)}, expected 13)")

    elif msg_id == CAN_ID_POWER_TELEMETRY:
        if len(data) == 20:
            v_tether, i_tether, v5, i5, b1, b2, b3, b4, temp, status = struct.unpack("<HHHHHHHHhH", data[:20])
            print(f"[{ts}] [CAN 0x300] Power: Tether={v_tether/1000.0:4.1f}V @ {i_tether/1000.0:4.1f}A | 5V Rail={v5/1000.0:4.2f}V | Bricks=[{b1}mA, {b2}mA, {b3}mA, {b4}mA] | Temp={temp/10.0:.1f}C")
        else:
            print(f"[{ts}] [CAN 0x300] WARNING: Malformed Power Telemetry packet (len={len(data)}, expected 20)")

def main():
    parser = argparse.ArgumentParser(description="X19 CAN FD Telemetry Sniffer")
    parser.add_argument("--interface", default="can0", help="CAN interface (default: can0 or vcan0)")
    args = parser.parse_args()

    print(f"Connecting to {args.interface} (CAN FD)...")
    try:
        bus = can.Bus(channel=args.interface, interface="socketcan", fd=True)
    except Exception as e:
        print(f"Error opening interface {args.interface}: {e}")
        return

    print("Sniffer running. Press Ctrl+C to exit.\n")
    try:
        for msg in bus:
            decode_msg(msg)
    except KeyboardInterrupt:
        print("\nSniffer stopped.")

if __name__ == "__main__":
    main()
