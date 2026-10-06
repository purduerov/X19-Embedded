#!/usr/bin/env python3
"""Minimal host-side test for the STM32 'esc_passthrough_4way' firmware.

Opens COM6 at 115200, sends MSP handshake like esc-configurator.com, then
switches into 4-way and sends the same cmd_InterfaceTestAlive /
cmd_DeviceInitFlash packets the site uses.
"""
import sys
import serial

COM = sys.argv[1] if len(sys.argv) > 1 else "COM6"
BAUD = 115200


def msp_frame(cmd_id_lo, cmd_id_hi, payload=b""):
    # MSP V1 frame: b'$M<' len cmd payload checksum
    # Note: cmd_id_hi is unused for V1
    body = bytearray()
    body.append(len(payload))
    body.append(cmd_id_lo)
    body.extend(payload)
    crc = body[0] ^ body[1]
    for b in body[2:]:
        crc ^= b
    return b"$M<" + bytes(body) + bytes([crc])


def fourway_frame(cmd, payload, address=0):
    body = bytearray()
    body.append(0x2F)
    body.append(cmd)
    body.append((address >> 8) & 0xFF)
    body.append(address & 0xFF)
    body.append(len(payload) if payload else 1)
    if payload:
        body.extend(payload)
    else:
        body.append(0x00)
    crc = 0
    for b in body:
        crc ^= b << 8
        crc &= 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    body.append((crc >> 8) & 0xFF)
    body.append(crc & 0xFF)
    return bytes(body)


def read_all(s, timeout=0.5):
    import time
    s.timeout = timeout
    data = s.read(4096)
    return data


def main():
    print(f"Opening {COM} @ {BAUD}")
    with serial.Serial(COM, BAUD, timeout=1, write_timeout=1) as s:
        # MSP MSP_API_VERSION = 1
        print("TX api version")
        s.write(msp_frame(1, 0))
        print("RX:", read_all(s, 0.5).hex())

        print("TX fc variant")
        s.write(msp_frame(2, 0))
        print("RX:", read_all(s, 0.5).hex())

        print("TX board info")
        s.write(msp_frame(4, 0))
        print("RX:", read_all(s, 0.5).hex())

        print("TX motor data (104)")
        s.write(msp_frame(104, 0))
        print("RX:", read_all(s, 0.5).hex())

        print("TX set 4way passthrough (245)")
        s.write(msp_frame(245, 0))
        print("RX:", read_all(s, 0.5).hex())

        # Firmware holds the line in reset + boot-settle for ~1.3s before it
        # will answer any 4-way command, so wait that out first.
        print("Waiting 2.0s for ESC reset + boot settle...")
        import time

        time.sleep(2.0)

        print("TX cmd_InterfaceTestAlive (0x30)")
        s.write(fourway_frame(0x30, []))
        r = read_all(s, 0.5)
        print("RX:", r.hex(), "| ack =", hex(r[5]) if len(r) > 5 else "n/a")

        print("TX cmd_DeviceInitFlash (0x37) target 0")
        s.write(fourway_frame(0x37, [0, 0]))
        r = read_all(s, 2.0)
        print("RX:", r.hex(), "| ack =", hex(r[5]) if len(r) > 5 else "n/a")
        if len(r) >= 9:
            p = r[5:9]
            edges = (p[0] << 8) | p[1]
            print(
                "  line_edges=%d bytes_read=%d reason=%d (%s)"
                % (
                    edges,
                    p[2],
                    p[3],
                    {
                        1: "timeout waiting for ESC byte",
                        2: "read all 8 bytes",
                    }.get(p[3], "unknown"),
                )
            )
            if edges == 0:
                print(
                    "  >> ESC never drove the line: it is not listening "
                    "(wrong state, no power, or wrong pin)"
                )
            else:
                print(
                    "  >> ESC DID move the line (%d edges) but bytes failed "
                    "to decode: sampling/timing issue" % edges
                )

        print("TX cmd_DeviceRead (0x3A) addr 0x1A00 len 48")
        s.write(fourway_frame(0x3A, [48], 0x1A00))
        r = read_all(s, 2.0)
        print("RX:", r.hex(), "| ack =", hex(r[5]) if len(r) > 5 else "n/a")


if __name__ == "__main__":
    main()
