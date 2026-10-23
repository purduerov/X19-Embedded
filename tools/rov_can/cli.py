"""`rov can ...` - sniff and stimulate the vehicle CAN bus (real or simulated)."""

from __future__ import annotations

import argparse
import struct
import sys
import time
import types
from typing import List, Optional

from .transport import _SIL_DIR, open_transport

if str(_SIL_DIR) not in sys.path:
    sys.path.insert(0, str(_SIL_DIR))
import sil_protocol as P  # noqa: E402

PWM_MIN_US, PWM_NEUTRAL_US, PWM_MAX_US = 1000, 1500, 2000
ESTOP_FRAME = b"\xAA\x55\x01"


def _decoder():
    """Reuse the sniffer's decoder when python-can is importable; else raw hex."""
    try:
        from tools import can_sniffer

        return can_sniffer.decode_msg
    except (ImportError, SystemExit):
        return None


def cmd_sniff(t, seconds: float) -> int:
    decode = _decoder()
    end = time.monotonic() + seconds if seconds > 0 else None
    n = 0
    print("sniffing" + (f" for {seconds:g}s" if end else " (Ctrl+C to stop)"))
    try:
        while end is None or time.monotonic() < end:
            frame = t.receive_message(0.5)
            if frame is None:
                continue
            n += 1
            can_id, data = frame
            if decode:
                decode(types.SimpleNamespace(arbitration_id=can_id, data=data, timestamp=time.time()))
            else:
                print(f"[0x{can_id:03X}] {len(data):2d}B {data.hex(' ')}")
    except KeyboardInterrupt:
        pass
    print(f"{n} frame(s) seen")
    return 0 if n else 1


def cmd_estop(t) -> int:
    t.send_message(P.CAN_ID_EMERGENCY_BREAK, ESTOP_FRAME)
    print(f"sent emergency break 0x{P.CAN_ID_EMERGENCY_BREAK:03X} {ESTOP_FRAME.hex(' ')}")
    return 0


def cmd_pwm(t, values: List[int], hz: float, seconds: float) -> int:
    if len(values) == 1:
        values = values * 8
    if len(values) != 8:
        print("error: give 1 value (all channels) or 8 values", file=sys.stderr)
        return 2
    bad = [v for v in values if not PWM_MIN_US <= v <= PWM_MAX_US]
    if bad:
        print(f"error: {bad} outside {PWM_MIN_US}-{PWM_MAX_US} us (neutral is {PWM_NEUTRAL_US})", file=sys.stderr)
        return 2
    payload = struct.pack("<8H", *values)
    period = 1.0 / hz
    end = time.monotonic() + seconds
    sent = 0
    try:
        while time.monotonic() < end:
            t.send_message(P.CAN_ID_THRUSTER_CMD, payload)
            sent += 1
            time.sleep(period)
    finally:
        t.send_message(P.CAN_ID_THRUSTER_CMD, struct.pack("<8H", *([PWM_NEUTRAL_US] * 8)))
    print(f"sent {sent} PWM frame(s) {values}, then returned all channels to {PWM_NEUTRAL_US} us")
    return 0


def cmd_send(t, can_id: int, hexdata: str) -> int:
    t.send_message(can_id, bytes.fromhex(hexdata.replace(" ", "")))
    print(f"sent 0x{can_id:03X} {hexdata}")
    return 0


def build_parser(p: Optional[argparse.ArgumentParser] = None) -> argparse.ArgumentParser:
    p = p or argparse.ArgumentParser(prog="rov can")
    p.add_argument("-i", "--interface", default="can0", help="SocketCAN interface (default can0)")
    p.add_argument("--sil", metavar="HOST[:PORT]", help="use the host SIL bridge instead of SocketCAN")
    sub = p.add_subparsers(dest="can_action", required=True)
    s = sub.add_parser("sniff", help="decode live traffic")
    s.add_argument("-t", "--seconds", type=float, default=0, help="stop after N seconds (default: until Ctrl+C)")
    sub.add_parser("estop", help="broadcast emergency break (0x001)")
    w = sub.add_parser("pwm", help="send thruster PWM (0x100); returns to neutral when done")
    w.add_argument("values", type=int, nargs="+", help="one value for all 8 channels, or 8 values (us)")
    w.add_argument("--hz", type=float, default=50.0)
    w.add_argument("-t", "--seconds", type=float, default=1.0)
    r = sub.add_parser("send", help="send one raw frame")
    r.add_argument("can_id", type=lambda x: int(x, 0))
    r.add_argument("hexdata")
    return p


def run(args: argparse.Namespace) -> int:
    try:
        t = open_transport(args.interface, args.sil)
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    with t:
        if args.can_action == "sniff":
            return cmd_sniff(t, args.seconds)
        if args.can_action == "estop":
            return cmd_estop(t)
        if args.can_action == "pwm":
            return cmd_pwm(t, args.values, args.hz, args.seconds)
        return cmd_send(t, args.can_id, args.hexdata)


if __name__ == "__main__":
    sys.exit(run(build_parser().parse_args()))
