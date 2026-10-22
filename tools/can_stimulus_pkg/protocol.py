"""Purdue ROV X19 CAN Stimulus - Protocol Framing & Validation."""

from __future__ import annotations

import argparse
from typing import Tuple

from .constants import MAX_SIL_PAYLOAD, NUM_THRUSTERS, PWM_MAX_US, PWM_MIN_US


def validate_payload(payload: bytes) -> bytes:
    """Return payload as bytes, rejecting anything the packet cannot carry."""
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise TypeError(f"CAN payload must be bytes, got {type(payload).__name__}")
    payload = bytes(payload)
    if len(payload) > MAX_SIL_PAYLOAD:
        raise ValueError(
            f"SIL frame payload is {len(payload)} bytes; the packet carries at most "
            f"{MAX_SIL_PAYLOAD} (sil_protocol.SIL_PACKET_FMT)"
        )
    return payload


def validate_can_id(can_id: int) -> int:
    """Return can_id after checking it is a plausible arbitration id."""
    if not isinstance(can_id, int) or isinstance(can_id, bool):
        raise TypeError(f"CAN id must be an int, got {type(can_id).__name__}")
    if not 0 <= can_id <= 0x1FFFFFFF:
        raise ValueError(f"CAN id 0x{can_id:X} is out of range")
    return can_id


def describe_can_id(can_id: int) -> str:
    """Render an arbitration id the way every message in this tool does."""
    return f"CAN 0x{can_id:03X}"


def _validate_pulse_us(pulse_us: object) -> int:
    if not isinstance(pulse_us, int) or isinstance(pulse_us, bool):
        raise TypeError(f"PWM pulse width must be an int, got {type(pulse_us).__name__}")
    if not PWM_MIN_US <= pulse_us <= PWM_MAX_US:
        raise ValueError(
            f"PWM {pulse_us} us is outside the firmware envelope "
            f"{PWM_MIN_US}-{PWM_MAX_US} us (rov_parameters.h:39-40)"
        )
    return pulse_us


def _validate_channel(channel: object) -> int:
    if not isinstance(channel, int) or isinstance(channel, bool):
        raise TypeError(f"thruster channel must be an int, got {type(channel).__name__}")
    if not 0 <= channel < NUM_THRUSTERS:
        raise ValueError(
            f"thruster channel {channel} does not exist; the board drives {NUM_THRUSTERS} "
            "channels (rov_parameters.h:37)"
        )
    return channel


def int_literal(text: str) -> int:
    """Parse an operator-supplied integer written in any of the usual literal forms."""
    stripped = text.strip()
    bases = {"0x": 16, "0o": 8, "0b": 2}
    base = bases.get(stripped[:2].lower(), 10)
    try:
        return int(stripped, base)
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"{text!r} is not an integer; write plain decimal (10, 010, 1023) or a "
            "prefixed literal (0x3FF, 0o777, 0b1010)"
        ) from None


def _raw_frame(text: str) -> Tuple[int, bytes]:
    """Parse 0x100:AABB into (can_id, payload)."""
    if ":" not in text:
        raise argparse.ArgumentTypeError(
            f"--raw-send expects CAN_ID:HEXBYTES (for example 0x001:AA5501), got {text!r}"
        )
    id_text, _, hex_text = text.partition(":")
    can_id = int_literal(id_text)
    try:
        payload = bytes.fromhex(hex_text.replace(" ", ""))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid hex payload {hex_text!r}: {exc}") from exc
    if len(payload) > MAX_SIL_PAYLOAD:
        raise argparse.ArgumentTypeError(
            f"payload is {len(payload)} bytes; a SIL frame carries at most {MAX_SIL_PAYLOAD}"
        )
    return can_id, payload
