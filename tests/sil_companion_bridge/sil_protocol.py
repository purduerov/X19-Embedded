"""
Purdue ROV Software-in-the-Loop (SIL) Protocol Definitions for Python.
Matches the C framing used in rov_can_protocol.h and sil_bridge_server.c.
"""

import struct
from dataclasses import dataclass
from typing import List, Tuple

# Arbitration CAN IDs
CAN_ID_EMERGENCY_BREAK   = 0x001
CAN_ID_EFUSE_FAULT_ALERT = 0x005
CAN_ID_THRUSTER_CMD      = 0x100
CAN_ID_SOLENOID_CMD      = 0x110
CAN_ID_NAV_TELEMETRY     = 0x200
CAN_ID_ENV_TELEMETRY     = 0x210
CAN_ID_POWER_TELEMETRY   = 0x300
CAN_ID_USB_HUB_TELEMETRY = 0x310

SIL_MAGIC_HEADER = 0x524F5643  # "ROVC"
SIL_MAGIC_HEADER_LEGACY = 0x58313943  # "X19C"
SIL_PACKET_FMT = "<IIB64s"
SIL_PACKET_SIZE = struct.calcsize(SIL_PACKET_FMT)

@dataclass
class ThrusterCommand:
    pwm_us: List[int]  # 8 channels (1000 - 2000 us)

    def pack(self) -> bytes:
        if len(self.pwm_us) != 8:
            raise ValueError("ThrusterCommand requires exactly 8 PWM values")
        return struct.pack("<8H", *self.pwm_us)

    @classmethod
    def unpack(cls, data: bytes) -> "ThrusterCommand":
        pwms = struct.unpack("<8H", data[:16])
        return cls(pwm_us=list(pwms))

@dataclass
class SolenoidCommand:
    solenoid_mask: int  # 10 bits

    def pack(self) -> bytes:
        return struct.pack("<H", self.solenoid_mask & 0x03FF)

    @classmethod
    def unpack(cls, data: bytes) -> "SolenoidCommand":
        (mask,) = struct.unpack("<H", data[:2])
        return cls(solenoid_mask=mask & 0x03FF)

@dataclass
class NavTelemetry:
    q_w: float
    q_x: float
    q_y: float
    q_z: float
    gyro_x_rad_s: float
    gyro_y_rad_s: float
    gyro_z_rad_s: float
    depth_meters: float
    imu_status: int

    @classmethod
    def unpack(cls, data: bytes) -> "NavTelemetry":
        qw, qx, qy, qz, gx, gy, gz, depth, status = struct.unpack("<8fB", data[:33])
        return cls(
            q_w=qw, q_x=qx, q_y=qy, q_z=qz,
            gyro_x_rad_s=gx, gyro_y_rad_s=gy, gyro_z_rad_s=gz,
            depth_meters=depth, imu_status=status
        )

@dataclass
class EnvTelemetry:
    pressure_hpa: float
    humidity_pct: float
    temperature_c: float
    leak_flags: int

    @classmethod
    def unpack(cls, data: bytes) -> "EnvTelemetry":
        p, h, t, flags = struct.unpack("<3fB", data[:13])
        return cls(pressure_hpa=p, humidity_pct=h, temperature_c=t, leak_flags=flags)

@dataclass
class PowerTelemetry:
    tether_voltage_mv: int
    tether_current_ma: int
    v5_voltage_mv: int
    v5_current_ma: int
    v12_current_ma: List[int]
    pcb_temp_c_tenths: int
    status_flags: int

    @classmethod
    def unpack(cls, data: bytes) -> "PowerTelemetry":
        tv, ti, v5v, v5i, b1, b2, b3, b4, temp, flags = struct.unpack("<HHHH4HhH", data[:20])
        return cls(
            tether_voltage_mv=tv, tether_current_ma=ti,
            v5_voltage_mv=v5v, v5_current_ma=v5i,
            v12_current_ma=[b1, b2, b3, b4],
            pcb_temp_c_tenths=temp, status_flags=flags
        )

def pack_sil_can_frame(can_id: int, payload: bytes) -> bytes:
    """Wraps a CAN FD frame into the SIL TCP stream format."""
    padded = payload.ljust(64, b"\x00")[:64]
    return struct.pack(SIL_PACKET_FMT, SIL_MAGIC_HEADER, can_id, len(payload), padded)

def unpack_sil_can_frame(chunk: bytes) -> Tuple[int, bytes]:
    """Unpacks a SIL TCP frame into (can_id, payload_bytes)."""
    magic, can_id, length, data = struct.unpack(SIL_PACKET_FMT, chunk[:SIL_PACKET_SIZE])
    if magic != SIL_MAGIC_HEADER and magic != SIL_MAGIC_HEADER_LEGACY:
        raise ValueError(f"Invalid magic header: {hex(magic)}")
    return can_id, data[:length]
