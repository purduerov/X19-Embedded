"""
Unit & Integration Test: X19 CAN FD DBC Fidelity Test.
Verifies that encoding and decoding messages using shared/can/x19_rov.dbc
matches the exact binary layout of the C structs in rov_can_protocol.h.
"""

import math
import struct
from pathlib import Path
import cantools

DBC_PATH = Path(__file__).resolve().parent.parent / "shared" / "can" / "x19_rov.dbc"


def test_dbc_loads():
    assert DBC_PATH.exists(), f"DBC file not found at {DBC_PATH}"
    db = cantools.database.load_file(str(DBC_PATH))
    assert len(db.messages) == 13


def test_thruster_cmd_fidelity():
    db = cantools.database.load_file(str(DBC_PATH))
    msg = db.get_message_by_name("THRUSTER_CMD")
    pwms = [1500, 1600, 1400, 1550, 1450, 1900, 1100, 1500]

    # C struct layout: 8x uint16_t (<8H)
    c_raw = struct.pack("<8H", *pwms)

    signals = {f"thruster_{i+1}_pwm_us": pwms[i] for i in range(8)}
    dbc_encoded = msg.encode(signals)
    assert dbc_encoded == c_raw

    dbc_decoded = msg.decode(c_raw)
    for i in range(8):
        assert dbc_decoded[f"thruster_{i+1}_pwm_us"] == pwms[i]


def test_emergency_break_fidelity():
    db = cantools.database.load_file(str(DBC_PATH))
    msg = db.get_message_by_name("EMERGENCY_BREAK")

    # Byte 0: 0xAA, Byte 1: 0x55, Byte 2: 0x06 (probe 0 & probe 1 active)
    c_raw = bytes([0xAA, 0x55, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00])
    decoded = msg.decode(c_raw)
    assert decoded["magic_byte_0"] == 0xAA
    assert decoded["magic_byte_1"] == 0x55
    assert decoded["leak_bme280"] == 0
    assert decoded["leak_floor_probe_0"] == 1
    assert decoded["leak_floor_probe_1"] == 1


def test_nav_telemetry_fidelity():
    db = cantools.database.load_file(str(DBC_PATH))
    msg = db.get_message_by_name("NAV_TELEMETRY")

    # C struct: uint64_t ts, 4x float quat, 3x float gyro, 1x float depth, uint8_t status (<Q8fB)
    ts_us = 123456789
    qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
    gx, gy, gz = 0.05, -0.02, 0.15
    depth = 2.45
    status = 3

    c_raw = struct.pack("<Q8fB", ts_us, qw, qx, qy, qz, gx, gy, gz, depth, status)
    decoded = msg.decode(c_raw)

    assert decoded["timestamp_us"] == ts_us
    assert math.isclose(decoded["q_w"], qw, abs_tol=1e-5)
    assert math.isclose(decoded["gyro_x_rad_s"], gx, abs_tol=1e-5)
    assert math.isclose(decoded["depth_meters"], depth, abs_tol=1e-3)
    assert decoded["imu_status"] == status


def test_env_telemetry_fidelity():
    db = cantools.database.load_file(str(DBC_PATH))
    msg = db.get_message_by_name("ENV_TELEMETRY")

    # C struct: float press, float hum, float temp, uint8 leak, uint16 rail_mv, uint16 rail_ma (<3fBHH)
    press = 1013.25
    hum = 45.5
    temp = 24.2
    leak = 0x01  # BME280 leak bit
    rail_mv = 5120
    rail_ma = 1450

    c_raw = struct.pack("<3fBHH", press, hum, temp, leak, rail_mv, rail_ma)
    decoded = msg.decode(c_raw)

    assert math.isclose(decoded["pressure_hpa"], press, abs_tol=1e-2)
    assert math.isclose(decoded["humidity_pct"], hum, abs_tol=1e-2)
    assert math.isclose(decoded["temperature_c"], temp, abs_tol=1e-2)
    assert decoded["leak_bme280"] == 1
    assert decoded["leak_floor_probe_0"] == 0
    assert decoded["rail_voltage_mv"] == rail_mv
    assert decoded["rail_current_ma"] == rail_ma


def test_power_telemetry_fidelity():
    db = cantools.database.load_file(str(DBC_PATH))
    msg = db.get_message_by_name("POWER_TELEMETRY")

    # C struct: <HHHH4HhH (tether_v, tether_i, v5, i5, b1, b2, b3, b4, pcb_temp_tenths, status)
    tether_v = 48000
    tether_i = 12500
    v5_v = 5200
    v5_i = 3200
    b1, b2, b3, b4 = 1500, 1600, 1400, 1550
    pcb_temp = 325  # 32.5 deg C
    status = 0x0001

    c_raw = struct.pack("<HHHH4HhH", tether_v, tether_i, v5_v, v5_i, b1, b2, b3, b4, pcb_temp, status)
    decoded = msg.decode(c_raw)

    assert decoded["tether_voltage_mv"] == tether_v
    assert decoded["tether_current_ma"] == tether_i
    assert decoded["v5_voltage_mv"] == v5_v
    assert decoded["v12_brick1_current_ma"] == b1
    assert math.isclose(decoded["pcb_temp_c"], 32.5, abs_tol=1e-2)
    assert decoded["status_flags"] == status


if __name__ == "__main__":
    test_dbc_loads()
    test_thruster_cmd_fidelity()
    test_emergency_break_fidelity()
    test_nav_telemetry_fidelity()
    test_env_telemetry_fidelity()
    test_power_telemetry_fidelity()
    print("All DBC fidelity tests passed successfully!")
