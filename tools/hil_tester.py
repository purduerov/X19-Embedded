#!/usr/bin/env python3
"""
Purdue ROV Hardware-In-the-Loop (HIL) Automated CAN FD Test Bench.
Executes physical bus timing and telemetry checks against live STM32 nodes.
Purdue ROV 2026-2027.
"""

import argparse
import struct
import sys
import time

try:
    import can
except ImportError:
    can = None

CAN_ID_THRUSTER_CMD = 0x100
CAN_ID_NAV_TELEMETRY = 0x200


def _listen_for_nav(bus, duration_sec: float) -> int:
    deadline = time.monotonic() + duration_sec
    count = 0
    while time.monotonic() < deadline:
        try:
            msg = bus.recv(timeout=min(0.05, max(0.0, deadline - time.monotonic())))
        except Exception as exc:
            print(f"[FAIL] CAN receive error while sampling navigation telemetry: {exc}")
            return count
        if msg is not None and msg.arbitration_id == CAN_ID_NAV_TELEMETRY:
            count += 1
    return count


def run_hil_test(interface: str) -> bool:
    if can is None:
        print("[FAIL] python-can is not installed. Run 'pip install python-can'.")
        return False

    print("==================================================")
    print("   PURDUE ROV SUBSEA EMBEDDED HIL TEST BENCH RUNNER")
    print(f"   Interface: {interface} (CAN FD @ 1M/5M)")
    print("==================================================")

    try:
        bus = can.Bus(channel=interface, interface="socketcan", fd=True)
    except Exception as exc:
        print(f"[FAIL] Could not connect to CAN interface {interface}: {exc}")
        return False

    nav_ok = False
    command_tx_ok = False
    watchdog_ok = False
    periodic_task = None
    try:
        print("\n[TEST 1] Listening for 100 Hz Nav Telemetry (0x200) from Control Board...")
        nav_count = _listen_for_nav(bus, 2.0)
        frequency_hz = nav_count / 2.0
        print(f"  Received {nav_count} packets in 2.0s (~{frequency_hz:.1f} Hz)")
        nav_ok = frequency_hz >= 80.0
        if nav_ok:
            print("  [PASS] Navigation telemetry stream is within frequency tolerance.")
        else:
            print("  [FAIL] Navigation telemetry frequency is below the 80 Hz minimum.")

        print("\n[TEST 2] Injecting 8-Channel Thruster PWMs (0x100) at 100 Hz...")
        test_pwm = [1550, 1550, 1550, 1550, 1450, 1450, 1450, 1450]
        payload = struct.pack("<8H", *test_pwm)
        message = can.Message(
            arbitration_id=CAN_ID_THRUSTER_CMD,
            data=payload,
            is_extended_id=False,
            is_fd=True,
        )
        try:
            periodic_task = bus.send_periodic(message, 0.01)
            if periodic_task is None or not hasattr(periodic_task, "stop"):
                print("  [FAIL] CAN backend did not return a controllable periodic task.")
            else:
                time.sleep(0.50)
                command_tx_ok = True
                print("  [PASS] Submitted 50 PWM command frames without a transmit error.")
        except Exception as exc:
            print(f"  [FAIL] PWM command transmission failed: {exc}")
        finally:
            if periodic_task is not None and hasattr(periodic_task, "stop"):
                try:
                    periodic_task.stop()
                except Exception as exc:
                    command_tx_ok = False
                    print(f"  [FAIL] Could not stop PWM command task: {exc}")

        print("\n[TEST 3] Testing heartbeat watchdog timeout (> 100ms silence)...")
        print("  [FAIL] The current vehicle CAN protocol exposes no actuator/PWM readback frame.")
        print("         A watchdog trip cannot be proven from CAN telemetry alone; add a")
        print("         hardware-observable PWM/status frame before marking HIL successful.")

        passed = nav_ok and command_tx_ok and watchdog_ok
        print("\n==================================================")
        if passed:
            print("   HIL TEST SUITE COMPLETED SUCCESSFULLY")
        else:
            print("   HIL TEST SUITE FAILED")
        print("==================================================")
        return passed
    finally:
        try:
            bus.shutdown()
        except Exception as exc:
            print(f"[WARN] Failed to close CAN interface {interface}: {exc}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Purdue ROV Embedded HIL Test Runner")
    parser.add_argument("--interface", default="can0", help="CAN interface (default: can0)")
    args = parser.parse_args()
    return 0 if run_hil_test(args.interface) else 1


if __name__ == "__main__":
    sys.exit(main())
