"""Purdue ROV CLI - Serial Terminal Monitor."""

from __future__ import annotations

import os
import sys
import time

try:
    import serial
except ImportError:
    serial = None


def open_monitor(port: str, baud: int, timeout: float | None = None) -> None:
    """Open and stream serial communication with hardware target."""
    if sys.platform == "darwin" and port.startswith("/dev/tty."):
        cu_candidate = port.replace("/dev/tty.", "/dev/cu.")
        if os.path.exists(cu_candidate):
            port = cu_candidate

    if serial is None:
        print("\033[1;33mNotice: 'pyserial' package is not installed.\033[0m")
        if sys.platform == "darwin" or sys.platform.startswith("linux"):
            print(f"You can use native terminal monitor: screen {port} {baud}")
            print("Or install pyserial: pip install pyserial\n")
        sys.exit("Error: 'pyserial' package is required for built-in serial monitor.")

    duration_info = f" (for {timeout}s)" if timeout else " (Ctrl+C to exit)"
    print(f"\033[1;32m=== Opening Serial Monitor on {port} @ {baud} baud{duration_info} ===\033[0m\n")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        # CRITICAL for ST-Link V2/V3 USB CDC on macOS, Linux, and Windows:
        # DTR (Data Terminal Ready) and RTS (Request To Send) MUST be asserted.
        # Otherwise, the ST-Link CDC firmware buffers data internally and never flushes over USB.
        ser.dtr = True
        ser.rts = True
        ser.reset_input_buffer()
    except (serial.SerialException, PermissionError) as e:
        err_msg = str(e)
        if "PermissionError" in err_msg or "Access is denied" in err_msg or "Resource temporarily unavailable" in err_msg:
            sys.exit(
                f"\033[1;31mError: Port '{port}' is busy or access is denied.\033[0m\n"
                f"Another program (such as esc-configurator.com in your browser, STM32CubeProgrammer, or another terminal) is currently holding this port.\n"
                f"Please disconnect or close the other program and re-run this command."
            )
        sys.exit(f"Failed to open port {port}: {e}")

    start_time = time.time()
    try:
        while True:
            if timeout is not None and (time.time() - start_time) >= timeout:
                break
            data = ser.read(1024)
            if data:
                sys.stdout.write(data.decode("utf-8", errors="replace"))
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("\n\033[1;33mMonitor session terminated by user.\033[0m")
    finally:
        ser.close()
