"""Purdue ROV CLI - Hardware, Probes, & Toolchain Discovery."""

from __future__ import annotations

import glob
import os
import shutil
import subprocess
import sys
from pathlib import Path

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


def find_stm32programmer_cli(required: bool = True) -> str | None:
    """Find STM32_Programmer_CLI executable across system PATH and standard ST directories."""
    found = shutil.which("STM32_Programmer_CLI") or shutil.which("STM32_Programmer_CLI.exe")
    if found:
        return found

    candidates = [
        Path(r"C:\ST\STM32CubeCLT_1.22.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"),
        Path(r"C:\ST\STM32CubeCLT\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"),
        Path(r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"),
        Path(r"C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"),
        Path("/opt/st/stm32cubeclt/STM32CubeProgrammer/bin/STM32_Programmer_CLI"),
        Path("/opt/ST/STM32CubeCLT/STM32CubeProgrammer/bin/STM32_Programmer_CLI"),
        Path("/opt/st/stm32cubeprogrammer/bin/STM32_Programmer_CLI"),
        Path("/usr/local/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"),
        Path("/usr/local/bin/STM32_Programmer_CLI"),
        Path("/opt/homebrew/bin/STM32_Programmer_CLI"),
        Path("/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI"),
        Path("/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOS/bin/STM32_Programmer_CLI"),
        Path("/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/STM32_Programmer_CLI"),
        Path("/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOS/STM32_Programmer_CLI"),
        Path("/Applications/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI"),
        Path("/Applications/STM32CubeProgrammer.app/Contents/MacOS/bin/STM32_Programmer_CLI"),
        Path("/Applications/STM32CubeProgrammer.app/Contents/MacOs/STM32_Programmer_CLI"),
        Path("/Applications/STM32CubeProgrammer.app/Contents/MacOS/STM32_Programmer_CLI"),
        Path("/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"),
    ]
    for c in candidates:
        if c.is_file():
            return str(c)

    if required:
        sys.exit(
            "Error: 'STM32_Programmer_CLI' could not be found.\n"
            "Please ensure STM32CubeProgrammer or STM32CubeCLT is installed and added to PATH."
        )
    return None


def find_cubemx(required: bool = True) -> str | None:
    """Find STM32CubeMX executable across system PATH and standard ST installation directories."""
    found = shutil.which("STM32CubeMX") or shutil.which("STM32CubeMX.exe")
    if found:
        return found

    candidates = [
        Path(os.environ.get("LOCALAPPDATA", "")) / "Programs/STM32CubeMX/STM32CubeMX.exe",
        Path(r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeMX\STM32CubeMX.exe"),
        Path(r"C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeMX\STM32CubeMX.exe"),
        Path("/Applications/STMicroelectronics/STM32CubeMX.app/Contents/MacOs/STM32CubeMX"),
        Path("/Applications/STM32CubeMX.app/Contents/MacOs/STM32CubeMX"),
        Path("/usr/local/STMicroelectronics/STM32CubeMX/STM32CubeMX"),
        Path("/opt/STMicroelectronics/STM32CubeMX/STM32CubeMX"),
    ]
    for c in candidates:
        if c.is_file():
            return str(c)

    if required:
        sys.exit(
            "Error: 'STM32CubeMX' could not be found.\n"
            "Please ensure STM32CubeMX is installed."
        )
    return None


def get_stlink_probes() -> list[dict]:
    """Scan and enumerate all ST-Link probes connected via USB."""
    cli_path = find_stm32programmer_cli(required=False)
    if not cli_path:
        return []
    try:
        res = subprocess.run(
            [cli_path, "-l"],
            capture_output=True,
            text=True,
            check=False,
        )
    except Exception:
        return []

    probes = []
    lines = res.stdout.splitlines()
    current_probe: dict[str, str] = {}
    for line in lines:
        line = line.strip()
        if line.startswith("ST-Link Probe"):
            if "sn" in current_probe:
                probes.append(current_probe)
            current_probe = {}
        elif "ST-LINK SN" in line:
            parts = line.split(":")
            if len(parts) >= 2:
                current_probe["sn"] = parts[1].strip()
        elif "Board Name" in line:
            parts = line.split(":")
            if len(parts) >= 2:
                current_probe["board"] = parts[1].strip()

    if "sn" in current_probe:
        probes.append(current_probe)
    return probes


def detect_connected_board(probes: list[dict] | None = None) -> tuple[str | None, str | None, str | None]:
    """Detect board key (e.g. g474, f411) from connected ST-Link hardware probes.

    Returns (board_key, probe_sn, board_name).
    """
    if probes is None:
        probes = get_stlink_probes()
    if not probes:
        return None, None, None
    board_str = probes[0].get("board", "").upper()
    sn = probes[0].get("sn")
    raw_name = probes[0].get("board", "Unknown Target")
    if "G474" in board_str:
        return "g474", sn, raw_name
    elif "F411" in board_str:
        return "f411", sn, raw_name
    elif "F446" in board_str:
        return "f446", sn, raw_name
    elif "F401" in board_str:
        return "f401", sn, raw_name
    elif "G431" in board_str:
        return "g431", sn, raw_name
    elif "C542" in board_str or "C5" in board_str:
        return "stm32c5", sn, raw_name
    return None, sn, raw_name


def match_serial_port(probe_sn: str | None = None) -> str | None:
    """Correlate an ST-Link probe serial number to its CDC-ACM Virtual COM Port."""
    if sys.platform == "darwin":
        cu_ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*"))
        if cu_ports:
            if probe_sn:
                for cp in cu_ports:
                    if probe_sn.lower() in cp.lower():
                        return cp
            if not probe_sn:
                return cu_ports[0]

    if serial is None:
        if sys.platform == "darwin":
            cu = glob.glob("/dev/cu.usbmodem*")
            return cu[0] if cu else None
        return None

    ports = list(serial.tools.list_ports.comports())

    # On macOS, /dev/tty.* blocks waiting for DCD; remap to /dev/cu.*
    for p in ports:
        if sys.platform == "darwin" and p.device.startswith("/dev/tty."):
            cu_candidate = p.device.replace("/dev/tty.", "/dev/cu.")
            if os.path.exists(cu_candidate):
                p.device = cu_candidate

    # 1. Match by ST-Link USB Serial Number substring
    if probe_sn:
        for port in ports:
            if port.serial_number and (probe_sn.lower() in port.serial_number.lower() or port.serial_number.lower() in probe_sn.lower()):
                return port.device

    # 2. Match any STMicroelectronics CDC USB device (VID: 0x0483)
    for port in ports:
        if getattr(port, "vid", None) == 0x0483:
            return port.device

    # 3. Match descriptions or device paths containing STLink, STM32, or usbmodem (macOS)
    for port in ports:
        desc = (port.description or "").lower()
        dev = (port.device or "").lower()
        if "stlink" in desc or "stm32" in desc or "virtual com" in desc or "usbmodem" in desc or "usbmodem" in dev:
            return port.device

    # 4. Fallback for macOS / Linux CDC devices
    for port in ports:
        dev = (port.device or "").lower()
        if "/dev/cu.usb" in dev or "/dev/ttyacm" in dev:
            return port.device

    # 5. Linux /dev/ttyACM* glob fallback
    if sys.platform.startswith("linux"):
        acm_ports = sorted(glob.glob("/dev/ttyACM*"))
        if acm_ports:
            return acm_ports[0]

    return None
