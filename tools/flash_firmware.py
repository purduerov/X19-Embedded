#!/usr/bin/env python3
"""Cross-platform flashing utility for X19 Embedded.

Locates STM32_Programmer_CLI (standalone or bundled in STM32CubeCLT) across
Windows, macOS, and Linux, and invokes it with proper connection arguments.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

# Add repo root to import check_prereqs helper
REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

try:
    from tools.check_prereqs import find_cube_programmer
except ImportError:
    find_cube_programmer = None


def main() -> int:
    parser = argparse.ArgumentParser(description="Flash STM32 firmware using STM32_Programmer_CLI")
    parser.add_argument("--port", default="SWD", choices=["SWD", "USB1"], help="Target connection interface (default: SWD)")
    parser.add_argument("--file", "-w", required=False, default="", help="Path to firmware image (.elf, .hex, .bin)")
    parser.add_argument("--read-only", action="store_true", help="Read back memory to verify connection without writing")
    args = parser.parse_args()

    cli = find_cube_programmer() if find_cube_programmer else None
    if cli is None:
        print("\nERROR: STM32_Programmer_CLI not found on system PATH or default install directories.")
        print("To flash hardware, install STM32CubeCLT (recommended) or STM32CubeProgrammer from ST:")
        print("  https://www.st.com/en/development-tools/stm32cubeclt.html")
        print("  https://www.st.com/en/development-tools/stm32cubeprog.html\n")
        return 1

    if args.read_only:
        cmd = [str(cli), "-c", f"port={args.port}", "-r32", "0x08000000", "16"]
    else:
        if not args.file:
            print("\nERROR: No firmware file specified. Use --file <path> to specify the firmware image.\n")
            return 2
        file_path = Path(args.file).resolve()
        if not file_path.exists():
            print(f"\nERROR: Firmware file not found: {file_path}")
            print("Please build the target firmware before flashing.\n")
            return 2
        cmd = [str(cli), "-c", f"port={args.port}", "-w", str(file_path), "-v", "-rst"]

    print(f"Invoking programmer: {cli}")
    print(f"Executing: {' '.join(cmd)}")
    return subprocess.run(cmd).returncode


if __name__ == "__main__":
    raise SystemExit(main())
