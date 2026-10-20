"""Purdue ROV CLI - SWD Firmware Flasher via ST-Link."""

from __future__ import annotations

import sys

from .config import run_cmd
from .discovery import find_stm32programmer_cli, get_stlink_probes


def flash_binary(elf_path: str, probe_sn: str | None = None) -> str:
    """Flash an ELF binary to target microcontroller via ST-Link programmer."""
    cli_path = find_stm32programmer_cli()
    probes = get_stlink_probes()

    if not probes:
        sys.exit(
            "\033[1;31mError: No ST-Link debug probes detected via USB.\033[0m\n"
            "Please verify your Nucleo board or ST-Link is plugged in and drivers are installed."
        )

    selected_sn = probe_sn
    if not selected_sn:
        if len(probes) == 1:
            selected_sn = probes[0]["sn"]
            board_name = probes[0].get("board", "ST-Link Target")
            print(f"\033[1;36mAuto-selected ST-Link SN: {selected_sn} ({board_name})\033[0m")
        else:
            print("\nMultiple ST-Link probes detected:")
            for idx, p in enumerate(probes):
                print(f"  [{idx}] SN: {p['sn']} ({p.get('board', 'Unknown Target')})")
            while True:
                try:
                    choice_str = input("Select probe index: ").strip()
                    choice = int(choice_str)
                    if 0 <= choice < len(probes):
                        selected_sn = probes[choice]["sn"]
                        break
                    print(f"Error: Invalid index. Must be between 0 and {len(probes) - 1}.")
                except (ValueError, EOFError, KeyboardInterrupt):
                    sys.exit("\nOperation cancelled by user.")

    cmd = [
        cli_path,  # type: ignore
        "-c", "port=SWD", f"sn={selected_sn}", "mode=UR", "reset=HWrst",
        "-d", elf_path,
        "-v",
        "-rst"
    ]
    print(f"\033[1;32m=== Flashing Binary to Target via ST-Link ({selected_sn}) ===\033[0m")
    run_cmd(cmd)
    return selected_sn
