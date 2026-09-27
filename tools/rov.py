#!/usr/bin/env python3
"""
Purdue ROV Unified Developer Interface (rov CLI).
Automates multi-target embedded monorepo build, flash, test, and monitoring tasks.
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

# Python 3.11+ tomllib support
try:
    import tomllib
except ImportError:
    try:
        import tomli as tomllib
    except ImportError:
        sys.exit("Error: 'tomllib' or 'tomli' package is required for Python < 3.11.")

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

# Enable ANSI escape processing on Windows console
if sys.platform == "win32":
    os.system("")

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = REPO_ROOT / "rov.toml"


def load_config() -> dict:
    if not CONFIG_PATH.is_file():
        sys.exit(f"Error: Missing configuration file at {CONFIG_PATH}")
    with open(CONFIG_PATH, "rb") as f:
        return tomllib.load(f)


def find_stm32programmer_cli() -> str:
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
        Path("/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"),
    ]
    for c in candidates:
        if c.is_file():
            return str(c)

    sys.exit(
        "Error: 'STM32_Programmer_CLI' could not be found.\n"
        "Please ensure STM32CubeProgrammer or STM32CubeCLT is installed and added to PATH."
    )


def run_cmd(cmd: list[str], cwd: Path = REPO_ROOT) -> None:
    display = " ".join(cmd)
    print(f"\033[1;34m>>> Running:\033[0m {display}")
    result = subprocess.run(cmd, cwd=cwd)
    if result.returncode != 0:
        sys.exit(f"\033[1;31mCommand failed with exit code {result.returncode}\033[0m")


def build_target(node: str, board: str, profile: str = "debug") -> str:
    config = load_config()

    if node not in config["nodes"]:
        valid_nodes = ", ".join(config["nodes"].keys())
        sys.exit(f"Error: Unknown node '{node}'. Valid nodes: {valid_nodes}")

    if board not in config["boards"]:
        valid_boards = ", ".join(config["boards"].keys())
        sys.exit(f"Error: Unknown board '{board}'. Valid boards: {valid_boards}")

    node_cfg = config["nodes"][node]
    board_cfg = config["boards"][board]

    if board not in node_cfg.get("supported_boards", []):
        supported = ", ".join(node_cfg.get("supported_boards", []))
        sys.exit(f"Error: Node '{node}' is not configured for board '{board}'. Supported: {supported}")

    preset_name = board_cfg.get("preset", "arm-c542-debug")
    build_dir_rel = board_cfg.get("build_dir", f"build/{preset_name}")
    build_dir = REPO_ROOT / build_dir_rel

    target_key = f"target_{board}"
    binary_key = f"binary_{board}"

    target_name = node_cfg.get(target_key)
    binary_rel = node_cfg.get(binary_key)

    print(f"\033[1;32m=== Configuring & Building Node: {node} for Board: {board} ({profile}) ===\033[0m")

    # If build directory does not exist or CMakeCache is missing, run configure preset
    if not (build_dir / "CMakeCache.txt").exists():
        if board == "g474":
            run_cmd(["cmake", "--preset", preset_name], cwd=REPO_ROOT / "nodes/node2_control_board")
        else:
            run_cmd(["cmake", "--preset", preset_name], cwd=REPO_ROOT)

    # Build target
    if board == "g474":
        run_cmd(["cmake", "--build", str(build_dir)], cwd=REPO_ROOT)
    elif target_name:
        run_cmd(["cmake", "--build", str(build_dir), "--target", target_name], cwd=REPO_ROOT)
    else:
        run_cmd(["cmake", "--build", str(build_dir)], cwd=REPO_ROOT)

    if binary_rel:
        elf_path = REPO_ROOT / binary_rel
        if elf_path.is_file():
            return str(elf_path)

    found = list(build_dir.glob("**/*.elf"))
    if found:
        return str(found[0])

    sys.exit(f"Error: Could not locate built ELF binary for {node} on {board} in {build_dir}")


def get_stlink_probes() -> list[dict]:
    """Scan and enumerate all ST-Link probes connected via USB."""
    cli_path = find_stm32programmer_cli()
    try:
        res = subprocess.run(
            [cli_path, "-l"],
            capture_output=True,
            text=True,
            check=False
        )
    except Exception as e:
        sys.exit(f"Error executing STM32_Programmer_CLI: {e}")

    probes = []
    lines = res.stdout.splitlines()
    current_probe = {}
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


def match_serial_port(probe_sn: str | None = None) -> str | None:
    """Correlate an ST-Link probe serial number to its CDC-ACM Virtual COM Port."""
    if serial is None:
        return None

    ports = serial.tools.list_ports.comports()

    # 1. Match by ST-Link USB Serial Number substring
    if probe_sn:
        for port in ports:
            if port.vid == 0x0483 and port.serial_number:
                if probe_sn.lower() in port.serial_number.lower() or port.serial_number.lower() in probe_sn.lower():
                    return port.device

    # 2. Match any STMicroelectronics CDC USB device (VID: 0x0483)
    for port in ports:
        if port.vid == 0x0483:
            return port.device

    # 3. Match descriptions with STLink or USB Serial
    for port in ports:
        desc = (port.description or "").lower()
        if "stlink" in desc or "stm32" in desc or "virtual com" in desc:
            return port.device

    return None


def flash_binary(elf_path: str, probe_sn: str | None = None) -> str:
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
            choice = int(input("Select probe index: "))
            selected_sn = probes[choice]["sn"]

    cmd = [
        cli_path,
        "-c", "port=SWD", f"sn={selected_sn}", "mode=UR", "reset=HWrst",
        "-d", elf_path,
        "-v",
        "-rst"
    ]
    print(f"\033[1;32m=== Flashing Binary to Target via ST-Link ({selected_sn}) ===\033[0m")
    run_cmd(cmd)
    return selected_sn


def open_monitor(port: str, baud: int) -> None:
    if serial is None:
        sys.exit("Error: 'pyserial' package is not installed. Run 'pip install pyserial'.")

    print(f"\033[1;32m=== Opening Serial Monitor on {port} @ {baud} baud (Ctrl+C to exit) ===\033[0m\n")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"Failed to open port {port}: {e}")

    try:
        while True:
            data = ser.read(1024)
            if data:
                sys.stdout.write(data.decode("utf-8", errors="replace"))
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("\n\033[1;33mMonitor session terminated by user.\033[0m")
    finally:
        ser.close()


def main():
    config = load_config()
    default_node = config["workspace"].get("default_node", "pi_shield")
    default_board = config["workspace"].get("default_board", "f411")
    default_baud = config["workspace"].get("default_baud", 115200)

    parser = argparse.ArgumentParser(
        description="Purdue ROV Unified Embedded Developer Interface (rov CLI)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  rov build -n pi_shield -b f411      Build Node 1 Pi Shield for NUCLEO-F411
  rov flash -n pi_shield -b f411      Build and flash to connected dev board
  rov run -n pi_shield -b f411        Build, flash, and open live serial monitor
  rov monitor                         Auto-detect COM port and open serial monitor
  rov test                            Run 25 Host SIL CTest unit test suites
  rov devices                         List connected ST-Link probes and COM ports
"""
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    # 1. Build
    build_p = subparsers.add_parser("build", help="Build node firmware")
    build_p.add_argument("-n", "--node", default=default_node, help=f"Target node (default: {default_node})")
    build_p.add_argument("-b", "--board", default=default_board, help=f"Target board (default: {default_board})")
    build_p.add_argument("-p", "--profile", default="debug", choices=["debug", "release"])

    # 2. Flash
    flash_p = subparsers.add_parser("flash", help="Build and flash firmware to target")
    flash_p.add_argument("-n", "--node", default=default_node, help=f"Target node (default: {default_node})")
    flash_p.add_argument("-b", "--board", default=default_board, help=f"Target board (default: {default_board})")
    flash_p.add_argument("-s", "--serial-number", help="Target ST-Link serial number")
    flash_p.add_argument("-p", "--profile", default="debug", choices=["debug", "release"])

    # 3. Run (Build + Flash + Serial Monitor)
    run_p = subparsers.add_parser("run", help="Build, flash, and open live serial monitor")
    run_p.add_argument("-n", "--node", default=default_node, help=f"Target node (default: {default_node})")
    run_p.add_argument("-b", "--board", default=default_board, help=f"Target board (default: {default_board})")
    run_p.add_argument("-s", "--serial-number", help="Target ST-Link serial number")
    run_p.add_argument("--baud", type=int, default=default_baud, help=f"Serial baud rate (default: {default_baud})")
    run_p.add_argument("--port", help="Explicit serial COM port (auto-detected if omitted)")

    # 4. Monitor
    mon_p = subparsers.add_parser("monitor", help="Open serial monitor")
    mon_p.add_argument("--port", help="Serial port device path (auto-detected if omitted)")
    mon_p.add_argument("--baud", type=int, default=default_baud, help=f"Serial baud rate (default: {default_baud})")

    # 5. Test
    test_p = subparsers.add_parser("test", help="Execute Host SIL CTest test suites")
    test_p.add_argument("-R", "--regex", help="Filter test suite by name regex")

    # 6. Devices
    subparsers.add_parser("devices", help="List connected ST-Link probes and serial COM ports")

    args = parser.parse_args()

    if args.command == "build":
        elf = build_target(args.node, args.board, args.profile)
        print(f"\033[1;32m[SUCCESS] Built target binary:\033[0m {elf}")

    elif args.command == "flash":
        elf = build_target(args.node, args.board, args.profile)
        flash_binary(elf, args.serial_number)
        print(f"\033[1;32m[SUCCESS] Target flashed and verified successfully.\033[0m")

    elif args.command == "run":
        elf = build_target(args.node, args.board, "debug")
        sn = flash_binary(elf, args.serial_number)

        port = args.port or match_serial_port(sn)
        if not port:
            print("\033[1;33mWarning: Could not automatically detect serial port. Listing available ports:\033[0m")
            if serial:
                for p in serial.tools.list_ports.comports():
                    print(f"  {p.device}: {p.description}")
            sys.exit(1)
        open_monitor(port, args.baud)

    elif args.command == "monitor":
        port = args.port
        if not port:
            probes = get_stlink_probes()
            sn = probes[0]["sn"] if probes else None
            port = match_serial_port(sn)

        if not port:
            sys.exit("Error: No serial port could be determined. Use --port COMx to specify.")
        open_monitor(port, args.baud)

    elif args.command == "test":
        print("\033[1;32m=== Building & Executing Host SIL CTest Test Suites ===\033[0m")
        run_cmd(["cmake", "--preset", "sil-debug"], cwd=REPO_ROOT)
        run_cmd(["cmake", "--build", "--preset", "sil-debug"], cwd=REPO_ROOT)
        cmd = ["ctest", "--preset", "sil-debug"]
        if args.regex:
            cmd.extend(["-R", args.regex])
        run_cmd(cmd, cwd=REPO_ROOT)

    elif args.command == "devices":
        print("\n\033[1;36m=== Connected ST-Link Probes ===\033[0m")
        try:
            probes = get_stlink_probes()
            if probes:
                for idx, p in enumerate(probes):
                    print(f"  [{idx}] SN: {p['sn']} | Board: {p.get('board', 'Target')}")
            else:
                print("  No ST-Link probes detected.")
        except Exception as e:
            print(f"  Error enumerating probes: {e}")

        print("\n\033[1;36m=== Available Serial COM Ports ===\033[0m")
        if serial:
            ports = serial.tools.list_ports.comports()
            if ports:
                for p in ports:
                    vid_str = f"VID:{hex(p.vid)}" if p.vid else ""
                    print(f"  {p.device}: {p.description} {vid_str}")
            else:
                print("  No COM ports detected.")
        print()


if __name__ == "__main__":
    main()
