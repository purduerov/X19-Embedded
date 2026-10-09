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
import time
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
    msys_bin = Path(r"C:\msys64\ucrt64\bin")
    if msys_bin.is_dir() and str(msys_bin) not in os.environ.get("PATH", ""):
        os.environ["PATH"] = f"{msys_bin};{os.environ.get('PATH', '')}"


REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = REPO_ROOT / "rov.toml"


def load_config() -> dict:
    if not CONFIG_PATH.is_file():
        sys.exit(f"Error: Missing configuration file at {CONFIG_PATH}")
    with open(CONFIG_PATH, "rb") as f:
        return tomllib.load(f)


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


def generate_cubemx(target: str = "all") -> None:
    cubemx = find_cubemx(required=True)
    ioc_map = {
        "g474": REPO_ROOT / "boards/g474_nucleo/g474_nucleo.ioc",
        "f411": REPO_ROOT / "boards/f411_nucleo/f411_nucleo.ioc",
        "pi_shield": REPO_ROOT / "nodes/node1_pi_shield/node1_pi_shield.ioc",
        "control_board": REPO_ROOT / "nodes/node2_control_board/node2_control_board.ioc",
        "power_slab": REPO_ROOT / "nodes/node3_power_slab/node3_power_slab.ioc",
        "testing_and_rnd": REPO_ROOT / "nodes/testing_and_rnd/testing_and_rnd.ioc",
    }

    targets = list(ioc_map.keys()) if target in ("all", None) else [target]
    for t in targets:
        ioc_path = ioc_map.get(t)
        if not ioc_path or not ioc_path.is_file():
            print(f"Skipping unknown or missing target: {t}")
            continue
        print(f"\033[1;36m=== Regenerating CubeMX Project for {t} ({ioc_path.name}) ===\033[0m")
        script_file = REPO_ROOT / f".cubemx_gen_{t}.tmp"
        try:
            # STM32CubeMX CLI accepts script commands
            script_file.write_text(f"config load {ioc_path.resolve()}\nproject generate\nexit\n", encoding="utf-8")
            run_cmd([cubemx, "-q", str(script_file.resolve())], cwd=ioc_path.parent)
            print(f"\033[1;32m[SUCCESS] Regenerated {t}\033[0m\n")
        finally:
            if script_file.is_file():
                script_file.unlink()


def run_cmd(cmd: list[str], cwd: Path = REPO_ROOT) -> None:
    display = " ".join(cmd)
    print(f"\033[1;34m>>> Running:\033[0m {display}")
    env = os.environ.copy()
    msys_bin = Path("C:/msys64/ucrt64/bin")
    if msys_bin.is_dir():
        env["PATH"] = str(msys_bin) + os.pathsep + env.get("PATH", "")
    result = subprocess.run(cmd, cwd=cwd, env=env)
    if result.returncode != 0:
        sys.exit(f"\033[1;31mCommand failed with exit code {result.returncode}\033[0m")


def build_target(node: str, board: str, profile: str = "debug", custom_file: str | None = None) -> str:
    config = load_config()

    file_path = None
    if custom_file or node == "sandbox":
        node = "sandbox"
        file_path = (Path(custom_file).resolve() if custom_file else (REPO_ROOT / "sandbox/sandbox.c")).as_posix()
        if not Path(file_path).is_file():
            sys.exit(f"Error: Specified scratch source file does not exist: {file_path}")

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

    display_source = f" (source: {file_path})" if file_path else ""
    print(f"\033[1;32m=== Configuring & Building Node: {node} for Board: {board} ({profile}){display_source} ===\033[0m")

    # If building sandbox target, re-configure CMake with the active SANDBOX_FILE definition
    if node == "sandbox" and file_path:
        cmake_cfg = ["cmake"]
        if board == "host":
            cmake_cfg.extend(["--preset", "sil-debug"])
        else:
            cmake_cfg.extend(["-S", str(REPO_ROOT), "-B", str(build_dir)])
        cmake_cfg.append(f"-DSANDBOX_FILE={file_path}")
        run_cmd(cmake_cfg, cwd=REPO_ROOT)
    elif not (build_dir / "CMakeCache.txt").exists():
        run_cmd(["cmake", "--preset", preset_name], cwd=REPO_ROOT)

    # Build target
    cmd = ["cmake", "--build", str(build_dir)]
    if target_name:
        cmd.extend(["--target", target_name])
    run_cmd(cmd, cwd=REPO_ROOT)

    if binary_rel:
        elf_path = REPO_ROOT / binary_rel
        if elf_path.is_file():
            return str(elf_path)

    found = list(build_dir.glob("**/*.elf")) or list(build_dir.glob("**/*.exe"))
    if found:
        return str(found[0])

    sys.exit(f"Error: Could not locate built binary for {node} on {board} in {build_dir}")


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
            check=False
        )
    except Exception:
        return []

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
    # 0. Check native macOS /dev/cu.usbmodem* devices directly
    if sys.platform == "darwin":
        import glob
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
            import glob
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
        import glob
        acm_ports = sorted(glob.glob("/dev/ttyACM*"))
        if acm_ports:
            return acm_ports[0]

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


def open_monitor(port: str, baud: int, timeout: float | None = None) -> None:
    if sys.platform == "darwin" and port.startswith("/dev/tty."):
        cu_candidate = port.replace("/dev/tty.", "/dev/cu.")
        if os.path.exists(cu_candidate):
            port = cu_candidate

    if serial is None:
        print("\033[1;33mNotice: 'pyserial' package is not installed.\033[0m")
        if sys.platform == "darwin" or sys.platform.startswith("linux"):
            print(f"You can use native terminal monitor: screen {port} {baud}")
            print(f"Or install pyserial: pip install pyserial\n")
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
  rov scan -b f411                    Build, flash & stream I2C scanner on NUCLEO-F411
  rov scan -b g474                    Build, flash & stream I2C scanner on NUCLEO-G474
  rov sandbox -b f411                 Build, flash & monitor sandbox/sandbox.c on F411
  rov sandbox -b g474                 Build, flash & monitor sandbox/sandbox.c on G474
  rov sandbox -b host                 Run sandbox/sandbox.c natively on Host SIL
  rov sandbox -f my_test.c -b f411    Run custom scratch C file on NUCLEO-F411
  rov build -n pi_shield -b f411      Build Node 1 Pi Shield for NUCLEO-F411
  rov flash -n pi_shield -b f411      Build and flash Node 1 to connected dev board
  rov monitor                         Auto-detect COM port and open serial monitor
  rov test                            Run 25 Host SIL CTest unit test suites
  rov devices                         List connected ST-Link probes and COM ports
  rov check                           Verify toolchain, compiler, and programmer prerequisites
"""
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    # 1. Scan (Zero-code automated I2C bus scanner)
    scan_p = subparsers.add_parser("scan", help="Build, flash, and monitor zero-code I2C bus scanner on connected dev board")
    scan_p.add_argument("-b", "--board", default=None, help=f"Target dev board (f411, f446, f401, g474, g431) (auto-detected if omitted, default: {default_board})")
    scan_p.add_argument("-s", "--serial-number", help="Target ST-Link serial number")
    scan_p.add_argument("--baud", type=int, default=default_baud, help=f"Serial baud rate (default: {default_baud})")
    scan_p.add_argument("--port", help="Explicit serial COM port (auto-detected if omitted)")

    # 2. Sandbox (Zero-boilerplate developer scratchpad / single-file test)
    sandbox_p = subparsers.add_parser("sandbox", help="Build, flash, and monitor developer sandbox or custom scratch C file")
    sandbox_p.add_argument("-b", "--board", default=None, help=f"Target board (f411, f446, f401, g474, g431, host) (auto-detected if omitted, default: {default_board})")
    sandbox_p.add_argument("-f", "--file", help="Custom scratch C file containing app_main() (default: sandbox/sandbox.c)")
    sandbox_p.add_argument("-s", "--serial-number", help="Target ST-Link serial number")
    sandbox_p.add_argument("--baud", type=int, default=default_baud, help=f"Serial baud rate (default: {default_baud})")
    sandbox_p.add_argument("--port", help="Explicit serial COM port (auto-detected if omitted)")
    sandbox_p.add_argument("-t", "--timeout", type=float, default=None, help="Monitor timeout in seconds (auto-exits after duration)")

    # 3. Build
    build_p = subparsers.add_parser("build", help="Build node or sandbox firmware")
    build_p.add_argument("-n", "--node", default=None, help=f"Target node (auto-selected per board if omitted, default: {default_node})")
    build_p.add_argument("-b", "--board", default=None, help=f"Target board (auto-detected if omitted, default: {default_board})")
    build_p.add_argument("-f", "--file", help="Custom scratch C file containing app_main() to build in sandbox")
    build_p.add_argument("-p", "--profile", default="debug", choices=["debug", "release"])

    # 3. Flash
    flash_p = subparsers.add_parser("flash", help="Build and flash firmware to target")
    flash_p.add_argument("-n", "--node", default=None, help=f"Target node (auto-selected per board if omitted, default: {default_node})")
    flash_p.add_argument("-b", "--board", default=None, help=f"Target board (auto-detected if omitted, default: {default_board})")
    flash_p.add_argument("-f", "--file", help="Custom scratch C file containing app_main() to flash in sandbox")
    flash_p.add_argument("-s", "--serial-number", help="Target ST-Link serial number")
    flash_p.add_argument("-p", "--profile", default="debug", choices=["debug", "release"])

    # 4. Run (Build + Flash + Serial Monitor / Host Execution)
    run_p = subparsers.add_parser("run", help="Build, flash, and open live serial monitor (or run on host)")
    run_p.add_argument("-n", "--node", default=None, help=f"Target node (auto-selected per board if omitted, default: {default_node})")
    run_p.add_argument("-b", "--board", default=None, help=f"Target board (auto-detected if omitted, default: {default_board})")
    run_p.add_argument("-f", "--file", help="Custom scratch C file containing app_main() to run in sandbox")
    run_p.add_argument("-s", "--serial-number", help="Target ST-Link serial number")
    run_p.add_argument("--baud", type=int, default=default_baud, help=f"Serial baud rate (default: {default_baud})")
    run_p.add_argument("--port", help="Explicit serial COM port (auto-detected if omitted)")
    run_p.add_argument("-t", "--timeout", type=float, default=None, help="Monitor timeout in seconds (auto-exits after duration)")

    # 5. Monitor
    mon_p = subparsers.add_parser("monitor", help="Open serial monitor")
    mon_p.add_argument("--port", help="Serial port device path (auto-detected if omitted)")
    mon_p.add_argument("--baud", type=int, default=default_baud, help=f"Serial baud rate (default: {default_baud})")
    mon_p.add_argument("-t", "--timeout", type=float, default=None, help="Monitor timeout in seconds (auto-exits after duration)")

    # 6. Test
    test_p = subparsers.add_parser("test", help="Execute Host SIL CTest test suites")
    test_p.add_argument("-R", "--regex", help="Filter test suite by name regex")
    test_p.add_argument("--node", choices=["pi_shield", "control_board", "power_slab", "all"],
                        help="Only run tests labelled node:<node>")
    test_p.add_argument("--suite", choices=["app", "driver", "shared", "integration", "safety", "power",
                                            "fuzz", "burnin", "bsp", "services"],
                        help="Only run tests labelled suite:<suite>")
    test_p.add_argument("-k", "--filter",
                        help="Unity case-name substring, applied via the ROV_TEST_FILTER env var")
    test_p.add_argument("--target", action="store_true",
                        help="Run the target_* readiness contracts instead of the host SIL suite")
    test_p.add_argument("--preset", default=None, help="Optional CMakePresets build preset")
    test_p.add_argument("--dry-run", action="store_true", help="Print the resolved commands without running them")

    # 7. Devices
    subparsers.add_parser("devices", help="List connected ST-Link probes and serial COM ports")

    # 8. Check
    subparsers.add_parser("check", help="Verify build tools, ARM cross-compiler, flashing CLI, and Python dependencies")

    # 9. Generate (STM32CubeMX CLI)
    gen_p = subparsers.add_parser("generate", help="Regenerate peripheral drivers from .ioc files via STM32CubeMX CLI")
    gen_p.add_argument(
        "-t", "--target",
        choices=["all", "g474", "f411", "pi_shield", "control_board", "power_slab", "testing_and_rnd"],
        default="all",
        help="Target board or node to regenerate (default: all)"
    )

    args = parser.parse_args()

    # Intelligent Auto-Detection of Board and Node
    if hasattr(args, "board"):
        if args.board is None:
            detected_board, detected_sn, board_name = detect_connected_board()
            if detected_board:
                args.board = detected_board
                print(f"\033[1;36mAuto-detected connected dev board: {args.board} ({board_name})\033[0m")
            else:
                args.board = default_board

    if hasattr(args, "node"):
        if args.node is None:
            if getattr(args, "board", None) in ("g474", "g431"):
                args.node = "control_board"
            elif getattr(args, "board", None) in ("f411", "f446", "f401"):
                args.node = "pi_shield"
            else:
                args.node = default_node

    if args.command == "scan":
        print(f"\033[1;36m=== Preparing Hardware I2C Diagnostic Bus Scanner for {args.board} ===\033[0m")
        elf = build_target("rnd", args.board, "debug")
        sn = flash_binary(elf, args.serial_number)
        port = args.port or match_serial_port(sn)
        if not port:
            print("\033[1;33mWarning: Could not automatically detect serial port. Listing available ports:\033[0m")
            if serial:
                for p in serial.tools.list_ports.comports():
                    print(f"  {p.device}: {p.description}")
            sys.exit(1)
        open_monitor(port, args.baud)

    elif args.command == "sandbox":
        elf = build_target("sandbox", args.board, "debug", getattr(args, "file", None))
        if args.board == "host":
            print(f"\033[1;32m=== Executing Host SIL Sandbox ({elf}) ===\033[0m\n")
            run_cmd([elf], cwd=REPO_ROOT)
            return

        sn = flash_binary(elf, args.serial_number)
        port = args.port or match_serial_port(sn)
        if not port:
            print("\033[1;33mWarning: Could not automatically detect serial port. Listing available ports:\033[0m")
            if serial:
                for p in serial.tools.list_ports.comports():
                    print(f"  {p.device}: {p.description}")
            sys.exit(1)
        open_monitor(port, args.baud, getattr(args, "timeout", None))

    elif args.command == "build":
        elf = build_target(args.node, args.board, args.profile, getattr(args, "file", None))
        print(f"\033[1;32m[SUCCESS] Built target binary:\033[0m {elf}")

    elif args.command == "flash":
        elf = build_target(args.node, args.board, args.profile, getattr(args, "file", None))
        flash_binary(elf, args.serial_number)
        print(f"\033[1;32m[SUCCESS] Target flashed and verified successfully.\033[0m")

    elif args.command == "run":
        elf = build_target(args.node, args.board, "debug", getattr(args, "file", None))
        if args.board == "host":
            print(f"\033[1;32m=== Executing Host SIL Sandbox ({elf}) ===\033[0m\n")
            run_cmd([elf], cwd=REPO_ROOT)
            return

        sn = flash_binary(elf, args.serial_number)
        port = args.port or match_serial_port(sn)
        if not port:
            print("\033[1;33mWarning: Could not automatically detect serial port. Listing available ports:\033[0m")
            if serial:
                for p in serial.tools.list_ports.comports():
                    print(f"  {p.device}: {p.description}")
            sys.exit(1)
        open_monitor(port, args.baud, getattr(args, "timeout", None))

    elif args.command == "monitor":
        port = args.port
        if not port:
            probes = get_stlink_probes()
            sn = probes[0]["sn"] if probes else None
            port = match_serial_port(sn)

        if not port:
            print("\033[1;31mError: No serial port could be determined automatically.\033[0m")
            if sys.platform == "darwin":
                import glob
                modems = glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*")
                if modems:
                    print(f"Found candidate USB modem ports: {modems}")
            if serial:
                ports = list(serial.tools.list_ports.comports())
                if ports:
                    print("Available serial ports:")
                    for p in ports:
                        print(f"  {p.device}: {p.description}")
            sys.exit("Please specify the port explicitly with: --port <port>")
        open_monitor(port, args.baud, getattr(args, "timeout", None))

    elif args.command == "test":
        build_dir = "build-native"
        cmd = ["ctest", "--test-dir", build_dir, "--output-on-failure"]
        if args.regex:
            cmd += ["-R", args.regex]
        if args.node and args.node != "all":
            cmd += ["-L", f"node:{args.node}"]
        if args.suite:
            cmd += ["-L", f"suite:{args.suite}"]
        if args.target:
            cmd += ["-R", "^target_"]
        else:
            cmd += ["-E", "^target_"]

        if args.filter:
            os.environ["ROV_TEST_FILTER"] = args.filter

        if args.preset:
            build_cmds = [["cmake", "--preset", args.preset], ["cmake", "--build", "--preset", args.preset]]
        else:
            build_cmds = [["cmake", "-B", build_dir, "-G", "Ninja"], ["cmake", "--build", build_dir]]

        if args.dry_run:
            for c in build_cmds:
                print(" ".join(c))
            print(" ".join(cmd))
            if args.filter:
                print(f"(env ROV_TEST_FILTER={args.filter})")
            return

        print("\033[1;32m=== Building & Executing Host SIL CTest Test Suites ===\033[0m")
        for c in build_cmds:
            run_cmd(c, cwd=REPO_ROOT)
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
        if sys.platform == "darwin":
            import glob
            mac_ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*"))
            for mp in mac_ports:
                print(f"  {mp} (macOS USB CDC)")
        if serial:
            ports = list(serial.tools.list_ports.comports())
            if ports:
                for p in ports:
                    vid_str = f"VID:{hex(p.vid)}" if getattr(p, "vid", None) else ""
                    print(f"  {p.device}: {p.description} {vid_str}")
            elif sys.platform != "darwin":
                print("  No COM ports detected.")
        print()

    elif args.command == "check":
        sys.path.insert(0, str(REPO_ROOT))
        from tools.check_prereqs import main as run_check_prereqs
        sys.exit(run_check_prereqs())

    elif args.command == "generate":
        generate_cubemx(args.target)


if __name__ == "__main__":
    main()
