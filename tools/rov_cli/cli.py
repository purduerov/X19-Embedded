"""Purdue ROV Unified Developer Interface (rov CLI) - Main Dispatcher."""

from __future__ import annotations

import argparse
import os
import sys

from .build import build_target, generate_cubemx
from .config import REPO_ROOT, load_config, resolve_target, run_cmd
from .discovery import (
    detect_connected_board,
    get_stlink_probes,
    match_serial_port,
    serial,
)
from .flash import flash_binary
from .monitor import open_monitor


def main() -> None:
    config = load_config()
    default_node = config["workspace"].get("default_node", "pi_shield")
    default_board = config["workspace"].get("default_board", "f411")
    default_baud = config["workspace"].get("default_baud", 115200)

    parser = argparse.ArgumentParser(
        description="Purdue ROV Unified Embedded Developer Interface (rov CLI)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  rov run multinode_can.c -b f411         Build, flash & monitor scratch C file on F411
  rov run control_board -b g474           Build, flash & monitor Node 2 on G474
  rov run -b f411                         Build, flash & monitor default node on F411
  rov build my_code.c -b f411             Build custom C file on NUCLEO-F411
  rov build control_board -b g474         Build Node 2 for NUCLEO-G474
  rov flash control_board -b g474         Build and flash Node 2
  rov monitor                             Auto-detect COM port and open serial monitor
  rov test                                Run Host SIL CTest test suites
  rov devices                             List connected ST-Link probes and COM ports
  rov generate                            Regenerate peripheral drivers from .ioc files
""",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    # 1. Run (Build + Flash + Serial Monitor / Host Execution)
    run_p = subparsers.add_parser("run", help="Build, flash, and open live serial monitor (or run on host)")
    run_p.add_argument("target", nargs="?", default=None, help="Target C source file (e.g. my_code.c) or node name (e.g. control_board). Defaults to board default node if omitted.")
    run_p.add_argument("-n", "--node", default=None, help=f"Target node (auto-selected per board if omitted, default: {default_node})")
    run_p.add_argument("-b", "--board", default=None, help=f"Target board (auto-detected if omitted, default: {default_board})")
    run_p.add_argument("-f", "--file", help="Custom scratch C file containing app_main() to run")
    run_p.add_argument("-s", "--serial-number", help="Target ST-Link serial number")
    run_p.add_argument("--baud", type=int, default=default_baud, help=f"Serial baud rate (default: {default_baud})")
    run_p.add_argument("--port", help="Explicit serial COM port (auto-detected if omitted)")
    run_p.add_argument("-t", "--timeout", type=float, default=None, help="Monitor timeout in seconds (auto-exits after duration)")

    # 2. Build
    build_p = subparsers.add_parser("build", help="Build node or target firmware")
    build_p.add_argument("target", nargs="?", default=None, help="Target C source file (e.g. my_code.c) or node name (e.g. control_board). Defaults to board default node if omitted.")
    build_p.add_argument("-n", "--node", default=None, help=f"Target node (auto-selected per board if omitted, default: {default_node})")
    build_p.add_argument("-b", "--board", default=None, help=f"Target board (auto-detected if omitted, default: {default_board})")
    build_p.add_argument("-f", "--file", help="Custom scratch C file containing app_main() to build")
    build_p.add_argument("-p", "--profile", default="debug", choices=["debug", "release"])

    # 3. Flash
    flash_p = subparsers.add_parser("flash", help="Build and flash firmware to target")
    flash_p.add_argument("target", nargs="?", default=None, help="Target C source file (e.g. my_code.c) or node name (e.g. control_board). Defaults to board default node if omitted.")
    flash_p.add_argument("-n", "--node", default=None, help=f"Target node (auto-selected per board if omitted, default: {default_node})")
    flash_p.add_argument("-b", "--board", default=None, help=f"Target board (auto-detected if omitted, default: {default_board})")
    flash_p.add_argument("-f", "--file", help="Custom scratch C file containing app_main() to flash")
    flash_p.add_argument("-s", "--serial-number", help="Target ST-Link serial number")
    flash_p.add_argument("-p", "--profile", default="debug", choices=["debug", "release"])

    # 4. Monitor
    mon_p = subparsers.add_parser("monitor", help="Open serial monitor")
    mon_p.add_argument("--port", help="Serial port device path (auto-detected if omitted)")
    mon_p.add_argument("--baud", type=int, default=default_baud, help=f"Serial baud rate (default: {default_baud})")
    mon_p.add_argument("-t", "--timeout", type=float, default=None, help="Monitor timeout in seconds (auto-exits after duration)")

    # 5. Devices
    subparsers.add_parser("devices", help="List connected ST-Link probes and serial COM ports")

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

    # 7. Generate (STM32CubeMX CLI)
    gen_p = subparsers.add_parser("generate", help="Regenerate peripheral drivers from .ioc files via STM32CubeMX CLI")
    gen_p.add_argument(
        "target",
        nargs="?",
        default="all",
        help="Target board, node, folder name, or .ioc path to regenerate (default: all)",
    )
    gen_p.add_argument(
        "-t", "--target-flag",
        dest="target_flag",
        default=None,
        help=argparse.SUPPRESS,
    )

    # 8. Check
    subparsers.add_parser("check", help="Verify build tools, ARM cross-compiler, flashing CLI, and Python dependencies")

    # 9. CAN (sniff / estop / pwm / send over SocketCAN or the SIL bridge)
    can_p = subparsers.add_parser("can", help="Sniff and stimulate the vehicle CAN bus (real can0 or --sil)")
    from tools.rov_can.cli import build_parser as _build_can_parser
    _build_can_parser(can_p)

    args = parser.parse_args()

    # Intelligent Auto-Detection of Board (for commands taking a board)
    if hasattr(args, "board") and args.board is None:
        detected_board, detected_sn, board_name = detect_connected_board()
        if detected_board:
            args.board = detected_board
            print(f"\033[1;36mAuto-detected connected dev board: {args.board} ({board_name})\033[0m")

    # Resolve target and board for build, flash, and run commands
    if args.command in ("build", "flash", "run"):
        target_input = getattr(args, "target", None)
        if target_input is None:
            if getattr(args, "file", None):
                target_input = args.file
            elif getattr(args, "node", None):
                target_input = args.node

        try:
            resolved_node, resolved_board, resolved_file = resolve_target(
                target_input, getattr(args, "board", None), config
            )
        except ValueError as e:
            sys.exit(f"Error: {e}")

        args.node = resolved_node
        args.board = resolved_board
        args.file = resolved_file

    if args.command == "build":
        elf = build_target(args.node, args.board, args.profile, getattr(args, "file", None))
        print(f"\033[1;32m[SUCCESS] Built target binary:\033[0m {elf}")

    elif args.command == "flash":
        elf = build_target(args.node, args.board, args.profile, getattr(args, "file", None))
        flash_binary(elf, args.serial_number)
        print("\033[1;32m[SUCCESS] Target flashed and verified successfully.\033[0m")

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

    elif args.command == "can":
        from tools.rov_can.cli import run as run_can
        sys.exit(run_can(args))

    elif args.command == "generate":
        target = getattr(args, "target", None) or getattr(args, "target_flag", None) or "all"
        generate_cubemx(target)

