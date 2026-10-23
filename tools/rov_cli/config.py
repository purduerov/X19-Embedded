"""Purdue ROV CLI - Configuration & Environment Utilities."""

from __future__ import annotations

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
        import tomli as tomllib  # type: ignore
    except ImportError:
        sys.exit("Error: 'tomllib' or 'tomli' package is required for Python < 3.11.")

# Enable ANSI escape processing on Windows console
if sys.platform == "win32":
    os.system("")
    msys_bin = Path(r"C:\msys64\ucrt64\bin")
    if msys_bin.is_dir() and str(msys_bin) not in os.environ.get("PATH", ""):
        os.environ["PATH"] = f"{msys_bin};{os.environ.get('PATH', '')}"

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
CONFIG_PATH = REPO_ROOT / "rov.toml"


def parse_ioc_info(ioc_file: Path | str) -> dict[str, str]:
    """Extract MCU model, family, architecture, and compile flags from an .ioc file.

    Defaults to STM32G474RET6 / Cortex-M4 if not specified.
    """
    mcu = "STM32G474RET6"
    family = ""
    arch = "arm-cortex-m4"

    p = Path(ioc_file) if ioc_file else None
    if p and p.is_file():
        try:
            with open(p, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    line = line.strip()
                    if line.startswith("Mcu.CPN="):
                        val = line.split("=", 1)[1].strip()
                        if val:
                            mcu = val
                    elif line.startswith("Mcu.Family="):
                        family = line.split("=", 1)[1].strip().upper()
                    elif line.startswith("Mcu.Name="):
                        if not mcu or mcu == "STM32G474RET6":
                            val = line.split("=", 1)[1].strip()
                            if val:
                                mcu = val
        except Exception:
            pass

    # Infer family from MCU model if missing
    mcu_upper = mcu.upper()
    if not family:
        if "STM32C5" in mcu_upper:
            family = "STM32C5"
        elif "STM32F4" in mcu_upper:
            family = "STM32F4"
        elif "STM32G4" in mcu_upper:
            family = "STM32G4"
        elif "STM32H7" in mcu_upper:
            family = "STM32H7"
        elif "STM32F7" in mcu_upper:
            family = "STM32F7"
        elif "STM32L4" in mcu_upper:
            family = "STM32L4"
        elif "STM32F1" in mcu_upper:
            family = "STM32F1"
        elif "STM32G0" in mcu_upper:
            family = "STM32G0"
        elif "STM32F0" in mcu_upper:
            family = "STM32F0"
        else:
            family = "STM32G4"

    # Architecture, CPU flags, and fallback baseline board
    if "STM32C5" in family:
        arch = "arm-cortex-m33"
        cpu_flags = "-mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb"
        fallback_board = "stm32c5"
        fallback_lib = "stm32c5_platform"
    elif "STM32H7" in family:
        arch = "arm-cortex-m7"
        cpu_flags = "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb"
        fallback_board = "g474_nucleo"
        fallback_lib = "g474_platform"
    elif "STM32F7" in family:
        arch = "arm-cortex-m7"
        cpu_flags = "-mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb"
        fallback_board = "f411_nucleo"
        fallback_lib = "f411_platform"
    elif "STM32F4" in family:
        arch = "arm-cortex-m4"
        cpu_flags = "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb"
        fallback_board = "f411_nucleo"
        fallback_lib = "f411_platform"
    elif "STM32G4" in family:
        arch = "arm-cortex-m4"
        cpu_flags = "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb"
        fallback_board = "g474_nucleo"
        fallback_lib = "g474_platform"
    elif "STM32L4" in family:
        arch = "arm-cortex-m4"
        cpu_flags = "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb"
        fallback_board = "g474_nucleo"
        fallback_lib = "g474_platform"
    elif "STM32F0" in family or "STM32G0" in family:
        arch = "arm-cortex-m0plus"
        cpu_flags = "-mcpu=cortex-m0plus -mthumb"
        fallback_board = "f411_nucleo"
        fallback_lib = "f411_platform"
    elif "STM32F1" in family:
        arch = "arm-cortex-m3"
        cpu_flags = "-mcpu=cortex-m3 -mthumb"
        fallback_board = "f411_nucleo"
        fallback_lib = "f411_platform"
    else:
        arch = "arm-cortex-m4"
        cpu_flags = "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb"
        fallback_board = "f411_nucleo"
        fallback_lib = "f411_platform"

    return {
        "mcu": mcu,
        "family": family,
        "arch": arch,
        "cpu_flags": cpu_flags,
        "fallback_board": fallback_board,
        "fallback_lib": fallback_lib,
    }


def _parse_ioc_mcu(ioc_file: Path) -> tuple[str, str]:
    """Extract (mcu_name, architecture) from an .ioc file.

    Defaults to ('STM32G474RET6', 'arm-cortex-m4') if not found.
    """
    info = parse_ioc_info(ioc_file)
    return info["mcu"], info["arch"]


def _auto_discover_boards(config: dict) -> dict:
    """Auto-discover board folders in boards/ and register them in config."""
    boards_dir = REPO_ROOT / "boards"
    if not boards_dir.is_dir():
        return config

    boards_map = config.setdefault("boards", {})
    nodes_map = config.setdefault("nodes", {})

    for b_dir in boards_dir.iterdir():
        if not b_dir.is_dir() or b_dir.name.startswith((".", "_")):
            continue

        folder_name = b_dir.name
        short_name = folder_name.split("_")[0] if "_" in folder_name else folder_name

        ioc_files = [p for p in b_dir.glob("*.ioc") if not p.name.endswith(".bak")]
        ioc_file = ioc_files[0] if ioc_files else Path("")

        info = parse_ioc_info(ioc_file)
        mcu = info["mcu"]
        arch = info["arch"]
        family = info["family"]

        candidate_keys = [folder_name]
        if short_name != folder_name and short_name not in boards_map:
            candidate_keys.append(short_name)

        for key in candidate_keys:
            if key not in boards_map:
                boards_map[key] = {
                    "display_name": f"{folder_name} (Auto-Discovered Board)",
                    "architecture": arch,
                    "mcu": mcu,
                    "family": family,
                    "preset": "arm-c542-debug",
                    "flash_driver": "stm32cubeprog",
                    "default_baud": 115200,
                    "build_dir": "build/arm-c542-debug",
                }

        for node_name, node_cfg in nodes_map.items():
            supported = node_cfg.setdefault("supported_boards", [])
            for key in candidate_keys:
                if key not in supported:
                    supported.append(key)

                target_key = f"target_{key}"
                binary_key = f"binary_{key}"
                if target_key not in node_cfg:
                    node_cfg[target_key] = f"{key}_{node_name}.elf"
                if binary_key not in node_cfg:
                    node_cfg[binary_key] = f"build/arm-c542-debug/boards/{folder_name}/{key}_{node_name}.elf"

    return config


def load_config() -> dict:
    """Load and parse the declarative rov.toml configuration file, with auto-discovered boards."""
    if not CONFIG_PATH.is_file():
        sys.exit(f"Error: Missing configuration file at {CONFIG_PATH}")
    with open(CONFIG_PATH, "rb") as f:
        cfg = tomllib.load(f)
    return _auto_discover_boards(cfg)


def resolve_target(
    target_arg: str | None,
    board_arg: str | None,
    config: dict,
) -> tuple[str, str, str | None]:
    """Resolve target CLI input to (node_name, board_name, custom_file_path).

    Args:
        target_arg: Optional positional target argument (e.g. "my_code.c", "control_board", or None).
        board_arg: Explicit board passed via -b / --board or detected, or None.
        config: Loaded rov configuration dictionary from load_config().

    Returns:
        (node_name, board_name, custom_file_path)
        - If target_arg is a .c file or existing file path: ("sandbox", board_name, resolved_path)
        - If target_arg is a node name or alias: (canonical_node, board_name, None)
        - If target_arg is None: (default_node_for_board, board_name, None)
    """
    default_node = config.get("workspace", {}).get("default_node", "pi_shield")
    default_board = config.get("workspace", {}).get("default_board", "f411")

    # 1. Resolve board
    if board_arg is not None:
        board_name = board_arg
        if not board_name.strip():
            valid_boards = ", ".join(config.get("boards", {}).keys())
            raise ValueError(f"Unknown board ''. Valid boards: {valid_boards}")
        if board_name not in config.get("boards", {}):
            valid_boards = ", ".join(config.get("boards", {}).keys())
            raise ValueError(f"Unknown board '{board_name}'. Valid boards: {valid_boards}")
    else:
        board_name = default_board

    # 2. If target is omitted
    if not target_arg:
        if board_arg is not None:
            if board_name in ("g474", "g431"):
                node_name = "control_board"
            elif board_name in ("f411", "f446", "f401"):
                node_name = "pi_shield"
            else:
                node_name = default_node
        else:
            node_name = default_node
        return node_name, board_name, None

    # 3. Check if target is a C source file or existing file path
    target_str = str(target_arg)
    is_c_file = target_str.lower().endswith(".c")
    p = Path(target_str)

    if is_c_file or p.is_file() or (REPO_ROOT / target_str).is_file() or (REPO_ROOT / "sandbox" / target_str).is_file():
        # Locate existing file in priority order
        if p.is_file():
            resolved_path = p.resolve().as_posix()
        elif (REPO_ROOT / target_str).is_file():
            resolved_path = (REPO_ROOT / target_str).resolve().as_posix()
        elif (REPO_ROOT / "sandbox" / target_str).is_file():
            resolved_path = (REPO_ROOT / "sandbox" / target_str).resolve().as_posix()
        else:
            # File ending in .c that doesn't exist yet on disk
            if p.is_absolute():
                resolved_path = p.as_posix()
            else:
                resolved_path = (Path.cwd() / target_str).resolve().as_posix()

        return "sandbox", board_name, resolved_path

    # 4. Check if target is a known node or alias
    node_aliases = {
        "node1": "pi_shield",
        "node1_pi_shield": "pi_shield",
        "pi": "pi_shield",
        "pi_shield": "pi_shield",
        "node2": "control_board",
        "node2_control_board": "control_board",
        "control": "control_board",
        "control_board": "control_board",
        "node3": "power_slab",
        "node3_power_slab": "power_slab",
        "power": "power_slab",
        "power_slab": "power_slab",
        "sandbox": "sandbox",
        "rnd": "rnd",
    }
    target_lower = target_str.lower()
    if target_lower in node_aliases:
        return node_aliases[target_lower], board_name, None

    if target_lower in config.get("nodes", {}):
        return target_lower, board_name, None

    # 5. Check if user passed a board name as target
    if target_lower in config.get("boards", {}):
        raise ValueError(
            f"Target '{target_arg}' is a board name. Did you mean: python rov.py run -b {target_arg} ?"
        )

    # 6. Target is unrecognized
    valid_nodes = ", ".join(config.get("nodes", {}).keys())
    valid_boards = ", ".join(config.get("boards", {}).keys())
    raise ValueError(
        f"Unknown node '{target_arg}'. Valid nodes: {valid_nodes}. Valid boards: {valid_boards}."
    )



def sync_rov_toml() -> None:
    """Scan boards/ and append newly discovered board configs to rov.toml."""
    boards_dir = REPO_ROOT / "boards"
    if not boards_dir.is_dir():
        print("No boards/ directory found.")
        return

    content = CONFIG_PATH.read_text(encoding="utf-8")
    existing_cfg = tomllib.loads(content)
    existing_boards = existing_cfg.get("boards", {})

    new_boards_found = []

    for b_dir in sorted(boards_dir.iterdir()):
        if not b_dir.is_dir() or b_dir.name.startswith((".", "_")):
            continue

        folder_name = b_dir.name
        base_name = folder_name.removesuffix("_nucleo")
        if folder_name in existing_boards or base_name in existing_boards:
            continue

        ioc_files = [p for p in b_dir.glob("*.ioc") if not p.name.endswith(".bak")]
        ioc_file = ioc_files[0] if ioc_files else Path("")
        mcu, arch = _parse_ioc_mcu(ioc_file)

        board_key = folder_name
        block = f"""
[boards.{board_key}]
display_name = "{folder_name} (Auto-Discovered)"
architecture = "{arch}"
mcu = "{mcu}"
preset = "arm-c542-debug"
flash_driver = "stm32cubeprog"
default_baud = 115200
"""
        new_boards_found.append((board_key, block))

    if not new_boards_found:
        print("\033[1;32m[SUCCESS] rov.toml is already up-to-date with all boards in boards/.\033[0m")
        return

    with open(CONFIG_PATH, "a", encoding="utf-8") as f:
        f.write("\n# ==============================================================================\n")
        f.write("# Auto-Discovered Hardware Target Boards\n")
        f.write("# ==============================================================================\n")
        for board_key, block in new_boards_found:
            f.write(block)
            print(f"\033[1;36m+ Added [boards.{board_key}] to rov.toml\033[0m")

    print(f"\033[1;32m[SUCCESS] Synced {len(new_boards_found)} new board(s) into rov.toml.\033[0m")


def run_cmd(cmd: list[str], cwd: Path = REPO_ROOT) -> None:
    """Execute a system command in the given directory with colored terminal logs."""
    display = " ".join(cmd)
    print(f"\033[1;34m>>> Running:\033[0m {display}")
    env = os.environ.copy()
    msys_bin = Path("C:/msys64/ucrt64/bin")
    if msys_bin.is_dir():
        env["PATH"] = str(msys_bin) + os.pathsep + env.get("PATH", "")
    result = subprocess.run(cmd, cwd=cwd, env=env)
    if result.returncode != 0:
        sys.exit(f"\033[1;31mCommand failed with exit code {result.returncode}\033[0m")


def clean_stray_parent_cache(build_dir: Path) -> None:
    """Remove a stray CMake cache written at the preset parent by a raw `cmake -B build`.

    Presets own their build trees under `build/<preset>/`. A cache at the parent pins the
    compiler for that directory and makes CMake silently ignore a changed toolchain file on
    every later configure, which is the recurring 'stale build' failure. Only a parent
    directory literally named `build` is touched, so the repo root and non-preset trees
    (for example `build-native`) are never affected, and the preset subdirectories survive.
    """
    parent = build_dir.parent
    if parent.name != "build" or not (parent / "CMakeCache.txt").exists():
        return

    print(
        f"\033[1;33mStray CMake cache at {parent.name}/CMakeCache.txt "
        f"(written by a raw `cmake -B {parent.name}`). Removing parent-level configure "
        f"artifacts so presets stay authoritative...\033[0m"
    )
    for cache_copy in parent.glob("CMakeCache*.txt"):
        cache_copy.unlink()
    for directory in ("CMakeFiles", "Testing"):
        shutil.rmtree(parent / directory, ignore_errors=True)
    for name in (
        "build.ninja",
        "cmake_install.cmake",
        "compile_commands.json",
        "CTestTestfile.cmake",
        ".ninja_deps",
        ".ninja_log",
    ):
        artifact = parent / name
        if artifact.exists():
            artifact.unlink()
