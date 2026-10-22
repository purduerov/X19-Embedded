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


def _parse_ioc_mcu(ioc_file: Path) -> tuple[str, str]:
    """Extract (mcu_name, architecture) from an .ioc file.

    Defaults to ('STM32G474RET6', 'arm-cortex-m4') if not found.
    """
    mcu = "STM32G474RET6"
    arch = "arm-cortex-m4"
    if not ioc_file.is_file():
        return mcu, arch

    try:
        with open(ioc_file, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                line = line.strip()
                if line.startswith("Mcu.CPN="):
                    mcu = line.split("=", 1)[1].strip()
                elif line.startswith("Mcu.Family="):
                    family = line.split("=", 1)[1].strip().upper()
                    if "STM32C5" in family:
                        arch = "arm-cortex-m33"
                    elif "STM32F4" in family or "STM32G4" in family:
                        arch = "arm-cortex-m4"
                    elif "STM32H7" in family:
                        arch = "arm-cortex-m7"
    except Exception:
        pass

    return mcu, arch


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

        mcu, arch = _parse_ioc_mcu(ioc_file)

        candidate_keys = [folder_name]
        if short_name != folder_name and short_name not in boards_map:
            candidate_keys.append(short_name)

        for key in candidate_keys:
            if key not in boards_map:
                boards_map[key] = {
                    "display_name": f"{folder_name} (Auto-Discovered Board)",
                    "architecture": arch,
                    "mcu": mcu,
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
