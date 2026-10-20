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


def load_config() -> dict:
    """Load and parse the declarative rov.toml configuration file."""
    if not CONFIG_PATH.is_file():
        sys.exit(f"Error: Missing configuration file at {CONFIG_PATH}")
    with open(CONFIG_PATH, "rb") as f:
        return tomllib.load(f)


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
