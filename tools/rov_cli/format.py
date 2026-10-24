"""Purdue ROV CLI - Automated Code Formatting with clang-format."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent

CLANG_FORMAT_PATHS = [
    Path(r"C:\msys64\ucrt64\bin\clang-format.exe"),
    Path(r"C:\Program Files\LLVM\bin\clang-format.exe"),
    Path(r"C:\Program Files (x86)\LLVM\bin\clang-format.exe"),
    Path("/usr/bin/clang-format"),
    Path("/usr/local/bin/clang-format"),
    Path("/opt/homebrew/bin/clang-format"),
]

PROJECT_SOURCE_DIRS = [
    "nodes",
    "shared",
    "drivers",
    "services",
    "boards",
    "sandbox",
    "tests",
]

EXCLUDE_SEGMENTS = {
    "Drivers",
    "Middlewares",
    "build",
    "build-native",
    "build-test",
    "build-host",
    "build-native-coverage",
    ".git",
    ".agents",
    ".venv",
    "venv",
    "unity",
}

VALID_EXTENSIONS = {".c", ".h", ".cpp", ".hpp"}


def find_clang_format() -> Path | None:
    """Locate clang-format on PATH or common installation locations."""
    which_p = shutil.which("clang-format")
    if which_p:
        return Path(which_p)
    for cand in CLANG_FORMAT_PATHS:
        if cand.is_file():
            return cand
    return None


def is_excluded(path: Path) -> bool:
    """Check if any path segment matches the exclusion set."""
    parts = set(path.parts)
    return bool(parts & EXCLUDE_SEGMENTS)


def collect_format_files(paths: list[str] | None = None) -> list[Path]:
    """Collect C/C++ files to format, filtering out vendor libraries and build output."""
    files: list[Path] = []
    seen: set[Path] = set()

    if paths:
        for p_str in paths:
            p = Path(p_str)
            if not p.is_absolute():
                p = (REPO_ROOT / p).resolve()
            if p.is_file():
                if p.suffix.lower() in VALID_EXTENSIONS and p not in seen:
                    seen.add(p)
                    files.append(p)
            elif p.is_dir():
                for sub in p.rglob("*"):
                    if sub.is_file() and sub.suffix.lower() in VALID_EXTENSIONS:
                        if not is_excluded(sub) and sub not in seen:
                            seen.add(sub)
                            files.append(sub)
    else:
        for folder_name in PROJECT_SOURCE_DIRS:
            folder = REPO_ROOT / folder_name
            if not folder.is_dir():
                continue
            for sub in folder.rglob("*"):
                if sub.is_file() and sub.suffix.lower() in VALID_EXTENSIONS:
                    if not is_excluded(sub) and sub not in seen:
                        seen.add(sub)
                        files.append(sub)

    return sorted(files)


def run_format(check_only: bool = False, paths: list[str] | None = None) -> int:
    """Format or check C/C++ source code using clang-format."""
    clang_fmt = find_clang_format()
    if not clang_fmt:
        print("\033[1;31mError: clang-format executable not found on PATH or standard install paths.\033[0m")
        print("\nTo install clang-format:")
        print("  Windows: winget install LLVM.LLVM  (or choco install llvm)")
        print("  Linux:   sudo apt install clang-format")
        print("  macOS:   brew install clang-format\n")
        return 1

    config_file = REPO_ROOT / ".clang-format"
    if not config_file.is_file():
        print(f"\033[1;31mError: .clang-format configuration file not found at {config_file}\033[0m")
        return 1

    target_files = collect_format_files(paths)
    if not target_files:
        print("\033[1;33mNo C/C++ source files found matching target criteria.\033[0m")
        return 0

    mode_str = "Checking formatting for" if check_only else "Formatting"
    print(f"\033[1;36m=== {mode_str} {len(target_files)} C/C++ source file(s) ===\033[0m")

    # Chunk into batches of 50 to avoid Windows command line length limits
    batch_size = 50
    violating_files: list[Path] = []

    for i in range(0, len(target_files), batch_size):
        batch = target_files[i : i + batch_size]
        batch_args = [str(f.resolve()) for f in batch]

        if check_only:
            for single_file in batch:
                cmd = [str(clang_fmt), "--dry-run", "--Werror", str(single_file.resolve())]
                res = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
                if res.returncode != 0:
                    violating_files.append(single_file)
        else:
            cmd = [str(clang_fmt), "-i"] + batch_args
            res = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
            if res.returncode != 0:
                print(f"\033[1;31mError running clang-format:\033[0m\n{res.stderr}")
                return res.returncode

    if check_only:
        if violating_files:
            print(f"\n\033[1;31m[FAIL] {len(violating_files)} file(s) not formatted according to .clang-format:\033[0m")
            for vf in violating_files:
                try:
                    rel = vf.relative_to(REPO_ROOT).as_posix()
                except ValueError:
                    rel = str(vf)
                print(f"  - {rel}")
            print("\nRun \033[1;32mpython rov.py format\033[0m to format these files automatically.\n")
            return 1
        print(f"\033[1;32m[SUCCESS] All {len(target_files)} file(s) adhere to .clang-format.\033[0m\n")
        return 0

    print(f"\033[1;32m[SUCCESS] Successfully formatted {len(target_files)} file(s) according to .clang-format.\033[0m\n")
    return 0
