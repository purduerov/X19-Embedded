"""Purdue ROV CLI - Firmware Binary Size & Stack Usage Audit."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

from .config import REPO_ROOT


def _find_arm_size() -> str | None:
    return shutil.which("arm-none-eabi-size")


def run_size_audit(build_dir: str | None = None, top: int = 15) -> int:
    """Report ELF section sizes (.text/.data/.bss) and top -fstack-usage (.su) functions."""
    search_root = (REPO_ROOT / build_dir) if build_dir else (REPO_ROOT / "build")
    if not search_root.is_dir():
        print(f"No build directory found at '{search_root.relative_to(REPO_ROOT).as_posix()}'.")
        print("Build a target first (e.g., `cmake --preset arm-c542-debug && cmake --build --preset arm-c542-debug`).")
        return 0

    elves = sorted(search_root.rglob("*.elf"))
    arm_size = _find_arm_size()
    if elves:
        print(f"=== ELF Size Summary ({len(elves)} binary/binaries) ===")
        if arm_size:
            res = subprocess.run(
                [arm_size, *(str(e) for e in elves)],
                cwd=REPO_ROOT,
                capture_output=True,
                text=True,
            )
            print(res.stdout.rstrip())
        else:
            for e in elves:
                print(f"  {e.relative_to(REPO_ROOT).as_posix()} ({e.stat().st_size} bytes on disk; arm-none-eabi-size not on PATH)")
    else:
        print(f"No .elf binaries found under '{search_root.relative_to(REPO_ROOT).as_posix()}'.")

    su_files = sorted(search_root.rglob("*.su"))
    if su_files:
        entries: list[tuple[int, str, str]] = []
        for su in su_files:
            for line in su.read_text(encoding="utf-8", errors="replace").splitlines():
                parts = line.split("\t")
                if len(parts) >= 3:
                    try:
                        bytes_used = int(parts[1])
                        entries.append((bytes_used, parts[0], parts[2]))
                    except ValueError:
                        continue
        entries.sort(reverse=True, key=lambda x: x[0])
        print(f"\n=== Top {min(top, len(entries))} Stack Usage Functions (.su) ===")
        for bytes_used, loc, kind in entries[:top]:
            print(f"  {bytes_used:5d} B  [{kind}]  {loc}")
    else:
        print("\nNo .su stack-usage files found (compile with -fstack-usage to enable).")

    return 0
