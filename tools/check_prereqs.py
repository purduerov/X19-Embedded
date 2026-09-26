#!/usr/bin/env python3
"""
Report which X19 build/flash prerequisites are reachable, and how to get the
missing ones. Read-only: it installs nothing and writes nothing.

Run it from the VS Code task palette ("Check: Prerequisites") or directly:

    python tools/check_prereqs.py

Python rather than a shell script, deliberately: the same check then runs
unmodified on Windows (PowerShell or cmd), macOS and Linux, and VS Code's task
runner does not need a particular shell to be configured first.

Exits 0 when every REQUIRED tool is present, 1 otherwise, so a new teammate can
gate their own onboarding on it.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

# STM32CubeProgrammer's CLI is normally not on PATH. On Windows it installs to a
# fixed location, so look there too before declaring it missing.
CUBE_PROGRAMMER_PATHS = {
    "Windows": [
        Path(os.environ.get("ProgramFiles", r"C:\Program Files"))
        / "STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe",
        Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"))
        / "STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe",
    ],
    "Darwin": [
        Path("/Applications/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI"),
        Path.home() / "Applications/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI",
    ],
    "Linux": [
        Path("/opt/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"),
        Path("/usr/local/bin/STM32_Programmer_CLI"),
    ],
}

INSTALL_HINTS = {
    "cmake": (
        "REQUIRED to configure. Install CMake 3.22 or newer:\n"
        "           brew install cmake | apt install cmake | winget install Kitware.CMake"
    ),
    "ninja": (
        "REQUIRED as the CMake generator:\n"
        "           brew install ninja | apt install ninja-build | winget install Ninja-build.Ninja"
    ),
    "arm-none-eabi-gcc": (
        "REQUIRED to build node firmware. GNU Arm Embedded toolchain 13.x, on PATH:\n"
        "           brew install --cask gcc-arm-embedded | apt install gcc-arm-none-eabi\n"
        "           winget install Arm.GnuArmEmbeddedToolchain"
    ),
    "STM32_Programmer_CLI": (
        "REQUIRED to flash. Install STM32CubeProgrammer as a system application:\n"
        "           winget install STMicroelectronics.STM32CubeProgrammer, or st.com\n"
        "           The ST VS Code extension does NOT bundle it. If you install it\n"
        "           somewhere else, update .vscode/tasks.json to match."
    ),
    "python": "REQUIRED for the host SIL suites and the stimulus tool",
    "streamlit": (
        "optional, dashboard tests only:\n"
        "           python -m pip install \"streamlit>=1.40,<2\""
    ),
}

REQUIRED = ("cmake", "ninja", "arm-none-eabi-gcc", "STM32_Programmer_CLI")


def tool_version(argv: list[str]) -> str:
    try:
        out = subprocess.run(
            argv, capture_output=True, text=True, timeout=15, check=False
        )
    except (OSError, subprocess.SubprocessError):
        return ""
    return (out.stdout or out.stderr).strip().splitlines()[0] if (out.stdout or out.stderr) else ""


def find_cube_programmer() -> Path | None:
    on_path = shutil.which("STM32_Programmer_CLI")
    if on_path:
        return Path(on_path)
    for candidate in CUBE_PROGRAMMER_PATHS.get(sys.platform, []):
        if candidate.exists():
            return candidate
    return None


def check_streamlit() -> bool:
    try:
        import importlib.util

        return importlib.util.find_spec("streamlit") is not None
    except (ImportError, ValueError):
        return False


def main() -> int:
    print()
    print("X19 Embedded - build and flash prerequisites")
    print("=" * 45)
    print()

    missing_required: list[str] = []
    missing_optional: list[str] = []
    rows: list[tuple[str, str, bool]] = []

    rows.append(("cmake", tool_version(["cmake", "--version"]), bool(shutil.which("cmake"))))
    rows.append(("ninja", tool_version(["ninja", "--version"]), bool(shutil.which("ninja"))))
    rows.append(
        (
            "arm-none-eabi-gcc",
            tool_version(["arm-none-eabi-gcc", "--version"]),
            bool(shutil.which("arm-none-eabi-gcc")),
        )
    )

    programmer = find_cube_programmer()
    rows.append(
        (
            "STM32_Programmer_CLI",
            str(programmer) if programmer else "",
            programmer is not None,
        )
    )

    python_ok = True
    rows.append(("python", sys.version.split()[0], python_ok))

    for name, detail, present in rows:
        if present:
            print(f"  [ ok ]   {name:<22} {detail}")
        else:
            print(f"  [MISS]  {name:<22} {INSTALL_HINTS[name]}")
            missing_required.append(name)

    if check_streamlit():
        print(f"  [ ok ]   {'streamlit':<22} dashboard UI can be tested")
    else:
        print(f"  [opt]   {'streamlit':<22} {INSTALL_HINTS['streamlit']}")
        missing_optional.append("streamlit")

    print()
    if missing_required:
        print(f"{len(missing_required)} REQUIRED tool(s) missing: {', '.join(missing_required)}")
        print("Until those are installed you can build and test the host SIL side only.")
        print()
        return 1

    print("All required tools are present.")
    if missing_optional:
        print(f"{len(missing_optional)} optional item(s) missing: {', '.join(missing_optional)}")
    print()
    print("Next: Terminal > Run Task > Build: Host SIL (Debug), or one of the Build: Node tasks.")
    print("Before flashing anything, run 'Flash: Read back and verify (no write)' to see what is attached.")
    print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
