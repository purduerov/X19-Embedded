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

import glob
import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def platform_key() -> str:
    """Canonical platform name.

    ``sys.platform`` is "win32" on Windows, "darwin" on macOS and "linux" on
    Linux -- NOT the capitalised names used as the keys below. Keying the tables
    off sys.platform directly silently matches nothing on every platform, which
    would make the CubeProgrammer default-path probe never run and report a
    correctly installed programmer as missing.
    """
    if sys.platform.startswith("win"):
        return "Windows"
    if sys.platform == "darwin":
        return "Darwin"
    return "Linux"

# Standard search paths and patterns for tools that may be installed but not yet on active PATH.
ARM_GCC_SEARCH_PATTERNS = {
    "Windows": [
        r"C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\*\bin\arm-none-eabi-gcc.exe",
        r"C:\Program Files\Arm GNU Toolchain arm-none-eabi\*\bin\arm-none-eabi-gcc.exe",
        r"C:\Program Files (x86)\GNU Arm Embedded Toolchain\*\bin\arm-none-eabi-gcc.exe",
        r"C:\Program Files\GNU Arm Embedded Toolchain\*\bin\arm-none-eabi-gcc.exe",
        r"C:\ST\STM32CubeCLT*\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe",
        r"C:\ST\STM32CubeCLT*\GNU-tools-arm-embedded\bin\arm-none-eabi-gcc.exe",
        r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeCLT*\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe",
        r"C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeCLT*\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe",
    ],
    "Darwin": [
        "/opt/homebrew/bin/arm-none-eabi-gcc",
        "/usr/local/bin/arm-none-eabi-gcc",
        "/Applications/ArmGNUToolchain/*/arm-none-eabi/bin/arm-none-eabi-gcc",
        "/opt/ST/STM32CubeCLT*/GNU-tools-for-STM32/bin/arm-none-eabi-gcc",
        "/opt/ST/STM32CubeCLT*/GNU-tools-arm-embedded/bin/arm-none-eabi-gcc",
    ],
    "Linux": [
        "/opt/arm-none-eabi/bin/arm-none-eabi-gcc",
        "/opt/st/stm32cubeclt*/GNU-tools-for-STM32/bin/arm-none-eabi-gcc",
        "/opt/st/stm32cubeclt*/GNU-tools-arm-embedded/bin/arm-none-eabi-gcc",
        "/usr/bin/arm-none-eabi-gcc",
    ],
}

CUBE_PROGRAMMER_PATTERNS = {
    "Windows": [
        Path(os.environ.get("ProgramFiles", r"C:\Program Files"))
        / "STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe",
        Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"))
        / "STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe",
        r"C:\ST\STM32CubeCLT*\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
        r"C:\ST\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
        r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeCLT*\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
        r"C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeCLT*\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
    ],
    "Darwin": [
        Path("/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI"),
        Path("/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOS/bin/STM32_Programmer_CLI"),
        Path("/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/STM32_Programmer_CLI"),
        Path("/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOS/STM32_Programmer_CLI"),
        Path("/Applications/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI"),
        Path("/Applications/STM32CubeProgrammer.app/Contents/MacOS/bin/STM32_Programmer_CLI"),
        Path("/Applications/STM32CubeProgrammer.app/Contents/MacOs/STM32_Programmer_CLI"),
        Path("/Applications/STM32CubeProgrammer.app/Contents/MacOS/STM32_Programmer_CLI"),
        Path.home() / "Applications/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI",
        Path("/opt/homebrew/bin/STM32_Programmer_CLI"),
        Path("/usr/local/bin/STM32_Programmer_CLI"),
        "/opt/ST/STM32CubeCLT*/STM32CubeProgrammer/bin/STM32_Programmer_CLI",
    ],
    "Linux": [
        Path("/opt/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"),
        Path("/usr/local/bin/STM32_Programmer_CLI"),
        "/opt/st/stm32cubeclt*/STM32CubeProgrammer/bin/STM32_Programmer_CLI",
    ],
}

INSTALL_HINTS = {
    "cmake": (
        "REQUIRED to configure. Install CMake 3.22 or newer:\n"
        "           brew install cmake | apt install cmake | choco install cmake"
    ),
    "ninja": (
        "REQUIRED as the CMake generator:\n"
        "           brew install ninja | apt install ninja-build | choco install ninja"
    ),
    "arm-none-eabi-gcc": (
        "REQUIRED to build node firmware. GNU Arm Embedded toolchain:\n"
        "           macOS:   brew install arm-none-eabi-gcc\n"
        "           Linux:   sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi\n"
        "           Windows: choco install gcc-arm-embedded  (or winget install Arm.GnuArmEmbeddedToolchain)"
    ),
    "STM32_Programmer_CLI": (
        "REQUIRED to flash physical targets. Install STM32CubeProgrammer or STM32CubeCLT:\n"
        "           Download from st.com: https://www.st.com/en/development-tools/stm32cubeprog.html\n"
        "           (macOS: if Gatekeeper blocks installer, run: sudo xattr -cr ~/Downloads/SetupSTM32CubeProgrammer.app)"
    ),
    "python": "REQUIRED for the host SIL suites and the declarative rov CLI",
    "pyserial": (
        "REQUIRED for rov CLI serial monitor and USB CDC device detection:\n"
        "           pip install -r requirements.txt  (or pip install pyserial)"
    ),
    "streamlit": (
        "optional, dashboard tests only:\n"
        "           python -m pip install \"streamlit>=1.40,<2\""
    ),
}

REQUIRED = ("cmake", "ninja", "arm-none-eabi-gcc", "STM32_Programmer_CLI", "pyserial")


def tool_version(argv: list[str]) -> str:
    try:
        out = subprocess.run(
            argv, capture_output=True, text=True, timeout=15, check=False
        )
    except (OSError, subprocess.SubprocessError):
        return ""
    return (out.stdout or out.stderr).strip().splitlines()[0] if (out.stdout or out.stderr) else ""


def find_arm_gcc() -> tuple[Path | None, bool]:
    on_path = shutil.which("arm-none-eabi-gcc")
    if on_path:
        return Path(on_path), True
    for pattern in ARM_GCC_SEARCH_PATTERNS.get(platform_key(), []):
        matches = glob.glob(str(pattern))
        if matches:
            matches.sort(reverse=True)
            return Path(matches[0]), False
    return None, False


def find_cube_programmer() -> Path | None:
    on_path = shutil.which("STM32_Programmer_CLI")
    if on_path:
        return Path(on_path)
    for entry in CUBE_PROGRAMMER_PATTERNS.get(platform_key(), []):
        if isinstance(entry, Path) and entry.exists():
            return entry
        matches = glob.glob(str(entry))
        if matches:
            matches.sort(reverse=True)
            return Path(matches[0])
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
    arm_path, arm_on_path = find_arm_gcc()
    if arm_path:
        arm_ver = tool_version([str(arm_path), "--version"])
        arm_detail = arm_ver if arm_on_path else f"{arm_ver} (at {arm_path}; restart terminal for PATH)"
        rows.append(("arm-none-eabi-gcc", arm_detail, True))
    else:
        rows.append(("arm-none-eabi-gcc", "", False))

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

    pyserial_ok = False
    pyserial_detail = ""
    try:
        import serial
        pyserial_ok = True
        pyserial_detail = getattr(serial, "__version__", "installed")
    except ImportError:
        pass
    rows.append(("pyserial", pyserial_detail, pyserial_ok))

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
