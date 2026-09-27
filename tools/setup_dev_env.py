#!/usr/bin/env python3
"""
One-command developer environment setup for X19-Embedded.

    python tools/setup_dev_env.py            # install what is missing
    python tools/setup_dev_env.py --dry-run  # print the plan, change nothing

Idempotent: re-running it only fixes what is still missing. Safe to re-run after
a partial failure.

WHAT IT CAN INSTALL AUTOMATICALLY
  - the VS Code extensions listed in .vscode/extensions.json
  - the GNU Arm Embedded toolchain, via winget / brew / apt

WHAT IT CANNOT, AND SAYS SO
  - STM32CubeProgrammer. It is not published in winget, and the STM32Cube VS Code
    extension does not bundle it. ST ships it only from their website, so this
    prints the link and the expected install path rather than pretending to
    automate it. Everything except flashing works without it.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
EXTENSIONS_JSON = REPO_ROOT / ".vscode" / "extensions.json"

CUBE_PROGRAMMER_URL = "https://www.st.com/en/development-tools/stm32cubeprog.html"

# Only the extensions the repo actually depends on. The STM32Cube extension is
# excluded on purpose -- see the note in .vscode/extensions.json.
SKIP_EXTENSIONS = {"stmicroelectronics.stm32-vscode-extension"}

def platform_key() -> str:
    """Canonical platform name.

    ``sys.platform`` is "win32" on Windows, "darwin" on macOS and "linux" on
    Linux -- NOT the capitalised names used as the keys below. Keying the tables
    off sys.platform directly silently matches nothing on every platform, which
    made the toolchain step a no-op and would have made the CubeProgrammer
    default-path probe never run.
    """
    if sys.platform.startswith("win"):
        return "Windows"
    if sys.platform == "darwin":
        return "Darwin"
    return "Linux"


TOOLCHAIN_COMMANDS = {
    "Windows": [["winget", "install", "-e", "--id", "Arm.GnuArmEmbeddedToolchain",
                 "--accept-source-agreements", "--accept-package-agreements"]],
    "Darwin": [["brew", "install", "--cask", "gcc-arm-embedded"]],
    "Linux": [["sudo", "apt-get", "install", "-y", "gcc-arm-none-eabi"]],
}

INSTALL_HINT = {
    "Windows": "winget install -e --id Arm.GnuArmEmbeddedToolchain",
    "Darwin": "brew install --cask gcc-arm-embedded",
    "Linux": "sudo apt-get install -y gcc-arm-none-eabi",
}


def run(cmd: list[str], dry_run: bool) -> int:
    printable = " ".join(cmd)
    if dry_run:
        print(f"      would run: {printable}")
        return 0
    print(f"      running:  {printable}")
    try:
        return subprocess.run(cmd, check=False).returncode
    except OSError as exc:
        print(f"      could not launch: {exc}")
        return 127


def have(binary: str) -> bool:
    return shutil.which(binary) is not None


def find_cube_programmer() -> Path | None:
    try:
        from tools.check_prereqs import find_cube_programmer as _find
        return _find()
    except ImportError:
        p = shutil.which("STM32_Programmer_CLI")
        return Path(p) if p else None


def find_code_cli() -> list[str] | None:
    """Locate the VS Code CLI shim.

    On Windows ``code`` on PATH resolves to Code.exe, which is the GUI binary:
    invoking it with --install-extension opens extra windows instead of doing a
    quiet install. The real CLI is bin\\code.cmd next to it. Prefer that; fall
    back to ``code`` elsewhere, where the shim is a normal executable.
    """
    if sys.platform.startswith("win"):
        for base in (
            Path(os.environ.get("LOCALAPPDATA", "")) / "Programs/Microsoft VS Code/bin",
            Path(os.environ.get("ProgramFiles", r"C:\Program Files")) / "Microsoft VS Code/bin",
        ):
            shim = base / "code.cmd"
            if shim.exists():
                return [str(shim)]
    found = shutil.which("code")
    return [found] if found else None


def installed_extensions(cli: list[str]) -> set[str]:
    try:
        out = subprocess.run(
            cli + ["--list-extensions"], capture_output=True, text=True, timeout=60, check=False
        )
    except (OSError, subprocess.SubprocessError):
        return set()
    return {line.strip().lower() for line in out.stdout.splitlines() if line.strip()}


def step_extensions(dry_run: bool, skip: bool) -> None:
    print("[1/3] VS Code extensions")
    if skip:
        print("    - skipped (--skip-extensions)")
        return
    if not EXTENSIONS_JSON.exists():
        print("    - .vscode/extensions.json not found; skipping")
        return

    data = json.loads(EXTENSIONS_JSON.read_text(encoding="utf-8"))
    wanted = [e for e in data.get("recommendations", []) if e not in SKIP_EXTENSIONS]

    cli = find_code_cli()
    if cli is None:
        print("    - VS Code CLI not found. Open the folder in VS Code and accept the")
        print("      recommended extensions when prompted; same result, no windows spawned.")
        return

    if not dry_run:
        print(f"    - using CLI: {cli[0]}")

    # Queried even in a dry run: --list-extensions only reads, and without it the
    # preview would claim it would reinstall extensions that are already present,
    # which is exactly the kind of thing a dry run must not get wrong.
    present = installed_extensions(cli)

    for ext in wanted:
        if ext.lower() in present:
            print(f"    - {ext}: already installed")
            continue
        print(f"    - {ext}")
        # No --force: it makes VS Code reload its window after every install.
        run(cli + ["--install-extension", ext, "--quiet"], dry_run)

    if not dry_run:
        print("    - done. No windows were opened or reloaded.")


def step_toolchain(dry_run: bool) -> None:
    print("[2/3] GNU Arm Embedded toolchain (needed to build node firmware)")
    try:
        from tools.check_prereqs import find_arm_gcc
        arm_path, arm_on_path = find_arm_gcc()
    except ImportError:
        arm_path, arm_on_path = shutil.which("arm-none-eabi-gcc"), True

    if arm_path:
        note = "" if arm_on_path else " (restart terminal/VS Code to refresh PATH)"
        print(f"    - already installed: {arm_path}{note}")
        return
    for cmd in TOOLCHAIN_COMMANDS.get(platform_key(), []):
        if shutil.which(cmd[0]) is None:
            print(f"    - {cmd[0]} not found; install the toolchain manually:")
            print(f"        {INSTALL_HINT.get(platform_key(), 'see arm.com')}")
            return
        if run(cmd, dry_run) != 0:
            print(f"    - install failed; install it manually:")
            print(f"        {INSTALL_HINT.get(platform_key(), 'see arm.com')}")
            return
    print("    - note: a new shell may be needed before arm-none-eabi-gcc is on PATH")


def step_programmer(dry_run: bool) -> None:
    print("[3/3] STM32CubeCLT / STM32CubeProgrammer (needed only to FLASH)")
    found = find_cube_programmer()
    if found:
        print(f"    - already installed: {found}")
        return
    print("    - NOT INSTALLED:")
    print("        ST does not publish its flasher in winget/apt/brew repositories.")
    print("        Download STM32CubeCLT (recommended for VS Code) or STM32CubeProgrammer:")
    print("          STM32CubeCLT:        https://www.st.com/en/development-tools/stm32cubeclt.html")
    print(f"          STM32CubeProgrammer: {CUBE_PROGRAMMER_URL}")
    print()
    print("    Everything except flashing works without it:")
    print("      - building the host SIL side, and cross-compiling node firmware")
    print("      - running all 25 CTest suites and the Python SIL suites")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true", help="print the plan without changing anything")
    parser.add_argument(
        "--skip-extensions",
        action="store_true",
        help="do not touch VS Code extensions; install them from the Extensions view instead",
    )
    args = parser.parse_args()

    print()
    print("X19 Embedded - developer environment setup")
    print("=" * 42)
    if args.dry_run:
        print("DRY RUN: nothing will be installed.")
    print()

    step_extensions(args.dry_run, args.skip_extensions)
    print()
    step_toolchain(args.dry_run)
    print()
    step_programmer(args.dry_run)

    print()
    print("Verifying:")
    os.environ["PYTHONPATH"] = str(REPO_ROOT)
    try:
        from tools.check_prereqs import main as check_main

        return check_main()
    except ImportError:
        sys.path.insert(0, str(REPO_ROOT))
        from tools.check_prereqs import main as check_main

        return check_main()


if __name__ == "__main__":
    raise SystemExit(main())
