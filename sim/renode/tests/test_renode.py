"""Automated test runner for Renode firmware simulations."""

import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = SCRIPT_DIR.parent.parent
SCRIPT_PATH = SCRIPT_DIR / "run_renode.ps1"


def run_target(target: str, duration: str = "00:00:02") -> bool:
    print(f"\n[TEST] Running Renode simulation target: '{target}' (Duration: {duration})...")
    cmd = [
        "powershell",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        str(SCRIPT_PATH),
        "-Target",
        target,
        "-Duration",
        duration,
    ]
    proc = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
    if proc.returncode != 0:
        print(f"[FAIL] Target '{target}' exited with code {proc.returncode}")
        print(proc.stderr)
        return False

    # Check for HardFault or critical crash markers
    if "HardFault" in proc.stdout:
        print(f"[FAIL] Target '{target}' triggered a CPU HardFault!")
        return False

    print(f"[PASS] Target '{target}' booted and simulated cleanly.")
    return True


def main():
    targets = ["node1", "node2", "multi"]
    all_passed = True
    for t in targets:
        if not run_target(t):
            all_passed = False

    if all_passed:
        print("\nAll Renode simulations (individual + multi-node) passed successfully!")
        sys.exit(0)
    else:
        print("\nSome Renode simulations failed.")
        sys.exit(1)


if __name__ == "__main__":
    main()
