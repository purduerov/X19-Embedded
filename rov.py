#!/usr/bin/env python3
"""
Purdue ROV Unified Embedded Developer Interface (rov CLI).
Platform-agnostic entry point. Works natively on Windows, Linux, and macOS.

Usage:
    python rov.py run [target] [-b board]
    python rov.py build [target] [-b board]
    python rov.py flash [target] [-b board]
    python rov.py monitor
    python rov.py test
    python rov.py devices
    python rov.py generate [target]
"""

import sys
from pathlib import Path

# Add tools directory to module search path
REPO_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(REPO_ROOT / "tools"))

from rov import main

if __name__ == "__main__":
    main()
