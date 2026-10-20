#!/usr/bin/env python3
"""Purdue ROV Unified Developer Interface (rov CLI).

Automates multi-target embedded monorepo build, flash, test, and monitoring tasks.

This module provides the top-level CLI façade, delegating to the modular
tools.rov_cli package.
"""

from __future__ import annotations

import sys
from pathlib import Path

# Ensure tools directory is on sys.path so rov_cli can be imported
TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

try:
    from tools.rov_cli import (
        CONFIG_PATH,
        REPO_ROOT,
        build_target,
        clean_stray_parent_cache,
        detect_connected_board,
        find_cubemx,
        find_stm32programmer_cli,
        flash_binary,
        generate_cubemx,
        get_stlink_probes,
        load_config,
        main,
        match_serial_port,
        open_monitor,
        run_cmd,
    )
except ImportError:
    from rov_cli import (  # type: ignore
        CONFIG_PATH,
        REPO_ROOT,
        build_target,
        clean_stray_parent_cache,
        detect_connected_board,
        find_cubemx,
        find_stm32programmer_cli,
        flash_binary,
        generate_cubemx,
        get_stlink_probes,
        load_config,
        main,
        match_serial_port,
        open_monitor,
        run_cmd,
    )

__all__ = [
    "CONFIG_PATH",
    "REPO_ROOT",
    "build_target",
    "clean_stray_parent_cache",
    "detect_connected_board",
    "find_cubemx",
    "find_stm32programmer_cli",
    "flash_binary",
    "generate_cubemx",
    "get_stlink_probes",
    "load_config",
    "main",
    "match_serial_port",
    "open_monitor",
    "run_cmd",
]

if __name__ == "__main__":
    main()
