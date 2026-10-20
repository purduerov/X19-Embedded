"""Purdue ROV Unified Developer Interface (rov CLI)."""

from .build import build_target, generate_cubemx
from .cli import main
from .config import CONFIG_PATH, REPO_ROOT, clean_stray_parent_cache, load_config, run_cmd
from .discovery import (
    detect_connected_board,
    find_cubemx,
    find_stm32programmer_cli,
    get_stlink_probes,
    match_serial_port,
)
from .flash import flash_binary
from .monitor import open_monitor

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
