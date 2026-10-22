"""Purdue ROV CLI - Firmware Build & CubeMX Code Generation."""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

from .config import REPO_ROOT, clean_stray_parent_cache, load_config, run_cmd
from .discovery import find_cubemx


def discover_ioc_targets() -> tuple[dict[str, Path], list[Path]]:
    """Dynamically scan for all .ioc files in boards/, nodes/, and tools/.

    Returns:
        (lookup_map, all_unique_ioc_paths)
    """
    lookup: dict[str, Path] = {}
    unique_paths: list[Path] = []
    seen_paths: set[Path] = set()

    search_roots = [
        REPO_ROOT / "boards",
        REPO_ROOT / "nodes",
        REPO_ROOT / "tools",
    ]

    for root in search_roots:
        if not root.is_dir():
            continue
        for ioc in sorted(root.glob("**/*.ioc")):
            if ioc.name.endswith(".bak") or ioc.name.startswith("."):
                continue
            resolved = ioc.resolve()
            if resolved in seen_paths:
                continue
            seen_paths.add(resolved)
            unique_paths.append(resolved)

            rel_str = resolved.relative_to(REPO_ROOT).as_posix()
            stem = ioc.stem.lower()
            parent_dir = ioc.parent.name.lower()

            lookup[rel_str.lower()] = resolved
            lookup[stem] = resolved
            lookup[parent_dir] = resolved

            if "_" in parent_dir:
                short_parent = parent_dir.split("_")[0]
                lookup.setdefault(short_parent, resolved)
            if "_" in stem:
                short_stem = stem.split("_")[0]
                lookup.setdefault(short_stem, resolved)

            if parent_dir.startswith("node") and "_" in parent_dir:
                parts = parent_dir.split("_", 1)
                lookup.setdefault(parts[0], resolved)
                lookup.setdefault(parts[1], resolved)

    return lookup, unique_paths


def generate_cubemx(target: str = "all") -> None:
    """Regenerate HAL drivers from CubeMX .ioc files.

    Dynamically discovers .ioc files across boards/, nodes/, and tools/, allowing
    developers to simply create a board/node directory with an .ioc file and generate immediately.
    """
    cubemx = find_cubemx(required=True)
    lookup, all_ioc_paths = discover_ioc_targets()

    targets_to_run: list[Path] = []

    if target in ("all", None):
        targets_to_run = all_ioc_paths
    else:
        candidate_path = Path(target)
        if not candidate_path.is_absolute():
            candidate_path = (REPO_ROOT / candidate_path).resolve()

        if candidate_path.is_file() and candidate_path.suffix == ".ioc":
            targets_to_run = [candidate_path]
        elif candidate_path.is_dir():
            found_in_dir = [p for p in candidate_path.glob("*.ioc") if not p.name.endswith(".bak")]
            if found_in_dir:
                targets_to_run = [found_in_dir[0].resolve()]
            else:
                sys.exit(f"\033[1;31mError: No .ioc file found in directory: {target}\033[0m")
        else:
            matched = lookup.get(target.strip().lower())
            if matched:
                targets_to_run = [matched]
            else:
                sub_matches = [
                    p for p in all_ioc_paths
                    if target.lower() in p.stem.lower() or target.lower() in p.parent.name.lower()
                ]
                if len(sub_matches) == 1:
                    targets_to_run = sub_matches
                elif len(sub_matches) > 1:
                    print(f"\033[1;33mTarget '{target}' is ambiguous, matches multiple boards/nodes:\033[0m")
                    for sm in sub_matches:
                        print(f"  - {sm.relative_to(REPO_ROOT).as_posix()}")
                    sys.exit(1)
                else:
                    print(f"\033[1;31mError: Target '{target}' not found.\033[0m")
                    print("\033[1;36mAvailable discovered .ioc targets:\033[0m")
                    for p in all_ioc_paths:
                        rel = p.relative_to(REPO_ROOT).as_posix()
                        print(f"  - {p.parent.name} ({rel})")
                    sys.exit(1)

    for ioc_path in targets_to_run:
        target_name = ioc_path.parent.name
        print(f"\033[1;36m=== Regenerating CubeMX Project for {target_name} ({ioc_path.name}) ===\033[0m")
        script_file = REPO_ROOT / f".cubemx_gen_{target_name}.tmp"
        try:
            script_file.write_text(f"config load {ioc_path.resolve()}\nproject generate\nexit\n", encoding="utf-8")
            run_cmd([cubemx, "-q", str(script_file.resolve())], cwd=ioc_path.parent)
            print(f"\033[1;32m[SUCCESS] Regenerated {target_name}\033[0m\n")
        finally:
            if script_file.is_file():
                script_file.unlink()


def build_target(node: str, board: str, profile: str = "debug", custom_file: str | None = None) -> str:
    """Configure CMake and build target binary according to profile and board mappings."""
    config = load_config()

    file_path = None
    if custom_file or node == "sandbox":
        node = "sandbox"
        file_path = (Path(custom_file).resolve() if custom_file else (REPO_ROOT / "sandbox/sandbox.c")).as_posix()
        if not Path(file_path).is_file():
            sys.exit(f"Error: Specified scratch source file does not exist: {file_path}")

    if node not in config["nodes"]:
        valid_nodes = ", ".join(config["nodes"].keys())
        sys.exit(f"Error: Unknown node '{node}'. Valid nodes: {valid_nodes}")

    if board not in config["boards"]:
        valid_boards = ", ".join(config["boards"].keys())
        sys.exit(f"Error: Unknown board '{board}'. Valid boards: {valid_boards}")

    node_cfg = config["nodes"][node]
    board_cfg = config["boards"][board]

    if board not in node_cfg.get("supported_boards", []):
        supported = ", ".join(node_cfg.get("supported_boards", []))
        sys.exit(f"Error: Node '{node}' is not configured for board '{board}'. Supported: {supported}")

    preset_name = board_cfg.get("preset", "arm-c542-debug")
    if profile == "release" and preset_name.endswith("-debug"):
        preset_name = preset_name[:-6] + "-release"
    elif profile == "debug" and preset_name.endswith("-release"):
        preset_name = preset_name[:-8] + "-debug"

    build_dir_rel = board_cfg.get("build_dir", f"build/{preset_name}")
    if profile == "release" and "debug" in build_dir_rel:
        build_dir_rel = build_dir_rel.replace("debug", "release")
    elif profile == "debug" and "release" in build_dir_rel:
        build_dir_rel = build_dir_rel.replace("release", "debug")
    build_dir = REPO_ROOT / build_dir_rel

    # A raw `cmake -B build` at the preset parent poisons every later configure; clear it.
    clean_stray_parent_cache(build_dir)

    target_key = f"target_{board}"
    binary_key = f"binary_{board}"

    target_name = node_cfg.get(target_key)
    binary_rel = node_cfg.get(binary_key)
    if binary_rel and profile == "release" and "debug" in binary_rel:
        binary_rel = binary_rel.replace("debug", "release")

    display_source = f" (source: {file_path})" if file_path else ""
    print(f"\033[1;32m=== Configuring & Building Node: {node} for Board: {board} ({profile}){display_source} ===\033[0m")

    # Validate cache: if building for embedded board, ensure cache was generated with ARM cross-compiler
    cache_file = build_dir / "CMakeCache.txt"
    if board != "host" and cache_file.exists():
        try:
            cache_text = cache_file.read_text(encoding="utf-8", errors="ignore")
            if "arm-none-eabi" not in cache_text:
                print(f"\033[1;33mStale or non-ARM CMake cache detected in {build_dir_rel}. Cleaning cache...\033[0m")
                shutil.rmtree(build_dir, ignore_errors=True)
        except Exception:
            pass

    # If building sandbox target, re-configure CMake with the active SANDBOX_FILE definition
    if node == "sandbox" and file_path:
        cmake_cfg = ["cmake", "--preset", preset_name, f"-DSANDBOX_FILE={file_path}"]
        run_cmd(cmake_cfg, cwd=REPO_ROOT)
    elif not (build_dir / "CMakeCache.txt").exists():
        run_cmd(["cmake", "--preset", preset_name], cwd=REPO_ROOT)

    # Build target
    cmd = ["cmake", "--build", str(build_dir)]
    if target_name:
        cmd.extend(["--target", target_name])
    run_cmd(cmd, cwd=REPO_ROOT)

    if binary_rel:
        elf_path = REPO_ROOT / binary_rel
        if elf_path.is_file():
            return str(elf_path)

    found = list(build_dir.glob("**/*.elf")) or list(build_dir.glob("**/*.exe"))
    if found:
        return str(found[0])

    sys.exit(f"Error: Could not locate built binary for {node} on {board} in {build_dir}")
