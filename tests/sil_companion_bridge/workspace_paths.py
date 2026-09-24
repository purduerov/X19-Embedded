"""Locate sibling X19 software repositories for the SIL companion bridge.

The embedded repository is commonly checked out as a worktree outside the
multi-repository workspace.  Do not rely on a fixed relative path; callers can
provide X19_WORKSPACE_ROOT, or the resolver searches common workspace layouts.
"""

from __future__ import annotations

import os
from pathlib import Path


def _candidate_roots() -> list[Path]:
    roots: list[Path] = []
    configured_root = os.environ.get("X19_WORKSPACE_ROOT")
    if configured_root:
        roots.append(Path(configured_root))

    current = Path.cwd().resolve()
    roots.extend([current, *current.parents])
    script_root = Path(__file__).resolve().parent
    roots.extend([script_root, *script_root.parents])

    # This fallback keeps the documented Windows development layout usable when
    # the embedded repository is opened from a separate git worktree.
    roots.append(Path.home() / "Documents" / "Engineering" / "ROV")

    unique_roots: list[Path] = []
    seen: set[Path] = set()
    for root in roots:
        normalized = root.expanduser()
        if normalized not in seen:
            seen.add(normalized)
            unique_roots.append(normalized)
    return unique_roots


def find_x19_repo_dir(repo_name: str) -> Path:
    """Return a validated path to an X19-Core or X19-Surface checkout."""
    env_name = "X19_CORE_DIR" if repo_name == "X19-Core" else "X19_SURFACE_DIR"
    configured = os.environ.get(env_name)
    candidates: list[Path] = []
    if configured:
        candidates.append(Path(configured).expanduser())

    for root in _candidate_roots():
        candidates.extend(
            [
                root / "System_Software" / repo_name,
                root / repo_name,
            ]
        )

    required_parts = (
        ("src", "python", "messaging")
        if repo_name == "X19-Core"
        else ("src", "zmq", "python", "messaging")
    )
    for candidate in candidates:
        if candidate.joinpath(*required_parts).is_dir():
            return candidate.resolve()

    searched = "\n".join(f"  - {candidate}" for candidate in candidates)
    raise FileNotFoundError(
        f"Could not locate {repo_name}. Set {env_name} to its checkout path, "
        f"or set X19_WORKSPACE_ROOT to the multi-repository workspace root.\n"
        f"Searched:\n{searched}"
    )
