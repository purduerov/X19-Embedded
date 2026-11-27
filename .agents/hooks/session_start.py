#!/usr/bin/env python3
"""Antigravity SessionStart hook: hydrate context with repo status and recent lessons."""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
LESSONS_PATH = REPO_ROOT / "docs" / "lessons.md"
ROW_RE = re.compile(r"^\|\s*(L-\d+)\s*\|\s*[^|]+\|\s*([^|]+?)\s*\|\s*([^|]+?)\s*\|")


def _git_summary() -> str:
    try:
        branch = subprocess.run(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            timeout=3,
        ).stdout.strip() or "unknown"
        return f"branch={branch}"
    except Exception:
        return "git=unavailable"


def _recent_lessons(limit: int = 5) -> list[str]:
    if not LESSONS_PATH.is_file():
        return []
    rows: list[str] = []
    for line in LESSONS_PATH.read_text(encoding="utf-8", errors="replace").splitlines():
        m = ROW_RE.match(line.strip())
        if m:
            rows.append(f"- {m.group(1)} [{m.group(2).strip()}]: {m.group(3).strip()[:100]}")
    return rows[-limit:]


def main() -> int:
    try:
        _ = sys.stdin.read()
        lines = [
            f"[X19 Harness Active] ({_git_summary()})",
            "Rules: Always state assumptions explicitly (target board/MCU, pinout, preset, or ambiguous specs).",
            "Knowledge: See docs/INDEX.md or run `python rov.py memory search <topic>` before guessing.",
        ]
        recent = _recent_lessons()
        if recent:
            lines.append("Recent lessons (docs/lessons.md):")
            lines.extend(recent)
        out = {
            "injectSteps": [
                {"systemMessage": {"systemMessage": "\n".join(lines)}}
            ]
        }
        print(json.dumps(out))
    except Exception:
        print("{}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
