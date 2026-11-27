#!/usr/bin/env python3
"""Antigravity PreToolUse hook: protect harness guardrails and gate destructive commands."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

PROTECTED_EXACT = {
    ".agents/hooks.json",
    "tools/check_invariants.py",
}
PROTECTED_PREFIXES = (".agents/hooks/",)

DESTRUCTIVE_CMD_RE = re.compile(
    r"(?:"
    r"\bgit\s+reset\s+--hard\b"
    r"|\bgit\s+push\s+(?:--force|-f)\b"
    r"|\brm\s+-[^\s]*r[^\s]*f\b"
    r"|\brm\s+-rf\b"
    r"|\bRemove-Item\b.*-Recurse.*-Force"
    r")",
    re.IGNORECASE,
)


def _normalize_rel(raw_path: str) -> str | None:
    if not raw_path:
        return None
    p = Path(raw_path)
    if not p.is_absolute():
        p = REPO_ROOT / p
    try:
        return p.resolve().relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError:
        return None


def main() -> int:
    try:
        raw = sys.stdin.read()
        payload = json.loads(raw) if raw.strip() else {}
        tool_call = payload.get("toolCall") or {}
        name = str(tool_call.get("name") or "")
        args = tool_call.get("args") or {}

        if name in {"edit_file", "write_file"}:
            target = args.get("Path") or args.get("path") or ""
            rel = _normalize_rel(str(target))
            if rel and (rel in PROTECTED_EXACT or rel.startswith(PROTECTED_PREFIXES)):
                print(
                    json.dumps(
                        {
                            "decision": "force_ask",
                            "reason": f"Protected harness guardrail file ({rel}); confirm edit before proceeding.",
                        }
                    )
                )
                return 0

        if name == "run_command":
            cmd = str(args.get("CommandLine") or "")
            if DESTRUCTIVE_CMD_RE.search(cmd):
                print(
                    json.dumps(
                        {
                            "decision": "force_ask",
                            "reason": "Destructive command detected; confirm execution.",
                        }
                    )
                )
                return 0
    except Exception:
        pass

    print(json.dumps({"decision": "allow"}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
