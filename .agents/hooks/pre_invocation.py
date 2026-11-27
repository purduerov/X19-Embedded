#!/usr/bin/env python3
"""Antigravity PreInvocation hook: inject pending invariant violations."""

from __future__ import annotations

import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PENDING_LINT_FILE = REPO_ROOT / ".agents" / "state" / "pending_lint.json"


def main() -> int:
    try:
        _ = sys.stdin.read()
        if not PENDING_LINT_FILE.is_file():
            print("{}")
            return 0

        data = json.loads(PENDING_LINT_FILE.read_text(encoding="utf-8"))
        PENDING_LINT_FILE.unlink(missing_ok=True)
        violations = data.get("violations") or []
        if not violations:
            print("{}")
            return 0

        lines = ["[X19 Invariant Guard] Fix these rule violations in your recent edit:"]
        for v in violations[:15]:
            lines.append(f"- {v.get('file')}:{v.get('line')}: [{v.get('rule')}] {v.get('message')}")
        out = {
            "injectSteps": [
                {"systemMessage": {"systemMessage": "\n".join(lines)}}
            ],
            "terminationBehavior": "",
        }
        print(json.dumps(out))
    except Exception:
        print("{}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
