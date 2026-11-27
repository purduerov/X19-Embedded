#!/usr/bin/env python3
"""Antigravity PostToolUse hook: record edits and run fast invariant checks."""

from __future__ import annotations

import json
import sys
from dataclasses import asdict
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
STATE_DIR = REPO_ROOT / ".agents" / "state"
EDITS_FILE = STATE_DIR / "session_edits.json"
PENDING_LINT_FILE = STATE_DIR / "pending_lint.json"
METRICS_FILE = STATE_DIR / "session_metrics.json"


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


def _load_json(path: Path, default: dict) -> dict:
    if not path.is_file():
        return default
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else default
    except Exception:
        return default


def main() -> int:
    try:
        raw = sys.stdin.read()
        payload = json.loads(raw) if raw.strip() else {}
        tool_call = payload.get("toolCall") or {}
        args = tool_call.get("args") or {}
        target = args.get("Path") or args.get("path") or ""
        rel = _normalize_rel(str(target))
        conv_id = str(payload.get("conversationId") or "default")

        if rel:
            STATE_DIR.mkdir(parents=True, exist_ok=True)
            edits = _load_json(EDITS_FILE, {})
            conv_files = edits.get(conv_id, [])
            if rel not in conv_files:
                conv_files.append(rel)
            edits[conv_id] = conv_files
            EDITS_FILE.write_text(json.dumps(edits, indent=2), encoding="utf-8")

            if rel.endswith((".c", ".h")) and not any(
                seg in rel.split("/") for seg in ("Drivers", "Middlewares", "vendor")
            ):
                try:
                    sys.path.insert(0, str(REPO_ROOT))
                    from tools.rov_cli.format import find_clang_format

                    clang_fmt = find_clang_format()
                    if clang_fmt and (REPO_ROOT / rel).is_file():
                        import subprocess

                        subprocess.run(
                            [str(clang_fmt), "-i", str(REPO_ROOT / rel)],
                            cwd=REPO_ROOT,
                            capture_output=True,
                            timeout=5,
                        )
                except Exception:
                    pass

            if rel.endswith((".c", ".h")) or rel == "GEMINI.md":
                sys.path.insert(0, str(REPO_ROOT / "tools"))
                from check_invariants import scan_invariants

                violations = [
                    asdict(v)
                    for v in scan_invariants(REPO_ROOT, target_files=[rel])
                    if v.severity == "error"
                ]
                if violations:
                    PENDING_LINT_FILE.write_text(
                        json.dumps({"conversationId": conv_id, "violations": violations}, indent=2),
                        encoding="utf-8",
                    )
                    metrics = _load_json(METRICS_FILE, {})
                    conv_m = metrics.get(conv_id) or {}
                    conv_m["lintCatches"] = int(conv_m.get("lintCatches", 0)) + len(violations)
                    metrics[conv_id] = conv_m
                    METRICS_FILE.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    except Exception:
        pass
    print("{}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
