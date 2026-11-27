#!/usr/bin/env python3
"""Antigravity Stop hook: enforce lint/format gates, prompt once for capture, and log session."""

from __future__ import annotations

import datetime as _dt
import json
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
STATE_DIR = REPO_ROOT / ".agents" / "state"
EDITS_FILE = STATE_DIR / "session_edits.json"
CAPTURE_FILE = STATE_DIR / "capture_prompted.json"
METRICS_FILE = STATE_DIR / "session_metrics.json"
SESSION_LOG_FILE = STATE_DIR / "session_log.jsonl"


def _load_json(path: Path, default):
    if not path.is_file():
        return default
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return default


def _infer_tags(files: list[str]) -> list[str]:
    tags: set[str] = set()
    joined = " ".join(files).lower()
    if "can" in joined:
        tags.add("CAN")
    if "sil" in joined or "sim/" in joined:
        tags.add("SIL")
    if "drivers/" in joined:
        tags.add("DRIVERS")
    if "services/" in joined:
        tags.add("SERVICES")
    if "node1" in joined:
        tags.add("NODE1")
    if "node2" in joined:
        tags.add("NODE2")
    if "node3" in joined:
        tags.add("NODE3")
    if "tools/" in joined or ".agents/" in joined:
        tags.add("TOOLING")
    return sorted(tags) or ["GENERAL"]


def _check_clang_format(ch_files: list[str]) -> list[str]:
    sys.path.insert(0, str(REPO_ROOT))
    from tools.rov_cli.format import find_clang_format

    clang_fmt = find_clang_format()
    if not clang_fmt:
        return []
    existing = [str(REPO_ROOT / f) for f in ch_files if (REPO_ROOT / f).is_file()]
    if not existing:
        return []
    res = subprocess.run(
        [str(clang_fmt), "--dry-run", "--Werror", *existing],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        timeout=8,
    )
    if res.returncode != 0:
        return [f"clang-format failed on: {', '.join(ch_files)}. Run `python rov.py format`."]
    return []


def main() -> int:
    try:
        raw = sys.stdin.read()
        payload = json.loads(raw) if raw.strip() else {}
        if payload.get("terminationReason") != "NO_TOOL_CALL" or not payload.get("fullyIdle", True):
            print("{}")
            return 0

        conv_id = str(payload.get("conversationId") or "default")
        continuation_active = bool(payload.get("continuationActive", False))
        edits = _load_json(EDITS_FILE, {})
        edited_files: list[str] = edits.get(conv_id, []) if isinstance(edits, dict) else []

        ch_files = [f for f in edited_files if f.endswith((".c", ".h")) and (REPO_ROOT / f).is_file()]
        if not continuation_active and ch_files:
            sys.path.insert(0, str(REPO_ROOT / "tools"))
            from check_invariants import scan_invariants

            errors = [v for v in scan_invariants(REPO_ROOT, target_files=ch_files) if v.severity == "error"]
            fmt_errors = _check_clang_format(ch_files)
            if errors or fmt_errors:
                reasons = [f"{v.file}:{v.line}: [{v.rule}] {v.message}" for v in errors[:10]] + fmt_errors
                print(
                    json.dumps(
                        {
                            "decision": "continue",
                            "reason": "Fix invariant/format issues before finishing:\n" + "\n".join(reasons),
                        }
                    )
                )
                return 0

        prompted = _load_json(CAPTURE_FILE, [])
        if not isinstance(prompted, list):
            prompted = []
        source_edits = [f for f in edited_files if f.endswith((".c", ".h", ".py"))]
        needs_capture = (
            not continuation_active
            and conv_id not in prompted
            and "docs/lessons.md" not in edited_files
            and (len(source_edits) >= 2 or len(edited_files) >= 3)
        )
        if needs_capture:
            STATE_DIR.mkdir(parents=True, exist_ok=True)
            prompted.append(conv_id)
            CAPTURE_FILE.write_text(json.dumps(prompted[-50:], indent=2), encoding="utf-8")
            print(
                json.dumps(
                    {
                        "decision": "continue",
                        "reason": (
                            "Knowledge Capture Check: Did this session uncover a non-obvious hardware quirk, "
                            "dead end, or reusable workflow? If yes, record it via `python rov.py memory add ...` "
                            "or update `.agents/skills/`; if not, briefly state there is nothing new to capture and conclude."
                        ),
                    }
                )
            )
            return 0

        if edited_files:
            STATE_DIR.mkdir(parents=True, exist_ok=True)
            metrics = _load_json(METRICS_FILE, {})
            conv_m = metrics.pop(conv_id, {}) if isinstance(metrics, dict) else {}
            entry = {
                "timestamp": _dt.datetime.now(_dt.timezone.utc).isoformat(),
                "conversationId": conv_id,
                "files": edited_files,
                "fileCount": len(edited_files),
                "lintCatches": int(conv_m.get("lintCatches", 0)),
                "capturePrompted": conv_id in prompted,
                "tags": _infer_tags(edited_files),
            }
            with SESSION_LOG_FILE.open("a", encoding="utf-8") as fh:
                fh.write(json.dumps(entry) + "\n")
            if isinstance(metrics, dict):
                METRICS_FILE.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
            if isinstance(edits, dict) and conv_id in edits:
                del edits[conv_id]
                EDITS_FILE.write_text(json.dumps(edits, indent=2), encoding="utf-8")

        print("{}")
    except Exception:
        print("{}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
