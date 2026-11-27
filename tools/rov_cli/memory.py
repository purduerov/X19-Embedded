"""Purdue ROV CLI - Structured Institutional Memory & Knowledge Management."""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import re
from pathlib import Path

from .config import REPO_ROOT

GEMINI_PATH = REPO_ROOT / "GEMINI.md"
LESSONS_PATH = REPO_ROOT / "docs" / "lessons.md"
INDEX_PATH = REPO_ROOT / "docs" / "INDEX.md"
SESSION_LOG_PATH = REPO_ROOT / ".agents" / "state" / "session_log.jsonl"
TEAMWORK_DIR = REPO_ROOT / ".agents" / "teamwork"

LESSON_ROW_RE = re.compile(
    r"^\|\s*(L-\d+)\s*\|\s*([^|]+?)\s*\|\s*([^|]+?)\s*\|\s*([^|]+?)\s*\|\s*([^|]+?)\s*\|\s*([^|]+?)\s*\|\s*([^|]*?)\s*\|$"
)
PATH_REF_RE = re.compile(r"`([A-Za-z0-9_./-]+\.[A-Za-z0-9]+)`")


def _parse_lessons(path: Path = LESSONS_PATH) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    rows: list[dict[str, str]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        m = LESSON_ROW_RE.match(line.strip())
        if m:
            rows.append(
                {
                    "id": m.group(1),
                    "date": m.group(2),
                    "tag": m.group(3),
                    "symptom": m.group(4),
                    "cause": m.group(5),
                    "fix": m.group(6),
                    "supersedes": m.group(7).strip(),
                }
            )
    return rows


def _sanitize_cell(text: str) -> str:
    return " ".join(text.replace("|", "/").splitlines()).strip()


def cmd_status() -> int:
    print("=== X19-Embedded Knowledge & Harness Status ===")
    if GEMINI_PATH.is_file():
        text = GEMINI_PATH.read_text(encoding="utf-8", errors="replace")
        lines = len(text.splitlines())
        bytes_len = len(text.encode("utf-8"))
        pct = min(100, int((lines / 150) * 100))
        print(f"  GEMINI.md:       {lines}/150 lines ({pct}% budget), {bytes_len}/12000 bytes")
    else:
        print("  GEMINI.md:       missing")

    lessons = _parse_lessons()
    tag_counts: dict[str, int] = {}
    for row in lessons:
        tag = row["tag"]
        tag_counts[tag] = tag_counts.get(tag, 0) + 1
    tag_summary = ", ".join(f"{k}:{v}" for k, v in sorted(tag_counts.items())) or "none"
    print(f"  docs/lessons.md: {len(lessons)} entries ({tag_summary})")

    session_count = 0
    total_lint_catches = 0
    total_capture_prompts = 0
    if SESSION_LOG_PATH.is_file():
        for ln in SESSION_LOG_PATH.read_text(encoding="utf-8", errors="replace").splitlines():
            if not ln.strip():
                continue
            session_count += 1
            try:
                rec = json.loads(ln)
                total_lint_catches += int(rec.get("lintCatches", 0))
                if rec.get("capturePrompted"):
                    total_capture_prompts += 1
            except json.JSONDecodeError:
                pass
    print(
        f"  session_log:     {session_count} logged session(s) "
        f"(lintCatches:{total_lint_catches}, capturePrompted:{total_capture_prompts})"
    )

    teamwork_count = 0
    if TEAMWORK_DIR.is_dir():
        teamwork_count = sum(1 for p in TEAMWORK_DIR.iterdir() if p.is_dir())
    print(f"  .agents/teamwork:{teamwork_count} raw agent trace dir(s)")

    skills_dir = REPO_ROOT / ".agents" / "skills"
    skill_count = sum(1 for p in skills_dir.glob("*/SKILL.md")) if skills_dir.is_dir() else 0
    print(f"  repo skills:     {skill_count} active skill(s)")
    return 0


def cmd_search(query: str) -> int:
    q = query.lower()
    targets: list[Path] = []
    if LESSONS_PATH.is_file():
        targets.append(LESSONS_PATH)
    if INDEX_PATH.is_file():
        targets.append(INDEX_PATH)
    docs_dir = REPO_ROOT / "docs"
    if docs_dir.is_dir():
        for p in sorted(docs_dir.glob("*.md")):
            if p not in targets:
                targets.append(p)
    skills_dir = REPO_ROOT / ".agents" / "skills"
    if skills_dir.is_dir():
        targets.extend(sorted(skills_dir.glob("*/SKILL.md")))

    hits = 0
    for path in targets:
        rel = path.relative_to(REPO_ROOT).as_posix()
        for lineno, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
            if q in line.lower():
                print(f"{rel}:{lineno}: {line.strip()[:140]}")
                hits += 1
                if hits >= 40:
                    print("... (truncated at 40 matches)")
                    return 0
    if hits == 0:
        print(f"No knowledge entries matched '{query}'.")
    return 0


def cmd_add(tag: str, symptom: str, cause: str, fix: str, supersedes: str = "-") -> int:
    LESSONS_PATH.parent.mkdir(parents=True, exist_ok=True)
    if not LESSONS_PATH.is_file():
        LESSONS_PATH.write_text(
            "# X19-Embedded Lessons & Decision Log\n\n"
            "| ID | Date | Tag | Symptom / Context | Root Cause / Dead End | Verified Rule / Fix | Supersedes |\n"
            "| :--- | :--- | :--- | :--- | :--- | :--- | :--- |\n",
            encoding="utf-8",
        )

    existing = _parse_lessons()
    max_num = 0
    norm_symptom = _sanitize_cell(symptom).lower()
    norm_fix = _sanitize_cell(fix).lower()
    for row in existing:
        try:
            max_num = max(max_num, int(row["id"].split("-")[1]))
        except (IndexError, ValueError):
            pass
        if row["symptom"].lower() == norm_symptom or row["fix"].lower() == norm_fix:
            print(f"Duplicate lesson detected ({row['id']}); skipping append.")
            return 0

    next_id = f"L-{max_num + 1:03d}"
    today = _dt.date.today().isoformat()
    row_line = (
        f"| {next_id} | {today} | {_sanitize_cell(tag).upper()} | "
        f"{_sanitize_cell(symptom)} | {_sanitize_cell(cause)} | "
        f"{_sanitize_cell(fix)} | {_sanitize_cell(supersedes or '-')} |\n"
    )
    with LESSONS_PATH.open("a", encoding="utf-8") as fh:
        fh.write(row_line)
    print(f"Added {next_id} to docs/lessons.md")
    return 0


def cmd_distill(limit: int = 10, include_teamwork: bool = False) -> int:
    if SESSION_LOG_PATH.is_file():
        lines = [
            ln.strip()
            for ln in SESSION_LOG_PATH.read_text(encoding="utf-8", errors="replace").splitlines()
            if ln.strip()
        ]
        print(f"=== Recent Logged Sessions (last {min(limit, len(lines))} of {len(lines)}) ===")
        for raw in lines[-limit:]:
            try:
                rec = json.loads(raw)
                tags = ",".join(rec.get("tags", [])) or "general"
                files = ", ".join(rec.get("files", [])[:5])
                print(f"  [{rec.get('timestamp', '?')}] ({tags}) files: {files}")
            except json.JSONDecodeError:
                continue
    else:
        print("No session_log.jsonl entries yet.")

    if include_teamwork and TEAMWORK_DIR.is_dir():
        handoffs = sorted(TEAMWORK_DIR.glob("*/handoff.md"))
        print(f"\n=== .agents/teamwork handoffs ({len(handoffs)} total, showing up to {limit}) ===")
        for h in handoffs[:limit]:
            first_line = next(
                (ln.strip() for ln in h.read_text(encoding="utf-8", errors="replace").splitlines() if ln.strip()),
                "(empty)",
            )
            print(f"  {h.relative_to(REPO_ROOT).as_posix()}: {first_line[:100]}")
    return 0


def cmd_prune() -> int:
    lessons = _parse_lessons()
    if not lessons:
        print("No lessons found in docs/lessons.md.")
        return 0

    superseded_ids = {row["supersedes"] for row in lessons if row["supersedes"] not in {"", "-"}}
    issues = 0
    for row in lessons:
        if row["id"] in superseded_ids:
            print(f"[SUPERSEDED] {row['id']} is superseded by a newer entry.")
            issues += 1
        combined = f"{row['symptom']} {row['cause']} {row['fix']}"
        for ref in PATH_REF_RE.findall(combined):
            if "/" in ref and not (REPO_ROOT / ref).exists():
                print(f"[STALE PATH] {row['id']} references missing path '{ref}'.")
                issues += 1

    if issues == 0:
        print(f"Memory prune check passed: all {len(lessons)} lesson(s) reference valid paths and are active.")
    return 0


def run_memory(args: argparse.Namespace) -> int:
    sub = args.memory_command
    if sub == "status":
        return cmd_status()
    if sub == "search":
        return cmd_search(args.query)
    if sub == "add":
        return cmd_add(args.tag, args.symptom, args.cause, args.fix, args.supersedes)
    if sub == "distill":
        return cmd_distill(limit=args.limit, include_teamwork=args.teamwork)
    if sub == "prune":
        return cmd_prune()
    return 1
