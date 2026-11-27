#!/usr/bin/env python3
"""Enforce X19-Embedded firmware safety invariants, layering, and repo hygiene.

Combines ``tools/check_layers.py`` with deterministic checks for:
- No dynamic memory allocation (malloc/calloc/realloc/free) in firmware
- No HAL_Delay() in non-platform firmware code
- STM32CubeMX main.c app_main() trampoline inside USER CODE BEGIN/END 2
- No emojis in source or documentation files
- Repo hygiene (no root .ioc files, no '*(1)*' sync-conflict duplicates)
- Instruction bloat limits on GEMINI.md and docs/lessons.md

Run from anywhere:
    python tools/check_invariants.py
    python tools/check_invariants.py --files shared/src/can.c --json
    python tools/check_invariants.py --self-test
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import check_layers  # noqa: E402

REPO_ROOT = TOOLS_DIR.parent

ALLOC_RE = re.compile(r"(?<![.\w>])(?:malloc|calloc|realloc|free)\s*\(")
HAL_DELAY_RE = re.compile(r"\bHAL_Delay\s*\(")
EMOJI_RE = re.compile(r"[\U0001F300-\U0001FAFF]")
CONFLICT_DUP_RE = re.compile(r" \(\d+\)(?:\.[^.]+)?$")

SKIP_DIRS = {
    ".git",
    "Drivers",
    "Middlewares",
    "build",
    "build-native",
    "build-test",
    "build-host",
    "build-native-coverage",
    "venv",
    ".venv",
    "__pycache__",
    ".freebuff",
    ".vibe-wise",
    ".superpowers",
}

GEMINI_MAX_LINES = 150
GEMINI_MAX_BYTES = 12000
LESSONS_MAX_LINES = 250


@dataclass
class Violation:
    file: str
    line: int
    rule: str
    severity: str  # "error" | "warning"
    message: str


def _rel_path(path: Path, root: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return path.as_posix()


def _is_ignored(path: Path, root: Path) -> bool:
    try:
        rel = path.resolve().relative_to(root.resolve())
    except ValueError:
        return True
    parts = rel.parts
    if any(p in SKIP_DIRS or p.startswith("build-") for p in parts):
        return True
    if len(parts) >= 2 and parts[0] == ".agents" and parts[1] in {"teamwork", "state"}:
        return True
    if len(parts) >= 2 and parts[0] == "tests" and parts[1] == "vendor":
        return True
    return False


def _is_firmware_source(rel: str) -> bool:
    if not (rel.endswith(".c") or rel.endswith(".h")):
        return False
    if rel.startswith(("shared/", "drivers/", "services/")):
        return True
    if rel.startswith("nodes/") and "/Drivers/" not in rel:
        return True
    return False


def _resolve_target_files(files: list[str] | None, root: Path) -> set[Path] | None:
    if not files:
        return None
    resolved: set[Path] = set()
    for f in files:
        p = Path(f)
        if not p.is_absolute():
            p = root / p
        resolved.add(p.resolve())
    return resolved


def scan_invariants(
    root: Path,
    target_files: list[str] | None = None,
) -> list[Violation]:
    """Scan ``root`` (or ``target_files``) and return all invariant violations."""
    violations: list[Violation] = []
    target_set = _resolve_target_files(target_files, root)

    # 1. Layer boundary check via check_layers.py
    for path, lineno, msg in check_layers.scan(root):
        if target_set is not None and path.resolve() not in target_set:
            continue
        violations.append(
            Violation(
                file=_rel_path(path, root),
                line=lineno,
                rule="layer-boundary",
                severity="error",
                message=msg,
            )
        )

    # Determine files to scan for source/doc rules
    if target_set is not None:
        candidates = [p for p in sorted(target_set) if p.is_file() and not _is_ignored(p, root)]
    else:
        candidates = [
            p
            for p in root.rglob("*")
            if p.is_file() and not _is_ignored(p, root)
        ]

    for path in candidates:
        rel = _rel_path(path, root)

        # 2. Repo hygiene: sync-conflict duplicates anywhere
        if CONFLICT_DUP_RE.search(path.name):
            violations.append(
                Violation(
                    file=rel,
                    line=1,
                    rule="repo-hygiene",
                    severity="error",
                    message=f"sync-conflict duplicate file detected: '{rel}'",
                )
            )
            continue

        # 3. Firmware C/H checks
        if _is_firmware_source(rel):
            raw = path.read_text(encoding="utf-8", errors="replace")
            stripped = check_layers.strip_comments(raw)
            layer = check_layers.classify(path, root)

            for lineno, line in enumerate(stripped.splitlines(), 1):
                if ALLOC_RE.search(line):
                    violations.append(
                        Violation(
                            file=rel,
                            line=lineno,
                            rule="no-dynamic-alloc",
                            severity="error",
                            message="dynamic memory allocation (malloc/calloc/realloc/free) is forbidden in vehicle firmware",
                        )
                    )
                if layer != "platform" and HAL_DELAY_RE.search(line):
                    violations.append(
                        Violation(
                            file=rel,
                            line=lineno,
                            rule="no-hal-delay",
                            severity="error",
                            message="HAL_Delay() is forbidden outside platform code",
                        )
                    )

            # CubeMX trampoline check on nodes/*/Core/Src/main.c
            if rel.startswith("nodes/") and rel.endswith("/Core/Src/main.c"):
                m = re.search(
                    r"/\*\s*USER CODE BEGIN 2\s*\*/(.*?)/\*\s*USER CODE END 2\s*\*/",
                    raw,
                    re.DOTALL,
                )
                if m and "app_main()" not in m.group(1):
                    lineno = raw[: m.start()].count("\n") + 1
                    violations.append(
                        Violation(
                            file=rel,
                            line=lineno,
                            rule="cubemx-trampoline",
                            severity="error",
                            message="CubeMX main.c must call app_main() inside USER CODE BEGIN 2",
                        )
                    )

        # 4. No emojis in code or markdown docs
        if path.suffix.lower() in {".c", ".h", ".py", ".md"}:
            raw = path.read_text(encoding="utf-8", errors="replace")
            for lineno, line in enumerate(raw.splitlines(), 1):
                if EMOJI_RE.search(line):
                    violations.append(
                        Violation(
                            file=rel,
                            line=lineno,
                            rule="no-emoji",
                            severity="error",
                            message="emojis are not permitted in code, comments, or documentation",
                        )
                    )

    # 5. Root-level .ioc check (matches CI)
    if target_set is None:
        for ioc in root.glob("*.ioc"):
            violations.append(
                Violation(
                    file=_rel_path(ioc, root),
                    line=1,
                    rule="repo-hygiene",
                    severity="error",
                    message="STM32CubeMX .ioc files must live inside nodes/<node_name>/, not repo root",
                )
            )

    # 6. Instruction bloat guards
    gemini_path = root / "GEMINI.md"
    if gemini_path.is_file() and (target_set is None or gemini_path.resolve() in target_set):
        text = gemini_path.read_text(encoding="utf-8", errors="replace")
        line_count = len(text.splitlines())
        byte_count = len(text.encode("utf-8"))
        if line_count > GEMINI_MAX_LINES or byte_count > GEMINI_MAX_BYTES:
            violations.append(
                Violation(
                    file="GEMINI.md",
                    line=1,
                    rule="instruction-bloat",
                    severity="warning",
                    message=(
                        f"GEMINI.md is {line_count} lines / {byte_count} bytes "
                        f"(budget: <= {GEMINI_MAX_LINES} lines / {GEMINI_MAX_BYTES} bytes); "
                        "move details to docs/ or .agents/skills/"
                    ),
                )
            )

    lessons_path = root / "docs" / "lessons.md"
    if lessons_path.is_file() and (target_set is None or lessons_path.resolve() in target_set):
        line_count = len(lessons_path.read_text(encoding="utf-8", errors="replace").splitlines())
        if line_count > LESSONS_MAX_LINES:
            violations.append(
                Violation(
                    file="docs/lessons.md",
                    line=1,
                    rule="memory-bloat",
                    severity="warning",
                    message=(
                        f"docs/lessons.md has {line_count} lines (budget: <= {LESSONS_MAX_LINES}); "
                        "run 'python rov.py memory prune' or consolidate superseded entries"
                    ),
                )
            )

    return violations


def self_test() -> int:
    """Exercise all invariant rules against a synthetic temporary tree."""
    fixtures = {
        "shared/include/ok.h": "#pragma once\n",
        "shared/src/bad_alloc.c": "void f(void) { void *p = malloc(16); free(p); }\n",
        "services/src/bad_delay.c": "void g(void) { HAL_Delay(10); }\n",
        "nodes/n1/Core/Src/main.c": (
            "int main(void) {\n"
            "  /* USER CODE BEGIN 2 */\n"
            "  /* missing trampoline */\n"
            "  /* USER CODE END 2 */\n"
            "}\n"
        ),
        "nodes/n2/Core/Src/main.c": (
            "int main(void) {\n"
            "  /* USER CODE BEGIN 2 */\n"
            "  app_main();\n"
            "  /* USER CODE END 2 */\n"
            "}\n"
        ),
        "docs/bad_emoji.md": "Status \U0001F680\n",
        "README (1).md": "duplicate\n",
        "GEMINI.md": "\n".join(f"line {i}" for i in range(GEMINI_MAX_LINES + 5)) + "\n",
    }

    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        for rel, content in fixtures.items():
            p = root / rel
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(content, encoding="utf-8")

        violations = scan_invariants(root)
        rules_hit = {v.rule for v in violations}
        expected_rules = {
            "no-dynamic-alloc",
            "no-hal-delay",
            "cubemx-trampoline",
            "no-emoji",
            "repo-hygiene",
            "instruction-bloat",
        }
        missing = expected_rules - rules_hit
        # Ensure nodes/n2/Core/Src/main.c (valid trampoline) was NOT flagged
        n2_flagged = any(v.file == "nodes/n2/Core/Src/main.c" for v in violations)

        if missing or n2_flagged:
            print(f"[FAIL] invariant self-test failed. missing={missing}, n2_flagged={n2_flagged}")
            for v in violations:
                print(f"  {v.file}:{v.line}: [{v.rule}] {v.message}")
            return 1

    print(f"[OK] invariant checker self-test: all {len(expected_rules)} rule families verified")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Enforce X19-Embedded firmware invariants and repo hygiene.")
    parser.add_argument("--files", nargs="*", default=None, help="Specific file paths to check.")
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON output.")
    parser.add_argument("--compact", action="store_true", help="Cap output to 20 violation lines plus summary.")
    parser.add_argument("--strict", action="store_true", help="Treat warnings as errors.")
    parser.add_argument("--self-test", action="store_true", help="Run synthetic fixture self-test and exit.")
    parser.add_argument("--root", default=None, help="Repository root (defaults to auto-detected repo root).")
    args = parser.parse_args(argv)

    if args.self_test:
        return self_test()

    root = Path(args.root).resolve() if args.root else REPO_ROOT
    violations = scan_invariants(root, target_files=args.files)

    errors = [v for v in violations if v.severity == "error"]
    warnings = [v for v in violations if v.severity == "warning"]
    failed = bool(errors) or (args.strict and bool(warnings))

    if args.json:
        payload = {
            "passed": not failed,
            "errorCount": len(errors),
            "warningCount": len(warnings),
            "violations": [asdict(v) for v in violations],
        }
        print(json.dumps(payload, indent=2))
    else:
        limit = 20 if args.compact else len(violations)
        for v in violations[:limit]:
            print(f"{v.file}:{v.line}: {v.severity}: [{v.rule}] {v.message}")
        if args.compact and len(violations) > limit:
            print(f"... ({len(violations) - limit} more omitted; re-run without --compact for full list)")
        if not violations:
            print("Invariant check passed: 0 errors, 0 warnings.")
        else:
            print(f"\nInvariant summary: {len(errors)} error(s), {len(warnings)} warning(s).")

    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
