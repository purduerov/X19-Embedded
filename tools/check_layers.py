#!/usr/bin/env python3
"""Enforce the X19 firmware layer boundaries.

The contract in README section 5 and the board-agnostic architecture spec is a
one-way dependency graph:

    app (nodes/*/Core/Src/app.c, power_sequence.c)
      -> services
        -> drivers
          -> shared

Platform code (``boards/*/platform`` and the node BSP/CAN transport files) is a
peer implementation layer: it may include ``shared`` and the vendor HAL, but it
must not include drivers, services, or application code.

This checker is deliberately source-level and conservative. It checks direct
includes and direct ``HAL_*`` / ``__HAL_*`` / ``LL_*`` calls; it does not expand
the transitive include graph.

Run it from anywhere:

    python tools/check_layers.py
    python tools/check_layers.py --self-test

Exit status is 0 when the tree matches the contract, 1 on a violation.
"""

from __future__ import annotations

import argparse
import re
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

SCAN_GLOBS = (
    "shared/**/*.c",
    "shared/**/*.h",
    "drivers/**/*.c",
    "drivers/**/*.h",
    "services/**/*.c",
    "services/**/*.h",
    "boards/*/platform/**/*.c",
    "boards/*/platform/**/*.h",
    "nodes/*/Core/Src/app.c",
    "nodes/*/Core/Src/power_sequence.c",
    "nodes/*/Core/Src/bsp.c",
    "nodes/*/Core/Inc/app.h",
    "nodes/*/Core/Inc/power_sequence.h",
    "nodes/testing_and_rnd/Src/app.c",
    "nodes/testing_and_rnd/Src/bsp.c",
    "nodes/testing_and_rnd/Src/can_f4.c",
    "nodes/testing_and_rnd/Inc/*.h",
)

INDEX_GLOBS = (
    "shared/include/**/*.h",
    "drivers/include/**/*.h",
    "services/include/**/*.h",
    "boards/*/Core/Inc/**/*.h",
    "boards/*/platform/**/*.h",
    "nodes/*/Core/Inc/**/*.h",
    "nodes/testing_and_rnd/Inc/**/*.h",
)

SKIP_PARTS = {
    "Drivers",
    "Middlewares",
    ".git",
    ".agents",
    "build",
    "build-native",
    "build-test",
    "build-host",
    "build-native-coverage",
    "venv",
    ".venv",
}

# CubeMX board headers. They resolve to the platform layer, never to app.
BOARD_HEADERS = {
    "main.h",
    "tim.h",
    "can.h",
    "gpio.h",
    "usart.h",
    "i2c.h",
    "spi.h",
    "adc.h",
    "dma.h",
    "stm32f4xx_hal_conf.h",
    "stm32g4xx_hal_conf.h",
    "stm32c5xx_hal_conf.h",
}

HAL_HEADER_RE = re.compile(
    r"^(?:stm32[a-z0-9]*xx_(?:hal|ll)(?:_[a-z0-9_]+)?|stm32[a-z0-9]*xx|core_cm[a-z0-9]*|cmsis[a-z0-9_]*)\.h$",
    re.IGNORECASE,
)
HAL_CALL_RE = re.compile(r"\b(?:HAL|__HAL|LL)_[A-Za-z0-9_]+\b")
INCLUDE_RE = re.compile(r'^\s*#\s*include\s+([<"])([^>"]+)[>"]')

ALLOWED = {
    "shared": {"shared", "system", "external"},
    "drivers": {"shared", "drivers", "system", "external"},
    "services": {"shared", "drivers", "services", "system", "external"},
    "app": {"shared", "drivers", "services", "app", "system", "external"},
    "platform": {"shared", "platform", "hal", "system", "external"},
}


def is_skipped(path: Path, root: Path) -> bool:
    """Skip vendor/build paths, but only relative to the scanned root."""
    try:
        rel = path.resolve().relative_to(root.resolve())
    except ValueError:
        return True
    return any(part in SKIP_PARTS or part.startswith("build-") for part in rel.parts)


def classify(path: Path, root: Path) -> str:
    """Return the layer that owns ``path``."""
    try:
        rel = path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return "external"

    if rel.startswith("shared/"):
        return "shared"
    if rel.startswith("drivers/"):
        return "drivers"
    if rel.startswith("services/"):
        return "services"
    if rel.startswith("boards/"):
        return "platform"
    if rel.startswith("nodes/"):
        name = path.name
        if name in ("bsp.c", "can_f4.c") or name in BOARD_HEADERS:
            return "platform"
        return "app"
    return "external"


def strip_comments(text: str) -> str:
    """Blank out C comments while preserving line numbers."""
    out: list[str] = []
    i = 0
    n = len(text)
    in_block = False
    while i < n:
        if in_block:
            if text.startswith("*/", i):
                out.append("  ")
                i += 2
                in_block = False
            elif text[i] == "\n":
                out.append("\n")
                i += 1
            else:
                out.append(" ")
                i += 1
        else:
            if text.startswith("/*", i):
                out.append("  ")
                i += 2
                in_block = True
            elif text.startswith("//", i):
                while i < n and text[i] != "\n":
                    out.append(" ")
                    i += 1
            else:
                out.append(text[i])
                i += 1
    return "".join(out)


def collect_sources(root: Path) -> list[Path]:
    sources: set[Path] = set()
    for pattern in SCAN_GLOBS:
        for path in root.glob(pattern):
            if path.is_file() and not is_skipped(path, root):
                sources.add(path)
    return sorted(sources)


def collect_headers(root: Path) -> list[Path]:
    headers: set[Path] = set()
    for pattern in INDEX_GLOBS:
        for path in root.glob(pattern):
            if path.is_file() and not is_skipped(path, root):
                headers.add(path)
    return sorted(headers)


def resolve_include(include: str, source: Path, root: Path, headers: list[Path]) -> str:
    """Resolve a quoted include to a layer, or to ``external`` if unknown."""
    local = (source.parent / include)
    if local.is_file():
        return classify(local, root)

    normalized = Path(include).as_posix()
    matches = []
    for header in headers:
        rel = header.relative_to(root).as_posix()
        if rel == normalized or rel.endswith("/" + normalized):
            matches.append(header)

    if matches:
        source_layer = classify(source, root)
        same_layer = [h for h in matches if classify(h, root) == source_layer]
        return classify((same_layer or matches)[0], root)

    name = Path(include).name
    if HAL_HEADER_RE.match(name):
        return "hal"
    if name in BOARD_HEADERS:
        return "platform"
    return "external"


def scan(root: Path) -> list[tuple[Path, int, str]]:
    sources = collect_sources(root)
    headers = collect_headers(root)
    errors: list[tuple[Path, int, str]] = []

    for source in sources:
        layer = classify(source, root)
        if layer not in ALLOWED:
            continue

        text = strip_comments(source.read_text(encoding="utf-8", errors="replace"))
        for lineno, line in enumerate(text.splitlines(), 1):
            match = INCLUDE_RE.match(line)
            if match:
                delimiter, include = match.groups()
                if delimiter == "<":
                    target = "hal" if HAL_HEADER_RE.match(Path(include).name) else "system"
                else:
                    target = resolve_include(include, source, root, headers)

                if target not in ALLOWED[layer]:
                    errors.append(
                        (
                            source,
                            lineno,
                            f"{layer} code may not include {target} ('{include}')",
                        )
                    )

            if layer != "platform":
                for call in HAL_CALL_RE.finditer(line):
                    errors.append(
                        (
                            source,
                            lineno,
                            f"vendor HAL call {call.group(0)} is confined to platform code",
                        )
                    )

    return errors


def self_test() -> int:
    """Exercise the checker against a tiny tree with known violations."""
    fixtures = {
        "shared/include/ok.h": "#pragma once\n",
        "services/include/svc.h": "#pragma once\n",
        "drivers/src/ok.c": '#include "ok.h"\n',
        "drivers/src/up.c": '#include "svc.h"\n',
        "boards/b/Core/Inc/main.h": "#pragma once\n",
        "boards/b/platform/good.c": (
            '#include "ok.h"\n'
            '#include "stm32g4xx_hal.h"\n'
            "int f(void) { return HAL_GetTick(); }\n"
        ),
        "nodes/n/Core/Inc/app.h": "#pragma once\n",
        "nodes/n/Core/Src/app.c": (
            '#include "app.h"\n'
            '#include "main.h"\n'
            "int g(void) { return HAL_GPIO_ReadPin(0, 0); }\n"
        ),
    }

    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        for rel, content in fixtures.items():
            path = root / rel
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8")

        errors = scan(root)
        messages = [message for _, _, message in errors]

        expected = (
            len(errors) == 3
            and any("up.c" in path.name and "services" in msg for path, _, msg in errors)
            and any("main.h" in msg and "platform" in msg for _, _, msg in errors)
            and any("HAL_GPIO_ReadPin" in msg for _, _, msg in errors)
        )
        if not expected:
            print("[FAIL] layer checker self-test produced unexpected results:")
            for path, line, message in errors:
                print(f"  {path}:{line}: {message}")
            return 1

    print("[OK] layer checker self-test: 3 expected violations detected")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Enforce X19 firmware layer boundaries.")
    parser.add_argument("root", nargs="?", default=None, help="Repository root (default: the checker's repo).")
    parser.add_argument("--self-test", action="store_true", help="Run the checker's own fixture test and exit.")
    args = parser.parse_args(argv)

    if args.self_test:
        return self_test()

    root = Path(args.root).resolve() if args.root else REPO_ROOT
    if not root.is_dir():
        print(f"error: root is not a directory: {root}", file=sys.stderr)
        return 2

    sources = collect_sources(root)
    if not sources:
        print(f"error: no first-party sources found under {root}", file=sys.stderr)
        print("refusing to report a pass on an empty scan", file=sys.stderr)
        return 2

    errors = scan(root)
    for path, line, message in errors:
        try:
            rel = path.relative_to(root).as_posix()
        except ValueError:
            rel = str(path)
        print(f"{rel}:{line}: error: {message}")

    if errors:
        print(f"\nLayer check failed: {len(errors)} violation(s) across {sources} file(s).")
        print("Allowed direction: app -> services -> drivers -> shared; HAL only in platform code.")
        return 1

    print(f"Layer check passed: {len(sources)} file(s) honor the platform boundaries.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
