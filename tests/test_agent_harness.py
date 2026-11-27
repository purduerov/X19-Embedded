"""Tests for the X19-Embedded agent harness, invariant linter, and lifecycle hooks."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
HOOKS_DIR = REPO_ROOT / ".agents" / "hooks"
STATE_DIR = REPO_ROOT / ".agents" / "state"


def _run_hook(script_name: str, payload: dict) -> dict:
    res = subprocess.run(
        [sys.executable, str(HOOKS_DIR / script_name)],
        cwd=REPO_ROOT / ".agents",
        input=json.dumps(payload),
        capture_output=True,
        text=True,
        timeout=15,
    )
    if res.returncode != 0:
        raise AssertionError(f"{script_name} exited {res.returncode}: {res.stderr}")
    return json.loads(res.stdout.strip())


class TestAgentHarness(unittest.TestCase):
    def setUp(self) -> None:
        self._backups: dict[Path, bytes | None] = {}
        for name in (
            "pending_lint.json",
            "session_edits.json",
            "capture_prompted.json",
            "session_metrics.json",
            "session_log.jsonl",
        ):
            p = STATE_DIR / name
            self._backups[p] = p.read_bytes() if p.is_file() else None

    def tearDown(self) -> None:
        for p, data in self._backups.items():
            if data is None:
                p.unlink(missing_ok=True)
            else:
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_bytes(data)

    def test_session_start_injects_context_and_assumptions_rule(self) -> None:
        out = _run_hook("session_start.py", {"conversationId": "test-conv-1"})
        msg = out["injectSteps"][0]["systemMessage"]["systemMessage"]
        self.assertIn("X19 Harness Active", msg)
        self.assertNotIn("dirty_files", msg)
        self.assertIn("assumptions", msg.lower())
        self.assertIn("Recent lessons (docs/lessons.md):", msg)
        self.assertIn("L-00", msg)

    def test_post_edit_to_pre_invocation_roundtrip_on_violation(self) -> None:
        bad_c = REPO_ROOT / "shared" / "src" / "_tmp_harness_bad.c"
        try:
            bad_c.write_text("void tmp_fn(void) { void *p = malloc(8); free(p); }\n", encoding="utf-8")
            post_out = _run_hook(
                "post_edit.py",
                {
                    "conversationId": "test-conv-viol",
                    "toolCall": {"name": "write_file", "args": {"Path": str(bad_c)}},
                },
            )
            self.assertEqual(post_out, {})

            pre_out = _run_hook("pre_invocation.py", {"conversationId": "test-conv-viol"})
            msg = pre_out["injectSteps"][0]["systemMessage"]["systemMessage"]
            self.assertIn("no-dynamic-alloc", msg)

            # Second invocation should be clean because pending_lint.json was consumed
            pre_out_2 = _run_hook("pre_invocation.py", {"conversationId": "test-conv-viol"})
            self.assertEqual(pre_out_2, {})
        finally:
            bad_c.unlink(missing_ok=True)

    def test_on_stop_blocks_on_lint_then_prompts_once_then_logs(self) -> None:
        conv_id = "test-conv-stop"
        bad_c = REPO_ROOT / "shared" / "src" / "_tmp_harness_stop.c"
        other_c = REPO_ROOT / "shared" / "src" / "_tmp_harness_stop2.c"
        try:
            bad_c.write_text("void tmp_stop(void) { void *p = malloc(4); }\n", encoding="utf-8")
            _run_hook(
                "post_edit.py",
                {
                    "conversationId": conv_id,
                    "toolCall": {"name": "write_file", "args": {"Path": str(bad_c)}},
                },
            )
            # 1. Stop should block because of malloc violation
            stop_1 = _run_hook(
                "on_stop.py",
                {
                    "conversationId": conv_id,
                    "terminationReason": "NO_TOOL_CALL",
                    "fullyIdle": True,
                    "continuationActive": False,
                },
            )
            self.assertEqual(stop_1.get("decision"), "continue")
            self.assertIn("no-dynamic-alloc", stop_1.get("reason", ""))

            # 2. Fix single file -> trivial 1-file edit should NOT waste a capture turn
            bad_c.write_text("void tmp_stop(void) {}\n", encoding="utf-8")
            # Add a second source file to verify multi-file capture prompt
            other_c.write_text("void tmp_stop2(void) {}\n", encoding="utf-8")
            _run_hook(
                "post_edit.py",
                {
                    "conversationId": conv_id,
                    "toolCall": {"name": "write_file", "args": {"Path": str(other_c)}},
                },
            )
            stop_2 = _run_hook(
                "on_stop.py",
                {
                    "conversationId": conv_id,
                    "terminationReason": "NO_TOOL_CALL",
                    "fullyIdle": True,
                    "continuationActive": False,
                },
            )
            self.assertEqual(stop_2.get("decision"), "continue")
            self.assertIn("Knowledge Capture Check", stop_2.get("reason", ""))

            # 3. With continuationActive=True, Stop must allow termination and log metrics
            stop_3 = _run_hook(
                "on_stop.py",
                {
                    "conversationId": conv_id,
                    "terminationReason": "NO_TOOL_CALL",
                    "fullyIdle": True,
                    "continuationActive": True,
                },
            )
            self.assertEqual(stop_3, {})
            log_text = (STATE_DIR / "session_log.jsonl").read_text(encoding="utf-8")
            self.assertIn(conv_id, log_text)
            self.assertIn('"lintCatches": 1', log_text)
            self.assertIn('"capturePrompted": true', log_text)
        finally:
            bad_c.unlink(missing_ok=True)
            other_c.unlink(missing_ok=True)

    def test_pre_tool_use_protects_guardrails_and_destructive_commands(self) -> None:
        # Protected guardrail file -> force_ask
        res1 = _run_hook(
            "pre_tool_use.py",
            {
                "toolCall": {
                    "name": "edit_file",
                    "args": {"Path": str(REPO_ROOT / ".agents" / "hooks.json")},
                }
            },
        )
        self.assertEqual(res1.get("decision"), "force_ask")

        res2 = _run_hook(
            "pre_tool_use.py",
            {
                "toolCall": {
                    "name": "write_file",
                    "args": {"Path": str(REPO_ROOT / "tools" / "check_invariants.py")},
                }
            },
        )
        self.assertEqual(res2.get("decision"), "force_ask")

        # Destructive shell command -> force_ask
        res3 = _run_hook(
            "pre_tool_use.py",
            {
                "toolCall": {
                    "name": "run_command",
                    "args": {"CommandLine": "git reset --hard HEAD~1"},
                }
            },
        )
        self.assertEqual(res3.get("decision"), "force_ask")

        # Normal file edit & command -> allow
        res4 = _run_hook(
            "pre_tool_use.py",
            {
                "toolCall": {
                    "name": "edit_file",
                    "args": {"Path": str(REPO_ROOT / "shared" / "src" / "can.c")},
                }
            },
        )
        self.assertEqual(res4.get("decision"), "allow")

    def test_post_edit_auto_formats_c_source_when_clang_format_available(self) -> None:
        sys.path.insert(0, str(REPO_ROOT))
        from tools.rov_cli.format import find_clang_format

        if not find_clang_format():
            self.skipTest("clang-format not installed")

        tmp_c = REPO_ROOT / "shared" / "src" / "_tmp_harness_fmt.c"
        try:
            tmp_c.write_text("void   tmp_fmt(  void  ){return;}\n", encoding="utf-8")
            _run_hook(
                "post_edit.py",
                {
                    "conversationId": "test-conv-fmt",
                    "toolCall": {"name": "write_file", "args": {"Path": str(tmp_c)}},
                },
            )
            formatted = tmp_c.read_text(encoding="utf-8")
            self.assertIn("void tmp_fmt(void)", formatted)
        finally:
            tmp_c.unlink(missing_ok=True)

    def test_repo_map_is_up_to_date(self) -> None:
        res = subprocess.run(
            [sys.executable, str(REPO_ROOT / "tools" / "gen_repo_map.py"), "--check"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            timeout=10,
        )
        self.assertEqual(res.returncode, 0, msg=res.stdout + res.stderr)


if __name__ == "__main__":
    unittest.main()
