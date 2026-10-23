#!/usr/bin/env python3
"""Master Test Runner for X19-Embedded End-to-End (E2E) Test Suite.

Discovers and executes all E2E tests across Tiers 1-4, producing structured
summary tables and exiting with code 0 on success or 1 on failure.

Usage:
    python tests/e2e/run_all.py
    python tests/e2e/run_all.py --tier 1
    python tests/e2e/run_all.py --tier 2
    python tests/e2e/run_all.py --tier 3
    python tests/e2e/run_all.py --tier 4
    python tests/e2e/run_all.py --strict
    python tests/e2e/run_all.py --json
    python tests/e2e/run_all.py -k <test_name_substring>
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import unittest
from pathlib import Path
from typing import Any, Dict, List

# Ensure repository root is on sys.path
REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

TIER_DIRS = {
    1: REPO_ROOT / "tests" / "e2e" / "tier1_features",
    2: REPO_ROOT / "tests" / "e2e" / "tier2_boundaries",
    3: REPO_ROOT / "tests" / "e2e" / "tier3_combinations",
    4: REPO_ROOT / "tests" / "e2e" / "tier4_real_world",
}

TIER_NAMES = {
    1: "Tier 1: Feature Coverage",
    2: "Tier 2: Boundary & Corner Cases",
    3: "Tier 3: Cross-Feature Combinations",
    4: "Tier 4: Real-World Scenarios",
}


class StructuredTestResult(unittest.TextTestResult):
    """Custom test result collector capturing structured counts and errors."""

    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__(*args, **kwargs)
        self.success_count = 0
        self.records: List[Dict[str, Any]] = []

    def addSuccess(self, test: unittest.TestCase) -> None:
        super().addSuccess(test)
        self.success_count += 1
        self.records.append({"name": str(test), "status": "PASSED"})

    def addFailure(self, test: unittest.TestCase, err: Any) -> None:
        super().addFailure(test, err)
        self.records.append({"name": str(test), "status": "FAILED", "error": str(err[1])})

    def addError(self, test: unittest.TestCase, err: Any) -> None:
        super().addError(test, err)
        self.records.append({"name": str(test), "status": "ERROR", "error": str(err[1])})

    def addSkip(self, test: unittest.TestCase, reason: str) -> None:
        super().addSkip(test, reason)
        self.records.append({"name": str(test), "status": "SKIPPED", "reason": reason})


def run_tier(
    tier_num: int,
    verbose: bool = False,
    pattern: str = "test_*.py",
    filter_substr: str | None = None,
) -> tuple[int, int, int, int, float, List[Dict[str, Any]]]:
    """Run all tests in a given tier directory.

    Returns:
        (total_tests, passed, skipped, failed, elapsed_time, records)
    """
    tier_path = TIER_DIRS[tier_num]
    loader = unittest.TestLoader()

    # Discover tests in tier directory
    suite = loader.discover(start_dir=str(tier_path), pattern=pattern, top_level_dir=str(REPO_ROOT))

    # Apply filter if provided
    if filter_substr:
        filtered_suite = unittest.TestSuite()
        for test in _flatten_suite(suite):
            if filter_substr.lower() in str(test).lower():
                filtered_suite.addTest(test)
        suite = filtered_suite

    stream = open(os.devnull, "w") if not verbose else sys.stderr
    verbosity = 2 if verbose else 0
    runner = unittest.TextTestRunner(
        stream=stream,
        resultclass=StructuredTestResult,
        verbosity=verbosity,
    )

    t0 = time.time()
    result = runner.run(suite)
    elapsed = time.time() - t0

    passed = getattr(result, "success_count", 0)
    skipped = len(result.skipped)
    failed = len(result.failures) + len(result.errors)
    total = result.testsRun

    return total, passed, skipped, failed, elapsed, getattr(result, "records", [])


def _flatten_suite(suite: unittest.TestSuite):
    for item in suite:
        if isinstance(item, unittest.TestSuite):
            yield from _flatten_suite(item)
        else:
            yield item


def main() -> None:
    parser = argparse.ArgumentParser(description="Purdue ROV X19-Embedded E2E Master Test Runner")
    parser.add_argument("--tier", type=int, choices=[1, 2, 3, 4], help="Run only tests from specified tier (1-4)")
    parser.add_argument("-k", "--filter", help="Only run tests matching substring")
    parser.add_argument("-v", "--verbose", action="store_true", help="Print verbose test execution progress")
    parser.add_argument("--strict", action="store_true", help="Enforce strict mode (no milestone skipping)")
    parser.add_argument("--json", action="store_true", help="Output machine-readable JSON results")
    args = parser.parse_args()

    if args.strict:
        os.environ["ROV_E2E_STRICT"] = "1"

    tiers_to_run = [args.tier] if args.tier else [1, 2, 3, 4]

    summary_records = {}
    overall_total = 0
    overall_passed = 0
    overall_skipped = 0
    overall_failed = 0
    start_time = time.time()

    if not args.json:
        print("=" * 78)
        print(" Purdue ROV X19-Embedded End-to-End (E2E) Test Suite")
        print("=" * 78)

    for tier_num in tiers_to_run:
        tier_title = TIER_NAMES[tier_num]
        total, passed, skipped, failed, elapsed, records = run_tier(
            tier_num,
            verbose=args.verbose,
            filter_substr=args.filter,
        )

        overall_total += total
        overall_passed += passed
        overall_skipped += skipped
        overall_failed += failed

        summary_records[tier_title] = {
            "total": total,
            "passed": passed,
            "skipped": skipped,
            "failed": failed,
            "elapsed_sec": round(elapsed, 3),
            "records": records,
        }

        if not args.json:
            status_indicator = "[PASS]" if failed == 0 else "[FAIL]"
            print(f"  {status_indicator} {tier_title:<40}  Passed: {passed:>3}  Skipped: {skipped:>2}  Failed: {failed:>2}  ({elapsed:.2f}s)")

    total_elapsed = time.time() - start_time

    if args.json:
        out = {
            "overall": {
                "total": overall_total,
                "passed": overall_passed,
                "skipped": overall_skipped,
                "failed": overall_failed,
                "elapsed_sec": round(total_elapsed, 3),
                "status": "PASS" if overall_failed == 0 else "FAIL",
            },
            "tiers": summary_records,
        }
        print(json.dumps(out, indent=2))
    else:
        print("-" * 78)
        pass_rate = (overall_passed / (overall_total - overall_skipped) * 100) if (overall_total - overall_skipped) > 0 else 100.0
        print(f" TOTAL: {overall_total} tests | Passed: {overall_passed} | Skipped: {overall_skipped} | Failed: {overall_failed}")
        print(f" Effective Pass Rate: {pass_rate:.1f}% | Wall Time: {total_elapsed:.2f}s")
        print("=" * 78)
        if overall_failed == 0:
            print("[SUCCESS] All executed end-to-end tests completed without errors.")
        else:
            print(f"[FAILURE] {overall_failed} test(s) failed.")

    sys.exit(0 if overall_failed == 0 else 1)


if __name__ == "__main__":
    main()
