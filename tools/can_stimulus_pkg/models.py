"""Purdue ROV X19 CAN Stimulus - Report & Output Models."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List, NamedTuple, Sequence, Tuple

# Path setup to ensure sil_protocol is reachable
TOOLS_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = TOOLS_DIR.parent
SIL_BRIDGE_DIR = REPO_ROOT / "tests" / "sil_companion_bridge"
if str(SIL_BRIDGE_DIR) not in sys.path:
    sys.path.insert(0, str(SIL_BRIDGE_DIR))

from sil_protocol import unpack_sil_output_status

from .constants import SOLENOID_VALVES


class OutputStatus(NamedTuple):
    """A decoded 0x7FE snapshot of the mocked BSP outputs plus the sim clock."""

    pwms: Tuple[int, ...]
    brake_active: bool
    solenoids: int
    sim_time_ms: int

    @classmethod
    def decode(cls, payload: bytes) -> OutputStatus:
        pwms, brake_active, solenoids, sim_time_ms = unpack_sil_output_status(payload)
        return cls(tuple(pwms), brake_active, solenoids, sim_time_ms)

    def describe(self) -> str:
        brake = "BRAKE" if self.brake_active else "brake-off"
        return (
            f"sim={self.sim_time_ms}ms pwms={list(self.pwms)} {brake} "
            f"solenoids=0x{self.solenoids:04X}"
        )


class CheckResult:
    """One named observation with its verdict and the evidence behind it."""

    __slots__ = ("name", "passed", "detail")

    def __init__(self, name: str, passed: bool, detail: str = "") -> None:
        self.name = name
        self.passed = bool(passed)
        self.detail = detail

    def __repr__(self) -> str:  # pragma: no cover
        return f"CheckResult(name={self.name!r}, passed={self.passed!r}, detail={self.detail!r})"

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, CheckResult):
            return NotImplemented
        return (self.name, self.passed, self.detail) == (other.name, other.passed, other.detail)


class StimulusReport:
    """The aggregate verdict."""

    __slots__ = ("results",)

    def __init__(self, results: Sequence[CheckResult]) -> None:
        self.results: List[CheckResult] = list(results)

    @property
    def ok(self) -> bool:
        return bool(self.results) and all(result.passed for result in self.results)

    @property
    def failures(self) -> List[CheckResult]:
        return [result for result in self.results if not result.passed]

    @property
    def passed(self) -> List[CheckResult]:
        return [result for result in self.results if result.passed]

    def format(self, include_checks: bool = True) -> str:
        lines = []
        if include_checks:
            for result in self.results:
                lines.append(format_result(result))
        lines.append(
            f"Summary: {len(self.results)} check(s), {len(self.passed)} passed, "
            f"{len(self.failures)} failed"
        )
        if self.failures:
            names = ", ".join(failure.name for failure in self.failures)
            lines.append(f"[FAIL] {len(self.failures)} check(s) failed: {names}")
            for failure in self.failures:
                lines.append(f"[FAIL] {failure.name} failed: {failure.detail}")
        return "\n".join(lines)

    def __repr__(self) -> str:  # pragma: no cover
        return f"StimulusReport({self.results!r}, ok={self.ok!r})"


def format_result(result: CheckResult) -> str:
    """One line per check. Used both by the live stream and by the final report."""
    return f"[{'PASS' if result.passed else 'FAIL'}] {result.name}: {result.detail}"


def energised_coils(mask: int) -> List[str]:
    """Name every coil a mask energises, e.g. 0x0001 -> ['valve 0 coil A']."""
    return [
        f"valve {valve} coil {'AB'[coil]}"
        for valve in range(SOLENOID_VALVES)
        for coil in (0, 1)
        if mask & (1 << (2 * valve + coil))
    ]
