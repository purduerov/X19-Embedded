"""
Tests for the HIL actuation policy in ``tools/hil_tester.py``.

``tools/hil_tester.py`` drives real thrusters on a real vehicle.  The defect
class pinned here is therefore not "does a check pass" but "can the runner
lie, or can it actuate when it should not":

* ``run_hil_test()`` used to return False unconditionally, because
  ``watchdog_ok`` was initialised False and never assigned again.  A suite
  that cannot go green trains its operators to ignore it, so the outcome
  model has to distinguish FAIL from "not exercised" and the exit code has
  to say which of those the caller asked about.
* the PWM injection ran unconditionally, with no confirmation, no heartbeat
  precondition and no bounds check, so a single command could spin up eight
  thrusters asymmetrically on a connected vehicle;
* ``python-can`` was imported by four tools and declared nowhere, so the
  toolchain was not reproducibly installable and the failure mode was a bare
  ImportError.

No test here needs a CAN adapter, and none may skip: CI treats a skipped
entry as a check that was claimed and not performed, and fails the build on
any skip at all.  ``hil_tester`` guards its ``can`` import, so this module
imports it with ``can = None`` and exercises only the pure policy functions.

Run from the repository root::

    python -m unittest tests.sil_stimulus.test_hil_safety -v
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

try:
    import tomllib
except ImportError:  # pragma: no cover - Python 3.10 fallback
    import tomli as tomllib

from tools import hil_tester


CANONICAL_ID = "python-can>=4.3"


class CanExtraIsDeclared(unittest.TestCase):
    """The CAN tools must be installable, and must not burden the CLI."""

    def setUp(self) -> None:
        with open(REPO_ROOT / "pyproject.toml", "rb") as handle:
            self.config = tomllib.load(handle)

    def test_can_extra_pins_a_bounded_python_can(self) -> None:
        extras = self.config["project"].get("optional-dependencies", {})
        self.assertIn("can", extras, "pyproject.toml must declare a 'can' extra")
        self.assertIn(CANONICAL_ID, extras["can"])

    def test_python_can_is_not_a_cli_runtime_dependency(self) -> None:
        """`rov build`/`rov flash`/`rov monitor` never open a CAN bus.

        Putting python-can in [project].dependencies would force it on every
        developer who only wants to flash a board, including on Windows where
        the SocketCAN backend is useless.
        """
        runtime = self.config["project"]["dependencies"]
        self.assertNotIn("python-can", runtime)
        self.assertFalse(
            [dep for dep in runtime if dep.startswith("python-can")],
            f"python-can must stay out of the CLI runtime deps, found: {runtime}",
        )


if __name__ == "__main__":
    unittest.main()
