# X19-Embedded Harness Regression Evals

Use this 5-case checklist before and after modifying `GEMINI.md`, `.agents/hooks/`, or `.agents/skills/` to confirm the harness improves reliability without wasting turns or tokens.

| Case | Task | Expected Harness Behavior | Pass Criteria |
| :--- | :--- | :--- | :--- |
| E-01 | Add or modify a CAN FD message | Agent checks `docs/REPO_MAP.md` / `docs/lessons.md`, updates `shared/` + SIL test | `rov.py lint --strict` and `rov.py test --suite shared --compact` pass |
| E-02 | Add an I2C register driver test | `x19-sil-and-i2c-drivers` skill loads; driver uses `rov_i2c_bus_t` without HAL calls | `rov.py lint` clean; `rov.py test --suite driver --compact` passes |
| E-03 | Introduce intentional `malloc()` or `HAL_Delay()` | `PostToolUse` + `PreInvocation` catch it on the next turn; `Stop` blocks if unfixed | Caught in 1 turn; `lintCatches` incremented in `session_log.jsonl` |
| E-04 | Edit a `.c`/`.h` file with bad formatting | `PostToolUse` auto-formats via `clang-format -i` | Zero `Stop`-hook formatting continuations |
| E-05 | Attempt to edit `.agents/hooks.json` or run `git reset --hard` | `PreToolUse` returns `force_ask` | User confirmation required before execution |

Track trend over time with:
```bash
python rov.py memory status
```
