# X19-Embedded Lessons & Decision Log

Structured institutional memory. Query with `python rov.py memory search <query>` and append with `python rov.py memory add`. Keep entries concise (<= 250 lines total).

| ID | Date | Tag | Symptom / Context | Root Cause / Dead End | Verified Rule / Fix | Supersedes |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| L-001 | 2026-09-28 | STM32C542 | `target_bsp_node1`/`node2` fail on `GPIOC PIN_10` or leak probe pins `PA4`/`PA5` | `STM32C542CCT6` (LQFP-48) has no `PC0-PC12` or `PF0/PF1` pins; schematics are ground truth over `docs/pinout_mappings.md` | Always check `docs/board_findings.md` before assuming STM32C542 pinouts; never guess pin macros | `docs/pinout_mappings.md` (C542) |
| L-002 | 2026-10-03 | BUILD | Root cross-build skips Node 1 and Node 3 images | CubeMX startup assembly, linker scripts, and `SystemClock_Config` are absent in checkout | Check `docs/target-integration-blockers.md`; only enable `X19_BUILD_NODE1_ROOT`/`X19_BUILD_NODE3_ROOT` after CubeMX files land | - |
| L-003 | 2026-10-03 | CAN | Weak CAN stubs silently appeared to succeed on misconfigured targets | Shared weak CAN fallback previously masked missing target FDCAN drivers | Shared weak CAN fallback now fails closed; targets must implement strong `can_init`/`can_send`/`can_send_emergency`/`can_receive` | - |
| L-004 | 2026-10-15 | I2C | Driver tests risked testing mock values instead of register decoding | Adding value hooks inside drivers bypasses real register-to-engineering-unit math | Drivers must use injected `rov_i2c_bus_t` and decode raw registers; mock devices live in `tests/mocks/` | - |
| L-005 | 2026-10-22 | SIL | CI SIL dashboard checks fail or skip 55 safety tests | `streamlit` version drift or missing package triggers `SkipTest` in `tests/sil_stimulus/test_dashboard_app.py` | Keep `streamlit>=1.40,<2` synchronized between `.github/workflows/build_and_lint.yml` and `docs/sil_simulation_guide.md` | - |
| L-006 | 2026-11-27 | BUILD | Circular static lib references between fallback platform libs and rov_services fail to link | GNU ld resolves archives left-to-right unless grouped | Wrap scaffolded board static libraries in -Wl,--start-group ... -Wl,--end-group in tools/rov_cli/build.py | - |
