# SIL Accuracy and Test Ergonomics — Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the hand-rolled `assert()` test harness with Unity, decouple the SIL clock from wall time, add per-node test selection, converge CI/local builds, and add a drift guard so the suite count can never silently rot again.

**Architecture:** Vendor Unity into `tests/vendor/unity/` behind a `rov_unity` static library, with a thin `ROV_TEST_MAIN`/`ROV_RUN_TEST` harness that honours a name filter. Introduce `tests/harness/sil_clock.h` as the single virtual clock and route `mock_bsp` and the bridge server through explicit `LOGICAL`/`REALTIME` scheduling. Register every node test through one `x19_add_node_test()` CMake function that sets CTest labels, driven by a CMake registry kept honest against `rov.toml` by a test-time consistency check. Expose `--node`/`--suite`/`-k`/`--target`/`--preset` on `rov.py`.

**Tech Stack:** C11, CMake (Ninja), CTest, Unity (vendored, MIT), Python 3 (for the consistency test and CLI), `ctest --preset` / labels / `-R`.

**Spec:** `docs/superpowers/specs/2026-10-03-sil-accuracy-and-test-ergonomics-design.md`

## Global Constraints

- L1 is frozen: `shared/include/bsp.h` and `shared/include/can_interface.h` are byte-identical before and after.
- Do not edit `nodes/*/Core/Src/bsp.c` or `main.c`. Do not rename `nodeN_app_init`/`nodeN_app_step` symbols.
- Physics is out of scope; do not touch `tests/mocks/mock_physics.c` semantics.
- No new CMake configure-time Python dependency (`cmake -B build-native` must work without Python on `PATH`). The CMake/`rov.toml` consistency check runs at test time, not configure time.
- C/C++ formatting follows `Embedded/X19-Embedded/.clang-format`; CI compiles with `-Wall -Wextra -Wpedantic -Wshadow -Wdouble-promotion -Werror`.
- No emojis in code comments, commits, or docs.
- Do not push to remote. Do not post GitHub comments without explicit permission.

## Review Focus

- A Unity test that aborts hard (e.g. `TEST_ASSERT_EACH_EQUAL_INT8` on a null pointer, or a divide-by-zero) must still be reported as a failure by the runner, not hang the process. **Test:** `test_rov_test_main` adds a deliberately-failing case invoked via `ROV_TEST_FILTER=fails` in a subprocess and asserts the exit code and output contain that case's name.
- Switching the bridge server to `LOGICAL` mode must not change per-cycle physics `dt`. **Test:** `test_sil_bridge_logical` asserts that after 100 logical cycles the mock time reports exactly 1000 ms and `mock_physics_get_state` matches the realtime-mode result for the first 100 cycles.
- `rov.py test --node control_board` must run *only* control-board-labelled tests, never the multi-node or driver tests. **Test:** `test_rov_node_filter` runs the CLI in dry-run and asserts the exact `ctest -L` label argument produced.
- The CMake/`rov.toml` consistency test must fail when a node is added to one source but not the other. **Test:** `test_registry_consistency` injects a synthetic mismatch into a temp copy of both files and asserts non-zero exit.
- `rov.py sim --realtime` must still satisfy a check that derives its budget from the engine's `sim_time_ms` (the §7.4a contract), while `--logical` must not sleep. **Test:** `test_rov_sim_modes` runs `--logical` over 200 cycles and asserts no per-cycle `Sleep` in the profile, and `--realtime` for 200 ms asserts at least 150 ms wall.

---

### Task 1: Vendor Unity and add the rov_unity library

**Files:**
- Create: `tests/vendor/unity/unity.c`
- Create: `tests/vendor/unity/unity.h`
- Create: `tests/vendor/unity/unity_internals.h`
- Create: `tests/vendor/unity/LICENSE.txt`
- Modify: `tests/CMakeLists.txt:1-16`

**Interfaces:**
- Consumes: nothing.
- Produces: CMake target `rov_unity` providing `unity.h`. Every later task links it.

- [ ] **Step 1: Write the failing test**

```c
/* tests/test_unity_smoke.c */
#include "unity.h"
void setUp(void) {}
void tearDown(void) {}
static void test_addition(void) { TEST_ASSERT_EQUAL_INT(4, 2 + 2); }
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_addition);
    return UNITY_END();
}
```

Register it in `tests/CMakeLists.txt` temporarily:

```cmake
add_executable(test_unity_smoke test_unity_smoke.c)
target_link_libraries(test_unity_smoke PRIVATE rov_unity)
add_test(NAME test_unity_smoke COMMAND test_unity_smoke)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake -B build-native -G Ninja && cmake --build build-native --target test_unity_smoke`
Expected: FAIL — `rov_unity` target does not exist, so configure or link errors.

- [ ] **Step 3: Write minimal implementation**

Fetch Unity v2.6.0 (ThrowTheSwitch, MIT) `unity.c`, `unity.h`, `unity_internals.h`, `LICENSE.txt` into `tests/vendor/unity/`. Then add to `tests/CMakeLists.txt` before the first test target:

```cmake
add_library(rov_unity STATIC vendor/unity/unity.c)
target_include_directories(rov_unity PUBLIC vendor/unity)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build build-native --target test_unity_smoke && ctest --test-dir build-native -R "^test_unity_smoke$" --output-on-failure`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add tests/vendor/unity tests/test_unity_smoke.c tests/CMakeLists.txt
git commit -m "build(tests): vendor Unity and add rov_unity target"
```

### Task 2: Add the ROV_TEST_MAIN filter harness

**Files:**
- Create: `tests/harness/rov_test_main.h`
- Create: `tests/test_rov_test_main.c`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `rov_unity` from Task 1.
- Produces: macros `ROV_TEST_MAIN()` and `ROV_RUN_TEST(fn)`. All 26 host SIL targets will use these in Task 12.

- [ ] **Step 1: Write the failing test**

```c
/* tests/test_rov_test_main.c */
#include "harness/rov_test_main.h"
void setUp(void) {}
void tearDown(void) {}
static void test_alpha(void) { TEST_ASSERT_TRUE(1); }
static void test_beta(void)  { TEST_ASSERT_TRUE(1); }
ROV_TEST_MAIN() {
    UNITY_BEGIN();
    ROV_RUN_TEST(test_alpha);
    ROV_RUN_TEST(test_beta);
    return UNITY_END();
}
```

Run with `ROV_TEST_FILTER=beta` — only beta should run. For now the macro does not exist.

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build-native --target test_rov_test_main`
Expected: FAIL — `harness/rov_test_main.h: No such file or directory`.

- [ ] **Step 3: Write minimal implementation**

```c
/* tests/harness/rov_test_main.h */
#ifndef ROV_TEST_MAIN_H
#define ROV_TEST_MAIN_H
#include "unity.h"
#include <stdlib.h>
#include <string.h>

#define ROV_RUN_TEST(fn)                                                          \
    do {                                                                          \
        const char *filter = getenv("ROV_TEST_FILTER");                           \
        if (!filter || filter[0] == '\0' || strstr(#fn, filter)) {                \
            RUN_TEST(fn);                                                         \
        }                                                                         \
    } while (0)

#define ROV_TEST_MAIN() int main(void)

#endif
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build build-native --target test_rov_test_main && $env:ROV_TEST_FILTER='beta'; ./build-native/tests/test_rov_test_main.exe`
Expected: PASS — output lists only `test_beta`, exit 0.

- [ ] **Step 5: Commit**

```bash
git add tests/harness/rov_test_main.h tests/test_rov_test_main.c tests/CMakeLists.txt
git commit -m "feat(tests): add ROV_TEST_MAIN filter harness"
```

### Task 3: Introduce the single sil_clock and route mock_bsp through it

**Files:**
- Create: `tests/harness/sil_clock.h`
- Create: `tests/harness/sil_clock.c`
- Modify: `tests/mocks/mock_bsp.c:147-159`
- Test: `tests/test_sil_clock.c`

**Interfaces:**
- Consumes: nothing.
- Produces: `sil_clock_now_us()`, `sil_clock_advance_us()`, `sil_clock_reset()`. `mock_bsp` will delegate its clock to these, so both stay in lockstep.

- [ ] **Step 1: Write the failing test**

```c
/* tests/test_sil_clock.c */
#include "harness/sil_clock.h"
#include "unity.h"
void setUp(void) {}
void tearDown(void) {}
static void test_starts_zero(void) { sil_clock_reset(); TEST_ASSERT_EQUAL_UINT64(0, sil_clock_now_us()); }
static void test_advances(void)   { sil_clock_reset(); sil_clock_advance_us(1500); TEST_ASSERT_EQUAL_UINT64(1500, sil_clock_now_us()); }
ROV_TEST_MAIN() { UNITY_BEGIN(); ROV_RUN_TEST(test_starts_zero); ROV_RUN_TEST(test_advances); return UNITY_END(); }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build-native --target test_sil_clock`
Expected: FAIL — `harness/sil_clock.h` missing.

- [ ] **Step 3: Write minimal implementation**

```c
/* tests/harness/sil_clock.c */
#include "sil_clock.h"
static uint64_t g_now_us;
void sil_clock_reset(void) { g_now_us = 0; }
uint64_t sil_clock_now_us(void) { return g_now_us; }
void sil_clock_advance_us(uint64_t dt_us) { g_now_us += dt_us; }
```
```c
/* tests/harness/sil_clock.h */
#ifndef SIL_CLOCK_H
#define SIL_CLOCK_H
#include <stdint.h>
void sil_clock_reset(void);
uint64_t sil_clock_now_us(void);
void sil_clock_advance_us(uint64_t dt_us);
#endif
```

Then in `mock_bsp.c` replace the body of `time_get_us()` and `mock_bsp_advance_time_us()` to delegate to `sil_clock_*` (keep `mock_bsp_advance_time_ms` calling into it). Wire `mock_bsp_reset()` to call `sil_clock_reset()`. Add `tests/harness/sil_clock.c` to the `rov_mocks` object library sources.

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build build-native && ctest --test-dir build-native -R "^(test_sil_clock|test_bsp|test_timesync)$" --output-on-failure`
Expected: PASS — existing BSP and timesync tests still pass, proving the delegation preserved behaviour.

- [ ] **Step 5: Commit**

```bash
git add tests/harness tests/mocks/mock_bsp.c tests/CMakeLists.txt tests/test_sil_clock.c
git commit -m "refactor(tests): route mock_bsp clock through sil_clock"
```

### Task 4: Add LOGICAL / REALTIME scheduling to the bridge server

**Files:**
- Modify: `tests/sil_bridge_server.c:99-136,322-324,488`
- Test: `tests/test_sil_bridge_logical.c`

**Interfaces:**
- Consumes: `sil_clock` from Task 3, `mock_bsp_*` from Task 3.
- Produces: CLI flags `--logical` / `--realtime`; a `sil_scheduler_mode_t` type. SP3's CTest labelled tests will rely on the default being `LOGICAL`.

- [ ] **Step 1: Write the failing test**

```c
/* tests/test_sil_bridge_logical.c */
#include "unity.h"
#include "harness/sil_clock.h"
#include "mocks/mock_bsp.h"
void setUp(void) {}
void tearDown(void) {}
static void test_logical_time_matches_cycles(void) {
    sil_clock_reset();
    for (int i = 0; i < 100; i++) { mock_bsp_advance_time_ms(10); }
    TEST_ASSERT_EQUAL_UINT32(1000, (uint32_t)(sil_clock_now_us() / 1000));
}
ROV_TEST_MAIN() { UNITY_BEGIN(); ROV_RUN_TEST(test_logical_time_matches_cycles); return UNITY_END(); }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build-native --target test_sil_bridge_logical`
Expected: FAIL — target does not exist yet, or `mock_bsp_advance_time_ms` does not yet drive `sil_clock`.

- [ ] **Step 3: Write minimal implementation**

In `sil_bridge_server.c` add a mode flag parsed from argv:

```c
typedef enum { SIL_MODE_LOGICAL, SIL_MODE_REALTIME } sil_scheduler_mode_t;
static sil_scheduler_mode_t s_mode = SIL_MODE_LOGICAL;
/* in argv parsing: */
/* } else if (strcmp(argv[i], "--realtime") == 0) { s_mode = SIL_MODE_REALTIME; } */
```

Make the platform sleep conditional:

```c
if (s_mode == SIL_MODE_REALTIME) { platform_sleep_ms(10); }
```

Leave `mock_bsp_advance_time_ms(10);` as-is (it now advances `sil_clock` via Task 3).

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build build-native && ctest --test-dir build-native -R "^test_sil_bridge_logical$" --output-on-failure`
Expected: PASS. Also confirm `test_sil_burnin` (LOGICAL by default) runs faster than before.

- [ ] **Step 5: Commit**

```bash
git add tests/sil_bridge_server.c tests/test_sil_bridge_logical.c tests/CMakeLists.txt
git commit -m "feat(sil): add LOGICAL/REALTIME scheduling modes"
```

### Task 5: Add x19_add_node_test registry with CTest labels

**Files:**
- Modify: `tests/CMakeLists.txt:113-167,170-247`
- Test: run `ctest -L` filter.

**Interfaces:**
- Consumes: the 26 test target shapes already present.
- Produces: CMake function `x19_add_node_test(NAME <n> NODE <pi_shield|control_board|power_slab|all> SUITE <app|driver|shared|integration|safety|power|fuzz|burnin|bsp|services> SOURCES ...)` that creates the executable, links it, and sets `LABELS "node:<node>;suite:<suite>"`.

- [ ] **Step 1: Write the failing test**

Run: `ctest --test-dir build-native -L "node:control_board" --show-only=json-v1`
Expected: FAIL — no tests have labels yet, so the filter reports zero tests.

- [ ] **Step 2: Run to confirm the failure**

Run the command above. Expected output: `Total Tests: 0`.

- [ ] **Step 3: Write minimal implementation**

At the top of `tests/CMakeLists.txt` after the mocks library:

```cmake
function(x19_add_node_test)
    cmake_parse_arguments(N "" "NAME;NODE;SUITE" "SOURCES;INCLUDES;LIBS" ${ARGN})
    add_executable(${N_NAME} ${N_SOURCES})
    target_include_directories(${N_NAME} PRIVATE ${N_INCLUDES})
    target_link_libraries(${N_NAME} PRIVATE ${N_LIBS})
    if(N_NODE)
        set_tests_properties(${N_NAME} PROPERTIES LABELS "node:${N_NODE}")
    endif()
    if(N_SUITE)
        set_property(TEST ${N_NAME} APPEND PROPERTY LABELS "suite:${N_SUITE}")
    endif()
    add_test(NAME ${N_NAME} COMMAND ${N_NAME})
endfunction()
```

Then convert the node-scoped targets (node1, node2, node3, services, multi_node, safety, power, fuzz, burnin, and the per-driver tests where a clear node applies) to call it. Driver tests that serve no specific node get `SUITE driver` and no `NODE`.

- [ ] **Step 4: Run to verify it passes**

Run: `ctest --test-dir build-native -L "node:control_board" --output-on-failure`
Expected: PASS — exactly the control-board node test(s) run.

- [ ] **Step 5: Commit**

```bash
git add tests/CMakeLists.txt
git commit -m "build(tests): register node/suite labels on all SIL targets"
```

### Task 6: Introduce the sil_vehicle node table

**Files:**
- Create: `tests/harness/sil_vehicle.h`
- Create: `tests/harness/sil_vehicle.c`
- Modify: `tests/sil_bridge_server.c:64-72,299-489`

**Interfaces:**
- Consumes: `node1_app_init`/`node1_app_step`, `node2_*`, `node3_*` signatures.
- Produces: `sil_node_t SIL_NODES[]` ordered Node2, Node1, Node3; `sil_vehicle_init()`, `sil_vehicle_step()`.

- [ ] **Step 1: Write the failing test**

```c
/* tests/test_sil_vehicle.c */
#include "harness/sil_vehicle.h"
#include "unity.h"
void setUp(void) {}
void tearDown(void) {}
static void test_three_nodes_registered(void) {
    TEST_ASSERT_EQUAL_INT(3, sil_vehicle_node_count());
    TEST_ASSERT_EQUAL_STRING("control_board", sil_vehicle_node(0)->name);
}
ROV_TEST_MAIN() { UNITY_BEGIN(); ROV_RUN_TEST(test_three_nodes_registered); return UNITY_END(); }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build-native --target test_sil_vehicle`
Expected: FAIL — `harness/sil_vehicle.h` missing.

- [ ] **Step 3: Write minimal implementation**

```c
/* tests/harness/sil_vehicle.h */
#ifndef SIL_VEHICLE_H
#define SIL_VEHICLE_H
#include <stdint.h>
typedef struct {
    const char *name;
    uint8_t     can_node_id;
    void      (*init)(void);
    void      (*step)(void);
} sil_node_t;
int sil_vehicle_node_count(void);
const sil_node_t *sil_vehicle_node(int idx);
void sil_vehicle_reset_all(void);
#endif
```

```c
/* tests/harness/sil_vehicle.c */
#include "sil_vehicle.h"
#include "rov_parameters.h"
extern void node1_app_init(void); extern void node1_app_step(void);
extern void node2_app_init(void); extern void node2_app_step(void);
extern void node3_app_init(void); extern void node3_app_step(void);
static const sil_node_t SIL_NODES[] = {
    { "control_board", ROV_NODE_CONTROL_BOARD, node2_app_init, node2_app_step },
    { "pi_shield",     ROV_NODE_PI_SHIELD,     node1_app_init, node1_app_step },
    { "power_slab",    ROV_NODE_POWER_SLAB,    node3_app_init, node3_app_step },
};
int sil_vehicle_node_count(void) { return (int)(sizeof(SIL_NODES)/sizeof(SIL_NODES[0])); }
const sil_node_t *sil_vehicle_node(int idx) { return &SIL_NODES[idx]; }
```

In `sil_bridge_server.c`, replace the six `extern` declarations and the hardcoded init/step block with a loop over `SIL_NODES`.

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build build-native && ctest --test-dir build-native -R "^test_sil_vehicle$" --output-on-failure`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add tests/harness/sil_vehicle.* tests/sil_bridge_server.c tests/test_sil_vehicle.c tests/CMakeLists.txt
git commit -m "refactor(sil): centralise node lifecycle in sil_vehicle table"
```

### Task 7: Extend rov.py CLI (--node, --suite, -k, --target, --preset)

**Files:**
- Modify: `tools/rov.py:417-419,508-515`
- Modify: `CMakePresets.json`

**Interfaces:**
- Consumes: labels from Task 5, node table from Task 6.
- Produces: `rov.py test --node X`, `--suite Y`, `-k Z`, `--target`, `--preset P`.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_rov_cli.py
import subprocess, sys
def test_node_flag_lists_only_that_node():
    out = subprocess.run([sys.executable, "tools/rov.py", "test", "--node", "control_board", "--dry-run"],
                         capture_output=True, text=True)
    assert "-L node:control_board" in out.stdout
    assert "sil-debug" not in out.stdout
```

Run it — it fails because the flags do not exist.

- [ ] **Step 2: Run to confirm failure**

Run: `python -m pytest tests/test_rov_cli.py -k node_flag`
Expected: FAIL — argparse exits 2 "unrecognized arguments: --node".

- [ ] **Step 3: Write minimal implementation**

In `tools/rov.py`:

```python
test_p.add_argument("--node", choices=["pi_shield", "control_board", "power_slab", "all"])
test_p.add_argument("--suite", choices=["app", "driver", "shared", "integration", "safety", "power", "fuzz", "burnin", "bsp", "services"])
test_p.add_argument("-k", "--filter", help="Unity case-name substring")
test_p.add_argument("--target", action="store_true", help="Include target_* readiness contracts")
test_p.add_argument("--preset", default=None)
test_p.add_argument("--dry-run", action="store_true")
```

Replace the test dispatch:

```python
elif args.command == "test":
    build_dir = REPO_ROOT / "build-native"
    if args.dry_run:
        print(f"cmake -B {build_dir} -G Ninja")
        print(f"cmake --build {build_dir}")
        labels = []
        if args.node and args.node != "all":
            labels.append(f"node:{args.node}")
        if args.suite:
            labels.append(f"suite:{args.suite}")
        label_args = []
        if labels:
            label_args = ["-L", "&&".join(labels)] if len(labels) == 1 else ["-L", "&&".join(labels)]
        cmd = ["ctest", "--test-dir", str(build_dir), "--output-on-failure"] + label_args
        if args.target:
            cmd = ["ctest", "--test-dir", str(build_dir), "--output-on-failure", "-R", "^target_"]
        if args.filter:
            cmd += ["-R", args.filter]
        print(" ".join(cmd))
        return
    run_cmd(["cmake", "-B", "build-native", "-G", "Ninja"], cwd=REPO_ROOT)
    run_cmd(["cmake", "--build", "--preset", args.preset] if args.preset else ["cmake", "--build", "build-native"], cwd=REPO_ROOT)
    cmd = ["ctest", "--test-dir", "build-native", "--output-on-failure"]
    if args.node and args.node != "all": cmd += ["-L", f"node:{args.node}"]
    if args.suite: cmd += ["-L", f"suite:{args.suite}"]
    if args.target: cmd += ["-R", "^target_"]
    if args.filter: cmd += ["-R", args.filter]
    run_cmd(cmd, cwd=REPO_ROOT)
```

Remove `cmake --preset sil-debug` as the default; keep the `sil-debug`/`sil-release` presets in `CMakePresets.json` for explicit opt-in only.

- [ ] **Step 4: Run to verify it passes**

Run: `python -m pytest tests/test_rov_cli.py`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/rov.py CMakePresets.json tests/test_rov_cli.py
git commit -m "feat(cli): add node/suite/case/target/preset test selection"
```

### Task 8: Add the build-native convergence and docs

**Files:**
- Modify: `AGENTS.md`, `README.md`, `docs/sil_simulation_guide.md`
- Delete: `tests/build/`

**Interfaces:**
- Consumes: Task 7's `rov.py` behaviour.
- Produces: one canonical host tree, `build-native`.

- [ ] **Step 1: Write the failing check**

Run: `Select-String -Path AGENTS.md -Pattern "build-native" | Select-Object -First 3`
Expected: references already exist but `rov.py test` still defaults to `sil-debug`; the docs and CLI disagree.

- [ ] **Step 2: Confirm**

Run `python rov.py test --dry-run` before Task 7's change is fully documented — note the output says `sil-debug`.

- [ ] **Step 3: Minimal implementation**

Update `AGENTS.md`, `README.md`, and `docs/sil_simulation_guide.md` to state that `build-native` is canonical and that `sil-debug`/`sil-release` presets are opt-in via `--preset`. Delete the stale `tests/build/` directory.

- [ ] **Step 4: Run to verify**

Run: `Get-ChildItem tests/build -ErrorAction SilentlyContinue`
Expected: no output.

- [ ] **Step 5: Commit**

```bash
git add AGENTS.md README.md docs/sil_simulation_guide.md
git rm -r --cached tests/build
git commit -m "docs: converge on build-native as canonical host tree"
```

### Task 9: CMake/`rov.toml` consistency test (test-time)

**Files:**
- Create: `tests/test_registry_consistency.py`
- Modify: `tests/CMakeLists.txt` (register it)

**Interfaces:**
- Consumes: the new CMake registry labels and `rov.toml`.
- Produces: a test-time check that fails when the two disagree.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_registry_consistency.py
import re, tomllib, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
class TestRegistryConsistency(unittest.TestCase):
    def test_node_keys_match(self):
        rov = tomllib.loads((ROOT / "rov.toml").read_text())
        cmake = (ROOT / "tests" / "CMakeLists.txt").read_text()
        for key in rov["nodes"]:
            self.assertIn(f'N_NODE "{key}"', cmake, f"{key} missing from CMake registry")
if __name__ == "__main__": unittest.main()
```

Because the registry doesn't yet emit `N_NODE "key"` literally, this should fail until Task 5's function is actually called with every node. (If it passes, the registry is already complete.)

- [ ] **Step 2: Run to verify current state**

Run: `python -m unittest tests.test_registry_consistency -v`
Expected: fails if any node is missing from the registry; this is the signal to wire the remaining node tests into Task 5.

- [ ] **Step 3: Minimal implementation**

Ensure every node in `rov.toml` appears in at least one `x19_add_node_test(... NODE ...)` call in `tests/CMakeLists.txt`.

- [ ] **Step 4: Run to verify it passes**

Run: `python -m unittest tests.test_registry_consistency -v`
Expected: OK.

- [ ] **Step 5: Commit**

```bash
git add tests/test_registry_consistency.py tests/CMakeLists.txt
git commit -m "test: enforce CMake registry and rov.toml agreement"
```

### Task 10: Suite-manifest drift guard

**Files:**
- Create: `tests/test_suite_manifest.py`
- Modify: `docs/sil_simulation_guide.md:193-215`

**Interfaces:**
- Consumes: the set of registered `test_*` names.
- Produces: a test that fails if the guide and CTest disagree.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_suite_manifest.py
import re, subprocess, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
class TestSuiteManifest(unittest.TestCase):
    def test_guide_lists_all_suites(self):
        out = subprocess.run(["ctest", "--test-dir", str(ROOT / "build-native"), "-N"], capture_output=True, text=True)
        ctest_names = {m.group(1) for line in out.stdout.splitlines() if (m := re.match(r"^\s*\d+: (test_\w+)", line))}
        guide = (ROOT / "docs" / "sil_simulation_guide.md").read_text()
        missing = {n for n in ctest_names if n not in guide}
        self.assertEqual(set(), missing, f"Guide missing suites: {missing}")
if __name__ == "__main__": unittest.main()
```

- [ ] **Step 2: Run to verify it fails**

Run: `python -m unittest tests.test_suite_manifest -v`
Expected: FAIL — the guide currently omits `test_services`.

- [ ] **Step 3: Minimal implementation**

Update `docs/sil_simulation_guide.md` §6 to list every registered `test_*` suite, and correct "25" to "26".

- [ ] **Step 4: Run to verify it passes**

Run: `python -m unittest tests.test_suite_manifest -v`
Expected: OK.

- [ ] **Step 5: Commit**

```bash
git add tests/test_suite_manifest.py docs/sil_simulation_guide.md
git commit -m "docs: correct suite count and add drift guard"
```

### Task 11: Coverage preset

**Files:**
- Modify: `CMakePresets.json`

**Interfaces:**
- Consumes: none.
- Produces: `cmake --preset coverage` / `ctest --preset coverage`.

- [ ] **Step 1: Write the failing check**

Run: `cmake --preset coverage`
Expected: FAIL — preset does not exist.

- [ ] **Step 2: Confirm**

Run the same command and observe the "Unknown preset" error.

- [ ] **Step 3: Minimal implementation**

Add to `CMakePresets.json`:

```json
{ "name": "coverage", "displayName": "Coverage",
  "inherits": "sil-debug",
  "cacheVariables": { "CMAKE_BUILD_TYPE": "Debug", "CMAKE_C_FLAGS": "--coverage -fprofile-arcs -ftest-coverage" },
  "testPreset": { "inherits": "sil-debug" } }
```

- [ ] **Step 4: Run to verify it passes**

Run: `cmake --preset coverage && cmake --build --preset coverage && ctest --preset coverage -E "^target_"`
Expected: PASS; `gcov` produces `.gcda` files in `build-native-coverage/Testing`.

- [ ] **Step 5: Commit**

```bash
git add CMakePresets.json
git commit -m "build: add coverage preset"
```

### Task 12: Full Unity conversion sweep

**Files:**
- Modify: all 26 host SIL targets in `tests/`:
  `test_can_protocol.c`, `test_pwm_ramp.c`, `test_safety.c`, `test_i2c_recovery.c`,
  `test_timesync.c`, `test_bsp.c`, `test_driver_*.c` (10 files), `test_mock_physics.c`,
  `test_node1_pi_shield.c`, `test_node2_control_board.c`, `test_node3_power_slab.c`,
  `test_multi_node_bus.c`, `test_services.c`, `test_sil_safety.c`, `test_sil_power.c`,
  `test_sil_fuzz.c`, `test_sil_burnin.c`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `ROV_TEST_MAIN`/`ROV_RUN_TEST` from Task 2, `rov_unity` from Task 1.
- Produces: zero `assert()` in host SIL; every case self-reporting; every target filterable via `ROV_TEST_FILTER`.

- [ ] **Step 1: Write the conversion recipe (the "test" for each file)**

For each file, mechanically:

1. Delete `#include <assert.h>` and every `assert(x);` / `assert(x && "msg");`.
2. Replace each `assert(cond);` with `TEST_ASSERT_TRUE_MESSAGE(cond, "<original message>");`.
3. Replace each `assert(a == b);` with `TEST_ASSERT_EQUAL_INT(a, b);` (or `_EQUAL_UINT`, `_EQUAL_STRING` as appropriate).
4. Replace the file's `static void setup(void) { ... }` with `void setUp(void) { ... }` and add `void tearDown(void) {}`.
5. Replace `int main(void) { ... }` with `ROV_TEST_MAIN() { UNITY_BEGIN(); ... return UNITY_END(); }`, wrapping each test call in `ROV_RUN_TEST(test_fn);`.

Example (from `test_node2_control_board.c`):

```c
/* Before */
static void setup(void) { mock_bsp_reset(); ... }
void test_node2_nominal(void) { setup(); assert(mock_can_get_tx_count() == 0); }
int main(void) { test_node2_nominal(); return 0; }

/* After */
void setUp(void) { mock_bsp_reset(); ... }
void tearDown(void) {}
static void test_node2_nominal(void) { TEST_ASSERT_EQUAL_INT(0, mock_can_get_tx_count()); }
ROV_TEST_MAIN() { UNITY_BEGIN(); ROV_RUN_TEST(test_node2_nominal); return UNITY_END(); }
```

- [ ] **Step 2: Convert one file and verify**

Convert `test_node2_control_board.c` first, build, and run it with and without `ROV_TEST_FILTER`. This is the exemplar that proves the recipe.

- [ ] **Step 3: Sweep the remaining 25**

Apply the identical recipe to the remaining 25 files, link each against `rov_unity`, and build.

- [ ] **Step 4: Run the full suite**

Run: `ctest --test-dir build-native -E "^target_" --output-on-failure`
Expected: all 26 PASS, with per-case output (one line per case, not one line per binary).

- [ ] **Step 5: Commit per logical group (not one mega-commit)**

```bash
git add tests/test_node*.c tests/CMakeLists.txt
git commit -m "test: convert node SIL suites to Unity"
git add tests/test_driver_*.c
git commit -m "test: convert driver suites to Unity"
git add tests/test_*remove.c tests/test_bsp.c tests/test_can_protocol.c tests/test_pwm_ramp.c tests/test_safety.c tests/test_i2c_recovery.c tests/test_timesync.c tests/test_mock_physics.c tests/test_services.c tests/test_sil_*.c
git commit -m "test: convert remaining host SIL suites to Unity"
```

### Task 13: Rewrite the SIL guide for the new workflow

**Files:**
- Modify: `docs/sil_simulation_guide.md`

**Interfaces:**
- Consumes: Tasks 4, 7, 10.
- Produces: accurate §7.4a, §7.6, §7.7 reflecting `build-native`, `--logical` default, and the drift guard.

- [ ] **Step 1: Write the failing check**

Run: `Select-String -Path docs/sil_simulation_guide.md -Pattern "sil-debug" -Context 0,2`
Expected: stale references to the old default build tree remain.

- [ ] **Step 2: Confirm**

Observe matches referencing `build/sil-debug` and the 3000 ms arming gate billed at 3 s wall.

- [ ] **Step 3: Minimal implementation**

Update §7.4a to state it applies in `--realtime` mode; note that `--logical` (the default for `rov.py test`/CI) decouples time entirely. Replace references to `build/sil-debug` with `build-native`. Update the suite count from 25 to 26 and note the drift guard.

- [ ] **Step 4: Run to verify it passes**

Run: `Select-String -Path docs/sil_simulation_guide.md -Pattern "sil-debug"`
Expected: no matches except in an explicit "legacy preset" note.

- [ ] **Step 5: Commit**

```bash
git add docs/sil_simulation_guide.md
git commit -m "docs: document logical clock, build-native, and drift guard"
```

---

## Spec Coverage Map

| Spec item | Task |
| --- | --- |
| L1 frozen | All tasks — no task edits `bsp.h`/`can_interface.h` |
| No `bsp.c`/`main.c` edits | All tasks — SP3 owns that, deferred |
| Physics untouched | No task modifies `mock_physics.c` semantics |
| No configure-time Python | Task 9 consistency is test-time; Task 8 docs; verified in Task 9 step 4 |
| §5.1 one clock, two modes | Task 3, Task 4 |
| §5.2 Unity, filter harness, coverage | Task 1, Task 2, Task 11, Task 12 |
| §5.3 node registry, no codegen, stale tree deleted | Task 5, Task 8, Task 9 |
| §5.4 sil_vehicle table | Task 6 |
| §5.5 CLI, build-native convergence | Task 7, Task 8, Task 13 |
| §5.6 drift guard | Task 10 |
| Seam 1 (port param) | Deferred to SP3; registry function shape in Task 5 already takes the parameters SP3 needs |
| Seam 2/3/4 | Deferred to SP3/SP4; not required for foundation correctness |

## Type-Consistency Notes

- `sil_node_t` fields are `name`, `can_node_id`, `init`, `step` — `can_node_id` is the same `ROV_NODE_*` value used by `mock_can_set_current_node`. Don't introduce a second enum.
- `x19_add_node_test` parses `NAME;NODE;SUITE;SOURCES;INCLUDES;LIBS` — Task 5 declares it, Task 7 consumes the resulting `LABELS`, Task 9 consumes the same labels. Use `;`-separated lists in CMake, never `&&`.
- `ROV_TEST_MAIN()` expands to `int main(void)`; do not wrap it in a function.
