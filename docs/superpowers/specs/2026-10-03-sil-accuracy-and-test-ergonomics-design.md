# SIL Accuracy and Test Ergonomics — Design

**Date:** 2026-10-03
**Status:** Approved design, awaiting spec review
**Scope:** Roadmap for all four sub-projects; detailed design for the foundation (sub-projects 1 + 2)

---

## 1. Problem statement

The host Software-in-the-Loop (SIL) environment is already more capable than
most embedded projects reach, but its fidelity is **unevenly distributed** and
its ergonomics make single-node work slower than it needs to be.

Four goals, in the order they were identified as painful:

1. **Determinism** — tests whose results do not depend on host load, and which
   fail reproducibly enough to bisect.
2. **Coverage of the HAL/BSP layer** — prove the layer that actually configures
   silicon, rather than mocking past it.
3. **Trustworthiness** — no false greens; a pass must be earned and a skip must
   never read as a pass.
4. **Fidelity** — catch bugs that only real hardware would show.

Plus a cross-cutting ergonomics goal: **testing a single node should be a
one-flag operation.**

### 1.1 Verified current state

All claims below were read from the source, not inferred.

**Fidelity is high in one place and low in others.**

| Layer | Fidelity | Evidence |
| --- | --- | --- |
| BME280 | Genuinely excellent. Inverse datasheet model; bisection-solves the raw ADC words that would produce a requested engineering value. | `tests/mocks/mock_bme280.c:217-277` |
| CAN bus | Structurally real but has no physics. FIFOs, broadcast, bus-off, TX-fail, drop count, TX history. No bit timing, arbitration, ACK, error frames, or TEC/REC error confinement. | `tests/mocks/mock_can.c` |
| I2C | Half a stub. `bsp_i2c_mem_read`/`write` hit a real 128x256 register file, but `bsp_i2c_write` is a no-op returning `true`, `bsp_i2c_read` fills zeros, `bsp_i2c_probe` **always returns `false`**, `bsp_i2c_scan` returns 0. | `tests/mocks/mock_bsp.c:302-330` |
| MS5837, INA226/237, TMP1075, TPS25990, IMUs | Injected numbers, no device models. Pure get/set structs. | `tests/mocks/mock_sensors.c` |
| HAL / `bsp.c` | Never in the loop. Host SIL mocks `bsp_*` directly, so the real `nodes/*/Core/Src/bsp.c` never executes. | `shared/src/rov_bsp_stub.c` weak symbols overridden by `tests/mocks/mock_bsp.c` |
| Physics | One invented model. 18.5 kg, 35 N quadratic thrust, `norm^1.8` current draw. Constants are not datasheet-derived. The guide states +2.1 N buoyancy; the code computes ~2.58 N from 184 N. | `tests/mocks/mock_physics.c:16-48`; `docs/sil_simulation_guide.md:171` |
| Clock | Welded to wall clock in the live path: virtual time advances 10 ms, then the host sleeps 10 ms. | `tests/sil_bridge_server.c:323` and `:488` |

**Two clocks can disagree.** `tests/hardware/fakes/fake_hal.c:109-115` keeps its
own `s_tick_ms`, advanced by `HAL_Delay`, independent of `mock_bsp.c`'s
`g_mock_time_us`. `HAL_GetTick()` and `time_get_ms()` can therefore report
different times within one run. This is a latent defect, not a hypothetical.

**There is no test framework.** All 26 `test_*` executables are hand-written
`int main(void)` calling `void test_*()` helpers with bare `assert()`. A repo-wide
search for Unity, CMocka, Ceedling, GoogleTest, Catch2, and doctest returns no
matches. Consequences: `assert()` aborts on first failure so later failures are
never reported, and CTest cannot select an individual case.

**`tests/CMakeLists.txt` is 349 lines of near-copy-paste.** Five targets
(`test_multi_node_bus`, `test_sil_safety`, `test_sil_power`, `test_sil_fuzz`,
`test_sil_burnin`) each repeat the same four `app.c` paths and three include
directories.

**`rov.toml` is not read by CMake.** It is consumed only by `tools/rov.py` via
`tomllib`/`tomli`. Meanwhile `rov.toml:79-125` already declares every node.

**There is a stale committed build tree.** `tests/build/` contains hardcoded
paths to a different machine (`C:/Users/aman/Documents/Programming/X19-ROV/...`)
and predates four currently-registered tests.

**Documentation has already drifted.** `docs/sil_simulation_guide.md:193-215`
declares "25 Test Suites" and enumerates 25. There are 26 registered `test_*`
targets; `test_services` (`tests/CMakeLists.txt:170-179`) is missing from the
table.

**`rov.py test` cannot reach the `target_*` contracts.** The `sil-debug` test
preset filters out `^target_` (`CMakePresets.json`) before the user's `-R` is
applied, so the readiness contracts are unreachable through the CLI.

**Two build trees are in use.** `rov.py test` builds `build/sil-debug/`, while
`AGENTS.md`, `README.md`, and CI use `build-native/`.

### 1.2 The load-bearing discovery

**All three node `app.c` files include zero HAL headers.** They include only
`app.h`, `bsp.h`, `can_interface.h`, driver headers, service headers, `rov_*.h`,
and libc. The HAL-heavy `main.h` / `stm32g4xx_hal_conf.h` are reachable only
from `bsp.c` and `main.c`.

The L1 boundary is therefore **already clean**. Sub-project 3 can compile the
real `bsp.c` into host tests without touching application code — and because
that only *compiles* `bsp.c` rather than editing it, the "do not modify
teammate-authored code" constraint does not block it.

Node 2 is structurally different from nodes 1 and 3: it is a full CubeMX project
(553-line generated `main.c`, its own `Drivers/CMSIS` and `Drivers/BSP`, a real
`stm32g4xx_hal_conf.h`), whereas nodes 1 and 3 have 26-line hand-written
`main.c` stubs. The L0 simulator must satisfy three different `bsp.c`
implementations.

---

## 2. Goals and non-goals

### Goals

- G1. Host SIL results are independent of host load; failures reproduce.
- G2. The real `bsp.c` executes under test, and its findings are reported.
- G3. No test can report success without having actually run; skips are visible.
- G4. Missing device and bus fidelity is modelled rather than stubbed.
- G5. `rov.py test --node <node>` runs exactly that node's tests.

### Non-goals

- **N1. `bsp.h` and `can_interface.h` stay byte-identical.** Every accuracy gain
  comes from below the contract, never from changing it. This is what keeps the
  work free of new flight risk and preserves the meaning of existing tests.
- **N2. Do not edit `nodes/*/Core/Src/bsp.c` or `main.c`.** These carry commits
  from `Rex` and `Tanay Ubale`, and `AGENTS.md` restricts us to review feedback
  on teammate-authored code. Sub-project 3 therefore **reports** BSP findings
  rather than fixing them.
- **N3. Do not rename per-node app symbols** (`node1_app_init` etc.) to a
  uniform name. That would edit teammate-authored files.
- **N4. Physics accuracy is out of scope for this roadmap.** Held for a separate
  spec against real T200 datasheet curves.
- **N5. Do not introduce Python as a CMake configure-time dependency.**

---

## 3. Architecture: four layers

The SIL currently has a *mock-past-the-BSP* shape. Everything below the
application layer is replaced wholesale, and the real `bsp.c` never runs:

```
app.c  --calls-->  bsp_* / can_*   -->  [rov_mocks: strong overrides]
                                              ^
                                weak stubs in rov_bsp_stub.c
```

That is why `target_bsp_node*` exists as a **separate universe** with its own
`fake_hal.c`, its own include-shadowing trick (`-UROV_UNIT_TEST`,
`hardware/fakes` first on the include path), and its own set of stubs. Two
parallel universes, two sets of fakes, and a seam between them held together by
preprocessor conditionals.

The roadmap replaces this with one stack:

```
+-------------------------------------------------------------+
| L4  Vehicle     sil_bridge_server, stimulus tools, UI      |  integration
+-------------------------------------------------------------+
| L3  Nodes       node1/2/3 app.c + power_sequence.c         |  per-node SIL  <- SP2
+-------------------------------------------------------------+
| L2  Services    actuator / safety / env / power            |  already exists
+-------------------------------------------------------------+
| L1  Ports       bsp.h, can_interface.h   (FROZEN, N1)     |  THE CONTRACT
+-------------------------------------------------------------+
| L0  Silicon     one deterministic HAL simulator            |  <- SP3
|                 GPIO, TIM, FDCAN, I2C, ADC, IWDG           |
+-------------------------------------------------------------+
```

Two consequences follow:

- **`target_bsp_node*` stops being a separate universe.** It becomes "the same L0
  simulator, running the real `bsp.c`." `tests/hardware/fakes/` and the `-U`
  flag gymnastics are deleted. One fake HAL serves both host SIL and the
  readiness contracts.
- **Sub-project 4's I2C work is not a new abstraction.** Today
  `bsp_i2c_probe` hardcodes `false` and `bsp_i2c_read` fills zeros because there
  is no I2C controller to ask. Making L0 real makes those honest calls into a
  device bus. The "honest I2C transaction layer" is a *consequence* of SP3, not
  an independent project.

---

## 4. Roadmap

Four sub-projects, strictly ordered. SP1 and SP2 are independent of each other
and both unblocked; they are delivered together because they edit the same files
and because `--node` filtering is only useful once per-case selection exists.

| SP | Name | Delivers | Blocked by |
| --- | --- | --- | --- |
| 1 | Determinism + test framework | Logical clock, Unity harness, per-case selection and reporting, coverage preset | nothing |
| 2 | Single-node ergonomics | Node registry, node table, `rov.py test --node`, `rov.py sim`, CTest labels, `build-native` convergence | nothing |
| 3 | HAL/BSP coverage | L0 simulator, real `bsp.c` under host test, per-node BSP findings report, `hardware/fakes/` retired | 1, 2 |
| 4 | Device + bus fidelity | Honest I2C transaction layer, inverse device models, CAN error model | 3 |

### 4.1 Sub-project interfaces (the seams)

The foundation's most important job is placing four seams correctly. Get these
wrong and SP3/SP4 need rework.

**Seam 1 — Ports become a parameter, not an assumption.**
Every node test currently hardcodes `target_link_libraries(... rov_mocks)`. SP3
needs the same test to link the real `bsp.c` plus L0 instead. So the registry
function takes the port as an argument. SP1/SP2 ship `MOCK` only; SP3 adds
`HAL_SIM`. Retrofitting the parameter later means touching every node test
twice.

**Seam 2 — Device models attach to a bus and are verified separately.**
`mock_bme280.c` is the best existing asset in the repo, and its header already
states the correct rule: the model answers "what raw words would a real part
produce," and it is verified in `test_driver_bme280.c` **separately** from the
driver test that consumes it. SP4 standardises that shape, plus fault injection
mirroring `mock_can`'s existing `set_bus_off` / `set_tx_fail` / `set_drop_count`.

**Seam 3 — The `0x7FE` output-status readback becomes port-sourced.**
`0x7FE` is a SIL-only channel reading `mock_bsp`'s recorded actuator state.
Once real `bsp.c` runs, `mock_bsp` is no longer where the actuators live — L0
is. The readback must be sourced from whichever port is linked, through a
single accessor, so that `tools/can_stimulus.py`'s receive-only checks keep
working unchanged across both ports.

**Seam 4 — Failure reporting is uniform.**
Unity for C, `unittest` for Python, CTest for exit codes. One rule: **a skip is
not a pass, and a pass must be earned.** `docs/sil_simulation_guide.md` §7.7
already argues this for Python (CI asserts on `ok` count *and* zero skips
rather than trusting the exit code); SP1 extends the same discipline to C.

---

## 5. Foundation design (SP1 + SP2)

### 5.1 One clock, two scheduling modes

`mock_bsp.c` already owns a virtual clock (`g_mock_time_us`). The defect is that
`sil_bridge_server.c` advances it and then sleeps the same amount:

```c
mock_bsp_advance_time_ms(10);   /* :323 virtual time */
platform_sleep_ms(10);          /* :488 wall time    <-- the coupling */
```

Extract this into an explicit scheduler with two modes:

| Mode | Behaviour | Used by |
| --- | --- | --- |
| `LOGICAL` (default) | Advance virtual time by exactly `dt`; never sleep. Runs at CPU speed. | tests, CI, `rov.py sim` |
| `REALTIME` | Sleep to hold the wall clock, for watching the dashboard. | `rov.py sim --realtime`, manual inspection |

`dt` becomes a parameter (default 10 ms) rather than a literal duplicated in two
places. The 3000 ms ESC arming gate that currently costs 3 s of wall time drops
to ~3000 compute steps. This is most of the 3.5-minute Python suite.

Physics continues to step inside `mock_bsp_advance_time_ms`, so the plant and
the firmware always observe the same `dt`. That is already correct and must be
preserved.

`docs/sil_simulation_guide.md` §7.4a ("a check can fail for a reason that is not
the vehicle") is **correct in `REALTIME` mode and is not being deleted.** It
will be rewritten to name the mode it applies to, rather than removing a true
warning whose cause is only half removed.

The foundation introduces `tests/harness/sil_clock.h` as the single clock and
routes `mock_bsp` through it. It does **not** rewire `tests/hardware/fakes/fake_hal.c`,
because that file is deleted in SP3 and touching it now is pure churn.

### 5.2 Unity

Vendored to `tests/vendor/unity/`, exposed as a `rov_unity` static library. All
26 host SIL `test_*` targets convert (the 6 `target_*` readiness contracts keep
their bespoke `CHECK` macro for now — they are red and SP3 supersedes them):
`setUp`/`tearDown` replace the per-file `static void setup(void)`, and
`TEST_ASSERT_*` replaces `assert()`. The primary gain is behavioural:
`TEST_ASSERT_*` does **not** abort, so one run reports every failure instead of
only the first.

**Stock Unity cannot select a single test by name.** This is a property of
Unity, not something the design can work around. Unity explicitly permits
replacing `UnityDefaultTestRun` with a user-supplied `main()`, which is the
conventional solution. `tests/harness/rov_test_main.h` therefore provides
`ROV_TEST_MAIN()` and a `ROV_RUN_TEST(fn)` macro that honours an
`ROV_TEST_FILTER` environment variable, exposing `rov.py test -k <case>`.
Approximately 40 lines of glue.

**Unity is a new vendored dependency** in a repository whose `requirements.txt`
deliberately declares almost nothing (`pyserial`, `tomli`). This was an
explicit decision.

A `--coverage` preset (`-ftest-coverage -fprofile-arcs`) is added. Off by
default because it slows the build, but it yields a machine-generated number in
place of a hand-maintained suite count.

**Sequencing within SP1:** land the framework plus one exemplar node test,
verify it, then perform the mechanical sweep across the remaining 25.
Half-converted would leave two conventions coexisting.

### 5.3 Node registry (no code generation)

CMake has no TOML parser, so "make `rov.toml` authoritative" requires a
mechanism. Two options were considered:

- *Checked-in generated manifest.* `tools/gen_test_manifest.py` reads
  `rov.toml` and emits `tests/cmake/nodes_generated.cmake` which CMake
  `include()`s, with a staleness check. **Rejected** in favour of the option
  below: it adds a generated file to the repository for no benefit, given N5.
- *Pure-CMake registry plus a consistency test.* **Chosen.**

Design: nodes are declared once in a CMake registry. `rov.toml` remains
authoritative for the Python CLI (it already is). Two sources exist, and one
enforced invariant keeps them honest: a test asserts that the CMake registry's
node keys and `supported_boards` match `rov.toml`. Drift becomes a test failure
rather than a silent divergence — weaker than codegen, and deliberately so,
because the cost is one small test.

The consistency check is a Python test registered with CTest, not a
configure-time step, so `cmake -B build-native` still works without Python on
`PATH` (N5). It runs under `rov.py test` and in CI.

`tests/build/` is deleted as part of this work. It is a stale committed tree
containing paths to a different machine and predates four current tests.

### 5.4 A node table instead of three bare externs

`sil_bridge_server.c:64-72` hand-declares each node, and the scheduler order
Node2 → Node1 → Node3 is hardcoded in the loop body. These collapse into one
table in `tests/harness/sil_vehicle.h`:

```c
typedef struct {
    const char *name;        /* "pi_shield" - matches the rov.toml key */
    uint8_t     can_node_id; /* ROV_NODE_* */
    void      (*init)(void);
    void      (*step)(void);
} sil_node_t;
```

Per-node symbols keep their existing distinct names (`node1_app_init` vs
`node2_app_init`) per N3. The benefit is that adding a fourth node becomes one
table row plus one registry entry, rather than 20 CMake lines and 3 externs.

Step order becomes array order. **The existing Node2 → Node1 → Node3 order is
preserved exactly** and documented, because changing it could shift the
telemetry timing that existing tests assert against. Making the order visible in
one place is the win; changing it is not.

### 5.5 CLI surface

```
rov.py test                              # all host SIL (as today)
rov.py test --node control_board         # exactly that node's tests
rov.py test --suite app|driver|shared    # filter by kind
rov.py test -k heartbeat                 # one Unity case, via ROV_TEST_FILTER
rov.py test --target                     # the target_* readiness contracts
rov.py sim --node control_board          # headless engine, logical clock
rov.py sim --realtime                    # dashboard-friendly
```

Node filtering uses **native CTest labels** (`LABELS "node:control_board"` →
`ctest -L`), not hand-rolled string matching.

Two gaps this closes:

1. `rov.py test` currently **cannot reach** the `target_*` contracts, because the
   preset filters out `^target_` before `-R` is applied. `--target` fixes it.
2. `rov.py test` builds `build/sil-debug/` while `AGENTS.md`, `README.md`, and CI
   use `build-native/`. These converge on **`build-native`, configured with
   plain `cmake -B build-native -G Ninja`** — the exact invocation `AGENTS.md`
   already documents. `rov.py test` therefore drops its
   `cmake --preset sil-debug` dependency and calls plain
   `cmake` / `cmake --build` / `ctest --test-dir build-native`.

   Note that the two host presets cannot both be re-pointed at `build-native`,
   because a preset's `binaryDir` is fixed and `sil-debug` and `sil-release`
   differ only in build type. Rather than invent a third tree, the host presets
   stop being the default path: `CMakePresets.json` retains them for anyone who
   wants debug/release separation in their own directories, and `rov.py` grows
   `--preset <name>` as an explicit escape hatch. CI and documentation are
   updated to name `build-native` everywhere.

### 5.6 Suite-manifest drift guard

Beyond correcting the "25 suites" figure, a `test_suite_manifest` CTest test
asserts the registered target list matches a documented manifest, so the count
cannot silently rot again.

---

## 6. Explicitly out of scope for the foundation

- Does not touch `nodes/*/Core/Src/bsp.c` or `main.c`. Reads all three; edits none.
- Does not delete `tests/hardware/fakes/`. That is SP3's work; deleting it early
  breaks the red readiness contracts without replacing them.
- Does not model devices or CAN errors. Seam 2 exists so SP4 lands cleanly; the
  models land in SP4.
- Does not touch physics (N4).
- Does not change L1 (N1).

---

## 7. Known costs

| Cost | Rationale |
| --- | --- |
| Vendored Unity, ~600 lines to maintain | Buys non-aborting assertions and per-case selection. Two of the four goals are unreachable without it. |
| Converting 26 host SIL targets is the bulk of SP1 | Framework plus one exemplar lands first, then the sweep, to avoid two coexisting conventions. |
| `rov.py` gains flags; CI and docs change for build-tree convergence | "Works locally" and "works in CI" become the same tree. |
| Two sources of truth for nodes, one enforced invariant | Simpler than codegen; drift is loud rather than silent. |
| SP3 must satisfy three different `bsp.c` HAL surfaces | Node 2 is a full CubeMX project; nodes 1 and 3 are hand-written stubs. |

---

## 8. Open items deferred to later specs

- Whether L0 should model ADC, given no current code path uses it.
- Real T200 thrust and current curves to replace the invented physics constants,
  and reconciling the +2.1 N vs ~2.58 N buoyancy discrepancy.
- Whether a uniform `rov_app_init` symbol is worth the teammate-file churn that
  N3 currently forbids.
- Whether CMock's generated mocks are used in SP4, or whether hand-written
  device models suffice.
