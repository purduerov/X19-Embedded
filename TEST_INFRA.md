# End-to-End (E2E) Test Infrastructure Specification

Purdue ROV X19-Embedded Subsystem  
Document Version: 1.0.0  
Date: 2026-10-22  
Audience: Development Team, Orchestrator, Quality Assurance & Auditors  

---

## 1. Test Architecture Overview

The X19-Embedded E2E test infrastructure provides an independent, opaque-box, requirement-driven verification suite covering the vehicle build system, the consolidated developer CLI (`rov`), multi-target cross-compilation, host Software-in-the-Loop (SIL) simulation, and CAN multi-transport telemetry networks.

### Design Principles
- **Opaque-Box Verification**: Tests interact with the subsystem strictly through user-facing interfaces (the `python rov.py` CLI, external TCP sockets emulating the companion computer bridge, and CTest test runners). Tests do not rely on private internal helper functions.
- **Progressive Testability & Strict Mode**: Tests adapt during parallel development milestones. Tests for completed milestones run strict assertions; tests for in-flight milestones verify interface contracts and execute cleanly with `--strict` during final milestone verification (M5).
- **Zero External Dependencies**: Built entirely upon standard Python (`unittest`, `socket`, `struct`, `subprocess`, `tempfile`, `pathlib`) and standard CMake/CTest utilities, guaranteeing consistent execution on developer workstations and CI runners without requiring unbundled third-party packages.
- **Fail-Safe Invariant Enforcement**: Directly enforces critical vehicle safety rules (thruster neutral PWM at 1500 us, emergency break prioritization, and loss-of-heartbeat disarm).

```
tests/e2e/
├── __init__.py
├── e2e_context.py                   # Shared test harness, fixtures, mock SIL server, CAN packet codecs
├── run_all.py                       # Automated master test runner with structured reporting
├── tier1_features/                  # Tier 1: Functional Feature Coverage (R1-R4)
│   ├── test_r1_direct_file.py       # Positional .c file execution for build/run/flash
│   ├── test_r1_node_dispatch.py     # Positional node target execution without -n
│   ├── test_r1_default_fallback.py  # Default board/node fallback when target omitted
│   ├── test_r1_subcommand_removal.py# Verification that scan, sandbox, sync are eliminated
│   ├── test_r2_board_discovery.py   # Dynamic .ioc discovery in boards/ without toml edits
│   ├── test_r3_cross_compilation.py # Cross-compilation for f411, g474, and stm32c5 into valid ELFs
│   ├── test_r3_sil_ctest.py         # 100% pass on all 36 SIL CTest suites and scope filters
│   └── test_r4_can_transport.py     # Unified CAN multi-transport interface & 64-byte FD frames
├── tier2_boundaries/                # Tier 2: Boundary & Corner Cases
│   ├── test_nonexistent_file.py     # Missing .c files, deep paths, directory-as-file handling
│   ├── test_invalid_c_syntax.py     # Syntax errors, missing app_main(), #error, empty files
│   ├── test_nonexistent_board.py    # Unknown, empty, or malformed board targets
│   ├── test_unknown_node.py         # Unknown node names, typos, unsupported board/node combos
│   ├── test_missing_ioc.py          # Missing, empty, or corrupted .ioc files, .ioc.bak ignored
│   ├── test_port_collision.py       # Port collisions, refused connections, closed sockets
│   └── test_timeout_handling.py     # Serial monitor timeout, CAN recv timeout, burst buffers
├── tier3_combinations/              # Tier 3: Cross-Feature Combinations (Pairwise)
│   ├── test_direct_file_custom_board.py # Direct .c file combined with multiple hardware targets
│   ├── test_cli_flags_matrix.py     # Matrix of CLI commands x flags (--profile, -t, --suite)
│   └── test_can_sil_bridge.py       # Bidirectional CAN FD telemetry & commands over SIL TCP
└── tier4_real_world/                # Tier 4: Real-World Scenarios & Workflows
    ├── test_custom_board_full_workflow.py # Add board -> auto-discover -> generate -> compile
    ├── test_rapid_prototyping_workflow.py # Rapid algorithm scratchpad -> build -> test verify
    ├── test_safety_estop_workflow.py      # Active thruster PWM -> CAN E-stop disarm to 1500 us
    ├── test_telemetry_stream_workflow.py  # High-frequency Nav, Env, and Power telemetry decode
    └── test_ping_pong_workflow.py         # Node 1 ping -> Node 2 pong time-sync latency
```

---

## 2. Testing Methodology

The test suite applies four rigorous software testing methodologies:

### A. Category-Partition Method
Input domains for the CLI and CAN transport are decomposed into equivalence partitions:
1. **Target Types**: Positional `.c` scratch file (relative or absolute), valid node name (`pi_shield`, `control_board`, `power_slab`), omitted (default fallback), invalid string.
2. **Board Hardware**: Bench targets (`f411`, `g474`), production vehicle target (`stm32c5`), dynamically discovered custom board directory, missing/unknown board.
3. **Build Profiles**: `debug` (unoptimized, debug symbols enabled), `release` (optimized `-O3` / `-Os`).
4. **Transport Protocols**: Local TCP SIL bridge (`SilTcpTransport` on `127.0.0.1:8765`), physical Linux CAN (`SocketCanTransport` on `can0`).

### B. Boundary Value Analysis (BVA)
Tests probe exact boundaries across parameters:
- **CAN FD Payload Size**: 0 bytes (empty frame), 8 bytes (classic CAN payload), 64 bytes (maximum CAN FD DLC boundary), and 65 bytes (oversized error condition).
- **Timeouts**: `0.0s` (non-blocking immediate poll), `0.1s` (rapid timeout), negative timeouts (rejected with `ValueError`), and long socket readouts.
- **Port Numbers**: Port 0 (ephemeral auto-assignment), valid unprivileged range (1024-65535), out-of-range port numbers (< 0 or > 65535).
- **Source Code Boundaries**: Empty 0-byte `.c` file, file with preprocessor errors, file without `app_main()`.

### C. Pairwise Interaction Testing
Systematic cross-combination of independent features:
- Direct scratch file x Target board (`f411`, `g474`, `stm32c5`, host)
- Command x Flag matrix (`run -t`, `build -p release`, `test --node --suite`, `build -f -p release`)
- Network burst x TCP socket reconnect lifecycle

### D. Real-World Workload Simulation
End-to-end integration workflows modeling real subsea vehicle operational scenarios:
- **Rapid Prototyping**: Writing an algorithm in a scratch file, compiling for hardware bench, and verifying regression-free SIL test pass.
- **Safety Critical E-Stop**: Streaming active thruster pulses and triggering CAN ID `0x001` (Emergency Break) to verify immediate neutral disarm (1500 us).
- **Telemetry Streaming**: Processing high-frequency subsea data streams (100 Hz Nav, 10 Hz Env, 20 Hz Power) through decoding pipelines.
- **Multi-Node Time Synchronization**: Roundtrip time-sync ping (0x011) and pong (0x012) between Node 1 and Node 2.

---

## 3. Feature Inventory Mapping

Every feature from `PROJECT.md` and requirement from `ORIGINAL_REQUEST.md` is mapped to automated test suites:

| Req | Feature # | Description | Primary Test Module | Tier |
|-----|-----------|-------------|---------------------|------|
| R1  | 4         | Eliminate Redundant CLI Subcommands (scan, sandbox, sync) | `tier1_features/test_r1_subcommand_removal.py` | Tier 1 |
| R1  | 5         | Direct Positional Target Dispatch (`rov run/build/flash <file.c>`) | `tier1_features/test_r1_direct_file.py` | Tier 1 |
| R1  | 5         | Direct Positional Target Dispatch (`rov run [node]`) | `tier1_features/test_r1_node_dispatch.py` | Tier 1 |
| R1  | 5         | Default Target & Board Fallback When Omitted | `tier1_features/test_r1_default_fallback.py` | Tier 1 |
| R2  | 7         | Frictionless Custom Board & `.ioc` Discovery | `tier1_features/test_r2_board_discovery.py` | Tier 1 |
| R2  | 8         | Auto-Scaffold Board `CMakeLists.txt` on Generate | `tier1_features/test_r2_board_discovery.py` | Tier 1 |
| R2  | 9         | Target Custom Board with Direct File Execution | `tier3_combinations/test_direct_file_custom_board.py` | Tier 3 |
| R3  | 1         | STM32C542 Target Decoupling & Scaffolding | `tier1_features/test_r3_cross_compilation.py` | Tier 1 |
| R3  | 2         | Target Collision & Header Resolution (`app.h`) | `tier1_features/test_r3_cross_compilation.py` | Tier 1 |
| R3  | 3         | Clean Cross-Compilation for all Targets (g474, f411, stm32c5) | `tier1_features/test_r3_cross_compilation.py` | Tier 1 |
| R3  | 6         | Fix `rov test` Execution Scope (prevent node clobbering) | `tier1_features/test_r3_sil_ctest.py` | Tier 1 |
| R3  | 6         | 100% Pass Rate Across All 36 Host SIL CTest Suites | `tier1_features/test_r3_sil_ctest.py` | Tier 1 |
| R4  | 10        | Consolidated CAN Testing Workflow | `tier1_features/test_r4_can_transport.py` | Tier 1 |
| R4  | 11        | Multi-Transport CAN Support (`SocketCanTransport`, `SilTcpTransport`) | `tier1_features/test_r4_can_transport.py` | Tier 1 |
| R4  | 12        | Remove Artificial SIL Blockers & Wire Framing (`ROVC` / `X19C`) | `tier1_features/test_r4_can_transport.py` | Tier 1 |
| R4  | 14        | Multi-Node CAN FD Protocol Verification | `tier4_real_world/test_ping_pong_workflow.py` | Tier 4 |
| -   | -         | Non-Existent File & Directory Handling | `tier2_boundaries/test_nonexistent_file.py` | Tier 2 |
| -   | -         | Invalid C Syntax, Missing Symbols, and Link Failures | `tier2_boundaries/test_invalid_c_syntax.py` | Tier 2 |
| -   | -         | Non-Existent & Malformed Board Names | `tier2_boundaries/test_nonexistent_board.py` | Tier 2 |
| -   | -         | Unknown Node Names & Incompatible Board Mappings | `tier2_boundaries/test_unknown_node.py` | Tier 2 |
| -   | -         | Missing & Corrupt `.ioc` Files, Backup File Filtering | `tier2_boundaries/test_missing_ioc.py` | Tier 2 |
| -   | -         | TCP Port Collisions, Closed Sockets, Network Timeouts | `tier2_boundaries/test_port_collision.py` | Tier 2 |
| -   | -         | Serial Monitor & CAN Receive Timeout Enforcement | `tier2_boundaries/test_timeout_handling.py` | Tier 2 |
| -   | -         | CLI Command & Flag Combinations Matrix | `tier3_combinations/test_cli_flags_matrix.py` | Tier 3 |
| -   | -         | Bidirectional CAN Telemetry & Commands over SIL TCP | `tier3_combinations/test_can_sil_bridge.py` | Tier 3 |
| -   | -         | End-to-End Custom Board Addition & Compilation Lifecycle | `tier4_real_world/test_custom_board_full_workflow.py` | Tier 4 |
| -   | -         | Rapid Algorithm Prototyping & SIL Regression Verification | `tier4_real_world/test_rapid_prototyping_workflow.py` | Tier 4 |
| -   | -         | Subsea Safety E-Stop & 1500 us Thruster Neutral Invariant | `tier4_real_world/test_safety_estop_workflow.py` | Tier 4 |
| -   | -         | High-Frequency Nav, Env, and Power Telemetry Decoding | `tier4_real_world/test_telemetry_stream_workflow.py` | Tier 4 |

---

## 4. Test Execution & Reporting

### Master Test Runner
The automated master test runner `tests/e2e/run_all.py` executes the entire suite and formats results into clean summary tables:

```bash
# Execute entire E2E test suite across all 4 tiers:
python tests/e2e/run_all.py

# Execute specific tier:
python tests/e2e/run_all.py --tier 1
python tests/e2e/run_all.py --tier 2
python tests/e2e/run_all.py --tier 3
python tests/e2e/run_all.py --tier 4

# Run with verbose test-by-test output:
python tests/e2e/run_all.py -v

# Run with filter pattern:
python tests/e2e/run_all.py -k "cross_compile"
python tests/e2e/run_all.py -k "sil_tcp"

# Run in strict mode (fails on any unimplemented milestone):
python tests/e2e/run_all.py --strict

# Output machine-readable JSON summary for CI pipelines:
python tests/e2e/run_all.py --json
```

### Direct Unittest Invocation
Any test file or test class can be executed independently using standard Python `unittest`:

```bash
python -m unittest tests.e2e.tier1_features.test_r3_cross_compilation -v
python -m unittest tests.e2e.tier2_boundaries.test_timeout_handling -v
python -m unittest tests.e2e.tier3_combinations.test_can_sil_bridge -v
python -m unittest tests.e2e.tier4_real_world.test_safety_estop_workflow -v
```

---

## 5. Authoritative Expected Output Derivation

For every test case, expected outputs are derived from explicit, immutable system contracts:
1. **ELF Binary Header Invariants**: Checked via 32-bit ELF specifications (`e_ident[EI_MAG0..3] == 0x7F 'ELF'`, `EI_CLASS == 1`, `EI_DATA == 1`, `e_machine == 40` (ARM Architecture)).
2. **SIL CAN Packet Wire Format**: Derived from `tests/sil_bridge_server.c` struct `sil_can_packet_t` (73-byte wire format: 4-byte `ROVC` header, 4-byte arbitration ID, 1-byte length, 64-byte payload).
3. **Subsea Safety Invariants**: Derived from vehicle requirements in `GEMINI.md` and `PROJECT.md`:
   - Thruster neutral PWM is strictly 1500 us.
   - CAN ID `0x001` (Emergency Break) preempts all thruster commands.
   - Heartbeat loss (>250ms) disarms thruster outputs.
4. **Host SIL CTest Oracle**: Verified against the 36 unit and integration test suites defined in `tests/CMakeLists.txt`.

---

## 6. Audit & Test Integrity Compliance

In compliance with the project's testing integrity standards:
- **No Facade Tests**: Every test executes real system processes, real ARM cross-compiler invocations, real TCP network sockets, or real file generation.
- **No Tautological Assertions**: Every assert tests concrete system state (exit codes, generated binary headers, decoded packet fields, exact error messages).
- **Independent & Isolated**: Each test runs in its own context, creates temporary scratch files or boards, and automatically cleans up upon completion.
