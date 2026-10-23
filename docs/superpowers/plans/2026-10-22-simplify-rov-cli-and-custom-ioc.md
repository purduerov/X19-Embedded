# Implementation Plan: Simplify ROV CLI & Streamline Custom .ioc Workflow

**Date:** 2026-10-22  
**Goal:** Consolidate the `rov` CLI to eliminate over-engineering, allow passing `.c` files directly to `run`/`flash`/`build` (without needing `-f` or `-n sandbox`), and make adding custom board `.ioc` files effortless.

---

## 1. Problem & Architecture Review

Currently, `rov` suffers from unnecessary layers of indirection and bloated subcommands:
1. **Too many fragmented subcommands:** `sandbox`, `scan`, `sync`, `generate`, `build`, `flash`, `run`, `monitor`, `test`, `devices`, `check`.
   * For example, `rov scan` is just a hardcoded shortcut to `rov sandbox -f sandbox/i2c_scanner.c`.
   * `rov sandbox -f foo.c` forces developers to remember an arbitrary "sandbox" concept when they simply want to compile and run `foo.c`.
2. **Awkward File Execution:** Developers cannot just do `python rov.py run my_code.c`. They have to pass flags like `-f sandbox/my_code.c` or define arbitrary nodes in `rov.toml`.
3. **Rigid Board Configurations:** Adding a custom board or a new `.ioc` configuration historically required editing `rov.toml` across multiple node mappings (`supported_boards`, `target_*`, `binary_*`).

---

## 2. Target Developer Experience

We want `rov` to feel as simple and fast as running standard developer tools:

```powershell
# 1. Run any C file directly on the auto-detected board (builds, flashes, opens serial monitor):
python rov.py run my_code.c
python rov.py run sandbox/multinode_can.c
python rov.py run my_code.c -b g474

# 2. Build or Flash any C file without opening the serial monitor:
python rov.py build my_code.c
python rov.py flash my_code.c -s <SERIAL_NUMBER>

# 3. Run or flash standard named nodes:
python rov.py run control_board
python rov.py flash pi_shield -b g474

# 4. Zero-argument run:
python rov.py run          # Runs the default node for the connected board

# 5. Adding any new .ioc board:
# Just create boards/<new_board>/<new_board>.ioc!
python rov.py generate <new_board>
python rov.py run my_code.c -b <new_board>
```

---

## 3. Consolidated CLI Architecture

### Commands Kept & Streamlined (7 commands total):
1. **`run [target]`**: Build, flash, and open live serial monitor (where `target` can be `path/to/file.c`, a node name, or omitted).
2. **`build [target]`**: Compile only (produces `.elf`).
3. **`flash [target]`**: Compile and flash to MCU Flash via ST-Link.
4. **`monitor`**: Open serial monitor (auto-detects COM port or accepts `--port`).
5. **`devices`**: List connected ST-Link USB probes and COM ports.
6. **`test`**: Run host SIL CTest test suites.
7. **`generate [target]`**: Regenerate peripheral drivers via STM32CubeMX CLI.

### Subcommands Removed / Merged:
* `sandbox` $\to$ Merged into `run`, `build`, `flash` (passing a `.c` file runs it directly).
* `scan` $\to$ Replaced by `python rov.py run sandbox/i2c_scanner.c`.
* `sync` $\to$ Dynamic auto-discovery in `config.py` already resolves boards on the fly; no manual sync needed.
* `check` $\to$ Kept as a flag or lightweight diagnostic helper.

---

## 4. Implementation Tasks

### Task 1: Direct File Target Resolution in `tools/rov_cli/build.py`
- Modify `build_target` to accept `target: str | None` as its primary positional specifier.
- If `target` ends with `.c` (or is a valid file path on disk):
  - Automatically configure CMake with `-DSANDBOX_FILE=<absolute_path>`.
  - Compile the board's dynamic sandbox target (`<board>_sandbox.elf`).
- If `target` is a known node (`control_board`, `pi_shield`, `power_slab`):
  - Compile that node for the selected board.
- If `target` is omitted:
  - Default based on the board (e.g. `g474` $\to$ `control_board`, `f411` $\to$ `pi_shield`).
- **Verification:** Run `python rov.py build sandbox/multinode_can.c -b g474` and verify it compiles `g474_sandbox.elf`.

### Task 2: Simplify CLI Parser in `tools/rov_cli/cli.py`
- Update `run`, `build`, and `flash` subparsers to accept a single optional positional argument: `target` (e.g. `[target]`).
- Deprecate/remove separate `sandbox` and `scan` subparsers.
- Ensure `-b/--board`, `-s/--serial-number`, and `-p/--port` remain available across all flash/run commands.
- Provide clear help examples showing:
  - `rov run my_code.c`
  - `rov run control_board`
  - `rov flash my_code.c`
- **Verification:** Run `python rov.py --help` and verify the simplified command list.

### Task 3: Zero-Boilerplate Board Inclusion in `boards/CMakeLists.txt`
- Ensure `boards/CMakeLists.txt` uses globbing to discover all subdirectories in `boards/` that contain a `CMakeLists.txt`.
- For newly generated CubeMX boards, allow them to participate in CMake builds immediately upon driver generation.
- **Verification:** Verify `cmake --preset arm-c542-debug` runs with 0 errors.

### Task 4: Documentation & Sanity Verification
- Update `README.md` and `GEMINI.md` to document the streamlined commands:
  - `python rov.py run <file.c>`
  - `python rov.py build <file.c>`
  - `python rov.py flash <file.c>`
- Run the full CTest test suite (`python rov.py test`) to ensure no regressions.
- Verify building multiple targets (`control_board`, `multinode_can.c`) on `g474` and `f411`.
