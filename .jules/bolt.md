## 2026-03-31 - Table-Driven CRC16 CCITT Computation
**Learning:** Bit-by-bit CRC calculation loops in Python introduce significant interpreter overhead per bit shift/XOR operation. Using a 256-entry precomputed lookup table replaces 8 loop iterations per byte with a single indexing operation.
**Action:** Use precomputed 256-entry lookup tables for CRC computations in Python host/flasher utilities to achieve ~8.7x speedups.

## 2025-05-21 - Cache Time String Formatting in High-Throughput Streams
**Learning:** Re-formatting timestamps via `time.strftime` and `time.localtime` on every message in high-frequency data streams (e.g., 100 Hz CAN telemetry) incurs significant overhead due to repeated conversions and string allocations for duplicate second values.
**Action:** Cache the formatted time string based on integer seconds (`int(timestamp)`) and reuse the cached string when subsequent messages fall within the same second.

## 2024-05-18 - FPU Math Optimization

**Learning:** On Cortex-M microcontrollers with single-precision FPUs (like STM32 Cortex-M33 / M4F), floating-point division is computationally expensive (often taking ~14 cycles), whereas multiplication is very fast (1-3 cycles). Also, when formatting C code recursively (`find ... | xargs clang-format`), it is extremely important to exclude third-party vendor SDKs (e.g., `Drivers/CMSIS`, `Drivers/STM32F4xx_HAL_Driver`), as modifying these creates large diffs and breaks upstream update compatibility.
**Action:** When performing repeated math operations (e.g., quaternion normalization), compute the inverse of the divisor once (using 1 division) and multiply it for the remaining components. When formatting, apply `clang-format` only to first-party source files (e.g., specific files like `drivers/src/lsm6dsoxtr.c`) to avoid accidentally modifying external libraries.

## 2026-09-07 - Optimize MS5837 Depth Calculation
**Learning:** FPU division on Cortex-M4 takes ~14 cycles compared to 1 cycle for multiplication. For high-frequency calculations where the divisor is static (like fluid density in depth formulas), caching the inverse avoids repetitive division penalties.
**Action:** Always precompute inverses for static divisors in embedded loops to utilize hardware multiplication.

## 2026-09-07 - Hoisting Loop Invariants in High-Frequency Loops
**Learning:** In high-frequency control loops (e.g., 1kHz PWM updaters), moving invariant calculations (like constants multiplied by loop variables that don't depend on the iterator) outside the `for` loop saves redundant CPU cycles (multiplications).
**Action:** Always identify variables and mathematical operations inside `for` or `while` loops that do not change during iterations. Hoist them to a temporary variable outside the loop.

## 2026-09-09 - Safely Hoisting Loop Invariants with Conditional Execution
**Learning:** When hoisting a loop invariant that determines which path an entire loop should take (e.g., stopping all thrusters vs. stepping all thrusters), branching the loop outside based on the condition prevents redundant branching at each iteration.
**Action:** If a high-frequency loop's execution path strictly depends on a state that does not change during the loop, hoist the condition out and duplicate the loop structure for each path to eliminate branching overhead.
