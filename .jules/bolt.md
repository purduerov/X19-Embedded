## 2026-03-31 - Table-Driven CRC16 CCITT Computation
**Learning:** Bit-by-bit CRC calculation loops in Python introduce significant interpreter overhead per bit shift/XOR operation. Using a 256-entry precomputed lookup table replaces 8 loop iterations per byte with a single indexing operation.
**Action:** Use precomputed 256-entry lookup tables for CRC computations in Python host/flasher utilities to achieve ~8.7x speedups.
