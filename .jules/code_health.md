## 2026-09-09 - Code Health: Refactoring Functions with Excessive Parameters
**Learning:** Functions with excessive parameters (e.g., 8+ individual float arguments for an IMU read) are brittle, hard to read, and difficult to extend.
**Action:** Refactor them by encapsulating related parameters into a single struct (e.g., `imu_data_t`). Pass the struct by value or pointer. Always verify all call sites, including tests and driver implementations, are updated.
