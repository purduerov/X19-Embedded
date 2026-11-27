---
name: x19-sil-and-i2c-drivers
description: >-
  Implement and test hardware-independent I2C/CAN drivers and Host SIL tests in
  X19-Embedded using the injected rov_i2c_bus_t pattern and SIL bridge stimulus.
  Use when adding or debugging sensor drivers, mock register devices, or SIL tests.
---

# SIL & Injected I2C Driver Pattern

## 1. Injected Bus Architecture

- Drivers (`drivers/src/*.c`) must accept an injected `rov_i2c_bus_t` interface instead of calling ST HAL I2C functions directly.
- Host unit tests (`tests/test_driver_*.c`) drive the real driver code against mock register-level devices in `tests/mocks/`.
- Never add hardcoded "value hooks" inside drivers; convert raw registers to engineering units per datasheet.

## 2. Running Driver & SIL Tests

```bash
python rov.py test --suite driver --compact
python rov.py test --suite safety --compact
python rov.py can sniff --sil localhost:8765
```

## 3. References

- `docs/sil_simulation_guide.md`
- `docs/superpowers/specs/2026-10-15-injected-i2c-bus-driver-architecture.md` (check `docs/INDEX.md` for exact filenames)
