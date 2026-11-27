---
name: x19-cubemx-and-layers
description: >-
  Safely regenerate STM32CubeMX peripheral code, preserve the app_main() trampoline,
  and maintain X19-Embedded one-way firmware layer boundaries. Use when touching .ioc
  files, nodes/*/Core/Src/main.c, boards/*/platform/, or resolving layer violations.
---

# CubeMX Regeneration & Layer Boundaries

## 1. One-Way Layer Contract

`app (nodes/*/Core/Src/app.c, power_sequence.c)` -> `services` -> `drivers` -> `shared`

- **Platform layer** (`boards/*/platform/`, `bsp.c`, `can_f4.c`, CubeMX board headers): may include `shared` and vendor HAL (`stm32*xx_hal.h`), NEVER `drivers`, `services`, or `app`.
- **Non-platform code**: NEVER call `HAL_*`, `__HAL_*`, `LL_*`, `HAL_Delay()`, or `malloc`/`free`.

## 2. Safe CubeMX Regeneration

1. Run `python rov.py generate <target>` (or `all`) to invoke STM32CubeMX CLI and scaffold board configs.
2. Verify `nodes/*/Core/Src/main.c` still calls `app_main();` strictly inside `/* USER CODE BEGIN 2 */` ... `/* USER CODE END 2 */`.
3. Keep all application logic in `Core/Src/app.c`, never in `main.c`.

## 3. Verification

Run after any edit to `.c`, `.h`, or `.ioc` files:
```bash
python rov.py lint --format-check
```
