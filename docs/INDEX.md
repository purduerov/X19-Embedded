# X19-Embedded Documentation Index

Use this index to locate authoritative documentation on demand instead of loading full docs into context.

## Hardware & Silicon Ground Truth

- [board_findings.md](board_findings.md) — **Source of truth** for STM32C542CCT6 schematic audit (supersedes `pinout_mappings.md` on STM32C542 pins; explains missing `PC0-PC12` and PCB stub status).
- [target-integration-blockers.md](target-integration-blockers.md) — Current target-only gaps (missing Node 1/3 CubeMX startup files, weak CAN fallback, sensor integration status).
- [dshot_hardware_architecture.md](dshot_hardware_architecture.md) — DShot thruster timer/DMA hardware architecture.
- [pinout_mappings.md](pinout_mappings.md) — Legacy STM32G431CB pinout table (check `board_findings.md` before assuming STM32C542 pin assignments).

## Simulation & Developer Workflows

- [sil_simulation_guide.md](sil_simulation_guide.md) — Host SIL simulation architecture, Python dashboard/stimulus, and Streamlit version contract (`streamlit>=1.40,<2`).
- [vscode-embedded-workflow.md](vscode-embedded-workflow.md) — VS Code tasks, launch configs, and STM32 debugging setup.

## Design Specs & Plans

- `docs/superpowers/specs/` — Architecture & design specifications (SIL dashboard, board-agnostic `.ioc` architecture, injected I2C bus drivers, Node 1 rail telemetry).
- `docs/superpowers/plans/` — Step-by-step implementation plans corresponding to the specs above.

## Institutional Memory

- [lessons.md](lessons.md) — Structured Episodic & Semantic log of hardware errata, build traps, dead ends, and verified fixes (`python rov.py memory search <query>`).
- [REPO_MAP.md](REPO_MAP.md) — Auto-generated compact index of public header types and function signatures (`python rov.py map`).
- [harness_evals.md](harness_evals.md) — 5-case regression rubric for validating harness, skill, and hook changes.
