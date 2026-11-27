---
name: x19-knowledge-capture
description: >-
  Distill session learnings, hardware findings, and dead ends into docs/lessons.md,
  promote repeatable mistakes into tools/check_invariants.py, and prune stale memory.
  Use at the end of complex tasks or when running memory maintenance.
---

# X19 Knowledge Capture & Memory Gardening

## 1. Where New Knowledge Goes

1. **Repeatable mistake that a script can catch** -> Add a deterministic rule to `tools/check_invariants.py` (highest leverage).
2. **Hardware quirk, silicon errata, or debugging dead end** -> Record via:
   ```bash
   python rov.py memory add --tag <TAG> --symptom "..." --cause "..." --fix "..."
   ```
3. **Multi-step workflow we will repeat** -> Update or create a skill in `.agents/skills/<name>/SKILL.md`.
4. **Universal, non-negotiable invariant** -> Add a 1-line directive to `GEMINI.md` (only if under the 150-line budget).

## 2. Periodic Consolidation & Pruning

- Inspect memory health: `python rov.py memory status`
- Review recent session traces: `python rov.py memory distill --limit 10 --teamwork`
- Detect stale paths or superseded entries: `python rov.py memory prune`
- Keep `docs/lessons.md` under 250 lines by merging duplicates and marking superseded rows.
