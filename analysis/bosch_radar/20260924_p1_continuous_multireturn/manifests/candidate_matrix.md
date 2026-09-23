# Candidate Matrix

All candidates are analysis-only modules compiled from the exact current production source. None changes the worktree
production file.

| Candidate | Bounded causal rule | Route269 (prevent/delay/unchanged) | Corpus grouping-changed scans | Persistent new merges | Decision |
|---|---|---:|---:|---:|---|
| A `Mature-pair hysteresis` | prior mature edge, small excess, last strict within 300 ms | 3 / 8 / 0 | 18,577 | 1,212 | NO-GO |
| B `N-scan confirmation` | small excess fractures on second consecutive scan; large excess immediate | 2 / 9 / 0 | 14,913 | 633 | NO-GO |
| C `Quantization-aware` | prior edge only, at most one decoded LSB (`0.25 m`, `0.03125 m`), 160 ms | 1 / 8 / 2 | 11,400 | 436 | NO-GO |
| D `Stable core` | prior 3+ member group; exactly one marginal failed edge; all others strict | 0 / 0 / 11 | 3,881 | 356 | NO-GO |
| E `Representative hold` | grouping/PID unchanged; retain present prior rep within continuity-cost 0.25 | 0 / 0 / 11 | 0 | 0 | RESEARCH-READY ONLY |

A-D suppress 10,050 / 10,108 / 8,047 / 1,415 baseline fractures, but also create 21,430 / 17,121 / 12,800 /
4,518 new merge objects corpus-wide. Frozen simultaneous controls alone cannot establish moving actor safety, so these
semantic grouping changes are rejected for production.

E reduces representative changes from 58,505 to 50,553 (7,952 fewer) without changing grouping or PID births, but it
changes publication coordinates on 80,770 scans. That effect needs actor/downstream validation before any production
claim. It is not a fix for Route269 group fracture.

For A-D, peak candidate state was 86 dictionary entries (`fail_counts + last_strict_ns`), bounded by live/recent pair
keys. E adds no state.
