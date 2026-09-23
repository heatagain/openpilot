# Validation Matrix

| Validation | Result | Evidence boundary |
|---|---|---|
| current-HEAD source replay | PASS | 847 segments, 504,673 scans, 34 routes |
| Route269 raw436/raw482 exact event replay | PASS | 11 fracture / 11 rejoin, no raw482 dropout during its 255-scan life |
| Route2bc S21 stable control | PASS | 11-scan window: raw822, PID1000845, member822, representative822 unchanged |
| synthetic A-M | PASS | 13 deterministic cases × 6 variants, 774 rows |
| lifecycle reset/gap | PASS | timestamp regression refused; >0.3 s gap clears candidate pair state; new manager has empty state |
| frozen SAME controls | 41/41 evaluable, 0 additional-fragment regressions | prior frozen labels only |
| simultaneous DIFFERENT controls | 37/37 evaluable, 0 false merges | not a moving-sequential substitute |
| prefix invariance | PASS 25/25 | A-E at 25%, 50%, 75%, Route269 event cutoff, 100% |
| production behavior mutation | NONE | source SHA256 unchanged |
| actor-level safety | NOT VERIFIED | moving actor evidence intentionally not used |
| A1M CPU | NOT MEASURED | x86 wall-clock microbenchmark only |
