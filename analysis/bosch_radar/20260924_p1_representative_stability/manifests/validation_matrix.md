# Validation matrix

| Gate | Measurement | Required for production candidate |
|---|---|---|
| Group partition | scan-by-scan raw member partition | mismatch 0 |
| PID ownership | raw→PID and PID→members | mismatch 0 |
| Stale representative | chosen raw belongs to current members | 0 |
| Route269 | full segment exact grouping/PID/member parity | regression 0 |
| Route2bc S21 | exact 11-scan control window and full segment | new rep change/regression 0 |
| Route280 S15 | PID1000004 first publication | delay 0 ms |
| Known P0 | target PID publication scan count | duration increase 0 |
| Downstream | lead gain/loss, ID/swap, aLead, planner-input | no critical regression |
| Prefix | 25/50/75/event/100 truncated replay | exact PASS |
| State | bounded state lifecycle | bounded peak |
| CPU | rotated full-path policy order, per-scan distribution | bounded; A1M `NOT MEASURED` |
| Real car | on-road validation | `NOT PERFORMED` in this study |

Gate 순서는 identity equivalence와 stale safety가 대표 churn 감소보다 우선한다. 한 gate라도 실패하면 production patch를 만들지 않는다.

## Actual result

| Gate | Result |
|---|---|
| Corpus | PASS: 34 routes, 847 segments, 504,673 completed scans |
| Candidate E legacy reproduction | PASS: 50,553 representative changes, 80,770 legacy publication-changed scans |
| Group partition | PASS for E/E1/E2/E3/E4/E5: mismatch 0 |
| PID ownership / PID→members | FAIL for every candidate: 19,513 / 4,368 / 14,620 / 15,374 / 3,113 / 114,096 scans |
| Stale representative | PASS: 0 for every candidate |
| Route269 | E1-E4 exact; E 1 PID/member mismatch; E5 316 PID/member mismatches |
| Route2bc S20→S21 warm control | E-E4 exact on 11 scans; E5 11 PID/member mismatches |
| Route280 S15 cut-in | PASS mechanically: 0 ms delay for every candidate |
| Known P0 publication duration | PASS mechanically: positive regression 0 for every candidate |
| Prefix | PASS: 68/68 E1-E4 comparisons |
| Synthetic | PASS: 13 cases × 7 policies, 231 rows, stale 0 |
| Production radar regression | PASS: 241 passed, 6 deselected, 3 environment/config warnings |
| Native MPC/acados requested acceleration | NOT RUN |
| A1M CPU | NOT MEASURED |
| Real car | NOT PERFORMED |

Overall: `REPRESENTATIVE CANDIDATES NO-GO`.
