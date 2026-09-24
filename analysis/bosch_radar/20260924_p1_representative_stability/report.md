# MRRevo14F Representative Stability / Downstream Impact

## Summary

**CONFIRMED FACT:** current source와 34-route corpus에서 58,505 representative changes를 재현했다. 이 중 26,574건은 physical PID와 member set이 완전히 같은 `R1_STABLE_GROUP_REP_CHANGE`이고, 32,456건은 old representative가 여전히 current member인 optional handoff였다. Candidate E의 legacy 수치도 `58,505 → 50,553`, publication-changed `80,770`으로 정확히 재현했다.

**새로운 핵심 결과:** 기존 E 보고의 “grouping/PID unchanged”는 group partition 및 PID birth count 수준이었다. current-source exact raw→PID와 PID→members 비교에서는 E가 19,513 scan을 바꿨다. Representative는 publication만이 아니라 ambiguous PID assignment score의 `+3` input이므로, shared selector 변경이 PID ownership으로 되먹임된다. E1–E5 모두 full corpus exact PID/member parity를 실패했다.

**Final Decision: `REPRESENTATIVE CANDIDATES NO-GO`.** Production code와 tests는 수정하지 않았다. Offline replay는 runtime/vehicle safety proof가 아니며 real-car 적용은 `NOT PERFORMED`다.

## Repository State

- Root: `C:\CarrotRadarResearch\openpilot-p1-pid-continuity`
- Branch: `heatagain/bosch-p1-pid-continuity`
- Baseline HEAD: `609bffa58d6cb1004f64c245521013848698cdcf`
- Primary: `C:\CarrotRadarResearch\openpilot`, `b936b4a31a5bdfe4e1e51d1e54815aac58276258`, clean
- Primary/P1 provider LF-normalized SHA-256: both `eec1c34448534f11cd8cd479a7ceb8d59be83421224130d7bb74f9943ab67f75`
- Artifact: `analysis/bosch_radar/20260924_p1_representative_stability`

Primary checkout은 수정하지 않았다. Git 명령은 serial로 실행했고 reset/clean/stash/force-push를 사용하지 않았다.

## Scope Boundary vs Claude Raw-Hop Study

이번 연구는 grouping이 끝난 뒤의 `physical PID + current member set + representative candidates`에서 시작했다. Moving raw hop 217건, PID carry 214/217, raw assignment candidates/loser, one-to-one solver, same-slot bonus, raw identity transfer, track stealing, actor A/B attribution, 새 moving GT, Claude artifact는 읽거나 변경하거나 tuning input으로 사용하지 않았다. Raw association/actor identity dependency는 unresolved이며 이번 decision을 production으로 승격시키지 않는다.

## Current Representative Algorithm

Existing PID는 prior anchor를 `d + v*dt`와 yaw로 project한 뒤 current observed members만 평가한다. Lexicographic key는 (1) `|d-px| + 0.5|y-py| + 0.5|v-prior.v|`, (2) prior representative 선호, (3) vision support, (4) OEM slot, (5) older age, (6) smaller raw ID다. 2–6은 scalar cost가 정확히 같은 경우만 tie-break다. New PID는 current member dRel median, support, age, raw ID 순이다.

Stale/prior coordinate는 후보가 아니다. 다만 selected representative는 다음 projection뿐 아니라 ambiguous PID assignment score에도 들어가므로 selector 변경을 publication-only로 간주할 수 없다. 세부 코드는 `manifests/representative_algorithm.md`에 기록했다.

## Corpus

현재 source로 34 routes, 847 segments, 504,673 completed scans를 replay했다. Baseline과 E/E1–E5는 같은 input과 policy-rotated order를 사용했다. Scratch cache와 raw log는 commit 대상에서 제외했다.

## Representative Event Taxonomy

| Category | Count |
|---|---:|
| R1 stable PID/member, rep only | 26,574 |
| R2 old rep disappeared | 23,356 |
| R3 member-add coincident | 0 |
| R4 split | 2,746 |
| R5 merge | 5,756 |
| R6 publication reentry | 0 |
| R7 reset/boundary | 73 |

Old representative disappearance로 반드시 바뀐 event는 26,049건이고, old representative가 계속 valid member였던 optional change는 32,456건이다.

## Stable-Group Representative Changes

R1 26,574건은 old/new 모두 같은 current member set에 실제 존재한다. Event table에는 두 candidate의 current d/y/v, age, camera/OEM flags, scalar score, dwell, publication/lead role을 보존했다. `aRel`은 provider가 NaN으로 publication하므로 `N/A_NAN`이며 decoded acceleration인 것처럼 만들지 않았다.

## Ping-Pong Analysis

전체 A→B→A는 24,544 patterns이며 window별 count는 ≤0.2 s 5,579, ≤0.5 s 17,281, ≤1.0 s 20,934, ≤2.0 s 23,310이다. A→B→A→B는 11,427, A→B→C→A는 253이다. Member set까지 동일한 A→B→A는 ≤0.2/0.5/1.0/2.0 s에서 2,090 / 6,188 / 6,892 / 7,157이다.

## Dwell-Time Distribution

전체 representative tenure는 n=597,915, p50=0.199 s, p90=2.600 s, p95=3.999 s, p99=11.300 s, max=61.791 s다. Stable-membership tenure는 n=723,896, p50=0.199 s, p90=2.000 s, p95=3.300 s, p99=8.700 s, max=61.791 s다. 전체 1/2/3-scan tenure는 163,757 / 129,498 / 41,466이고 stable-membership에서는 195,845 / 159,184 / 60,608이다.

## Selection Cost / Near-Tie Analysis

R1 best-vs-old scalar score delta는 n=26,574, p50=0.863, p75=1.617, p90=2.000, p95=2.234, p99=2.682, max=3.469이다. `<=0.25` near-tie는 5,337건이다. Threshold 0.25는 이 분포와 prior E definition을 보존한 corpus-derived cap이며 future duration/GT를 사용하지 않았다.

## Coordinate Jump Distribution

| Population | Field | p50 | p95 | p99 | max |
|---|---|---:|---:|---:|---:|
| All 58,505 | dRel m | 1.75 | 4.50 | 5.75 | 9.25 |
| All | yRel m | 0.156 | 1.094 | 1.656 | 5.625 |
| All | vRel m/s | 0.00 | 0.50 | 1.00 | 2.75 |
| R1 26,574 | dRel m | 2.00 | 3.75 | 4.50 | 6.75 |
| R1 | yRel m | 0.125 | 0.844 | 1.281 | 4.469 |
| R1 | vRel m/s | 0.00 | 0.50 | 0.75 | 1.75 |

`ΔaRel`은 `N/A_NAN`이다.

## Publication Impact

58,505 rep changes 중 58,505건이 publication-visible event에 대응했다: same-coordinate 33, coordinate change 58,463, gap 2, new public point 7. Alias change는 1이다. 이는 group/PID 유지가 곧 published coordinate 유지라는 뜻이 아님을 확인한다.

## Candidate-E Reproduction

E는 prior representative가 current candidate이고 scalar cost가 best+0.25 이내이면 retained한다. Legacy 결과 `50,553 rep changes`, `80,770 publication-changed scans`를 정확히 재현했다. 현재 final `RadarData` point comparator는 80,769 scans로 one-scan metric-definition difference가 있다. Directly suppressed baseline transitions는 9,661, 새 alternative transition까지 고려한 net reduction은 7,952, suppressed ping-pong returns는 5,631이다.

그러나 E는 group partition mismatch 0/stale 0인 동시에 exact PID/member mismatch 19,513, publication set change 17,454, alias change 64,838, `aLead` point change 54,755를 만들었다. 따라서 과거 `RESEARCH-READY ONLY`보다 엄격한 현 gate에서는 NO-GO다.

## Why E Changes 80,770 Publication Scans

한 번의 retained representative decision은 다음 scan에서 baseline과 다른 anchor/projected continuity 및 PID ownership choice로 이어져 divergence가 여러 scan 지속된다. Suppressed-event별 publication divergence는 scans p50/p90/p95/p99/max = 4 / 196 / 347 / 545 / 590, duration p50/p90/p95/p99/max = 0.299 / 19.500 / 34.590 / 54.330 / 58.881 s다. 따라서 7,952 net event reduction과 80,770 changed scans는 1:1이 아니다.

## liveTracks → radarState Path

`BoschPhysicalObject → alias → PID-owned BoschLeadAccelerationEstimator → publication filter → bosch_fill_point → RadarData.points/liveTracks → DPathRadarController → radarState.leadOne/leadTwo → LongitudinalPlanner` 경로를 current code로 확인했다. `bosch_fill_point`는 chosen surface의 dRel/yRel/vRel과 PID-owned aLead를 싣고 `aRel/yvRel/jLead`는 NaN으로 둔다.

## Lead Selection Impact

Baseline representative-change timestamp의 same-scan temporal exposure 58,505건 중 `NO_DOWNSTREAM_EFFECT=58,337`, `LEAD_COORDINATE_ONLY=12`, `LEAD_ID_CHANGED=0`, `LEAD_GAIN=10`, `LEAD_LOSS=20`, `LEAD1↔LEAD2_CHANGE=0`이다. 총 radarState-affected events는 168이다. 이는 counterfactual suppression 효과가 아니라 baseline temporal coincidence이므로 `LOG-BASED INFERENCE`다.

## aLead Impact

Baseline temporal `A_LEAD_CHANGED=126`이다. Candidate E counterfactual은 point-level aLead 54,755 scans, selected-lead aLead 731 scans을 바꿨다. E1/E2/E3/E4/E5 selected-lead aLead change는 0 / 256 / 256 / 26 / 15,839 scans이다. Provider estimator는 physical PID 소유이고 fresh completed scan만 갱신하지만 publication surface vRel 차이가 speed history를 바꿀 수 있다. 실차 감속 귀속은 `NOT VERIFIED`다.

## Planner Input Impact

Baseline temporal planner-input affected events는 168이다. Candidate E/E1/E2/E3/E4/E5의 offline DPath planner-input changed scans는 5,094 / 569 / 3,852 / 3,857 / 746 / 34,205다. Native MPC/acados requested acceleration은 Windows에서 `NOT RUN`; requested accel 및 vehicle actuation claim은 하지 않는다.

## Large-Jump Forensics

R1 p99 union(`d>=4.5` 또는 `y>=1.28125` 또는 `v>=0.75`)은 1,077건이고 그중 score delta≤0.25는 257건이다. 이 subset에서 camera/OEM positive-support flag change는 각각 0건이었다. 모든 row는 정의상 old/new가 current members다. Top 200 trace는 `traces/large_jump/stable_r1_p99_tail.csv`에 좌표, support, score, age를 보존했다. Actor/surface truth는 판정하지 않았으므로 “harmful” 확정은 `NOT VERIFIED`다.

## Route269 Control

Group partition regression은 모든 candidate 0으로 split/rejoin mechanics를 바꾸지 않았다. Exact PID/member mismatch는 E 1, E1–E4 0, E5 316 scans이다. E1–E4 publication은 9/38/39/7 scans 바뀌었지만 radarState change 0이었다. Route269은 target이 아닌 control이며 E와 E5는 strict exact control도 실패한다.

## Route2bc Stable Truck Control

Trusted synthetic-ID namespace와 같은 S20→S21 warm replay에서 11/11 scans가 raw822/PID1000845/member822/representative822로 재현됐다. E–E4는 group/PID/member/target regression 0; E5는 PID/member/target regression 11/11이다. S3부터 시작하면 같은 geometry라도 synthetic counters가 달라지므로 exact identifier control에는 사용하지 않았다.

## Route280 Cut-in Control

PID1000004 first publication delay는 E/E1/E2/E3/E4/E5 모두 0.0 ms이고 target coordinate changed scan도 0이다. 이는 mechanical offline control PASS이며 actor safety proof는 아니다.

## Candidate E1

Ping-pong return-leg only: rep 56,137(net -2,368), ping-pong returns -2,794, publication 16,562, radarState/planner 569, lead gain/loss 0/0, stale 0. 그러나 PID/member mismatch 4,368으로 NO-GO.

## Candidate E2

Two-scan stable-membership confirmation: rep 52,021(net -6,484), ping-pong returns -5,074, publication 65,437, radarState/planner 3,852, lead gain/loss 2/2, stale 0. PID/member mismatch 14,620으로 NO-GO.

## Candidate E3

Support-change-aware retention: rep 51,780(net -6,725), ping-pong returns -5,115, publication 66,676, radarState/planner 3,857, lead gain/loss 2/2, stale 0. PID/member mismatch 15,374으로 NO-GO.

## Candidate E4

Jump-aware weak-advantage retention: rep 57,587(net -918), ping-pong returns -624, publication 17,456, radarState/planner 746, lead gain/loss 0/0, stale 0. Publication은 legacy E보다 63,314 scans 적지만 PID/member mismatch 3,113으로 NO-GO.

## Candidate E5

Medoid comparison: rep 40,129(net -18,376), publication 269,591, radarState/planner 34,205, lead gain/loss 75/77, PID/member mismatch 114,096, Route2bc regression 11/11. Broad NO-GO이며 production 후보가 아니다.

## Corpus Comparison

| Candidate | Rep changes | Net reduction | Ping-pong suppressed | Publication scans | PID/member mismatch | Lead gain/loss | Result |
|---|---:|---:|---:|---:|---:|---:|---|
| E | 50,553 | 7,952 | 5,631 | 80,769 | 19,513 | 2/2 | NO-GO |
| E1 | 56,137 | 2,368 | 2,794 | 16,562 | 4,368 | 0/0 | NO-GO |
| E2 | 52,021 | 6,484 | 5,074 | 65,437 | 14,620 | 2/2 | NO-GO |
| E3 | 51,780 | 6,725 | 5,115 | 66,676 | 15,374 | 2/2 | NO-GO |
| E4 | 57,587 | 918 | 624 | 17,456 | 3,113 | 0/0 | NO-GO |
| E5 | 40,129 | 18,376 | 16,378 | 269,591 | 114,096 | 75/77 | NO-GO |

Accepted best candidate는 `NONE`. Mechanically narrowest trade-off는 E1/E4지만 identity gate 실패 때문에 “best”로 승격하지 않았다. Known P0 target publication-duration positive regression은 모든 candidate 0이다.

## Prefix Invariance

E1–E4를 Route269 및 세 forensic keys의 25/50/75/event/100% cutoff에서 검증했다. 68/68 exact comparisons PASS, future data dependency 0이다.

## Synthetic / Unit Tests

Deterministic 13 cases(A–M) × 7 policies = 231 rows를 실제 selector로 실행했고 stale representative 0이다. Production candidate가 없으므로 `test_radar.py`를 변경하지 않았다. Existing production radar regression은 shim environment에서 241 passed, 6 deselected, 3 config warnings다.

## CPU / State

Timing unit은 x86 Windows의 `provider.update()` call이며 completed scans와 분리했다(5,046,336 calls / 504,673 scans). Baseline mean/p95/p99/max는 152.56 / 935.0 / 1,850.4 / 84,532.1 µs, state peak 13이다. E1은 196.69 / 1,166.5 / 2,384.1 / 301,530.2 µs, state peak 14; E2는 198.02 / 1,174.4 / 2,395.0 / 261,654.1 µs, state peak 14다. Stateless E/E3/E4/E5 policy state peak는 0이다. Timing은 shared host noise를 포함하며 production performance proof가 아니다. **A1M CPU = NOT MEASURED.**

## Production Decision

모든 candidate가 full-corpus PID/member exact equivalence를 실패했다. E/E2/E3에는 lead gain/loss도 있고 E5 blast radius는 더 크다. Identity gate가 첫 gate이므로 prefix, stale, Route280, P0, CPU 결과가 좋아도 production patch를 정당화하지 못한다. Production code changed: no. Tests changed: no.

## Git / Commit / Push

P1 전용 branch에 report/scripts/small tables/manifests/traces만 stage한다. `scratch/`, raw logs, video, cache, `__pycache__`는 `.gitignore`로 제외한다. Commit은 `research: analyze Bosch representative stability`와 `Docs-Not-Needed: read-only Bosch research artifacts; no user-visible behavior change`로 생성하고 `origin/heatagain/bosch-p1-pid-continuity`만 non-force push한다. Final/remote SHA는 commit 이후 최종 응답에서 검증한다.

## Final Decision

**`REPRESENTATIVE CANDIDATES NO-GO`**

Representative churn mechanism, publication persistence, downstream exposure는 확인됐다. 하지만 current selector는 PID assignment와 결합되어 있어 “representative-only” intervention이 아니었다. Actor/raw-association truth 없이 더 좁은 threshold를 튜닝하거나 downstream을 patch하는 것은 범위 밖이며 안전 근거가 아니다.

## Next Minimal Work

다음 최소 작업은 production selector 변경이 아니라, PID ownership 결정이 끝난 뒤 publication surface만 선택하는 truly post-assignment analysis prototype을 별도 정의하는 것이다. 그 prototype도 same current members만 사용하고 exact raw→PID/PID→members parity 0 mismatch를 먼저 증명해야 한다. 이후에만 same-drive actor evidence와 native planner replay를 추가한다. Real-car applied: **NOT PERFORMED**.
