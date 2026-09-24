# MRRevo14F Post-Assignment Publication Surface Study

## Summary

**CONFIRMED FACT — final decision: `PUBLICATION CHURN MOSTLY BENIGN`.** Analysis-only role separation은 구현 가능했고 FIRST GATE는 PASS했다. P0–P6 모두 34 routes / 847 segments / 504,673 scans에서 raw/group/PID/member/tracking representative/qualification/publication set/alias/complete PID-owned aLead mismatch가 0이다. 그러나 어느 후보도 overall dRel/yRel/vRel p99 jump tail을 개선하지 못했다. Ping-pong을 가장 많이 줄인 narrow하지 않은 P4는 dRel/yRel p99를 각각 0.0390 m/0.03125 m 악화시키고 lead ID change 1 scan을 만들었다. 가장 좁은 causal comparison P2도 dRel/yRel p99를 0.0169 m/0.03125 m 악화시켰다.

따라서 publication-only 분리는 identity feedback 제거에는 성공했지만, 별도 selector의 downstream 품질 이득은 확인되지 않았다. Production code는 변경하지 않았고 P1–P6 어느 것도 production candidate로 retain하지 않는다.

## Repository State

- Root: `C:\CarrotRadarResearch\openpilot-p1-pid-continuity`
- Branch: `heatagain/bosch-p1-pid-continuity`
- Baseline HEAD: `c29c4fa8b2d99e15a11879c0f718f73d53b2b4e3`
- Artifact: `analysis/bosch_radar/20260924_p1_post_assignment_publication_surface`
- Primary checkout: read-only; study-start HEAD `b936b4a31a5bdfe4e1e51d1e54815aac58276258`
- Primary/P1 provider LF-normalized SHA-256: `eec1c34448534f11cd8cd479a7ceb8d59be83421224130d7bb74f9943ab67f75`
- Production source changed: no
- Production tests changed: no

## Scope Boundary vs Claude PID-Carry Study

Moving raw hop 217/H2 27, PID carry 25, direct carry, inheritance scoring/veto, stable-member PID anchor, provisional PID, PID confidence, Claude artifact 및 threshold는 읽거나 수정하거나 success metric으로 사용하지 않았다. 이 연구는 baseline raw association/grouping/PID assignment/tracking representative/PID-owned state가 끝난 뒤에만 시작했다.

## Current Pipeline

Current source의 실제 경로는 `raw -> grouping -> physical PID -> tracking representative -> association -> qualification/publication-state update -> alias allocation -> PID-owned aLead update -> final publication_view -> bosch_fill_point -> RadarData/liveTracks -> DPathRadarController/radarState -> planner input`이다.

## Exact Post-Assignment Boundary

Study insertion point는 `radar_interface.py`의 `objects = self.bosch.publication_view(...)` 직후이자 `bosch_append_points(...)` 직전, study-start source 기준 lines 6243–6244 사이다. 이 시점에는 physical PID/member/representative, association, qualification, publication set, alias와 baseline PID-owned aLead가 모두 확정돼 있다. Candidate는 이 immutable 결과를 읽어 cloned point의 `dRel/yRel/vRel`만 바꾼다.

## Tracking / Publication Role Separation

Tracking representative는 baseline 그대로 다음 scan projection과 PID assignment에 사용된다. Publication surface state는 physical PID keyed이며 publication absence, PID death/replacement, member-set change, input gap >=300 ms, timestamp/reset, segment boundary에서 폐기된다. Candidate가 고른 member는 current completed scan에서 관측되고 previous completed scan보다 새 timestamp를 가진 실제 member여야 한다. Stale/coasted/past/synthetic/interpolated/centroid point는 선택하지 않는다.

## Baseline Corpus

- Routes: 34
- Segments: 847
- Completed scans: 504,673
- Prior baseline cache parity: scan 0, qualification 0, source-current publication 0, scan-count 0 mismatch
- Current production OEM-nearer surface vs requested P0 tracking-representative surface: 9,400 point decisions / 9,370 scans different; DPath output 6,462 scans different. 이 값은 candidate 결과와 분리한 sensitivity다.

## Baseline Publication Surface

P0는 요청대로 `publication surface = tracking representative`다. P0 published-surface switch는 58,496, stable-member consecutive switch는 26,570이다. 기존 tracking study의 confirmed facts인 representative changes 58,505, R1 stable rep-only 26,574, optional handoff 32,456, A→B→A 24,544, same-member-set <=0.5 s 6,188도 그대로 보존한다. 새 수치가 9/4/3건 작은 이유는 이번 metric이 baseline publication set 안에서 consecutive published lifecycle만 세기 때문이다.

## Upstream Invariance Harness

Complete live replay 결과는 모든 P0–P6에서 다음 mismatch가 각각 0이다.

| invariant | mismatch |
|---|---:|
| raw IDs/mapping | 0 |
| group partition | 0 |
| physical PID | 0 |
| PID→members | 0 |
| tracking representative | 0 |
| qualification | 0 |
| publication set | 0 |
| alias | 0 |
| internal aLead complete state | 0 |

Internal aLead signature는 counters, reset/expiry/publication lifecycle fields, every-PID timestamp/update-count/aLead/state-age를 포함한다. **FIRST GATE: PASS.**

## Stable Group Events

P0 stable-member published switch는 26,570이다. P2는 25,125(-1,445), P6는 24,908(-1,662)이지만, switch 감소가 jump tail 개선으로 이어지지 않았다. Stable mechanical multi-return controls 500개에서 P2/P6의 baseline-different scan은 각각 11/12이고 upstream regression은 0이다. Prior frozen SAME controls 41개는 matching grouped multi-return episode가 0이므로 surface migration 평가는 `UNEVALUABLE`이다.

## Baseline Ping-Pong

- Published A→B→A: 24,541
- Published A→B→A→B: 11,426
- Strict stable-member A→B→A <=0.5 s: 4,670
- Surface dwell 1/2/3 scans: 165,067 / 130,063 / 41,547; total 336,677

기존 tracking-only A→B→A 24,544와 same-member-set <=0.5 s 6,188은 별도 prior confirmed fact다. 이번 4,670은 publication lifecycle과 세 dwell run 전체의 동일-member-set을 요구하는 더 좁은 denominator다.

## Baseline Jump Distribution

| coordinate | p50 | p95 | p99 | max |
|---|---:|---:|---:|---:|
| dRel m | 1.755833 | 4.488698 | 5.749926 | 9.325016 |
| yRel m | 0.15625 | 1.09375 | 1.65625 | 5.625 |
| vRel m/s | 0 | 0.5 | 1.0 | 2.75 |

Stable-group dRel/yRel/vRel p99는 4.652062 m / 1.28125 m / 0.75 m/s다.

## Candidate P1

Switch 55,701, A→B→A 22,243, baseline-different publication scans 4,772, lead gain/loss 0/0. Ping-pong은 2,298 줄지만 dRel/yRel p99가 5.788956 m/1.6875 m로 악화되고 lead ID change 1, exposed-lead aLead change 1, planner-input change 9가 발생한다. Reject.

## Candidate P2

Switch 57,146, A→B→A 23,200, baseline-different publication scans 2,106, lead gain/loss 0/0. A→B→A는 1,341(5.46%) 줄고 1/2/3-scan dwell은 1,306(0.388%) 줄었다. RadarState/planner input은 같은 lead ID의 coordinate-only 1 scan만 바뀌었다. 그러나 dRel/yRel/vRel p99 reduction은 -0.016878 m / -0.03125 m / 0 m/s, 즉 두 coordinate tail이 악화됐다. **Best nonproduction comparison이지만 retain하지 않는다.**

## Candidate P3

Switch 58,149, A→B→A 24,279, changed scans 815, lead gain/loss 0/0. Large-jump gate임에도 dRel p99는 5.754443 m로 0.004517 m 악화되고 yRel p99도 0.03125 m 악화된다. RadarState/planner coordinate-only 1 scan. Reject.

## Candidate P4

Switch 55,701, A→B→A 22,243, changed scans 4,769, lead gain/loss 0/0. P1보다 3 scan 좁지만 jump-tail 악화가 동일하고 lead ID change 1/exposed aLead change 1/planner-input change 7이 있다. 가장 큰 narrow-family ping-pong 감소만으로 production gate를 통과하지 못한다. Reject.

## Candidate P5

Broad medoid reference는 switch 45,731, A→B→A 14,910으로 줄지만 changed scans 140,009, lead coordinate-only 8,984, lead ID 380, gain 77, loss 71, planner-input 9,487다. dRel/yRel/vRel p99도 6.719940 m/2.0625 m/1.25 m/s로 크게 악화되고 divergence max는 598 scans/59.70 s다. **NO-GO reference.**

## Candidate P6

Switch 56,937, A→B→A 23,078, changed scans 2,662, lead gain/loss/ID change 0/0/0. RadarState/planner coordinate-only 2 scans이다. 그러나 dRel/yRel/vRel p99 reduction은 -0.018635 m / -0.03125 m / 0 m/s다. P2보다 blast radius가 크고 tail이 더 나빠 hybrid 이득이 없다. Reject.

## Publication Divergence

| policy | changed scans | different decisions | episodes | episode scans p50/p95/p99/max | duration p50/p95/p99/max s |
|---|---:|---:|---:|---|---|
| P1 | 4,772 | 5,389 | 5,161 | 1/1/2/4 | 0/0/0.100526/0.300882 |
| P2 | 2,106 | 2,379 | 2,264 | 1/1/2/4 | 0/0/0.100492/0.300511 |
| P3 | 815 | 820 | 778 | 1/1/2/4 | 0/0/0.109929/0.300183 |
| P4 | 4,769 | 5,386 | 5,158 | 1/1/2/4 | 0/0/0.100526/0.300882 |
| P5 | 140,009 | 210,932 | 34,216 | 3/22/43/598 | 0.197/2.100/4.200/59.698 |
| P6 | 2,662 | 2,947 | 2,803 | 1/1/2/4 | 0/0/0.100749/0.300511 |

Narrow candidates는 prior shared Candidate E 80,770 divergent scans보다 훨씬 좁지만, 그 자체가 tail 품질 개선 증거는 아니다.

## Coordinate / aLead Consistency

Internal PID-owned aLead mismatch는 모든 후보에서 0이다. |ΔvRel| >=0.75 m/s인 changed point는 P1/P2/P3/P4/P5/P6에서 134/44/132/134/4,037/133건이다. Exposed selected-lead aLead change는 P1/P4/P5에서 1/1/424 scan이고 P2/P3/P6은 0이다. P2는 changed geometry와 baseline aLead 조합이 lead exposure를 바꾸지는 않았지만, tail 개선도 만들지 않았다.

## radarState Impact

| policy | no downstream effect | radarState changed | coordinate only | lead ID | gain | loss | swap |
|---|---:|---:|---:|---:|---:|---:|---:|
| P1 | 4,763 | 9 | 8 | 1 | 0 | 0 | 0 |
| P2 | 2,105 | 1 | 1 | 0 | 0 | 0 | 0 |
| P3 | 814 | 1 | 1 | 0 | 0 | 0 | 0 |
| P4 | 4,762 | 7 | 6 | 1 | 0 | 0 | 0 |
| P5 | 130,497 | 9,512 | 8,984 | 380 | 77 | 71 | 0 |
| P6 | 2,660 | 2 | 2 | 0 | 0 | 0 | 0 |

## Lead Selection Impact

P2의 유일한 effect는 route `00000252--d191bee66c--0`, scan 359에서 lead ID 90을 유지한 coordinate-only 변화다. P6은 같은 scan과 route `0000025d--70dff69b9d--23`, scan 281에서 lead ID를 유지한 coordinate-only 변화 2건이다. Narrow candidates P2/P3/P6은 lead gain/loss/ID/swap을 만들지 않았다. P1/P4는 ID change 1건 때문에 더 넓은 정책을 retain하지 않는다.

## Planner Input Impact

Planner가 읽는 lead status/track/dRel/vRel/aLeadK/radar tuple changed scans는 P1/P2/P3/P4/P5/P6에서 9/1/1/7/9,487/2다. Current `DPathRadarController`는 exact code로 replay했다. Native longitudinal MPC/acados requested acceleration replay는 Windows dependency 한계로 `NOT RUN`이다.

## Route269 Control

지정 segment `00000269--4014745f93--7` 600 scans에서 P0–P6의 raw/group/PID/member/tracking representative/qualification/publication set/alias/internal aLead regression은 모두 0이다. P6 publication coordinate만 7 scan 달랐다. Split/rejoin/PID birth/tracking representative를 “fix”하지 않았다.

## Route2bc Control

S20→S21 warm control 11 scans, raw822/PID1000845/member822/rep822에서 P0–P6 publication/tracking/PID/member regression은 모두 0이다.

## Route280 Cut-in

S15 PID1000004 first-publication delay는 P0–P6 모두 0.0 ms이고 target-coordinate changed scans도 모두 0이다. Publication set/PID/tracking representative/internal aLead mismatch는 0이다.

## Multi-Return Controls

Stable mechanical controls 500개에서 P2/P6 baseline-different scans는 11/12다. 모든 upstream identity invariant는 0이다. P5는 15,705 scan이 달라 broad medoid가 정상 reflector progression을 과도하게 재정의함을 보여준다. 새 actor GT는 만들지 않았다.

## P0 Regression

Known P0 controls의 positive publication-duration regression은 P0–P6 모두 0이다. Publication set은 exact baseline이다.

## Prefix Invariance

Route269 25/50/75/event/100%, Route280 및 corpus 양끝 controls의 P1–P6 comparison 102/102 PASS, failure 0이다.

## CPU / State

Windows x86에서 alternating policy order로 selector+clone elapsed를 측정했다. 이는 A1M CPU가 아니라 짧은 호출의 elapsed proxy이며 max는 host scheduling outlier를 포함한다.

- P0 mean/p50/p95/p99/max: 272.91/220.1/555.5/895.8/404,680.8 us; summed wall 137.73 s
- P2 mean/p50/p95/p99/max: 381.89/306.4/782.1/1,244.7/288,062.5 us; summed wall 192.73 s
- P2 publication-only state peak: 32 physical PIDs
- Complexity: current published members only, O(group members); global search 없음
- A1M CPU: **NOT MEASURED**

## Production Decision

| gate | result |
|---|---|
| upstream exact invariance | PASS |
| tracking representative exact | PASS |
| internal aLead exact | PASS |
| publication set exact | PASS |
| stale surface 0 | PASS |
| Route2bc regression 0 | PASS |
| Route280 delay 0 | PASS |
| known P0 duration regression 0 | PASS |
| published ping-pong reduction | PASS for P1–P6 |
| large-jump tail reduction | **FAIL for every candidate** |
| new lead gain/loss | PASS for P1–P4/P6; FAIL P5 |
| planner blast acceptable | narrow P2/P3/P6 small, P5 FAIL |
| prefix | PASS |
| CPU bounded | PASS on x86 elapsed proxy; A1M NOT MEASURED |

No candidate satisfies all gates. Production patch는 만들지 않는다.

## Git / Commit / Push

Report/scripts/manifests/small tables/small traces만 P1 branch에 명시 stage한다. Scratch/raw logs/video/cache/`__pycache__`는 제외한다. Commit/push SHA는 self-referential artifact 밖의 final handoff에 기록한다.

## Final Decision

## PUBLICATION CHURN MOSTLY BENIGN

**CONFIRMED FACT:** publication-only role separation은 upstream identity feedback 0으로 구현됐다. **LOG-BASED OFFLINE RESULT:** P2는 2,106 changed scans로 A→B→A를 1,341건 줄이고 downstream coordinate-only 1 scan만 바꾸지만 dRel/yRel p99 tail을 개선하지 못했다. P1/P4의 더 큰 ping-pong 감소는 tail 악화와 lead ID change를 동반한다. P5 medoid는 대규모 lead/planner regression을 만든다. 따라서 observed representative surface churn의 대부분은 별도 sticky selector로 안전하게 제거할 수 있는 downstream discontinuity가 아니었다.

이 결론은 offline replay/controls에 한정된다. Real-car safety/runtime/vehicle/NAS proof가 아니다.

## Next Minimal Work

이 family를 threshold tuning으로 계속 확장하지 않는다. 다시 열 최소 조건은 새로운 independent evidence가 overall 및 stable-group dRel/yRel/vRel tail을 실제로 줄이면서 lead/planner regression 0을 보이는 경우다. 그 전에는 P0/current production을 유지한다. Native MPC/acados, A1M CPU, real-car 적용은 각각 `NOT RUN`, `NOT MEASURED`, **`NOT PERFORMED`**다.
