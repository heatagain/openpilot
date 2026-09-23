# Bosch MRRevo14F P1 physical identity continuity 연구

## 최종 판정

**INCONCLUSIVE; production physical-PID 변경은 NO-GO.**

이번 연구에서 P1의 재현 가능한 원인은 찾았지만, 안전하게 고칠 수 있는 production candidate는 확보하지 못했다. Route269 S7의 확인된 box-truck 사례는 short dropout/reacquisition 실패가 아니라, 동일 raw482가 계속 관측되는 동안 complete-link pair가 경계에서 실패하고 parent PID가 representative raw436 child에 귀속되면서 raw482 singleton에 새 PID가 반복 생성되는 **group ownership oscillation**이다. 반면 요청의 Route2bc S21 화물차 사례는 현재 연속 replay의 11개 scan에서 raw822/PID1000845/member822/representative822가 모두 고정되어 P1 identity failure가 아니었다.

moving-to-moving sequential DIFFERENT GT가 **0**이므로, 새로운 stitching/lineage rule의 가장 중요한 safety denominator가 없다. timeout extension, grouping gate 확대, representative hysteresis, bounded ancestry/owner-veto의 네 접근을 검토했지만 production code와 SHADOW는 만들지 않았다. 따라서 PID continuity 개선은 **0**, 신규 false stitch/false merge도 **production 변화가 없으므로 0**이지만, 이것은 candidate 안전성이 증명됐다는 뜻이 아니다.

실차 적용, NAS 배포, 현재 차량의 `liveTracks → radarState → planner/control` 검증은 수행하지 않았다. **REAL-VEHICLE DEPLOYMENT: NOT PERFORMED.**

## 1. Environment

- repository root: `C:\CarrotRadarResearch\openpilot-p1-pid-continuity`
- worktree: 위 경로만 사용; primary checkout은 읽기 이외에 수정하지 않았다.
- branch: `heatagain/bosch-p1-pid-continuity`
- baseline HEAD: `581285fb7dbed511fec4ba9e0edd256f5b5aa96e`
- production candidate commit: **N/A — production candidate 없음**
- provider: `opendbc_repo/opendbc/car/hyundai/radar_interface.py`
- provider LF-normalized SHA-256: `eec1c34448534f11cd8cd479a7ceb8d59be83421224130d7bb74f9943ab67f75`
- worktree raw-byte SHA는 checkout EOL 변환 때문에 prior validated raw-byte SHA와 다르지만 LF-normalized source는 동일하다. [provenance manifest](manifests/evidence_provenance.json)가 두 hash와 모든 입력 hash를 보존한다.
- Bosch RadarTracks의 기존 production mode는 그대로다. P91, B/CAND_C, B5, burst, camera/OEM association, side-pass publication state를 변경하지 않았다.
- trace/source 연구는 offline/read-only이며 future frame으로 과거 결정을 수정하지 않았다.

## 2. 문제 정의와 identity 층

이번 P1 범위는 physical PID continuity, 실제 dropout 후 reacquisition, 대형차 multi-return/group/representative 안정성이다. P0 phantom/clone/false-surface 정책은 수정하지 않는다.

현재 코드 경로는 `0x602–0x611 raw CAN → 32 half-slots → BoschRawTrackManager → BoschObjectGroupManager complete-link → physical PID assignment → representative → camera/OEM association → qualification/publication filters → alias/RadarPoint → liveTracks → radarState`다. 마지막 `liveTracks` 이후는 이번 연구에서 replay하지 않았다.

| 층 | ownership / 수명 | P1 해석 |
|---|---|---|
| CAN slot | scan의 0–31 half-slot | slot 변경은 raw ID/PID 변경과 동의어가 아니다. |
| raw track ID | 운동학 global assignment; 최근 관측 후 0.3 s coast | slot을 바꿔도 유지될 수 있고, hard gate를 넘는 새 actor에는 재사용하지 않는다. |
| member group | `Δd≤3 m`, `Δy≤1.5 m`, `Δv≤1.5 m/s` 등의 mature pair를 모두 만족하는 complete-link group | 한 pair의 경계 실패가 group fracture를 만든다. |
| physical PID | 최근 member ownership overlap, prior representative retention, PID age score로 group에 1:1 할당 | 두 child에 같은 PID를 줄 수 없다. 흡수된 singleton ownership은 즉시 제거된다. |
| representative | prior representative의 predicted position 연속성으로 member 중 선택 | representative 교체는 PID 교체와 다르다. |
| public alias | PID별 32–95 allocator binding, 미게시 grace 최대 0.5 s | alias/publication gap은 physical PID 사망이 아니다. |

## 3. Baseline dataset과 GT 경계

| evidence | 수량 | 해석 |
|---|---:|---|
| current-remap sequential SAME | 4 / 2 routes | 영상 GT; 4건 모두 baseline에서 distinct PID. |
| exact O1 SAME raw482 | 3 / Route269 한 route | 동일 box truck의 same raw group↔singleton positive. |
| sequential DIFFERENT | 4 / 3 routes | static/parked actor 전환; 이 중 strict blind-order 3건. |
| moving-to-moving sequential DIFFERENT | **0** | stitching safety gate의 핵심 빈 분모. |
| moving-to-moving sequential SAME | 1 | positive 보조자료이며 DIFFERENT 분모가 아니다. |
| simultaneous DIFFERENT vehicle pairs | 37 | concurrent false-merge controls; sequential reacquisition GT가 아니다. |
| whole-corpus current-provider replay | 847 segments / 504,673 scans / 34 routes | mechanical prevalence; SAME/DIFFERENT GT로 승격하지 않는다. |

세부 분모는 [gt_denominators.csv](tables/gt_denominators.csv), source hash는 [evidence_provenance.json](manifests/evidence_provenance.json)에 있다. `AMBIGUOUS` endpoint를 SAME으로 올리지 않았다.

## 4. Baseline finding A — physical PID continuity

### Route269 S7 root cause

**CONFIRMED FACT:** raw482는 255개 완료 scan 동안 slot28, raw age 1→255로 연속 관측됐다. 이 구간에 `{436,482}` parent group에서 11번 분리되어 매번 새 singleton PID를 얻었고 11번 parent group에 재편입됐다. raw dropout, raw slot churn, association reset이 직접 원인이 아니다.

**CONFIRMED FACT:** 11개 split의 실패 pair는 모두 `436↔482`다. 분포는 distance diameter 6, lateral growth 4, lateral diameter 1이다. 초과량은 distance 0.25 m, lateral diameter 0.03125 m, lateral growth 0.03125–0.1875 m였다.

**CONFIRMED FACT:** split 때 parent PID score는 representative raw436 child가 `10×overlap + 3×representative retention`으로 raw482 child보다 높다. raw436 child가 parent PID를 유지하고 raw482 singleton은 새 PID를 받는다. 재편입 scan에 old singleton의 raw482 ownership이 제거되므로 다음 split은 다시 새 PID birth다.

**LOG-BASED INFERENCE:** mechanism은 `pair boundary failure → complete-link split → representative child가 parent PID 승계 → singleton new PID → rejoin → singleton owner cleanup → next new PID`다. 물리적 sensor surface가 왜 그 경계를 넘었는지는 모든 11개에 대한 영상 판독이 없으므로 확정하지 않는다.

전수 corpus에는 같은 mechanical split birth가 16,526건, 3초 내 parent return이 5,767건 있었다. 이는 prevalence일 뿐 물리 SAME 분자가 아니다.

## 5. Baseline finding B — Route2bc S21 large truck

요청의 cold identifier는 Track68/PID1000245/raw239였고, current continuous warm replay에서는 Track76/PID1000845/raw822로 대응된다. numeric ID 자체를 서로 동일시하지 않았다.

20.865–21.865 s의 [11-scan trace](traces/route2bc_s21_identity_window.csv):

- raw ID: `822` 단일
- physical PID: `1000845` 단일
- members: `{822}` 단일
- representative: `822` 단일
- representative change: `0`
- member-set change: `0`
- group split/merge: `0`
- public alias는 일부 scan에서 미게시였지만 다시 Track76을 사용했으며 physical identity는 유지됐다.

**CONFIRMED FACT:** 이 window에는 P1 PID churn, group fracture, representative oscillation, nearby-object false stitch가 없다.

**P0 INTERACTION FOUND:** 이 return은 대형차 측면의 실제 scattering point가 lateral로 이동한 장면이며, 현행 side-pass 처리는 publication-only다. raw/member/PID/representative/alias ownership을 바꾸지 않는다. P0 원인과 정책은 이번 branch에서 재정의하거나 수정하지 않았다.

## 6. Reacquisition finding

현재 baseline은 raw/group를 약 0.3 s coast하지만, distinct physical PID를 old PID로 다시 잇는 정책은 없다. 확인된 SAME 4건은 baseline에서 distinct PID이나, 그중 exact O1 3건은 object disappearance가 아니라 두 child가 동시에 존재하는 split이다. 따라서 이를 short-gap reacquisition 성공률로 계산하지 않는다.

- SAME reacquisition latency: **UNEVALUABLE**
- moving DIFFERENT false-stitch denominator: **0 / UNEVALUABLE**
- timeout extension 효과: Route269 root case에는 **0/11** — raw482가 사라지지 않았기 때문이다.
- ambiguity가 있는 새 birth를 old object로 fail-open stitch하는 정책: **구현 안 함**

## 7. Large-vehicle multi-return / representative finding

Route269 영상 SAME 3건은 한 box truck의 동일 raw surface와 group owner oscillation을 확정한다. 그러나 parent와 singleton 두 physical groups가 split scan에 동시에 존재하므로 숫자 PID 하나를 양 child에 줄 수 없다. group radius를 넓히면 이 사건의 11/11 boundary split은 기계적으로 사라질 수 있지만, 근접 실제 차량 false merge safety를 증명할 moving hard negative가 없다.

Route2bc S21의 representative churn은 0/11이며 jump 분포는 event가 없어 **UNEVALUABLE**다. 전 corpus의 대표점 jump p50/p95/p99/max는 이번 compact study에서 새로 측정하지 않았다. 기존 source는 representative-independent physical assignment를 일부 갖고 있지만, Route269의 fracture는 representative 선택 이전 complete-link에서 시작한다.

## 8. Candidate history

전체 표는 [candidate_history.csv](tables/candidate_history.csv).

### Candidate A — timeout/coast extension

- idea: 짧은 gap 동안 old raw/physical state를 더 오래 유지.
- result: Route269 exact root capture **0/11**.
- failure: raw482가 255 scan 연속 관측돼 bridge할 gap이 없다.
- decision: `REJECTED_MECHANISM_MISS`; stale identity/false stitch 위험만 늘릴 수 있어 구현하지 않았다.

### Candidate B — complete-link gate 확대

- idea: distance/lateral diameter 또는 growth gate를 넓힘.
- result: recorded 11 boundary split은 **hypothetically 11/11** 막을 수 있다.
- failure: 전역 grouping semantics를 바꾸며 truck 옆 승용차·근접 병렬 차량 false merge 위험이 있다. actual moving sequential DIFFERENT 분모가 0이다.
- decision: `REJECTED_FALSE_MERGE_RISK`; threshold sweep, code, replay candidate를 만들지 않았다.

### Candidate C — representative hysteresis

- idea: 대표 교체 이득이 작으면 old representative 유지.
- result: Route269 exact root capture **0/11**.
- failure: representative raw436은 이미 안정적이며 complete-link split이 representative 선택보다 먼저 일어난다.
- decision: `REJECTED_MECHANISM_MISS`; Route2bc도 representative churn 0이라 개선할 target이 없다.

### Candidate D — bounded ancestry + owner contradiction veto

- idea: 최근 absorbed-member ancestry를 bounded/causal diagnostic으로 보존하되, active parent와 singleton이 동시에 있으면 physical stitch를 veto.
- result: Route269 11/11을 O1 family로 표시할 수 있지만 이는 **family label only**이며 PID continuity 개선이 아니다.
- failure: 한 raw의 active owner contradiction을 해결하지 못하고 moving DIFFERENT=0이라 SHADOW safety gate도 실패한다.
- decision: `INCONCLUSIVE_NOT_IMPLEMENTED`; SHADOW/production state 모두 추가하지 않았다.

### Final candidate

선택된 production candidate는 없다. baseline provider를 그대로 유지한다. research-only script/report/table만 남긴다.

## 9. Metrics — baseline vs final

| metric | baseline | final | 판정 |
|---|---:|---:|---|
| Route269 exact new PID births | 11 | 11 | 개선 0 |
| confirmed SAME distinct-PID transitions | 4 | 4 | 개선 0 |
| confirmed sequential DIFFERENT stitched as same PID | 0/4 | 0/4 | 변화 없음; static-only 분모 |
| moving sequential DIFFERENT denominator | 0 | 0 | UNEVALUABLE |
| Route2bc representative changes | 0/11 | 0/11 | stable |
| Route2bc member-set changes | 0/11 | 0/11 | stable |
| Route280 S15 tracker→publication | 0.0 ms | 0.0 ms | no candidate delay |
| whole-corpus mechanical split births | 16,526 | 16,526 | unlabeled mechanics; 개선 0 |

`false split`, PID churn/episode, representative jump distribution, reacquisition latency는 episode-level physical GT 분모가 충분하지 않아 임의 비율을 만들지 않았다. 원시 값은 [metrics.csv](tables/metrics.csv).

## 10. Regression / safety

- false stitch: baseline sequential DIFFERENT 4건 모두 old/new PID가 달라 **관측 0/4**. 후보가 없으므로 신규 0이지만 moving actor safety는 UNEVALUABLE.
- false merge: 37 simultaneous DIFFERENT vehicle pair는 baseline control이며 candidate가 없어 신규 merge 0. sequential stitching 안전 증명으로 사용하지 않는다.
- actual cut-in: Route280 S15 current replay에서 PID1000004가 첫 completed tracker scan에 qualified/published, tracker→publication 0.0 ms, actor group split birth 0.
- Route259 S11 / Route264 S22 등 critical corpus는 847-segment replay에 포함됐으나 이번 pass에서 새 영상 actor remap을 하지 않았으므로 사건별 `NOT VERIFIED` 경계를 유지한다.
- P0 false surface 장수화: production state/history 변화가 없어 신규 장수화 경로 없음. candidate B/D hypothetical safety는 검증하지 않았다.
- `radard.py`, planner, MPC, control, UI, generic provider: 수정 없음.

## 11. Prefix invariance / causality

새 candidate가 없으므로 candidate prefix는 N/A다. 현재 provider의 prior current-source validation을 provenance와 함께 재사용했다.

- Route269 trace OFF/ON: 600 scan mismatch 0; prefix 25/50/75/event/100% **5/5 PASS**.
- Route2bc/large-vehicle/cut-in side-pass current-source prefix set: **25/25 PASS**.
- 이번 evidence builder는 offline 결과를 production decision으로 되먹이지 않으며 미래 정보는 GT review에만 사용했다.

이 결과는 offline provider prefix이며 runtime/downstream causality 증명이 아니다.

## 12. Synthetic/unit tests

[requested_test_coverage.csv](tables/requested_test_coverage.csv)에 요청한 Case 1–12를 현재 test 이름에 매핑했다. stable return, slot handoff, member disappearance, one-scan raw recovery, new target hard gate, close vehicles, representative switch, truck+adjacent, cut-in, timestamp gap, provider timeout, segment reset을 포함한다.

- P1 worktree green subset: **241 passed, 6 deselected**.
- unfiltered non-Group3 run: **241 passed, 3 environment failures, 3 deselected**.
- 3 failures는 generated DBC 2개 부재에 따른 `FileNotFoundError`; P1 assertion failure가 아니다.
- Group3는 기존 local fixture/API skew로 제외했다.

정확한 command와 제한은 [test_results.md](manifests/test_results.md).

## 13. CPU / boundedness

production candidate와 새 state가 없으므로 baseline→final 코드 경로 및 complexity delta는 **exactly zero by construction**이다. Candidate per-update mean/p95/p99/max와 replay wall time 비교는 N/A이며, prior P0 side-pass CPU 수치를 P1 candidate 수치로 재사용하지 않았다.

- new history/state count: 0
- new global matching/search: 0
- x86 P1 candidate CPU: N/A / NOT MEASURED
- **A1M CPU: NOT MEASURED**

## 14. Evidence grade

### CONFIRMED FACT

- Route269 raw482의 255-scan 연속 관측, 11 split/11 rejoin, exact pair failures, PID assignment/cleanup sequence.
- 영상 확인 SAME 4건과 sequential DIFFERENT 4건; moving DIFFERENT 0건.
- Route2bc S21 11-scan raw/PID/member/representative 안정성.
- production Bosch code가 이번 연구에서 변경되지 않았음.

### LOG-BASED INFERENCE

- Route269 P1 root는 dropout/reacquisition보다 complete-link boundary와 exclusive owner assignment의 상호작용이다.
- Route2bc S21 사용자가 느낀 현상은 P1 identity churn이 아니라 P0 publication/control interaction 경계에 있다.

### HYPOTHESIS

- bounded ancestry는 family-level diagnostic에 유용할 수 있으나 physical PID reuse 권한으로 쓰면 안 된다.
- large-object extent model은 gate 확대보다 나을 수 있으나 moving DIFFERENT와 truck+adjacent actor GT 없이 평가할 수 없다.

### NOT VERIFIED / NOT MEASURED

- hypothetical Candidate B/D false merge, real-car runtime, `radarState`/planner/control, NAS deployment, A1M CPU.

## 15. 결론과 다음 최소 조건

P1의 실제 root cause는 확인됐지만 identity precision을 보장할 데이터가 부족하다. 특히 moving-to-moving sequential DIFFERENT가 0인 상태에서 old PID resurrection, timeout extension, single-link/expanded grouping, history transfer를 활성화하면 안 된다. 따라서 최종 판정은 **INCONCLUSIVE**, production physical-PID candidate는 **NO-GO**다.

다음 연구의 entry gate는 독립 route의 moving SAME과 moving DIFFERENT actor 전이를 각각 확보하고, current-scan prediction residual·member ancestry·owner contradiction을 포함한 analysis-only SHADOW에서 먼저 `new false stitch=0`, `new actual-vehicle false merge=0`, cut-in delay=0, prefix PASS를 보이는 것이다. 그 전에는 PID continuity 숫자를 개선하기 위한 production mutation을 승인하지 않는다.
