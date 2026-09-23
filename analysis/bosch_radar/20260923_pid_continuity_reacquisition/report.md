# Bosch MRRevo14F physical PID continuity / reacquisition 감사

## 판정

**INCONCLUSIVE; distinct-PID stitching의 production 적용은 NO-GO.** 현재 HEAD에서 확인된 핵심 실패는 무조건적인 raw 소실 후 재획득이 아니라 **동일 raw return의 group 편입/분리와 surface migration에 따른 PID 분절**이다. 과거 영상 GT의 SAME sequential transition 12건 중 4건만 현재 HEAD의 양 endpoint를 유일하게 재매핑할 수 있었다. 4건 모두 distinct PID였고, 그중 Route269 S7의 3건은 같은 raw ID가 기존 group PID와 신규 singleton PID 사이를 오간다. 남은 8건은 현재 객체 후보가 겹쳐 재매핑이 불확실하다. 동일 feature 범위의 영상 확인 DIFFERENT sequential transition은 확보하지 못했다. 따라서 오래된 PID에 새 PID의 history를 붙이는 정책은 설계·활성화하지 않았다.

이 연구는 **offline replay**다. 실차 적용, NAS 배포, 현재 `radarState`/planner/제어 동치 및 A1M CPU는 **NOT VERIFIED / NOT MEASURED**다.

## 1. 목적과 checkout

- 목적: 실제 같은 객체의 불필요한 PID 분절 원인을 규명하고, 다른 객체를 잇는 false stitch를 우선 방지한다. PID 숫자 수명은 성공 지표가 아니다.
- production checkout: `C:\CarrotRadarResearch\openpilot`; 시작 branch `heatagain/bosch-radar-tracks`, HEAD `d5c8b555351e6b38422446aecfb9fb20820d6432`.
- 시작 `git status --short`, `git diff --name-only`, `git diff --check`: 모두 empty. `origin`은 `https://github.com/heatagain/openpilot.git`, `upstream`은 `https://github.com/ajouatom/openpilot.git`.
- replay한 provider 파일 SHA-256: `76055c522059469f3bef686476bc9cb0b6fe1499960d4e68cff7a81bf982524d`.
- 연구 root `C:\CarrotRadarResearch`는 Git repository가 아니므로 연구 결과를 그 아래 지정 경로에도 복사하고, 동일한 reviewable 원본은 이 checkout의 `analysis/bosch_radar/20260923_pid_continuity_reacquisition/`에 둔다. `scratch/`의 압축 per-scan cache는 로컬 재현자료로 유지하고 Git에는 넣지 않는다.

## 2. 현재 baseline 구조 — 코드 기준

`radar_interface.py`의 경로는 `0x602`–`0x611` Bosch track raw CAN → 32개 half-slot decode → `BoschRawTrackManager` raw ID → `BoschObjectGroupManager` complete-link member group 및 physical PID → 대표 raw return → camera/OEM association → qualification → alias allocation → Bosch publication filters → `RadarPoint` → `radarcan`의 `liveTracks` → `radard_dpath`의 `radarState.leadOne/Two/...`다. 마지막 두 단계는 코드 경로를 읽었고 이번 재생에서 실행하지 않았다.

| identity 층 | 현재 의미 | 결정 근거 |
|---|---|---|
| raw slot | 매 scan의 0–31 위치; CAN 주소 `0x602 + slot//2` | raw ID가 다른 slot으로 이동할 수 있다. |
| raw track ID | one-to-one 운동학적 연계의 내부 ID | scan-time `d+vRel·Δt`와 yaw 회전 좌표의 `|Δd|≤3.5 m`, `|Δv|≤2 m/s`, 거리별 bearing gate 12°/<15m, 5°/<30m, 2.5°/<60m, 2°/그 이상; 비용은 정규화 제곱 3항 평균에서 same-slot 보너스 최대 0.03을 뺀 값. 경쟁 후보를 포함한 전역 assignment. 최근 관측 후 0.3초까지 내부 coast하며 미관측 raw는 출력하지 않는다. |
| member/group | 복수 raw return의 complete-link 묶음 | `|Δd|≤3m`, `|Δy|≤1.5m`, `|Δv|≤1.5m/s`, stationary/moving 충돌 배제, 최소 3관측·0.18초, pair gap≤0.16초, evidence window 0.8초. 모든 member pair가 통과해야 한다. |
| physical PID | group에 부여된 `>=1000000` 내부 ID | 최근 0.3초 member ownership overlap이 있을 때 `10×overlap + 3×대표 member 유지 + 작은 age/tie` score로 종전 PID를 승계한다. 겹치는 raw 이력이 없으면 새 PID. group state/coasted member는 0.3초 후 만료하고, 다른 group에 흡수된 member의 종전 ownership은 제거한다. |
| representative | 동일 PID 내부 대표 raw | 이전 대표 위치를 `d+vRel·Δt`, yaw로 투영해 연속성을 평가한다. 대표 교체는 PID 교체와 다르다. |
| camera/OEM | association 및 publication 보조 증거 | camera curve reacquire는 `ACTIVE`지만 **같은 physical PID의 camera episode**만 최대 800ms 회복한다. 70–100m, yaw≥0.015rad/s, OEM word1, lateral≤2.35m, bearing≤0.026rad 등으로 제한된다. distinct PID를 이어주지 않는다. OEM selection 자체는 physical identity가 아니다. |
| public alias | `trackId` 32–95의 allocator binding | physical PID와 독립적이며 미게시 binding은 최대 0.5초 grace, 새 PID는 다른 alias를 받을 수 있다. tracking은 alias를 읽지 않는다. |

`_provisional_birth_view`, P91 `ACTIVE`, OEM gate `ACTIVE`, camera-extended `ACTIVE_TEST`, B1/family, Candidate B burst, B5 `ACTIVE` 등은 final publication에 작용한다. 이들은 raw/member/PID merge를 수행하지 않는다. `NO_CURRENT_RAW` 상태를 PID 사망으로 간주하거나, 미게시를 PID 단절로 간주하면 안 된다. provider timeout은 일부 publication/filter history를 reset하지만 현재 코드에서 0.3초 raw/group state의 distinct-PID 강제 stitching은 없다.

## 3. 데이터, 재생, GT 경계

- 현재 HEAD로 요청 사건 중심 **14 segment / 8,398 완료 scan**, 과거 영상 GT의 sequential SAME 후보 **9 segment / 5,286 완료 scan**을 cold segment replay했다. 합계 23개 입력 segment / **13,684 scan**이다. Route26d S14→S15와 Route28f S35→S36→S37은 한 provider로 이어 **추가 3,000 scan**을 warm replay했다. 중복 scan을 새 독립 근거로 합산하지 않는다.
- 입력은 기존 847-segment decoded offline cache의 해당 segment이고, 대상 14개 모두 로컬 `rlog.zst`와 `qcamera.ts` 파일이 있다. 세부 경로 및 scan 수는 [input_inventory.csv](manifests/input_inventory.csv). 영상은 이번 run에서 새로 라벨링하지 않았다.
- prior video GT 출처: `analysis/bosch_radar/20260913_ldws_pid_reacquisition_shadow/tables/transition_ground_truth.csv` (SHA-256 `3a1fb8ef07e36dce3aa6fdfbdb1c20f8f60656de200cb17db8f3a47123f21c2e`)와 `analysis/20260920_MRRevo14F_surface_fragment_parent_association/tables/typeA_gt_cumulative_v3.csv` (SHA-256 `f5bfd0f169cf2d91fd17b64f03032ecc09646d464d27dfdbb0e81c1ea14775ce`). 옛 PID 숫자를 현재 PID라고 가정하지 않고 endpoint 절대시간·geometry로 재매핑했다. 2m/2m/2m/s 창 안의 유일 객체만 `UNIQUE_GEOMETRY`로 적고, 복수 후보는 `AMBIGUOUS_REMAP`으로 남겼다. 이는 새 영상 GT가 아니다.
- [forensic_focus.csv](traces/forensic_focus.csv)는 `time / raw slot / raw ID / physical PID / public ID / dRel / yRel / vRel / members / representative / camera / OEM / publication / GT`를 **scan별**로 담는다. 순차 후보의 양 endpoint ±1.5초는 [sequential_endpoint_windows.csv](traces/sequential_endpoint_windows.csv), 현재 재매핑 판정은 [sequential_head_remap.csv](tables/sequential_head_remap.csv)에 있다. 내부 state만 있고 qualified object가 없는 row는 `INTERNAL_ONLY`로 표기했다. 이 row만으로 raw 미관측과 qualification 탈락을 더 분리할 수 없다.

## 4. discontinuity taxonomy와 forensic 판정

| 유형 | HEAD에서의 관측 | identity 해석 |
|---|---|
| slot 변경 | raw ID와 PID를 유지한 이동이 반복됨. | PID 단절 아님. |
| member/representative 변경 | Route264 S6 PID1000000의 `{1,9,25}`→`{9,25}` 및 대표 `1`→`9`. | group 구성과 대표 변화 자체는 PID 단절 아님. |
| group split / surface migration | 같은 scan에 기존 PID1000000 `{9,25}`와 신규 PID1000037 `{1}`이 공존. Route28f S32의 PID1000539/1000540도 두 scan 공존 후 raw500이 기존 PID1000539의 member로 편입. | prior GT의 SAME surface family이지만 단일 사망→재획득이 아니다. TYPE5. |
| group owner oscillation | Route269 S7: raw482가 PID1000618 singleton → 다음 scan PID1000462 `{436,482}` group → PID1000634 singleton → 다시 group에 귀속. 별도 두 시점도 같은 패턴으로 PID1000790→1000817, PID1000832→1000849. | 동일 raw의 관측 지속 중 group 소유권 이동으로 신규 singleton PID가 생긴다. 이전 singleton PID를 무조건 되살리면 현재 group owner와 충돌할 수 있다. TYPE5. |
| 짧은 물리 state coast | qualified row가 사라져도 state가 남는 scan 존재. | 미게시/미관측을 PID death로 셀 수 없다. |
| qualification / HOLD / DROP | Route28b S33 PID1000118은 현재 replay에서 5개 qualified scan 모두 미게시; Route28f S37 PID1000369는 첫 scan 게시 후 두 qualified scan 미게시 및 내부 state coast. | publication 변화이며 distinct-PID stitch 증거가 아니다. 현재 cold replay와 과거 recorded lead 결과를 동치로 읽지 않는다. |
| alias-only churn | 대상 cold replay의 동일 PID 연속 관측에서 alias switch **0회**. 신규 PID는 별 alias를 받음. | alias 숫자를 physical identity로 쓰지 않는다. |
| segment / process reset | cold segment replay의 PID 번호가 prior warm/route replay와 달라질 수 있다. | Route26d S15 Track88/73의 옛 numeric PID를 현 HEAD에 직접 대입하지 않는다. |

**SAME / DIFFERENT / AMBIGUOUS:** [ground_truth_cases.csv](tables/ground_truth_cases.csv)의 개별 provenance를 사용한다. `CONFIRMED FACT` prior USER VIDEO GT의 Route269 S7 sequential 세 쌍은 같은 box truck이며, 현재 HEAD에서는 각각 distinct singleton PID로 유일 대응된다. `LOG-BASED INFERENCE` 현재 raw482는 사이 scan에서 기존 group의 member라 원인은 group ownership oscillation이다. Route266 S16의 prior 영상 GT는 적재 트럭 SAME이며 현재 양 endpoint는 PID1000384/raw358과 PID1000394/raw277로 유일 대응되지만, distinct raw surface라 단순 PID stitch 안전성으로 환원하지 않는다. Route259 S2의 prior `DIFFERENT_OTHER` 두 PID1000004/1000106은 현재 scan에도 동시에 존재하고 병합되지 않는다. 이것은 확인된 **동시 DIFFERENT control**이며 사망→신규 birth형 hard negative는 아니다.

prior sequential SAME 12건 중 현재 양 endpoint 유일 대응은 **4/12**, 모두 distinct PID; **8/12는 재매핑 불확실**이다. 현재 재매핑 성공률을 physical SAME reacquisition 성공률로 부르지 않는다. `SAME reacquisition rate = UNEVALUABLE`, confirmed sequential DIFFERENT denominator `0`으로 false-stitch 안전률도 `UNEVALUABLE`이다. 기존 2026-09-21 lead-ahead 연구의 가까운 세 후보도 직접 동일 차량 GT가 없어 `AMBIGUOUS`를 유지한다.

## 5. 주요 regression 및 cut-in

| 사건 | 이번 HEAD 확인 | 한계 |
|---|---|---|
| Route26d S14/S15 Track88/73/0 | S14→S15 warm 1,200 scan 재생, [warm trace](traces/route26d_s14_s15.csv). 과거 #88의 585→590/PID1000603→1000608 식별자는 현재 warm 재생에 직접 대응되지 않는다. #0은 별도 SCC alias이지 Bosch physical PID가 아니다. | 과거 qcamera 판정은 보존하지만 현 HEAD의 같은 숫자 비교는 `NOT VERIFIED`. |
| Route28b S33, Route28f S32/S37 | 세 segment 현재 재생. S32는 raw499/500의 동시 singleton→group 편입; S37의 신규 PID 게시 1 scan 확인. | S32는 앞 segment가 없고 S33/S37 recorded downstream lead parity는 이번 연구에서 `NOT VERIFIED`. |
| T33/P91 | Route27f S16, Route274 S33 현재 재생. | 사건 단위 영상 GT/false-merge 판정은 새로 하지 않았다. |
| Route259 S11, Route264 S22 | 입력 가용·각 600 scan 재생. | 해당 segment의 특정 sequential SAME/DIFFERENT 사건 GT는 `NOT AVAILABLE`로 남긴다. |
| 실제 cut-in Route280 S15 | prior video GT의 진입 차량은 cold replay PID1000004/raw5로 전체 599 scan 동안 유지. 첫 완료 scan `3365390867699 ns`에 qualified 및 published, prior first raw `3365290715397 ns` 뒤 약 100.15ms. 진입 시점 `3388790105081 ns`에도 published. | 현 HEAD의 `lead eligible`, radarState, 제어 시각은 `NOT MEASURED`; baseline/candidate 지연은 candidate가 없어 N/A. |
| 근접 두 실차·반대차로·원거리 lead·구조물 | Route259 S2 DIFFERENT/vehicle-structure, Route26d S96 lead-ahead, Route280 S7 fragment 등 입력을 재생. | 각 category의 현재 영상 기반 false stitch/merge 검증 분모는 `UNEVALUABLE`; 범주 전체 통과 주장 없음. |

## 6. 상태기계, prefix, false stitch

birth→maturity→disappearance/coast→expiration의 raw/group state와 alias grace는 별개다. 현재 코드는 raw/group 0.3초 coast 및 absorbed-member ownership 제거를 사용한다. Route269 S7에서 stale singleton PID를 억지로 되살리면 raw482가 이미 group PID1000462에 귀속된 scan과 충돌한다. 이는 설계 단계의 구체적 false-stitch/false-merge 위험이다. 서로 다른 실제 차량의 사망→신규 birth 전이 GT가 **0**이므로 어떠한 새 stitch 정책의 `false stitch 0`도 주장할 수 없다.

현재 HEAD cold provider에 대해 사건 이후까지 잘라 재실행한 **5/5 prefix**의 모든 identity/physical-state/publication signature가 full replay 해당 prefix와 일치했다: Route26d S15 147 scan, Route28f S32 521, S37 322, Route264 S6 52, Route259 S2 254. [prefix_event_invariance.csv](tables/prefix_event_invariance.csv). 초기 prefix 6회도 통과했다. 이 검사는 **현재 baseline의 offline prefix 일치**이며 새 candidate, process reset, downstream prefix를 검증하지 않는다.

## 7. candidate, 비교, 성능

production candidate와 SHADOW decision rule은 **구현하지 않았다**. 현재 확인된 주된 문제는 distinct-PID 재연결보다 complete-link group owner 충돌과 surface migration이다. 가장 작은 다음 연구는 신규 stitch 없이, `old PID / new PID / raw member owner / group 경쟁 후보 / scan gap / 예측 d·y / vRel residual / camera·OEM 보조 상태`를 current scan까지의 정보만으로 trace하는 것이다. 단일 nearest-neighbor나 shared camera ID만으로 SAME을 선택하지 않는다.

| 지표 | baseline 관측 | candidate |
|---|---|---|
| SAME 불필요 PID 단절 | 유일 재매핑된 prior SAME 4건 모두 distinct PID; 8건 unresolved | N/A |
| DIFFERENT false stitch | 확인된 동시 DIFFERENT control 1쌍에서는 병합 없음; sequential DIFFERENT GT 0 | N/A / UNEVALUABLE |
| false merge / 신규 false drop / true lead loss | 대상 현 HEAD provider trace 일부만 관측 | N/A / NOT VERIFIED |
| 실제 cut-in 최초 publication | Route280 S15 첫 완료 scan에 published | N/A |
| public alias switch within one PID | 대상 replay event scan에서 0 | N/A |
| CPU mean/p95/p99/max·object count·wall time | **NOT MEASURED**; A1M CPU **NOT MEASURED** | N/A |

## 8. 검증 결과와 남은 위험

- `test_radar.py`의 Windows Params shim 실행: **244 passed, 3 deselected** (`-k 'not group3'`). 전체 실행은 **245 passed, 2 failed**이며 두 실패는 기존 Group3 `KeyError: 'OBJECT_ID'` fixture/DBC skew다. 직접 pytest는 Windows의 `openpilot.common.params_pyx` 부재로 collection 실패했다. 이를 Bosch assertion 회귀로 세지 않는다.
- 현 HEAD provider replay 23개 입력 segment, warm chain 2개, 사건 prefix 5개는 실행됐다. generic radard/MPC/control, 실제 recorded `liveTracks→radarState` parity, 실차 runtime 및 A1M CPU는 실행하지 않았다.
- `scripts/validate_study.py`: provider hash, 14개 priority input/8,398 scan, 12개 sequential GT의 4/8 재매핑 구분, 사건 prefix 5개 및 Route269 raw owner lineage를 검증해 PASS했다.
- 알려진 위험: 영상 GT의 현 HEAD 일부 endpoint 재매핑 모호성(8/12), 확인된 DIFFERENT sequential hard negative 0, segment cold-start와 runtime warm 상태의 차이, current replay의 qualified-only 추적으로 raw 미관측과 qualifier 배제를 완전 분리하지 못함, group owner 충돌, camera/OEM ID의 물리 identity 불충분성.

**다음 최소 작업:** Route269 S7의 raw482/group owner를 `trace_decisions`까지 켜서 scan별 ownership score와 competing object를 기록하고, 동일 운동학 envelope의 영상 확인 DIFFERENT 사망→birth 전이를 독립 route에서 확보한다. 그 뒤 bounded/causal SHADOW만 만들고 false stitch·cut-in·prefix·CPU를 먼저 검증한다. 이번 결과로 production PID/history mutation을 승인하지 않는다.
