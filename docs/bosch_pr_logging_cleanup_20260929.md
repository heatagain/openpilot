# Bosch 레이더 트랙 PR 로그 정리와 조향 조사 인계

## 비교 기준과 범위

- 작업 브랜치: `heatagain/bosch-radar-tracks`
- 수정 전 HEAD: `fe400f0f466be8632c0cf8ce818f46cf70eb0b89` (작업 트리 clean)
- 비교 기준: 로컬에 저장된 `upstream/carrot-wip`의 `1e7eb8575433692431b872f96841c42a9df71c94`.
- 로컬 `carrot-wip`와 `origin/carrot-wip`는 더 오래된 `ffdc2d12fcd94b61e27087e5c215a5b3d342aeaf`이므로 PR 차이 점검에는 이미 병합된 upstream 기준을 사용했다. 이번 작업에서 원격 fetch는 하지 않았다.
- 기존 로그 파일을 삭제한 작업이 아니라 앞으로 생성될 브랜치 전용 진단 출력의 소스 정리다.

## 제거한 출력

1. `carlog.research` 로거와 `radarcan`의 swaglog IPC 연결. 이 경로는 `logMessage`/rlog용이며 stderr/tmux handler는 없었다.
2. `BOSCH_MIRROR_BIRTH_*`, `BOSCH_MIRROR_M3_*`, `BOSCH_ROAD_EDGE`, `B1_COMPANION_*`, `BOSCH_BURST_MULTIRETURN_*`, `BOSCH_SIDEPASS_LATERAL_*` 출력 및 출력 전용 문자열/반복문.
3. `radard_input_health_invalid_transition` warning과 그 실패 예외 출력. 유효→무효 전이에 서비스 alive/freq/valid/age와 radar errors를 rlog/tmux에 남기던 임시 진단이다. 도입 `178e6b6c`, 격리 `03a775eb`를 Git에서 참조할 수 있다.
4. 테스트의 폐기된 research logger mock과 빈 메시지 assertion.

트랙 계산, physical PID/alias, publication 필터, 내부 `last_events`/`last_decisions` 및 replay 점검용 상태는 유지했다. raw CAN, `liveTracks`, `radarState` 등 정상 서비스 기록과 upstream 공통 로그도 유지했다. `e9360163`의 Bosch liveTracks polling 보정은 실제 동작 수정이므로 유지했다. 조향 controller/carstate/safety 수정은 없다.

## 레이더 PR 이후 별도 조향 브랜치에서 재개

사용자 요청: 레이더 트랙 PR을 먼저 진행하고, 일시적 조향 불가 문제는 이후 별도 브랜치에서 다룬다. 이번에는 그 브랜치를 생성하거나 조향 해법을 적용하지 않았다.

현재 기준과 HEAD 사이에는 Hyundai `carcontroller.py`, `carstate.py`, 조향 이벤트/safety 파일의 브랜치 전용 차이가 없다. 제거한 radard 입력 건강 진단을 MDPS 조향 fault 진단 또는 해결책으로 간주하면 안 된다.

기존 자료: `C:/CarrotRadarResearch/analysis/steering/20260915_route28b_steer_temp_unavailable/report.md`와 같은 폴더의 `tables/event_index.csv`, `tables/causal_timeline.csv`, `tables/hypothesis_matrix.csv`, `manifests/input_manifest.json`, `scripts/analyze_route28b.py`, `scripts/validate_outputs.py`.

- 과거 분석 대상: Route `0000028b--ca855432c0`, Elantra AD, source `47533cd1554a1d9a3f916a5e318bc3b395d70ec8`.
- **CONFIRMED FACT (기존 보고서)**: 네 pulse에서 raw `MDPS12.CF_Mdps_ToiFlt` 상승이 `carState.steerFaultTemporary`보다 2.796–4.262 ms 선행했다. `ToiUnavail`은 0이었다.
- **LOG-BASED INFERENCE (기존 보고서)**: S10/S57 저각도 transient와 S78 고각도 사건을 분리해야 한다. S78은 85° 초과 약 1.073초, command 280, `MaxAngleFrames=100` request cut과 연관됐다.
- **HYPOTHESIS**: AD 고각도 request cut 시점/프레임 제한은 별도 검토 후보이며 검증된 수정이 아니다. 저각도 사건을 firmware/hardware 결함으로 단정하지 않는다.
- 재개 시 당시 raw MDPS12/LKAS11, 설정, source SHA, 최신 재발 로그를 다시 확인하고 원인별 검증부터 한다. 이번에는 과거 보고서만 다시 읽었으며 raw 로그 재분석·실차 검증은 하지 않았다.

## 검증과 보관

### 첨부 tmux와 기존 연구 기록 대조 (후속 확인)

앞선 설명은 조향 controller의 브랜치 차이와 Route28b 자료만 확인하여, 사용자가 기억한 tmux 진단과 기존 연구 기록의 연결을 빠뜨렸다. 아래 두 출력을 구분해서 인계한다.

- `C:/Users/이동윤/Documents/카카오톡 받은 파일/tmux-4.log` 972–973행, `tmux-3.log` 575/579행: `State.enabled => softDisabling [7]` / 복귀. 현재 cereal enum 7은 `steerTempUnavailable`이다. 출력 위치는 `openpilot/selfdrive/selfdrived/state.py` 41/54행이며 비교 기준 `upstream/carrot-wip`에도 동일하게 존재한다. Bosch 브랜치 전용 추가 로그가 아니므로 유지했다. 이 출력만으로 MDPS raw fault 원인을 확정할 수 없다.
- `tmux-3.log` 836/924/955/999행: `radard_input_health_invalid_transition` 임시 진단 4건. 858행에는 별도의 `softDisabling [25, 115]`가 있다. 진단에서 `liveTracks.freq_ok=False`, `all_alive=True`, `all_valid=True`, radar 오류 필드 false가 확인된다. 이 긴 구조화 출력이 이번에 제거한 `_log_invalid_input_transition`의 출력이다.
- `analysis/bosch_radar/20260916_mirror_family_shadow/report.md` 40–42행에는 research rlog 경로와 함께, upstream 대비 남겨둔 추가 tmux 출력이 요청된 `radard_input_health_invalid_transition` 및 fail-safe exception이라고 명시돼 있다. 이 경로는 workspace의 `C:/CarrotRadarResearch/analysis/` 아래다.
- `C:/CarrotRadarResearch/analysis/bosch_radar/20260918_livetracks_frequency_rootcause/report.md`는 Route29b의 같은 진단 4건을 조사한 후속 보고서다. 보고서에서는 publisher의 약 20 Hz와 consumer의 약 10–11 Hz 계수 차이, modelV2-only poll/conflate, 11.2 Hz 하한을 다뤘다. 과거 보고서의 원인 등급을 현재 실차 검증 결과로 확대하지 않는다.

조향 `[7]` 조사와 radard 입력 진단을 모두 찾았으며, 추가 진단 제거는 완료된 상태다. 향후 별도 조향 브랜치에서는 `[7]` 사건을 해당 rlog의 MDPS12/LKAS11과 대조한다. `[25,115]` 및 radard 진단을 같은 조향 사건으로 자동 합치지 않는다.

- Hyundai radar 및 Bosch camera/mirror/road-edge 기존 테스트: **561 passed, 2 failed**. 두 실패는 Group3 DBC의 `OBJECT_ID` 누락이며 보관한 수정 전 소스를 로드한 비교 실행에서도 동일하게 재현됐다.
- `test_radard_dpath.py`: Windows의 `msgq` 부재로 collection 실패, **NOT RUN**. 통과로 간주하지 않는다.
- 소스 구문 검사와 `git diff --check` 수행. 제거한 연구 로거/임시 event의 production Python 참조가 남지 않는지 확인.
- 수정 전 다섯 소스와 정리/비교 스크립트: `C:/CarrotRadarResearch/analysis/20260929_bosch_pr_log_cleanup/`. 로컬 보관이며 원격 백업은 아니다.
- 로그 출력만 정리했으며 실차 적용, NAS 배포, commit/push/PR 생성은 이 작업에 포함하지 않았다.
