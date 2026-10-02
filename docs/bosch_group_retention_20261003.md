# Bosch mature group retention: inactive experimental port

2026-10-03, `heatagain/bosch-radar-tracks`, pre-edit HEAD
`c75fc0ae5a6ea6b2cf084d99d42017069148db23`.

## 적용 범위와 기본 동작

`SHAPE_MEMORY_2000` 연구 후보를 `radar_bosch.py`의 명시적 인증서 타입과
`_BoschMatureGroupRetention`으로 이식했습니다. 기본값은
`BoschGroupingConfig.mature_retention_enabled=False` 및
`BoschRadarProvider(..., mature_group_retention=False)`입니다. 기본 manager는
retention 저장소를 할당하지 않습니다. Params, schema, UI 및 다른 제조사 경로는
추가하거나 변경하지 않았습니다. 활성 실험은 명시적으로 생성자에서 선택해야 합니다.

엄격한 pair evidence 생성·삭제와 complete-link 형성, 기존 raw/PID 소유권,
representative 선택 및 common ancestry 경로는 유지했습니다. 실험 옵션을 켰을 때만
엄격한 pair 계산 뒤, 기존 complete-link solver 앞에서 이전 스캔의 정확한 raw
멤버 집합에 제한된 retention edge를 제공할 수 있습니다. 이를 새 pair evidence로
기록하지 않으며, 분리된 자식 그룹에 edge를 제공하여 강제로 합치지 않습니다.

## 후보 규칙과 저장 상한

- 이전 exact family가 엄격한 pair evidence로 2초 이상 성숙한 경우만 적용합니다.
- 마지막 strict 관측부터 최대 2초 동안 유지하며, archive는 최대 3초입니다.
- 거리 지름은 기존 값 +0.25m, 횡방향 지름은 기존 값 +0.03125m를 넘지 않습니다.
  상대 속도차 0.5m/s 초과 및 stationary/moving 충돌은 유지할 수 없습니다.
- strict outsider 또는 5m/2m/1m/s 이내 near outsider가 있으면 거부합니다.
- 멤버 누락, age 비증가, 입력 gap, extent/motion/shape 성장과 인증서 만료는
  인증서를 취소합니다. archive는 다른 멤버와 섞인 이전 그룹도 거부합니다.
- 역사적 component 성장이 있더라도 최근 strict 관측의 벡터 길이 +0.25m
  안이면 회전 설명을 허용합니다. 최근 strict 기준이 없거나 길이도 성장하면 거부합니다.
- archive는 같은 exact family가 기존 엄격 조건으로 자연스럽게 재합류한 뒤에만
  성숙도 재사용을 허용합니다. PID 상태나 다른 PID의 history는 복사하지 않습니다.
- live family 16개, archive family 64개, 인증서당 strict history 21개,
  현재 raw age 32개로 제한합니다. archive 초과 시 마지막 strict 시각의 최신순,
  family tuple 순으로 결정적으로 남깁니다.

## 인증서 snapshot 복사 최적화

Archive 저장과 자연 재합류 복원에서는 typed `clone()`으로 인증서를 복사합니다.
History list와 각 offsets dict는 별도로 복사하고, 반복된 entry/dict의 내부 참조는
보존합니다. Producer가 만드는 int/float tuple 등 immutable leaf는 공유합니다.
임의 subclass, 추가 attribute 또는 mutable leaf에 대한 일반 `deepcopy` 대체는
아닙니다. Int32 overflow의 manager transaction staging은 기존 `deepcopy`를 유지합니다.
Retention 조건, history 값, PID 소유권, 기본 OFF 정책은 바뀌지 않습니다.

후속 `20261003_bosch_group_certificate_perf_v6` 검증에서 affected 148파일의
88,821 records 및 최신 Route2ee 34파일의 연속 19,936 records를 비교했습니다.
Route2ee는 최초 cold start 1회와 상태를 유지한 파일 경계 33개를 포함합니다.
Provider/certificate/owner/stats/events 및 전체 lead IEEE 값의 불일치는 0입니다.
Mutable snapshot 격리, 반복 내부 alias, manager serialization과 overflow transaction을
검사하는 5개 회귀 테스트를 추가해 집중 테스트는 총 28개입니다.

32-return 4×8 구성의 같은 입력 paired desktop WSL 측정에서 manager 중앙값은
7.564ms에서 2.412ms로 줄었습니다. 다른 연구가 함께 실행된 desktop 측정이며,
실차 CPU·scheduling deadline 또는 물리 객체 동일성 검증을 대신하지 않습니다.

## 검증과 근거

새 결과 디렉터리는
`C:\CarrotRadarResearch\analysis\20261003_bosch_group_production_port_v4`입니다.
기존 연구 스크립트·보고서·입력 cache는 읽기만 했습니다. 이식 전 소스와 검증용
포트 소스를 각각 `pre_edit_radar_bosch.py`, `frozen_port_radar_bosch.py`에 저장했습니다.

집중 테스트 `test_bosch_mature_retention.py`의 23개 사례가 통과했습니다.
boolean 검증, 기본 모드, 엄격 pair evidence 불변, lease 경계, 미성숙/미형성,
geometry/motion/outsider, 멤버 누락, age rollback, gap, shape, natural rejoin,
archive pruning 및 상한, timestamp 거부의 원자성을 검사합니다.

봉인 후보와 포트의 synthetic mechanics 비교는 15개 사례 × 65스캔 = 975스캔에서
불일치 0입니다. `ambiguous_independent_pair`는 기계적 동등성만 확인하며,
물리 객체 동일성 보호를 입증하는 사례로 승격하지 않습니다.

Windows 일반 pytest 실행은 `params_pyx` 부재로 수집 단계 NOT RUN입니다.
두 번째 시도는 pytest-xdist 부재에 따른 기존 `-n/--dist` 옵션 오류였습니다.
환경 의존 conftest 및 해당 옵션을 제외한 집중 실행에서 23개 테스트가 통과했습니다.
세 번째 시도의 motion fixture는 strict edge에 retention guard를 요구해 실패했으며,
비엄격 edge 상황으로 수정한 뒤 통과했습니다. 모든 시도 로그를 보존했습니다.

기록 입력의 선택된 full-provider/downstream 동등성 결과는
`parity_result.json`에서 확인합니다. 비교는 기본 모드 대 이식 전 소스와 활성 모드
대 동결한 `SHAPE_MEMORY_2000` prototype입니다. provider 전체 내부 상태에서 실험
컨테이너와 새 config bit만 제외한 exact digest, raw/group/PID/소유권, qualification,
publication, alias, aLead 및 Python `DPathRadarController` 리드를 포함합니다.
검증 도구는 NumPy assignment fallback을 사용하므로 native solver/IPC 증명은 아닙니다.

선택된 Route269 S7, Route280 S15, Route2bc S21, Route255 S38, Route259 S11의
총 2,999스캔에서 두 비교 모두 snapshot 및 full-provider 상태 불일치 0이며,
retention 통계와 저장 peak도 정확히 일치했습니다. 활성 후보는 85 group-scans에서
85 edge를 유지했습니다. 이는 선택된 이식 동등성이고 전체 corpus 재검증이 아닙니다.

Replay freeze 뒤 스타일 수정은 `zip(..., strict=False)`의 기존 기본값 명시,
이전 그룹의 `set(generator)`를 set comprehension으로 변환, retention event의
`dict(keyword)`를 같은 key/order의 literal로 변환한 세 표현뿐입니다.
`final_style_equivalence.json`은 정확히 이 세 위치만 정규화한 AST의 전체 일치와
최종 focused tests 23개 통과를 연결합니다. 기존 Ruff 지적 28개는 그대로이고,
포트 및 새 테스트의 추가 지적은 0입니다. 관련 없는 기존 지적은 수정하지 않았습니다.

SHA-256:

- 이식 전: `1838d9be3456169fb2ae546112b39ef341c90c3e2cd5e49d160a89d1cb3a315b`
- Replay freeze: `ed4a89fa7a928f256ca8a2bb00a2648e9739ac732be873b04c25d65ff379dbe7`
- 최종 포트: `d27215c146145e4af92ec37b4069e706e9c5fe5122bbe3e714e407fd42255a1f`

## 상태 구분

**CONFIRMED FACT:** 기본 비활성 API 이식 및 집중 테스트·synthetic mechanics 동등성입니다.
기록 replay 동등성은 선택된 입력과 비교 범위에 한정됩니다.

**UNVERIFIED / AMBIGUOUS:** 이전 strict family의 연속성은 물리 객체 SAME 증거가
아닙니다. 이전 연구의 frozen SAME/DIFFERENT actor family에 실제 retention 노출은
0이었고, 새 merge의 물리 동일성은 UNVERIFIED, 최신 영상 사례 8/8은 AMBIGUOUS였습니다.
독립 짧은 family IO01은 0.4005초 성숙도로 후보의 2초 규칙에서 그대로 남았습니다.

**NO-GO:** 활성 production 기본값 및 실차 적용입니다. 비활성 실험 포트의 저장소
통합·공유와 활성화/배포 허가는 별개입니다. 실제 후보가 개입한 SAME 및 moving
DIFFERENT 대조군, 짧은 family 범위, native downstream/IPC/MPC/control/차량 검증이
후속 근거로 필요합니다. 이번 Windows replay는 이를 대신하지 않습니다.
