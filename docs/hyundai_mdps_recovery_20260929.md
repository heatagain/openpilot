# Elantra legacy MDPS 경고 회복 대기

사용자가 선택한 범위는 `HYUNDAI_ELANTRA` + `steerControlType=torque` +
`HyundaiFlags.LEGACY`이다. `CANFD` 또는 `ANGLE_CONTROL`이면 제외한다.
이 fingerprint는 2017~2019년형이 공유하므로 2017년형만 식별하는 기능은 아니다.
별도 설정은 추가하지 않았다. 해당 조건에서 자동 적용된다.

## 동작

`steerFaultTemporary` 및 `controlsd`의 토크 허용 조건은 바꾸지 않는다.
Hyundai carstate/carcontroller/hyundaican/Panda safety 코드는 수정하지 않는다.
따라서 raw fault에 따른 토크 중단과 기존 clear 이후 request 재개 순서를 유지한다.
ToiActive 복귀 전까지 request를 계속 OFF로 묶는 방식은 사용하지 않는다.

card에서 원본 수신 MDPS12(0x251, bus0), SAS11(0x2B0, bus0), 송신 echo
LKAS11(0x340, bus128)을 읽어 경고용 상태만 별도로 전달한다.
기존 parser의 raw fault 값을 대체하거나 수정하지 않는다.

- 직전 정상 활성·request 상태를 확인한 ToiFlt 사건만 최대 120ms 기다린다.
  ToiFlt clear와 ToiActive=1을 실제 관측해야 대기를 끝낼 수 있다.
- ToiUnavail/Def/SErr, 단독 또는 clear 이후 지속되는 FailStat,
  MDPS/LKAS checksum·counter 이상, CAN invalid, stale/malformed/out-of-order
  입력은 대기 성공으로 처리하지 않는다. checksum 계산은 Elantra legacy
  MDPS additive 및 LKAS 기본 additive 방식에 한정한다.
- 기존 85도 고각도 guard 또는 LKAS ToiFlt request를 직전 500ms에 관측하면
  대기를 우회한다. protection-only 우회는 기존 driver/silent/event-clear
  조건으로 돌아간다. 고각도 경고를 새로운 suppression 정책에 넣지 않는다.
- 같은 receive batch의 severe/무결성 오류는 그 batch의 정상 frame보다 우선한다.
  늦게 도착한 frame으로 이미 지난 deadline의 성공을 소급하지 않는다.
- MDPS/SAS/LKAS freshness는 기존 100Hz parser의 10-period 한계인 100ms로
  제한한다. 소비 측도 sample timestamp와 120ms deadline을 확인한다.
- 대기 중에는 `steerTempUnavailablePending`의 NO_ENTRY만 유지한다.
  이미 주행 중인 경우 warning/softDisable을 생성하지 않지만 새 engage는 막는다.
- 대기 실패 시에는 `steerTempUnavailable`로 전달한다. 이때 driver override나
  silent 경고 이력 때문에 승격이 다시 생략되지 않는다.
- 승격까지 지난 시간을 기존 3초 soft-disable 예산에서 빼므로 3초를 다시 시작하지
  않는다. 판정·표시는 100Hz 실행 주기와 실제 scheduling 지연의 영향을 받는다.
- 반복 pulse에는 MDPS가 연속 3초 정상 상태인 것이 관측되기 전까지 새 grace를
  허용하지 않는다. 3초는 기존 soft-disable horizon을 재사용한 보수적 재진입 제한이며
  새 threshold sweep 결과가 아니다. 재시작 직후에는 선행 정상 관측 없이 grace를 주지 않는다.

`hyundaiMdpsRecovery`는 capnp에 append한 선택적 상태다. 기존 로그의 default는
available=false라서 이전 데이터만으로 경고를 보류하지 않는다. pending/warning/
delayed/forceWarning과 원본 onset, sample timestamp를 기록해 판정 경로를 구분한다.
raw 토크 차단 여부와 이 필드를 혼용하면 안 된다.

## 검증과 한계

- focused tests 37개: 범위 제한, checksum/counter/severe, startup, stale,
  동일 batch, deadline 전/정확히/후, repeated pulse, NO_ENTRY,
  override 시 timeout 승격, raw 토크 gate 보존, capnp roundtrip,
  3초 disable 예산, CAN 오류 전환 시 원래 deadline, protection 기존 event gate.
- HEAD 상태머신과 기본 호출 결과 7,680개 조합 동일. 새 elapsed 인자를 사용하지
  않는 차량의 기본 상태 전이·timer·alert type 결과를 비교했다.
- 기존 26개 raw episode의 MDPS/LKAS/SAS payload와 기록 carState tick을 입력한
  조건부 replay: 저각도 6건 회복, protection 19건 및 AMBIGUOUS 1건 우회/유지.
  Route2e0만 보면 2건 회복, 나머지 16건 유지.
- Windows에서 platform-only `msgq`가 없어 전체 card/selfdrived 프로세스 실행은
  하지 않았다. 통합 경계 테스트는 실제 production class/function AST와 실제
  capnp schema를 실행하고 platform/UI 의존성만 대체한다. 실제 alert rendering과
  음향 출력을 테스트한 것은 아니다.
- replay는 사건별 앞 1초부터 뒤 500ms까지의 입력이며 기록 제어 응답을 고정한다.
  제어 feedback, 전체 주행 장기 상태, 프로세스 scheduling, 실차 severe fault,
  사용자 체감 효과를 입증하지 않는다. 반복 재진입은 별도 synthetic test로 검사했다.

이 구현은 원래의 frozen counterfactual과 별도인 guarded 구현이다. raw fault에
연결된 화면 경고 완화가 목적이며 MDPS 내부 fault나 잠깐의 assist 단절 자체를
없애는 기능은 아니다. 특히 조향만 활성인 상태에서는 원래 표시 경고가 관측되지
않았으므로 그 상태의 화면 개선을 주장하지 않는다. OPKR 비교는 범위에서 제외한다.

로컬 구현·오프라인 검증 상태이다. 차량 적용·commit·push는 하지 않았다.
기존 연구의 실제 severe coverage 부족 및 물리 FailStat 의미의 불확실성은 그대로 남는다.
