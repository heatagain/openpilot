[실차 테스트 30초 요약]

기어 **P**, **주차브레이크**, **차량 완전 정차**, **openpilot disengaged** 상태에서 실행해 주세요.
AlwaysLateral도 비활성이어야 합니다. 아래 명령은 테스트 코드를 설치하고 card를 한 번 재시작한 뒤 사용합니다.

```bash
cd /data/openpilot
chmod +x tools/syswarning_test.sh   # 설치 후 한 번만
./tools/syswarning_test.sh status
./tools/syswarning_test.sh 0
./tools/syswarning_test.sh 1
./tools/syswarning_test.sh 2
./tools/syswarning_test.sh 3
./tools/syswarning_test.sh 4
./tools/syswarning_test.sh 5
./tools/syswarning_test.sh 6
./tools/syswarning_test.sh off
```

각 값을 입력하고 계기판 반응을 관찰한 뒤 다음 값을 입력해 주세요. 자동 순환 명령은 제공하지 않습니다.
명령마다 120초 동안 유효하며 같은 명령을 다시 입력하면 시간을 갱신합니다.
값 변경/OFF에는 rebuild나 card 재시작이 필요하지 않습니다. 설치·코드 제거 후에는 한 번 재시작해야 합니다.

`0`은 테스트 값 0을 강제합니다. `off`는 기존 DM/HUD 경고 로직을 그대로 사용합니다.
예를 들어 원래 DM 경고가 3이면 테스트 0은 이를 0으로 바꾸지만 OFF는 3을 유지합니다.

상태 확인은 값 입력 후 약 1초 뒤 실행해 주세요.

```bash
./tools/syswarning_test.sh status
```

`ON (requested)`는 요청 파일이 유효하다는 뜻입니다. `Controller state: APPLYING`은
컨트롤러의 최근 메모리 상태이며 실제 CAN 송신·계기판 표시 확인을 의미하지 않습니다.
`UNKNOWN`이면 아직 새 요청을 읽지 않았거나 card가 없거나 대상 차량 경로가 아니거나 상태가 오래되었습니다.
`GATE_BLOCKED_REISSUE_COMMAND` / `DISARMED_REISSUE_COMMAND`이면 조건을 맞춘 뒤 원하는 값을 다시 입력해 주세요.
이동·engage 후 재정차만으로는 이전 테스트가 다시 켜지지 않습니다.

작업 기준과 코드 경로

- 작업 시작 브랜치: `heatagain/bosch-radar-tracks`
- 작업 시작 커밋: `613a4ca9de7a06a1228aa8471480434d14c94056`
- 작업 시작 `git status --short`: 빈 출력. 기존 변경사항이 없었습니다.
- 기준 경로: `opendbc_repo/opendbc/car/hyundai/`. `HYUNDAI_ELANTRA`는 2017–19
  `HyundaiPlatformConfig`이며 CAN-FD 플랫폼이 아닙니다. DBC는 `hyundai_kia_generic`입니다.
- `interface.py`는 카메라 bus 2의 `0x485` 존재 시 `SEND_LFA`를 동적으로 설정합니다.
  테스트는 **정확히 HYUNDAI_ELANTRA이며 CANFD/SEND_LFA가 없는 경우**만 허용합니다.
  AD GT/i30, AD 이후 세대와 다른 차량에는 적용하지 않습니다. 실제 차량의 현재 flags는 아직 확인하지 않았습니다.

```text
CarController.update() Classic CAN 분기 (DT_CTRL=0.01, 명목 100 Hz)
  -> CS.lkas11 존재 + lkas11_active (기존 최초 한 프레임 대기 유지)
  -> hyundaican.create_lkas11(..., dm_alert=hudControl.driverMonitoringAlert)
  -> values["CF_Lkas_SysWarning"] (기존 정책, 이어서 허용된 테스트 값)
  -> packer.make_can_msg("LKAS11", 0, values)
  -> 기존 counter/checksum 계산
  -> CAN ID 832 / 0x340, bus 0, 8 bytes
```

`process_hud_alert()`가 HUD visualAlert와 차선 표시로 `sys_warning`, `sys_state`, 차선 경고를 생성합니다.
Elantra의 기본 `SysWarning`은 0입니다. 일반 `sys_warning`을 3으로 바꾸는 코드는 현재 주석입니다.
`SEND_LFA`가 없는 차량의 DM 단계 1/2/3은 Elantra에서 모두 `SysWarning=3`이며
경고가 있으면 `LdwsSysState=3`으로 표시 상태를 맞춥니다. Santa Fe/Optima는 별도 기존 분기를 사용합니다.
DM production 정책과 SEND_LFA의 별도 `LFAHDA_MFC` 경고 경로는 변경하지 않았습니다.

현재 DBC 정의는 `CF_Lkas_SysWarning : 6|4@1+`, `CF_Lkas_LdwsSysState : 2|4@1+`입니다.
두 신호 모두 little-endian unsigned이고 scale=1, offset=0입니다. 테스트 범위는 0–6으로 제한합니다.
현재 코드의 다른 차량용 색상/차임 주석을 AD의 실차 mapping으로 간주하지 않습니다.

구현 방식과 보호 조건

공용 Params 키 등록/네이티브 재빌드를 피하는 **방법 B**를 선택했습니다.
새로운 Params 키는 typed key registry 변경을 요구하며, 별도 CAN 송신기는 기존 송신과 중복될 수 있습니다.
임시 요청은 `/dev/shm/openpilot_syswarning_test/request.json`에 atomic replace로 저장합니다.
tmpfs이므로 재부팅 시 지워집니다. Python 표준 라이브러리만 사용합니다.

대상 컨트롤러의 daemon reader가 0.5초 간격으로 요청을 읽고 `status.json`을 기록합니다.
100 Hz 경로에서는 메모리 캐시와 monotonic clock만 확인하며 파일 I/O, Params 접근, thread join을 추가하지 않습니다.
reader가 2초 이상 갱신되지 않으면 테스트를 취소합니다. 잘못된/누락된/만료된 요청과 reader 시작 실패도 OFF입니다.
card 시작 전 만들어진 요청은 무시하므로 재시작 후 명령을 새로 입력해야 합니다.

아래 조건을 매 컨트롤러 프레임에서 모두 확인합니다.

- `CS.out.canValid`이고 `not CS.out.canTimeout`
- `CS.out.standstill`
- finite `abs(CS.out.vEgo) < 0.1` 및 `abs(CS.out.vEgoRaw) < 0.1` m/s
- `CS.out.gearShifter == park` 및 `CS.out.parkingBrake`
- `not CC.enabled`, `not CC.latActive`, `not CC.longActive`

하나라도 이탈하면 해당 요청 token을 취소하고 같은 프레임부터 기존 경고 로직으로 복귀합니다.
정상 조건으로 돌아와도 새 명령이 필요합니다. 요청 자체는 120초 후 만료됩니다.
OFF/값 변경은 다음 reader 갱신에 반영됩니다(명목 0.5초; reader 지연 시 최대 2초 freshness gate).
기존 CarState가 제공하는 CAN 유효성에 의존하며 센서 고장이나 실제 차량 정지를 독립적으로 인증하지 않습니다.

허용된 테스트 값은 **기존 DM 처리 뒤, checksum 계산 앞**에서 적용합니다.
1–6은 기존 DM 표시 조건을 재사용해 `LdwsSysState=3`으로 맞춥니다.
0은 `SysWarning`만 0으로 바꾸며 `LdwsSysState`는 기존 계산값을 유지합니다.
토크 `CR_Lkas_StrToqReq`, `CF_Lkas_ActToi`, `CF_Lkas_ToiFlt`, 차선 경고,
rolling counter와 checksum 알고리즘은 그대로입니다. 경고 값을 바꾸면 checksum 결과 바이트는 정상적으로 재계산됩니다.

설치와 복구

테스트는 로컬 구현이며 차량 설치/커밋/push는 수행하지 않았습니다.
제공된 `syswarning_test_changes.patch`를 차량의 `/data/syswarning_test_changes.patch`로 복사한 뒤,
차량 체크아웃과 다른 변경사항을 먼저 확인해 주세요. 현재 브랜치/코드에 맞지 않으면 apply 검사가 실패하며 강제로 적용하지 않습니다.

```bash
cd /data/openpilot
git branch --show-current
git rev-parse HEAD
git status --short
git apply --check /data/syswarning_test_changes.patch
git apply /data/syswarning_test_changes.patch
chmod +x tools/syswarning_test.sh
```

P단·주차브레이크 상태에서 장치의 정상 재시작 기능으로 한 번 재시작한 후 원하는 값을 입력해 주세요.
Python 코드만 변경했으므로 이 기능을 위해 새로운 Params/native/Panda build를 요구하지 않습니다.
장치의 기존 시작 절차가 하는 빌드는 별개입니다.

기능만 OFF:

```bash
cd /data/openpilot
./tools/syswarning_test.sh off
```

약 1초 뒤 status를 확인해 주세요. 테스트 OFF에서도 production DM 경고는 정상적으로 표시될 수 있습니다.

코드 완전 제거(이번 변경만 역적용):

```bash
cd /data/openpilot
./tools/syswarning_test.sh off
git apply -R --check /data/syswarning_test_changes.patch
git apply -R /data/syswarning_test_changes.patch
git status --short
```

검사에 실패하면 중단하고 충돌 hunk만 검토해야 합니다. 파일 전체 복사/reset/clean으로 우회하지 마세요.
이 절차는 patch와 겹치지 않는 다른 작업을 보존합니다. 같은 hunk나 이번에 추가한 파일을 나중에 편집했다면 검사가 실패할 수 있습니다.
제거 후 장치의 정상 재시작 기능으로 한 번 재시작하면 메모리에 남아 있던 테스트 reader도 종료됩니다.
원복 patch는 저장소 밖 `/data`에 보관하므로 코드 제거로 함께 사라지지 않습니다.

변경 파일과 검증

기존 파일 두 개:

- `opendbc_repo/opendbc/car/hyundai/carcontroller.py`: 대상 차량에서만 reader 생성, 캐시/gate 결과를 기존 LKAS11 호출에 전달.
- `opendbc_repo/opendbc/car/hyundai/hyundaican.py`: optional 테스트 인자, 경고/표시 상태만 checksum 앞에서 변경.

추가 파일 네 개:

- `opendbc_repo/opendbc/car/hyundai/syswarning_test.py`: reader, gate, CLI와 상태 확인.
- `tools/syswarning_test.sh`: 위치 독립적인 한 명령 wrapper.
- `opendbc_repo/opendbc/car/hyundai/tests/test_syswarning_test.py`: CAN decode/gate/CLI/controller 회귀 검증.
- `docs/elantra_ad_syswarning_test.md`: 이 문서와 실차 관찰 표.

Bosch raw/tracks/grouping/PID/reacquisition/aLead/provider/planner/longitudinal/fusion/SCC 코드는 수정하지 않았습니다.
작업 전 기존 파일 백업, 원본 비교 runner, 전체 diff와 검증 로그는 PC의
`C:\CarrotRadarResearch\analysis\20261001_lkas_syswarning_test\`에 보존합니다.
reverse patch는 작업 시작 스냅샷과 이번 추가 파일만 포함합니다.

검증 결과:

- 작업 직전 `hyundaican.py`와 수정본의 OFF CAN payload/address/bus **2,560/2,560 완전 일치**.
  Elantra, SEND_LFA Sonata, Santa Fe CRC8, Sonata LF 6B, Optima를 포함하며
  counter wrap, enabled, HUD warning, DM 0–3, stock-field 입력을 바꾸었습니다.
- 작업 직전 controller+formatter와 수정본의 OFF **6,000/6,000 프레임 모든 CAN/actuator 일치**.
  Elantra stock-long 설정에서 속도·조향 토크·각도·engage·HUD·DM 입력을 변화시켰습니다.
- 신규 회귀 및 기존 DM/MDPS 회귀 **247 passed**. DBC decode로 0–6, 표시 상태,
  checksum/counter, 나머지 신호 보존, 다른 차량 차단을 확인했습니다.
  gate 이탈, 재정차 후 재명령, NaN/Inf, 잘못된 파일, 만료, reader 지연,
  card 재시작, 0/OFF, background reader, 실제 controller 실시간 전환도 포함합니다.
- Python 문법 및 wrapper `bash -n` 검사 통과. 실제 wrapper의 0/status/off/status 실행 통과
  (WSL에는 card가 없으므로 controller 상태 UNKNOWN이 정상입니다).
- 실제 작업 트리의 reverse `--check` 통과. 별도 임시 복사본에서 forward/reverse 왕복 후
  원래 파일 내용 복구 및 새 파일 제거를 확인했습니다. 같은 수정 파일에 추가한 별도 사용자 메모는 보존되며,
  나중에 수정된 새 파일은 제거 검사 실패로 보호됩니다. `git diff --check`도 통과했습니다.
- Windows 기본 import는 `params_pyx` 부재로 실패했습니다. 기존 WSL Python 3.12.14와
  기존 Linux native Params 의존성을 사용해 **현재 체크아웃 소스**를 검증했습니다.
  설정 값은 테스트에서 deterministic fixture로 제공하며 실제 차량 Params/IPC 검증은 아닙니다.
  설치된 pure-Python pytest를 재사용하고 병렬/기타 플러그인을 끈 결과,
  사용하지 않는 asyncio/cpp 설정 관련 경고 세 개가 있습니다.
  최초 controller 테스트의 builder/reader fixture 오류는 reader 입력으로 수정한 후 전체 통과했습니다.

실제 차량 CAN wire capture, 계기판/차임,
장치 스케줄링 지연과 실제 card 재시작은 아직 검증하지 않았습니다.
1–6의 화면/색상/점멸/차임과 `LdwsSysState=3`의 AD 표시 효과는 실차 관찰 대상입니다.
결과를 production warning mapping에 반영하는 작업은 별도입니다.

실차 관찰 표

| SysWarning | 화면/문구 | 아이콘 | 색상 | 점멸 | 차임 | 기타 |
|---:|---|---|---|---|---|---|
| 0 | | | | | | |
| 1 | | | | | | |
| 2 | | | | | | |
| 3 | | | | | | |
| 4 | | | | | | |
| 5 | | | | | | |
| 6 | | | | | | |

핸들 아이콘, LKAS 경고 문구, 차선 아이콘, 노랑/빨강, 단발/반복 차임을 각각 기록해 주세요.
각 값은 위의 `LdwsSysState` 조건과 함께 관찰한 결과라는 점도 기록해 주세요.
