# Source architecture

## 범위

이 연구는 `opendbc_repo/opendbc/car/hyundai/radar_interface.py`의 Bosch 경로를 읽고, source를 메모리 안에서만 변환해 replay한다. primary checkout, production provider, raw association, grouping, `radard`, planner, MPC, control, UI는 수정하지 않는다.

## Physical group 이후 흐름

1. `BoschRawTrackManager`가 raw detection을 track으로 만든다.
2. `BoschObjectGroupManager`가 이미 만들어진 current cluster와 이전 physical state를 연결한다.
3. `_bosch_group_representative`가 current members 중 한 개의 whole tuple을 continuity anchor로 고른다.
4. 그 representative의 `d_rel/y_rel/v_rel`가 `BoschPhysicalObject`에 저장된다.
5. 다음 scan의 PID assignment solver는 member overlap 외에도 이전 representative가 새 cluster에 있는지에 `+3`을 준다 (`radar_interface.py:3031-3033`). 따라서 representative는 publication-only 값이 아니다.
6. alias allocation 후 PID-owned `BoschLeadAccelerationEstimator`가 fresh completed scan만 갱신한다.
7. publication-only filters와 OEM-nearer surface 선택을 거쳐 `RadarData`/`liveTracks`가 된다.
8. `DPathRadarController`가 `leadOne/leadTwo`를 만들고 `radarState`에 싣는다.
9. longitudinal planner는 두 lead와 `aLeadK`를 MPC/preview 입력으로 사용한다.

## Evidence boundary

- raw association decision, moving raw hop, actor endpoint, track stealing은 이 연구에서 분석하지 않는다.
- scan-by-scan replay 차이는 offline causal implementation evidence다. 실차 안전성 또는 실제 감속 증거가 아니다.
- native longitudinal MPC/requested acceleration replay는 이 Windows 연구에서 실행하지 않는다.
