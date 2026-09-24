# Downstream path

## Code path

`BoschPhysicalObject`
→ alias allocation
→ PID-owned `BoschLeadAccelerationEstimator.update`
→ Bosch publication filters / `bosch_published_surface`
→ `bosch_fill_point`
→ `RadarData.points` (`liveTracks`)
→ `DPathRadarController.update`
→ `radarState.leadOne/leadTwo`
→ `LongitudinalPlanner.update`
→ MPC/lead preview input

## Fields

- `bosch_fill_point` copies one published surface's `dRel/yRel/vRel`.
- `aRel`, `yvRel`, `jLead` are explicitly NaN; decoded Bosch acceleration을 가장하지 않는다.
- `aLead`는 physical PID가 소유하며 fresh completed scan에서만 estimator를 갱신한다. Held publication은 값을 재사용하고 estimator update는 0이다.
- estimator 입력 `vLead = vEgo + published-surface vRel`; representative/publication 변화가 speed history와 이후 `aLead`에 영향을 줄 수 있다.
- DPath primary lead output은 radar point의 `aLead`를 `aLead`와 `aLeadK`로 전달한다 (`radar_motion/primary.py:3821-3831`).
- planner는 selected lead의 `dRel/vRel/aLeadK`를 longitudinal planning input으로 사용한다.

## Replay boundary

이 연구는 current DPath controller까지 동일 입력으로 offline replay한다. `planner-input changed`는 `radarState`에서 planner가 읽는 lead tuple이 달라진 scan이다. Native MPC/acados requested acceleration은 Windows 환경에서 실행하지 않았으므로 `NOT RUN`; 실차 감속도 `NOT VERIFIED`다.
