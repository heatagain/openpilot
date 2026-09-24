# Event definitions

연속 completed scan에서 같은 numeric physical PID를 비교한다. Raw association과 actor identity를 새로 판정하지 않는다.

| Code | Mechanical definition |
|---|---|
| R1 `STABLE_GROUP_REP_CHANGE` | PID 동일, ordered member set 동일, old rep도 current member, representative만 변경 |
| R2 `MEMBER_DISAPPEAR_REP_CHANGE` | old representative가 current member에서 사라짐 |
| R3 `MEMBER_ADD_REP_CHANGE` | current member가 추가된 scan에서 representative 변경 |
| R4 `GROUP_SPLIT_REP_CHANGE` | old members가 둘 이상의 current PID로 갈라짐 |
| R5 `GROUP_MERGE_REP_CHANGE` | current members가 둘 이상의 prior PID에서 옴 |
| R6 `PUBLICATION_REENTRY_REP_CHANGE` | prior publication 부재 경계에서 representative 변경 |
| R7 `RESET_BOUNDARY` | 위 분류에 속하지 않는 state/boundary 경계 |

Publication 분류:

| Code | Mechanical definition |
|---|---|
| P0 | before/after 모두 미발행 |
| P1 | 둘 다 발행되고 public tuple 동일 |
| P2 | 둘 다 발행되며 `d/y/v/aLead` 중 하나 이상 변경 |
| P3 | public alias 변경은 별도 counter로 기록 |
| P4 | before published, after absent |
| P5 | before absent, after published |

`radarstate_effect`는 baseline의 직전 scan 대비 동시 변화라서 temporal exposure다. Candidate counterfactual과 다른 의미이며 단독으로 causal safety effect를 뜻하지 않는다.
