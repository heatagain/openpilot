# Current representative algorithm

Source: `opendbc_repo/opendbc/car/hyundai/radar_interface.py:2506-2529`, current P1 baseline HEAD.

## Existing physical object

이전 anchor를 `dt`와 yaw로 투영한다.

`px, py = rotate(prior.d_rel + prior.v_rel * dt, prior.y_rel, -yaw_rate * dt)`

각 current member의 lexicographic cost는 다음 순서다.

1. scalar continuity cost: `abs(d-px) + 0.5*abs(y-py) + 0.5*abs(v-prior.v_rel)`
2. previous representative가 아니면 불리함
3. current vision support가 없으면 불리함
4. current OEM-selected slot이 아니면 불리함
5. age가 클수록 유리함
6. raw ID가 작을수록 유리함

중요: 2-6은 scalar continuity가 정확히 같은 경우에만 tie-break다. Camera/OEM support는 더 나쁜 continuity scalar를 역전시키는 가중치가 아니다.

## New physical object

이전 anchor가 없으면 current member `d_rel` median과의 거리, vision, OEM slot, age, raw ID 순으로 고른다.

## Hard conditions / state

- 후보는 current group의 observed members뿐이다.
- singleton은 그 한 member를 즉시 고른다.
- stale coordinate, previous-scan coordinate, synthetic coordinate를 만들지 않는다.
- group geometry는 selector 이전에 확정된다.
- representative는 다음 scan projection과 ambiguous PID assignment score에 되먹임된다. 따라서 selector를 바꾼 뒤 exact PID mapping을 별도로 검사해야 한다.

## Candidate E reproduction detail

이전 artifact의 E는 shared selector에서 prior representative가 current candidate이고 scalar continuity가 best보다 `0.25` 이내이면 prior를 유지했다. shared selector는 physical group과 provisional bundle 양쪽에서 호출되므로 E reproducer는 이 기존 범위도 그대로 보존한다. E1-E5 selective 연구는 physical-group call에만 적용한다.
