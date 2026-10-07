# Bosch large-vehicle split: maturity continuity and coast collapse (2026-10-08)

## Problem
On the 2026-10-07 drives (routes 2f1/2f2/2f4, build 1e012875, 1.88 h longitudinal-active), camera-confirmed large vehicles were published as 2–3 separate front-radar points for 579 scans. Each scan was attributed to one cause:

| Cause | Scans | Stage |
|---|---|---|
| Group formed, but maturity was still building or kept resetting | 208 | this change |
| Coast (one camera confirmation missing) publishes both members | 23 | this change |
| Class-6 truck-P2 conditions (mostly dd > 9.0 m) | 162 | later stage |
| Far member outside the camera longitudinal association window | 161 | later stage |

In `_maturity`, 190 of 812 confirmed calls reset:
- 119 failed the lateral bound. Residual p50 was 0.34 m at a median range of 66 m, against a fixed 0.26 m bound.
- 71 failed the range bound. Residual p50 was 0.67 m, against 0.525 m.

## Change (`BoschCameraExtendedGrouping`)
- **Maturity continuity bounds.**
  - Range: `1.0 + 2.5 dt²`, previously `0.5 + 2.5 dt²`.
  - Lateral: `0.0625 + 2 dt + 0.005 · d_rel`, previously without the range term.
  - The speed bound is unchanged.
- **Coast collapse.** A group may stay collapsed through a coast only if all of the following hold:
  - it was already mature on the previous scan;
  - continuity still passes within `BOSCH_CAMERA_MATURITY_HOLD_NS`;
  - every member was measured this scan;
  - the representative is the nearest member.
  
  A coast never makes a group mature, and never hides a nearer surface.
- The camera association, strict-edge and truck-P2 gates are unchanged. Only pairs the camera already ties to one large-vehicle object are affected.

## Evidence
Lockstep faithful replay of the same drives, HEAD vs candidate. Criteria were frozen before the results were seen (`analysis/20261008_truck_fix/GO_CRITERIA_B1.md`).

- **Tracker output:** identical. Scan digests matched in 160/160 segments.
- **Large-vehicle split scans:** 598 → 478 (−20 %).
- **Pairs the camera sees as two vehicles:** 196 scans, newly collapsed 0.
- **New collapses that HEAD never made in the same episode:** 6, checked by video and numbers. All six were one camera class-1 object with several returns of the same truck.
- **lead_one / lead_two / cut-in risk:** never farther, faster or lost. 2 frames were nearer (the same truck's rear surface).
- **Design change made after results:** the nearest-representative condition on coast collapse was added after the first results. It is recorded in the criteria appendix and did not change any measured result.
- **Tests:** 691 Bosch tests pass, including new tests for the range-scaled lateral bound, coast collapse onto the nearest member, and coast never maturing a group.

## Stage 2: truck-P2 range spread (12 m)
Long trucks and buses put their strongest returns 9–11 m apart. These pairs failed `BOSCH_TRUCK_P2_DD_MAX_M = 9.0` (105 of 162 unmerged class-6 scans). The limit is now 12.0 m, the same as the geometry-candidate and class-1 limit. Width (2.40 m), dy, dv, the 5 confirmations and freshness are unchanged.

Lockstep replay against stage 1, with criteria frozen first (`analysis/20261008_truck_fix/GO_CRITERIA_B2.md`):
- **Tracker output:** identical.
- **Large-vehicle split scans:** 478 → 357.
- **Pairs the camera sees as two vehicles:** newly collapsed 0.
- **New collapses:** 101 scans, all from one red bus (about 12 m long) in the adjacent lane, approaching from 51 m to 19 m. Video shows the hidden point on the same bus.
- **lead_one / lead_two:** unchanged.
- **Cut-in risk:** the same bus was dropped for 6 frames (0.3 s). Its lateral position stayed at −3.1 m and it did not cut in.
- **Tests:** 692 Bosch tests pass. The boundary test now accepts 10.75 m and 12.00 m and rejects 12.01 m.

## Stage 3: far-member footprint association
After stage 2, 153 split scans had a far member no camera object explained. In 151 of them, the far member failed only the A0 bearing gate (by 0.016–0.11 rad). The camera bearing span describes the rear face of the vehicle; the front of a long body in an adjacent lane sits farther away, so its bearing moves toward the centre.

For strict pairs only, the far member is now associated with the near member's camera object when all of the following hold:
- the near member is A0-assigned to a large object (class 1, or class 6 with width ≥ 2.40 m);
- the far member is A0 UNRESOLVED, so no camera object passed all of its gates;
- it lies behind the rear face by at most 12 m;
- it is laterally within `min(1.75, width/2 + 0.5)`;
- its speed is within 0.5 m/s of the near member.

The exported `last_associations` (used by companion deferral and other consumers) keep the A0 verdicts. For this geometry, the footprint now precedes the class-6 A0 near-miss recovery. The recovery remains for cases the footprint does not cover.

Lockstep replay against stage 2, with criteria frozen first (`analysis/20261008_truck_fix/GO_CRITERIA_B3.md`):
- **Tracker output:** identical.
- **Large-vehicle split scans:** 357 → 232.
- **Pairs the camera sees as two vehicles:** newly collapsed 0.
- **New collapses:** 15 scans in two episodes, both checked by video and numbers. One was a tank/box truck at a toll gate, the other a box truck in a tunnel. Each was one class-1 camera object, with the far point 6–7 m behind the rear face.
- **lead_one / lead_two / cut-in risk:** unchanged.
- **Tests:** 701 Bosch tests pass.

**Known residual risk:** an occluded vehicle that the camera does not see, travelling at the same speed within 12 m ahead of a short large vehicle, can be merged with it. The representative stays the nearest surface, so lead_one is unaffected. lead_two could be hidden.

Over all three stages, large-vehicle split scans on these drives fell from 598 to 232.

## Stage 4: one interval for camera class-1 sets onto their nearest member
A camera class-1 (large vehicle) verdict on both members already ties them to one body. Such a set now matures after **one** stable interval instead of two. The shortcut applies only when the representative is the nearest member, because the representative prefers an OEM-selected member: a moved OEM anchor on a farther member must still wait the full requirement, otherwise a nearer surface could be hidden. Class-6 truck-P2 sets keep two intervals, and the motion-continuity checks are unchanged.

Lockstep replay against stage 3 (`analysis/20261008_truck_fix/GO_CRITERIA_B4_B5.md`):
- **Tracker output:** identical.
- **Large-vehicle split scans:** 232 → 199.
- **Pairs the camera sees as two vehicles:** newly collapsed 0.
- **New collapses:** 11 scans. Nine were checked numerically: one wide class-1 object, or inside the stage-3 footprint. Two narrower class-1 objects (1.95 m and 2.15 m wide) were confirmed by video as single box trucks.
- **lead_one:** one frame nearer, none farther or lost.
- **Design change made after results:** the nearest-member condition was added after the first results. It did not change any measured result and is recorded in the criteria appendix.

## Stage 5: three truck-P2 confirmations
`BOSCH_TRUCK_P2_CONFIRMATIONS` goes from 5 to 3. A wide class-6 pair now opens after 0.2 s of continuous confirmation of the same camera object and truck geometry, instead of 0.4 s. Width, dd, dy, dv, freshness and the A0 recovery rule are unchanged.

Lockstep replay against stage 4:
- **Tracker output:** identical.
- **Large-vehicle split scans:** 199 → 182.
- **Pairs the camera sees as two vehicles:** newly collapsed 0.
- **New collapses:** one scan, on the red bus beside the ego vehicle (checked by video).
- **Cut-in risk:** the same bus was dropped for 3 frames; its lateral position held at −3.1 m and it did not cut in.

Over the five stages, large-vehicle split scans on these drives fell from 598 to 182.

## Limits
- This is recorded-input replay only. Device timing, closed-loop behaviour and vehicle display are unvalidated.
- 478 split scans remain. They belong to the truck-P2 and association-window stages, which have their own frozen criteria.
