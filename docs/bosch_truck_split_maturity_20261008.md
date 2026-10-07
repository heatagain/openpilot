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

## Limits
- This is recorded-input replay only. Device timing, closed-loop behaviour and vehicle display are unvalidated.
- 478 split scans remain. They belong to the truck-P2 and association-window stages, which have their own frozen criteria.
