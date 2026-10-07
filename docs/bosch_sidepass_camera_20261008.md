# Bosch side-pass: camera-still arming with a camera cap (2026-10-08)

## Problem
The 2026-10-07 drives (routes 2f1/2f2/2f4, 1.88 h longitudinal-active) applied 50 cut-in pre-decelerations. Two were real cut-ins, confirmed on video. About 45 of the others came from adjacent vehicles being overtaken:
- As ego closed on the vehicle, its Bosch return moved from the rear face to the near corner, then onto the side.
- The published lateral therefore moved 0.3–2.7 m toward the path, while the same vehicle's camera object moved only 0–0.2 m.
- All 50 were singletons.
- The existing side-pass estimator arms only on range/vRel slide evidence, so it almost never caught them.

## Change (`_BoschSidePassLateralEstimator`)
**New arm reason `CAMERA`.** It applies only to a singleton that meets all of the following:
- closing at vRel ≤ −0.5 m/s, a same-direction mover, d ≤ 45 m, inside the existing zone
- continuously A0-assigned to a camera object for 0.4–0.8 s
- over that window, raw |y| moved inward ≥ 0.3 m while camera |lat| moved inward ≤ 0.1 m

**While armed:**
- The published |y| follows the anchor (the raw value at the window start) minus the camera's own inward motion.
- The published |y| is never outward of the camera's latest |lat| (camera cap).
- The published |y| is never nearer than the raw return.
- The correction is held at 2.0 m, not released.

**Release:**
- Immediate on corridor entry, OEM selection, or camera inward motion ≥ 0.3 m since arm (`CAMERA_ENTRY`).
- On camera loss or no longer closing, `EVIDENCE_END` releases after 3 scans and realigns at 0.1 m/s.

**Provider wiring:** the provider passes the camera-extended scan's fresh camera objects. The `SLIDE` arm is unchanged, and the change is publication-only.

## Why the camera cap
A first candidate without the cap delayed cut-in risk on 5 of 165 identified real lane entries, by 0.1–2.7 s. In two of them the camera lagged the radar by 0.5–1.2 s at the start of a real lane change. In all five the held value sat outward of the camera's own lateral. Capping at the camera lateral removes those delays. The cost: returns that slide from outward of the camera down to it are no longer corrected, and that case cannot be told apart from a real lane change.

## Validation (recorded-input lockstep replay against `fa6c40e8`, 752 segments)
- **Real lane entries** (294 camera-labelled; 165 identified): cut-in risk, lead_two and lead_one were never later and never lost. One risk was earlier, and one appeared only with this change.
- **Video-confirmed cut-ins** (2f4 s40, 2f2 s16): unchanged.
- **False radar cut-in risk episodes:**

  | Data | Before | After |
  |---|---|---|
  | 2f1/2f2/2f4 drives | 57 | 43 |
  | side-pass corpus | 278 | 203 |

  Real-entry episodes were unchanged.
- **Leads:** lead_one was never farther, faster or lost. lead_two was lost on 12 frames, all one adjacent-lane sedan that stayed in its lane on video (a false lead_two).
- **Integrity:** scan digests were identical on 752/752 segments.
- **CPU:** paired same-input runs over 4 rounds (2f2 s10–19, 60,003 calls) measured a median of +0.7% cycles. Per-round values ranged from −2.8% to +6.6%, which is within run-to-run noise.
- **Limits:**
  - The candidate was shaped after seeing the uncapped candidate's failures on the same data, so these results are in-sample.
  - Out-of-sample confirmation is the next real drive.
  - Closed-loop behaviour is unvalidated.

## Reverted (2026-10-08)
After release, a geometric entry check without camera GT found 2 more real cut-ins where this change delayed cut-in risk. It covered 468 radar-identified entries; the camera GT had 165. lead_one timing was unchanged in both.
- `00000273` s35: 0.5 s later. A one-scan outward camera jump (3.60→3.80 m) was read as "camera still". The arm held the point 0.42 m outward for one scan, and that broke DPath inward progress.
- `0000025c` s8: 0.15 s later. A long-held arm released 0.3 m after the camera started moving.

The change was reverted to keep the zero-delay rule for real entries. A retry must at least:
- not arm on a camera whose lateral jumps in either direction;
- include the geometric entry check in its frozen criteria.
