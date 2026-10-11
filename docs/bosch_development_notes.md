# Bosch MRRevo14F development decisions

These entries were kept in `AGENTS.md` on the Bosch development branch. They record user-approved Bosch provider decisions and their validation limits. They moved here so the shared upstream `AGENTS.md` is not changed by the Bosch PR.

- On 2026-10-08, the camera-still Bosch side-pass arm (cea76210) was reverted.
  A geometric entry check (468 radar-identified cut-ins, beyond the 165 camera-GT
  ones) found it delayed cut-in risk on 2 real entries (0.15 s, 0.5 s; lead_one
  unchanged). A one-scan outward camera jump armed it. Fix false cut-in
  pre-decelerations only within the Bosch provider (never the carrot planner), and
  freeze the geometric entry check into any retry's criteria. See
  docs/bosch_sidepass_camera_20261008.md.

- On 2026-10-08, the user approved making Bosch context timing deterministic, with
  a post-hoc determinism criterion recorded. set_bosch_context now treats a
  modelV2/livePose stamped after now_ns as not yet current and keeps the
  previously accepted message. Before, it invalidated the road-edge model,
  emptied path/cue, or dropped yaw, so IPC latency flickered roadside points.
  Stale (>200 ms) or missing context still invalidates. Replay: leads unchanged
  over 160 segments; extra corridor removals were all stationary roadside or
  oncoming objects. See docs/bosch_context_timing_20261008.md.

- On 2026-10-08, the user approved fixing Bosch large-vehicle splits in risk order
  under world-robust criteria frozen before results. Stage 1 changes only
  camera-extended maturity: range bound 1.0+2.5dt^2, lateral bound adds 0.005*d_rel,
  and an already-mature group stays collapsed through a coast only if every member
  is measured this scan and the representative is the nearest member. A coast never
  matures a group; association/strict-edge/truck-P2 gates are unchanged. On the
  2f1/2f2/2f4 drives, split scans fell 598->478. Camera two-vehicle pairs saw no new
  collapse, and leads were never farther or lost. Recorded replay only. See
  docs/bosch_truck_split_maturity_20261008.md. Stage 2 raises the class-6 truck-P2
  range spread limit from 9.0 to 12.0 m (other P2 gates unchanged): split scans
  fell 478->357, all new collapses were one adjacent-lane bus, and leads were unchanged.
  Stage 3 adds a strict-pair-only footprint association. A far member is A0
  UNRESOLVED only on bearing, its near member is A0-assigned to a large camera
  object, and it lies <=12 m behind that object's rear face, laterally within
  min(1.75, w/2+0.5), at |dv|<=0.5. Exported A0 verdicts are unchanged.
  Split scans fell 357->232, with no new two-vehicle collapse and no lead change.
  Residual risk: an unseen same-speed vehicle just ahead of a short large vehicle
  can be merged (lead_one unaffected; lead_two may hide). Stage 4 lets a camera
  class-1 set mature after one stable interval, but only when its representative
  is the nearest member (a far OEM anchor still waits two). Class-6 P2 still needs
  two. Split scans fell 232->199, with no new two-vehicle collapse and no lead
  farther or lost. Stage 5 lowers truck-P2 confirmations 5->3 (other P2 gates
  unchanged): split scans fell 199->182, and the only new collapse was the
  adjacent bus.
  Stage 6 (2026-10-09) lowers the class-6 truck-P2 width floor 2.40->2.25 m (the
  5.5 m spacing floor is unchanged): over 885 replay segments split scans fell
  3,494->3,174, no new two-vehicle collapse, leads never farther or lost. Rejected:
  a 5.0 m spacing floor (merged a parked car) and a 16 m class-1 range (merged a car
  ahead of a medium truck); the camera reports width, not length.

- On 2026-10-04, the user approved default activation of Bosch mature group
  retention with the world-robust surface guard for vehicle testing, and the
  replacement of the SCC-slope aLead criterion by a known-truth braking test.
  A retained family may hide only members its published surface covers
  (nearest within 0.25 m, not faster by >0.25 m/s, |dPath| <= max(hidden, 0.5 m),
  model path required, held <= 0.5 s); otherwise the strict split stands. No PID
  history copy or threshold relaxation. Recorded/synthetic evidence only; device
  timing, closed-loop and vehicle behaviour remain unvalidated. See
  docs/bosch_group_retention_20261003.md.

