# Candidate protocol

All decisions use the current completed scan plus bounded per-PID publication history. Membership change, PID change, publication gap, timestamp regression, provider gap of at least 300 ms, selector reset, and segment boundary fail open to the baseline tracking representative.

| Policy | Causal rule |
|---|---|
| P0 | publish the current tracking representative |
| P1 | retain the previous current/fresh surface only when the member set is unchanged and its production continuity disadvantage is at most 0.25 |
| P2 | P1 near-tie plus no camera/OEM support superiority, a bounded 2 s prior tracking-representative observation of the challenger, and no R1-p99-normalized geometry contradiction |
| P3 | P1 near-tie plus no support superiority and at least one R1-p95 coordinate jump component |
| P4 | P1 near-tie plus explicit immediate handoff on camera/OEM positive-support superiority |
| P5 | broad reference: current-member medoid using current d/y/v geometry, then camera/OEM/age/raw tie breaks |
| P6 | narrow union of the P2 causal ping-pong gate and P3 large-jump gate, both behind the same near-tie and support gates |

The 0.25 near-tie cap and R1 p95/p99 jump values are frozen from the preceding representative-stability study. They are not tuned against future return, lead, actor, PID carry, or Claude artifacts.
