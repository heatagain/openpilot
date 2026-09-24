# Invariant contract

For every completed scan and every P0-P6 policy, the replay snapshots the following baseline provider components before surface selection and compares them again after surface selection and downstream replay:

- raw-track output tuple;
- group partition;
- physical PID to member mapping;
- PID member set;
- tracking representative;
- qualification output;
- publication PID set;
- public alias map;
- complete PID-owned aLead estimator signature (timestamps, reset/publication state, update counts and per-PID state).

Any nonzero mismatch is `IMPLEMENTATION INVALID` and blocks candidate evaluation. Baseline replay is additionally compared scan-by-scan with the prior current-source 847-segment cache for completed-scan timestamps, qualified objects, and production-current publication rows.

Freshness is fail-closed: the selected raw member must belong to the current object and its observation timestamp must be later than the preceding completed scan. No coasted-only, past, interpolated, centroid or synthetic point is eligible.
