# Baseline Definition

Baseline is an unmodified current-HEAD `BoschRawTrackManager` plus `BoschObjectGroupManager` replay across every
available decoded corpus segment.

A `GROUP_FRACTURE` is counted only when a previous multi-member physical object has at least two of its raw members
still observed at the next completed scan and those members occupy at least two current groups. Thus raw dropout and
raw reassignment are not silently called group fracture. A rejoin is the first later scan in the same segment where
the exact observed parent member family is again one physical group.

An oscillation family is keyed by segment and failed raw pair and requires at least two fractures. `split_rejoin` is
segment-local and never crosses a segment boundary.

For the strict fracture table, an event is publication-visible when it creates/removes a published point, changes a
public alias, changes the representative, or creates a publication gap. `internal-only` means none of those effects.
This is a mechanical visibility definition, not a planner/control safety assertion.

Counts from detailed failure-edge tables can exceed fracture counts because one fracture may expose more than one
failed complete-link edge. Zero-denominator control questions remain `UNEVALUABLE`; missing evidence is never zero.
