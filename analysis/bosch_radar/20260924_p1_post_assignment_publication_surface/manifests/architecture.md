# Architecture

The replay retains the current provider pipeline unchanged:

`raw detection -> raw association -> grouping -> physical PID -> tracking representative -> association -> qualification -> alias allocation -> PID-owned aLead update -> publication eligibility -> publication surface clone -> RadarPoint -> DPathRadarController -> radarState -> planner input`

P0-P6 are not provider variants. One baseline provider is run once per input segment. Each policy receives the same already-qualified, already-filtered tuple, the same public alias, and the same already-computed PID-owned aLead. It returns one current observed member's whole `dRel/yRel/vRel` tuple for an analysis-only RadarPoint clone.

No policy result is assigned to `BoschPhysicalObject`, the raw/group manager, qualification state, alias allocator, or acceleration estimator. The next provider scan therefore sees only baseline state.

The current production source also has an OEM-nearer publication surface. The requested research reference P0 is explicitly the tracking representative, so the study records `SOURCE_CURRENT` versus P0 differences separately rather than silently treating them as identical.
