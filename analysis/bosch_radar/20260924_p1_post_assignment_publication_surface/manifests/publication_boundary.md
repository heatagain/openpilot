# Exact publication boundary

Current source order in `opendbc_repo/opendbc/car/hyundai/radar_interface.py` is:

1. `publication_aliases.update(...)`
2. `BoschLeadAccelerationEstimator.update(...)`
3. `BoschRadarProvider.publication_view(...)`
4. `bosch_append_points(...)`

The analysis-only insertion point is between steps 3 and 4 (current source lines 6243-6244 at study start). At this point:

- physical PID, PID ownership, members and tracking representative are final for the completed scan;
- camera/OEM association and qualification have already run;
- publication eligibility and publication set are final;
- public aliases are allocated;
- PID-owned aLead state has already updated from the baseline tracking input.

The selector may read these values and current member observations. It may only replace the cloned RadarPoint's `dRel/yRel/vRel`. It cannot write any provider object or any state read by a later provider scan.
