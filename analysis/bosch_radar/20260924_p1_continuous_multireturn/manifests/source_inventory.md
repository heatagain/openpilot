# Source Inventory

## Authoritative code

- `opendbc_repo/opendbc/car/hyundai/radar_interface.py`: current P1 worktree Bosch implementation; executed directly by baseline replay.
- `opendbc_repo/opendbc/car/hyundai/tests/test_radar.py`: read-only Bosch test inventory and focused regression target.

## Read-only replay evidence

- established 847-segment decoded corpus under `20260919_MRRevo14F_parent_sibling_publication_shadow/scratch/corpus`
- established current-provider replay loader under `20260921_MRRevo14F_lead_ahead_identity_reacquisition/scripts`
- completed P1 report and Route2bc control trace under `analysis/bosch_radar/20260924_p1_pid_continuity`
- completed group-owner mechanics artifact under `analysis/20260923_MRRevo14F_group_owner_oscillation`
- frozen `SAME_PHYSICAL_OBJECT` controls from `typeA_identity_gt_v2_frozen.csv`
- frozen simultaneous `DIFFERENT_PHYSICAL_OBJECTS` controls from the completed group-owner study

## Explicitly excluded

- parallel moving-identity study root: not read, written, watched, or used as a dependency
- qcamera/video: not opened
- new actor identity adjudication: not performed
- raw rlog/video and large replay cache: not copied into this artifact

The ignored local full-row tables `group_lifetime.csv`, `mechanical_raw_events.csv`, `pid_births.csv`, and
`representative_changes.csv` are deterministic replay products. Their aggregate evidence and the smaller event tables
are tracked; they can be regenerated with the checked-in scripts.
