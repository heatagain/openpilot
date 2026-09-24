# Final validation

Date: 2026-09-24 (Asia/Seoul)

## Passed

- Baseline current-source replay: 34 routes, 847 segments, 504,673 completed scans.
- Candidate E/E1/E2/E3/E4/E5 full-corpus replay: 847 segments and 504,673 completed scans per policy.
- Candidate E legacy reproduction: 50,553 representative changes and 80,770 legacy publication-changed scans.
- Artifact fail-closed validation: PASS, 16/16 required tables present, no corpus/prefix/synthetic/Route2bc baseline failure.
- Prefix invariance: 68/68 exact comparisons PASS.
- Synthetic selector harness: 13 cases × 7 policies = 231 rows, stale representative 0.
- Route2bc S20→S21 warm control: baseline raw822/PID1000845/rep822 for 11 scans; E-E4 target regression 0; E5 exact PID/member regression correctly detected 11/11.
- Route280 S15 cut-in mechanical control: first-publication delay 0 ms for every policy.
- Known P0 mechanical duration control: positive duration regression 0 for every policy.
- Production radar regression: 241 passed, 6 deselected, 3 environment/config warnings in 3.30 s.
- Python compileall for all study scripts: PASS.
- Ruff for all study scripts: PASS.
- Primary checkout remained clean; normalized provider source hash matched the P1 worktree.

## Deliberate exclusions / not run

- `test_radar.py` exclusions: existing Windows fixture/API-skew cases `group3`, `optional_upper_track_bank_and_32_slot_compatibility`, `tentative_overpass_reflection_is_not_confirmed`, and `confirmed_vehicle_remains_eligible`; no test was weakened or edited.
- Native Linux IPC / MPC / acados requested-acceleration replay: NOT RUN.
- A1M CPU: NOT MEASURED.
- Real-car validation/application: NOT PERFORMED.
- Actor/raw-association attribution: OUT OF SCOPE and NOT VERIFIED.

## Decision gate

Study mechanics are validated, but every representative policy has nonzero full-corpus exact PID/member mismatch. No production candidate exists; decision is `REPRESENTATIVE CANDIDATES NO-GO`.
