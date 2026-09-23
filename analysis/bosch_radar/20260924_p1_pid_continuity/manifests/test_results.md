# Test and replay results

## PASS

- `scripts/build_evidence.py`: rebuilt the compact evidence bundle from read-only source artifacts.
- `scripts/validate_study.py`: `PASS: provider source, 11/11 owner cycles, GT denominators, Route2bc identity, four candidates, 12 requested cases`.
- Current P1 worktree `test_radar.py`, Windows Params shim, excluding Group3 and three tests whose generated DBC files are absent from this worktree: **241 passed, 6 deselected, 3 warnings in 0.96s**.
- External group-owner validator: **PASS**, current source hash/847 segments/504,673 scans/raw482 11 exact splits/controls/parity/prefix/links.
- External owner-GT validator: **PASS**, 28 nearby candidates remained ambiguous and no label was promoted without unique endpoint evidence.
- Existing baseline prefix evidence reused read-only: Route269 **5/5**; side-pass study Route2bc and controls **25/25**.

## Environment-limited / not passed

- Unfiltered `test_radar.py -k 'not group3'`: **241 passed, 3 failed, 3 deselected**. The three failures are `FileNotFoundError` for the untracked/generated `hyundai_kia_mando_front_radar_generated.dbc` and `hyundai_canfd_radar_generated.dbc`, not Bosch P1 assertions. No file was copied from the primary checkout.
- Group3 tests: excluded because the known local fixture/API skew is outside this P1 scope.
- The moving-identity study validator's repository subcheck returned exit 129 when Git crashed in the Codex environment. Its data checks still showed moving-to-moving DIFFERENT=0 and SHADOW gate=false. This is recorded as infrastructure failure, not a P1 conclusion.
- Native `radarState`/planner/MPC/control replay: **NOT RUN**.
- Device/A1M CPU, real-car runtime, NAS deployment and real-car validation: **NOT MEASURED / NOT PERFORMED**.

## Exact green pytest command

```powershell
& C:\CarrotRadarResearch\analysis\.venv\Scripts\python.exe `
  analysis\bosch_radar\20260924_p1_pid_continuity\scripts\run_pytest_with_params_shim.py `
  C:\CarrotRadarResearch\openpilot-p1-pid-continuity\opendbc_repo\opendbc\car\hyundai\tests\test_radar.py `
  -k 'not group3 and not optional_upper_track_bank_and_32_slot_compatibility and not tentative_overpass_reflection_is_not_confirmed and not confirmed_vehicle_remains_eligible' -q
```
