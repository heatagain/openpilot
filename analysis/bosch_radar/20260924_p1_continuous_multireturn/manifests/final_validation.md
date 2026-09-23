# Final Validation

## PASS

- `python -m compileall -q .../scripts`: PASS
- analysis-script Ruff (`TID251` local `common` import and intentional `ISC002` source-template concatenation excluded): PASS
- `scripts/validate_study.py`: PASS; source SHA, corpus counts, Route269 11/11, 255-scan raw482 continuity,
  candidate/control counts, publication counts, synthetic matrix, lifecycle, prefix, and report headings verified
- synthetic A-M: 13 cases × 6 variants, 774 scan rows; baseline G = 10 split / 10 rejoin
- prefix invariance: 25/25 PASS
- frozen controls: 41/41 SAME and 37/37 simultaneous DIFFERENT evaluable; zero recorded regressions for A-E
- focused Windows-compatible `test_radar.py`: 241 passed, 6 deselected, 3 configuration warnings
- production source SHA256 before/after research: `fb756fb400b8408840c68b5ee32d6cae795ec939c9cdea634fb5836a235851a9`

## Known environment failures

The complete `test_radar.py` file was executed: 241 passed and 6 failed. All six failures stop before the relevant test
logic because this checkout lacks generated DBC files:

- `hyundai_kia_mando_front_radar_generated.dbc`: one test
- `hyundai_canfd_radar_generated.dbc`: five tests

The focused pass deselects exactly those six. This is an environment/fixture limit, not a candidate result; production
and test sources were not changed to hide it.

## Not executed / not verified

- A1M CPU: **NOT MEASURED**
- real-car application: **NOT PERFORMED**
- native planner/MPC/control replay: NOT EXECUTED
- downstream safety effect: NOT VERIFIED
- moving sequential DIFFERENT actor safety: NOT USED / NOT VERIFIED
- NAS deployment or on-device runtime: NOT PERFORMED

## Mutation check

- production code changed: NO
- tests changed: NO
- research artifact changed: YES
- new actor GT created: NO
- moving actor identity evidence used: NO
- parallel Claude artifact modified: NO
