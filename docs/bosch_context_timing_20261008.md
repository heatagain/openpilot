# Bosch context timing: future-stamped modelV2/livePose (2026-10-08)

## Problem
`RadarInterface.set_bosch_context` accepted a modelV2 only when `0 <= now_ns - model_ns <= 200 ms`, where `now_ns` is the radar batch receive time. A model stamped even a few ms after `now_ns` did more than get skipped:
- it **invalidated** the road-edge model;
- it emptied the path and vision cue for that call.

A future-stamped livePose likewise set yaw to None.

On the device, whether radarcan has already received a just-published model depends on IPC scheduling. The road-edge filter therefore switched off intermittently, and stationary roadside returns flickered into publication. In replay the effect explained the R0.2 fidelity failure on route 2f4. Moving the context cutoff 3 ms earlier gave:
- publication exact match: 83.1 % → 99.5 %;
- replay-only points: 6.6 % → 0.006 %;
- road-edge removals: 25,397 → 33,486.

## Change
A message stamped after `now_ns` is treated as not yet current. The call keeps the previously accepted message, and the first call that reaches the newer stamp takes it. The 200 ms freshness rule still applies to the accepted message, so stale or missing context still invalidates as before. Every consumer selects samples at or before its own timestamp:
- road-edge scan and surface views;
- B5;
- mirror.

## Evidence
Same-input replay, HEAD vs FIX (`analysis/20261008_ctx_determinism/GO_CRITERIA.md`):

| Check | Result |
|---|---|
| Determinism, 0 vs −3 ms | 99.96 % (HEAD 83.0 %) |
| Determinism, 0 vs −10 ms | 99.27 % (HEAD 82.0 %) |
| lead_one / lead_two / cut-in risk, 160 segments | 0 differences |
| Corridor points FIX removes (dRel < 100 m, \|y\| ≤ 2 m) | 6 scenes, video-checked: curve-outside guardrails, flowerbeds and roadside fixtures, plus one oncoming vehicle beyond the median. No real vehicle lost |
| Tests | 704 Bosch tests pass, including future model/pose, stale/missing model and pose handover |

The −10 ms determinism pair missed the pre-registered 99.9 % threshold. A 10 ms delay legitimately selects the previous model (50 ms older). The user approved a post-hoc criterion change ("≤ 3 ms pairs ≥ 99.9 %, 10 ms pair reduced vs HEAD"); it is recorded in the criteria appendix.

## Limits
- The evaluation ran with radar_bosch at 1e012875. The later truck-split commits change camera-extended grouping only.
- Device delivery timing and closed-loop behaviour are unvalidated.
