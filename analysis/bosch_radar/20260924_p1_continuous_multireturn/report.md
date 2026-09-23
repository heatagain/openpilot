# MRRevo14F P1 Continuous Multi-Return Mechanics

## Summary

**CONFIRMED FACT (current-HEAD offline replay):** P1 continuity loss includes a large, distinct
`CONTINUOUS_GROUP_TOPOLOGY_OSCILLATION` family. Across 847 segments / 504,673 completed scans / 34 routes,
the strict detector found 21,012 `GROUP_FRACTURE` events, 7,861 exact-family split→rejoin events, and 2,555
repeated-pair oscillation families. These are mechanical events, not actor GT.

Route269 S7 raw436/raw482 reproduced exactly: raw482 existed continuously for 255 scans; 11 fractures and 11 rejoins
all failed the `436↔482` edge; no raw dropout or slot-churn explanation is needed. The parent PID followed the
raw436/representative child, the raw482 singleton received a new PID, rejoin removed the singleton's ownership in the
same scan, and the next fracture therefore created another fresh PID.

Candidates A-D reduced Route269 fracture but changed grouping widely and created new merge objects. Candidate E
changed no grouping and reduced representative changes by 7,952, but changed publication coordinates on 80,770
scans. Therefore the production decision is **MECHANISM CONFIRMED / CANDIDATES NO-GO**. E remains an
analysis-only representative-stability lead, not `READY`.

No moving-actor identity evidence was used, no new actor GT was created, no video was opened, and the parallel Claude
artifact was not read or modified. Real-car application was **NOT PERFORMED**.

## Repository State

- repo root: `C:\CarrotRadarResearch\openpilot-p1-pid-continuity`
- branch: `heatagain/bosch-p1-pid-continuity`
- baseline HEAD: `b9039544f1dba61a34c5dc059e08316357256932`
- baseline production SHA256: `fb756fb400b8408840c68b5ee32d6cae795ec939c9cdea634fb5836a235851a9`
- primary checkout: untouched
- final HEAD and remote verification: recorded in the final handoff after commit/push

See `manifests/repo_state.md` and `manifests/source_inventory.md`.

## Scope Boundary vs Claude Moving-Identity Study

This study begins with continuous raw surfaces and follows grouping mechanics. It does not decide which physical actor
a surface belongs to. It did not mine moving↔moving sequential `DIFFERENT`, cut-in, cut-out, overtake, merge, endpoint
attribution, bounded lineage, owner-contradiction veto, or actor-aware stitching. The parallel study root was excluded
from every script and was neither read nor written.

Existing frozen simultaneous `DIFFERENT` and frozen `SAME` records were used only as fixed regression controls.
Simultaneous `DIFFERENT` is not treated as a substitute for moving sequential `DIFFERENT`.

## Current Bosch Grouping Architecture

The current path is:

`raw CAN → decoded slot detections → BoschRawTrackManager → pair evidence → complete-link clusters → physical PID
assignment → representative → camera/OEM/qualification → publication view → public alias`.

The identities are intentionally separate:

| Layer | Key/owner | Birth and update | Expiry/reset | Priority/history |
|---|---|---|---|---|
| CAN slot | decoded slot index | every completed scan | scan-local | sensor address only |
| raw track | monotonic `raw_track_id`; manager-owned | unmatched detection births; matched observation increments `age_scans` | 0.3 s coast then delete; manager/provider reset | global one-to-one distance/speed/bearing gated cost; same-slot is weak bonus |
| pair evidence | sorted `(raw_a, raw_b)` | current geometry appends `(time, |Δd|, |Δy|)` | 0.8 s window; >0.16 s pair gap clears; hard geometry rejection removes | maturity ≥3 observations and ≥0.18 s; growth from recent minimum |
| group | current raw-member set | deterministic complete-link merge | rebuilt each scan | existing common owner first, then maximum cross-edge normalized cost, then indices |
| physical object | monotonic PID; group manager-owned | global overlap assignment or new PID | 0.3 s state/member coast; stolen membership removed same scan | `10*overlap + 3*prior-rep retention + age tie + PID tie` |
| representative | a whole raw member | selected after PID assignment | changes with selected member/group | projected continuity, prior representative, vision, OEM slot, age, raw ID |
| public alias | physical PID → alias 32..95 | allocated only for published PIDs | dead PID or >0.5 s unpublished grace | FIFO free/released alias; tracking never reads alias |

Camera/OEM evidence and qualification run after the physical object exists; publication-only filters do not own raw or
physical identity. A numeric equality between slot, raw ID, PID, representative ID, and alias has no semantic meaning.

## Baseline Corpus

The available corpus was re-inventoried and fully replayed: 847 segments, 504,673 scans, 34 routes. Baseline used the
unmodified current P1 source, not a reconstructed copy. The full mechanical bundle is a local ignored cache; the
checked-in scripts regenerate it. Smaller event tables and aggregate manifests retain the audited evidence.

The strict definition requires a prior multi-member group and at least two still-observed members landing in different
current groups. This avoids counting raw dropout as fracture. One fracture may contain multiple failed complete-link
edges, so edge-distribution counts can be larger than 21,012.

## Mechanical Event Taxonomy

The study keeps `RAW_DROPOUT`, `RAW_REASSIGNMENT`, `GROUP_FRACTURE`, `REP_CHANGE`, `PUBLICATION_GAP`, and
`ALIAS_CHANGE` separate. `GROUP_FRACTURE` is not called reacquisition. Exact definitions and cause precedence are in
`manifests/event_taxonomy.md`.

## Route269 Exact Root Cause

Current HEAD reproduced the authoritative sequence:

- raw482 observed on every scan 345..599: 255/255; raw dropout **NO**
- raw436 observed for 274 scans; both raw436/raw482 present together on 246 scans
- 11 split / 11 exact-pair rejoin; median split interval 1.939583 s; max 4.290204 s
- median rejoin latency 0.500352 s
- failed edge: `436↔482` in all 11 events
- cause: distance diameter 6, lateral diameter 1, lateral growth 4
- excess: distance `3.25 - 3.0 = +0.25 m` ×6; lateral diameter `1.53125 - 1.5 = +0.03125 m` ×1;
  lateral growth `+0.03125 m` ×2 and `+0.1875 m` ×2
- raw436/representative child retained the parent PID in all 11; raw482 singleton birthed a new PID in all 11
- all 11 were publication-visible by the study definition; none created a publication gap

The trace records every scan's raw coordinates, group/PID, representative, pair cause/margin, owner scores, publication,
and alias in `traces/route269/route269_s7_raw436_raw482.csv`. Historical numeric PIDs were not mapped onto current PIDs.

## Route2bc Stable Truck Control

The existing Route2bc S21 20.865275..21.865343 s control was checked against the same current source hash. Across the
11-scan window, raw822, PID1000845, member set `{822}`, and representative822 remained stable; the public alias was 76
when published. Thus a user-identified truck case is not mechanically forced to multi-return PID instability. This
control says nothing about the earlier P0 braking cause.

## Complete-Link Failure Distribution

Event-primary causes over 21,012 fractures were:

| Cause | Fractures |
|---|---:|
| `DISTANCE_DIAMETER` | 9,114 |
| `DISTANCE_GROWTH` | 2,736 |
| `LATERAL_DIAMETER` | 4,461 |
| `LATERAL_GROWTH` | 3,993 |
| `VELOCITY_DIAMETER` | 246 |
| `OTHER_STATIONARY_MOVING` | 433 |
| `OTHER_IMMATURE_PAIR` | 14 |
| `OTHER_COMPLETE_LINK_ORDER` | 15 |

The active strict thresholds are `|Δd| ≤ 3.0 m`, `|Δy| ≤ 1.5 m`, `|Δv| ≤ 1.5 m/s`, longitudinal growth ≤1.0 m,
lateral growth ≤0.75 m, pair gap ≤0.16 s, evidence window 0.8 s, and maturity ≥3 samples spanning ≥0.18 s.

Complete-link requires every cross-edge between two candidate clusters. Therefore A-B PASS and B-C PASS are
insufficient when A-C FAIL. The merge loop first prefers clusters sharing an old PID owner, then the smallest maximum
cross-edge normalized cost. One marginal outer edge can prevent a 3-member large-object cluster even while the two
adjacent edges pass; it cannot turn into unrestricted single-link chaining.

## Oscillation Families

There were 2,555 segment/pair families with at least two fractures. Of all strict fractures, 7,861 regained the exact
parent member family later in the same segment. Route269 `436|482` had 11/11 and is a high-count, quantized-boundary
positive case, but it is not a unique mechanism.

The family table records split/rejoin count, median/max interval, rejoin latency, fresh PID births, representative
retention, cause histogram, and publication visibility.

## Threshold Excess Distribution

Failure-edge excess distributions were:

| Cause | n | p50 | p90 | p95 | p99 | max |
|---|---:|---:|---:|---:|---:|---:|
| distance diameter (m) | 9,408 | 0.25 | 1.75 | 2.25 | 3.00 | 5.75 |
| distance growth (m) | 2,985 | 0.25 | 1.00 | 1.25 | 1.75 | 2.00 |
| lateral diameter (m) | 4,683 | 0.125 | 0.84375 | 1.28125 | 2.34375 | 6.59375 |
| lateral growth (m) | 4,132 | 0.09375 | 0.34375 | 0.4375 | 0.625 | 0.75 |
| velocity diameter (m/s) | 259 | 0.25 | 0.50 | 0.75 | 1.25 | 1.75 |

Small quantized excess is common but not universal. Route269's exact `0.25 m` and `0.03125 m` steps support a
quantization-boundary explanation for that family, not a global epsilon rule for the corpus.

## PID Ownership Mechanics

At fracture, each child scores every old PID it overlaps:

`10*member_overlap + 3*representative_retention + min(age_scans,1000)*1e-5 + 1/(pid+1)`.

The global maximum assignment prevents two children from owning the same old PID. Across the 21,012 fracture rows,
the parent PID went to the representative child 18,392 times and to the member-majority child 19,064 times. Majority
and representative preference conflicted in 1,840 events, showing that neither rule alone describes ownership.

There were 16,612 fresh births attributable to fracture children. Other birth categories were 412,498 ordinary raw
or group births and 12,593 segment cold starts; these categories must not be merged into the fracture count.

## Representative Stability

There were 58,505 actual representative changes in 184,486 membership/representative transition rows. Categories:

- stable group + rep change: 26,574
- split + rep retained: 48,567
- split + rep changed: 22,441
- merge + rep changed: 5,576
- other merge/member changes: 81,328

Absolute jump distribution over representative changes:

| Coordinate | p50 | p95 | p99 | max |
|---|---:|---:|---:|---:|
| `dRel` (m) | 1.75 | 4.50 | 5.75 | 9.25 |
| `yRel` (m) | 0.15625 | 1.09375 | 1.65625 | 5.625 |
| `vRel` (m/s) | 0.0 | 0.5 | 1.0 | 2.75 |

58,496/58,505 changes were published immediately before and after. Representative churn is therefore a separate and
often externally visible phenomenon, but it does not explain Route269's repeated child PID births.

## Public Alias / Publication Impact

Among the 21,012 strict fractures, 21,012 were externally visible and 0 were internal-only under the stated mechanical
definition. There were 21,010 new published child points, 2,730 representative changes, one alias change, and zero
publication gaps. These fields overlap and must not be summed.

This result does not mean every fracture changes planner behavior. It says the emitted RadarPoint set/coordinate/alias
changed mechanically. Planner/control and real-car effects were not tested.

## Large Multi-Return Mechanical Signature

The corpus contained 89,921 multi-member group episodes: 82,124 two-member and 7,797 three-or-more-member episodes
(maximum 7). 15,326 episodes used more than one representative over their lifetime.

Sensor-space distributions were: longitudinal extent p50 2.0 m, p95/p99/max 3.0 m; lateral extent p50 0.5 m,
p95 1.40625 m, p99/max 1.5 m; `vRel` spread p50 0.25 m/s, p95 0.75, p99 1.25, max 1.5. Episode duration was
p50 0.300386 s, p95 2.699663 s, p99 5.300038 s, max 59.719989 s.

The 500 mechanically stable controls were much longer-lived: duration p50 8.904407 s, p95 42.046324 s, max
59.719989 s; 37 had three members and one had four. Large/multi-return structure can therefore remain stable inside
the current strict geometry.

The 41 frozen `SAME` controls were all evaluable, but their exact anchor pairs were already separate groups in current
baseline; none was a grouped multi-return episode at that anchor. They are useful as “do not add another fracture”
controls, not as a denominator for a confirmed large-object signature. The geometry above remains mechanical, not new
actor attribution.

## Synthetic Harness

`scripts/synthetic_harness.py` executes the actual current grouping manager through deterministic A-M sequences:
stable pair; just-below/one-step-above distance; lateral diameter/growth; one-scan and 10× oscillation; 3-member
A-B/B-C pass A-C fail; one-edge oscillation; representative disappearance; third-return approach; close independent
groups; elongated truck-like group plus adjacent point.

Baseline plus A-E produced 774 rows (13 cases × 6 variants). Timestamp regression was rejected without mutation;
an input gap beyond 0.3 s cleared candidate pair memory; a fresh provider began with empty state. The harness imports
production classes and only compiles candidate-specific method substitutions, avoiding a hand-written second grouping
implementation.

## Candidate A

Mature-pair hysteresis preserved a previously strict mature edge through a small bounded excursion when the last strict
observation was within 300 ms. Route269: 3 prevented, 8 delayed. Corpus: 10,050 suppressed fractures, 18,577 changed
grouping scans, 21,430 new merge objects, 1,212 persistent new merge episodes, and 320,744 publication-changed scans.
**NO-GO**: broad semantic change and false-retention risk outweigh the target improvement.

## Candidate B

N-scan confirmation held only the first small exceed and fractured on the second; large exceed fractured immediately.
Route269: 2 prevented, 9 delayed. Corpus: 10,108 suppressed fractures, 14,913 changed grouping scans, 17,121 new merge
objects, 633 persistent episodes, 310,280 publication-changed scans. **NO-GO**: confirmation delay is causal/bounded
but still far too broad.

## Candidate C

Quantization-aware treatment retained only a prior edge and only within one decoded distance/lateral LSB, with a 160 ms
strict recency bound. Route269: 1 prevented, 8 delayed, 2 unchanged. Corpus: 8,047 suppressed fractures, 11,400 changed
grouping scans, 12,800 new merge objects, 436 persistent episodes, 289,510 publication-changed scans. **NO-GO**:
resolution awareness does not establish actor sameness.

## Candidate D

Stable-core geometry applied only to a prior 3+ member group with exactly one marginal failed edge and every other edge
strict. It never converted to single-link. Route269's two-member pair was unchanged 11/11. Corpus still changed 3,881
grouping scans, created 4,518 new merge objects and 356 persistent episodes, and changed publication on 107,055 scans.
**NO-GO**: narrower, but no target benefit and still nontrivial merge risk.

## Candidate E

Representative-independent continuity retained the prior representative only while it remained a member and its
continuity cost was within 0.25 of the current best. Grouping and PID assignment were byte-path unchanged. It reduced
representative changes from 58,505 to 50,553, with zero grouping/member/new-merge change and unchanged 441,484 PID
births. It nevertheless changed publication coordinates on 80,770 scans. **RESEARCH-READY ONLY**, not
`REPRESENTATIVE-ONLY IMPROVEMENT READY`; actor/downstream safety and a more selective gate are still missing.

## Corpus Comparison

| Candidate | Suppressed fractures | Group-changed scans | New merge objects | Persistent new merges | New split objects | Publication-changed scans |
|---|---:|---:|---:|---:|---:|---:|
| A | 10,050 | 18,577 | 21,430 | 1,212 | 11,613 | 320,744 |
| B | 10,108 | 14,913 | 17,121 | 633 | 8,920 | 310,280 |
| C | 8,047 | 11,400 | 12,800 | 436 | 6,296 | 289,510 |
| D | 1,415 | 3,881 | 4,518 | 356 | 2,840 | 107,055 |
| E | 0 | 0 | 0 | 0 | 0 | 80,770 |

PID-birth reduction alone was not ranked first. Confirmed-control stability, independent-group preservation, new merge
risk, publication continuity, representative jump, PID births, and state/CPU were considered in that order.

## Existing SAME Controls

All 41 frozen `SAME_PHYSICAL_OBJECT` controls were evaluable for A-E and showed zero additional-fragment regression.
However, baseline already kept the exact endpoint pair separate at all 41 selected anchors, so this result is a weak
non-regression result rather than evidence that A-D safely join same actors. Route269 raw436/raw482 supplies the exact
positive mechanical case; no new SAME label was produced.

## Existing DIFFERENT Controls

All 37 frozen simultaneous `DIFFERENT_PHYSICAL_OBJECTS` controls were evaluable and A-E produced zero false merge at
the exact control anchors. This result is necessary but insufficient. It is explicitly not used as moving sequential
`DIFFERENT` evidence and does not authorize A-D.

## Prefix Invariance

A-E passed 25/25 exact prefix comparisons at 25%, 50%, 75%, Route269 event cutoff (scan 407), and 100%. For each
candidate, replaying a truncated prefix produced identical decisions and state through the cutoff. No future lifetime,
future merge, future disappearance, or future actor GT was used.

## State Lifecycle

A-D pair state is born only for a currently live/prior compatible pair, updates on current scans, is pruned to live or
recent evidence, and is cleared by the 0.3 s input-gap reset. Timestamp regression raises before mutation. Provider or
segment reset constructs a new empty manager. Raw expiry/PID expiry removes keys from the live set. Peak A-D candidate
state was 86 entries; E added zero state.

## CPU / Complexity

Alternating-order x86 benchmarking used 8 segments × 3 repetitions, 24 segment runs and 144,393 provider updates per
variant. Values are wall-clock microseconds per update and include Windows scheduling outliers:

| Variant | mean | p50 | p95 | p99 | max | wall s |
|---|---:|---:|---:|---:|---:|---:|
| baseline | 145.262 | 21.7 | 762.7 | 1,564.7 | 210,915.9 | 66.635 |
| A | 152.094 | 22.0 | 784.04 | 1,642.208 | 238,647.8 | 66.538 |
| B | 149.732 | 21.7 | 786.0 | 1,604.38 | 207,962.7 | 66.685 |
| C | 150.277 | 21.9 | 795.34 | 1,609.016 | 192,055.9 | 67.453 |
| D | 146.189 | 21.6 | 767.14 | 1,558.008 | 217,138.6 | 65.510 |
| E | 145.878 | 21.6 | 753.2 | 1,601.804 | 230,887.1 | 65.263 |

No noise-level speedup claim is made. A-D remain bounded O(number of candidate pairs) state layered on the existing
O(n²) pair geometry with n≤32; E is O(group members) and stateless. **A1M CPU = NOT MEASURED**.

## Actor-Identity Dependency

A-D are mechanically understandable and causal, and they pass the available frozen anchors, but production enablement
requires moving sequential `DIFFERENT` actor evidence. Candidate E requires actor-aware and downstream publication
validation because it changes which physical surface supplies coordinates. This report deliberately leaves that as
`ACTOR_IDENTITY_DEPENDENCY`; it did not wait for or consume the parallel study.

## Production Decision

No production behavior or tests were changed. Candidate substitutions exist only in analysis scripts. The production
gate fails on actor-level evidence and, for A-D, on broad new merge/persistent-merge counts. E fails the external
publication-validation gate. Failed candidate production code was not left in the worktree.

## Git / Commit / Push

Only this research artifact is intended for staging. `scratch`, raw logs, video, local replay bundles, `__pycache__`,
and four large deterministic full-row tables are ignored. Commit hash, message, pushed branch, and verified remote SHA
are recorded after the final serial Git operations.

## Final Decision

**MECHANISM CONFIRMED / CANDIDATES NO-GO.** Continuous raw observation can still oscillate at complete-link geometry
and growth boundaries, repeatedly changing group topology, ownership, PID birth, representative, and publication.
Route269 is confirmed as this mechanism rather than reacquisition. A-D are unsafe to activate from mechanical evidence
alone. E is the best mechanical candidate but remains research-only.

Evidence limits: offline replay parity is not downstream runtime proof, vehicle proof, NAS proof, or safety proof.

## Next Minimal Work

1. Combine this fixed candidate/event matrix with independently completed moving sequential `DIFFERENT` evidence,
   without retuning on partially written artifacts.
2. If that evidence closes A-D, rerun the same 847-segment, synthetic, prefix, frozen-control, and CPU gates unchanged.
3. For E, reduce the 80,770 publication-changed scans with a narrowly justified gate and validate downstream lead/control
   effects before considering a production patch.
4. Measure A1M CPU separately; do not extrapolate the x86 benchmark.
