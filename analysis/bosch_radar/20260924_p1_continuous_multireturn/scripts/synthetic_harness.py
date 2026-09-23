"""Deterministic adversarial harness using the actual Bosch group manager."""
from __future__ import annotations

import copy
import json

from candidate_modules import CANDIDATES, load_candidate_module
from common import MANIFESTS, TABLES, TRACES, write_csv, write_json


STEP = 100_000_000


def _track(module, raw_id, slot, ns, age, d, y, v=0.0):
  detection = module.BoschRawDetection(ns, slot, float(d), float(y), float(v), raw_word=raw_id)
  return module.BoschRawTrack(raw_id, detection, age)


def _scan(specs, repeat=1):
  return [tuple(specs) for _ in range(repeat)]


def cases():
  stable = [(1, 1, 40.0, 0.0), (2, 2, 42.75, 0.25)]
  return {
    "A_STABLE_PAIR": _scan(stable, 5),
    "B_DISTANCE_AT_THRESHOLD": _scan([(1, 1, 40.0, 0.0), (2, 2, 43.0, 0.0)], 5),
    "C_DISTANCE_PLUS_ONE_STEP": _scan(stable, 4) + _scan([(1, 1, 40.0, 0.0), (2, 2, 43.25, 0.25)], 1),
    "D_LATERAL_PLUS_ONE_STEP": _scan([(1, 1, 40.0, 0.0), (2, 2, 42.75, 1.25)], 4) + _scan([(1, 1, 40.0, 0.0), (2, 2, 42.75, 1.53125)], 1),
    "E_LATERAL_GROWTH": _scan([(1, 1, 40.0, 0.0), (2, 2, 42.5, 0.5)], 4) + _scan([(1, 1, 40.0, 0.0), (2, 2, 42.5, 1.28125)], 1),
    "F_ONE_SCAN_SPLIT_REJOIN": _scan(stable, 4) + _scan([(1, 1, 40.0, 0.0), (2, 2, 43.25, 0.25)], 1) + _scan(stable, 4),
    "G_TEN_OSCILLATIONS": _scan(stable, 4) + sum((_scan([(1, 1, 40.0, 0.0), (2, 2, 43.25, 0.25)], 1) + _scan(stable, 3) for _ in range(10)), []),
    "H_THREE_MEMBER_CHAIN": _scan([(1, 1, 40.0, 0.0), (2, 2, 42.0, 0.0), (3, 3, 43.25, 0.0)], 5),
    "I_THREE_MEMBER_EDGE_OSCILLATION": _scan([(1, 1, 40.0, 0.0), (2, 2, 41.5, 0.0), (3, 3, 43.0, 0.0)], 4) +
                                        sum((_scan([(1, 1, 40.0, 0.0), (2, 2, 41.5, 0.0), (3, 3, 43.25, 0.0)], 1) +
                                             _scan([(1, 1, 40.0, 0.0), (2, 2, 41.5, 0.0), (3, 3, 43.0, 0.0)], 3) for _ in range(4)), []),
    "J_REPRESENTATIVE_DISAPPEARANCE": _scan(stable, 5) + _scan([(2, 2, 42.75, 0.25)], 2),
    "K_NEW_THIRD_RAW_APPROACH": _scan(stable, 5) + _scan(stable + [(3, 3, 41.5, -0.25)], 4),
    "L_TWO_INDEPENDENT_CLOSE_GROUPS": _scan([(1, 1, 40.0, -1.75), (2, 2, 40.25, -1.5),
                                               (3, 3, 40.5, 1.5), (4, 4, 40.75, 1.75)], 5),
    "M_TRUCK_ELONGATED_PLUS_ADJACENT": _scan([(1, 1, 40.0, 0.0), (2, 2, 43.0, 0.0),
                                                (3, 3, 43.25, 1.75)], 5),
  }


def run_case(module, name, sequence):
  manager = module.BoschObjectGroupManager(module.BoschGroupingConfig(provisional_enabled=False))
  rows = []
  raw_age = {}
  previous_partition = None
  splits = rejoins = births = rep_changes = 0
  fractured_families = set()
  prior_by_pid = {}
  for scan, specs in enumerate(sequence):
    ns = (scan + 1) * STEP
    tracks = []
    for raw_id, slot, d, y, *rest in specs:
      raw_age[raw_id] = raw_age.get(raw_id, 0) + 1
      tracks.append(_track(module, raw_id, slot, ns, raw_age[raw_id], d, y, rest[0] if rest else 0.0))
    objects = manager.update(ns, tracks, v_ego=20.0)
    partition = tuple(sorted(tuple(sorted(member.raw_track_id for member in obj.members)) for obj in objects))
    if previous_partition is not None:
      previous_pairs = {frozenset(group) for group in previous_partition if len(group) > 1}
      current_owner = {raw: group for group in partition for raw in group}
      for group in previous_pairs:
        if len({current_owner.get(raw) for raw in group if raw in current_owner}) > 1:
          splits += 1
          fractured_families.add(group)
      current_pairs = {frozenset(group) for group in partition if len(group) > 1}
      for family in list(fractured_families):
        if any(family.issubset(group) for group in current_pairs):
          rejoins += 1
          fractured_families.remove(family)
    current_by_pid = {obj.physical_track_id: obj for obj in objects}
    births += sum(pid not in prior_by_pid for pid in current_by_pid)
    rep_changes += sum(pid in prior_by_pid and prior_by_pid[pid].representative_raw_track_id != obj.representative_raw_track_id
                       for pid, obj in current_by_pid.items())
    rows.append({"case": name, "scan": scan, "scan_ns": ns,
                 "partition": ";".join("|".join(map(str, group)) for group in partition),
                 "pids": ";".join(str(obj.physical_track_id) for obj in objects),
                 "representatives": ";".join(str(obj.representative_raw_track_id) for obj in objects),
                 "relaxed_edges": len(getattr(manager, "_p1_relaxed_edges_last", ())),
                 "state_size": len(getattr(manager, "_p1_fail_counts", {})) + len(getattr(manager, "_p1_last_strict_ns", {}))})
    previous_partition = partition
    prior_by_pid = current_by_pid
  return rows, {"candidate": getattr(module, "BOSCH_P1_RESEARCH_CANDIDATE", "BASELINE"), "case": name,
                "scans": len(sequence), "final_partition": rows[-1]["partition"],
                "split_events": splits, "rejoin_events": rejoins, "pid_births": births,
                "representative_changes": rep_changes,
                "state_peak": max((row["state_size"] for row in rows), default=0)}


def lifecycle_checks(module):
  manager = module.BoschObjectGroupManager(module.BoschGroupingConfig(provisional_enabled=False))
  stable = [(1, 1, 40.0, 0.0), (2, 2, 42.75, 0.25)]
  age = 0
  for scan in range(4):
    age += 1
    ns = (scan+1)*STEP
    manager.update(ns, [_track(module, raw, slot, ns, age, d, y) for raw, slot, d, y in stable], v_ego=20.)
  before = copy.deepcopy(manager.__dict__)
  timestamp_refused = False
  try:
    manager.update(4*STEP, [], v_ego=20.)
  except ValueError:
    timestamp_refused = True
  timestamp_unchanged = before == manager.__dict__
  gap_ns = 9*STEP
  manager.update(gap_ns, [], v_ego=20.)
  stale_state = (len(getattr(manager, "_p1_prior_edges", ())) + len(getattr(manager, "_p1_fail_counts", {})) +
                 len(getattr(manager, "_p1_last_strict_ns", {})))
  reset_manager = module.BoschObjectGroupManager(module.BoschGroupingConfig(provisional_enabled=False))
  return {"candidate": getattr(module, "BOSCH_P1_RESEARCH_CANDIDATE", "BASELINE"),
          "timestamp_regression_refused": int(timestamp_refused),
          "timestamp_regression_state_unchanged": int(timestamp_unchanged),
          "stale_state_after_gap": stale_state,
          "provider_reset_fresh_state": int(not getattr(reset_manager, "_p1_prior_edges", ())) }


def main() -> None:
  all_rows, summaries, lifecycle = [], [], []
  for index, candidate in enumerate(CANDIDATES):
    module = load_candidate_module(candidate, f"p1_synth_{index}")
    for name, sequence in cases().items():
      rows, summary = run_case(module, name, sequence)
      for row in rows:
        row["candidate"] = candidate
      all_rows.extend(rows)
      summary["candidate"] = candidate
      summaries.append(summary)
      if candidate == "BASELINE":
        write_csv(TRACES / "synthetic" / f"{name.lower()}.csv", rows)
    life = lifecycle_checks(module)
    life["candidate"] = candidate
    lifecycle.append(life)
  write_csv(TABLES / "synthetic_results.csv", summaries)
  write_csv(TABLES / "synthetic_lifecycle.csv", lifecycle)
  write_json(MANIFESTS / "synthetic_summary.json", {"cases": len(cases()), "variants": len(CANDIDATES),
                                                     "results": summaries, "lifecycle": lifecycle})
  print(json.dumps({"cases": len(cases()), "variants": len(CANDIDATES), "rows": len(all_rows)}, indent=2))


if __name__ == "__main__":
  main()
