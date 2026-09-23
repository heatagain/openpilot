"""Evaluate candidates on already-frozen SAME and simultaneous DIFFERENT controls.

The script consumes prior frozen trace mappings only. It performs no video read,
endpoint attribution, or new actor adjudication.
"""
from __future__ import annotations

import csv
import json
import multiprocessing as mp
from collections import defaultdict
from pathlib import Path

from candidate_modules import CANDIDATES
from common import (FROZEN_GT, GROUP_STUDY, MANIFESTS, TABLES, WORKSPACE, corpus_keys,
                    key_for_route_segment, process_count, write_csv, write_json)
from evaluate_group_candidate import _raw_owner, _run_variant


DIFFERENT_TRACES = (WORKSPACE / "20260920_MRRevo14F_surface_fragment_parent_association" /
                    "traces" / "critical_real_vehicle")
SAME_TRACES = (WORKSPACE / "analysis" / "20260920_MRRevo14F_typeA_identity_restratification" /
               "traces" / "clone")


def _trace_anchor(path: Path):
  with path.open(encoding="utf-8", newline="") as stream:
    rows = [row for row in csv.DictReader(stream)
            if row.get("a_present") == "1" and row.get("b_present") == "1" and
            row.get("a_representative") and row.get("b_representative")]
  if not rows:
    return None
  return min(rows, key=lambda row: abs(float(row.get("t_from_birth_s") or 0.0)))


def load_controls():
  keys = corpus_keys()
  controls = []
  with (GROUP_STUDY / "tables" / "different_transition_controls.csv").open(encoding="utf-8", newline="") as stream:
    different = list(csv.DictReader(stream))
  for row in different:
    trace = DIFFERENT_TRACES / f"{row['candidate_id']}.csv"
    anchor = _trace_anchor(trace) if trace.exists() else None
    key = key_for_route_segment(row["route"], int(row["segment"]), keys)
    if anchor and key:
      controls.append({"candidate_id": row["candidate_id"], "relation": "DIFFERENT_PHYSICAL_OBJECTS",
                       "key": key, "scan_ns": int(anchor["scan_ns"]),
                       "raw_a": int(anchor["a_representative"]), "raw_b": int(anchor["b_representative"]),
                       "source": str(trace)})

  with FROZEN_GT.open(encoding="utf-8", newline="") as stream:
    same = [row for row in csv.DictReader(stream) if row["identity_relation"] == "SAME_PHYSICAL_OBJECT"]
  for row in same:
    trace = SAME_TRACES / f"{row['candidate_id']}.csv"
    anchor = _trace_anchor(trace) if trace.exists() else None
    key = key_for_route_segment(row["route"], int(row["segment"]), keys)
    if anchor and key:
      controls.append({"candidate_id": row["candidate_id"], "relation": "SAME_PHYSICAL_OBJECT",
                       "key": key, "scan_ns": int(anchor["scan_ns"]),
                       "raw_a": int(anchor["a_representative"]), "raw_b": int(anchor["b_representative"]),
                       "source": str(trace)})
  by_key = defaultdict(list)
  for row in controls:
    by_key[row["key"]].append(row)
  return dict(by_key)


_CONTROLS = load_controls()


def process_key(key: str):
  runs = {candidate: _run_variant(key, candidate)["snapshots"] for candidate in CANDIDATES}
  baseline = runs["BASELINE"]
  rows = []
  for control in _CONTROLS[key]:
    index = min(range(len(baseline)), key=lambda i: abs(baseline[i]["ns"] - control["scan_ns"]))
    base_owner = _raw_owner(baseline[index])
    raw_a, raw_b = control["raw_a"], control["raw_b"]
    baseline_evaluable = raw_a in base_owner and raw_b in base_owner
    baseline_same = baseline_evaluable and base_owner[raw_a]["pid"] == base_owner[raw_b]["pid"]
    for candidate in CANDIDATES[1:]:
      owner = _raw_owner(runs[candidate][index])
      evaluable = baseline_evaluable and raw_a in owner and raw_b in owner
      candidate_same = evaluable and owner[raw_a]["pid"] == owner[raw_b]["pid"]
      if control["relation"] == "DIFFERENT_PHYSICAL_OBJECTS":
        regression = evaluable and not baseline_same and candidate_same
        reason = "FALSE_MERGE" if regression else "PRESERVED_SEPARATE" if evaluable else "UNEVALUABLE"
      else:
        regression = evaluable and baseline_same and not candidate_same
        reason = "ADDITIONAL_FRAGMENT" if regression else "NOT_WORSE" if evaluable else "UNEVALUABLE"
      rows.append({"candidate": candidate, **control, "matched_scan_ns": baseline[index]["ns"],
                   "baseline_evaluable": int(baseline_evaluable), "baseline_same_group": int(baseline_same),
                   "candidate_evaluable": int(evaluable), "candidate_same_group": int(candidate_same),
                   "regression": int(regression), "reason": reason})
  return rows


def main() -> None:
  rows = []
  keys = sorted(_CONTROLS)
  with mp.Pool(processes=process_count(8), maxtasksperchild=4) as pool:
    for count, result in enumerate(pool.imap_unordered(process_key, keys, chunksize=1), 1):
      rows.extend(result)
      if count % 5 == 0 or count == len(keys):
        print(f"{count}/{len(keys)} control segments", flush=True)
  rows.sort(key=lambda row: (row["candidate"], row["relation"], row["candidate_id"]))
  write_csv(TABLES / "frozen_control_regression.csv", rows)
  summary = []
  for candidate in CANDIDATES[1:]:
    subset = [row for row in rows if row["candidate"] == candidate]
    summary.append({
      "candidate": candidate,
      "different_controls": sum(row["relation"] == "DIFFERENT_PHYSICAL_OBJECTS" for row in subset),
      "different_evaluable": sum(row["relation"] == "DIFFERENT_PHYSICAL_OBJECTS" and row["candidate_evaluable"] for row in subset),
      "different_false_merges": sum(row["relation"] == "DIFFERENT_PHYSICAL_OBJECTS" and row["regression"] for row in subset),
      "same_controls": sum(row["relation"] == "SAME_PHYSICAL_OBJECT" for row in subset),
      "same_evaluable": sum(row["relation"] == "SAME_PHYSICAL_OBJECT" and row["candidate_evaluable"] for row in subset),
      "same_regressions": sum(row["relation"] == "SAME_PHYSICAL_OBJECT" and row["regression"] for row in subset),
    })
  write_csv(TABLES / "frozen_control_summary.csv", summary)
  write_json(MANIFESTS / "frozen_control_summary.json", {"segments": len(keys), "summary": summary})
  print(json.dumps({"segments": len(keys), "summary": summary}, indent=2))


if __name__ == "__main__":
  mp.freeze_support()
  main()
