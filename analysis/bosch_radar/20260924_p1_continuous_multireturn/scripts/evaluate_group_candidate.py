"""Full-corpus, analysis-only comparison of five mechanical candidates."""
from __future__ import annotations

import csv
import json
import multiprocessing as mp
import time
from collections import defaultdict

from candidate_modules import CANDIDATES, load_candidate_module
from common import (FROZEN_GT, MANIFESTS, SOURCE, TABLES, configure_imports, corpus_keys,
                    ensure_dirs, key_for_route_segment, load_replay_module, percentile,
                    process_count, write_csv, write_json)


ROUTE269_KEY = "00000269--4014745f93--7"
ROUTE269_EVENTS = ((400, 405), (407, 408), (450, 454), (463, 470), (482, 500),
                   (503, 512), (517, 534), (537, 540), (557, 570), (583, 585), (586, 587))


def _obj(obj) -> dict:
  return {"pid": obj.physical_track_id, "members": tuple(sorted(m.raw_track_id for m in obj.members)),
          "representative": obj.representative_raw_track_id, "d": obj.d_rel, "y": obj.y_rel,
          "v": obj.v_rel, "age": obj.age_scans}


def _run_variant(key: str, candidate: str) -> dict:
  configure_imports()
  import replay_harness as rh
  replay = load_replay_module()
  observations = []
  timings = []
  original_load = rh.load_module

  def hooked_load(name):
    module = load_candidate_module(candidate, f"{name}_{candidate.lower()}", SOURCE)
    original_provider = module.BoschRadarProvider

    class TimedProvider(original_provider):
      def update(self, *args, **kwargs):
        started = time.perf_counter_ns()
        result = super().update(*args, **kwargs)
        timings.append(time.perf_counter_ns() - started)
        if result is not None:
          group = self.tracker.group_manager
          observations.append({
            "ns": self.last_scan_timestamp_ns,
            "objects": [_obj(obj) for obj in self._debug_objects],
            "state_peak": getattr(group, "_p1_state_peak", 0),
            "relaxed_edges": len(getattr(group, "_p1_relaxed_edges_last", ())),
          })
        return result

    module.BoschRadarProvider = TimedProvider
    return module

  rh.load_module = hooked_load
  try:
    records = replay.run_segment(key)
  finally:
    rh.load_module = original_load
  if len(records) != len(observations):
    raise AssertionError(f"{candidate} {key}: record/snapshot mismatch")
  for snapshot, record in zip(observations, records, strict=True):
    snapshot["published"] = tuple(sorted(
      (tuple(obj["raw_ids"]), int(obj["representative"]), float(obj["d"]), float(obj["y"]),
       float(obj["v"]), int(obj["public_alias"]))
      for obj in record["objects"] if obj["published"]
    ))
  return {"snapshots": observations, "timings": timings}


def _raw_owner(snapshot: dict) -> dict[int, dict]:
  return {raw: obj for obj in snapshot["objects"] for raw in obj["members"]}


def _partition(snapshot: dict) -> tuple:
  return tuple(sorted(obj["members"] for obj in snapshot["objects"]))


def _baseline_fractures(snapshots: list[dict]) -> list[tuple[int, tuple[int, ...]]]:
  events = []
  for index in range(1, len(snapshots)):
    current = _raw_owner(snapshots[index])
    for parent in snapshots[index-1]["objects"]:
      if len(parent["members"]) < 2:
        continue
      observed = [raw for raw in parent["members"] if raw in current]
      if len(observed) >= 2 and len({current[raw]["pid"] for raw in observed}) >= 2:
        events.append((index, parent["members"]))
  return events


def _persistent_merge_episodes(baseline: list[dict], candidate: list[dict]) -> tuple[int, int]:
  active = {}
  episodes = []
  for index, (base, cand) in enumerate(zip(baseline, candidate, strict=True)):
    base_owner = _raw_owner(base)
    current = set()
    for obj in cand["objects"]:
      base_groups = {base_owner[raw]["pid"] for raw in obj["members"] if raw in base_owner}
      if len(base_groups) > 1:
        key = obj["members"]
        current.add(key)
        active[key] = active.get(key, [index, 0])
        active[key][1] += 1
    for key in list(active):
      if key not in current:
        start, scans = active.pop(key)
        episodes.append((start, index-1, scans, key))
  for key, (start, scans) in active.items():
    episodes.append((start, len(candidate)-1, scans, key))
  persistent = [episode for episode in episodes if episode[2] >= 3]
  return len(episodes), len(persistent)


def _representative_changes(snapshots: list[dict]) -> int:
  count = 0
  for previous, current in zip(snapshots, snapshots[1:], strict=False):
    old = {obj["pid"]: obj for obj in previous["objects"]}
    for obj in current["objects"]:
      prior = old.get(obj["pid"])
      if prior and prior["representative"] != obj["representative"]:
        count += 1
  return count


def _controls_by_key(keys: list[str]) -> dict[str, list[dict]]:
  result = defaultdict(list)
  if not FROZEN_GT.exists():
    return result
  with FROZEN_GT.open(encoding="utf-8", newline="") as stream:
    for row in csv.DictReader(stream):
      relation = row.get("identity_relation", "")
      if relation not in ("SAME_PHYSICAL_OBJECT", "DIFFERENT_PHYSICAL_OBJECTS"):
        continue
      key = key_for_route_segment(row["route"], int(row["segment"]), keys)
      if key is None:
        continue
      row = dict(row)
      row["key"] = key
      row["target_ns"] = round((int(row["segment"]) * 60 + float(row["time"])) * 1e9)
      result[key].append(row)
  return result


_ALL_KEYS = corpus_keys()
_CONTROLS = _controls_by_key(_ALL_KEYS)


def _evaluate_controls(key: str, baseline: list[dict], candidate: list[dict], name: str) -> list[dict]:
  rows = []
  for control in _CONTROLS.get(key, ()):
    index = min(range(len(baseline)), key=lambda i: abs(baseline[i]["ns"] - control["target_ns"]))
    base_by_pid = {obj["pid"]: obj for obj in baseline[index]["objects"]}
    try:
      pid_a, pid_b = int(control["object_a_pid"]), int(control["object_b_pid"])
    except (TypeError, ValueError):
      continue
    first, second = base_by_pid.get(pid_a), base_by_pid.get(pid_b)
    if first is None or second is None:
      rows.append({"candidate": name, "candidate_id": control["candidate_id"], "key": key,
                   "identity_relation": control["identity_relation"], "evaluable": 0,
                   "regression": "", "reason": "CURRENT_HEAD_PID_NOT_PRESENT"})
      continue
    cand_owner = _raw_owner(candidate[index])
    first_owners = {cand_owner[raw]["pid"] for raw in first["members"] if raw in cand_owner}
    second_owners = {cand_owner[raw]["pid"] for raw in second["members"] if raw in cand_owner}
    merged = bool(first_owners.intersection(second_owners))
    fragmented = len(first_owners) > 1 or len(second_owners) > 1
    relation = control["identity_relation"]
    regression = merged if relation == "DIFFERENT_PHYSICAL_OBJECTS" else fragmented
    rows.append({"candidate": name, "candidate_id": control["candidate_id"], "key": key,
                 "identity_relation": relation, "evaluable": 1, "regression": int(regression),
                 "reason": "FALSE_MERGE" if merged and relation == "DIFFERENT_PHYSICAL_OBJECTS"
                           else "ADDITIONAL_FRAGMENT" if fragmented and relation == "SAME_PHYSICAL_OBJECT" else "PRESERVED",
                 "baseline_raw_a": "|".join(map(str, first["members"])),
                 "baseline_raw_b": "|".join(map(str, second["members"])),
                 "scan_ns": baseline[index]["ns"]})
  return rows


def _process_segment(key: str) -> dict:
  runs = {candidate: _run_variant(key, candidate) for candidate in CANDIDATES}
  baseline = runs["BASELINE"]["snapshots"]
  fractures = _baseline_fractures(baseline)
  rows, route_rows, controls = [], [], []

  for candidate in CANDIDATES[1:]:
    current = runs[candidate]["snapshots"]
    if len(current) != len(baseline):
      raise AssertionError(f"{key} {candidate}: scan count changed")
    grouping_changed = member_count_changed = publication_changed = 0
    new_merges = new_splits = suppressed = pid_births = relaxed_scans = 0
    for base, cand in zip(baseline, current, strict=True):
      base_partition, candidate_partition = _partition(base), _partition(cand)
      grouping_changed += base_partition != candidate_partition
      member_count_changed += sorted(map(len, base_partition)) != sorted(map(len, candidate_partition))
      publication_changed += base["published"] != cand["published"]
      relaxed_scans += cand["relaxed_edges"] > 0
      base_owner, cand_owner = _raw_owner(base), _raw_owner(cand)
      for obj in cand["objects"]:
        if len({base_owner[raw]["pid"] for raw in obj["members"] if raw in base_owner}) > 1:
          new_merges += 1
      for obj in base["objects"]:
        if len({cand_owner[raw]["pid"] for raw in obj["members"] if raw in cand_owner}) > 1:
          new_splits += 1
      pid_births += sum(obj["age"] == 1 for obj in cand["objects"])
    for index, members in fractures:
      owners = _raw_owner(current[index])
      observed = [raw for raw in members if raw in owners]
      if len(observed) >= 2 and len({owners[raw]["pid"] for raw in observed}) == 1:
        suppressed += 1
    merge_episodes, persistent_merges = _persistent_merge_episodes(baseline, current)
    times_us = [value / 1000 for value in runs[candidate]["timings"]]
    rows.append({
      "key": key, "candidate": candidate, "scans": len(current),
      "baseline_fractures": len(fractures), "suppressed_fractures": suppressed,
      "grouping_changed_scans": grouping_changed, "member_count_changed_scans": member_count_changed,
      "new_merge_objects": new_merges, "new_split_objects": new_splits,
      "new_merge_episodes": merge_episodes, "persistent_new_merges": persistent_merges,
      "publication_changed_scans": publication_changed,
      "representative_changes": _representative_changes(current), "pid_births": pid_births,
      "relaxed_edge_scans": relaxed_scans,
      "state_peak": max((snap["state_peak"] for snap in current), default=0),
      "cpu_mean_us": sum(times_us)/len(times_us), "cpu_p95_us": percentile(times_us, .95),
      "cpu_p99_us": percentile(times_us, .99), "cpu_max_us": max(times_us),
    })
    controls.extend(_evaluate_controls(key, baseline, current, candidate))

    if key == ROUTE269_KEY:
      for event_index, (split_scan, rejoin_scan) in enumerate(ROUTE269_EVENTS, 1):
        owners_at_split = _raw_owner(current[split_scan])
        same_at_split = (436 in owners_at_split and 482 in owners_at_split and
                         owners_at_split[436]["pid"] == owners_at_split[482]["pid"])
        delayed_scan = ""
        if same_at_split:
          for scan in range(split_scan + 1, min(rejoin_scan + 1, len(current))):
            owners = _raw_owner(current[scan])
            if 436 in owners and 482 in owners and owners[436]["pid"] != owners[482]["pid"]:
              delayed_scan = scan
              break
        outcome = "DELAYED_SPLIT" if delayed_scan != "" else "PREVENTED_SPLIT" if same_at_split else "UNCHANGED_SPLIT"
        route_rows.append({"candidate": candidate, "event": event_index, "baseline_split_scan": split_scan,
                           "baseline_rejoin_scan": rejoin_scan, "outcome": outcome,
                           "candidate_split_scan": delayed_scan if delayed_scan != "" else split_scan if not same_at_split else "",
                           "candidate_same_group_at_baseline_split": int(same_at_split)})

  return {"key": key, "rows": rows, "route_rows": route_rows, "controls": controls,
          "baseline_scans": len(baseline), "baseline_rep_changes": _representative_changes(baseline)}


def main() -> None:
  ensure_dirs()
  keys = corpus_keys()
  if len(keys) != 847:
    raise RuntimeError(f"expected 847 corpus segments, found {len(keys)}")
  rows, route_rows, controls = [], [], []
  baseline_scans = baseline_rep_changes = 0
  with mp.Pool(processes=process_count(8), maxtasksperchild=5) as pool:
    for count, result in enumerate(pool.imap_unordered(_process_segment, keys, chunksize=1), 1):
      rows.extend(result["rows"])
      route_rows.extend(result["route_rows"])
      controls.extend(result["controls"])
      baseline_scans += result["baseline_scans"]
      baseline_rep_changes += result["baseline_rep_changes"]
      if count % 5 == 0 or count == len(keys):
        print(f"{count}/{len(keys)} segments; {len(rows)} candidate-segment comparisons", flush=True)

  rows.sort(key=lambda row: (row["candidate"], row["key"]))
  write_csv(TABLES / "candidate_segment_results.csv", rows)
  write_csv(TABLES / "route269_candidate_events.csv", sorted(route_rows, key=lambda row: (row["candidate"], row["event"])))
  write_csv(TABLES / "regression_summary.csv", sorted(controls, key=lambda row: (row["candidate"], row["candidate_id"])))

  summary_rows = []
  for candidate in CANDIDATES[1:]:
    subset = [row for row in rows if row["candidate"] == candidate]
    times_weight = sum(int(row["scans"]) for row in subset)
    control_subset = [row for row in controls if row["candidate"] == candidate]
    route_subset = [row for row in route_rows if row["candidate"] == candidate]
    summary_rows.append({
      "candidate": candidate, "segments": len(subset), "scans": sum(int(row["scans"]) for row in subset),
      "baseline_fractures": sum(int(row["baseline_fractures"]) for row in subset),
      "suppressed_fractures": sum(int(row["suppressed_fractures"]) for row in subset),
      "grouping_changed_scans": sum(int(row["grouping_changed_scans"]) for row in subset),
      "member_count_changed_scans": sum(int(row["member_count_changed_scans"]) for row in subset),
      "new_merge_objects": sum(int(row["new_merge_objects"]) for row in subset),
      "persistent_new_merges": sum(int(row["persistent_new_merges"]) for row in subset),
      "new_split_objects": sum(int(row["new_split_objects"]) for row in subset),
      "publication_changed_scans": sum(int(row["publication_changed_scans"]) for row in subset),
      "representative_changes": sum(int(row["representative_changes"]) for row in subset),
      "pid_births": sum(int(row["pid_births"]) for row in subset),
      "route269_prevented": sum(row["outcome"] == "PREVENTED_SPLIT" for row in route_subset),
      "route269_delayed": sum(row["outcome"] == "DELAYED_SPLIT" for row in route_subset),
      "route269_unchanged": sum(row["outcome"] == "UNCHANGED_SPLIT" for row in route_subset),
      "different_controls_evaluable": sum(row["identity_relation"] == "DIFFERENT_PHYSICAL_OBJECTS" and int(row["evaluable"])
                                            for row in control_subset),
      "different_control_regressions": sum(row["identity_relation"] == "DIFFERENT_PHYSICAL_OBJECTS" and row["regression"] not in ("", None)
                                            and int(row["regression"]) for row in control_subset),
      "same_controls_evaluable": sum(row["identity_relation"] == "SAME_PHYSICAL_OBJECT" and int(row["evaluable"])
                                       for row in control_subset),
      "same_control_regressions": sum(row["identity_relation"] == "SAME_PHYSICAL_OBJECT" and row["regression"] not in ("", None)
                                       and int(row["regression"]) for row in control_subset),
      "state_peak": max((int(row["state_peak"]) for row in subset), default=0),
      "cpu_mean_us_weighted": (sum(float(row["cpu_mean_us"])*int(row["scans"]) for row in subset)/times_weight
                                if times_weight else None),
      "cpu_p95_segment_us": percentile([float(row["cpu_p95_us"]) for row in subset], .95),
      "cpu_p99_segment_us": percentile([float(row["cpu_p99_us"]) for row in subset], .99),
      "cpu_max_us": max((float(row["cpu_max_us"]) for row in subset), default=None),
    })
  write_csv(TABLES / "candidate_results.csv", summary_rows)
  write_csv(TABLES / "cpu_summary.csv", summary_rows,
            ["candidate", "segments", "scans", "cpu_mean_us_weighted", "cpu_p95_segment_us",
             "cpu_p99_segment_us", "cpu_max_us", "state_peak"])
  manifest = {"source": str(SOURCE), "segments": len(keys), "baseline_scans": baseline_scans,
              "baseline_representative_changes": baseline_rep_changes, "candidates": summary_rows}
  write_json(MANIFESTS / "candidate_evaluation.json", manifest)
  print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
  mp.freeze_support()
  main()
