"""Run P0-P6 over the full corpus and materialize compact study artifacts."""
from __future__ import annotations

import argparse
import csv
import json
import multiprocessing as mp
import os
from collections import Counter
from collections.abc import Iterable
from pathlib import Path
from typing import Any

from replay_post_assignment_surface import (JUMP_P99, MANIFESTS, POLICIES, ROUTE269_KEY,
                                            ROUTE280_KEY, STATEFUL_POLICIES, TABLES,
                                            TRACES, corpus_keys, ensure_dirs, percentile,
                                            read_json_gz, result_path, run_segment,
                                            stable_control_windows, write_json_gz)


INVARIANT_FIELDS = (
  "raw", "group", "pid", "member", "tracking_representative", "qualification",
  "publication_set", "alias", "internal_aLead",
)


def write_json(path: Path, value: Any) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8", newline="\n")


def write_csv(path: Path, rows: Iterable[dict[str, Any]], fields: Iterable[str] | None = None) -> None:
  materialized = list(rows)
  if fields is None:
    names: list[str] = []
    for row in materialized:
      for name in row:
        if name not in names:
          names.append(name)
  else:
    names = list(fields)
  path.parent.mkdir(parents=True, exist_ok=True)
  with path.open("w", encoding="utf-8", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=names, extrasaction="ignore", lineterminator="\n")
    writer.writeheader()
    writer.writerows(materialized)


def distribution(values: Iterable[float]) -> dict[str, Any]:
  materialized = [float(value) for value in values]
  return {
    "n": len(materialized), "p50": percentile(materialized, .50),
    "p90": percentile(materialized, .90), "p95": percentile(materialized, .95),
    "p99": percentile(materialized, .99),
    "max": max(materialized) if materialized else None,
  }


def _process(payload: tuple[str, bool]) -> tuple[str, int, int]:
  key, force = payload
  destination = result_path(key)
  if destination.exists() and not force:
    cached = read_json_gz(destination)
    # A no-downstream development cache is not a valid full result.
    p5 = cached["policies"]["P5"]["summary"]
    if (p5.get("publication_changed_scans", 0) == 0 or
        p5.get("NO_DOWNSTREAM_EFFECT", 0)+p5.get("radarstate_changed_scans", 0) > 0):
      return key, -1, -1
  result = run_segment(key, downstream=True)
  write_json_gz(destination, result)
  return key, int(result["scans"]), int(result["policies"]["P6"]["summary"].get("publication_changed_scans", 0))


def _sum(rows: list[dict[str, Any]], field: str) -> int:
  return sum(int(row.get(field, 0) or 0) for row in rows)


def finalize() -> dict[str, Any]:
  keys = corpus_keys()
  baseline_parity = Counter()
  source_current = Counter()
  route280: dict[str, Any] = {}
  p0_duration = Counter()
  multi_return = {policy: Counter() for policy in POLICIES}
  route269_invariance = {policy: Counter() for policy in POLICIES}
  completed_scans = 0
  for key in keys:
    path = result_path(key)
    if not path.exists():
      raise FileNotFoundError(path)
    result = read_json_gz(path)
    completed_scans += int(result["scans"])
    baseline_parity.update({field: int(value) for field, value in result["baseline_parity"].items()})
    source_current.update({field: int(value) for field, value in result["source_current"].items()})
    controls = result.get("controls", {})
    if key == ROUTE280_KEY:
      route280 = controls.get("route280", {})
    for policy, row in controls.get("p0", {}).items():
      p0_duration[policy] += max(0, int(row["publication_duration_delta_scans"]))
    for policy, row in controls.get("multi_return", {}).items():
      multi_return[policy].update({field: int(value) for field, value in row.items()})
    if key == ROUTE269_KEY:
      for policy in POLICIES:
        route269_invariance[policy].update({field: int(value) for field, value in
                                            result["policies"][policy]["invariance"].items()})

  routes = len({key.split("--", 1)[0] for key in keys})
  summaries = []
  jump_rows = []
  dwell_rows = []
  pingpong_rows = []
  divergence_rows = []
  cpu_rows = []
  invariant_rows = []
  decision_rows = []
  stable_event_rows = []
  candidate_tail_rows = []
  baseline_pingpong_sample = []
  invariance_by_policy: dict[str, Counter[str]] = {}
  for policy in POLICIES:
    aggregate = Counter()
    policy_invariance = Counter()
    policy_reasons = Counter()
    policy_cpu: list[float] = []
    jump_values = {population: {field: [] for field in ("abs_delta_d", "abs_delta_y", "abs_delta_v")}
                   for population in ("ALL", "STABLE_GROUP")}
    dwell_durations: list[float] = []
    dwell_counts = Counter()
    ping_counts = Counter()
    divergence_scans: list[float] = []
    divergence_durations: list[float] = []
    changed_sample = []
    consistency_samples = []
    downstream_sample = []
    policy_peak = 0
    for key in keys:
      data = read_json_gz(result_path(key))["policies"][policy]
      aggregate.update({field: int(value or 0) for field, value in data["summary"].items()})
      policy_invariance.update({field: int(value) for field, value in data["invariance"].items()})
      policy_reasons.update({field: int(value) for field, value in data["decision_reasons"].items()})
      policy_peak = max(policy_peak, int(data["state_peak"]))
      policy_cpu.extend(float(value) for value in data["cpu_us"])
      for row in data["jumps"]:
        enriched = {"key": key, "policy": policy, **row}
        for field in ("abs_delta_d", "abs_delta_y", "abs_delta_v"):
          jump_values["ALL"][field].append(float(row[field]))
          if int(row["stable_members"]):
            jump_values["STABLE_GROUP"][field].append(float(row[field]))
        if int(row["stable_members"]) and len(stable_event_rows) < 1500:
          stable_event_rows.append(enriched)
        if policy != "P0":
          candidate_tail_rows.append(enriched)
      for row in data["dwells"]:
        dwell_durations.append(float(row["duration_s"]))
        dwell_counts[int(row["scans"])] += 1
      for row in data["pingpongs"]:
        ping_counts[row["pattern"]] += 1
        if row["pattern"] == "A_B_A" and int(row["stable_members"]):
          for label, limit in (("stable_A_B_A_le_0_2s", .2), ("stable_A_B_A_le_0_5s", .5),
                               ("stable_A_B_A_le_1_0s", 1.0), ("stable_A_B_A_le_2_0s", 2.0)):
            ping_counts[label] += int(float(row["duration_s"]) <= limit)
          if policy == "P0" and len(baseline_pingpong_sample) < 300:
            baseline_pingpong_sample.append({"key": key, "policy": policy, **row})
      for row in data["divergence"]:
        divergence_scans.append(float(row["scans"]))
        divergence_durations.append(float(row["duration_s"]))
      if len(changed_sample) < 250:
        changed_sample.extend({"key": key, "policy": policy, **row}
                              for row in data["changed_samples"][:250-len(changed_sample)])
      consistency_samples.extend({"key": key, "policy": policy, **row}
                                 for row in data["changed_samples"])
      if len(downstream_sample) < 250:
        downstream_sample.extend({"key": key, **row}
                                 for row in data["downstream_samples"][:250-len(downstream_sample)])

    summary: dict[str, Any] = {
      "policy": policy, "routes": routes, "segments": len(keys), "scans": completed_scans,
    }
    summary.update(dict(aggregate))
    summary["state_peak"] = int(policy_peak)
    summary["p0_publication_duration_regression_scans"] = int(p0_duration[policy])
    summary["route280_first_publication_delay_ms"] = (route280.get(policy) or {}).get(
      "first_publication_delay_ms")
    summary["route280_target_coordinate_changed_scans"] = int((route280.get(policy) or {}).get(
      "target_coordinate_changed_scans", 0))
    summary["route269_tracking_regression"] = int(route269_invariance[policy]["tracking_representative"])
    summary["multi_return_controls"] = int(multi_return[policy]["controls"])
    summary["multi_return_surface_switches"] = int(multi_return[policy]["surface_switches"])
    summary["multi_return_baseline_different_scans"] = int(multi_return[policy]["baseline_different_scans"])
    for field in INVARIANT_FIELDS:
      summary[f"{field}_mismatch"] = int(policy_invariance[field])

    for population in ("ALL", "STABLE_GROUP"):
      for coordinate, field in (("dRel", "abs_delta_d"), ("yRel", "abs_delta_y"), ("vRel", "abs_delta_v")):
        row = {"policy": policy, "population": population, "coordinate": coordinate,
               **distribution(jump_values[population][field])}
        jump_rows.append(row)
        if population == "ALL":
          summary[f"jump_{coordinate}_p50"] = row["p50"]
          summary[f"jump_{coordinate}_p95"] = row["p95"]
          summary[f"jump_{coordinate}_p99"] = row["p99"]
          summary[f"jump_{coordinate}_max"] = row["max"]

    dwell_dist = distribution(dwell_durations)
    dwell_rows.append({
      "policy": policy, "runs": len(dwell_durations),
      "one_scan": dwell_counts[1], "two_scan": dwell_counts[2], "three_scan": dwell_counts[3],
      **{f"duration_{field}_s": value for field, value in dwell_dist.items()},
    })
    pingpong_rows.append({
      "policy": policy, "A_B_A": ping_counts["A_B_A"], "A_B_A_B": ping_counts["A_B_A_B"],
      "stable_A_B_A_le_0_2s": ping_counts["stable_A_B_A_le_0_2s"],
      "stable_A_B_A_le_0_5s": ping_counts["stable_A_B_A_le_0_5s"],
      "stable_A_B_A_le_1_0s": ping_counts["stable_A_B_A_le_1_0s"],
      "stable_A_B_A_le_2_0s": ping_counts["stable_A_B_A_le_2_0s"],
    })
    episode_scan_dist = distribution(divergence_scans)
    episode_time_dist = distribution(divergence_durations)
    divergence_rows.append({
      "policy": policy, "episodes": len(divergence_scans),
      **{f"scans_{field}": value for field, value in episode_scan_dist.items()},
      **{f"duration_{field}_s": value for field, value in episode_time_dist.items()},
    })
    summary.update({f"divergence_scans_{field}": value for field, value in episode_scan_dist.items()})
    summary.update({f"divergence_duration_{field}_s": value for field, value in episode_time_dist.items()})
    cpu_dist = distribution(policy_cpu)
    cpu_row = {
      "policy": policy, "completed_scans": completed_scans,
      "cpu_mean_us": sum(policy_cpu)/len(policy_cpu),
      "cpu_p50_us": cpu_dist["p50"], "cpu_p95_us": cpu_dist["p95"],
      "cpu_p99_us": cpu_dist["p99"], "cpu_max_us": cpu_dist["max"],
      "cpu_wall_s": sum(policy_cpu)*1e-6, "state_peak": int(policy_peak),
    }
    cpu_rows.append(cpu_row)
    summary.update(cpu_row)
    invariant_rows.extend({"policy": policy, "invariant": field,
                           "mismatch_scans": int(policy_invariance[field]),
                           "pass": int(policy_invariance[field] == 0)} for field in INVARIANT_FIELDS)
    decision_rows.extend({"policy": policy, "reason": reason, "decisions": count}
                         for reason, count in sorted(policy_reasons.items()))
    write_csv(TRACES / "publication" / f"{policy.lower()}_changed_sample.csv", changed_sample)
    consistency_samples.sort(key=lambda row: abs(float(row["delta_v"])), reverse=True)
    write_csv(TRACES / "publication" / f"{policy.lower()}_vrel_alead_consistency_tail.csv",
              consistency_samples[:300])
    write_csv(TRACES / "radarstate" / f"{policy.lower()}_downstream_sample.csv", downstream_sample)
    invariance_by_policy[policy] = policy_invariance
    summaries.append(summary)

  by_policy = {row["policy"]: row for row in summaries}
  baseline = by_policy["P0"]
  for row in summaries:
    row["pingpong_reduction"] = baseline["A_B_A"]-row["A_B_A"]
    row["short_dwell_reduction"] = sum(baseline.get(f"dwell_{n}_scan", 0) for n in (1, 2, 3))-sum(
      row.get(f"dwell_{n}_scan", 0) for n in (1, 2, 3))
    for coordinate in ("dRel", "yRel", "vRel"):
      base_value = baseline.get(f"jump_{coordinate}_p99")
      value = row.get(f"jump_{coordinate}_p99")
      row[f"jump_{coordinate}_p99_reduction"] = (None if base_value is None or value is None else base_value-value)

  eligible = [row for row in summaries if row["policy"] in STATEFUL_POLICIES and
              all(int(row[f"{field}_mismatch"]) == 0 for field in INVARIANT_FIELDS) and
              int(row.get("stale_selected_surface", 0)) == 0 and
              int(row.get("LEAD_GAIN", 0)) == 0 and int(row.get("LEAD_LOSS", 0)) == 0 and
              int(row.get("LEAD_ID_CHANGED", 0)) == 0 and
              float(row.get("route280_first_publication_delay_ms") or 0.0) == 0.0 and
              int(row["p0_publication_duration_regression_scans"]) == 0 and
              int(row["pingpong_reduction"]) > 0 and
              float(row.get("jump_dRel_p99_reduction") or 0.0) > 0.0 and
              float(row.get("jump_yRel_p99_reduction") or 0.0) >= 0.0 and
              float(row.get("jump_vRel_p99_reduction") or 0.0) >= 0.0]
  best = (max(eligible, key=lambda row: (
    int(row["pingpong_reduction"]), float(row.get("jump_dRel_p99_reduction") or 0.0),
    -int(row["publication_changed_scans"]))) if eligible else None)
  comparison_pool = [row for row in summaries if row["policy"] in ("P2", "P3", "P6") and
                     int(row.get("LEAD_GAIN", 0)) == 0 and int(row.get("LEAD_LOSS", 0)) == 0 and
                     int(row.get("LEAD_ID_CHANGED", 0)) == 0 and int(row["pingpong_reduction"]) > 0]
  comparison = (max(comparison_pool, key=lambda row: (
    int(row["pingpong_reduction"])/max(1, int(row["publication_changed_scans"])),
    -int(row.get("PLANNER_INPUT_CHANGED", 0)), -int(row["publication_changed_scans"])))
                if comparison_pool else None)

  write_csv(TABLES / "baseline_publication_surfaces.csv", [baseline])
  write_csv(TABLES / "candidate_surface_decisions.csv", decision_rows)
  write_csv(TABLES / "stable_group_events.csv", stable_event_rows)
  write_csv(TABLES / "pingpong_events.csv", pingpong_rows)
  write_csv(TABLES / "jump_events.csv", jump_rows)
  write_csv(TABLES / "surface_dwell.csv", dwell_rows)
  write_csv(TABLES / "publication_coordinate_delta.csv", divergence_rows)
  write_csv(TABLES / "radarstate_delta.csv", summaries, (
    "policy", "scans", "publication_changed_scans", "NO_DOWNSTREAM_EFFECT",
    "radarstate_changed_scans", "LEAD_COORDINATE_ONLY", "LEAD_ID_CHANGED",
    "LEAD_GAIN", "LEAD_LOSS", "LEAD1_LEAD2_SWAP"))
  write_csv(TABLES / "lead_delta.csv", summaries, (
    "policy", "radarstate_changed_scans", "LEAD_COORDINATE_ONLY", "LEAD_ID_CHANGED",
    "LEAD_GAIN", "LEAD_LOSS", "LEAD1_LEAD2_SWAP", "ALEAD_EXPOSURE_CHANGED"))
  write_csv(TABLES / "alead_delta.csv", summaries, (
    "policy", "surface_decisions_different", "vrel_consistency_large_delta",
    "ALEAD_EXPOSURE_CHANGED", "internal_aLead_mismatch"))
  write_csv(TABLES / "planner_input_delta.csv", summaries, (
    "policy", "publication_changed_scans", "radarstate_changed_scans", "PLANNER_INPUT_CHANGED"))
  write_csv(TABLES / "candidate_comparison.csv", summaries)
  write_csv(TABLES / "invariance_results.csv", invariant_rows)
  write_csv(TABLES / "cpu_state_summary.csv", cpu_rows)
  candidate_tail_rows.sort(key=lambda row: max(row["abs_delta_d"]/JUMP_P99[0],
                                               row["abs_delta_y"]/JUMP_P99[1],
                                               row["abs_delta_v"]/JUMP_P99[2]), reverse=True)
  write_csv(TRACES / "large_jump" / "candidate_tail_sample.csv", candidate_tail_rows[:300])
  write_csv(TRACES / "stable" / "stable_group_switch_sample.csv", stable_event_rows[:300])
  write_csv(TRACES / "pingpong" / "baseline_stable_a_b_a_sample.csv", baseline_pingpong_sample)
  write_csv(TRACES / "controls" / "route269_invariance.csv", [
    {"policy": policy, "tracking_representative_regression": int(route269_invariance[policy]["tracking_representative"]),
     "group_regression": int(route269_invariance[policy]["group"]),
     "pid_regression": int(route269_invariance[policy]["pid"])}
    for policy in POLICIES])
  write_csv(TRACES / "controls" / "route280_cutin.csv", [
    {"policy": policy, **(route280.get(policy) or {})} for policy in POLICIES])
  write_csv(TRACES / "controls" / "known_p0_duration.csv", [
    {"policy": policy, "positive_publication_duration_regression_scans": int(p0_duration[policy])}
    for policy in POLICIES])
  write_csv(TRACES / "controls" / "stable_multireturn.csv", [
    {"policy": policy, **dict(multi_return[policy])} for policy in POLICIES])
  write_json(MANIFESTS / "baseline_summary.json", baseline)
  manifest = {
    "routes": routes, "segments": len(keys), "completed_scans": completed_scans,
    "baseline_parity": dict(baseline_parity), "source_current_vs_P0": dict(source_current),
    "candidates": summaries, "best_candidate": None if best is None else best["policy"],
    "best_candidate_summary": best,
    "best_nonproduction_comparison": None if comparison is None else comparison["policy"],
    "frozen_same_controls": {
      "controls": 41, "surface_evaluable": 0,
      "reason": "prior frozen SAME anchors had zero matching grouped multi-return episodes",
      "group_regressions": 0,
    },
    "stable_multi_return_controls": {
      "controls": sum(len(rows) for rows in stable_control_windows().values()),
      "by_policy": {policy: dict(multi_return[policy]) for policy in POLICIES},
    },
  }
  write_json(MANIFESTS / "candidate_evaluation.json", manifest)
  write_json(MANIFESTS / "invariance_summary.json", {
    "baseline_parity": dict(baseline_parity),
    "candidates": {policy: {field: int(invariance_by_policy[policy][field]) for field in INVARIANT_FIELDS}
                   for policy in POLICIES},
  })
  return manifest


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--force", action="store_true")
  parser.add_argument("--workers", type=int)
  parser.add_argument("--limit", type=int)
  parser.add_argument("--key", action="append", default=[])
  parser.add_argument("--no-finalize", action="store_true")
  args = parser.parse_args()
  ensure_dirs()
  keys = args.key or corpus_keys()
  if not args.key and len(keys) != 847:
    raise RuntimeError(f"expected 847 corpus segments, found {len(keys)}")
  if args.limit:
    keys = keys[:args.limit]
  workers = args.workers or max(1, min(2, os.cpu_count() or 1))
  payloads = [(key, args.force) for key in keys]
  with mp.Pool(processes=workers, maxtasksperchild=10) as pool:
    for index, (key, scans, changed) in enumerate(pool.imap_unordered(_process, payloads, chunksize=1), 1):
      print(f"{index}/{len(payloads)} {key}: scans={scans} P6_changed={changed}", flush=True)
  if not args.key and not args.limit and not args.no_finalize:
    manifest = finalize()
    print({"best_candidate": manifest["best_candidate"], "scans": manifest["completed_scans"]})


if __name__ == "__main__":
  mp.freeze_support()
  main()
