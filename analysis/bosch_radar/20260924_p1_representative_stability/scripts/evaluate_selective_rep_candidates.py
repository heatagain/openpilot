# ruff: noqa: TID251
"""Replay Candidate E and selective E1-E5 policies over the full corpus."""
from __future__ import annotations

import argparse
import multiprocessing as mp
from collections import Counter

from common import (MANIFESTS, P0_CONTROLS, ROUTE2BC_S21_KEY, ROUTE2BC_WINDOW_NS, ROUTE269_KEY,
                    ROUTE280_S15_KEY, TABLES, corpus_keys, ensure_dirs, percentile,
                    process_count, read_json, read_json_gz, segment_result_path,
                    write_csv, write_json, write_json_gz)
from replay_core import load_segment, run_policy


POLICIES = ("E", "E1", "E2", "E3", "E4", "E5")


def _policy_config() -> dict:
  summary = read_json(MANIFESTS / "baseline_representative_summary.json")
  threshold = float(summary["derived_selective_threshold"])
  jumps = summary["stable_jump_distribution"]
  return {
    "BOSCH_P1_REP_THRESHOLD": threshold,
    "BOSCH_P1_REP_HARD_THRESHOLD": .25,
    "BOSCH_P1_REP_CONFIRM_SCANS": 2,
    "BOSCH_P1_REP_PINGPONG_NS": 2_000_000_000,
    "BOSCH_P1_REP_JUMP_D": float(jumps["dRel"]["p95"]),
    "BOSCH_P1_REP_JUMP_Y": float(jumps["yRel"]["p95"]),
    "BOSCH_P1_REP_JUMP_V": float(jumps["vRel"]["p95"]),
  }


def _obj_by_pid(snapshot: dict) -> dict[int, dict]:
  return {int(row["pid"]): row for row in snapshot["objects"]}


def _pub_by_pid(snapshot: dict) -> dict[int, dict]:
  return {int(row["pid"]): row for row in snapshot["published"]}


def _owner(snapshot: dict) -> dict[int, int]:
  return {int(raw): int(row["pid"]) for row in snapshot["objects"] for raw in row["members"]}


def _partition(snapshot: dict) -> tuple:
  return tuple(sorted(tuple(row["members"]) for row in snapshot["objects"]))


def _pid_members(snapshot: dict) -> tuple:
  return tuple(sorted((int(row["pid"]), tuple(row["members"])) for row in snapshot["objects"]))


def _publication_signature(snapshot: dict, *, include_acceleration: bool = False) -> tuple:
  fields = ("pid", "alias", "published_raw", "d", "y", "v", "a") if include_acceleration else (
    "pid", "alias", "published_raw", "d", "y", "v")
  return tuple(tuple(row[field] for field in fields) for row in snapshot["published"])


def _legacy_p1_signature(snapshot: dict) -> tuple:
  published = _pub_by_pid(snapshot)
  return tuple(sorted(
    (tuple(obj["members"]), int(obj["representative"]), float(obj["d"]), float(obj["y"]),
     float(obj["v"]), int(published[obj["pid"]]["alias"]))
    for obj in snapshot["objects"] if int(obj["pid"]) in published
  ))


def _rep_changes(snapshots: list[dict]) -> int:
  total = 0
  for previous, current in zip(snapshots, snapshots[1:], strict=False):
    old = _obj_by_pid(previous)
    total += sum(pid in old and old[pid]["representative"] != row["representative"]
                 for pid, row in _obj_by_pid(current).items())
  return total


def _radar_signature(snapshot: dict, *, acceleration: bool = True) -> tuple:
  result = []
  fields = ("status", "track", "d", "y", "v", "a", "d_path", "radar", "model_prob")
  if not acceleration:
    fields = tuple(field for field in fields if field != "a")
  for name in ("lead_one", "lead_two"):
    lead = snapshot["radar"][name]
    result.append(tuple(lead[field] for field in fields))
  return tuple(result)


def _radar_effect(base: dict, candidate: dict) -> str:
  old = (base["radar"]["lead_one"], base["radar"]["lead_two"])
  new = (candidate["radar"]["lead_one"], candidate["radar"]["lead_two"])
  old_ids = tuple(lead["track"] if lead["status"] else -1 for lead in old)
  new_ids = tuple(lead["track"] if lead["status"] else -1 for lead in new)
  if old_ids == new_ids:
    if any(a["a"] != b["a"] for a, b in zip(old, new, strict=True)):
      return "A_LEAD_CHANGED"
    return "LEAD_COORDINATE_ONLY"
  old_set, new_set = {value for value in old_ids if value >= 0}, {value for value in new_ids if value >= 0}
  if len(new_set-old_set) and not len(old_set-new_set):
    return "LEAD_GAIN"
  if len(old_set-new_set) and not len(new_set-old_set):
    return "LEAD_LOSS"
  if old_ids == tuple(reversed(new_ids)) and old_set == new_set:
    return "LEAD1_LEAD2_CHANGE"
  return "LEAD_ID_CHANGED"


def _episodes(changed: list[bool], snapshots: list[dict]) -> list[dict]:
  rows = []
  start = None
  for index, value in enumerate(changed + [False]):
    if value and start is None:
      start = index
    elif not value and start is not None:
      end = index-1
      rows.append({"start_scan": start, "end_scan": end, "scans": end-start+1,
                   "duration_s": (snapshots[end]["scan_ns"]-snapshots[start]["scan_ns"])*1e-9})
      start = None
  return rows


def compare_runs(key: str, baseline: dict, candidate: dict, baseline_analysis: dict) -> dict:
  base = baseline["snapshots"]
  current = candidate["snapshots"]
  if len(base) != len(current):
    raise AssertionError(f"{key} {candidate['policy']}: scan count mismatch")
  policy = candidate["policy"]
  grouping_mismatch = pid_mismatch = member_mismatch = stale = 0
  publication_changed = legacy_publication_changed = publication_set_changed = 0
  alias_changed = a_lead_point_changed = radar_changed = planner_input_changed = 0
  radar_effects = Counter()
  publication_flags = []
  radar_rows, alead_rows = [], []
  for index, (old, new) in enumerate(zip(base, current, strict=True)):
    grouping_mismatch += _partition(old) != _partition(new)
    pid_mismatch += _owner(old) != _owner(new)
    member_mismatch += _pid_members(old) != _pid_members(new)
    stale += sum(obj["representative"] not in obj["members"] for obj in new["objects"])
    old_pub, new_pub = _pub_by_pid(old), _pub_by_pid(new)
    coordinate_changed = _publication_signature(old) != _publication_signature(new)
    publication_flags.append(coordinate_changed)
    publication_changed += coordinate_changed
    legacy_publication_changed += _legacy_p1_signature(old) != _legacy_p1_signature(new)
    publication_set_changed += set(old_pub) != set(new_pub)
    alias_changed += tuple(sorted((pid, row["alias"]) for pid, row in old_pub.items())) != tuple(
      sorted((pid, row["alias"]) for pid, row in new_pub.items()))
    common = set(old_pub).intersection(new_pub)
    changed_a = [pid for pid in common if old_pub[pid]["a"] != new_pub[pid]["a"]]
    if changed_a:
      a_lead_point_changed += 1
      alead_rows.append({"key": key, "policy": policy, "scan": index, "scan_ns": new["scan_ns"],
                         "pids": "|".join(map(str, sorted(changed_a)))})
    if _radar_signature(old) != _radar_signature(new):
      effect = _radar_effect(old, new)
      radar_effects[effect] += 1
      radar_changed += 1
      planner_input_changed += 1
      radar_rows.append({"key": key, "policy": policy, "scan": index, "scan_ns": new["scan_ns"],
                         "effect": effect,
                         "baseline_lead1": old["radar"]["lead_one"]["track"],
                         "candidate_lead1": new["radar"]["lead_one"]["track"],
                         "baseline_lead2": old["radar"]["lead_two"]["track"],
                         "candidate_lead2": new["radar"]["lead_two"]["track"]})

  base_event_keys = {(int(row["scan"]), int(row["pid"])): row for row in baseline_analysis["events"]}
  suppressed = []
  for (scan, pid), event in base_event_keys.items():
    if scan <= 0 or scan >= len(current):
      continue
    previous, now = _obj_by_pid(current[scan-1]), _obj_by_pid(current[scan])
    if pid in previous and pid in now and previous[pid]["representative"] == now[pid]["representative"]:
      suppressed.append({"event_id": event["event_id"], "scan": scan, "pid": pid,
                         "category": event["category"], "abs_delta_d": event["abs_delta_d"],
                         "abs_delta_y": event["abs_delta_y"], "abs_delta_v": event["abs_delta_v"]})
  pingpong_targets = {(int(row["end_scan"]), int(row["pid"]))
                      for row in baseline_analysis["pingpongs"] if row["pattern"] == "A_B_A"}
  pingpong_suppressed = sum((int(row["scan"]), int(row["pid"])) in pingpong_targets for row in suppressed)
  divergence = _episodes(publication_flags, current)
  for row in divergence:
    row.update({"key": key, "policy": policy})
  for row in suppressed:
    episode = next((ep for ep in divergence if ep["start_scan"] <= row["scan"] <= ep["end_scan"]), None)
    row["publication_divergence_scans"] = episode["scans"] if episode else 0
    row["publication_divergence_duration_s"] = episode["duration_s"] if episode else 0.0
    row.update({"key": key, "policy": policy})

  cutin_delay_ms = None
  cutin_target_coordinate_changed_scans = None
  if key == ROUTE280_S15_KEY:
    target_pid = 1000004
    base_first = next((row["scan_ns"] for row in base if target_pid in _pub_by_pid(row)), None)
    new_first = next((row["scan_ns"] for row in current if target_pid in _pub_by_pid(row)), None)
    if base_first is not None and new_first is not None:
      cutin_delay_ms = (new_first-base_first) * 1e-6
    cutin_target_coordinate_changed_scans = sum(
      _pub_by_pid(old).get(target_pid) != _pub_by_pid(new).get(target_pid)
      for old, new in zip(base, current, strict=True))
  route2bc_window_scans = None
  route2bc_window_regression_scans = None
  route2bc_window_rep_changes = None
  if key == ROUTE2BC_S21_KEY:
    start_ns, end_ns = ROUTE2BC_WINDOW_NS
    pairs = [(old, new) for old, new in zip(base, current, strict=True)
             if start_ns <= int(old["scan_ns"]) <= end_ns]
    route2bc_window_scans = len(pairs)
    route2bc_window_regression_scans = sum(
      (_pid_members(old), _publication_signature(old)) !=
      (_pid_members(new), _publication_signature(new)) for old, new in pairs)
    target_pid = 1000845
    target_reps = [(_obj_by_pid(new).get(target_pid) or {}).get("representative") for _, new in pairs]
    route2bc_window_rep_changes = sum(
      first is not None and second is not None and first != second
      for first, second in zip(target_reps, target_reps[1:], strict=False))
  p0_duration_delta = None
  if key in P0_CONTROLS:
    target_pid = P0_CONTROLS[key]
    p0_duration_delta = sum(target_pid in _pub_by_pid(row) for row in current) - sum(
      target_pid in _pub_by_pid(row) for row in base)

  summary = {
    "key": key, "policy": policy, "scans": len(current),
    "baseline_rep_changes": len(baseline_analysis["events"]),
    "candidate_rep_changes": _rep_changes(current), "suppressed_transitions": len(suppressed),
    "suppressed_pingpong_returns": pingpong_suppressed,
    "grouping_mismatch": grouping_mismatch, "pid_mismatch": pid_mismatch,
    "member_mismatch": member_mismatch, "stale_representative": stale,
    "publication_changed_scans": publication_changed,
    "legacy_p1_publication_changed_scans": legacy_publication_changed,
    "publication_set_changed_scans": publication_set_changed,
    "alias_changed_scans": alias_changed, "aLead_changed_scans": a_lead_point_changed,
    "radarstate_changed_scans": radar_changed, "planner_input_changed_scans": planner_input_changed,
    "lead_coordinate_only": radar_effects["LEAD_COORDINATE_ONLY"],
    "lead_id_changed": radar_effects["LEAD_ID_CHANGED"], "lead_gain": radar_effects["LEAD_GAIN"],
    "lead_loss": radar_effects["LEAD_LOSS"], "lead1_lead2_change": radar_effects["LEAD1_LEAD2_CHANGE"],
    "lead_aLead_changed": radar_effects["A_LEAD_CHANGED"],
    "state_peak": candidate["state_peak"], "cutin_delay_ms": cutin_delay_ms,
    "cutin_target_coordinate_changed_scans": cutin_target_coordinate_changed_scans,
    "route2bc_window_scans": route2bc_window_scans,
    "route2bc_window_regression_scans": route2bc_window_regression_scans,
    "route2bc_window_rep_changes": route2bc_window_rep_changes,
    "p0_publication_duration_delta_scans": p0_duration_delta,
  }
  return {"summary": summary, "suppressed": suppressed, "divergence": divergence,
          "radar_rows": radar_rows, "alead_rows": alead_rows,
          "timings_us": candidate["timings_us"]}


_CONFIG: dict | None = None


def _process(payload: tuple[str, bool, bool]) -> tuple[str, int]:
  key, force, corpus_member = payload
  phase = "candidates" if corpus_member else "candidate_controls"
  destination = segment_result_path(key, phase)
  if destination.exists() and not force:
    return key, -1
  baseline_phase = "baseline" if corpus_member else "controls"
  baseline_data = read_json_gz(segment_result_path(key, baseline_phase))
  context = load_segment(key)
  config = _policy_config()
  results = {}
  # Rotate the full-path policy order by segment so warm-cache/order effects are
  # distributed instead of always favoring the same candidate.
  offset = sum(key.encode("ascii")) % len(POLICIES)
  policy_order = POLICIES[offset:] + POLICIES[:offset]
  for policy in policy_order:
    run = run_policy(key, context, policy, config=config, downstream=True)
    results[policy] = compare_runs(key, baseline_data["run"], run, baseline_data["analysis"])
  write_json_gz(destination, {"key": key, "policies": results})
  return key, sum(result["summary"]["publication_changed_scans"] for result in results.values())


def _sum(rows: list[dict], field: str) -> int:
  return sum(int(row[field]) for row in rows if row.get(field) is not None)


def finalize() -> dict:
  baseline_summary = read_json(MANIFESTS / "baseline_representative_summary.json")
  stable_tail = baseline_summary["stable_jump_distribution"]
  all_results = []
  suppressed, divergence = [], []
  for key in corpus_keys():
    data = read_json_gz(segment_result_path(key, "candidates"))
    for policy in POLICIES:
      result = data["policies"][policy]
      all_results.append(result["summary"])
      suppressed.extend(result["suppressed"])
      divergence.extend(result["divergence"])
  control_data = read_json_gz(segment_result_path(ROUTE2BC_S21_KEY, "candidate_controls"))
  control_rows = [control_data["policies"][policy]["summary"] for policy in POLICIES]
  warm_path = MANIFESTS / "route2bc_warm_control.json"
  warm_rows = ({row["policy"]: row for row in read_json(warm_path)["candidates"]}
               if warm_path.exists() else {})
  summaries = []
  for policy in POLICIES:
    rows = [row for row in all_results if row["policy"] == policy]
    control = next(row for row in control_rows if row["policy"] == policy)
    warm = warm_rows.get(policy)
    summary = {"policy": policy, "segments": len(rows), "scans": _sum(rows, "scans")}
    for field in (
      "baseline_rep_changes", "candidate_rep_changes", "suppressed_transitions",
      "suppressed_pingpong_returns", "grouping_mismatch", "pid_mismatch", "member_mismatch",
      "stale_representative", "publication_changed_scans", "legacy_p1_publication_changed_scans",
      "publication_set_changed_scans", "alias_changed_scans", "aLead_changed_scans",
      "radarstate_changed_scans", "planner_input_changed_scans", "lead_coordinate_only",
      "lead_id_changed", "lead_gain", "lead_loss", "lead1_lead2_change", "lead_aLead_changed",
    ):
      summary[field] = _sum(rows, field)
    summary["state_peak"] = max(int(row["state_peak"]) for row in rows)
    summary["route269_grouping_regression"] = next(row["grouping_mismatch"] for row in rows if row["key"] == ROUTE269_KEY)
    summary["route2bc_grouping_regression"] = (
      warm["grouping_mismatch_scans"] if warm is not None else control["grouping_mismatch"])
    summary["route2bc_pid_regression"] = (
      warm["pid_mismatch_scans"] if warm is not None else control["pid_mismatch"])
    summary["route2bc_member_regression"] = (
      warm["member_mismatch_scans"] if warm is not None else control["member_mismatch"])
    summary["route2bc_publication_changed_scans"] = control["publication_changed_scans"]
    summary["route2bc_window_scans"] = warm["scans"] if warm is not None else control["route2bc_window_scans"]
    summary["route2bc_window_regression_scans"] = (
      warm["target_regression_scans"] if warm is not None else control["route2bc_window_regression_scans"])
    summary["route2bc_window_rep_changes"] = (
      warm["target_representative_changes"] if warm is not None else control["route2bc_window_rep_changes"])
    summary["route280_cutin_delay_ms"] = next(row["cutin_delay_ms"] for row in rows if row["key"] == ROUTE280_S15_KEY)
    summary["route280_target_coordinate_changed_scans"] = next(
      row["cutin_target_coordinate_changed_scans"] for row in rows if row["key"] == ROUTE280_S15_KEY)
    summary["p0_publication_duration_regression_scans"] = sum(
      max(0, int(row["p0_publication_duration_delta_scans"] or 0)) for row in rows
      if row["key"] in P0_CONTROLS)
    summaries.append(summary)

  # Exact full-scan timing quantiles, one policy at a time to cap memory.
  baseline_values = []
  baseline_state_peak = 0
  for key in corpus_keys():
    baseline = read_json_gz(segment_result_path(key, "baseline"))["run"]
    baseline_values.extend(float(value) for value in baseline["timings_us"])
    baseline_state_peak = max(baseline_state_peak, int(baseline["state_peak"]))
  baseline_cpu = {
    "policy": "BASELINE", "segments": len(corpus_keys()),
    "scans": int(baseline_summary["completed_scans"]),
    "update_calls": len(baseline_values),
    "cpu_mean_us": sum(baseline_values)/len(baseline_values),
    "cpu_p50_us": percentile(baseline_values, .50),
    "cpu_p95_us": percentile(baseline_values, .95),
    "cpu_p99_us": percentile(baseline_values, .99),
    "cpu_max_us": max(baseline_values), "cpu_wall_s": sum(baseline_values)*1e-6,
    "state_peak": baseline_state_peak,
  }
  for summary in summaries:
    values = []
    policy = summary["policy"]
    for key in corpus_keys():
      data = read_json_gz(segment_result_path(key, "candidates"))
      values.extend(float(value) for value in data["policies"][policy]["timings_us"])
    summary.update({
      "update_calls": len(values),
      "cpu_mean_us": sum(values)/len(values), "cpu_p50_us": percentile(values, .50),
      "cpu_p95_us": percentile(values, .95), "cpu_p99_us": percentile(values, .99),
      "cpu_max_us": max(values), "cpu_wall_s": sum(values)*1e-6,
    })
    policy_suppressed = [row for row in suppressed if row["policy"] == policy]
    summary["suppressed_large_tail_events"] = sum(
      float(row["abs_delta_d"]) >= float(stable_tail["dRel"]["p99"]) or
      float(row["abs_delta_y"]) >= float(stable_tail["yRel"]["p99"]) or
      float(row["abs_delta_v"]) >= float(stable_tail["vRel"]["p99"])
      for row in policy_suppressed)
    divergence_scans = [float(row["publication_divergence_scans"]) for row in policy_suppressed]
    divergence_duration = [float(row["publication_divergence_duration_s"]) for row in policy_suppressed]
    for label, values_for_distribution in (
        ("publication_divergence_scans", divergence_scans),
        ("publication_divergence_duration_s", divergence_duration)):
      summary.update({
        f"{label}_p50": percentile(values_for_distribution, .50),
        f"{label}_p90": percentile(values_for_distribution, .90),
        f"{label}_p95": percentile(values_for_distribution, .95),
        f"{label}_p99": percentile(values_for_distribution, .99),
        f"{label}_max": max(values_for_distribution) if values_for_distribution else None,
      })

  eligible = [row for row in summaries if row["policy"] in ("E1", "E2", "E3", "E4")
              and all(int(row[field]) == 0 for field in (
                "grouping_mismatch", "pid_mismatch", "member_mismatch", "stale_representative",
                "publication_set_changed_scans", "alias_changed_scans", "lead_gain", "lead_loss"))
              and float(row["route280_cutin_delay_ms"] or 0.0) == 0.0
              and int(row["route2bc_grouping_regression"]) == 0
              and int(row["route2bc_pid_regression"]) == 0
              and int(row["route2bc_member_regression"]) == 0
              and int(row["route2bc_window_regression_scans"]) == 0
              and int(row["p0_publication_duration_regression_scans"]) == 0]
  best = (max(eligible, key=lambda row: (int(row["suppressed_pingpong_returns"]),
                                         -int(row["publication_changed_scans"])))
          if eligible else None)
  write_csv(TABLES / "candidate_refinement_results.csv", summaries)
  write_csv(TABLES / "candidate_e_baseline.csv", [row for row in all_results if row["policy"] == "E"])
  write_csv(TABLES / "publication_duration_effect.csv", [row for row in suppressed if row["policy"] == "E"])
  write_csv(TABLES / "publication_coordinate_changes.csv", divergence)
  write_csv(TABLES / "radarstate_effects.csv", all_results, (
    "key", "policy", "scans", "radarstate_changed_scans", "planner_input_changed_scans",
    "lead_coordinate_only", "lead_id_changed", "lead_gain", "lead_loss",
    "lead1_lead2_change", "lead_aLead_changed"))
  write_csv(TABLES / "lead_selection_effects.csv", summaries, (
    "policy", "segments", "scans", "radarstate_changed_scans", "lead_coordinate_only",
    "lead_id_changed", "lead_gain", "lead_loss", "lead1_lead2_change"))
  write_csv(TABLES / "alead_effects.csv", all_results, (
    "key", "policy", "scans", "aLead_changed_scans", "lead_aLead_changed"))
  write_csv(TABLES / "regression_summary.csv", summaries)
  write_csv(TABLES / "cpu_summary.csv", [baseline_cpu, *summaries],
            ["policy", "segments", "scans", "update_calls", "cpu_mean_us", "cpu_p50_us", "cpu_p95_us",
             "cpu_p99_us", "cpu_max_us", "cpu_wall_s", "state_peak"])
  manifest = {"config": _policy_config(), "baseline_cpu": baseline_cpu, "candidates": summaries,
              "best_candidate": best["policy"] if best else None,
              "best_candidate_summary": best}
  write_json(MANIFESTS / "candidate_evaluation.json", manifest)
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
  if args.limit:
    keys = keys[:args.limit]
  payloads = [(key, args.force, True) for key in keys]
  workers = args.workers or process_count(2)
  with mp.Pool(processes=workers, maxtasksperchild=20) as pool:
    for index, (key, changed) in enumerate(pool.imap_unordered(_process, payloads, chunksize=1), 1):
      print(f"{index}/{len(payloads)} {key}: publication_changed_sum={changed}", flush=True)
  if not args.key and not args.limit:
    _process((ROUTE2BC_S21_KEY, args.force, False))
    if not args.no_finalize:
      print(finalize())


if __name__ == "__main__":
  mp.freeze_support()
  main()
