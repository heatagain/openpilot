# ruff: noqa: TID251
"""Classify baseline representative events, dwell runs, and oscillation patterns."""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict

from common import (MANIFESTS, TABLES, corpus_keys, ensure_dirs, percentile,
                    read_json_gz, segment_result_path, write_csv, write_json)


def _by_pid(snapshot: dict, field: str = "objects") -> dict[int, dict]:
  return {int(row["pid"]): row for row in snapshot[field]}


def _raw_owner(snapshot: dict) -> dict[int, int]:
  return {int(raw): int(obj["pid"]) for obj in snapshot["objects"] for raw in obj["members"]}


def _lead_changes(previous: dict, current: dict, alias: int | None) -> tuple[str, int]:
  if alias is None:
    return "NO_DOWNSTREAM_EFFECT", 0
  prior_leads = (previous["radar"]["lead_one"], previous["radar"]["lead_two"])
  current_leads = (current["radar"]["lead_one"], current["radar"]["lead_two"])
  prior_ids = tuple(lead["track"] if lead["status"] else -1 for lead in prior_leads)
  current_ids = tuple(lead["track"] if lead["status"] else -1 for lead in current_leads)
  if alias not in prior_ids and alias not in current_ids:
    return "NO_DOWNSTREAM_EFFECT", 0
  if alias not in prior_ids and alias in current_ids:
    return "LEAD_GAIN", 1
  if alias in prior_ids and alias not in current_ids:
    return "LEAD_LOSS", 1
  if prior_ids == current_ids:
    slot = current_ids.index(alias)
    old, new = prior_leads[slot], current_leads[slot]
    if old["a"] != new["a"]:
      return "A_LEAD_CHANGED", 1
    if any(old[field] != new[field] for field in ("d", "y", "v", "d_path")):
      return "LEAD_COORDINATE_ONLY", 1
    return "LEAD_SELECTED_NO_FIELD_CHANGE", 1
  if prior_ids == tuple(reversed(current_ids)) and set(prior_ids) == set(current_ids):
    return "LEAD1_LEAD2_CHANGE", 1
  return "LEAD_ID_CHANGED", 1


def _event_category(previous: dict, current: dict, old: dict, new: dict) -> str:
  old_members = set(old["members"])
  new_members = set(new["members"])
  prior_owner = _raw_owner(previous)
  current_owner = _raw_owner(current)
  split = len({current_owner[raw] for raw in old_members if raw in current_owner}) > 1
  merge = len({prior_owner[raw] for raw in new_members if raw in prior_owner}) > 1
  if split:
    return "R4_GROUP_SPLIT_REP_CHANGE"
  if merge:
    return "R5_GROUP_MERGE_REP_CHANGE"
  if old["representative"] not in new_members:
    return "R2_MEMBER_DISAPPEAR_REP_CHANGE"
  if new_members-old_members:
    return "R3_MEMBER_ADD_REP_CHANGE"
  if old_members == new_members:
    return "R1_STABLE_GROUP_REP_CHANGE"
  return "R6_PUBLICATION_REENTRY_REP_CHANGE" if not _by_pid(previous, "published").get(old["pid"]) else "R7_RESET_BOUNDARY"


def analyze_baseline_run(run: dict) -> dict:
  key = run["key"]
  snapshots = run["snapshots"]
  trace_by_key = {(int(row["scan_ns"]), int(row["pid"])): row for row in run["selection_trace"]
                  if row["pid"] is not None}
  events = []
  rep_runs: dict[int, list[dict]] = defaultdict(list)
  active: dict[int, dict] = {}

  for index, current in enumerate(snapshots):
    current_by_pid = _by_pid(current)
    current_pids = set(current_by_pid)
    for pid in list(active):
      if pid not in current_pids:
        rep_runs[pid].append(active.pop(pid))
    for pid, obj in current_by_pid.items():
      existing = active.get(pid)
      members = tuple(obj["members"])
      if existing is None:
        active[pid] = {"key": key, "pid": pid, "representative": obj["representative"],
                       "members": "|".join(map(str, members)), "start_scan": index, "end_scan": index,
                       "start_ns": current["scan_ns"], "end_ns": current["scan_ns"], "scans": 1}
      elif existing["representative"] == obj["representative"]:
        existing["end_scan"] = index
        existing["end_ns"] = current["scan_ns"]
        existing["scans"] += 1
      else:
        rep_runs[pid].append(existing)
        active[pid] = {"key": key, "pid": pid, "representative": obj["representative"],
                       "members": "|".join(map(str, members)), "start_scan": index, "end_scan": index,
                       "start_ns": current["scan_ns"], "end_ns": current["scan_ns"], "scans": 1}

    if index == 0:
      continue
    previous = snapshots[index-1]
    previous_by_pid = _by_pid(previous)
    old_published, new_published = _by_pid(previous, "published"), _by_pid(current, "published")
    for pid, new in current_by_pid.items():
      old = previous_by_pid.get(pid)
      if old is None or old["representative"] == new["representative"]:
        continue
      category = _event_category(previous, current, old, new)
      old_pub, new_pub = old_published.get(pid), new_published.get(pid)
      if old_pub is None and new_pub is None:
        publication_effect = "P0_INTERNAL_ONLY"
      elif old_pub is not None and new_pub is not None:
        same_coordinate = all(old_pub[field] == new_pub[field] for field in ("d", "y", "v", "a"))
        publication_effect = "P1_PUBLISHED_SAME_COORDINATE" if same_coordinate else "P2_PUBLISHED_COORDINATE_CHANGE"
      elif old_pub is not None:
        publication_effect = "P4_PUBLICATION_GAP"
      else:
        publication_effect = "P5_NEW_PUBLIC_POINT"
      alias = new_pub["alias"] if new_pub is not None else old_pub["alias"] if old_pub is not None else None
      downstream, affects = _lead_changes(previous, current, alias)
      trace = trace_by_key.get((int(current["scan_ns"]), pid), {})
      events.append({
        "event_id": f"{key}:{index}:{pid}", "key": key, "scan": index,
        "scan_ns": int(current["scan_ns"]), "pid": pid, "category": category,
        "old_members": "|".join(map(str, old["members"])),
        "new_members": "|".join(map(str, new["members"])),
        "old_representative": old["representative"], "new_representative": new["representative"],
        "old_rep_still_member": int(old["representative"] in new["members"]),
        "required_change": int(old["representative"] not in new["members"]),
        "delta_d": new["d"]-old["d"], "delta_y": new["y"]-old["y"], "delta_v": new["v"]-old["v"],
        "abs_delta_d": abs(new["d"]-old["d"]), "abs_delta_y": abs(new["y"]-old["y"]),
        "abs_delta_v": abs(new["v"]-old["v"]),
        "old_published": int(old_pub is not None), "new_published": int(new_pub is not None),
        "old_alias": old_pub["alias"] if old_pub is not None else "",
        "new_alias": new_pub["alias"] if new_pub is not None else "",
        "publication_effect": publication_effect,
        "publication_delta_d": (new_pub["d"]-old_pub["d"] if old_pub is not None and new_pub is not None else ""),
        "publication_delta_y": (new_pub["y"]-old_pub["y"] if old_pub is not None and new_pub is not None else ""),
        "publication_delta_v": (new_pub["v"]-old_pub["v"] if old_pub is not None and new_pub is not None else ""),
        "publication_delta_aLead": ((new_pub["a"] or 0.0)-(old_pub["a"] or 0.0)
                                    if old_pub is not None and new_pub is not None else ""),
        "radarstate_effect": downstream, "radarstate_affected": affects,
        "score_delta": trace.get("score_delta", ""), "old_cost": trace.get("prior_cost", ""),
        "new_cost": trace.get("best_cost", ""), "best_vision": int(bool(trace.get("best_vision", False))),
        "old_vision": int(bool(trace.get("prior_vision", False))),
        "best_oem": int(bool(trace.get("best_oem", False))), "old_oem": int(bool(trace.get("prior_oem", False))),
        "support_superiority": int(bool(trace.get("support_superiority", False))),
      })

  for pid, row in active.items():
    rep_runs[pid].append(row)
  dwells = []
  pingpongs = []
  for pid, runs in rep_runs.items():
    for row in runs:
      row = dict(row)
      row["duration_s"] = (row["end_ns"]-row["start_ns"]) * 1e-9
      dwells.append(row)
    for index in range(len(runs)-2):
      a, b, c = runs[index:index+3]
      if a["representative"] == c["representative"] and a["representative"] != b["representative"]:
        duration_s = (c["start_ns"]-a["end_ns"]) * 1e-9
        pingpongs.append({
          "key": key, "pid": pid, "pattern": "A_B_A", "start_scan": a["end_scan"],
          "end_scan": c["start_scan"], "duration_s": duration_s,
          "representatives": f"{a['representative']}|{b['representative']}|{c['representative']}",
          "stable_members": int(a["members"] == b["members"] == c["members"]),
          "member_sets": f"{a['members']};{b['members']};{c['members']}",
        })
    for index in range(len(runs)-3):
      a, b, c, d = runs[index:index+4]
      if a["representative"] == c["representative"] and b["representative"] == d["representative"] and a["representative"] != b["representative"]:
        pattern = "A_B_A_B"
      elif a["representative"] == d["representative"] and len({a["representative"], b["representative"], c["representative"]}) == 3:
        pattern = "A_B_C_A"
      else:
        continue
      pingpongs.append({
        "key": key, "pid": pid, "pattern": pattern, "start_scan": a["end_scan"],
        "end_scan": d["start_scan"], "duration_s": (d["start_ns"]-a["end_ns"]) * 1e-9,
        "representatives": "|".join(str(row["representative"]) for row in (a, b, c, d)),
        "stable_members": int(len({row["members"] for row in (a, b, c, d)}) == 1),
        "member_sets": ";".join(row["members"] for row in (a, b, c, d)),
      })
  return {"events": events, "dwells": dwells, "pingpongs": pingpongs}


def _distribution(rows: list[dict], field: str) -> dict:
  values = [float(row[field]) for row in rows if row.get(field) not in (None, "")]
  return {"n": len(values), "p50": percentile(values, .50), "p75": percentile(values, .75),
          "p90": percentile(values, .90), "p95": percentile(values, .95),
          "p99": percentile(values, .99), "max": max(values) if values else None}


def _stable_member_dwells(run: dict) -> list[dict]:
  """Representative tenure split whenever the current member set changes."""
  active: dict[int, dict] = {}
  rows = []
  for index, snapshot in enumerate(run["snapshots"]):
    current = _by_pid(snapshot)
    for pid in tuple(active):
      if pid not in current:
        rows.append(active.pop(pid))
    for pid, obj in current.items():
      members = tuple(obj["members"])
      prior = active.get(pid)
      if (prior is None or prior["representative"] != obj["representative"] or
          prior["members"] != "|".join(map(str, members))):
        if prior is not None:
          rows.append(prior)
        active[pid] = {
          "key": run["key"], "pid": pid, "representative": obj["representative"],
          "members": "|".join(map(str, members)), "start_scan": index, "end_scan": index,
          "start_ns": snapshot["scan_ns"], "end_ns": snapshot["scan_ns"], "scans": 1,
          "population": "STABLE_MEMBERSHIP",
        }
      else:
        prior["end_scan"] = index
        prior["end_ns"] = snapshot["scan_ns"]
        prior["scans"] += 1
  rows.extend(active.values())
  for row in rows:
    row["duration_s"] = (row["end_ns"]-row["start_ns"]) * 1e-9
  return rows


def finalize(*, phase: str = "baseline") -> dict:
  ensure_dirs()
  events, dwells, pingpongs = [], [], []
  scans = 0
  for key in corpus_keys():
    path = segment_result_path(key, phase)
    if not path.exists():
      raise FileNotFoundError(path)
    result = read_json_gz(path)
    scans += len(result["run"]["snapshots"])
    analysis = result["analysis"]
    dwell_end = {(int(row["pid"]), int(row["end_scan"])): row for row in analysis["dwells"]}
    snapshots = result["run"]["snapshots"]
    enriched_events = []
    for source_event in analysis["events"]:
      row = dict(source_event)
      index, pid = int(row["scan"]), int(row["pid"])
      old = _by_pid(snapshots[index-1]).get(pid)
      new = _by_pid(snapshots[index]).get(pid)
      prior_dwell = dwell_end.get((pid, index-1))
      if old is not None and new is not None:
        row.update({
          "group_size_before": len(old["members"]), "group_size_after": len(new["members"]),
          "old_d": old["d"], "old_y": old["y"], "old_v": old["v"], "old_age": old["age"],
          "new_d": new["d"], "new_y": new["y"], "new_v": new["v"], "new_age": new["age"],
          "old_aRel": "N/A_NAN", "new_aRel": "N/A_NAN",
        })
        current_members = {int(member["raw"]): member for member in new["member_rows"]}
        old_member = current_members.get(int(row["old_representative"]))
        new_member = current_members.get(int(row["new_representative"]))
        for label, member in (("old_member_current", old_member), ("new_member_current", new_member)):
          for field in ("d", "y", "v", "age", "slot", "measurement_ns"):
            row[f"{label}_{field}"] = member[field] if member is not None else ""
      old_pub = _by_pid(snapshots[index-1], "published").get(pid)
      new_pub = _by_pid(snapshots[index], "published").get(pid)
      row["old_published_aLead"] = old_pub["a"] if old_pub else ""
      row["new_published_aLead"] = new_pub["a"] if new_pub else ""
      old_alias = old_pub["alias"] if old_pub else None
      new_alias = new_pub["alias"] if new_pub else None
      old_roles = [name for name in ("lead_one", "lead_two")
                   if old_alias is not None and snapshots[index-1]["radar"][name]["status"] and
                   snapshots[index-1]["radar"][name]["track"] == old_alias]
      new_roles = [name for name in ("lead_one", "lead_two")
                   if new_alias is not None and snapshots[index]["radar"][name]["status"] and
                   snapshots[index]["radar"][name]["track"] == new_alias]
      row["lead_role_before"] = "|".join(old_roles)
      row["lead_role_after"] = "|".join(new_roles)
      row["previous_rep_dwell_scans"] = int(prior_dwell["scans"]) if prior_dwell else ""
      row["previous_rep_dwell_s"] = ((int(prior_dwell["end_ns"])-int(prior_dwell["start_ns"]))*1e-9
                                      if prior_dwell else "")
      enriched_events.append(row)
    events.extend(enriched_events)
    dwells.extend({**row, "population": "ALL_REPRESENTATIVE_TENURE"}
                  for row in analysis["dwells"])
    dwells.extend(_stable_member_dwells(result["run"]))
    pingpongs.extend(analysis["pingpongs"])
  events.sort(key=lambda row: (row["key"], int(row["scan"]), int(row["pid"])))
  dwells.sort(key=lambda row: (row["key"], int(row["start_scan"]), int(row["pid"])))
  pingpongs.sort(key=lambda row: (row["key"], int(row["start_scan"]), int(row["pid"]), row["pattern"]))
  stable = [row for row in events if row["category"] == "R1_STABLE_GROUP_REP_CHANGE"]
  write_csv(TABLES / "representative_events.csv", events)
  write_csv(TABLES / "stable_group_rep_changes.csv", stable)
  dwell_buckets: dict[tuple[str, int], list[float]] = defaultdict(list)
  for row in dwells:
    dwell_buckets[(row["population"], int(row["scans"]))].append(float(row["duration_s"]))
  write_csv(TABLES / "representative_dwell.csv", [
    {"population": population, "tenure_scans": tenure_scans, "count": len(values),
     "duration_p50_s": percentile(values, .50), "duration_p90_s": percentile(values, .90),
     "duration_p95_s": percentile(values, .95), "duration_p99_s": percentile(values, .99),
     "duration_max_s": max(values)}
    for (population, tenure_scans), values in sorted(dwell_buckets.items())
  ])
  write_csv(TABLES / "representative_pingpong.csv", pingpongs)
  cost_buckets = Counter(round(float(row["score_delta"]), 6) for row in stable
                         if row["score_delta"] not in (None, ""))
  write_csv(TABLES / "representative_cost_breakdown.csv", [
    {"score_delta_rounded_1e6": value, "count": count,
     "within_0_25": int(value <= .25 + 1e-12)}
    for value, count in sorted(cost_buckets.items())
  ])
  write_csv(TABLES / "representative_jump_distribution.csv", [
    {"population": population, "coordinate": coordinate, **_distribution(rows, field)}
    for population, rows in (("ALL", events), ("R1_STABLE_GROUP", stable))
    for coordinate, field in (("dRel", "abs_delta_d"), ("yRel", "abs_delta_y"), ("vRel", "abs_delta_v"))
  ])
  category_counts = Counter(row["category"] for row in events)
  effect_counts = Counter(row["publication_effect"] for row in events)
  downstream_counts = Counter(row["radarstate_effect"] for row in events)
  stable_scores = [row for row in stable if row["score_delta"] not in (None, "")]
  score_levels = Counter(round(float(row["score_delta"]), 9) for row in stable_scores)
  score_distribution = _distribution(stable_scores, "score_delta")
  stable_jump_distribution = {
    "dRel": _distribution(stable, "abs_delta_d"),
    "yRel": _distribution(stable, "abs_delta_y"),
    "vRel": _distribution(stable, "abs_delta_v"),
  }
  pingpong_event_ids = {(row["key"], int(row["pid"]), int(row["end_scan"])) for row in pingpongs if row["pattern"] == "A_B_A"}
  pingpong_scores = [row for row in stable if (row["key"], int(row["pid"]), int(row["scan"])) in pingpong_event_ids
                     and row["score_delta"] not in (None, "")]
  pingpong_score_distribution = _distribution(pingpong_scores, "score_delta")
  derived_threshold = pingpong_score_distribution["p75"]
  if derived_threshold is None:
    derived_threshold = score_distribution["p50"] or .25
  derived_threshold = min(.25, max(0.0, float(derived_threshold)))
  all_dwells = [row for row in dwells if row["population"] == "ALL_REPRESENTATIVE_TENURE"]
  stable_dwells = [row for row in dwells if row["population"] == "STABLE_MEMBERSHIP"]
  summary = {
    "routes": len({key.split("--", 1)[0] for key in corpus_keys()}),
    "segments": len(corpus_keys()), "completed_scans": scans,
    "representative_changes": len(events), "stable_group_representative_changes": len(stable),
    "required_old_rep_disappeared": sum(int(row["required_change"]) for row in events),
    "optional_old_rep_valid": sum(not int(row["required_change"]) for row in events),
    "categories": dict(sorted(category_counts.items())),
    "publication_effects": dict(sorted(effect_counts.items())),
    "publication_visible_rep_changes": sum(row["publication_effect"] != "P0_INTERNAL_ONLY" for row in events),
    "publication_gaps": effect_counts["P4_PUBLICATION_GAP"],
    "alias_changes": sum(row["old_alias"] not in (None, "") and row["new_alias"] not in (None, "") and
                         row["old_alias"] != row["new_alias"] for row in events),
    "downstream_temporal_effects": dict(sorted(downstream_counts.items())),
    "short_dwell": {str(count): sum(int(row["scans"]) == count for row in all_dwells) for count in (1, 2, 3)},
    "stable_membership_short_dwell": {
      str(count): sum(int(row["scans"]) == count for row in stable_dwells) for count in (1, 2, 3)},
    "dwell_distribution": {
      "all": _distribution(all_dwells, "duration_s"),
      "stable_membership": _distribution(stable_dwells, "duration_s"),
    },
    "pingpong": {str(window): sum(row["pattern"] == "A_B_A" and float(row["duration_s"]) <= window for row in pingpongs)
                 for window in (.2, .5, 1.0, 2.0)},
    "stable_membership_pingpong": {
      str(window): sum(row["pattern"] == "A_B_A" and int(row["stable_members"]) and
                       float(row["duration_s"]) <= window for row in pingpongs)
      for window in (.2, .5, 1.0, 2.0)},
    "pingpong_patterns": dict(sorted(Counter(row["pattern"] for row in pingpongs).items())),
    "stable_score_delta": score_distribution,
    "score_delta_min_positive": min((value for value in score_levels if value > 0.0), default=None),
    "score_delta_common_levels": [
      {"value": value, "count": count} for value, count in
      sorted(score_levels.items(), key=lambda item: (-item[1], item[0]))[:20]],
    "stable_jump_distribution": stable_jump_distribution,
    "pingpong_score_delta": pingpong_score_distribution,
    "derived_selective_threshold": derived_threshold,
    "near_tie_count": sum(float(row["score_delta"]) <= derived_threshold + 1e-12 for row in stable_scores),
  }
  write_json(MANIFESTS / "baseline_representative_summary.json", summary)
  return summary


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--phase", default="baseline")
  args = parser.parse_args()
  print(finalize(phase=args.phase))


if __name__ == "__main__":
  main()
