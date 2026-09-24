# ruff: noqa: TID251
"""Replay Route2bc S20..S21 continuously and validate the 11-scan truck control.

The trusted control's synthetic raw/PID namespace is defined by the S20 warm-up
horizon.  Starting earlier preserves geometry but advances the synthetic identity
counters, so it is not an exact identifier-level reproduction of this control.
"""
from __future__ import annotations

from common import (MANIFESTS, ROUTE2BC_WINDOW_NS, TABLES, TRACES, read_json,
                    write_csv, write_json)
from replay_core import (_latest, _object_dict, _published_point, load_segment)
from representative_modules import load_policy_module


ROUTE = "000002bc--30683d78c9"
SEGMENTS = (20, 21)
POLICIES = ("BASELINE", "E", "E1", "E2", "E3", "E4", "E5")
CONTEXT_MAX_AGE_NS = 500_000_000


def _config() -> dict:
  summary = read_json(MANIFESTS / "baseline_representative_summary.json")
  jumps = summary["stable_jump_distribution"]
  return {
    "BOSCH_P1_REP_THRESHOLD": float(summary["derived_selective_threshold"]),
    "BOSCH_P1_REP_HARD_THRESHOLD": .25,
    "BOSCH_P1_REP_CONFIRM_SCANS": 2,
    "BOSCH_P1_REP_PINGPONG_NS": 2_000_000_000,
    "BOSCH_P1_REP_JUMP_D": float(jumps["dRel"]["p95"]),
    "BOSCH_P1_REP_JUMP_Y": float(jumps["yRel"]["p95"]),
    "BOSCH_P1_REP_JUMP_V": float(jumps["vRel"]["p95"]),
  }


def _run(policy: str, config: dict) -> list[dict]:
  module = load_policy_module(policy, name=f"route2bc_warm_{policy.lower()}", config=config)
  from opendbc.car.carlog import carlog, researchlog
  module.carlog, module.researchlog = carlog, researchlog
  provider = module.BoschRadarProvider(1, qualification=True)
  estimator = module.BoschLeadAccelerationEstimator()
  selected = []
  last_now = 0
  start_ns, end_ns = ROUTE2BC_WINDOW_NS
  for segment in SEGMENTS:
    context = load_segment(f"{ROUTE}--{segment}")
    can_times = [item[0] for item in context.can]
    pose_times = [item[0] for item in context.poses]
    model_times = [item[0] for item in context.models]
    cursor = 0
    for state_ns, v_ego, _a_ego, _steering in context.states:
      batch = []
      while cursor < len(context.can) and can_times[cursor] <= state_ns:
        batch.append(context.can[cursor])
        cursor += 1
      now_ns = max(state_ns, batch[-1][0] if batch else 0, last_now)
      last_now = now_ns
      pose = _latest(pose_times, context.poses, now_ns)
      yaw = None
      if pose is not None and 0 <= now_ns-pose[0] <= CONTEXT_MAX_AGE_NS and pose[2]:
        yaw = -pose[1]
      model = _latest(model_times, context.models, now_ns)
      active_model = model if model is not None and 0 <= now_ns-model[0] <= CONTEXT_MAX_AGE_NS else None
      cues, path, model_ns, source_ns = (), (), 0, 0
      if active_model is not None:
        model_ns = active_model[0]
        if active_model[1] is not None:
          lead = active_model[1]
          cues = (module.BoschVisionCue(lead["x"] - 1.52, -lead["y"], lead["prob"]),)
        path = tuple(zip(active_model[2][0], active_model[2][1], strict=True))
        source_ns = active_model[3]
      flat = [(timestamp, [(address, data, source) for address, data, source in messages])
              for timestamp, messages in batch]
      qualified = provider.update(flat, now_ns=now_ns, v_ego=v_ego, yaw_rate_left=yaw,
                                  vision=cues, path=path, path_ns=model_ns or None,
                                  path_source_ns=source_ns)
      if qualified is None:
        continue
      scan_ns = int(provider.last_scan_timestamp_ns)
      alias = provider.publication_aliases.update(
        now_ns, (obj.physical_track_id for obj in qualified), provider.tracker.group_manager.states)
      acceleration = estimator.update(qualified, scan_ns, v_ego, provider.tracker.group_manager.states)
      published = provider.publication_view(qualified, now_ns)
      if not start_ns <= scan_ns <= end_ns:
        continue
      published_rows = [_published_point(module, obj, alias, acceleration, now_ns)[0] for obj in published]
      selected.append({"scan_ns": scan_ns, "objects": [_object_dict(obj) for obj in qualified],
                       "published": published_rows})
    module.BOSCH_P1_REP_TRACE.clear()
  return selected


def _owner(snapshot: dict) -> dict[int, int]:
  return {int(raw): int(obj["pid"]) for obj in snapshot["objects"] for raw in obj["members"]}


def _pid_members(snapshot: dict) -> tuple:
  return tuple(sorted((int(obj["pid"]), tuple(obj["members"])) for obj in snapshot["objects"]))


def _target(snapshot: dict) -> dict | None:
  return next((obj for obj in snapshot["objects"] if 822 in obj["members"]), None)


def main() -> None:
  config = _config()
  runs = {policy: _run(policy, config) for policy in POLICIES}
  baseline = runs["BASELINE"]
  if len(baseline) != 11:
    raise AssertionError(f"expected 11 control scans, got {len(baseline)}")
  trace = []
  for index, snapshot in enumerate(baseline):
    obj = _target(snapshot)
    pub = next((row for row in snapshot["published"] if obj is not None and row["pid"] == obj["pid"]), None)
    trace.append({"scan": index, "scan_ns": snapshot["scan_ns"],
                  "pid": obj["pid"] if obj else "", "members": "|".join(map(str, obj["members"])) if obj else "",
                  "representative": obj["representative"] if obj else "",
                  "d": obj["d"] if obj else "", "y": obj["y"] if obj else "", "v": obj["v"] if obj else "",
                  "published": int(pub is not None), "alias": pub["alias"] if pub else ""})
  write_csv(TRACES / "controls" / "route2bc_s21_stable_truck.csv", trace)
  rows = []
  for policy in POLICIES[1:]:
    current = runs[policy]
    rows.append({
      "policy": policy, "scans": len(current),
      "grouping_mismatch_scans": sum(
        tuple(sorted(tuple(o["members"]) for o in old["objects"])) !=
        tuple(sorted(tuple(o["members"]) for o in new["objects"]))
        for old, new in zip(baseline, current, strict=True)),
      "pid_mismatch_scans": sum(_owner(old) != _owner(new)
                                for old, new in zip(baseline, current, strict=True)),
      "member_mismatch_scans": sum(_pid_members(old) != _pid_members(new)
                                   for old, new in zip(baseline, current, strict=True)),
      "target_missing_scans": sum(_target(new) is None for new in current),
      "target_regression_scans": sum(_target(old) != _target(new)
                                     for old, new in zip(baseline, current, strict=True)),
      "target_representative_changes": sum(
        _target(first) is not None and _target(second) is not None and
        _target(first)["representative"] != _target(second)["representative"]
        for first, second in zip(current, current[1:], strict=False)),
    })
  write_csv(TABLES / "route2bc_warm_control.csv", rows)
  manifest = {
    "segments": list(SEGMENTS), "window_ns": list(ROUTE2BC_WINDOW_NS),
    "baseline_scans": len(baseline), "baseline_raw": sorted({raw for row in baseline for raw in (_target(row) or {"members": ()})["members"]}),
    "baseline_pids": sorted({_target(row)["pid"] for row in baseline if _target(row)}),
    "baseline_representatives": sorted({_target(row)["representative"] for row in baseline if _target(row)}),
    "candidates": rows,
  }
  write_json(MANIFESTS / "route2bc_warm_control.json", manifest)
  print(manifest)


if __name__ == "__main__":
  main()
