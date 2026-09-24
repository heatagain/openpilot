"""Synthetic, control, artifact and final fail-closed validation."""
from __future__ import annotations

import csv
import json
from types import SimpleNamespace
from typing import Any

from evaluate_publication_candidates import INVARIANT_FIELDS, write_csv, write_json
from replay_post_assignment_surface import (MANIFESTS, POLICIES, PRIOR, STUDY, TABLES, TRACES,
                                            PublicationSurfaceSelector)


REQUIRED_TABLES = (
  "baseline_publication_surfaces.csv", "candidate_surface_decisions.csv", "stable_group_events.csv",
  "pingpong_events.csv", "jump_events.csv", "surface_dwell.csv",
  "publication_coordinate_delta.csv", "radarstate_delta.csv", "lead_delta.csv", "alead_delta.csv",
  "planner_input_delta.csv", "candidate_comparison.csv", "invariance_results.csv",
  "prefix_invariance.csv", "cpu_state_summary.csv", "synthetic_results.csv",
)


def _member(values, ns: int):
  raw, slot, d_rel, y_rel, v_rel, age = values
  return SimpleNamespace(raw_track_id=raw, slot=slot, d_rel=d_rel, y_rel=y_rel, v_rel=v_rel,
                         age_scans=age, timestamp_ns=ns, recovered=False)


def _object(pid: int, members, representative: int, ns: int):
  point = next(member for member in members if member.raw_track_id == representative)
  return SimpleNamespace(
    physical_track_id=pid, members=tuple(members), representative_raw_track_id=representative,
    timestamp_ns=ns, d_rel=point.d_rel, y_rel=point.y_rel, v_rel=point.v_rel,
  )


def _cases() -> dict[str, list[dict[str, Any]]]:
  a = (1, 1, 30.0, 0.0, 0.0, 20)
  b = (2, 2, 31.0, .1, 0.0, 20)
  b_near = (2, 2, 30.1, .1, 0.0, 21)
  b_strong = (2, 2, 25.0, .1, -1.0, 21)
  c = (3, 3, 30.5, -.1, .25, 10)
  return {
    "A_SINGLE_MEMBER_STABLE": [{"pid": 1, "members": (a,), "rep": 1}, {"pid": 1, "members": (a,), "rep": 1}],
    "B_TWO_MEMBER_REP_STABLE": [{"pid": 1, "members": (a, b), "rep": 1}, {"pid": 1, "members": (a, b), "rep": 1}],
    "C_ONE_SCAN_NEAR_TIE_SWITCH": [{"pid": 1, "members": (a, b), "rep": 1}, {"pid": 1, "members": (a, b_near), "rep": 2}],
    "D_A_B_A": [{"pid": 1, "members": (a, b), "rep": 1}, {"pid": 1, "members": (a, b_near), "rep": 2}, {"pid": 1, "members": (a, b), "rep": 1}],
    "E_OLD_SURFACE_DISAPPEARS": [{"pid": 1, "members": (a, b), "rep": 1}, {"pid": 1, "members": (b_near,), "rep": 2}],
    "F_MEMBER_SET_CHANGE": [{"pid": 1, "members": (a, b), "rep": 1}, {"pid": 1, "members": (a, b_near, c), "rep": 2}],
    "G_PID_CHANGE": [{"pid": 1, "members": (a, b), "rep": 1}, {"pid": 2, "members": (a, b_near), "rep": 2}],
    "H_NEW_REP_CAMERA_SUPPORT": [{"pid": 1, "members": (a, b), "rep": 1}, {"pid": 1, "members": (a, b_near), "rep": 2, "camera": 2}],
    "I_NEW_REP_OEM_SUPPORT": [{"pid": 1, "members": (a, b), "rep": 1}, {"pid": 1, "members": (a, b_near), "rep": 2, "oem": 2}],
    "J_LARGE_JUMP_WEAK": [
      {"pid": 1, "members": ((1, 1, 35.0, 0.0, 0.0, 20), (2, 2, 35.5, .1, 0.0, 20)), "rep": 1},
      {"pid": 1, "members": ((1, 1, 35.0, 0.0, 0.0, 21), b_near), "rep": 2},
    ],
    "K_LARGE_JUMP_STRONG": [{"pid": 1, "members": (a, b), "rep": 1}, {"pid": 1, "members": (a, b_strong), "rep": 2}],
    "L_THREE_MEMBER_TRUCK": [{"pid": 1, "members": (a, b, c), "rep": 1}, {"pid": 1, "members": (a, b, c), "rep": 2}],
    "M_INPUT_RESET": [{"pid": 1, "members": (a, b), "rep": 1}, {"pid": 1, "members": (a, b_near), "rep": 2, "gap_ns": 400_000_000}],
    "N_SEGMENT_RESET": [{"pid": 1, "members": (a, b), "rep": 1}, {"pid": 1, "members": (a, b_near), "rep": 2, "reset": True}],
    "O_CUTIN_NEW_MEMBER": [{"pid": 1, "members": (a,), "rep": 1}, {"pid": 1, "members": (a, b_strong), "rep": 2}],
  }


def run_synthetic() -> dict[str, Any]:
  rows = []
  for policy in POLICIES:
    for case, frames in _cases().items():
      selector = PublicationSurfaceSelector(policy)
      ns = 1_000_000_000
      prior_by_pid = {}
      previous_scan_ns = None
      for index, frame in enumerate(frames):
        if index:
          ns += int(frame.get("gap_ns", 100_000_000))
        if frame.get("reset"):
          selector.reset()
        members = [_member(values, ns) for values in frame["members"]]
        obj = _object(frame["pid"], members, frame["rep"], ns)
        cue = ()
        if frame.get("camera") is not None:
          selected = next(member for member in members if member.raw_track_id == frame["camera"])
          cue = (SimpleNamespace(d_rel=selected.d_rel, y_rel=selected.y_rel, probability=1.0,
                                 distance_tolerance_m=.01, lateral_tolerance_m=.01),)
        representative_before = obj.representative_raw_track_id
        chosen, metadata = selector.select(
          obj, prior_tracking=prior_by_pid.get(frame["pid"]), timestamp_ns=ns,
          previous_scan_ns=previous_scan_ns, yaw_rate=0.0, cues=cue, oem_slot=frame.get("oem"))
        selector.finish_scan({frame["pid"]}, {frame["pid"]})
        rows.append({
          "policy": policy, "case": case, "frame": index, "pid": frame["pid"],
          "tracking_representative": obj.representative_raw_track_id,
          "published_surface": chosen.raw_track_id, "members": "|".join(str(m.raw_track_id) for m in members),
          "tracking_unchanged": int(obj.representative_raw_track_id == representative_before),
          "stale": int(chosen.raw_track_id not in {m.raw_track_id for m in members}),
          "reason": metadata["reason"], "fresh": metadata["fresh"],
        })
        prior_by_pid = {frame["pid"]: obj}
        previous_scan_ns = ns
  write_csv(TABLES / "synthetic_results.csv", rows)
  summary = {
    "policies": len(POLICIES), "cases": len(_cases()), "rows": len(rows),
    "tracking_mismatch": sum(not row["tracking_unchanged"] for row in rows),
    "stale_surface": sum(row["stale"] for row in rows),
    "nonfresh_surface": sum(not row["fresh"] for row in rows),
  }
  write_json(MANIFESTS / "synthetic_summary.json", summary)
  return summary


def validate_route2bc() -> dict[str, Any]:
  manifest = json.loads((PRIOR / "manifests" / "route2bc_warm_control.json").read_text(encoding="utf-8"))
  trace_path = PRIOR / "traces" / "controls" / "route2bc_s21_stable_truck.csv"
  with trace_path.open(encoding="utf-8", newline="") as stream:
    trace = list(csv.DictReader(stream))
  if (manifest["baseline_scans"] != 11 or manifest["baseline_raw"] != [822] or
      manifest["baseline_pids"] != [1000845] or manifest["baseline_representatives"] != [822]):
    raise AssertionError(manifest)
  rows = []
  for policy in POLICIES:
    selector = PublicationSurfaceSelector(policy)
    prior = None
    previous_scan_ns = None
    regression = 0
    for source in trace:
      ns = int(source["scan_ns"])
      member = _member((822, 0, float(source["d"]), float(source["y"]), float(source["v"]), 1), ns)
      obj = _object(1000845, [member], 822, ns)
      chosen, _ = selector.select(obj, prior_tracking=prior, timestamp_ns=ns,
                                  previous_scan_ns=previous_scan_ns, yaw_rate=0.0, cues=(), oem_slot=None)
      selector.finish_scan({1000845}, {1000845})
      regression += int(chosen.raw_track_id != 822)
      prior, previous_scan_ns = obj, ns
    rows.append({"policy": policy, "scans": len(trace), "publication_regression": regression,
                 "tracking_regression": 0, "pid_regression": 0, "member_regression": 0})
  write_csv(TABLES / "route2bc_control.csv", rows)
  write_csv(TRACES / "controls" / "route2bc_s21_stable_truck.csv", trace)
  result = {"baseline": {"scans": len(trace), "raw": 822, "pid": 1000845, "representative": 822},
            "candidates": rows}
  write_json(MANIFESTS / "route2bc_control.json", result)
  return result


def main() -> None:
  candidate = json.loads((MANIFESTS / "candidate_evaluation.json").read_text(encoding="utf-8"))
  prefix = json.loads((MANIFESTS / "prefix_summary.json").read_text(encoding="utf-8"))
  invariance = json.loads((MANIFESTS / "invariance_validation.json").read_text(encoding="utf-8"))
  synthetic = run_synthetic()
  route2bc = validate_route2bc()
  failures = []
  if (candidate["routes"], candidate["segments"], candidate["completed_scans"]) != (34, 847, 504_673):
    failures.append("corpus coverage")
  if any(int(value) for value in candidate["baseline_parity"].values()):
    failures.append(f"baseline parity {candidate['baseline_parity']}")
  if invariance["status"] != "PASS":
    failures.append("invariance gate")
  by_policy = {row["policy"]: row for row in candidate["candidates"]}
  if set(by_policy) != set(POLICIES):
    failures.append("candidate set")
  for policy, row in by_policy.items():
    if int(row["segments"]) != 847 or int(row["scans"]) != 504_673:
      failures.append(f"{policy} coverage")
    if int(row["stale_selected_surface"]):
      failures.append(f"{policy} stale surface")
    if any(int(row[f"{field}_mismatch"]) for field in INVARIANT_FIELDS):
      failures.append(f"{policy} invariant")
  if int(prefix["failed"]):
    failures.append("prefix")
  if synthetic["tracking_mismatch"] or synthetic["stale_surface"] or synthetic["nonfresh_surface"]:
    failures.append("synthetic")
  if any(row["publication_regression"] for row in route2bc["candidates"]):
    failures.append("Route2bc")
  missing = [name for name in REQUIRED_TABLES if not (TABLES / name).is_file()]
  failures.extend(f"missing {name}" for name in missing)
  result = {
    "status": "PASS" if not failures else "FAIL", "failures": failures,
    "corpus": {"routes": candidate["routes"], "segments": candidate["segments"],
               "scans": candidate["completed_scans"]},
    "candidate_set": list(POLICIES), "prefix": prefix, "synthetic": synthetic,
    "route2bc": route2bc, "required_tables": len(REQUIRED_TABLES), "missing_tables": missing,
    "report_present": (STUDY / "report.md").is_file(),
  }
  write_json(MANIFESTS / "validation_results.json", result)
  if failures:
    raise AssertionError(result)
  print(result)


if __name__ == "__main__":
  main()
