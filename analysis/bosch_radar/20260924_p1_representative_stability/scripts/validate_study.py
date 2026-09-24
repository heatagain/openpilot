# ruff: noqa: TID251
"""Fail closed on corpus integrity and required study artifacts."""
from __future__ import annotations

from common import MANIFESTS, STUDY, TABLES, read_json, write_json


REQUIRED_TABLES = (
  "representative_events.csv", "stable_group_rep_changes.csv", "representative_pingpong.csv",
  "representative_dwell.csv", "representative_cost_breakdown.csv",
  "representative_jump_distribution.csv", "publication_coordinate_changes.csv",
  "publication_duration_effect.csv", "radarstate_effects.csv", "lead_selection_effects.csv",
  "alead_effects.csv", "candidate_e_baseline.csv", "candidate_refinement_results.csv",
  "regression_summary.csv", "prefix_invariance.csv", "cpu_summary.csv",
)


def main() -> None:
  baseline = read_json(MANIFESTS / "baseline_representative_summary.json")
  candidates = read_json(MANIFESTS / "candidate_evaluation.json")
  prefix = read_json(MANIFESTS / "prefix_summary.json")
  synthetic = read_json(MANIFESTS / "synthetic_summary.json")
  route2bc = read_json(MANIFESTS / "route2bc_warm_control.json")
  expected = {
    "routes": 34, "segments": 847, "completed_scans": 504_673,
    "representative_changes": 58_505, "stable_group_representative_changes": 26_574,
  }
  failures = [f"baseline {field}: {baseline.get(field)} != {value}"
              for field, value in expected.items() if baseline.get(field) != value]
  rows = {row["policy"]: row for row in candidates["candidates"]}
  if set(rows) != {"E", "E1", "E2", "E3", "E4", "E5"}:
    failures.append(f"candidate set: {sorted(rows)}")
  for policy, row in rows.items():
    if int(row["segments"]) != 847 or int(row["scans"]) != 504_673:
      failures.append(f"{policy} corpus coverage")
    if int(row["stale_representative"]) != 0:
      failures.append(f"{policy} stale representative")
  e = rows.get("E", {})
  reproduction = {
    "candidate_rep_changes_expected": 50_553,
    "candidate_rep_changes_actual": e.get("candidate_rep_changes"),
    "legacy_publication_changed_expected": 80_770,
    "legacy_publication_changed_actual": e.get("legacy_p1_publication_changed_scans"),
  }
  missing = [name for name in REQUIRED_TABLES if not (TABLES / name).is_file()]
  failures.extend(f"missing table {name}" for name in missing)
  if int(prefix["failed"]) != 0:
    failures.append(f"prefix failures: {prefix['failed']}")
  if int(synthetic["stale_representatives"]) != 0 or int(synthetic["cases"]) != 13:
    failures.append("synthetic gate")
  if (route2bc["baseline_scans"] != 11 or route2bc["baseline_raw"] != [822] or
      route2bc["baseline_pids"] != [1000845] or route2bc["baseline_representatives"] != [822]):
    failures.append(f"Route2bc warm baseline: {route2bc}")
  if any(int(row["scans"]) != 11 or int(row["target_missing_scans"]) or
         int(row["target_representative_changes"]) for row in route2bc["candidates"]):
    failures.append("Route2bc warm candidate regression")
  result = {
    "status": "PASS" if not failures else "FAIL", "failures": failures,
    "baseline_expected": expected, "candidate_e_reproduction": reproduction,
    "candidate_e_exact_reproduction": (
      reproduction["candidate_rep_changes_actual"] == reproduction["candidate_rep_changes_expected"] and
      reproduction["legacy_publication_changed_actual"] == reproduction["legacy_publication_changed_expected"]),
    "prefix": prefix, "synthetic": synthetic,
    "route2bc_warm_control": route2bc,
    "required_tables": len(REQUIRED_TABLES), "missing_tables": missing,
    "report_present": (STUDY / "report.md").is_file(),
  }
  write_json(MANIFESTS / "validation_results.json", result)
  if failures:
    raise AssertionError(result)
  print(result)


if __name__ == "__main__":
  main()
