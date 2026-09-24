# ruff: noqa: TID251
"""Export small, reviewable traces from the per-segment replay caches."""
from __future__ import annotations

from common import (MANIFESTS, P0_CONTROLS, ROUTE2BC_S21_KEY, ROUTE2BC_WINDOW_NS,
                    ROUTE269_KEY, ROUTE280_S15_KEY, TABLES, TRACES, corpus_keys,
                    read_csv, read_json, read_json_gz, segment_result_path, write_csv)


def _by_pid(snapshot: dict, field: str) -> dict[int, dict]:
  return {int(row["pid"]): row for row in snapshot[field]}


def export_baseline() -> None:
  events = read_csv(TABLES / "representative_events.csv")
  stable = [row for row in events if row["category"] == "R1_STABLE_GROUP_REP_CHANGE"]
  pingpong = read_csv(TABLES / "representative_pingpong.csv")
  summary = read_json(MANIFESTS / "baseline_representative_summary.json")
  tail = summary["stable_jump_distribution"]
  large = [row for row in stable if (
    float(row["abs_delta_d"]) >= float(tail["dRel"]["p99"]) or
    float(row["abs_delta_y"]) >= float(tail["yRel"]["p99"]) or
    float(row["abs_delta_v"]) >= float(tail["vRel"]["p99"]))]
  large.sort(key=lambda row: max(
    float(row["abs_delta_d"])/max(float(tail["dRel"]["p99"]), 1e-12),
    float(row["abs_delta_y"])/max(float(tail["yRel"]["p99"]), 1e-12),
    float(row["abs_delta_v"])/max(float(tail["vRel"]["p99"]), 1e-12)), reverse=True)
  downstream = [row for row in events if int(row["radarstate_affected"])]
  stable_pingpong = [row for row in pingpong if row["pattern"] == "A_B_A" and
                     int(row["stable_members"]) and float(row["duration_s"]) <= 2.0]
  write_csv(TRACES / "stable_group" / "stable_group_sample.csv", stable[:100])
  write_csv(TRACES / "large_jump" / "stable_r1_p99_tail.csv", large[:200])
  write_csv(TRACES / "downstream" / "baseline_temporal_exposure_sample.csv", downstream[:200])
  write_csv(TRACES / "pingpong" / "stable_a_b_a_sample.csv", stable_pingpong[:200])

  route269 = [row for row in events if row["key"] == ROUTE269_KEY]
  write_csv(TRACES / "controls" / "route269_representative_events.csv", route269)

  # The cold S21 control cache has a different synthetic raw/PID namespace.
  # Preserve the authoritative S20->S21 warm trace when it has been generated.
  if not (MANIFESTS / "route2bc_warm_control.json").exists():
    route2bc = read_json_gz(segment_result_path(ROUTE2BC_S21_KEY, "controls"))["run"]["snapshots"]
    start_ns, end_ns = ROUTE2BC_WINDOW_NS
    control_rows = []
    for snapshot in route2bc:
      if not start_ns <= int(snapshot["scan_ns"]) <= end_ns:
        continue
      obj = _by_pid(snapshot, "objects").get(1000845)
      pub = _by_pid(snapshot, "published").get(1000845)
      control_rows.append({
        "scan": snapshot["scan"], "scan_ns": snapshot["scan_ns"],
        "pid": 1000845, "members": "|".join(map(str, obj["members"])) if obj else "",
        "representative": obj["representative"] if obj else "",
        "d": obj["d"] if obj else "", "y": obj["y"] if obj else "", "v": obj["v"] if obj else "",
        "published": int(pub is not None), "alias": pub["alias"] if pub else "",
      })
    write_csv(TRACES / "controls" / "route2bc_s21_stable_truck.csv", control_rows)

  route280 = read_json_gz(segment_result_path(ROUTE280_S15_KEY, "baseline"))["run"]["snapshots"]
  cutin_rows = []
  for snapshot in route280:
    pub = _by_pid(snapshot, "published").get(1000004)
    if pub is not None:
      cutin_rows.append({"scan": snapshot["scan"], "scan_ns": snapshot["scan_ns"], **pub})
  write_csv(TRACES / "controls" / "route280_s15_pid1000004.csv", cutin_rows)

  p0_rows = []
  for key, pid in P0_CONTROLS.items():
    run = read_json_gz(segment_result_path(key, "baseline"))["run"]["snapshots"]
    indexes = [int(row["scan"]) for row in run if pid in _by_pid(row, "published")]
    p0_rows.append({"key": key, "pid": pid, "published_scans": len(indexes),
                    "first_scan": indexes[0] if indexes else "", "last_scan": indexes[-1] if indexes else ""})
  write_csv(TRACES / "controls" / "known_p0_publication_duration.csv", p0_rows)


def export_candidates() -> None:
  if not (MANIFESTS / "candidate_evaluation.json").exists():
    return
  rows = []
  downstream_rows = []
  alead_rows = []
  for key in corpus_keys():
    policies = read_json_gz(segment_result_path(key, "candidates"))["policies"]
    result = policies["E"]
    rows.extend(result["suppressed"])
    if len(downstream_rows) < 250:
      downstream_rows.extend(result["radar_rows"][:250-len(downstream_rows)])
    if len(alead_rows) < 250:
      alead_rows.extend(result["alead_rows"][:250-len(alead_rows)])
  rows.sort(key=lambda row: (int(row["publication_divergence_scans"]),
                             float(row["publication_divergence_duration_s"])), reverse=True)
  write_csv(TRACES / "candidate_e" / "largest_publication_blast_radius.csv", rows[:250])
  write_csv(TRACES / "downstream" / "candidate_e_radarstate_sample.csv", downstream_rows)
  write_csv(TRACES / "candidate_e" / "alead_persistence_sample.csv", alead_rows)


def main() -> None:
  export_baseline()
  export_candidates()


if __name__ == "__main__":
  main()
