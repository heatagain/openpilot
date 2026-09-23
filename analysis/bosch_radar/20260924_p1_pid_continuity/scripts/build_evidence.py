#!/usr/bin/env python3
"""Build the compact, reviewable evidence bundle for the 2026-09-24 P1 study.

The source studies and route logs remain read-only.  This script deliberately does
not run Git, mutate provider state, or infer SAME/DIFFERENT labels from mechanics.
"""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path


STUDY = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[4]
WORKSPACE = REPO.parent
EXTERNAL = WORKSPACE / "analysis"

PROVIDER = REPO / "opendbc_repo/opendbc/car/hyundai/radar_interface.py"
GROUP = EXTERNAL / "20260923_MRRevo14F_group_owner_oscillation"
STATIC_GT = EXTERNAL / "20260923_MRRevo14F_opus55_endpoint_adjudication"
MOVING = EXTERNAL / "20260924_MRRevo14F_moving_identity_shadow"
SIDEPASS = EXTERNAL / "bosch_radar/20260923_sidepass_reflection_migration"


def rows(path: Path) -> list[dict[str, str]]:
  with path.open(newline="", encoding="utf-8-sig") as f:
    return list(csv.DictReader(f))


def write_csv(path: Path, fieldnames: list[str], data: list[dict[str, object]]) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  with path.open("w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()
    writer.writerows(data)


def sha256(path: Path) -> str:
  h = hashlib.sha256()
  with path.open("rb") as f:
    for block in iter(lambda: f.read(1024 * 1024), b""):
      h.update(block)
  return h.hexdigest()


def normalized_sha256(path: Path) -> str:
  data = path.read_bytes().replace(b"\r\n", b"\n")
  return hashlib.sha256(data).hexdigest()


def unique(values: list[str]) -> str:
  return "|".join(sorted(set(values), key=lambda value: int(value) if value.isdigit() else value))


def main() -> None:
  group_transitions_path = GROUP / "tables/group_owner_transitions.csv"
  group_timeline_path = GROUP / "tables/route269_raw482_timeline.csv"
  group_same_path = GROUP / "tables/same_transition_controls.csv"
  group_different_path = GROUP / "tables/different_transition_controls.csv"
  cutin_path = GROUP / "tables/cutin_controls.csv"
  prevalence_path = GROUP / "tables/whole_corpus_prevalence.json"
  static_different_path = STATIC_GT / "tables/sequential_different_gt.csv"
  moving_different_path = MOVING / "tables/moving_sequential_different_gt.csv"
  moving_same_path = MOVING / "tables/moving_sequential_same_gt.csv"
  moving_gate_path = MOVING / "tables/shadow_safety_summary.csv"
  sidepass_history_path = SIDEPASS / "tables/route2bc_lateral_history.csv"
  sidepass_manifest_path = SIDEPASS / "manifests/regression_manifest.json"
  sidepass_prefix_path = SIDEPASS / "tables/prefix_invariance.csv"

  transitions = rows(group_transitions_path)
  exact_splits = [row for row in transitions if row["from_members"] == "436|482" and row["to_members"] == "482" and row["from_pid"] != row["to_pid"]]
  rejoins = [row for row in transitions if row["from_members"] == "482" and row["to_members"] == "436|482" and int(row["scan_index"]) > 400]
  same = rows(group_same_path)
  simultaneous_different = rows(group_different_path)
  cutin = rows(cutin_path)
  static_different = rows(static_different_path)
  moving_different = rows(moving_different_path)
  moving_same = rows(moving_same_path)
  prevalence = json.loads(prevalence_path.read_text(encoding="utf-8"))

  sidepass_rows = rows(sidepass_history_path)
  route2bc = [row for row in sidepass_rows if row["segment"] == "21" and 20.8 <= float(row["time_s"]) <= 21.9]
  published = [bool(row["public_track_id"]) for row in route2bc]
  publication_transitions = sum(a != b for a, b in zip(published, published[1:], strict=False))
  route2bc_compact = [
    {
      key: row[key]
      for key in (
        "segment",
        "time_s",
        "scan_ns",
        "raw_slot",
        "raw_track_id",
        "public_track_id",
        "physical_pid",
        "members",
        "representative",
        "dRel",
        "yRel",
        "vRel",
        "representative_changed",
        "member_set_changed",
      )
    }
    for row in route2bc
  ]
  write_csv(
    STUDY / "traces/route2bc_s21_identity_window.csv",
    list(route2bc_compact[0]),
    route2bc_compact,
  )

  route2bc_summary = [
    {
      "event": "route2bc_S21_truck_side_body",
      "window_s": "20.8..21.9",
      "scans": len(route2bc),
      "raw_ids": unique([row["raw_track_id"] for row in route2bc]),
      "physical_pids": unique([row["physical_pid"] for row in route2bc]),
      "member_sets": unique([row["members"] for row in route2bc]),
      "representatives": unique([row["representative"] for row in route2bc]),
      "representative_changes": sum(int(row["representative_changed"] or 0) for row in route2bc),
      "member_set_changes": sum(int(row["member_set_changed"] or 0) for row in route2bc),
      "publication_state_transitions": publication_transitions,
      "interpretation": "P1_IDENTITY_STABLE_P0_INTERACTION_ONLY",
    }
  ]
  write_csv(STUDY / "tables/route2bc_summary.csv", list(route2bc_summary[0]), route2bc_summary)

  denominators = [
    {
      "denominator": "current-remap sequential SAME",
      "count": len(same),
      "routes": len({r["route"] for r in same}),
      "grade": "CONFIRMED VIDEO GT",
      "use": "continuity positive",
    },
    {
      "denominator": "exact O1 SAME raw482",
      "count": sum(r["subtype"] == "SAME_RAW_SURFACE" for r in same),
      "routes": 1,
      "grade": "CONFIRMED VIDEO GT",
      "use": "Route269 mechanism positive",
    },
    {
      "denominator": "sequential DIFFERENT total",
      "count": len(static_different),
      "routes": len({r["route"] for r in static_different}),
      "grade": "VIDEO GT",
      "use": "static/parked hard negative",
    },
    {
      "denominator": "strict-order sequential DIFFERENT",
      "count": sum(r["candidate_id"] != "VQ004" for r in static_different),
      "routes": 2,
      "grade": "VIDEO GT",
      "use": "strict hard negative",
    },
    {
      "denominator": "moving-to-moving sequential DIFFERENT",
      "count": len(moving_different),
      "routes": len({r["route"] for r in moving_different}),
      "grade": "EMPTY DENOMINATOR",
      "use": "moving stitch safety gate",
    },
    {
      "denominator": "moving-to-moving sequential SAME",
      "count": len(moving_same),
      "routes": len({r["route"] for r in moving_same}),
      "grade": "PROBABLE SAME VIDEO+RADAR",
      "use": "moving positive; not a DIFFERENT safety denominator",
    },
    {
      "denominator": "simultaneous DIFFERENT vehicle pairs",
      "count": len(simultaneous_different),
      "routes": len({r["route"] for r in simultaneous_different}),
      "grade": "VIDEO GT",
      "use": "concurrent false-merge controls; not sequential stitch GT",
    },
  ]
  write_csv(STUDY / "tables/gt_denominators.csv", list(denominators[0]), denominators)

  candidates = [
    {
      "candidate": "A_TIMEOUT_EXTENSION",
      "idea": "extend raw/physical coast for short-gap reacquisition",
      "root_case_capture": "0/11",
      "causal_result": "Route269 raw482 was observed continuously for 255 scans; no gap to bridge",
      "safety_result": "NOT_RUN",
      "decision": "REJECTED_MECHANISM_MISS",
    },
    {
      "candidate": "B_WIDEN_COMPLETE_LINK",
      "idea": "widen distance/lateral diameter or growth gates",
      "root_case_capture": "11/11 HYPOTHETICAL",
      "causal_result": "would suppress the recorded boundary splits but changes grouping globally",
      "safety_result": "actual-vehicle false merge denominator is incomplete; no global candidate replay",
      "decision": "REJECTED_FALSE_MERGE_RISK",
    },
    {
      "candidate": "C_REPRESENTATIVE_HYSTERESIS",
      "idea": "hold the previous representative unless the replacement is materially better",
      "root_case_capture": "0/11",
      "causal_result": "parent representative raw436 already remained stable; the pair gate split happens first",
      "safety_result": "NOT_RUN",
      "decision": "REJECTED_MECHANISM_MISS",
    },
    {
      "candidate": "D_BOUNDED_ANCESTRY_OWNER_VETO",
      "idea": "diagnose recent absorbed-member lineage with active-owner ambiguity veto",
      "root_case_capture": "11/11 FAMILY_LABEL_ONLY",
      "causal_result": "can name the O1 family causally but cannot give one PID to two simultaneous children",
      "safety_result": "SHADOW NOT RUN: moving-to-moving DIFFERENT denominator = 0",
      "decision": "INCONCLUSIVE_NOT_IMPLEMENTED",
    },
  ]
  write_csv(STUDY / "tables/candidate_history.csv", list(candidates[0]), candidates)

  metrics = [
    {
      "metric": "Route269 exact group-to-singleton new PID births",
      "baseline": len(exact_splits),
      "final": len(exact_splits),
      "unit": "events",
      "status": "UNCHANGED_NO_PRODUCTION_CANDIDATE",
    },
    {
      "metric": "Route269 returns to parent group",
      "baseline": len(rejoins),
      "final": len(rejoins),
      "unit": "events",
      "status": "UNCHANGED_NO_PRODUCTION_CANDIDATE",
    },
    {"metric": "confirmed SAME transitions with distinct PID", "baseline": len(same), "final": len(same), "unit": "events", "status": "NO_IMPROVEMENT"},
    {
      "metric": "confirmed sequential DIFFERENT baseline stitches",
      "baseline": sum(r["old_pid"] == r["new_pid"] for r in static_different),
      "final": sum(r["old_pid"] == r["new_pid"] for r in static_different),
      "unit": "events/4",
      "status": "0 OBSERVED; NO CANDIDATE",
    },
    {
      "metric": "moving-to-moving DIFFERENT safety denominator",
      "baseline": len(moving_different),
      "final": len(moving_different),
      "unit": "events",
      "status": "UNEVALUABLE",
    },
    {
      "metric": "whole-corpus mechanical split births",
      "baseline": prevalence["split_births"],
      "final": prevalence["split_births"],
      "unit": "events/847 segments",
      "status": "UNLABELED_MECHANICS",
    },
    {
      "metric": "whole-corpus parent returns <=3s",
      "baseline": prevalence["return_to_parent"],
      "final": prevalence["return_to_parent"],
      "unit": "events",
      "status": "NOT PHYSICAL SAME GT",
    },
    {
      "metric": "Route2bc S21 representative changes",
      "baseline": route2bc_summary[0]["representative_changes"],
      "final": route2bc_summary[0]["representative_changes"],
      "unit": "changes/11 scans",
      "status": "IDENTITY_STABLE",
    },
    {
      "metric": "Route2bc S21 member-set changes",
      "baseline": route2bc_summary[0]["member_set_changes"],
      "final": route2bc_summary[0]["member_set_changes"],
      "unit": "changes/11 scans",
      "status": "IDENTITY_STABLE",
    },
    {
      "metric": "Route280 S15 actual cut-in tracker-to-publication latency",
      "baseline": cutin[0]["tracker_to_publication_ms"],
      "final": cutin[0]["tracker_to_publication_ms"],
      "unit": "ms",
      "status": "UNCHANGED_NO_CANDIDATE",
    },
  ]
  write_csv(STUDY / "tables/metrics.csv", list(metrics[0]), metrics)

  coverage = [
    {"case": 1, "scenario": "stable return", "test": "test_one_lsb_jitter_at_three_metres_keeps_one_identity", "coverage": "DIRECT"},
    {
      "case": 2,
      "scenario": "same vehicle raw slot move",
      "test": "test_track_id_survives_slot_move_and_resets_with_age; test_physical_slot_handoff_is_deduplicated_and_keeps_track_id",
      "coverage": "DIRECT",
    },
    {"case": 3, "scenario": "partial member disappearance", "test": "test_coasted_member_carries_the_id_without_a_continuity_check", "coverage": "DIRECT"},
    {
      "case": 4,
      "scenario": "short complete dropout and reacquire",
      "test": "test_one_scan_miss_recovers_the_same_identity; test_the_coast_window_is_exactly_three_nominal_scan_periods",
      "coverage": "RAW_ID_ONLY; DISTINCT_PID_REACQUISITION_NOT_IMPLEMENTED",
    },
    {
      "case": 5,
      "scenario": "different target near old position",
      "test": "test_slot_reuse_by_a_distant_target_starts_a_new_identity; test_the_same_slot_bonus_cannot_reach_past_a_hard_gate",
      "coverage": "PARTIAL; MOVING_GT_DENOMINATOR_ZERO",
    },
    {
      "case": 6,
      "scenario": "two close parallel vehicles",
      "test": "test_two_multi_return_vehicles_keep_four_identities; test_a_standstill_queue_never_swaps_identities",
      "coverage": "DIRECT_SYNTHETIC",
    },
    {
      "case": 7,
      "scenario": "large truck with three returns",
      "test": "test_multi_member_object_is_published_without_temporal_extension",
      "coverage": "PARTIAL; PHYSICAL_SINGLE_GROUP_GT_LIMITED",
    },
    {
      "case": 8,
      "scenario": "representative A disappears and B remains",
      "test": "test_same_pid_representative_switch_and_new_pid_independence; test_slot_member_and_representative_changes_keep_alias",
      "coverage": "DIRECT",
    },
    {
      "case": 9,
      "scenario": "truck plus adjacent car",
      "test": "test_bosch_provisional_bundle_two_real_objects_not_merged; test_legitimate_large_vehicle_companion_is_kept",
      "coverage": "DIRECT_SYNTHETIC",
    },
    {"case": 10, "scenario": "cut-in", "test": "test_monotone_lateral_cutin_is_fail_open; Route280 S15 replay", "coverage": "DIRECT_SYNTHETIC_PLUS_LOG"},
    {
      "case": 11,
      "scenario": "input gap/fault/reset",
      "test": "test_a_timestamp_reset_is_refused_and_leaves_state_untouched; test_scan_gap_resets_every_state",
      "coverage": "DIRECT",
    },
    {
      "case": 12,
      "scenario": "segment/process reset",
      "test": "test_a_segment_boundary_gap_starts_new_identities; test_provider_timeout_drops_every_anchor",
      "coverage": "DIRECT",
    },
  ]
  write_csv(STUDY / "tables/requested_test_coverage.csv", list(coverage[0]), coverage)

  provenance_files = [
    PROVIDER,
    group_transitions_path,
    group_timeline_path,
    group_same_path,
    group_different_path,
    cutin_path,
    prevalence_path,
    static_different_path,
    moving_different_path,
    moving_same_path,
    moving_gate_path,
    sidepass_history_path,
    sidepass_manifest_path,
    sidepass_prefix_path,
  ]
  manifest = {
    "study": "20260924_p1_pid_continuity",
    "repository_root": str(REPO),
    "branch": "heatagain/bosch-p1-pid-continuity",
    "baseline_head": "581285fb7dbed511fec4ba9e0edd256f5b5aa96e",
    "provider_raw_sha256": sha256(PROVIDER),
    "provider_normalized_lf_sha256": normalized_sha256(PROVIDER),
    "provider_semantically_matches_validated_source": normalized_sha256(PROVIDER) == "eec1c34448534f11cd8cd479a7ceb8d59be83421224130d7bb74f9943ab67f75",
    "policy": {
      "source_artifacts": "read-only",
      "production_code_changed": False,
      "shadow_implemented": False,
      "future_information_used": False,
      "moving_different_entry_gate": len(moving_different),
    },
    "facts": {
      "route269_exact_splits": len(exact_splits),
      "route269_rejoins": len(rejoins),
      "route269_continuous_raw_scans": 255,
      "confirmed_same": len(same),
      "confirmed_sequential_different": len(static_different),
      "strict_sequential_different": sum(r["candidate_id"] != "VQ004" for r in static_different),
      "moving_sequential_different": len(moving_different),
      "moving_sequential_same": len(moving_same),
      "simultaneous_different_pairs": len(simultaneous_different),
      "route280_s15_tracker_to_publication_ms": float(cutin[0]["tracker_to_publication_ms"]),
      "route2bc_window_scans": len(route2bc),
      "route2bc_pids": unique([row["physical_pid"] for row in route2bc]),
      "route2bc_representative_changes": route2bc_summary[0]["representative_changes"],
      "route2bc_member_set_changes": route2bc_summary[0]["member_set_changes"],
      "corpus_segments": prevalence["segment_count"],
      "corpus_scans": prevalence["scan_count"],
      "corpus_split_births": prevalence["split_births"],
    },
    "sources": [{"path": str(path), "sha256": sha256(path)} for path in provenance_files],
  }
  (STUDY / "manifests/evidence_provenance.json").write_text(
    json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
  )
  print(json.dumps(manifest["facts"], ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
  main()
