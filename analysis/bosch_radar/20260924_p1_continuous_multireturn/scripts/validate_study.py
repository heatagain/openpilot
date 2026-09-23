#!/usr/bin/env python3
"""Fail closed on the final P1 continuous multi-return artifact."""

from __future__ import annotations

import csv
import hashlib
import json
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TABLES = ROOT / "tables"
MANIFESTS = ROOT / "manifests"
SOURCE = ROOT.parents[2] / "opendbc_repo" / "opendbc" / "car" / "hyundai" / "radar_interface.py"
EXPECTED_SOURCE_SHA256 = "fb756fb400b8408840c68b5ee32d6cae795ec939c9cdea634fb5836a235851a9"


def rows(name: str):
  with (TABLES / name).open(newline="", encoding="utf-8") as f:
    return list(csv.DictReader(f))


def require(condition: bool, message: str) -> None:
  if not condition:
    raise AssertionError(message)


def main() -> None:
  required_tables = (
    "all_group_splits.csv", "split_rejoin_events.csv", "oscillating_pairs.csv", "threshold_excess.csv",
    "group_lifetime.csv", "pid_births.csv", "owner_assignment.csv", "representative_changes.csv",
    "representative_jumps.csv", "publication_discontinuities.csv", "multi_return_clusters.csv",
    "stable_controls.csv", "candidate_results.csv", "regression_summary.csv", "cpu_summary.csv",
  )
  require(all((TABLES / name).exists() for name in required_tables), "missing required table")
  source_hash = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
  require(source_hash == EXPECTED_SOURCE_SHA256, f"production source changed: {source_hash}")

  baseline = json.loads((MANIFESTS / "baseline_replay.json").read_text(encoding="utf-8"))
  require((baseline["segments"], baseline["scans"], baseline["routes"]) == (847, 504673, 34), "corpus drift")
  require((baseline["group_fractures"], baseline["split_rejoins"]) == (21012, 7861), "event-count drift")
  mechanical = json.loads((MANIFESTS / "mechanical_summary.json").read_text(encoding="utf-8"))
  route269 = mechanical["route269_pair_family"]
  require((route269["split_count"], route269["rejoin_count"], route269["pid_births"]) == (11, 11, 11),
          "Route269 count drift")
  require(route269["raw_pair"] == "436|482", "Route269 edge drift")

  with (ROOT / "traces" / "route269" / "route269_s7_raw436_raw482.csv").open(newline="", encoding="utf-8") as f:
    route_trace = list(csv.DictReader(f))
  raw482 = [row for row in route_trace if row["raw482_observed"] == "1"]
  require(len(raw482) == 255 and [int(row["scan"]) for row in raw482] == list(range(345, 600)),
          "raw482 is not continuous for 255 scans")

  edges = [row for row in rows("threshold_excess.csv") if row["route"] == "00000269" and row["segment"] == "7"
           and row["raw_a"] == "436" and row["raw_b"] == "482"]
  require(len(edges) == 11, "Route269 failure edge rows")
  require(Counter(row["reason"] for row in edges) == Counter({"DISTANCE_DIAMETER": 6,
                                                               "LATERAL_GROWTH": 4,
                                                               "LATERAL_DIAMETER": 1}), "Route269 causes")

  prefix = json.loads((MANIFESTS / "prefix_summary.json").read_text(encoding="utf-8"))
  require(prefix == {"checks": 25, "passed": 25, "failed": 0}, "prefix validation failed")
  synthetic = json.loads((MANIFESTS / "synthetic_summary.json").read_text(encoding="utf-8"))
  require(synthetic["cases"] == 13 and synthetic["variants"] == 6 and len(synthetic["results"]) == 78,
          "synthetic case/variant matrix incomplete")
  lifecycle = rows("synthetic_lifecycle.csv")
  require(len(lifecycle) == 6 and all(row["timestamp_regression_refused"] == "1" and
                                      row["timestamp_regression_state_unchanged"] == "1" and
                                      row["stale_state_after_gap"] == "0" and
                                      row["provider_reset_fresh_state"] == "1" for row in lifecycle),
          "lifecycle failure")

  candidates = {row["candidate"]: row for row in rows("candidate_results.csv")}
  require(set(candidates) == {"A_HYSTERESIS", "B_CONFIRMATION", "C_QUANTIZED", "D_STABLE_CORE", "E_REP_HOLD"},
          "candidate set drift")
  require(candidates["E_REP_HOLD"]["grouping_changed_scans"] == "0", "candidate E changed grouping")
  require(candidates["E_REP_HOLD"]["representative_changes"] == "50553", "candidate E rep count drift")
  require(all(row["different_control_regressions"] == "0" and row["same_control_regressions"] == "0"
              for row in candidates.values()), "frozen control regression")

  publication = rows("publication_discontinuities.csv")
  require(len(publication) == 21012, "publication table count")
  require(sum(row["external_visible"] == "1" for row in publication) == 21012, "visibility count")
  require(sum(row["internal_only"] == "1" for row in publication) == 0, "internal-only count")

  report = (ROOT / "report.md").read_text(encoding="utf-8")
  required_headings = (
    "## Summary", "## Repository State", "## Scope Boundary vs Claude Moving-Identity Study",
    "## Current Bosch Grouping Architecture", "## Baseline Corpus", "## Mechanical Event Taxonomy",
    "## Route269 Exact Root Cause", "## Route2bc Stable Truck Control", "## Complete-Link Failure Distribution",
    "## Oscillation Families", "## Threshold Excess Distribution", "## PID Ownership Mechanics",
    "## Representative Stability", "## Public Alias / Publication Impact", "## Large Multi-Return Mechanical Signature",
    "## Synthetic Harness", "## Candidate A", "## Candidate B", "## Candidate C", "## Candidate D", "## Candidate E",
    "## Corpus Comparison", "## Existing SAME Controls", "## Existing DIFFERENT Controls", "## Prefix Invariance",
    "## State Lifecycle", "## CPU / Complexity", "## Actor-Identity Dependency", "## Production Decision",
    "## Git / Commit / Push", "## Final Decision", "## Next Minimal Work",
  )
  require(all(heading in report for heading in required_headings), "report heading missing")

  result = {
    "status": "PASS",
    "source_sha256": source_hash,
    "corpus": {"segments": 847, "scans": 504673, "routes": 34},
    "fractures": 21012,
    "split_rejoins": 7861,
    "route269": {"splits": 11, "rejoins": 11, "raw482_continuous_scans": 255},
    "prefix": prefix,
    "production_source_changed": False,
    "new_actor_gt": False,
    "moving_actor_identity_used": False,
  }
  (MANIFESTS / "study_validation.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
  print(json.dumps(result, indent=2))


if __name__ == "__main__":
  main()
