#!/usr/bin/env python3
"""Fail-closed validation for the compact P1 evidence bundle."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path


STUDY = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[4]


def rows(path: Path) -> list[dict[str, str]]:
  with path.open(newline="", encoding="utf-8-sig") as f:
    return list(csv.DictReader(f))


def normalized_sha256(path: Path) -> str:
  return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def main() -> None:
  manifest = json.loads((STUDY / "manifests/evidence_provenance.json").read_text(encoding="utf-8"))
  facts = manifest["facts"]
  assert manifest["branch"] == "heatagain/bosch-p1-pid-continuity"
  assert manifest["baseline_head"] == "581285fb7dbed511fec4ba9e0edd256f5b5aa96e"
  assert manifest["policy"]["production_code_changed"] is False
  assert manifest["policy"]["shadow_implemented"] is False
  assert manifest["policy"]["future_information_used"] is False
  assert normalized_sha256(REPO / "opendbc_repo/opendbc/car/hyundai/radar_interface.py") == "eec1c34448534f11cd8cd479a7ceb8d59be83421224130d7bb74f9943ab67f75"

  assert facts["route269_exact_splits"] == 11
  assert facts["route269_rejoins"] == 11
  assert facts["route269_continuous_raw_scans"] == 255
  assert facts["confirmed_same"] == 4
  assert facts["confirmed_sequential_different"] == 4
  assert facts["strict_sequential_different"] == 3
  assert facts["moving_sequential_different"] == 0
  assert facts["moving_sequential_same"] == 1
  assert facts["simultaneous_different_pairs"] == 37
  assert facts["route280_s15_tracker_to_publication_ms"] == 0.0
  assert facts["route2bc_window_scans"] == 11
  assert facts["route2bc_pids"] == "1000845"
  assert facts["route2bc_representative_changes"] == 0
  assert facts["route2bc_member_set_changes"] == 0
  assert facts["corpus_segments"] == 847
  assert facts["corpus_scans"] == 504673
  assert facts["corpus_split_births"] == 16526

  candidates = rows(STUDY / "tables/candidate_history.csv")
  assert [row["candidate"] for row in candidates] == [
    "A_TIMEOUT_EXTENSION",
    "B_WIDEN_COMPLETE_LINK",
    "C_REPRESENTATIVE_HYSTERESIS",
    "D_BOUNDED_ANCESTRY_OWNER_VETO",
  ]
  assert not any("GO" == row["decision"] for row in candidates)

  route2bc = rows(STUDY / "traces/route2bc_s21_identity_window.csv")
  assert len(route2bc) == 11
  assert {row["raw_track_id"] for row in route2bc} == {"822"}
  assert {row["physical_pid"] for row in route2bc} == {"1000845"}
  assert {row["members"] for row in route2bc} == {"822"}
  assert {row["representative"] for row in route2bc} == {"822"}

  coverage = rows(STUDY / "tables/requested_test_coverage.csv")
  assert [int(row["case"]) for row in coverage] == list(range(1, 13))
  print("PASS: provider source, 11/11 owner cycles, GT denominators, Route2bc identity, four candidates, 12 requested cases")


if __name__ == "__main__":
  main()
