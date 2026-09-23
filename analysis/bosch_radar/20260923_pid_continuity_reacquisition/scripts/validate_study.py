"""Validate the frozen offline evidence and its current-provider provenance."""

from __future__ import annotations

import csv
import hashlib
import json
from collections import Counter

from replay_identity import ROOT, STUDY


def rows(relative: str) -> list[dict]:
    with (STUDY / relative).open(encoding="utf-8", newline="") as inp:
        return list(csv.DictReader(inp))


def main() -> None:
    scope = json.loads((STUDY / "manifests" / "replay_scope.json").read_text(encoding="utf-8"))
    provider = ROOT / "openpilot" / "opendbc_repo" / "opendbc" / "car" / "hyundai" / "radar_interface.py"
    assert hashlib.sha256(provider.read_bytes()).hexdigest() == scope["provider_sha256"]
    inventory = rows("manifests/input_inventory.csv")
    assert len(inventory) == scope["cold_priority_segments"]
    assert sum(int(row["scans"]) for row in inventory) == scope["cold_priority_scans"]
    assert all(row["status"] == "AVAILABLE" and int(row["cache_count"]) == 1 for row in inventory)
    remap = rows("tables/sequential_head_remap.csv")
    assert len(remap) == scope["prior_same_confirmed_sequential_transitions"]
    assert Counter(row["head_status"] for row in remap) == {
        "DISTINCT_PID_ON_HEAD": 4, "UNRESOLVED_REMAP": 8,
    }
    prefix = rows("tables/prefix_event_invariance.csv")
    assert len(prefix) == scope["event_prefix_checks_passed"]
    assert all(row["equal"] == "1" and int(row["prefix_scans"]) > 0 for row in prefix)
    focus = rows("traces/forensic_focus.csv")
    owner = [(int(row["scan_ns"]), row["physical_pid"]) for row in focus
             if row["case"] == "R269_S7_GROUP_OWNER" and "482" in row["members"].split("|")]
    assert owner and {"1000618", "1000462", "1000634"}.issubset({pid for _, pid in owner})
    hard_negative = [row for row in focus if row["case"] == "R259_S2_HARD_NEGATIVE"]
    assert {"1000004", "1000106"}.issubset({row["physical_pid"] for row in hard_negative})
    print("PASS: provider hash; 14 priority segments/8398 scans; 12 sequential labels (4 unique, 8 unresolved); 5 event prefixes; focused lineage")


if __name__ == "__main__":
    main()
