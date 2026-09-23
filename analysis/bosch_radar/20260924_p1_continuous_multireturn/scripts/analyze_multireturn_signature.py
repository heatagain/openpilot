#!/usr/bin/env python3
"""Summarize sensor-space multi-return geometry without creating actor GT."""

from __future__ import annotations

import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
TABLES = ROOT / "tables"
MANIFESTS = ROOT / "manifests"


def read_csv(path: Path) -> list[dict[str, str]]:
  with path.open(newline="", encoding="utf-8") as f:
    return list(csv.DictReader(f))


def finite(values):
  return np.asarray([float(value) for value in values if value not in ("", None) and math.isfinite(float(value))])


def distribution(values) -> dict[str, float | int | None]:
  data = finite(values)
  if not len(data):
    return {"count": 0, "p50": None, "p95": None, "p99": None, "max": None}
  return {
    "count": int(len(data)),
    "p50": float(np.percentile(data, 50)),
    "p95": float(np.percentile(data, 95)),
    "p99": float(np.percentile(data, 99)),
    "max": float(np.max(data)),
  }


def member_set(row: dict[str, str]) -> set[int]:
  return {int(item) for item in row["members"].split("|") if item}


def main() -> None:
  episodes = read_csv(TABLES / "multi_return_clusters.csv")
  frozen = [row for row in read_csv(TABLES / "frozen_control_regression.csv")
            if row["candidate"] == "A_HYSTERESIS" and row["relation"] == "SAME_PHYSICAL_OBJECT"]

  by_key: dict[str, list[dict[str, str]]] = defaultdict(list)
  for episode in episodes:
    by_key[episode["key"]].append(episode)

  same_matches = []
  for control in frozen:
    scan_ns = int(control["matched_scan_ns"])
    raw_pair = {int(control["raw_a"]), int(control["raw_b"])}
    matched = next((episode for episode in by_key[control["key"]]
                    if int(episode["start_ns"]) <= scan_ns <= int(episode["end_ns"])
                    and raw_pair.issubset(member_set(episode))), None)
    if matched is not None:
      same_matches.append(matched)

  member_counts = Counter(int(row["member_count"]) for row in episodes)
  representative_switch_episodes = sum(len(row["representatives"].split("|")) > 1 for row in episodes)
  stable = read_csv(TABLES / "stable_controls.csv")
  stable_member_counts = Counter(int(row["member_count"]) for row in stable)

  metrics = [
    {"population": "ALL_MULTI_RETURN_EPISODES", "metric": "episodes", "value": len(episodes)},
    {"population": "ALL_MULTI_RETURN_EPISODES", "metric": "three_plus_member_episodes",
     "value": sum(count for members, count in member_counts.items() if members >= 3)},
    {"population": "ALL_MULTI_RETURN_EPISODES", "metric": "representative_switch_episodes",
     "value": representative_switch_episodes},
    {"population": "STABLE_MECHANICAL_CONTROLS", "metric": "episodes", "value": len(stable)},
    {"population": "STABLE_MECHANICAL_CONTROLS", "metric": "three_plus_member_episodes",
     "value": sum(count for members, count in stable_member_counts.items() if members >= 3)},
    {"population": "FROZEN_SAME_CONTROLS", "metric": "controls", "value": len(frozen)},
    {"population": "FROZEN_SAME_CONTROLS", "metric": "multi_return_episode_matches", "value": len(same_matches)},
  ]
  for population, rows in (("ALL_MULTI_RETURN_EPISODES", episodes),
                           ("STABLE_MECHANICAL_CONTROLS", stable),
                           ("FROZEN_SAME_MATCHED_EPISODES", same_matches)):
    for column in ("member_count", "max_d_extent", "max_y_extent", "max_v_spread", "duration_s"):
      dist = distribution(row[column] for row in rows)
      for statistic, value in dist.items():
        metrics.append({"population": population, "metric": f"{column}_{statistic}", "value": value})

  out = TABLES / "large_multireturn_summary.csv"
  with out.open("w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=("population", "metric", "value"))
    writer.writeheader()
    writer.writerows(metrics)

  summary = {
    "scope": "mechanical sensor-space only; no new actor identity adjudication",
    "episodes": len(episodes),
    "member_count_distribution": dict(sorted(member_counts.items())),
    "three_plus_member_episodes": sum(count for members, count in member_counts.items() if members >= 3),
    "representative_switch_episodes": representative_switch_episodes,
    "geometry": {
      column: distribution(row[column] for row in episodes)
      for column in ("max_d_extent", "max_y_extent", "max_v_spread", "duration_s")
    },
    "stable_controls": {
      "episodes": len(stable),
      "member_count_distribution": dict(sorted(stable_member_counts.items())),
      "geometry": {
        column: distribution(row[column] for row in stable)
        for column in ("max_d_extent", "max_y_extent", "max_v_spread", "duration_s")
      },
    },
    "frozen_same_controls": {
      "controls": len(frozen),
      "multi_return_episode_matches": len(same_matches),
      "geometry": {
        column: distribution(row[column] for row in same_matches)
        for column in ("member_count", "max_d_extent", "max_y_extent", "max_v_spread", "duration_s")
      },
    },
  }
  (MANIFESTS / "large_multireturn_summary.json").write_text(
    json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
  print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
  main()
