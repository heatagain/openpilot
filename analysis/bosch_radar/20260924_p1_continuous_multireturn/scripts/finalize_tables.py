#!/usr/bin/env python3
"""Join independently produced candidate, control, and benchmark summaries."""

from __future__ import annotations

import csv
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TABLES = ROOT / "tables"


def read_csv(name: str) -> list[dict[str, str]]:
  with (TABLES / name).open(newline="", encoding="utf-8") as f:
    return list(csv.DictReader(f))


def write_csv(name: str, rows: list[dict[str, object]]) -> None:
  path = TABLES / name
  with path.open("w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=list(rows[0]))
    writer.writeheader()
    writer.writerows(rows)


def main() -> None:
  candidates = read_csv("candidate_results.csv")
  controls = {row["candidate"]: row for row in read_csv("frozen_control_summary.csv")}
  cpu = {row["candidate"]: row for row in read_csv("cpu_summary.csv")}
  for row in candidates:
    control = controls[row["candidate"]]
    benchmark = cpu[row["candidate"]]
    row["different_controls_evaluable"] = control["different_evaluable"]
    row["different_control_regressions"] = control["different_false_merges"]
    row["same_controls_evaluable"] = control["same_evaluable"]
    row["same_control_regressions"] = control["same_regressions"]
    row["cpu_mean_us_weighted"] = benchmark["mean_us"]
    row["cpu_p95_segment_us"] = benchmark["p95_us"]
    row["cpu_p99_segment_us"] = benchmark["p99_us"]
    row["cpu_max_us"] = benchmark["max_us"]
  write_csv("candidate_results.csv", candidates)

  regression = []
  for row in read_csv("frozen_control_summary.csv"):
    regression.append({
      **row,
      "control_scope": "frozen SAME plus simultaneous DIFFERENT only",
      "moving_sequential_different": "NOT_USED",
      "new_actor_gt": "NO",
    })
  write_csv("regression_summary.csv", regression)
  print(f"joined {len(candidates)} candidates and {len(regression)} control summaries")


if __name__ == "__main__":
  main()
