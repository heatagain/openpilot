# ruff: noqa: TID251
"""Exact 25/50/75/event/100 percent prefix validation for selective policies."""
from __future__ import annotations

import argparse
import math

from common import (MANIFESTS, ROUTE269_KEY, ROUTE280_S15_KEY, TABLES, corpus_keys,
                    read_json, write_csv, write_json)
from replay_core import SegmentContext, load_segment, run_policy


POLICIES = ("E1", "E2", "E3", "E4")


def _signature(snapshot: dict) -> tuple:
  objects = tuple((row["pid"], tuple(row["members"]), row["representative"], row["d"], row["y"], row["v"])
                  for row in snapshot["objects"])
  published = tuple((row["pid"], row["alias"], row["published_raw"], row["d"], row["y"], row["v"], row["a"])
                    for row in snapshot["published"])
  return objects, published


def _truncate(context: SegmentContext, cutoff_ns: int) -> SegmentContext:
  return SegmentContext(
    tuple(row for row in context.can if row[0] <= cutoff_ns),
    tuple(row for row in context.states if row[0] <= cutoff_ns),
    tuple(row for row in context.poses if row[0] <= cutoff_ns),
    tuple(row for row in context.models if row[0] <= cutoff_ns),
  )


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--key", action="append", default=[])
  args = parser.parse_args()
  candidate_manifest = read_json(MANIFESTS / "candidate_evaluation.json")
  config = candidate_manifest["config"]
  default_keys = [ROUTE269_KEY, ROUTE280_S15_KEY, corpus_keys()[0], corpus_keys()[len(corpus_keys())//2]]
  keys = args.key or default_keys
  rows = []
  for key in keys:
    context = load_segment(key)
    for policy in POLICIES:
      full = run_policy(key, context, policy, config=config, downstream=False)
      count = len(full["snapshots"])
      cutoffs = [("25", max(1, math.ceil(count*.25))), ("50", max(1, math.ceil(count*.50))),
                 ("75", max(1, math.ceil(count*.75))), ("100", count)]
      if key == ROUTE269_KEY:
        cutoffs.insert(3, ("EVENT_407", min(408, count)))
      for label, length in cutoffs:
        cutoff_ns = int(full["snapshots"][length-1]["now_ns"])
        truncated = run_policy(key, _truncate(context, cutoff_ns), policy, config=config, downstream=False)
        expected = tuple(_signature(row) for row in full["snapshots"][:len(truncated["snapshots"])])
        actual = tuple(_signature(row) for row in truncated["snapshots"])
        rows.append({"key": key, "policy": policy, "cutoff": label, "expected_scans": len(expected),
                     "actual_scans": len(actual), "pass": int(expected == actual)})
  write_csv(TABLES / "prefix_invariance.csv", rows)
  summary = {"comparisons": len(rows), "passed": sum(int(row["pass"]) for row in rows),
             "failed": sum(not int(row["pass"]) for row in rows)}
  write_json(MANIFESTS / "prefix_summary.json", summary)
  if summary["failed"]:
    raise AssertionError(summary)
  print(summary)


if __name__ == "__main__":
  main()
