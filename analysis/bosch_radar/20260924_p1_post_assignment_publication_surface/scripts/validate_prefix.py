"""Exact 25/50/75/event/100% prefix validation for P1-P6."""
from __future__ import annotations

import argparse
import math

from evaluate_publication_candidates import write_csv, write_json
from replay_post_assignment_surface import (MANIFESTS, POLICIES, PRIOR, ROUTE269_KEY, ROUTE280_KEY,
                                            TABLES, corpus_keys, read_json_gz,
                                            surface_signatures)


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--key", action="append", default=[])
  args = parser.parse_args()
  keys = args.key or [ROUTE269_KEY, ROUTE280_KEY, corpus_keys()[0], corpus_keys()[len(corpus_keys())//2]]
  rows = []
  for key in keys:
    baseline = read_json_gz(PRIOR / "scratch" / "baseline" / f"{key}.json.gz")["run"]["snapshots"]
    full = surface_signatures(key)
    count = len(full["P0"])
    cutoffs = [
      ("25", max(1, math.ceil(count*.25))),
      ("50", max(1, math.ceil(count*.50))),
      ("75", max(1, math.ceil(count*.75))),
      ("100", count),
    ]
    if key == ROUTE269_KEY:
      cutoffs.insert(3, ("EVENT_407", min(408, count)))
    for label, length in cutoffs:
      cutoff_ns = int(baseline[length-1]["now_ns"])
      truncated = surface_signatures(key, cutoff_ns=cutoff_ns)
      for policy in POLICIES[1:]:
        actual = truncated[policy]
        expected = full[policy][:len(actual)]
        rows.append({
          "key": key, "policy": policy, "cutoff": label,
          "expected_scans": len(expected), "actual_scans": len(actual),
          "pass": int(expected == actual),
        })
  write_csv(TABLES / "prefix_invariance.csv", rows)
  summary = {"comparisons": len(rows), "passed": sum(row["pass"] for row in rows),
             "failed": sum(not row["pass"] for row in rows)}
  write_json(MANIFESTS / "prefix_summary.json", summary)
  if summary["failed"]:
    raise AssertionError(summary)
  print(summary)


if __name__ == "__main__":
  main()
