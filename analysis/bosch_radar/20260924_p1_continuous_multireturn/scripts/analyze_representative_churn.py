"""Summarize representative transitions separately from group fractures."""
from __future__ import annotations

import json
from collections import Counter

from common import MANIFESTS, SCRATCH, percentile, read_json_gz, write_json


def main() -> None:
  rows = read_json_gz(SCRATCH / "baseline_mechanical_bundle.json.gz")["rep_transitions"]
  changes = [row for row in rows if int(row["representative_changed"])]
  summary = {
    "transition_rows": len(rows), "representative_changes": len(changes),
    "categories": dict(sorted(Counter(row["category"] for row in rows).items())),
    "jump_abs": {
      axis: {"p50": percentile([float(row[axis]) for row in changes], .5),
             "p95": percentile([float(row[axis]) for row in changes], .95),
             "p99": percentile([float(row[axis]) for row in changes], .99),
             "max": max((float(row[axis]) for row in changes), default=None)}
      for axis in ("abs_delta_d", "abs_delta_y", "abs_delta_v")
    },
    "published_before_and_after": sum(int(row["old_published"]) and int(row["new_published"]) for row in changes),
  }
  write_json(MANIFESTS / "representative_summary.json", summary)
  print(json.dumps(summary, indent=2))


if __name__ == "__main__":
  main()
