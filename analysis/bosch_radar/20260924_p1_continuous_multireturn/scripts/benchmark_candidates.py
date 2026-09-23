"""Alternating-order repeated full-path x86 benchmark for research candidates."""
from __future__ import annotations

import json
import time
from collections import defaultdict

from candidate_modules import CANDIDATES
from common import MANIFESTS, TABLES, corpus_keys, percentile, write_csv, write_json
from evaluate_group_candidate import _run_variant


TARGETS = (
  ("00000259", 11), ("00000264", 22), ("00000269", 7), ("00000280", 15),
  ("0000028b", 33), ("0000028f", 32), ("0000028f", 37), ("00000294", 0),
)


def select_keys():
  available = corpus_keys()
  result = []
  for prefix, segment in TARGETS:
    matches = [key for key in available if key.startswith(prefix + "--") and key.endswith(f"--{segment}")]
    if len(matches) == 1:
      result.append(matches[0])
  if len(result) < 6:
    raise RuntimeError(f"benchmark coverage too small: {result}")
  return result


def main() -> None:
  keys = select_keys()
  orders = (CANDIDATES, tuple(reversed(CANDIDATES)), CANDIDATES[2:] + CANDIDATES[:2])
  raw = []
  timings = defaultdict(list)
  wall = defaultdict(list)
  for repeat, order in enumerate(orders, 1):
    for key_index, key in enumerate(keys):
      actual_order = order if key_index % 2 == 0 else tuple(reversed(order))
      for position, candidate in enumerate(actual_order):
        started = time.perf_counter()
        result = _run_variant(key, candidate)
        elapsed = time.perf_counter() - started
        values = [value / 1000 for value in result["timings"]]
        timings[candidate].extend(values)
        wall[candidate].append(elapsed)
        raw.append({"repeat": repeat, "key": key, "order_position": position,
                    "candidate": candidate, "updates": len(values), "scans": len(result["snapshots"]),
                    "mean_us": sum(values)/len(values), "p95_us": percentile(values, .95),
                    "p99_us": percentile(values, .99), "max_us": max(values), "wall_s": elapsed})
        print(f"repeat {repeat} {key} {candidate}: {elapsed:.3f}s", flush=True)
  summary = []
  for candidate in CANDIDATES:
    values = timings[candidate]
    summary.append({"candidate": candidate, "segments": len(wall[candidate]), "updates": len(values),
                    "mean_us": sum(values)/len(values), "p50_us": percentile(values, .5),
                    "p95_us": percentile(values, .95), "p99_us": percentile(values, .99),
                    "max_us": max(values), "wall_s": sum(wall[candidate]),
                    "A1M_CPU": "NOT MEASURED"})
  write_csv(TABLES / "cpu_benchmark_raw.csv", raw)
  write_csv(TABLES / "cpu_summary.csv", summary)
  write_json(MANIFESTS / "cpu_benchmark.json", {"keys": keys, "repeats": len(orders), "summary": summary})
  print(json.dumps({"keys": keys, "summary": summary}, indent=2))


if __name__ == "__main__":
  main()
