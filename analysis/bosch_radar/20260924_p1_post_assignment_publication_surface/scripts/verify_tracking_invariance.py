"""Fail closed on the full-corpus post-assignment invariance contract."""
from __future__ import annotations

import argparse
import json
import os
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed

from evaluate_publication_candidates import INVARIANT_FIELDS, write_csv, write_json
from replay_post_assignment_surface import MANIFESTS, POLICIES, TABLES, corpus_keys, run_segment


def _live_segment(key: str) -> dict:
  result = run_segment(key, downstream=False)
  return {
    "key": key, "scans": result["scans"], "baseline_parity": result["baseline_parity"],
    "candidates": {policy: result["policies"][policy]["invariance"] for policy in POLICIES},
  }


def live_summary(workers: int) -> dict:
  keys = corpus_keys()
  baseline = Counter()
  candidates = {policy: Counter() for policy in POLICIES}
  scans = 0
  with ProcessPoolExecutor(max_workers=workers) as pool:
    futures = {pool.submit(_live_segment, key): key for key in keys}
    for count, future in enumerate(as_completed(futures), 1):
      result = future.result()
      scans += int(result["scans"])
      baseline.update({field: int(value) for field, value in result["baseline_parity"].items()})
      for policy in POLICIES:
        candidates[policy].update({field: int(value) for field, value in
                                   result["candidates"][policy].items()})
      print(f"{count}/{len(keys)} {result['key']} scans={result['scans']}", flush=True)
  return {
    "routes": len({key.split("--", 1)[0] for key in keys}), "segments": len(keys), "scans": scans,
    "baseline_parity": dict(baseline),
    "candidates": {policy: dict(candidates[policy]) for policy in POLICIES},
    "aLead_signature": "complete debug snapshot including counters, lifecycle fields, and every PID state",
  }


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--live", action="store_true")
  parser.add_argument("--workers", type=int, default=max(1, min(2, os.cpu_count() or 1)))
  args = parser.parse_args()
  if args.live:
    source = live_summary(args.workers)
    write_json(MANIFESTS / "invariance_live_summary.json", source)
  else:
    source = json.loads((MANIFESTS / "invariance_summary.json").read_text(encoding="utf-8"))
  failures = []
  rows = []
  for policy in POLICIES:
    values = source["candidates"][policy]
    for field in INVARIANT_FIELDS:
      mismatch = int(values.get(field, -1))
      rows.append({"policy": policy, "invariant": field, "mismatch_scans": mismatch,
                   "pass": int(mismatch == 0)})
      if mismatch:
        failures.append(f"{policy}:{field}={mismatch}")
  for field, value in source["baseline_parity"].items():
    if int(value):
      failures.append(f"baseline_parity:{field}={value}")
  write_csv(TABLES / "invariance_results.csv", rows)
  result = {"status": "PASS" if not failures else "FAIL", "failures": failures,
            "baseline_parity": source["baseline_parity"], "candidate_rows": len(rows),
            "live_replay": bool(args.live), "routes": source.get("routes"),
            "segments": source.get("segments"), "scans": source.get("scans"),
            "aLead_signature": source.get("aLead_signature")}
  write_json(MANIFESTS / "invariance_validation.json", result)
  if failures:
    raise AssertionError(result)
  print(result)


if __name__ == "__main__":
  main()
