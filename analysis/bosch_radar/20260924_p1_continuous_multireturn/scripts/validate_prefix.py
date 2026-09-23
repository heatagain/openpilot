"""Execute causal prefix replays for every analysis-only candidate."""
from __future__ import annotations

import json

from candidate_modules import CANDIDATES
from common import MANIFESTS, TABLES, load_replay_module, write_csv, write_json
from evaluate_group_candidate import ROUTE269_KEY, _run_variant


def signature(snapshot):
  return (snapshot["ns"],
          tuple(sorted((obj["pid"], obj["members"], obj["representative"], obj["d"], obj["y"], obj["v"], obj["age"])
                       for obj in snapshot["objects"])),
          snapshot["published"])


def run_prefix(candidate: str, cutoff_ns: int):
  replay = load_replay_module()
  original_load_audit = replay.load_audit_module

  def load_truncated_audit():
    audit = original_load_audit()
    original_load_segment = audit.load_segment

    def load_segment(key):
      can, states, poses, models = original_load_segment(key)
      return can, [row for row in states if row[0] <= cutoff_ns], poses, models

    audit.load_segment = load_segment
    return audit

  replay.load_audit_module = load_truncated_audit
  try:
    return _run_variant(ROUTE269_KEY, candidate)["snapshots"]
  finally:
    replay.load_audit_module = original_load_audit


def main() -> None:
  rows = []
  labels = (("25%", .25), ("50%", .50), ("75%", .75), ("EVENT_407", None), ("100%", 1.0))
  for candidate in CANDIDATES[1:]:
    full = _run_variant(ROUTE269_KEY, candidate)["snapshots"]
    for label, fraction in labels:
      count = 408 if fraction is None else max(1, round(len(full) * fraction))
      count = min(count, len(full))
      # A completed Bosch scan is consumed on the following carState tick. Stop
      # immediately before the next completed scan rather than at the current
      # scan's availability timestamp.
      cutoff_ns = full[count]["ns"] - 1 if count < len(full) else 10**30
      prefix = run_prefix(candidate, cutoff_ns)
      compared = min(count, len(prefix))
      mismatches = sum(signature(prefix[index]) != signature(full[index]) for index in range(compared))
      length_match = len(prefix) == count
      rows.append({"candidate": candidate, "prefix": label, "expected_scans": count,
                   "actual_scans": len(prefix), "compared_scans": compared,
                   "mismatches": mismatches, "length_match": int(length_match),
                   "pass": int(length_match and mismatches == 0)})
      print(f"{candidate} {label}: {len(prefix)}/{count}, mismatch={mismatches}", flush=True)
  write_csv(TABLES / "prefix_invariance.csv", rows)
  summary = {"checks": len(rows), "passed": sum(row["pass"] for row in rows),
             "failed": sum(not row["pass"] for row in rows)}
  write_json(MANIFESTS / "prefix_summary.json", summary)
  print(json.dumps(summary, indent=2))


if __name__ == "__main__":
  main()
