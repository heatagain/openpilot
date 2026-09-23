"""Summarize physical-PID owner assignment on mined group fractures."""
from __future__ import annotations

import json
from collections import Counter, defaultdict

from common import MANIFESTS, SCRATCH, read_json_gz, write_json


def main() -> None:
  bundle = read_json_gz(SCRATCH / "baseline_mechanical_bundle.json.gz")
  rows = bundle["owner_rows"]
  events = defaultdict(list)
  for row in rows:
    events[row["event_id"]].append(row)
  parent_to_rep = parent_to_majority = conflict = 0
  for candidates in events.values():
    parent_pid = int(candidates[0]["parent_pid"])
    winner = next((row for row in candidates if int(row["prior_pid"]) == parent_pid and int(row["winner"])), None)
    if winner is None:
      continue
    if int(winner["representative_retained"]):
      parent_to_rep += 1
    parent_scores = [row for row in candidates if int(row["prior_pid"]) == parent_pid]
    majority = max(parent_scores, key=lambda row: (int(row["overlap"]), float(row["score"])))
    if majority["child_pid"] == winner["child_pid"]:
      parent_to_majority += 1
    if int(majority["representative_retained"]) != int(winner["representative_retained"]):
      conflict += 1
  births = Counter(row["cause"] for row in bundle["births"])
  summary = {
    "fracture_events_with_assignment_rows": len(events),
    "parent_pid_to_representative_child": parent_to_rep,
    "parent_pid_to_member_majority_child": parent_to_majority,
    "majority_representative_conflicts": conflict,
    "pid_birth_causes": dict(sorted(births.items())),
    "score_formula": "10*overlap + 3*representative_retention + min(age,1000)*1e-5 + 1/(pid+1)",
    "absorbed_cleanup": "same-scan removal of stolen raw ownership; empty non-output state retires",
  }
  write_json(MANIFESTS / "owner_assignment_summary.json", summary)
  print(json.dumps(summary, indent=2))


if __name__ == "__main__":
  main()
