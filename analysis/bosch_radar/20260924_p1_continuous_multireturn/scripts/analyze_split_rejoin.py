"""Post-process baseline fracture events into rejoin and oscillation families."""
from __future__ import annotations

import json
from collections import Counter, defaultdict

from common import MANIFESTS, SCRATCH, TABLES, percentile, read_json_gz, write_csv, write_json


def main() -> None:
  bundle = read_json_gz(SCRATCH / "baseline_mechanical_bundle.json.gz")
  splits = bundle["splits"]
  rejoined = [row for row in splits if row["rejoin_scan"] != ""]
  write_csv(TABLES / "split_rejoin_events.csv", rejoined)

  by_family = defaultdict(list)
  for row in splits:
    pair = row["failure_edge"] or row["parent_members"]
    by_family[(row["key"], pair)].append(row)
  families = []
  for (key, pair), rows in sorted(by_family.items()):
    if len(rows) < 2:
      continue
    times = sorted(int(row["split_ns"]) for row in rows)
    intervals = [(b-a)/1e9 for a, b in zip(times, times[1:], strict=False)]
    rejoin_latencies = [float(row["rejoin_latency_s"]) for row in rows if row["rejoin_latency_s"] != ""]
    reasons = Counter(row["failure_reason"] for row in rows)
    families.append({
      "family_id": f"{key}:{pair}", "key": key, "route": rows[0]["route"],
      "segment": rows[0]["segment"], "raw_pair": pair,
      "parent_member_sets": ";".join(sorted({row["parent_members"] for row in rows})),
      "split_count": len(rows), "rejoin_count": sum(row["rejoin_scan"] != "" for row in rows),
      "median_split_interval_s": percentile(intervals, .5) if intervals else "",
      "max_split_interval_s": max(intervals) if intervals else "",
      "median_rejoin_latency_s": percentile(rejoin_latencies, .5) if rejoin_latencies else "",
      "pid_births": sum(int(row["new_pid_births"]) for row in rows),
      "representative_retained": sum(not int(row["representative_change"]) for row in rows),
      "boundary_causes": json.dumps(dict(sorted(reasons.items())), separators=(",", ":")),
      "publication_visible": sum(int(row["external_visible"]) for row in rows),
      "internal_only": sum(int(row["internal_only"]) for row in rows),
    })
  write_csv(TABLES / "oscillating_pairs.csv", families)

  threshold_rows = bundle["thresholds"]
  distributions = []
  for reason in sorted({row["reason"] for row in threshold_rows}):
    values = [float(row["excess"]) for row in threshold_rows
              if row["reason"] == reason and row["excess"] not in ("", None)]
    distributions.append({
      "cause": reason, "count": len(values), "p50": percentile(values, .5),
      "p90": percentile(values, .9), "p95": percentile(values, .95),
      "p99": percentile(values, .99), "max": max(values) if values else None,
    })
  write_csv(TABLES / "threshold_excess_distribution.csv", distributions)

  # Stable mechanical controls are selected without actor adjudication: long,
  # unchanged multi-return episodes and the previously validated Route2bc trace.
  stable = []
  split_keys = {(row["key"], row["parent_pid"], row["parent_members"]) for row in splits}
  for row in bundle["group_lifetimes"]:
    if (int(row["member_count"]) >= 2 and int(row["scans"]) >= 20 and
        (row["key"], row["pid"], row["members"]) not in split_keys):
      stable.append({**row, "control_type": "STABLE_MULTI_RETURN_MECHANICAL"})
  stable.sort(key=lambda row: (-int(row["scans"]), row["key"], int(row["pid"])))
  write_csv(TABLES / "stable_controls.csv", stable[:500])

  summary = {
    "total_group_splits": len(splits), "split_rejoin": len(rejoined),
    "oscillation_families": len(families),
    "failure_causes": dict(sorted(Counter(row["failure_reason"] for row in splits).items())),
    "route269_pair_family": next((row for row in families
                                   if row["key"] == "00000269--4014745f93--7" and row["raw_pair"] == "436|482"), None),
    "threshold_distributions": distributions,
  }
  write_json(MANIFESTS / "mechanical_summary.json", summary)
  print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
  main()
