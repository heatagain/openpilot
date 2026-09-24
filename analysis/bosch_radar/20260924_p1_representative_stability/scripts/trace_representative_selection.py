# ruff: noqa: TID251
"""Print exact selector inputs/costs for one replayed event."""
from __future__ import annotations

import argparse

from common import read_json_gz, segment_result_path


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("key")
  parser.add_argument("--scan", type=int)
  parser.add_argument("--pid", type=int)
  args = parser.parse_args()
  run = read_json_gz(segment_result_path(args.key, "baseline"))["run"]
  for row in run["selection_trace"]:
    if args.pid is not None and int(row["pid"] or -1) != args.pid:
      continue
    if args.scan is not None:
      snapshot = run["snapshots"][args.scan]
      if int(row["scan_ns"]) != int(snapshot["scan_ns"]):
        continue
    print(row)


if __name__ == "__main__":
  main()
