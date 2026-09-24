# ruff: noqa: TID251
"""Print the stored downstream comparison for one segment and policy."""
from __future__ import annotations

import argparse

from common import read_json_gz, segment_result_path


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("key")
  parser.add_argument("policy", choices=("E", "E1", "E2", "E3", "E4", "E5"))
  args = parser.parse_args()
  result = read_json_gz(segment_result_path(args.key, "candidates"))["policies"][args.policy]
  print(result["summary"])
  for row in result["radar_rows"][:25]:
    print(row)


if __name__ == "__main__":
  main()
