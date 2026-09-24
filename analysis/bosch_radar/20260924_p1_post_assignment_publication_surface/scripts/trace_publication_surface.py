"""Print one cached changed-surface or downstream trace."""
from __future__ import annotations

import argparse

from replay_post_assignment_surface import POLICIES, read_json_gz, result_path


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("key")
  parser.add_argument("policy", choices=POLICIES)
  parser.add_argument("--downstream", action="store_true")
  args = parser.parse_args()
  data = read_json_gz(result_path(args.key))["policies"][args.policy]
  rows = data["downstream_samples" if args.downstream else "changed_samples"]
  for row in rows:
    print(row)


if __name__ == "__main__":
  main()
