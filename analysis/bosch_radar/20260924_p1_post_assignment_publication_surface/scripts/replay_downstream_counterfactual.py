"""Replay and print one segment's P0-P6 downstream counterfactual summary."""
from __future__ import annotations

import argparse

from replay_post_assignment_surface import POLICIES, run_segment


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("key")
  parser.add_argument("--policy", choices=POLICIES)
  args = parser.parse_args()
  result = run_segment(args.key, downstream=True)
  policies = (args.policy,) if args.policy else POLICIES
  for policy in policies:
    print(policy, result["policies"][policy]["summary"])


if __name__ == "__main__":
  main()
