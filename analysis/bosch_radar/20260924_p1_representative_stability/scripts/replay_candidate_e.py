# ruff: noqa: TID251
"""Convenience entrypoint: evaluate the authoritative Candidate E policy."""
from __future__ import annotations

import argparse

from common import MANIFESTS, read_json, read_json_gz, segment_result_path
from evaluate_selective_rep_candidates import compare_runs
from replay_core import load_segment, run_policy


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("key")
  args = parser.parse_args()
  baseline = read_json_gz(segment_result_path(args.key, "baseline"))
  run = run_policy(args.key, load_segment(args.key), "E", config=read_json(
    MANIFESTS / "candidate_evaluation.json").get("config", {}), downstream=True)
  print(compare_runs(args.key, baseline["run"], run, baseline["analysis"])["summary"])


if __name__ == "__main__":
  main()
