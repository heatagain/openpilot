# ruff: noqa: TID251
"""Replay the current baseline over 847 segments and mine representative events."""
from __future__ import annotations

import argparse
import multiprocessing as mp

from classify_rep_churn import analyze_baseline_run, finalize
from common import (ROUTE2BC_S21_KEY, corpus_keys, ensure_dirs, process_count,
                    segment_result_path, write_json_gz)
from replay_core import load_segment, run_policy


def _process(payload: tuple[str, bool, bool]) -> tuple[str, int, int]:
  key, force, corpus_member = payload
  destination = segment_result_path(key, "baseline" if corpus_member else "controls")
  if destination.exists() and not force:
    return key, -1, -1
  context = load_segment(key)
  run = run_policy(key, context, "BASELINE", downstream=True)
  analysis = analyze_baseline_run(run)
  write_json_gz(destination, {"run": run, "analysis": analysis})
  return key, len(run["snapshots"]), len(analysis["events"])


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--force", action="store_true")
  parser.add_argument("--workers", type=int)
  parser.add_argument("--limit", type=int)
  parser.add_argument("--key", action="append", default=[])
  parser.add_argument("--no-finalize", action="store_true")
  args = parser.parse_args()
  ensure_dirs()
  keys = args.key or corpus_keys()
  if not args.key and len(keys) != 847:
    raise RuntimeError(f"expected 847 segments, found {len(keys)}")
  if args.limit:
    keys = keys[:args.limit]
  payloads = [(key, args.force, True) for key in keys]
  workers = args.workers or process_count(2)
  with mp.Pool(processes=workers, maxtasksperchild=10) as pool:
    for index, (key, scans, changes) in enumerate(pool.imap_unordered(_process, payloads, chunksize=1), 1):
      print(f"{index}/{len(payloads)} {key}: scans={scans} rep_changes={changes}", flush=True)
  if not args.key and not args.limit:
    _process((ROUTE2BC_S21_KEY, args.force, False))
    if not args.no_finalize:
      print(finalize())


if __name__ == "__main__":
  mp.freeze_support()
  main()
