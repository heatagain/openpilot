"""Shared, read-only helpers for the P1 continuous multi-return study.

The study always executes the Bosch region from the P1 worktree's current
radar_interface.py.  Recorded CAN/context is read from the established offline
corpus.  No helper reads the parallel moving-identity study.
"""
from __future__ import annotations

import csv
import gzip
import hashlib
import importlib.util
import json
import os
import sys
from pathlib import Path


WORKSPACE = Path(r"C:\CarrotRadarResearch")
CHECKOUT = WORKSPACE / "openpilot-p1-pid-continuity"
STUDY = CHECKOUT / "analysis" / "bosch_radar" / "20260924_p1_continuous_multireturn"
TABLES = STUDY / "tables"
TRACES = STUDY / "traces"
MANIFESTS = STUDY / "manifests"
SCRATCH = STUDY / "scratch"
SOURCE = CHECKOUT / "opendbc_repo" / "opendbc" / "car" / "hyundai" / "radar_interface.py"
TEST_SOURCE = CHECKOUT / "opendbc_repo" / "opendbc" / "car" / "hyundai" / "tests" / "test_radar.py"
CORPUS = (WORKSPACE / "analysis" / "20260919_MRRevo14F_parent_sibling_publication_shadow" /
          "scratch" / "corpus")
REPLAY_SCRIPTS = WORKSPACE / "20260921_MRRevo14F_lead_ahead_identity_reacquisition" / "scripts"
GROUP_STUDY = WORKSPACE / "analysis" / "20260923_MRRevo14F_group_owner_oscillation"
FROZEN_GT = (WORKSPACE / "analysis" / "20260920_MRRevo14F_typeA_identity_restratification" /
             "tables" / "typeA_identity_gt_v2_frozen.csv")


def configure_imports() -> None:
  paths = (
    REPLAY_SCRIPTS,
    WORKSPACE / "analysis",
    WORKSPACE / "analysis" / "bosch_performance",
    CHECKOUT,
    CHECKOUT / "opendbc_repo",
  )
  for path in reversed(paths):
    value = str(path)
    if value not in sys.path:
      sys.path.insert(0, value)


def load_replay_module():
  """Load the established replay driver without colliding with this common.py."""
  cached = sys.modules.get("p1_replay_current_provider")
  if cached is not None:
    return cached
  configure_imports()
  previous_common = sys.modules.get("common")
  replay_common_path = REPLAY_SCRIPTS / "common.py"
  common_spec = importlib.util.spec_from_file_location("common", replay_common_path)
  if common_spec is None or common_spec.loader is None:
    raise ImportError(replay_common_path)
  replay_common = importlib.util.module_from_spec(common_spec)
  sys.modules["common"] = replay_common
  common_spec.loader.exec_module(replay_common)
  replay_path = REPLAY_SCRIPTS / "replay_current_provider.py"
  replay_spec = importlib.util.spec_from_file_location("p1_replay_current_provider", replay_path)
  if replay_spec is None or replay_spec.loader is None:
    raise ImportError(replay_path)
  replay = importlib.util.module_from_spec(replay_spec)
  sys.modules["p1_replay_current_provider"] = replay
  try:
    replay_spec.loader.exec_module(replay)
  finally:
    if previous_common is None:
      sys.modules.pop("common", None)
    else:
      sys.modules["common"] = previous_common
  return replay


def ensure_dirs() -> None:
  for path in (TABLES, TRACES, MANIFESTS, SCRATCH):
    path.mkdir(parents=True, exist_ok=True)


def corpus_keys() -> list[str]:
  return [path.name.removesuffix(".json.gz") for path in sorted(CORPUS.glob("*.json.gz"))]


def sha256(path: Path, *, normalize_lf: bool = False) -> str:
  data = path.read_bytes()
  if normalize_lf:
    data = data.replace(b"\r\n", b"\n")
  return hashlib.sha256(data).hexdigest()


def write_csv(path: Path, rows: list[dict], fields: list[str] | None = None) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  if fields is None:
    fields = list(rows[0]) if rows else []
  with path.open("w", newline="", encoding="utf-8") as stream:
    writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore", lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)


def read_csv(path: Path) -> list[dict[str, str]]:
  with path.open(encoding="utf-8", newline="") as stream:
    return list(csv.DictReader(stream))


def write_json(path: Path, value) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


def write_json_gz(path: Path, value) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  with gzip.open(path, "wt", encoding="utf-8") as stream:
    json.dump(value, stream, ensure_ascii=False, separators=(",", ":"))


def read_json_gz(path: Path):
  with gzip.open(path, "rt", encoding="utf-8") as stream:
    return json.load(stream)


def percentile(values: list[float], q: float) -> float | None:
  if not values:
    return None
  ordered = sorted(values)
  if len(ordered) == 1:
    return ordered[0]
  position = (len(ordered) - 1) * q
  lower = int(position)
  upper = min(lower + 1, len(ordered) - 1)
  weight = position - lower
  return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def key_for_route_segment(route: str, segment: int, keys: list[str]) -> str | None:
  token = route.lower().removeprefix("route")
  try:
    route_value = int(token, 16)
  except ValueError:
    return None
  prefix = f"{route_value:08x}--"
  suffix = f"--{segment}"
  matches = [key for key in keys if key.startswith(prefix) and key.endswith(suffix)]
  return matches[0] if len(matches) == 1 else None


def route_label(key: str) -> str:
  return f"Route{key.split('--', 1)[0].lstrip('0') or '0'}"


def process_count(default: int = 6) -> int:
  return max(1, min(default, os.cpu_count() or 1))
