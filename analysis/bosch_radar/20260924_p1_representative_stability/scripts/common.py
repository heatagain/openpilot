"""Shared paths and small helpers for the representative-stability study."""
from __future__ import annotations

import csv
import gzip
import hashlib
import json
import os
import sys
from collections.abc import Iterable
from pathlib import Path
from typing import Any


WORKSPACE = Path(r"C:\CarrotRadarResearch")
CHECKOUT = WORKSPACE / "openpilot-p1-pid-continuity"
STUDY = CHECKOUT / "analysis" / "bosch_radar" / "20260924_p1_representative_stability"
TABLES = STUDY / "tables"
TRACES = STUDY / "traces"
MANIFESTS = STUDY / "manifests"
SCRATCH = STUDY / "scratch"
SOURCE = CHECKOUT / "opendbc_repo" / "opendbc" / "car" / "hyundai" / "radar_interface.py"
TEST_SOURCE = CHECKOUT / "opendbc_repo" / "opendbc" / "car" / "hyundai" / "tests" / "test_radar.py"
CORPUS = (WORKSPACE / "analysis" / "20260919_MRRevo14F_parent_sibling_publication_shadow" /
          "scratch" / "corpus")
DATA = WORKSPACE / "data"
REPLAY_SCRIPTS = WORKSPACE / "20260921_MRRevo14F_lead_ahead_identity_reacquisition" / "scripts"
P1_PRIOR = CHECKOUT / "analysis" / "bosch_radar" / "20260924_p1_continuous_multireturn"

ROUTE269_KEY = "00000269--4014745f93--7"
ROUTE2BC_S21_KEY = "000002bc--30683d78c9--21"
ROUTE2BC_WINDOW_NS = (1_347_345_050_240, 1_348_345_117_576)
ROUTE280_S15_KEY = "00000280--91181081d4--15"
P0_CONTROLS = {
  "0000026d--2363789071--56": 1000191,
  "0000026d--2363789071--60": 1000275,
  "0000028b--ca855432c0--33": 1000118,
}


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


def ensure_dirs() -> None:
  for path in (
    TABLES, MANIFESTS, SCRATCH,
    TRACES / "stable_group", TRACES / "pingpong", TRACES / "large_jump",
    TRACES / "downstream", TRACES / "controls", TRACES / "candidate_e",
  ):
    path.mkdir(parents=True, exist_ok=True)


def corpus_keys() -> list[str]:
  return [path.name.removesuffix(".json.gz") for path in sorted(CORPUS.glob("*.json.gz"))]


def sha256(path: Path, *, normalize_lf: bool = False) -> str:
  data = path.read_bytes()
  if normalize_lf:
    data = data.replace(b"\r\n", b"\n")
  return hashlib.sha256(data).hexdigest()


def read_json(path: Path) -> Any:
  return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, value: Any) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8", newline="\n")


def read_json_gz(path: Path) -> Any:
  with gzip.open(path, "rt", encoding="utf-8") as stream:
    return json.load(stream)


def write_json_gz(path: Path, value: Any) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  with gzip.open(path, "wt", encoding="utf-8", newline="\n") as stream:
    json.dump(value, stream, ensure_ascii=False, separators=(",", ":"))


def write_csv(path: Path, rows: Iterable[dict[str, Any]], fields: Iterable[str] | None = None) -> None:
  materialized = list(rows)
  if fields is None:
    names: list[str] = []
    for row in materialized:
      for name in row:
        if name not in names:
          names.append(name)
  else:
    names = list(fields)
  path.parent.mkdir(parents=True, exist_ok=True)
  with path.open("w", encoding="utf-8", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=names, extrasaction="ignore", lineterminator="\n")
    writer.writeheader()
    writer.writerows(materialized)


def read_csv(path: Path) -> list[dict[str, str]]:
  with path.open(encoding="utf-8", newline="") as stream:
    return list(csv.DictReader(stream))


def percentile(values: Iterable[float], q: float) -> float | None:
  ordered = sorted(float(value) for value in values)
  if not ordered:
    return None
  if len(ordered) == 1:
    return ordered[0]
  position = (len(ordered) - 1) * q
  lower = int(position)
  upper = min(lower + 1, len(ordered) - 1)
  weight = position - lower
  return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def process_count(default: int = 2) -> int:
  override = os.environ.get("BOSCH_REP_WORKERS")
  requested = int(override) if override else default
  return max(1, min(requested, os.cpu_count() or 1))


def segment_result_path(key: str, phase: str) -> Path:
  return SCRATCH / phase / f"{key}.json.gz"


def route_segment(key: str) -> tuple[str, int]:
  route, _, segment = key.split("--")
  return route, int(segment)
