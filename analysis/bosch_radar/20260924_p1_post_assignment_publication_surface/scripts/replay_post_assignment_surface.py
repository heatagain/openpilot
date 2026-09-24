# ruff: noqa: TID251
"""Post-assignment publication-surface counterfactual replay.

The provider runs exactly once with the baseline tracking representative.  P0-P6
receive immutable, already-qualified and already-publication-filtered objects only
after alias allocation and the PID-owned aLead update.  Their output is cloned
RadarPoint geometry; it is never stored on a provider object or read next scan.
"""
from __future__ import annotations

import argparse
import copy
import csv
import gzip
import json
import math
import sys
import time
from collections import Counter, defaultdict
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any


CHECKOUT = Path(r"C:\CarrotRadarResearch\openpilot-p1-pid-continuity")
WORKSPACE = CHECKOUT.parent
STUDY = CHECKOUT / "analysis" / "bosch_radar" / "20260924_p1_post_assignment_publication_surface"
PRIOR = CHECKOUT / "analysis" / "bosch_radar" / "20260924_p1_representative_stability"
PRIOR_SCRIPTS = PRIOR / "scripts"
TABLES = STUDY / "tables"
TRACES = STUDY / "traces"
MANIFESTS = STUDY / "manifests"
SCRATCH = STUDY / "scratch"
FIXTURES = STUDY / "fixtures"
SOURCE = CHECKOUT / "opendbc_repo" / "opendbc" / "car" / "hyundai" / "radar_interface.py"
DATA = WORKSPACE / "data"
CORPUS = (WORKSPACE / "analysis" / "20260919_MRRevo14F_parent_sibling_publication_shadow" /
          "scratch" / "corpus")
P0_CONTROLS = {
  "0000026d--2363789071--56": 1000191,
  "0000026d--2363789071--60": 1000275,
  "0000028b--ca855432c0--33": 1000118,
}
ROUTE269_KEY = "00000269--4014745f93--7"
ROUTE280_KEY = "00000280--91181081d4--15"
ROUTE2BC_KEY = "000002bc--30683d78c9--21"
POLICIES = ("P0", "P1", "P2", "P3", "P4", "P5", "P6")
STATEFUL_POLICIES = ("P1", "P2", "P3", "P4", "P6")
MAX_GAP_NS = 300_000_000
PINGPONG_HISTORY_NS = 2_000_000_000
NEAR_TIE = 0.25
JUMP_P95 = (3.75, 0.84375, 0.50)
JUMP_P99 = (4.50, 1.28125, 0.75)
CONTEXT_MAX_AGE_NS = 200_000_000

if str(PRIOR_SCRIPTS) not in sys.path:
  sys.path.insert(0, str(PRIOR_SCRIPTS))

from common import configure_imports as _configure_prior_imports
from replay_core import _latest, _model_stub, load_segment
from representative_modules import load_policy_module


def configure_imports() -> None:
  _configure_prior_imports()


def ensure_dirs() -> None:
  for path in (
      TABLES, MANIFESTS, FIXTURES, SCRATCH / "segments", SCRATCH / "prefix",
      TRACES / "stable", TRACES / "pingpong", TRACES / "large_jump",
      TRACES / "publication", TRACES / "radarstate", TRACES / "controls"):
    path.mkdir(parents=True, exist_ok=True)


def corpus_keys() -> list[str]:
  return [path.name.removesuffix(".json.gz") for path in sorted(CORPUS.glob("*.json.gz"))]


def result_path(key: str) -> Path:
  return SCRATCH / "segments" / f"{key}.json.gz"


def read_json_gz(path: Path) -> Any:
  with gzip.open(path, "rt", encoding="utf-8") as stream:
    return json.load(stream)


def write_json_gz(path: Path, value: Any) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  with gzip.open(path, "wt", encoding="utf-8", newline="\n") as stream:
    json.dump(value, stream, ensure_ascii=False, separators=(",", ":"))


def percentile(values: Iterable[float], q: float) -> float | None:
  ordered = sorted(float(value) for value in values)
  if not ordered:
    return None
  if len(ordered) == 1:
    return ordered[0]
  position = (len(ordered)-1)*q
  lower = int(position)
  upper = min(lower+1, len(ordered)-1)
  weight = position-lower
  return ordered[lower]*(1.0-weight)+ordered[upper]*weight


def _member_row(member) -> dict[str, Any]:
  return {
    "raw": int(member.raw_track_id), "slot": int(member.slot),
    "d": float(member.d_rel), "y": float(member.y_rel), "v": float(member.v_rel),
    "age": int(member.age_scans), "measurement_ns": int(member.timestamp_ns),
    "vision": None, "oem": None,
  }


def _object_row(obj) -> dict[str, Any]:
  return {
    "pid": int(obj.physical_track_id),
    "members": [int(member.raw_track_id) for member in obj.members],
    "representative": int(obj.representative_raw_track_id),
    "d": float(obj.d_rel), "y": float(obj.y_rel), "v": float(obj.v_rel),
    "age": int(obj.age_scans), "oem": int(obj.oem_selected),
    "vision": int(obj.vision_supported),
    "member_rows": [_member_row(member) for member in obj.members],
  }


def _lead_row(lead: dict[str, Any] | None) -> dict[str, Any]:
  if not lead:
    return {"status": 0, "track": -1, "d": None, "y": None, "v": None, "a": None,
            "d_path": None, "radar": 0, "model_prob": 0.0}
  return {
    "status": int(bool(lead.get("status"))), "track": int(lead.get("radarTrackId", -1)),
    "d": float(lead.get("dRel", 0.0)), "y": float(lead.get("yRel", 0.0)),
    "v": float(lead.get("vRel", 0.0)), "a": float(lead.get("aLeadK", 0.0)),
    "d_path": float(lead.get("dPath", 0.0)), "radar": int(bool(lead.get("radar"))),
    "model_prob": float(lead.get("modelProb", 0.0)),
  }


def _raw_signature(provider) -> tuple:
  return tuple((int(track.raw_track_id), int(track.slot), int(track.timestamp_ns),
                float(track.d_rel), float(track.y_rel), float(track.v_rel),
                int(track.age_scans), int(bool(track.recovered)),
                -1 if track.previous_slot is None else int(track.previous_slot))
               for track in provider.tracker.last_raw_tracks)


def _group_signature(objects) -> tuple:
  return tuple((int(obj.physical_track_id),
                tuple(int(member.raw_track_id) for member in obj.members),
                int(obj.representative_raw_track_id), float(obj.d_rel),
                float(obj.y_rel), float(obj.v_rel), int(obj.age_scans))
               for obj in objects)


def _qualification_signature(objects) -> tuple:
  return tuple((int(obj.physical_track_id),
                tuple(int(member.raw_track_id) for member in obj.members),
                int(obj.representative_raw_track_id)) for obj in objects)


def _alias_signature(alias: dict[int, int]) -> tuple:
  return tuple(sorted((int(pid), int(public)) for pid, public in alias.items()))


def _alead_signature(estimator, now_ns: int) -> tuple:
  debug = estimator.debug_snapshot(now_ns)
  states = tuple((int(row["physical_pid"]), int(row["source_scan_timestamp_ns"]),
                  int(row["estimator_update_count"]), float(row["estimated_aLead"]),
                  int(row["state_age_ns"]))
                 for row in debug["states"])
  scalar_fields = (
    "last_scan_ns", "reset_count", "last_reset_reason", "last_publication_kind",
    "expired_count", "last_expiry_reason", "update_count", "state_count",
    "peak_state_count", "allocation_count", "held_publication_count",
    "held_update_count", "invalid_input_count", "corrupt_state_count",
    "clock_reset_count", "gap_reset_count",
  )
  return tuple(debug[field] for field in scalar_fields)+(states,)


def _upstream_components(provider, qualified, published, alias, estimator, now_ns: int) -> dict[str, tuple]:
  groups = _group_signature(provider._debug_objects)
  return {
    "raw": _raw_signature(provider),
    "group": tuple(tuple(row[1]) for row in groups),
    "pid": tuple((row[0], tuple(row[1])) for row in groups),
    "member": tuple((row[0], tuple(row[1])) for row in groups),
    "tracking_representative": tuple((row[0], row[2]) for row in groups),
    "qualification": _qualification_signature(qualified),
    "publication_set": tuple(int(obj.physical_track_id) for obj in published),
    "alias": _alias_signature(alias),
    "internal_aLead": _alead_signature(estimator, now_ns),
  }


def _vision_support(member, cues) -> bool:
  return any(math.isfinite(cue.d_rel) and math.isfinite(cue.y_rel) and cue.probability >= .7 and
             abs(member.d_rel-cue.d_rel) <= cue.distance_tolerance_m and
             abs(member.y_rel-cue.y_rel) <= cue.lateral_tolerance_m for cue in cues)


def _predicted_xy(prior, timestamp_ns: int, yaw_rate: float | None) -> tuple[float, float]:
  dt = (timestamp_ns-prior.timestamp_ns)*1e-9
  angle = -(yaw_rate or 0.0)*dt
  dx = prior.d_rel+prior.v_rel*dt
  cosine, sine = math.cos(angle), math.sin(angle)
  return dx*cosine-prior.y_rel*sine, dx*sine+prior.y_rel*cosine


@dataclass
class _SurfaceState:
  raw: int
  members: tuple[int, ...]
  last_ns: int
  recent_tracking: dict[int, int]


class PublicationSurfaceSelector:
  """Causal state that owns only a PID's external surface choice."""

  def __init__(self, policy: str):
    if policy not in POLICIES:
      raise ValueError(policy)
    self.policy = policy
    self.states: dict[int, _SurfaceState] = {}
    self.peak = 0
    self.reasons: Counter[str] = Counter()

  def reset(self) -> None:
    self.states.clear()

  def finish_scan(self, published_pids: set[int], live_pids: set[int]) -> None:
    # Publication gaps are fail-closed lifecycle boundaries: there is no surface
    # to carry while the PID is absent from the exact baseline publication set.
    for pid in tuple(self.states):
      if pid not in published_pids or pid not in live_pids:
        del self.states[pid]
    self.peak = max(self.peak, len(self.states))

  def select(self, obj, *, prior_tracking, timestamp_ns: int, previous_scan_ns: int | None,
             yaw_rate: float | None, cues, oem_slot: int | None) -> tuple[Any, dict[str, Any]]:
    pid = int(obj.physical_track_id)
    members = tuple(int(member.raw_track_id) for member in obj.members)
    by_raw = {int(member.raw_track_id): member for member in obj.members}
    baseline = by_raw[int(obj.representative_raw_track_id)]
    state = self.states.get(pid)
    valid_state = (state is not None and state.members == members and
                   0 <= timestamp_ns-state.last_ns < MAX_GAP_NS)
    prior_surface = by_raw.get(state.raw) if valid_state else None
    def fresh(member) -> bool:
      return previous_scan_ns is None or int(member.timestamp_ns) > previous_scan_ns
    same_members = bool(valid_state)
    baseline_cost = prior_cost = delta = None
    baseline_camera = _vision_support(baseline, cues)
    prior_camera = False if prior_surface is None else _vision_support(prior_surface, cues)
    baseline_oem = oem_slot is not None and int(baseline.slot) == int(oem_slot)
    prior_oem = prior_surface is not None and oem_slot is not None and int(prior_surface.slot) == int(oem_slot)
    support_superiority = (baseline_camera and not prior_camera) or (baseline_oem and not prior_oem)
    jump = (0.0, 0.0, 0.0) if prior_surface is None else (
      abs(float(baseline.d_rel)-float(prior_surface.d_rel)),
      abs(float(baseline.y_rel)-float(prior_surface.y_rel)),
      abs(float(baseline.v_rel)-float(prior_surface.v_rel)))
    if prior_tracking is not None and prior_surface is not None:
      px, py = _predicted_xy(prior_tracking, timestamp_ns, yaw_rate)

      def scalar(member) -> float:
        return (abs(float(member.d_rel)-px)+.5*abs(float(member.y_rel)-py)+
                .5*abs(float(member.v_rel)-float(prior_tracking.v_rel)))

      baseline_cost, prior_cost = scalar(baseline), scalar(prior_surface)
      delta = prior_cost-baseline_cost

    chosen = baseline
    reason = "BASELINE_TRACKING_REP"
    eligible = (len(obj.members) > 1 and same_members and prior_surface is not None and
                fresh(prior_surface) and int(prior_surface.raw_track_id) != int(baseline.raw_track_id) and
                delta is not None)
    near_tie = bool(eligible and delta <= NEAR_TIE+1e-12)
    normalized_p99 = max((jump[i]/JUMP_P99[i] for i in range(3)), default=0.0)
    large_p95 = any(jump[i] >= JUMP_P95[i]-1e-12 for i in range(3))
    recent = {} if state is None else dict(state.recent_tracking)
    recent_challenger = (int(baseline.raw_track_id) in recent and
                         0 <= timestamp_ns-recent[int(baseline.raw_track_id)] <= PINGPONG_HISTORY_NS)

    if self.policy == "P1" and near_tie:
      chosen, reason = prior_surface, "P1_STICKY_NEAR_TIE"
    elif self.policy == "P2" and near_tie and not support_superiority and recent_challenger and normalized_p99 <= 1.0+1e-12:
      chosen, reason = prior_surface, "P2_CAUSAL_PINGPONG"
    elif self.policy == "P3" and near_tie and not support_superiority and large_p95:
      chosen, reason = prior_surface, "P3_LARGE_JUMP_WEAK_ADVANTAGE"
    elif self.policy == "P4" and near_tie and not support_superiority:
      chosen, reason = prior_surface, "P4_SUPPORT_AWARE_STICKY"
    elif self.policy == "P5" and len(obj.members) > 1:
      def medoid(member) -> tuple:
        geometric = sum(abs(float(member.d_rel)-float(other.d_rel))+
                        .5*abs(float(member.y_rel)-float(other.y_rel))+
                        .5*abs(float(member.v_rel)-float(other.v_rel)) for other in obj.members)
        return (geometric, not _vision_support(member, cues),
                oem_slot is None or int(member.slot) != int(oem_slot),
                -int(member.age_scans), int(member.raw_track_id))
      chosen = min(obj.members, key=medoid)
      reason = "P5_CURRENT_MEMBER_MEDOID"
    elif self.policy == "P6" and near_tie and not support_superiority and (
        (recent_challenger and normalized_p99 <= 1.0+1e-12) or large_p95):
      chosen, reason = prior_surface, "P6_NARROW_PINGPONG_OR_JUMP"

    if not fresh(chosen):
      raise AssertionError(f"{self.policy} selected non-fresh raw {chosen.raw_track_id} for PID {pid}")
    recent = {raw: ns for raw, ns in recent.items() if 0 <= timestamp_ns-ns <= PINGPONG_HISTORY_NS}
    recent[int(baseline.raw_track_id)] = timestamp_ns
    if self.policy in STATEFUL_POLICIES:
      self.states[pid] = _SurfaceState(int(chosen.raw_track_id), members, timestamp_ns, recent)
    self.reasons[reason] += 1
    return chosen, {
      "reason": reason, "same_members": int(same_members), "fresh": int(fresh(chosen)),
      "score_delta": delta, "support_superiority": int(support_superiority),
      "baseline_camera": int(baseline_camera), "prior_camera": int(prior_camera),
      "baseline_oem": int(baseline_oem), "prior_oem": int(prior_oem),
      "jump_d": jump[0], "jump_y": jump[1], "jump_v": jump[2],
      "recent_challenger": int(recent_challenger),
    }


def _published_row_and_point(module, obj, member, alias: dict[int, int], a_lead: dict[int, float],
                             now_ns: int, metadata: dict[str, Any]) -> tuple[dict[str, Any], Any]:
  age_s = max(0.0, (now_ns-int(member.timestamp_ns))*1e-9)
  d32 = float(module.np.float32(member.d_rel))
  y32 = float(module.np.float32(member.y_rel))
  v32 = float(module.np.float32(member.v_rel))
  projected_d = float(module.np.float32(d32+v32*age_s))
  acceleration = float(a_lead.get(obj.physical_track_id, math.nan))
  pid = int(obj.physical_track_id)
  row = {
    "pid": pid, "alias": int(alias[pid]), "raw": int(member.raw_track_id),
    "anchor_rep": int(obj.representative_raw_track_id),
    "members": [int(item.raw_track_id) for item in obj.members],
    "d": projected_d, "y": y32, "v": v32,
    "a": acceleration if math.isfinite(acceleration) else None,
    "member_age_s": age_s, **metadata,
  }
  point = SimpleNamespace(
    trackId=int(alias[pid]), radarSource="frontRadar", dRel=projected_d, yRel=y32, vRel=v32,
    aRel=math.nan, yvRel=math.nan, vLead=0.0, aLead=acceleration, jLead=math.nan,
    measured=True, trackState=0,
  )
  return row, point


def _source_current_row(module, obj, alias, acceleration, now_ns: int) -> dict[str, Any]:
  raw, d_rel, y_rel, v_rel = module.bosch_published_surface(obj)
  member = next(item for item in obj.members if int(item.raw_track_id) == int(raw))
  age_s = max(0.0, (now_ns-int(member.timestamp_ns))*1e-9)
  d32 = float(module.np.float32(d_rel))
  y32 = float(module.np.float32(y_rel))
  v32 = float(module.np.float32(v_rel))
  return {
    "pid": int(obj.physical_track_id), "alias": int(alias[obj.physical_track_id]),
    "anchor_rep": int(obj.representative_raw_track_id), "published_raw": int(raw),
    "d": float(module.np.float32(d32+v32*age_s)), "y": y32, "v": v32,
    "a": (float(acceleration.get(obj.physical_track_id, math.nan))
          if math.isfinite(acceleration.get(obj.physical_track_id, math.nan)) else None),
    "member_age_s": age_s,
  }


def _radar_update(controller, *, active_model, model_ns: int, source_ns: int, now_ns: int,
                  v_ego: float, yaw: float | None, points) -> dict[str, dict[str, Any]]:
  model_time_ns = source_ns if source_ns > 0 else model_ns
  model_time_s = model_time_ns*1e-9 if model_time_ns > 0 else now_ns*1e-9
  radar_to_model_s = (model_time_ns-now_ns)*1e-9 if model_time_ns > 0 else 0.0
  output = controller.update(
    time_s=model_time_s, v_ego=float(v_ego), radar_points=points,
    model=_model_stub(active_model), yaw_rate_rad_s=float(yaw or 0.0),
    radar_to_model_time_s=radar_to_model_s,
  )
  return {"lead_one": _lead_row(output.lead_one), "lead_two": _lead_row(output.lead_two)}


def _publication_signature(rows: list[dict[str, Any]]) -> tuple:
  return tuple((row["pid"], row["alias"], row["raw"], row["d"], row["y"], row["v"], row["a"])
               for row in rows)


def _row_surface_signature(row: dict[str, Any] | None) -> tuple | None:
  if row is None:
    return None
  return row["pid"], row["alias"], row["raw"], row["d"], row["y"], row["v"], row["a"]


def _radar_signature(radar: dict[str, dict[str, Any]]) -> tuple:
  fields = ("status", "track", "d", "y", "v", "a", "d_path", "radar", "model_prob")
  return tuple(tuple(radar[name][field] for field in fields) for name in ("lead_one", "lead_two"))


def _planner_signature(radar: dict[str, dict[str, Any]]) -> tuple:
  fields = ("status", "track", "d", "v", "a", "radar")
  return tuple(tuple(radar[name][field] for field in fields) for name in ("lead_one", "lead_two"))


_STABLE_CONTROL_CACHE: dict[str, list[dict[str, Any]]] | None = None


def stable_control_windows() -> dict[str, list[dict[str, Any]]]:
  global _STABLE_CONTROL_CACHE
  if _STABLE_CONTROL_CACHE is None:
    source = (CHECKOUT / "analysis" / "bosch_radar" / "20260924_p1_continuous_multireturn" /
              "tables" / "stable_controls.csv")
    result: defaultdict[str, list[dict[str, Any]]] = defaultdict(list)
    with source.open(encoding="utf-8", newline="") as stream:
      for row in csv.DictReader(stream):
        result[row["key"]].append({
          "pid": int(row["pid"]), "start_scan": int(row["start_scan"]),
          "end_scan": int(row["end_scan"]), "members": row["members"],
        })
    _STABLE_CONTROL_CACHE = dict(result)
  return _STABLE_CONTROL_CACHE


def _effect(base: dict[str, dict[str, Any]], candidate: dict[str, dict[str, Any]]) -> str:
  old = (base["lead_one"], base["lead_two"])
  new = (candidate["lead_one"], candidate["lead_two"])
  old_ids = tuple(row["track"] if row["status"] else -1 for row in old)
  new_ids = tuple(row["track"] if row["status"] else -1 for row in new)
  if old_ids == new_ids:
    return "LEAD_COORDINATE_ONLY"
  old_set = {value for value in old_ids if value >= 0}
  new_set = {value for value in new_ids if value >= 0}
  if new_set-old_set and not old_set-new_set:
    return "LEAD_GAIN"
  if old_set-new_set and not new_set-old_set:
    return "LEAD_LOSS"
  if old_ids == tuple(reversed(new_ids)) and old_set == new_set and old_ids != new_ids:
    return "LEAD1_LEAD2_SWAP"
  return "LEAD_ID_CHANGED"


def _analyze_stream(policy: str, scans: list[dict[str, Any]], base_scans: list[dict[str, Any]],
                    cpu_us: list[float], decision_reasons: Counter[str], state_peak: int) -> dict[str, Any]:
  summary = Counter()
  jumps: list[dict[str, Any]] = []
  dwells: list[dict[str, Any]] = []
  pingpongs: list[dict[str, Any]] = []
  divergence: list[dict[str, Any]] = []
  changed_samples: list[dict[str, Any]] = []
  downstream_samples: list[dict[str, Any]] = []
  active_dwell: dict[int, dict[str, Any]] = {}
  runs: defaultdict[int, list[dict[str, Any]]] = defaultdict(list)
  active_divergence: dict[int, dict[str, Any]] = {}
  previous_rows: dict[int, dict[str, Any]] = {}

  def close_dwell(pid: int) -> None:
    row = active_dwell.pop(pid, None)
    if row is not None:
      row["duration_s"] = (row["end_ns"]-row["start_ns"])*1e-9
      dwells.append(row)
      runs[pid].append(row)

  def close_divergence(pid: int) -> None:
    row = active_divergence.pop(pid, None)
    if row is not None:
      row["duration_s"] = (row["end_ns"]-row["start_ns"])*1e-9
      divergence.append(row)

  for scan, base_scan in zip(scans, base_scans, strict=True):
    now = {int(row["pid"]): row for row in scan["published"]}
    base = {int(row["pid"]): row for row in base_scan["published"]}
    current_pids = set(now)
    for pid in tuple(active_dwell):
      if pid not in current_pids:
        close_dwell(pid)
    for pid in tuple(active_divergence):
      if pid not in current_pids:
        close_divergence(pid)
    changed_scan = _publication_signature(scan["published"]) != _publication_signature(base_scan["published"])
    summary["publication_changed_scans"] += int(changed_scan)
    summary["surface_decision_changed_scans"] += int(any(
      pid in base and row["raw"] != base[pid]["raw"] for pid, row in now.items()))
    summary["stale_selected_surface"] += sum(not int(row["fresh"]) or row["raw"] not in row["members"] for row in now.values())
    if changed_scan:
      if _radar_signature(scan["radar"]) == _radar_signature(base_scan["radar"]):
        summary["NO_DOWNSTREAM_EFFECT"] += 1
      else:
        effect = _effect(base_scan["radar"], scan["radar"])
        summary[effect] += 1
        summary["radarstate_changed_scans"] += 1
        if len(downstream_samples) < 25:
          downstream_samples.append({
            "scan": scan["scan"], "scan_ns": scan["scan_ns"], "policy": policy,
            "effect": effect,
            "baseline_lead1": base_scan["radar"]["lead_one"]["track"],
            "candidate_lead1": scan["radar"]["lead_one"]["track"],
            "baseline_lead2": base_scan["radar"]["lead_two"]["track"],
            "candidate_lead2": scan["radar"]["lead_two"]["track"],
          })
      exposed_a = tuple(base_scan["radar"][name]["a"] for name in ("lead_one", "lead_two")) != tuple(
        scan["radar"][name]["a"] for name in ("lead_one", "lead_two"))
      summary["ALEAD_EXPOSURE_CHANGED"] += int(exposed_a)
      summary["PLANNER_INPUT_CHANGED"] += int(
        _planner_signature(scan["radar"]) != _planner_signature(base_scan["radar"]))

    for pid, row in now.items():
      base_row = base[pid]
      is_different = row["raw"] != base_row["raw"]
      if is_different:
        summary["surface_decisions_different"] += 1
        summary["coordinate_points_different"] += int(
          (row["d"], row["y"], row["v"]) != (base_row["d"], base_row["y"], base_row["v"]))
        summary["vrel_consistency_large_delta"] += int(abs(row["v"]-base_row["v"]) >= JUMP_P99[2])
        if len(changed_samples) < 25:
          changed_samples.append({
            "scan": scan["scan"], "scan_ns": scan["scan_ns"], "pid": pid,
            "baseline_raw": base_row["raw"], "candidate_raw": row["raw"],
            "delta_d": row["d"]-base_row["d"], "delta_y": row["y"]-base_row["y"],
            "delta_v": row["v"]-base_row["v"], "aLead": row["a"], "reason": row["reason"],
          })
        episode = active_divergence.get(pid)
        if episode is None:
          active_divergence[pid] = {"pid": pid, "start_scan": scan["scan"], "end_scan": scan["scan"],
                                    "start_ns": scan["scan_ns"], "end_ns": scan["scan_ns"], "scans": 1}
        else:
          episode["end_scan"] = scan["scan"]
          episode["end_ns"] = scan["scan_ns"]
          episode["scans"] += 1
      else:
        close_divergence(pid)

      previous = previous_rows.get(pid)
      dwell = active_dwell.get(pid)
      members_label = "|".join(map(str, row["members"]))
      if dwell is None or dwell["raw"] != row["raw"]:
        if dwell is not None:
          close_dwell(pid)
        active_dwell[pid] = {"pid": pid, "raw": row["raw"], "members": members_label,
                             "start_scan": scan["scan"], "end_scan": scan["scan"],
                             "start_ns": scan["scan_ns"], "end_ns": scan["scan_ns"], "scans": 1,
                             "stable_members": 1}
      else:
        dwell["stable_members"] = int(bool(dwell["stable_members"]) and dwell["members"] == members_label)
        dwell["end_scan"] = scan["scan"]
        dwell["end_ns"] = scan["scan_ns"]
        dwell["scans"] += 1
      if previous is not None and previous["raw"] != row["raw"]:
        stable = previous["members"] == row["members"]
        summary["surface_switches"] += 1
        summary["stable_group_switches"] += int(stable)
        jumps.append({
          "pid": pid, "scan": scan["scan"], "scan_ns": scan["scan_ns"], "stable_members": int(stable),
          "old_raw": previous["raw"], "new_raw": row["raw"],
          "abs_delta_d": abs(row["d"]-previous["d"]),
          "abs_delta_y": abs(row["y"]-previous["y"]),
          "abs_delta_v": abs(row["v"]-previous["v"]),
        })
    previous_rows = now

  for pid in tuple(active_dwell):
    close_dwell(pid)
  for pid in tuple(active_divergence):
    close_divergence(pid)
  for pid, pid_runs in runs.items():
    for index in range(len(pid_runs)-2):
      a, b, c = pid_runs[index:index+3]
      if a["raw"] == c["raw"] and a["raw"] != b["raw"]:
        pingpongs.append({
          "pid": pid, "pattern": "A_B_A", "start_scan": a["end_scan"], "end_scan": c["start_scan"],
          "duration_s": (c["start_ns"]-a["end_ns"])*1e-9,
          "stable_members": int(a["stable_members"] and b["stable_members"] and c["stable_members"] and
                                a["members"] == b["members"] == c["members"]),
        })
    for index in range(len(pid_runs)-3):
      a, b, c, d = pid_runs[index:index+4]
      if a["raw"] == c["raw"] and b["raw"] == d["raw"] and a["raw"] != b["raw"]:
        pingpongs.append({
          "pid": pid, "pattern": "A_B_A_B", "start_scan": a["end_scan"], "end_scan": d["start_scan"],
          "duration_s": (d["start_ns"]-a["end_ns"])*1e-9,
          "stable_members": int(all(row["stable_members"] for row in (a, b, c, d)) and
                                len({a["members"], b["members"], c["members"], d["members"]}) == 1),
        })
  summary["A_B_A"] = sum(row["pattern"] == "A_B_A" for row in pingpongs)
  summary["A_B_A_B"] = sum(row["pattern"] == "A_B_A_B" for row in pingpongs)
  summary["stable_A_B_A_le_0_5s"] = sum(
    row["pattern"] == "A_B_A" and row["stable_members"] and row["duration_s"] <= .5 for row in pingpongs)
  for scans_count in (1, 2, 3):
    summary[f"dwell_{scans_count}_scan"] = sum(row["scans"] == scans_count for row in dwells)
  return {
    "summary": dict(summary), "jumps": jumps, "dwells": dwells, "pingpongs": pingpongs,
    "divergence": divergence, "changed_samples": changed_samples,
    "downstream_samples": downstream_samples, "cpu_us": cpu_us,
    "decision_reasons": dict(decision_reasons), "state_peak": int(state_peak),
  }


def run_segment(key: str, *, downstream: bool = True, cutoff_ns: int | None = None,
                include_streams: bool = False) -> dict[str, Any]:
  """Run one exact baseline provider and all post-assignment surface policies."""
  configure_imports()
  module = load_policy_module("BASELINE", name=f"post_surface_{key.replace('-', '_')}_{time.time_ns()}")
  from opendbc.car.carlog import carlog, researchlog
  module.carlog, module.researchlog = carlog, researchlog
  from openpilot.selfdrive.carrot.radar_motion import DPathRadarController

  context = load_segment(key)
  provider = module.BoschRadarProvider(1, qualification=True)
  estimator = module.BoschLeadAccelerationEstimator()
  selectors = {policy: PublicationSurfaceSelector(policy) for policy in POLICIES}
  base_controller = (DPathRadarController(enable_radar_tracks=1,
                                          front_radar_measurement_delay_s=0.0,
                                          production_live_tracks=True)
                     if downstream else None)
  candidate_controllers: dict[str, Any] = {}
  streams: dict[str, list[dict[str, Any]]] = {policy: [] for policy in POLICIES}
  cpu: dict[str, list[float]] = {policy: [] for policy in POLICIES}
  mismatch = {policy: Counter() for policy in POLICIES}
  baseline_parity = Counter()
  source_current_different_points = 0
  source_current_different_scans = 0
  source_current_radar_changed = 0
  previous_tracking: dict[int, Any] = {}
  previous_scan_ns: int | None = None
  prior_cache_path = PRIOR / "scratch" / "baseline" / f"{key}.json.gz"
  prior_cache = read_json_gz(prior_cache_path)["run"]["snapshots"] if prior_cache_path.exists() else []

  can_times = [item[0] for item in context.can]
  pose_times = [item[0] for item in context.poses]
  model_times = [item[0] for item in context.models]
  cursor = 0
  last_now = 0
  scan_index = 0
  for state_ns, v_ego, _a_ego, _steering in context.states:
    if cutoff_ns is not None and state_ns > cutoff_ns:
      break
    batch = []
    while cursor < len(context.can) and can_times[cursor] <= state_ns:
      if cutoff_ns is not None and context.can[cursor][0] > cutoff_ns:
        break
      batch.append(context.can[cursor])
      cursor += 1
    now_ns = max(state_ns, batch[-1][0] if batch else 0, last_now)
    if cutoff_ns is not None and now_ns > cutoff_ns:
      break
    last_now = now_ns
    pose = _latest(pose_times, context.poses, now_ns)
    yaw = None
    if pose is not None and 0 <= now_ns-pose[0] <= CONTEXT_MAX_AGE_NS and pose[2]:
      yaw = -pose[1]
    model = _latest(model_times, context.models, now_ns)
    active_model = model if model is not None and 0 <= now_ns-model[0] <= CONTEXT_MAX_AGE_NS else None
    cues, path, model_ns, source_ns = (), (), 0, 0
    if active_model is not None:
      model_ns = active_model[0]
      if active_model[1] is not None:
        lead = active_model[1]
        cues = (module.BoschVisionCue(lead["x"]-1.52, -lead["y"], lead["prob"]),)
      path = tuple(zip(active_model[2][0], active_model[2][1], strict=True))
      source_ns = active_model[3]
    flat = [(timestamp, [(address, data, source) for address, data, source in messages])
            for timestamp, messages in batch]
    qualified = provider.update(flat, now_ns=now_ns, v_ego=v_ego, yaw_rate_left=yaw,
                                vision=cues, path=path, path_ns=model_ns or None,
                                path_source_ns=source_ns)
    if qualified is None:
      continue
    scan_ns = int(provider.last_scan_timestamp_ns)
    alias = provider.publication_aliases.update(
      now_ns, (obj.physical_track_id for obj in qualified), provider.tracker.group_manager.states)
    acceleration = estimator.update(qualified, scan_ns, v_ego, provider.tracker.group_manager.states)
    published = provider.publication_view(qualified, now_ns)
    before = _upstream_components(provider, qualified, published, alias, estimator, now_ns)
    published_pids = {int(obj.physical_track_id) for obj in published}
    live_pids = {int(pid) for pid in provider.tracker.group_manager.states}
    source_rows = [_source_current_row(module, obj, alias, acceleration, now_ns) for obj in published]
    current_by_pid = {int(obj.physical_track_id): obj for obj in provider._debug_objects}

    order = POLICIES[scan_index % len(POLICIES):]+POLICIES[:scan_index % len(POLICIES)]
    scan_outputs = {}
    scan_points = {}
    for policy in order:
      selector = selectors[policy]
      started = time.perf_counter_ns()
      rows, points = [], []
      for obj in published:
        member, metadata = selector.select(
          obj, prior_tracking=previous_tracking.get(int(obj.physical_track_id)),
          timestamp_ns=scan_ns, previous_scan_ns=previous_scan_ns, yaw_rate=yaw,
          cues=cues, oem_slot=provider.last_oem_slot)
        row, point = _published_row_and_point(module, obj, member, alias, acceleration, now_ns, metadata)
        point.vLead = float(v_ego)+point.vRel
        rows.append(row)
        points.append(point)
      selector.finish_scan(published_pids, live_pids)
      cpu[policy].append((time.perf_counter_ns()-started)/1000.0)
      scan_outputs[policy] = {"scan": scan_index, "scan_ns": scan_ns, "now_ns": int(now_ns),
                              "published": rows,
                              "radar": {"lead_one": _lead_row(None), "lead_two": _lead_row(None)}}
      scan_points[policy] = points
      after_policy = _upstream_components(provider, qualified, published, alias, estimator, now_ns)
      for field in before:
        mismatch[policy][field] += int(before[field] != after_policy[field])

    if downstream:
      p0_signature = _publication_signature(scan_outputs["P0"]["published"])
      newly_divergent = [policy for policy in POLICIES[1:]
                         if policy not in candidate_controllers and
                         _publication_signature(scan_outputs[policy]["published"]) != p0_signature]
      base_before = copy.deepcopy(base_controller) if newly_divergent else None
      scan_outputs["P0"]["radar"] = _radar_update(
        base_controller, active_model=active_model, model_ns=model_ns, source_ns=source_ns,
        now_ns=now_ns, v_ego=v_ego, yaw=yaw, points=scan_points["P0"])
      for policy in POLICIES[1:]:
        controller = candidate_controllers.get(policy)
        if controller is None and policy not in newly_divergent:
          scan_outputs[policy]["radar"] = scan_outputs["P0"]["radar"]
          continue
        if controller is None:
          controller = copy.deepcopy(base_before)
          candidate_controllers[policy] = controller
        scan_outputs[policy]["radar"] = _radar_update(
          controller, active_model=active_model, model_ns=model_ns, source_ns=source_ns,
          now_ns=now_ns, v_ego=v_ego, yaw=yaw, points=scan_points[policy])

    for policy in POLICIES:
      streams[policy].append(scan_outputs[policy])
    p0_by_pid = {row["pid"]: row for row in scan_outputs["P0"]["published"]}
    current_diff = sum(row["published_raw"] != p0_by_pid[row["pid"]]["raw"] or
                       (row["d"], row["y"], row["v"]) !=
                       (p0_by_pid[row["pid"]]["d"], p0_by_pid[row["pid"]]["y"], p0_by_pid[row["pid"]]["v"])
                       for row in source_rows)
    source_current_different_points += current_diff
    source_current_different_scans += int(bool(current_diff))

    if scan_index < len(prior_cache):
      reference = prior_cache[scan_index]
      current_objects = [_object_row(obj) for obj in qualified]
      baseline_parity["scan"] += int(int(reference["scan_ns"]) != scan_ns)
      baseline_parity["qualification"] += int(reference["objects"] != current_objects)
      ref_pub = reference["published"]
      baseline_parity["publication"] += int(ref_pub != source_rows)
      if downstream and _radar_signature(reference["radar"]) != _radar_signature(scan_outputs["P0"]["radar"]):
        source_current_radar_changed += 1
    else:
      baseline_parity["scan_count"] += 1
    previous_tracking = current_by_pid
    previous_scan_ns = scan_ns
    scan_index += 1

  baseline_parity["scan_count"] += abs(len(prior_cache)-scan_index)
  analyses = {}
  for policy in POLICIES:
    analyses[policy] = _analyze_stream(
      policy, streams[policy], streams["P0"], cpu[policy], selectors[policy].reasons, selectors[policy].peak)
    analyses[policy]["invariance"] = dict(mismatch[policy])
    if include_streams:
      analyses[policy]["stream"] = streams[policy]
  controls: dict[str, Any] = {}
  if key == ROUTE280_KEY:
    target = 1000004
    base_first = next((scan["scan_ns"] for scan in streams["P0"]
                       if target in {row["pid"] for row in scan["published"]}), None)
    controls["route280"] = {}
    for policy in POLICIES:
      first = next((scan["scan_ns"] for scan in streams[policy]
                    if target in {row["pid"] for row in scan["published"]}), None)
      changed = 0
      for base_scan, current_scan in zip(streams["P0"], streams[policy], strict=True):
        base = {row["pid"]: row for row in base_scan["published"]}.get(target)
        current = {row["pid"]: row for row in current_scan["published"]}.get(target)
        changed += _row_surface_signature(base) != _row_surface_signature(current)
      controls["route280"][policy] = {
        "first_publication_delay_ms": (None if base_first is None or first is None
                                        else (first-base_first)*1e-6),
        "target_coordinate_changed_scans": changed,
      }
  if key in P0_CONTROLS:
    target = P0_CONTROLS[key]
    controls["p0"] = {}
    base_count = sum(target in {row["pid"] for row in scan["published"]} for scan in streams["P0"])
    for policy in POLICIES:
      current_count = sum(target in {row["pid"] for row in scan["published"]}
                          for scan in streams[policy])
      controls["p0"][policy] = {"publication_duration_delta_scans": current_count-base_count}
  windows = stable_control_windows().get(key, [])
  if windows:
    controls["multi_return"] = {}
    for policy in POLICIES:
      switches = divergence_scans = 0
      for control in windows:
        prior_raw = None
        for index in range(control["start_scan"], min(control["end_scan"]+1, len(streams[policy]))):
          current = {row["pid"]: row for row in streams[policy][index]["published"]}.get(control["pid"])
          baseline = {row["pid"]: row for row in streams["P0"][index]["published"]}.get(control["pid"])
          if current is None:
            prior_raw = None
            continue
          switches += int(prior_raw is not None and prior_raw != current["raw"])
          divergence_scans += int(baseline is not None and baseline["raw"] != current["raw"])
          prior_raw = current["raw"]
      controls["multi_return"][policy] = {
        "controls": len(windows), "surface_switches": switches,
        "baseline_different_scans": divergence_scans,
      }
  return {
    "key": key, "scans": scan_index, "policies": analyses,
    "baseline_parity": dict(baseline_parity),
    "source_current": {
      "different_points_vs_P0": source_current_different_points,
      "different_scans_vs_P0": source_current_different_scans,
      "radar_changed_scans_vs_P0": source_current_radar_changed,
    },
    "controls": controls,
  }


def surface_signatures(key: str, *, cutoff_ns: int | None = None) -> dict[str, list[tuple]]:
  result = run_segment(key, downstream=False, cutoff_ns=cutoff_ns, include_streams=True)
  signatures = {}
  for policy in POLICIES:
    signatures[policy] = [
      (int(scan["scan_ns"]), tuple((int(row["pid"]), int(row["raw"]), float(row["d"]),
                                    float(row["y"]), float(row["v"]), row["a"])
                                   for row in scan["published"]))
      for scan in result["policies"][policy].pop("stream")]
  return signatures


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("key")
  parser.add_argument("--no-downstream", action="store_true")
  parser.add_argument("--output", type=Path)
  args = parser.parse_args()
  ensure_dirs()
  result = run_segment(args.key, downstream=not args.no_downstream)
  destination = args.output or result_path(args.key)
  write_json_gz(destination, result)
  print({"key": args.key, "scans": result["scans"],
         "publication_changed": {p: result["policies"][p]["summary"].get("publication_changed_scans", 0)
                                 for p in POLICIES},
         "baseline_parity": result["baseline_parity"]})


if __name__ == "__main__":
  main()
