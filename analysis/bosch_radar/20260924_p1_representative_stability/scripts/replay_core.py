# ruff: noqa: TID251
"""Segment-local current-source replay used by all study phases."""
from __future__ import annotations

import bisect
import math
import time
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Any

from common import DATA, configure_imports
from representative_modules import load_policy_module


CONTEXT_MAX_AGE_NS = 200_000_000


@dataclass(frozen=True)
class SegmentContext:
  can: tuple
  states: tuple
  poses: tuple
  models: tuple


def load_segment(key: str) -> SegmentContext:
  """Read the rlog once and retain only inputs used by provider/downstream replay."""
  configure_imports()
  from rlog_io import events

  can, states, poses, models = [], [], [], []
  for event in events(DATA / f"{key}--rlog.zst"):
    which = event.which()
    timestamp_ns = int(event.logMonoTime)
    if which == "can":
      messages = tuple((int(msg.address), bytes(msg.dat), int(msg.src)) for msg in event.can)
      if messages:
        can.append((timestamp_ns, messages))
    elif which == "carState":
      state = event.carState
      states.append((timestamp_ns, float(state.vEgo), float(state.aEgo), float(state.steeringAngleDeg)))
    elif which == "livePose":
      pose = event.livePose
      angular = getattr(pose, "angularVelocityDevice", None)
      valid = bool(angular is not None and angular.valid and math.isfinite(angular.z)
                   and pose.inputsOK and pose.sensorsOK)
      poses.append((timestamp_ns, float(angular.z) if valid else 0.0, valid))
    elif which == "modelV2":
      model = event.modelV2
      lead = None
      if model.leadsV3:
        source = model.leadsV3[0]
        if source.x and source.y and source.v:
          lead = {
            "x": float(source.x[0]), "y": float(source.y[0]), "v": float(source.v[0]),
            "a": float(source.a[0]) if source.a else 0.0, "prob": float(source.prob),
            "x_std": float(source.xStd[0]) if source.xStd else 1.0,
            "y_std": float(source.yStd[0]) if source.yStd else 1.0,
            "v_std": float(source.vStd[0]) if source.vStd else 1.0,
          }
      position = getattr(model, "position", None)
      path = ((tuple(float(value) for value in position.x), tuple(float(value) for value in position.y))
              if position is not None else ((), ()))
      models.append((timestamp_ns, lead, path, int(getattr(model, "timestampEof", 0) or 0)))
  return SegmentContext(tuple(can), tuple(states), tuple(poses), tuple(models))


def _latest(times: list[int], items: tuple, now_ns: int):
  index = bisect.bisect_right(times, now_ns) - 1
  return items[index] if index >= 0 else None


def _model_stub(model) -> Any:
  if model is None:
    return SimpleNamespace(position=SimpleNamespace(x=(), y=()), leadsV3=())
  lead = model[1]
  leads = ()
  if lead is not None:
    leads = (SimpleNamespace(
      x=(lead["x"],), y=(lead["y"],), v=(lead["v"],), a=(lead["a"],),
      xStd=(lead["x_std"],), yStd=(lead["y_std"],), vStd=(lead["v_std"],),
      prob=lead["prob"],
    ),)
  return SimpleNamespace(
    position=SimpleNamespace(x=model[2][0], y=model[2][1]),
    leadsV3=leads,
  )


def _member_dict(member, *, supported: bool | None = None, oem: bool | None = None) -> dict:
  return {
    "raw": int(member.raw_track_id), "slot": int(member.slot),
    "d": float(member.d_rel), "y": float(member.y_rel), "v": float(member.v_rel),
    "age": int(member.age_scans), "measurement_ns": int(member.timestamp_ns),
    "vision": None if supported is None else int(supported),
    "oem": None if oem is None else int(oem),
  }


def _object_dict(obj) -> dict:
  return {
    "pid": int(obj.physical_track_id),
    "members": tuple(int(member.raw_track_id) for member in obj.members),
    "representative": int(obj.representative_raw_track_id),
    "d": float(obj.d_rel), "y": float(obj.y_rel), "v": float(obj.v_rel),
    "age": int(obj.age_scans), "oem": int(obj.oem_selected),
    "vision": int(obj.vision_supported),
    "member_rows": tuple(_member_dict(member) for member in obj.members),
  }


def _published_point(module, obj, alias: dict[int, int], a_lead: dict[int, float], now_ns: int) -> tuple[dict, Any]:
  raw_id, d_rel, y_rel, v_rel = module.bosch_published_surface(obj)
  member = next(member for member in obj.members if member.raw_track_id == raw_id)
  age_s = max(0.0, (now_ns - member.timestamp_ns) * 1e-9)
  # Match bosch_fill_point's Float32 write/read before its age projection.
  d32 = float(module.np.float32(d_rel))
  y32 = float(module.np.float32(y_rel))
  v32 = float(module.np.float32(v_rel))
  projected_d = float(module.np.float32(d32 + v32 * age_s))
  acceleration = float(a_lead.get(obj.physical_track_id, math.nan))
  track_id = int(alias[obj.physical_track_id])
  row = {
    "pid": int(obj.physical_track_id), "alias": track_id,
    "anchor_rep": int(obj.representative_raw_track_id), "published_raw": int(raw_id),
    "d": projected_d, "y": y32, "v": v32,
    "a": acceleration if math.isfinite(acceleration) else None,
    "member_age_s": age_s,
  }
  point = SimpleNamespace(
    trackId=track_id, radarSource="frontRadar", dRel=projected_d, yRel=y32, vRel=v32,
    aRel=math.nan, yvRel=math.nan, vLead=0.0, aLead=acceleration, jLead=math.nan,
    measured=True, trackState=0,
  )
  return row, point


def _lead_dict(lead: dict[str, Any] | None) -> dict:
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


_POLICY_MODULE_CACHE: dict[tuple, Any] = {}


def run_policy(key: str, context: SegmentContext, policy: str, *, config: dict[str, Any] | None = None,
               downstream: bool = True) -> dict:
  """Replay one policy from a fresh provider and optional current DPathRadarController."""
  configure_imports()
  module_key = (policy, tuple(sorted((config or {}).items())))
  module = _POLICY_MODULE_CACHE.get(module_key)
  if module is None:
    module = load_policy_module(policy, name=f"rep_cached_{policy.lower()}_{len(_POLICY_MODULE_CACHE)}",
                                config=config)
    _POLICY_MODULE_CACHE[module_key] = module
  module.BOSCH_P1_REP_TRACE.clear()
  from opendbc.car.carlog import carlog, researchlog
  module.carlog, module.researchlog = carlog, researchlog
  controller = None
  if downstream:
    from openpilot.selfdrive.carrot.radar_motion import DPathRadarController
    controller = DPathRadarController(enable_radar_tracks=1, front_radar_measurement_delay_s=0.0,
                                      production_live_tracks=True)

  provider = module.BoschRadarProvider(1, qualification=True)
  estimator = module.BoschLeadAccelerationEstimator()
  can_times = [item[0] for item in context.can]
  pose_times = [item[0] for item in context.poses]
  model_times = [item[0] for item in context.models]
  cursor = 0
  last_now = 0
  snapshots = []
  timings_us = []

  for state_ns, v_ego, a_ego, steering in context.states:
    batch = []
    while cursor < len(context.can) and can_times[cursor] <= state_ns:
      batch.append(context.can[cursor])
      cursor += 1
    now_ns = max(state_ns, batch[-1][0] if batch else 0, last_now)
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
        cues = (module.BoschVisionCue(lead["x"] - 1.52, -lead["y"], lead["prob"]),)
      path = tuple(zip(active_model[2][0], active_model[2][1], strict=True))
      source_ns = active_model[3]

    flat = [(timestamp, [(address, data, source) for address, data, source in messages])
            for timestamp, messages in batch]
    started = time.perf_counter_ns()
    qualified = provider.update(flat, now_ns=now_ns, v_ego=v_ego, yaw_rate_left=yaw,
                                vision=cues, path=path, path_ns=model_ns or None,
                                path_source_ns=source_ns)
    timings_us.append((time.perf_counter_ns() - started) / 1000.0)
    if qualified is None:
      continue

    scan_ns = int(provider.last_scan_timestamp_ns)
    alias = provider.publication_aliases.update(
      now_ns, (obj.physical_track_id for obj in qualified), provider.tracker.group_manager.states)
    acceleration = estimator.update(qualified, scan_ns, v_ego, provider.tracker.group_manager.states)
    published_objects = provider.publication_view(qualified, now_ns)
    published_rows, points = [], []
    for obj in published_objects:
      row, point = _published_point(module, obj, alias, acceleration, now_ns)
      point.vLead = float(v_ego) + point.vRel
      published_rows.append(row)
      points.append(point)

    radar = {"lead_one": _lead_dict(None), "lead_two": _lead_dict(None)}
    if controller is not None:
      model_time_ns = source_ns if source_ns > 0 else model_ns
      model_time_s = model_time_ns * 1e-9 if model_time_ns > 0 else now_ns * 1e-9
      radar_to_model_s = (model_time_ns-now_ns) * 1e-9 if model_time_ns > 0 else 0.0
      output = controller.update(
        time_s=model_time_s, v_ego=float(v_ego), radar_points=points,
        model=_model_stub(active_model), yaw_rate_rad_s=float(yaw or 0.0),
        radar_to_model_time_s=radar_to_model_s,
      )
      radar = {"lead_one": _lead_dict(output.lead_one), "lead_two": _lead_dict(output.lead_two)}

    snapshots.append({
      "scan": len(snapshots), "now_ns": int(now_ns), "scan_ns": scan_ns,
      "v_ego": float(v_ego), "a_ego": float(a_ego), "steering": float(steering),
      "yaw": float(yaw or 0.0), "model_ns": int(model_ns), "model_source_ns": int(source_ns),
      "objects": tuple(_object_dict(obj) for obj in qualified),
      "published": tuple(published_rows), "radar": radar,
      "state_peak": int(getattr(provider.tracker.group_manager, "_p1_rep_state_peak", 0)),
    })

  return {
    "key": key, "policy": policy, "snapshots": snapshots,
    "selection_trace": list(module.BOSCH_P1_REP_TRACE),
    "timings_us": timings_us,
    "state_peak": max((snapshot["state_peak"] for snapshot in snapshots), default=0),
  }
