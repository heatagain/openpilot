# ruff: noqa: ISC002, TID251
"""Compile analysis-only representative policies from the exact current source.

Only `_bosch_group_representative` and research instrumentation attached to the
group manager are substituted in memory. No generated provider is written.
"""
from __future__ import annotations

import ast
import sys
import types
from pathlib import Path
from typing import Any

from common import SOURCE, configure_imports


POLICIES = ("BASELINE", "E", "E1", "E2", "E3", "E4", "E5")


def _replace_once(source: str, old: str, new: str, label: str) -> str:
  count = source.count(old)
  if count != 1:
    raise RuntimeError(f"source drift at {label}: expected 1 exact match, found {count}")
  return source.replace(old, new, 1)


def _representative_function() -> str:
  return '''def _bosch_group_representative(candidates, raw_index, vision_supported, oem_slot, *,
                                prior=None, predicted_xy=None, policy_state=None,
                                timestamp_ns=None):
  """Production cost plus analysis-only causal representative policies."""
  if len(candidates) == 1:
    return candidates[0]
  if prior is not None:
    px, py = predicted_xy

    def cost(member):
      continuity = (abs(member.d_rel-px) + .5*abs(member.y_rel-py) +
                    .5*abs(member.v_rel-prior.v_rel))
      return (continuity, member.raw_track_id != prior.representative_raw_track_id,
              not vision_supported[raw_index[member.raw_track_id]], member.slot != oem_slot,
              -member.age_scans, member.raw_track_id)
  else:
    ordered = sorted(member.d_rel for member in candidates)
    middle = len(ordered)//2
    median = ordered[middle] if len(ordered) % 2 else (ordered[middle-1]+ordered[middle])/2

    def cost(member):
      return (abs(member.d_rel-median), False,
              not vision_supported[raw_index[member.raw_track_id]], member.slot != oem_slot,
              -member.age_scans, member.raw_track_id)

  best = min(candidates, key=cost)
  chosen = best
  reason = 'BASELINE_BEST'
  prior_member = None if prior is None else next(
    (member for member in candidates if member.raw_track_id == prior.representative_raw_track_id), None)
  members = tuple(member.raw_track_id for member in candidates)
  physical_prior = prior is not None and hasattr(prior, 'members') and hasattr(prior, 'physical_track_id')
  prior_members = (() if not physical_prior else tuple(member.raw_track_id for member in prior.members))
  same_members = physical_prior and members == prior_members
  delta = None if prior_member is None else cost(prior_member)[0] - cost(best)[0]
  best_vision = bool(vision_supported[raw_index[best.raw_track_id]])
  prior_vision = bool(prior_member is not None and vision_supported[raw_index[prior_member.raw_track_id]])
  best_oem = oem_slot is not None and best.slot == oem_slot
  prior_oem = prior_member is not None and oem_slot is not None and prior_member.slot == oem_slot
  support_superiority = (best_vision and not prior_vision) or (best_oem and not prior_oem)
  jump_d = 0. if prior_member is None else abs(best.d_rel-prior_member.d_rel)
  jump_y = 0. if prior_member is None else abs(best.y_rel-prior_member.y_rel)
  jump_v = 0. if prior_member is None else abs(best.v_rel-prior_member.v_rel)
  pid = None if not physical_prior else prior.physical_track_id
  now_ns = int(timestamp_ns or max(member.timestamp_ns for member in candidates))
  state = None if policy_state is None or pid is None else policy_state.get(pid)
  if state is not None and not 0 <= now_ns-state.get('last_ns', now_ns) <= 300_000_000:
    policy_state.pop(pid, None)
    state = None

  if prior_member is not None and best.raw_track_id != prior_member.raw_track_id:
    if BOSCH_P1_REP_POLICY == 'E' and delta <= .25 + 1e-12:
      chosen, reason = prior_member, 'E_WITHIN_025'
    elif BOSCH_P1_REP_POLICY == 'E1' and policy_state is not None and same_members and delta <= BOSCH_P1_REP_THRESHOLD + 1e-12 and not support_superiority:
      recent = {} if state is None or state.get('members') != members else state.get('recent', {})
      challenger_ns = recent.get(best.raw_track_id)
      if challenger_ns is not None and 0 <= now_ns-challenger_ns <= BOSCH_P1_REP_PINGPONG_NS:
        chosen, reason = prior_member, 'E1_RECENT_REP_PINGPONG'
    elif BOSCH_P1_REP_POLICY == 'E2' and policy_state is not None and same_members and delta <= BOSCH_P1_REP_HARD_THRESHOLD + 1e-12 and not support_superiority:
      if state is not None and state.get('members') == members and state.get('challenger') == best.raw_track_id:
        count = state.get('challenger_count', 0) + 1
      else:
        count = 1
      if count < BOSCH_P1_REP_CONFIRM_SCANS:
        chosen, reason = prior_member, 'E2_AWAIT_CONFIRMATION'
    elif BOSCH_P1_REP_POLICY == 'E3' and policy_state is not None and same_members and delta <= BOSCH_P1_REP_THRESHOLD + 1e-12 and not support_superiority:
      chosen, reason = prior_member, 'E3_NO_SUPPORT_CHANGE'
    elif BOSCH_P1_REP_POLICY == 'E4' and policy_state is not None and same_members and delta <= BOSCH_P1_REP_THRESHOLD + 1e-12 and not support_superiority and (
        jump_d >= BOSCH_P1_REP_JUMP_D or jump_y >= BOSCH_P1_REP_JUMP_Y or jump_v >= BOSCH_P1_REP_JUMP_V):
      chosen, reason = prior_member, 'E4_LARGE_JUMP_WEAK_ADVANTAGE'

  if BOSCH_P1_REP_POLICY == 'E5' and policy_state is not None:
    def medoid(member):
      geometric = sum(abs(member.d_rel-other.d_rel) + .5*abs(member.y_rel-other.y_rel) +
                      .5*abs(member.v_rel-other.v_rel) for other in candidates)
      return (geometric, not vision_supported[raw_index[member.raw_track_id]],
              member.slot != oem_slot, -member.age_scans, member.raw_track_id)
    chosen = min(candidates, key=medoid)
    reason = 'E5_CURRENT_MEDOID'

  if policy_state is not None and pid is not None and BOSCH_P1_REP_POLICY in ('E1', 'E2'):
    if state is None or state.get('members') != members or not 0 <= now_ns-state.get('last_ns', now_ns) <= 300_000_000:
      recent = {}
      dwell = 1
    else:
      recent = dict(state.get('recent', {}))
      dwell = state.get('dwell', 0) + 1 if prior.representative_raw_track_id == chosen.raw_track_id else 1
    recent[chosen.raw_track_id] = now_ns
    recent = {raw_id: ns for raw_id, ns in recent.items() if 0 <= now_ns-ns <= BOSCH_P1_REP_PINGPONG_NS}
    challenger_count = 0
    challenger = None
    if best.raw_track_id != prior.representative_raw_track_id and same_members:
      challenger = best.raw_track_id
      challenger_count = ((state or {}).get('challenger_count', 0) + 1
                          if state is not None and state.get('members') == members and state.get('challenger') == challenger else 1)
    policy_state[pid] = {'members': members, 'recent': recent, 'last_ns': now_ns,
                         'dwell': dwell, 'challenger': challenger,
                         'challenger_count': challenger_count}

  BOSCH_P1_REP_TRACE.append({
    'scan_ns': now_ns, 'pid': pid, 'members': members, 'prior_members': prior_members,
    'same_members': same_members, 'prior_rep': None if prior is None else prior.representative_raw_track_id,
    'baseline_best': best.raw_track_id, 'chosen': chosen.raw_track_id,
    'best_cost': cost(best)[0], 'prior_cost': None if prior_member is None else cost(prior_member)[0],
    'score_delta': delta, 'best_vision': best_vision, 'prior_vision': prior_vision,
    'best_oem': best_oem, 'prior_oem': prior_oem, 'support_superiority': support_superiority,
    'jump_d': jump_d, 'jump_y': jump_y, 'jump_v': jump_v, 'reason': reason,
    'candidates': tuple((member.raw_track_id, member.slot, member.d_rel, member.y_rel,
                         member.v_rel, member.age_scans, cost(member)[0],
                         bool(vision_supported[raw_index[member.raw_track_id]]), member.slot == oem_slot)
                        for member in candidates),
  })
  return chosen


'''


def _instrument(source: str) -> str:
  start = source.index("def _bosch_group_representative(")
  end = source.index("def _bosch_physical_component", start)
  source = source[:start] + _representative_function() + source[end:]
  source = _replace_once(
    source,
    "@dataclass(frozen=True)\nclass BoschVisionCue:\n",
    "BOSCH_P1_REP_POLICY = 'BASELINE'\n"
    "BOSCH_P1_REP_THRESHOLD = .25\n"
    "BOSCH_P1_REP_HARD_THRESHOLD = .25\n"
    "BOSCH_P1_REP_CONFIRM_SCANS = 2\n"
    "BOSCH_P1_REP_PINGPONG_NS = 2_000_000_000\n"
    "BOSCH_P1_REP_JUMP_D = 4.5\n"
    "BOSCH_P1_REP_JUMP_Y = 1.09375\n"
    "BOSCH_P1_REP_JUMP_V = .5\n"
    "BOSCH_P1_REP_TRACE = []\n\n\n"
    "@dataclass(frozen=True)\nclass BoschVisionCue:\n",
    "policy globals",
  )
  source = _replace_once(
    source,
    "    self.last_decisions: tuple[BoschAssociationDecision, ...] = ()\n",
    "    self.last_decisions: tuple[BoschAssociationDecision, ...] = ()\n"
    "    self._p1_rep_policy_state = {}\n"
    "    self._p1_rep_state_peak = 0\n",
    "policy state",
  )
  source = _replace_once(
    source,
    "      if size == 1:\n"
    "        rep = candidates[0]\n"
    "        oem_selected = rep.detection.slot == oem_slot\n",
    "      if size == 1:\n"
    "        rep = candidates[0]\n"
    "        if prior is not None:\n"
    "          self._p1_rep_policy_state.pop(prior.physical_track_id, None)\n"
    "        oem_selected = rep.detection.slot == oem_slot\n",
    "singleton state reset",
  )
  source = _replace_once(
    source,
    "          candidates, raw_index, vision_supported, oem_slot, prior=prior,\n"
    "          predicted_xy=(px, py) if prior else None)\n",
    "          candidates, raw_index, vision_supported, oem_slot, prior=prior,\n"
    "          predicted_xy=(px, py) if prior else None,\n"
    "          policy_state=self._p1_rep_policy_state, timestamp_ns=timestamp_ns)\n",
    "policy call",
  )
  source = _replace_once(
    source,
    "    self._update_common_ancestry(timestamp_ns, result)\n"
    "    stats['scans'] += 1\n",
    "    live_p1_pids = {obj.physical_track_id for obj in result}\n"
    "    for p1_pid in tuple(self._p1_rep_policy_state):\n"
    "      if p1_pid not in live_p1_pids:\n"
    "        del self._p1_rep_policy_state[p1_pid]\n"
    "    self._p1_rep_state_peak = max(self._p1_rep_state_peak, len(self._p1_rep_policy_state))\n"
    "    self._update_common_ancestry(timestamp_ns, result)\n"
    "    stats['scans'] += 1\n",
    "policy lifecycle",
  )
  return source


def load_policy_module(policy: str, *, name: str | None = None,
                       config: dict[str, Any] | None = None,
                       source_path: Path = SOURCE):
  if policy not in POLICIES:
    raise ValueError(policy)
  configure_imports()
  import replay_harness as rh

  source = _instrument(source_path.read_text(encoding="utf-8"))
  parsed = ast.parse(source)
  lines = source.splitlines()
  start = next(index for index, line in enumerate(lines, 1) if line.startswith("BOSCH_INACTIVE_WORD"))
  end = next(node.lineno for node in parsed.body if getattr(node, "name", None) == "RadarInterface")
  nodes = [node for node in parsed.body if start <= node.lineno < end]
  module_name = name or f"p1_rep_{policy.lower()}"
  module = types.ModuleType(module_name)
  sys.modules[module_name] = module
  exec(rh.PRELUDE, module.__dict__)
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source_path), "exec"), module.__dict__)
  from opendbc.car.radar_lead_filter import RadarLeadFilter
  module.RadarLeadFilter = RadarLeadFilter
  module.bosch_linear_sum_assignment = module.bosch_numpy_linear_sum_assignment
  module.BOSCH_P1_REP_POLICY = policy
  for key, value in (config or {}).items():
    setattr(module, key, value)
  return module
