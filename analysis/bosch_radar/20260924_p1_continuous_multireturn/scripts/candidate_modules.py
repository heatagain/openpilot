"""Compile analysis-only Bosch grouping candidates from the exact current source.

No generated source is written to the repository.  The transformations are
guarded by exact-match assertions so source drift fails closed.
"""
from __future__ import annotations

import ast
import sys
import types
from pathlib import Path

from common import SOURCE, configure_imports


CANDIDATES = ("BASELINE", "A_HYSTERESIS", "B_CONFIRMATION", "C_QUANTIZED", "D_STABLE_CORE", "E_REP_HOLD")


def _replace_once(source: str, old: str, new: str, label: str) -> str:
  count = source.count(old)
  if count != 1:
    raise RuntimeError(f"candidate source drift at {label}: expected 1 exact match, found {count}")
  return source.replace(old, new, 1)


def _instrument_grouping(source: str, candidate: str) -> str:
  if candidate == "BASELINE":
    return source
  if candidate == "E_REP_HOLD":
    return _replace_once(
      source,
      "  return min(candidates, key=cost)\n\n\ndef _bosch_physical_component",
      "  best = min(candidates, key=cost)\n"
      "  if prior is not None:\n"
      "    retained = next((member for member in candidates\n"
      "                     if member.raw_track_id == prior.representative_raw_track_id), None)\n"
      "    if retained is not None and cost(retained)[0] <= cost(best)[0] + .25:\n"
      "      return retained\n"
      "  return best\n\n\ndef _bosch_physical_component",
      "representative hold",
    )

  source = _replace_once(
    source,
    "@dataclass(frozen=True)\nclass BoschGroupingConfig:\n",
    f"BOSCH_P1_RESEARCH_CANDIDATE = {candidate!r}\n\n\n@dataclass(frozen=True)\nclass BoschGroupingConfig:\n",
    "candidate constant",
  )
  source = _replace_once(
    source,
    "    self.last_decisions: tuple[BoschAssociationDecision, ...] = ()\n",
    "    self.last_decisions: tuple[BoschAssociationDecision, ...] = ()\n"
    "    # Analysis-only state. It is local to this private module and never\n"
    "    # enters the checked-in provider.\n"
    "    self._p1_prior_edges = set()\n"
    "    self._p1_fail_counts = {}\n"
    "    self._p1_last_strict_ns = {}\n"
    "    self._p1_state_peak = 0\n"
    "    self._p1_relaxed_edges_last = set()\n",
    "candidate state",
  )
  source = _replace_once(
    source,
    "  def _update(self, timestamp_ns, raw_tracks, *, yaw_rate=None, v_ego=math.nan, oem_slot=None, vision=()):\n"
    "    raw_tracks = tuple(sorted(raw_tracks, key=lambda r: r.raw_track_id))\n"
    "    self.now_ns = timestamp_ns\n"
    "    c = self.config\n",
    "  def _update(self, timestamp_ns, raw_tracks, *, yaw_rate=None, v_ego=math.nan, oem_slot=None, vision=()):\n"
    "    raw_tracks = tuple(sorted(raw_tracks, key=lambda r: r.raw_track_id))\n"
    "    p1_previous_ns = self.now_ns\n"
    "    self.now_ns = timestamp_ns\n"
    "    c = self.config\n"
    "    if p1_previous_ns is not None and timestamp_ns-p1_previous_ns > round(c.coast_s*1e9):\n"
    "      self._p1_prior_edges.clear()\n"
    "      self._p1_fail_counts.clear()\n"
    "      self._p1_last_strict_ns.clear()\n"
    "      self._p1_relaxed_edges_last.clear()\n",
    "candidate input-gap reset",
  )
  source = _replace_once(
    source,
    "      if (timestamp_ns - self.pairs[pair].samples[-1][0] > c.evidence_window_s * 1e9 or\n"
    "          (i is not None and j is not None and abs(distances[i]-distances[j]) > c.distance_diameter_m)):\n"
    "        del self.pairs[pair]\n",
    "      p1_keep_distance_evidence = (\n"
    "        BOSCH_P1_RESEARCH_CANDIDATE in ('A_HYSTERESIS', 'B_CONFIRMATION', 'C_QUANTIZED', 'D_STABLE_CORE') and\n"
    "        pair in self._p1_prior_edges and i is not None and j is not None and\n"
    "        abs(distances[i]-distances[j]) <= c.distance_diameter_m + .25)\n"
    "      if (timestamp_ns - self.pairs[pair].samples[-1][0] > c.evidence_window_s * 1e9 or\n"
    "          (i is not None and j is not None and abs(distances[i]-distances[j]) > c.distance_diameter_m and\n"
    "           not p1_keep_distance_evidence)):\n"
    "        del self.pairs[pair]\n",
    "candidate evidence retention",
  )
  source = _replace_once(
    source,
    "    pair_candidates = lateral_rejections = velocity_rejections = motion_rejections = 0\n"
    "    provisional_matches = {}\n",
    "    pair_candidates = lateral_rejections = velocity_rejections = motion_rejections = 0\n"
    "    provisional_matches = {}\n"
    "    p1_candidate = BOSCH_P1_RESEARCH_CANDIDATE\n"
    "    p1_prior_edges = set(self._p1_prior_edges)\n"
    "    p1_strict_edges = set()\n"
    "    p1_relaxed_edges = set()\n"
    "    p1_small_failures = {}\n"
    "    p1_search_margin = .25 if p1_candidate in ('A_HYSTERESIS', 'B_CONFIRMATION', 'C_QUANTIZED', 'D_STABLE_CORE') else 0.\n",
    "candidate scan locals",
  )
  source = _replace_once(
    source,
    "        if distances[b]-distances[a] > c.distance_diameter_m:\n"
    "          break\n"
    "        pair_candidates += 1\n"
    "        i, j = (a, b) if a < b else (b, a)\n"
    "        key = (ids[i], ids[j])\n"
    "        delta_d, delta_y = distances[i]-distances[j], lateral[i]-lateral[j]\n"
    "        delta_v = abs(velocities[i]-velocities[j])\n",
    "        if distances[b]-distances[a] > c.distance_diameter_m + p1_search_margin:\n"
    "          break\n"
    "        pair_candidates += 1\n"
    "        i, j = (a, b) if a < b else (b, a)\n"
    "        key = (ids[i], ids[j])\n"
    "        delta_d, delta_y = distances[i]-distances[j], lateral[i]-lateral[j]\n"
    "        delta_v = abs(velocities[i]-velocities[j])\n"
    "        p1_dd, p1_dy = abs(delta_d), abs(delta_y)\n"
    "        p1_geometry_failure = None\n"
    "        if p1_dd > c.distance_diameter_m:\n"
    "          p1_geometry_failure = ('DISTANCE_DIAMETER', p1_dd-c.distance_diameter_m)\n",
    "candidate distance search",
  )
  source = _replace_once(
    source,
    "        if abs(delta_y) > c.lateral_diameter_m:\n"
    "          lateral_rejections += 1\n"
    "          self.pairs.pop(key, None)\n"
    "          continue\n"
    "        if delta_v > c.velocity_diameter_mps:\n"
    "          velocity_rejections += 1\n"
    "          self.pairs.pop(key, None)\n"
    "          continue\n"
    "        if (ground_speeds is not None and min(ground_speeds[i], ground_speeds[j]) <= c.stationary_speed_mps\n"
    "            and max(ground_speeds[i], ground_speeds[j]) >= c.moving_speed_mps):\n"
    "          motion_rejections += 1\n"
    "          self.pairs.pop(key, None)\n"
    "          continue\n"
    "        evidence = self.pairs.get(key)\n",
    "        if p1_dy > c.lateral_diameter_m:\n"
    "          lateral_rejections += 1\n"
    "          if p1_geometry_failure is None:\n"
    "            p1_geometry_failure = ('LATERAL_DIAMETER', p1_dy-c.lateral_diameter_m)\n"
    "        if delta_v > c.velocity_diameter_mps:\n"
    "          velocity_rejections += 1\n"
    "          if p1_geometry_failure is None:\n"
    "            p1_geometry_failure = ('VELOCITY_DIAMETER', delta_v-c.velocity_diameter_mps)\n"
    "        p1_motion_conflict = (ground_speeds is not None and min(ground_speeds[i], ground_speeds[j]) <= c.stationary_speed_mps\n"
    "                              and max(ground_speeds[i], ground_speeds[j]) >= c.moving_speed_mps)\n"
    "        if p1_motion_conflict:\n"
    "          motion_rejections += 1\n"
    "          if p1_geometry_failure is None:\n"
    "            p1_geometry_failure = ('STATIONARY_MOVING_CONFLICT', math.inf)\n"
    "        p1_geometry_relaxable = (\n"
    "          p1_candidate in ('A_HYSTERESIS', 'B_CONFIRMATION', 'C_QUANTIZED', 'D_STABLE_CORE') and\n"
    "          key in p1_prior_edges and not p1_motion_conflict and delta_v <= c.velocity_diameter_mps and\n"
    "          p1_dd <= c.distance_diameter_m + .25 and p1_dy <= c.lateral_diameter_m + .03125)\n"
    "        if p1_geometry_failure is not None and not p1_geometry_relaxable:\n"
    "          self.pairs.pop(key, None)\n"
    "          self._p1_fail_counts.pop(key, None)\n"
    "          self._p1_last_strict_ns.pop(key, None)\n"
    "          continue\n"
    "        evidence = self.pairs.get(key)\n",
    "candidate hard geometry",
  )
  source = _replace_once(
    source,
    "        abs_delta_d, abs_delta_y = abs(delta_d), abs(delta_y)\n"
    "        samples.append((timestamp_ns, abs_delta_d, abs_delta_y))\n"
    "        while timestamp_ns - samples[0][0] > c.evidence_window_s * 1e9:\n"
    "          samples.popleft()\n"
    "        # Widening from a recent minimum, not old larger converging separation.\n"
    "        stable = (abs_delta_d-min(s[1] for s in samples) <= c.max_relative_distance_growth_m and\n"
    "                  abs_delta_y-min(s[2] for s in samples) <= c.max_relative_lateral_growth_m)\n"
    "        mature = (len(samples) >= c.min_pair_observations and\n"
    "                  (timestamp_ns-samples[0][0])/1e9 + 1e-6 >= c.min_pair_span_s)\n"
    "        if stable and mature:\n",
    "        abs_delta_d, abs_delta_y = abs(delta_d), abs(delta_y)\n"
    "        p1_strict_geometry = p1_geometry_failure is None\n"
    "        if p1_strict_geometry:\n"
    "          samples.append((timestamp_ns, abs_delta_d, abs_delta_y))\n"
    "        while samples and timestamp_ns - samples[0][0] > c.evidence_window_s * 1e9:\n"
    "          samples.popleft()\n"
    "        if not samples:\n"
    "          self.pairs.pop(key, None)\n"
    "          continue\n"
    "        # Widening from a recent minimum, not old larger converging separation.\n"
    "        p1_distance_growth = abs_delta_d-min(s[1] for s in samples)\n"
    "        p1_lateral_growth = abs_delta_y-min(s[2] for s in samples)\n"
    "        stable = (p1_distance_growth <= c.max_relative_distance_growth_m and\n"
    "                  p1_lateral_growth <= c.max_relative_lateral_growth_m)\n"
    "        mature = (len(samples) >= c.min_pair_observations and\n"
    "                  (timestamp_ns-samples[0][0])/1e9 + 1e-6 >= c.min_pair_span_s)\n"
    "        p1_strict = p1_strict_geometry and stable and mature\n"
    "        if p1_strict:\n"
    "          p1_strict_edges.add(key)\n"
    "          self._p1_last_strict_ns[key] = timestamp_ns\n"
    "          self._p1_fail_counts[key] = 0\n"
    "        p1_growth_failure = None\n"
    "        if p1_distance_growth > c.max_relative_distance_growth_m:\n"
    "          p1_growth_failure = ('DISTANCE_GROWTH', p1_distance_growth-c.max_relative_distance_growth_m)\n"
    "        elif p1_lateral_growth > c.max_relative_lateral_growth_m:\n"
    "          p1_growth_failure = ('LATERAL_GROWTH', p1_lateral_growth-c.max_relative_lateral_growth_m)\n"
    "        p1_failure = p1_geometry_failure or p1_growth_failure\n"
    "        p1_relax = False\n"
    "        if not p1_strict and mature and key in p1_prior_edges and p1_failure is not None:\n"
    "          p1_reason, p1_excess = p1_failure\n"
    "          p1_small = ((p1_reason == 'DISTANCE_DIAMETER' and p1_excess <= .25 + 1e-12) or\n"
    "                      (p1_reason == 'LATERAL_DIAMETER' and p1_excess <= .03125 + 1e-12) or\n"
    "                      (p1_reason == 'DISTANCE_GROWTH' and p1_excess <= .25 + 1e-12) or\n"
    "                      (p1_reason == 'LATERAL_GROWTH' and p1_excess <= .1875 + 1e-12))\n"
    "          p1_quantized = ((p1_reason in ('DISTANCE_DIAMETER', 'DISTANCE_GROWTH') and p1_excess <= .25 + 1e-12) or\n"
    "                          (p1_reason in ('LATERAL_DIAMETER', 'LATERAL_GROWTH') and p1_excess <= .03125 + 1e-12))\n"
    "          if p1_candidate == 'A_HYSTERESIS':\n"
    "            p1_relax = p1_small and timestamp_ns-self._p1_last_strict_ns.get(key, -10**18) <= 300_000_000\n"
    "          elif p1_candidate == 'B_CONFIRMATION':\n"
    "            count = self._p1_fail_counts.get(key, 0) + 1\n"
    "            self._p1_fail_counts[key] = count\n"
    "            p1_relax = p1_small and count < 2\n"
    "          elif p1_candidate == 'C_QUANTIZED':\n"
    "            p1_relax = p1_quantized and timestamp_ns-self._p1_last_strict_ns.get(key, -10**18) <= 160_000_000\n"
    "          elif p1_candidate == 'D_STABLE_CORE':\n"
    "            p1_small_failures[key] = (i, j, p1_reason, p1_excess)\n"
    "        if p1_strict or p1_relax:\n"
    "          if p1_relax:\n"
    "            p1_relaxed_edges.add(key)\n",
    "candidate maturity and relaxation",
  )
  source = _replace_once(
    source,
    "          compatible[i] |= 1 << j\n"
    "          compatible[j] |= 1 << i\n"
    "          cost = abs_delta_d/c.distance_diameter_m + abs_delta_y/c.lateral_diameter_m + delta_v/c.velocity_diameter_mps\n"
    "          pair_cost[i*n+j] = pair_cost[j*n+i] = cost\n"
    "    self.last_pair_possible = n*(n-1)//2\n",
    "          compatible[i] |= 1 << j\n"
    "          compatible[j] |= 1 << i\n"
    "          cost = abs_delta_d/c.distance_diameter_m + abs_delta_y/c.lateral_diameter_m + delta_v/c.velocity_diameter_mps\n"
    "          pair_cost[i*n+j] = pair_cost[j*n+i] = cost\n"
    "    if p1_candidate == 'D_STABLE_CORE' and p1_small_failures:\n"
    "      for state in tuple(self.states.values()):\n"
    "        prior_members = tuple(sorted(m.raw_track_id for m in state.observation.members))\n"
    "        if len(prior_members) < 3 or not all(rid in raw_index for rid in prior_members):\n"
    "          continue\n"
    "        prior_pairs = [tuple(sorted((prior_members[a], prior_members[b])))\n"
    "                       for a in range(len(prior_members)) for b in range(a+1, len(prior_members))]\n"
    "        missing = [pair for pair in prior_pairs if pair not in p1_strict_edges]\n"
    "        if len(missing) != 1 or missing[0] not in p1_small_failures or missing[0] not in p1_prior_edges:\n"
    "          continue\n"
    "        i, j, _reason, _excess = p1_small_failures[missing[0]]\n"
    "        compatible[i] |= 1 << j\n"
    "        compatible[j] |= 1 << i\n"
    "        pair_cost[i*n+j] = pair_cost[j*n+i] = (abs(distances[i]-distances[j])/c.distance_diameter_m +\n"
    "                                                  abs(lateral[i]-lateral[j])/c.lateral_diameter_m +\n"
    "                                                  abs(velocities[i]-velocities[j])/c.velocity_diameter_mps)\n"
    "        p1_relaxed_edges.add(missing[0])\n"
    "    self._p1_prior_edges = p1_strict_edges | p1_relaxed_edges\n"
    "    self._p1_relaxed_edges_last = p1_relaxed_edges\n"
    "    live_pair_keys = set(self.pairs) | self._p1_prior_edges\n"
    "    self._p1_fail_counts = {key: value for key, value in self._p1_fail_counts.items() if key in live_pair_keys}\n"
    "    self._p1_last_strict_ns = {key: value for key, value in self._p1_last_strict_ns.items() if key in live_pair_keys}\n"
    "    self._p1_state_peak = max(self._p1_state_peak, len(self._p1_fail_counts) + len(self._p1_last_strict_ns))\n"
    "    self.last_pair_possible = n*(n-1)//2\n",
    "candidate stable core and lifecycle",
  )

  return source


def load_candidate_module(candidate: str, name: str | None = None, source_path: Path = SOURCE):
  if candidate not in CANDIDATES:
    raise ValueError(candidate)
  configure_imports()
  import replay_harness as rh

  source = _instrument_grouping(source_path.read_text(encoding="utf-8"), candidate)
  parsed = ast.parse(source)
  lines = source.splitlines()
  start = next(i for i, line in enumerate(lines, 1) if line.startswith("BOSCH_INACTIVE_WORD"))
  end = next(node.lineno for node in parsed.body if getattr(node, "name", None) == "RadarInterface")
  nodes = [node for node in parsed.body if start <= node.lineno < end]
  module_name = name or f"p1_{candidate.lower()}"
  module = types.ModuleType(module_name)
  sys.modules[module_name] = module
  exec(rh.PRELUDE, module.__dict__)
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source_path), "exec"), module.__dict__)
  module.bosch_linear_sum_assignment = module.bosch_numpy_linear_sum_assignment
  return module
