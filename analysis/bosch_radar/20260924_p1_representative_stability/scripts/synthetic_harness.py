# ruff: noqa: TID251
"""Deterministic A-M sequences through the actual representative selector."""
from __future__ import annotations

from common import MANIFESTS, TABLES, ensure_dirs, write_csv, write_json
from representative_modules import POLICIES, load_policy_module


def _track(module, raw: int, slot: int, d: float, y: float, v: float, age: int, ns: int):
  detection = module.BoschRawDetection(ns, slot, float(d), float(y), float(v), 1)
  return module.BoschRawTrack(raw, detection, age)


def _prior(module, members, rep, ns, pid=1):
  point = next(member for member in members if member.raw_track_id == rep)
  return module.BoschPhysicalObject(pid, ns, tuple(members), rep, point.d_rel, point.y_rel,
                                    point.v_rel, False, False, 10, "temporal_complete_link")


def _select(module, policy_state, prior, members, ns, vision=(), oem_slot=None):
  index = {member.raw_track_id: i for i, member in enumerate(members)}
  support = [member.raw_track_id in vision for member in members]
  dt = (ns-prior.timestamp_ns)*1e-9
  px = prior.d_rel + prior.v_rel*dt
  chosen = module._bosch_group_representative(
    tuple(members), index, support, oem_slot, prior=prior, predicted_xy=(px, prior.y_rel),
    policy_state=policy_state, timestamp_ns=ns)
  return chosen


def _run_policy(policy: str) -> list[dict]:
  module = load_policy_module(policy, name=f"synthetic_rep_{policy.lower()}")
  rows = []

  def run_case(name, frames, *, vision=(), oem=(), reset_at=()):
    state = {}
    ns = 1_000_000_000
    first = [_track(module, *values, ns) for values in frames[0]]
    prior = _prior(module, first, first[0].raw_track_id, ns)
    rows.append({"policy": policy, "case": name, "frame": 0, "chosen": prior.representative_raw_track_id,
                 "members": "|".join(str(member.raw_track_id) for member in first), "stale": 0})
    for index, frame in enumerate(frames[1:], 1):
      ns += 100_000_000
      if index in reset_at:
        state = {}
      members = [_track(module, *values, ns) for values in frame]
      chosen = _select(module, state, prior, members, ns,
                       vision=vision[index] if index < len(vision) else (),
                       oem_slot=oem[index] if index < len(oem) else None)
      trace = module.BOSCH_P1_REP_TRACE[-1]
      stale = chosen.raw_track_id not in {member.raw_track_id for member in members}
      rows.append({"policy": policy, "case": name, "frame": index, "chosen": chosen.raw_track_id,
                   "members": "|".join(str(member.raw_track_id) for member in members),
                   "stale": int(stale), "baseline_best": trace["baseline_best"],
                   "prior_rep": trace["prior_rep"], "score_delta": trace["score_delta"],
                   "jump_d": trace["jump_d"], "jump_y": trace["jump_y"],
                   "jump_v": trace["jump_v"], "reason": trace["reason"],
                   "state_size": len(state)})
      prior = _prior(module, members, chosen.raw_track_id, ns)

  stable = [(1, 1, 30.0, 0.0, 0.0, 20), (2, 2, 31.0, .1, 0.0, 20)]
  marginal = [(1, 1, 30.20, 0.0, 0.0, 21), (2, 2, 30.10, .1, 0.0, 21)]
  strong = [(1, 1, 35.0, 0.0, 0.0, 21), (2, 2, 30.0, .1, 0.0, 21)]
  three = stable + [(3, 3, 30.5, -.1, 0.0, 20)]
  run_case("A_STABLE_TWO_MEMBER", [stable, stable, stable])
  run_case("B_MARGINALLY_BETTER_ONE_SCAN", [stable, marginal, stable])
  run_case("C_MARGINALLY_BETTER_THREE_SCANS", [stable, marginal, marginal, marginal])
  run_case("D_A_B_A_PINGPONG", [stable, marginal, stable, marginal])
  run_case("E_OLD_REP_DISAPPEARS", [stable, [marginal[1]]])
  run_case("F_STRONG_CAMERA_SUPPORT", [stable, marginal], vision=((), (2,)))
  run_case("G_STRONG_OEM_SUPPORT", [stable, marginal], oem=(None, 2))
  weak_jump_prior = [(1, 1, 32.5, 0.0, 0.0, 20), (2, 2, 33.0, .1, 0.0, 20)]
  weak_jump = [(1, 1, 35.0, 0.0, 0.0, 21), (2, 2, 30.1, .1, 0.0, 21)]
  run_case("H_LARGE_D_WEAK_ADVANTAGE", [weak_jump_prior, weak_jump])
  run_case("I_LARGE_D_STRONG_ADVANTAGE", [stable, strong])
  run_case("J_THREE_MEMBER", [three, [three[1], three[0], three[2]]])
  run_case("K_INPUT_RESET", [stable, marginal, stable], reset_at=(2,))
  run_case("L_SEGMENT_RESET", [stable, marginal], reset_at=(1,))
  run_case("M_CUTIN_MEMBER_TRANSITION", [stable, [marginal[1], (3, 3, 24.0, 0.0, -2.0, 1)]])
  return rows


def main() -> None:
  ensure_dirs()
  rows = []
  for policy in POLICIES:
    rows.extend(_run_policy(policy))
  if any(int(row["stale"]) for row in rows):
    raise AssertionError("a policy selected a stale/non-member representative")
  write_csv(TABLES / "synthetic_results.csv", rows)
  summary = {"policies": len(POLICIES), "cases": 13, "rows": len(rows),
             "stale_representatives": sum(int(row["stale"]) for row in rows)}
  write_json(MANIFESTS / "synthetic_summary.json", summary)
  print(summary)


if __name__ == "__main__":
  main()
