"""Mine mechanical group fracture/ownership/representative events on current HEAD.

This is deliberately actor-agnostic.  A split is a state-machine event, not a
SAME/DIFFERENT label.  Each corpus segment starts from a cold provider, matching
the established 847-segment replay authority.
"""
from __future__ import annotations

import itertools
import json
import math
import multiprocessing as mp
from collections import defaultdict

from common import (CHECKOUT, MANIFESTS, SCRATCH, SOURCE, TABLES, TRACES,
                    configure_imports, corpus_keys, ensure_dirs, load_replay_module, process_count, sha256,
                    write_csv, write_json, write_json_gz)


DISTANCE_THRESHOLD = 3.0
LATERAL_THRESHOLD = 1.5
VELOCITY_THRESHOLD = 1.5
DISTANCE_GROWTH_THRESHOLD = 1.0
LATERAL_GROWTH_THRESHOLD = .75
STATIONARY_SPEED = .6
MOVING_SPEED = 1.4


def _member_text(values) -> str:
  return "|".join(map(str, sorted(values)))


def _object_row(obj) -> dict:
  return {
    "pid": obj.physical_track_id,
    "members": tuple(sorted(member.raw_track_id for member in obj.members)),
    "representative": obj.representative_raw_track_id,
    "d": obj.d_rel,
    "y": obj.y_rel,
    "v": obj.v_rel,
    "age": obj.age_scans,
    "slots": tuple(sorted(member.slot for member in obj.members)),
  }


def _pair_reason(snapshot: dict, raw_a: int, raw_b: int) -> dict:
  first = snapshot["raws"].get(raw_a)
  second = snapshot["raws"].get(raw_b)
  if first is None or second is None:
    return {"reason": "RAW_DROPOUT", "actual": "", "threshold": "", "excess": "",
            "delta_d": "", "delta_y": "", "delta_v": "", "distance_growth": "",
            "lateral_growth": "", "samples": 0}
  dd = abs(first["d"] - second["d"])
  dy = abs(first["y"] - second["y"])
  dv = abs(first["v"] - second["v"])
  if dd > DISTANCE_THRESHOLD:
    reason, actual, threshold = "DISTANCE_DIAMETER", dd, DISTANCE_THRESHOLD
  elif dy > LATERAL_THRESHOLD:
    reason, actual, threshold = "LATERAL_DIAMETER", dy, LATERAL_THRESHOLD
  elif dv > VELOCITY_THRESHOLD:
    reason, actual, threshold = "VELOCITY_DIAMETER", dv, VELOCITY_THRESHOLD
  else:
    world_a = abs(first["v"] + snapshot["v_ego"])
    world_b = abs(second["v"] + snapshot["v_ego"])
    if min(world_a, world_b) <= STATIONARY_SPEED and max(world_a, world_b) >= MOVING_SPEED:
      reason, actual, threshold = "OTHER_STATIONARY_MOVING", max(world_a, world_b), MOVING_SPEED
    else:
      samples = snapshot["pairs"].get((min(raw_a, raw_b), max(raw_a, raw_b)), ())
      current = samples and samples[-1][0] == snapshot["ns"]
      if not current:
        return {"reason": "OTHER_PAIR_EVIDENCE", "actual": "", "threshold": "", "excess": "",
                "delta_d": dd, "delta_y": dy, "delta_v": dv, "distance_growth": "",
                "lateral_growth": "", "samples": len(samples)}
      distance_growth = dd - min(sample[1] for sample in samples)
      lateral_growth = dy - min(sample[2] for sample in samples)
      if distance_growth > DISTANCE_GROWTH_THRESHOLD:
        reason, actual, threshold = "DISTANCE_GROWTH", distance_growth, DISTANCE_GROWTH_THRESHOLD
      elif lateral_growth > LATERAL_GROWTH_THRESHOLD:
        reason, actual, threshold = "LATERAL_GROWTH", lateral_growth, LATERAL_GROWTH_THRESHOLD
      elif len(samples) < 3 or (samples[-1][0] - samples[0][0]) / 1e9 + 1e-6 < .18:
        return {"reason": "OTHER_IMMATURE_PAIR", "actual": len(samples), "threshold": 3,
                "excess": "", "delta_d": dd, "delta_y": dy, "delta_v": dv,
                "distance_growth": distance_growth, "lateral_growth": lateral_growth,
                "samples": len(samples)}
      else:
        return {"reason": "OTHER_COMPLETE_LINK_ORDER", "actual": 0, "threshold": 0, "excess": 0,
                "delta_d": dd, "delta_y": dy, "delta_v": dv,
                "distance_growth": distance_growth, "lateral_growth": lateral_growth,
                "samples": len(samples)}
      return {"reason": reason, "actual": actual, "threshold": threshold,
              "excess": actual - threshold, "delta_d": dd, "delta_y": dy, "delta_v": dv,
              "distance_growth": distance_growth, "lateral_growth": lateral_growth,
              "samples": len(samples)}
  return {"reason": reason, "actual": actual, "threshold": threshold,
          "excess": actual - threshold, "delta_d": dd, "delta_y": dy, "delta_v": dv,
          "distance_growth": "", "lateral_growth": "", "samples": 0}


def _publication(snapshot: dict, pid: int) -> dict | None:
  return snapshot["published_by_pid"].get(pid)


def _replay_snapshots(key: str) -> tuple[list[dict], list[dict]]:
  configure_imports()
  import replay_harness as rh
  replay = load_replay_module()

  snapshots = []
  original_load = rh.load_module

  def hooked_load(name):
    module = original_load(name, source_path=SOURCE)
    original_provider = module.BoschRadarProvider

    class ObservedProvider(original_provider):
      def update(self, *args, **kwargs):
        group = self.tracker.group_manager
        old_states = {
          pid: {
            "members": tuple(sorted(state.member_last_seen)),
            "observed_members": tuple(sorted(member.raw_track_id for member in state.observation.members)),
            "representative": state.observation.representative_raw_track_id,
            "age": state.observation.age_scans,
            "d": state.observation.d_rel,
            "y": state.observation.y_rel,
            "v": state.observation.v_rel,
          }
          for pid, state in group.states.items()
        }
        result = super().update(*args, **kwargs)
        if result is not None:
          raw_rows = {
            raw.raw_track_id: {"slot": raw.slot, "age": raw.age_scans, "d": raw.d_rel,
                               "y": raw.y_rel, "v": raw.v_rel, "recovered": int(raw.recovered)}
            for raw in self.tracker.last_raw_tracks
          }
          snapshots.append({
            "ns": self.last_scan_timestamp_ns,
            "v_ego": float(kwargs.get("v_ego", math.nan)),
            "yaw_rate": float(kwargs.get("yaw_rate_left") or 0.0),
            "raws": raw_rows,
            "objects": [_object_row(obj) for obj in self._debug_objects],
            "old_states": old_states,
            "new_states": {
              pid: {"members": tuple(sorted(state.member_last_seen)),
                    "representative": state.observation.representative_raw_track_id,
                    "age": state.observation.age_scans}
              for pid, state in group.states.items()
            },
            "pairs": {pair: tuple(evidence.samples) for pair, evidence in group.pairs.items()},
          })
        return result

    module.BoschRadarProvider = ObservedProvider
    return module

  rh.load_module = hooked_load
  try:
    records = replay.run_segment(key)
  finally:
    rh.load_module = original_load
  if len(records) != len(snapshots):
    raise AssertionError(f"{key}: {len(records)} records != {len(snapshots)} snapshots")
  for snapshot, record in zip(snapshots, records, strict=True):
    snapshot["published_by_pid"] = {int(obj["pid"]): obj for obj in record["objects"]}
    snapshot["now_ns"] = record["now_ns"]
  return snapshots, records


def _process_segment(key: str) -> dict:
  snapshots, _records = _replay_snapshots(key)
  segment = int(key.rsplit("--", 1)[-1])
  route = key.split("--", 1)[0]
  splits, thresholds, owner_rows, births = [], [], [], []
  rep_transitions, publication_rows, raw_events = [], [], []
  route_trace = []

  active_groups: dict[tuple[int, tuple[int, ...]], dict] = {}
  group_lifetimes = []

  for index, snapshot in enumerate(snapshots):
    current_by_pid = {obj["pid"]: obj for obj in snapshot["objects"]}
    raw_to_obj = {raw: obj for obj in snapshot["objects"] for raw in obj["members"]}
    current_keys = {(obj["pid"], obj["members"]) for obj in snapshot["objects"]}
    for group_key in list(active_groups):
      if group_key not in current_keys:
        episode = active_groups.pop(group_key)
        episode["end_scan"] = index - 1
        episode["end_ns"] = snapshots[index - 1]["ns"] if index else episode["start_ns"]
        episode["duration_s"] = (episode["end_ns"] - episode["start_ns"]) / 1e9
        episode["representatives"] = _member_text(episode.pop("representative_set"))
        group_lifetimes.append(episode)
    for obj in snapshot["objects"]:
      group_key = (obj["pid"], obj["members"])
      episode = active_groups.get(group_key)
      if episode is None:
        episode = active_groups[group_key] = {
          "key": key, "route": route, "segment": segment, "pid": obj["pid"],
          "members": _member_text(obj["members"]), "member_count": len(obj["members"]),
          "start_scan": index, "start_ns": snapshot["ns"], "scans": 0,
          "representative_set": set(), "max_d_extent": 0.0, "max_y_extent": 0.0,
          "max_v_spread": 0.0,
        }
      episode["scans"] += 1
      episode["representative_set"].add(obj["representative"])
      raw_values = [snapshot["raws"][raw] for raw in obj["members"] if raw in snapshot["raws"]]
      if raw_values:
        episode["max_d_extent"] = max(episode["max_d_extent"],
                                      max(row["d"] for row in raw_values) - min(row["d"] for row in raw_values))
        episode["max_y_extent"] = max(episode["max_y_extent"],
                                      max(row["y"] for row in raw_values) - min(row["y"] for row in raw_values))
        episode["max_v_spread"] = max(episode["max_v_spread"],
                                      max(row["v"] for row in raw_values) - min(row["v"] for row in raw_values))

    if index == 0:
      for obj in snapshot["objects"]:
        births.append({"key": key, "route": route, "segment": segment, "scan": index,
                       "scan_ns": snapshot["ns"], "pid": obj["pid"],
                       "members": _member_text(obj["members"]), "member_count": len(obj["members"]),
                       "representative": obj["representative"], "cause": "SEGMENT_COLD_START"})
      continue

    previous = snapshots[index - 1]
    previous_by_pid = {obj["pid"]: obj for obj in previous["objects"]}
    previous_raw_to_obj = {raw: obj for obj in previous["objects"] for raw in obj["members"]}

    for raw in previous["raws"]:
      if raw not in snapshot["raws"]:
        raw_events.append({"key": key, "route": route, "segment": segment, "scan": index,
                           "scan_ns": snapshot["ns"], "raw_id": raw, "event": "RAW_DROPOUT"})
    for raw, row in snapshot["raws"].items():
      old = previous["raws"].get(raw)
      if old is not None and old["slot"] != row["slot"]:
        raw_events.append({"key": key, "route": route, "segment": segment, "scan": index,
                           "scan_ns": snapshot["ns"], "raw_id": raw, "event": "RAW_REASSIGNMENT",
                           "from_slot": old["slot"], "to_slot": row["slot"]})

    # PID births are mechanical. A birth is not a physical actor claim.
    old_pids = set(previous["new_states"])
    for obj in snapshot["objects"]:
      if obj["pid"] not in old_pids:
        causes = {previous_raw_to_obj[raw]["pid"] for raw in obj["members"] if raw in previous_raw_to_obj}
        cause = "GROUP_FRACTURE_CHILD" if causes else "RAW_OR_GROUP_BIRTH"
        births.append({"key": key, "route": route, "segment": segment, "scan": index,
                       "scan_ns": snapshot["ns"], "pid": obj["pid"],
                       "members": _member_text(obj["members"]), "member_count": len(obj["members"]),
                       "representative": obj["representative"], "cause": cause,
                       "prior_pids": _member_text(causes)})

    # Representative/member topology transitions for persistent physical PIDs.
    for pid in sorted(set(previous_by_pid).intersection(current_by_pid)):
      old, new = previous_by_pid[pid], current_by_pid[pid]
      old_set, new_set = set(old["members"]), set(new["members"])
      changed = old["representative"] != new["representative"]
      if old_set == new_set:
        topology = "GROUP_STABLE"
      elif new_set < old_set:
        topology = "GROUP_SPLIT"
      elif old_set < new_set:
        topology = "GROUP_MERGE"
      else:
        topology = "MEMBERSHIP_CHANGE"
      if topology == "GROUP_STABLE" and changed:
        category = "A_GROUP_STABLE_REP_CHANGE"
      elif topology == "GROUP_SPLIT" and not changed:
        category = "B_GROUP_SPLIT_REP_RETAINED"
      elif topology == "GROUP_SPLIT" and changed:
        category = "C_GROUP_SPLIT_REP_CHANGED"
      elif topology == "GROUP_MERGE" and changed:
        category = "D_GROUP_MERGE_REP_CHANGED"
      else:
        category = f"OTHER_{topology}_{'REP_CHANGED' if changed else 'REP_RETAINED'}"
      if changed or topology != "GROUP_STABLE":
        old_public, new_public = _publication(previous, pid), _publication(snapshot, pid)
        rep_transitions.append({
          "key": key, "route": route, "segment": segment, "scan": index, "scan_ns": snapshot["ns"],
          "pid": pid, "category": category, "topology": topology,
          "old_members": _member_text(old["members"]), "new_members": _member_text(new["members"]),
          "old_representative": old["representative"], "new_representative": new["representative"],
          "representative_changed": int(changed), "delta_d": new["d"] - old["d"],
          "delta_y": new["y"] - old["y"], "delta_v": new["v"] - old["v"],
          "abs_delta_d": abs(new["d"] - old["d"]), "abs_delta_y": abs(new["y"] - old["y"]),
          "abs_delta_v": abs(new["v"] - old["v"]),
          "old_published": int(bool(old_public and old_public["published"])),
          "new_published": int(bool(new_public and new_public["published"])),
          "old_alias": old_public["public_alias"] if old_public else "",
          "new_alias": new_public["public_alias"] if new_public else "",
        })

    # A group fracture requires at least two prior members still observed and
    # assigned to at least two current groups. Missing raws are a separate event.
    for parent in previous["objects"]:
      parent_members = set(parent["members"])
      if len(parent_members) < 2:
        continue
      observed = sorted(parent_members.intersection(raw_to_obj))
      children_by_pid = {}
      for raw in observed:
        child = raw_to_obj[raw]
        children_by_pid[child["pid"]] = child
      if len(observed) < 2 or len(children_by_pid) < 2:
        continue

      event_id = f"{key}:{index}:{parent['pid']}:{_member_text(parent_members)}"
      child_rows = sorted(children_by_pid.values(), key=lambda row: row["pid"])
      child_text = ";".join(f"{row['pid']}:{_member_text(row['members'])}" for row in child_rows)
      parent_owner_child = next((row for row in child_rows if row["pid"] == parent["pid"]), None)
      new_pids = [row["pid"] for row in child_rows if row["pid"] not in previous["new_states"]]
      failed_pairs = []
      for left, right in itertools.combinations(observed, 2):
        if raw_to_obj[left]["pid"] == raw_to_obj[right]["pid"]:
          continue
        detail = _pair_reason(snapshot, left, right)
        failed_pairs.append((left, right, detail))
        thresholds.append({
          "event_id": event_id, "key": key, "route": route, "segment": segment,
          "split_scan": index, "split_ns": snapshot["ns"], "parent_pid": parent["pid"],
          "parent_members": _member_text(parent_members), "raw_a": left, "raw_b": right,
          **detail,
        })

      primary = next((item for item in failed_pairs if not item[2]["reason"].startswith("OTHER_")),
                     failed_pairs[0] if failed_pairs else ("", "", {"reason": "OTHER", "excess": ""}))
      prev_public = _publication(previous, parent["pid"])
      child_public = [_publication(snapshot, row["pid"]) for row in child_rows]
      published_children = [row for row in child_public if row and row["published"]]
      new_public_point = any(row["pid"] != parent["pid"] for row in published_children)
      parent_public = _publication(snapshot, parent["pid"])
      publication_gap = bool(prev_public and prev_public["published"] and not published_children)
      alias_changed = bool(prev_public and parent_public and prev_public["published"] and parent_public["published"]
                           and prev_public["public_alias"] != parent_public["public_alias"])
      rep_changed = bool(parent_owner_child and parent_owner_child["representative"] != parent["representative"])
      external_visible = publication_gap or new_public_point or alias_changed or rep_changed
      split = {
        "event_id": event_id, "key": key, "route": route, "segment": segment,
        "split_scan": index, "split_ns": snapshot["ns"], "parent_pid": parent["pid"],
        "parent_members": _member_text(parent_members), "parent_member_count": len(parent_members),
        "observed_parent_members": _member_text(observed), "raw_dropout_count": len(parent_members)-len(observed),
        "child_count": len(child_rows), "children": child_text,
        "parent_representative": parent["representative"],
        "owner_child_pid": parent_owner_child["pid"] if parent_owner_child else "",
        "owner_child_members": _member_text(parent_owner_child["members"]) if parent_owner_child else "",
        "owner_child_representative": parent_owner_child["representative"] if parent_owner_child else "",
        "new_pid_births": len(new_pids), "new_pids": _member_text(new_pids),
        "failure_edge": f"{primary[0]}|{primary[1]}", "failure_reason": primary[2]["reason"],
        "failure_excess": primary[2].get("excess", ""),
        "publication_gap": int(publication_gap), "new_public_point": int(new_public_point),
        "alias_change": int(alias_changed), "representative_change": int(rep_changed),
        "external_visible": int(external_visible), "internal_only": int(not external_visible),
        "rejoin_scan": "", "rejoin_ns": "", "rejoin_latency_s": "", "rejoin_pid": "",
        "rejoin_members": "", "rejoin_to_parent_pid": 0,
      }
      splits.append(split)
      publication_rows.append({
        "event_id": event_id, "key": key, "route": route, "segment": segment,
        "split_scan": index, "split_ns": snapshot["ns"], "parent_pid": parent["pid"],
        "parent_alias": prev_public["public_alias"] if prev_public else "",
        "parent_published_before": int(bool(prev_public and prev_public["published"])),
        "published_child_count": len(published_children),
        "published_child_pids": _member_text(row["pid"] for row in published_children),
        "publication_gap": int(publication_gap), "new_public_point": int(new_public_point),
        "alias_change": int(alias_changed), "representative_change": int(rep_changed),
        "external_visible": int(external_visible), "internal_only": int(not external_visible),
      })

      for child in child_rows:
        members = set(child["members"])
        for prior_pid, prior in sorted(previous["new_states"].items()):
          overlap = len(members.intersection(prior["members"]))
          if not overlap:
            continue
          rep_retained = int(prior["representative"] in members)
          overlap_term = 10 * overlap
          rep_term = 3 * rep_retained
          age_term = min(prior["age"], 1000) * 1e-5
          pid_term = 1 / (prior_pid + 1)
          owner_rows.append({
            "event_id": event_id, "key": key, "route": route, "segment": segment,
            "split_scan": index, "split_ns": snapshot["ns"], "parent_pid": parent["pid"],
            "child_pid": child["pid"], "child_members": _member_text(child["members"]),
            "prior_pid": prior_pid, "overlap": overlap, "representative_retained": rep_retained,
            "overlap_term": overlap_term, "representative_term": rep_term,
            "age_term": age_term, "pid_tiebreak_term": pid_term,
            "score": overlap_term + rep_term + age_term + pid_term,
            "winner": int(prior_pid == child["pid"]),
          })

    if key == "00000269--4014745f93--7":
      raw436, raw482 = snapshot["raws"].get(436), snapshot["raws"].get(482)
      obj436, obj482 = raw_to_obj.get(436), raw_to_obj.get(482)
      pair = _pair_reason(snapshot, 436, 482) if raw436 and raw482 else {"reason": "RAW_DROPOUT", "excess": ""}
      owner_scores = []
      if obj482:
        members = set(obj482["members"])
        for pid, state in previous["new_states"].items():
          overlap = len(members.intersection(state["members"]))
          if overlap:
            owner_scores.append((pid, 10*overlap + 3*(state["representative"] in members) +
                                 min(state["age"], 1000)*1e-5 + 1/(pid+1)))
      route_trace.append({
        "scan": index, "scan_ns": snapshot["ns"], "raw436_observed": int(raw436 is not None),
        "raw482_observed": int(raw482 is not None), "raw436_slot": raw436["slot"] if raw436 else "",
        "raw482_slot": raw482["slot"] if raw482 else "", "raw436_age": raw436["age"] if raw436 else "",
        "raw482_age": raw482["age"] if raw482 else "", "raw436_d": raw436["d"] if raw436 else "",
        "raw482_d": raw482["d"] if raw482 else "", "raw436_y": raw436["y"] if raw436 else "",
        "raw482_y": raw482["y"] if raw482 else "", "raw436_v": raw436["v"] if raw436 else "",
        "raw482_v": raw482["v"] if raw482 else "", "pid436": obj436["pid"] if obj436 else "",
        "pid482": obj482["pid"] if obj482 else "", "group436": _member_text(obj436["members"]) if obj436 else "",
        "group482": _member_text(obj482["members"]) if obj482 else "",
        "representative436": obj436["representative"] if obj436 else "",
        "representative482": obj482["representative"] if obj482 else "",
        "pair_reason": pair["reason"], "pair_excess": pair.get("excess", ""),
        "owner_scores482": json.dumps(owner_scores, separators=(",", ":")),
        "published482": int(bool(obj482 and (pub := _publication(snapshot, obj482["pid"])) and pub["published"])),
        "alias482": pub["public_alias"] if obj482 and pub else "",
      })

  last_index = len(snapshots) - 1
  for episode in active_groups.values():
    episode["end_scan"] = last_index
    episode["end_ns"] = snapshots[-1]["ns"] if snapshots else episode["start_ns"]
    episode["duration_s"] = (episode["end_ns"] - episode["start_ns"]) / 1e9
    episode["representatives"] = _member_text(episode.pop("representative_set"))
    group_lifetimes.append(episode)

  # First within-segment mechanical rejoin of the exact parent raw family.
  for split in splits:
    members = {int(value) for value in split["parent_members"].split("|") if value}
    for future_index in range(split["split_scan"] + 1, len(snapshots)):
      future = snapshots[future_index]
      raw_to_obj = {raw: obj for obj in future["objects"] for raw in obj["members"]}
      if not members.issubset(raw_to_obj):
        continue
      owners = {raw_to_obj[raw]["pid"] for raw in members}
      if len(owners) != 1:
        continue
      obj = raw_to_obj[next(iter(members))]
      split.update(rejoin_scan=future_index, rejoin_ns=future["ns"],
                   rejoin_latency_s=(future["ns"] - split["split_ns"]) / 1e9,
                   rejoin_pid=obj["pid"], rejoin_members=_member_text(obj["members"]),
                   rejoin_to_parent_pid=int(obj["pid"] == split["parent_pid"]))
      break

  return {
    "key": key, "scans": len(snapshots), "splits": splits, "thresholds": thresholds,
    "owner_rows": owner_rows, "births": births, "rep_transitions": rep_transitions,
    "publication_rows": publication_rows, "raw_events": raw_events,
    "group_lifetimes": group_lifetimes, "route_trace": route_trace,
  }


def main() -> None:
  ensure_dirs()
  keys = corpus_keys()
  if len(keys) != 847:
    raise RuntimeError(f"expected 847 cached segments, found {len(keys)}")
  aggregate = defaultdict(list)
  summaries = []
  with mp.Pool(processes=process_count(), maxtasksperchild=20) as pool:
    for count, result in enumerate(pool.imap_unordered(_process_segment, keys, chunksize=1), 1):
      summaries.append({"key": result.pop("key"), "scans": result.pop("scans")})
      for name, rows in result.items():
        aggregate[name].extend(rows)
      if count % 10 == 0 or count == len(keys):
        print(f"{count}/{len(keys)} segments; {len(aggregate['splits'])} group fractures", flush=True)

  sort_fields = {
    "splits": ("key", "split_ns", "parent_pid"), "thresholds": ("key", "split_ns", "raw_a", "raw_b"),
    "owner_rows": ("key", "split_ns", "child_pid", "prior_pid"), "births": ("key", "scan_ns", "pid"),
    "rep_transitions": ("key", "scan_ns", "pid"), "publication_rows": ("key", "split_ns", "parent_pid"),
    "raw_events": ("key", "scan_ns", "raw_id"), "group_lifetimes": ("key", "start_ns", "pid"),
  }
  for name, fields in sort_fields.items():
    aggregate[name].sort(key=lambda row, keys=fields: tuple(row.get(key, "") for key in keys))

  write_csv(TABLES / "all_group_splits.csv", aggregate["splits"])
  write_csv(TABLES / "threshold_excess.csv", aggregate["thresholds"])
  write_csv(TABLES / "owner_assignment.csv", aggregate["owner_rows"])
  write_csv(TABLES / "pid_births.csv", aggregate["births"])
  write_csv(TABLES / "representative_changes.csv", aggregate["rep_transitions"])
  write_csv(TABLES / "representative_jumps.csv",
            [row for row in aggregate["rep_transitions"] if row["representative_changed"]])
  write_csv(TABLES / "publication_discontinuities.csv", aggregate["publication_rows"])
  write_csv(TABLES / "group_lifetime.csv", aggregate["group_lifetimes"])
  write_csv(TABLES / "multi_return_clusters.csv",
            [row for row in aggregate["group_lifetimes"] if row["member_count"] >= 2])
  write_csv(TABLES / "mechanical_raw_events.csv", aggregate["raw_events"])
  write_csv(TRACES / "route269" / "route269_s7_raw436_raw482.csv", aggregate["route_trace"])

  bundle = {name: rows for name, rows in aggregate.items() if name != "route_trace"}
  write_json_gz(SCRATCH / "baseline_mechanical_bundle.json.gz", bundle)
  manifest = {
    "source": str(SOURCE), "source_sha256": sha256(SOURCE), "checkout": str(CHECKOUT),
    "segments": len(summaries), "scans": sum(row["scans"] for row in summaries),
    "routes": len({row["key"].split("--", 1)[0] for row in summaries}),
    "group_fractures": len(aggregate["splits"]),
    "split_rejoins": sum(row["rejoin_scan"] != "" for row in aggregate["splits"]),
    "representative_transitions": sum(row["representative_changed"] for row in aggregate["rep_transitions"]),
    "publication_visible_fractures": sum(row["external_visible"] for row in aggregate["splits"]),
    "internal_only_fractures": sum(row["internal_only"] for row in aggregate["splits"]),
    "segment_summaries": sorted(summaries, key=lambda row: row["key"]),
  }
  write_json(MANIFESTS / "baseline_replay.json", manifest)
  print(json.dumps({key: value for key, value in manifest.items() if key != "segment_summaries"}, indent=2))


if __name__ == "__main__":
  mp.freeze_support()
  main()
