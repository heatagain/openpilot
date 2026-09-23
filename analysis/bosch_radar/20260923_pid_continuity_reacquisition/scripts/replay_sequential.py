"""Recheck previously video-labelled sequential transitions on current HEAD."""

from __future__ import annotations

import csv
import gzip
import json
from collections import defaultdict
from pathlib import Path

from replay_identity import ROOT, STUDY, CORPUS, replay


PRIOR = ROOT / "analysis" / "bosch_radar" / "20260913_ldws_pid_reacquisition_shadow"


def read_csv(path: Path) -> list[dict]:
    with path.open(encoding="utf-8-sig", newline="") as inp:
        return list(csv.DictReader(inp))


def endpoint(records: list[dict], ns: int, d: float, y: float, v: float,
             old_raw: set[int]) -> dict:
    nearest = min(records, key=lambda item: abs(item["scan_ns"] - ns))
    delta_ms = abs(nearest["scan_ns"] - ns) / 1e6
    if delta_ms > 150:
        return {"status": "NO_ALIGNED_SCAN", "delta_ms": round(delta_ms, 3)}
    candidates = []
    for obj in nearest["objects"]:
        dd, dy, dv = abs(obj["d"] - d), abs(obj["y"] - y), abs(obj["v"] - v)
        if dd <= 2.0 and dy <= 2.0 and dv <= 2.0:
            candidates.append((dd + dy + dv, obj))
    if len(candidates) != 1:
        return {"status": "NO_MATCH" if not candidates else "AMBIGUOUS_REMAP",
                "delta_ms": round(delta_ms, 3), "candidate_count": len(candidates)}
    obj = candidates[0][1]
    return {"status": "UNIQUE_GEOMETRY", "delta_ms": round(delta_ms, 3),
            "candidate_count": 1, "pid": obj["pid"], "raw_ids": obj["raw_ids"],
            "raw_overlap": len(set(obj["raw_ids"]) & old_raw),
            "alias": obj["public_alias"], "published": obj["published"],
            "d_m": obj["d"], "y_m": obj["y"], "v_mps": obj["v"],
            "scan_ns": nearest["scan_ns"]}


def main() -> None:
    gt = [row for row in read_csv(PRIOR / "tables" / "transition_ground_truth.csv")
          if row["label"] == "SAME_CONFIRMED"]
    inventory = {row["event_id"]: row for row in read_csv(PRIOR / "tables" / "pid_transition_inventory.csv")}
    grouped = defaultdict(list)
    for row in gt:
        prior = inventory[row["event_id"]]
        grouped[(row["route"], int(prior["old_segment"]))].append((row, prior))
    output = []
    traces = []
    for (route, segment), events in sorted(grouped.items()):
        matches = list(CORPUS.glob(f"{int(route, 16):08x}--*--{segment}.json.gz"))
        if len(matches) != 1:
            for label, _ in events:
                output.append({"event_id": label["event_id"], "route": route, "segment": segment,
                               "prior_gt": label["label"], "head_status": "NOT AVAILABLE"})
            continue
        key = matches[0].name.removesuffix(".json.gz")
        cache = STUDY / "scratch" / f"{key}.json.gz"
        if cache.exists():
            with gzip.open(cache, "rt", encoding="utf-8") as inp:
                records = json.load(inp)["records"]
        else:
            records = replay.run_segment(key)
            with gzip.open(cache, "wt", encoding="utf-8") as out:
                json.dump({"key": key, "records": records}, out, separators=(",", ":"))
        for label, prior in events:
            old_raw = {int(s) for s in prior["old_member_ids"].split("|") if s}
            new_raw = {int(s) for s in prior["new_member_ids"].split("|") if s}
            old = endpoint(records, int(prior["old_ns"]), float(prior["old_d"]),
                           float(prior["old_y"]), float(prior["old_v"]), old_raw)
            new = endpoint(records, int(prior["new_ns"]), float(prior["new_d"]),
                           float(prior["new_y"]), float(prior["new_v"]), new_raw)
            if old["status"] == new["status"] == "UNIQUE_GEOMETRY":
                status = "SAME_PID_ON_HEAD" if old["pid"] == new["pid"] else "DISTINCT_PID_ON_HEAD"
            else:
                status = "UNRESOLVED_REMAP"
            output.append({"event_id": label["event_id"], "route": route, "segment": segment,
                           "prior_gt": label["label"], "prior_source": label["label_source"],
                           "old_ns": prior["old_ns"], "new_ns": prior["new_ns"],
                           "prior_gap_s": prior["gap_s"], "head_status": status,
                           "old_match_status": old["status"], "new_match_status": new["status"],
                           "old_match_count": old.get("candidate_count", ""),
                           "new_match_count": new.get("candidate_count", ""),
                           "old_delta_ms": old.get("delta_ms", ""),
                           "new_delta_ms": new.get("delta_ms", ""),
                           "head_old_pid": old.get("pid", ""), "head_new_pid": new.get("pid", ""),
                           "head_old_raw": "|".join(map(str, old.get("raw_ids", []))),
                           "head_new_raw": "|".join(map(str, new.get("raw_ids", []))),
                           "old_raw_overlap": old.get("raw_overlap", ""),
                           "new_raw_overlap": new.get("raw_overlap", ""),
                           "head_old_alias": old.get("alias", ""),
                           "head_new_alias": new.get("alias", ""),
                           "head_old_published": old.get("published", ""),
                           "head_new_published": new.get("published", "")})
            for phase, endpoint_ns, match in (("OLD", int(prior["old_ns"]), old),
                                              ("NEW", int(prior["new_ns"]), new)):
                if "pid" not in match:
                    continue
                pid = match["pid"]
                for record in records:
                    if abs(record["scan_ns"] - endpoint_ns) > 1_500_000_000:
                        continue
                    obj = next((o for o in record["objects"] if o["pid"] == pid), None)
                    state = next((s for s in record["physical_states"] if s["pid"] == pid), None)
                    if obj is None and state is None:
                        continue
                    traces.append({"event_id": label["event_id"], "phase": phase,
                                   "scan_ns": record["scan_ns"], "pid": pid,
                                   "raw_ids": "|".join(map(str, obj["raw_ids"] if obj else state["raw_ids"])),
                                   "slots": "|".join(map(str, obj["slots"])) if obj else "",
                                   "representative": obj["representative"] if obj else state["representative"],
                                   "d_m": obj["d"] if obj else "", "y_m": obj["y"] if obj else "",
                                   "v_mps": obj["v"] if obj else "",
                                   "camera": obj["camera_association"] if obj else "",
                                   "oem": obj["word1"] if obj else "",
                                   "published": obj["published"] if obj else 0,
                                   "alias": obj["public_alias"] if obj else "",
                                   "state": "OBSERVED_QUALIFIED" if obj else "INTERNAL_ONLY"})
        print(route, segment, len(records), len(events), flush=True)
    dest = STUDY / "tables" / "sequential_head_remap.csv"
    with dest.open("w", encoding="utf-8", newline="") as out:
        writer = csv.DictWriter(out, fieldnames=list(output[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(output)
    dest = STUDY / "traces" / "sequential_endpoint_windows.csv"
    with dest.open("w", encoding="utf-8", newline="") as out:
        writer = csv.DictWriter(out, fieldnames=list(traces[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(traces)


if __name__ == "__main__":
    main()
