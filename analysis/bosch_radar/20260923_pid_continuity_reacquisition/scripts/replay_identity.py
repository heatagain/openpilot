"""Read-only Bosch PID continuity audit against the checked-out provider.

Uses the existing 847-segment decoded input cache and its replay harness. Each
segment is cold-started; cross-segment PID continuity is therefore unevaluable.
"""

from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import json
import sys
from pathlib import Path


ROOT = next(parent for parent in Path(__file__).resolve().parents
            if (parent / "openpilot" / "opendbc_repo").is_dir() and
            (parent / "analysis").is_dir())
STUDY = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "20260921_MRRevo14F_lead_ahead_identity_reacquisition" / "scripts"
CORPUS = ROOT / "analysis" / "20260919_MRRevo14F_parent_sibling_publication_shadow" / "scratch" / "corpus"
sys.path.insert(0, str(SOURCE))
import replay_current_provider as replay  # noqa: E402


CASES = (
    ("route26d", 14), ("route26d", 15), ("route26d", 96),
    ("route28b", 33), ("route28f", 32), ("route28f", 37),
    ("route259", 2), ("route259", 11), ("route264", 6),
    ("route264", 22), ("route27f", 16), ("route274", 33),
    ("route280", 7), ("route280", 15),
)


def csv_write(path: Path, rows: list[dict], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as out:
        writer = csv.DictWriter(out, fieldnames=fields, lineterminator="\n", extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def signature(record: dict) -> str:
    """The observable identity/publication state at a completed scan."""
    value = {
        "scan_ns": record["scan_ns"],
        "objects": record["objects"],
        "physical_states": record["physical_states"],
        "provisional": record["provisional"],
        "p91": record["p91"],
        "oem_gate": record["oem_gate"],
        "camera_extended": record["camera_extended"],
        "camera_companion": record["camera_companion"],
        "family_companion": record["family_companion"],
        "candidate_b_burst": record["candidate_b_burst"],
    }
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def run_prefix(key: str, limit: int) -> list[dict]:
    original = replay.load_audit_module
    audit = original()
    load_segment = audit.load_segment

    def truncated(segment_key: str):
        can, states, poses, models = load_segment(segment_key)
        return can, states[:limit], poses, models

    audit.load_segment = truncated
    replay.load_audit_module = lambda: audit
    try:
        return replay.run_segment(key)
    finally:
        replay.load_audit_module = original


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--prefix", action="store_true", help="rerun selected segment prefixes")
    args = parser.parse_args()
    (STUDY / "scratch").mkdir(parents=True, exist_ok=True)
    inventory, events, timelines, prefix_results = [], [], [], []
    for route, segment in CASES:
        matches = sorted(CORPUS.glob(f"{int(route[5:], 16):08x}--*--{segment}.json.gz"))
        row = {"route": route, "segment": segment, "cache_count": len(matches),
               "cache_path": str(matches[0]) if len(matches) == 1 else "",
               "status": "AVAILABLE" if len(matches) == 1 else "NOT AVAILABLE"}
        inventory.append(row)
        if len(matches) != 1:
            continue
        key = matches[0].name.removesuffix(".json.gz")
        records = replay.run_segment(key)
        if not records:
            row["status"] = "NO COMPLETED SCANS"
            continue
        row["scans"] = len(records)
        with gzip.open(STUDY / "scratch" / f"{key}.json.gz", "wt", encoding="utf-8") as out:
            json.dump({"key": key, "records": records}, out, separators=(",", ":"))
        first_ns = records[0]["scan_ns"]
        previous: dict[int, dict] = {}
        previous_state: set[int] = set()
        for record in records:
            ns = record["scan_ns"]
            sec = round((ns - first_ns) / 1e9, 6)
            current = {obj["pid"]: obj for obj in record["objects"]}
            states = {state["pid"]: state for state in record["physical_states"]}
            for pid, obj in current.items():
                prior = previous.get(pid)
                kinds = []
                if pid not in previous_state:
                    kinds.append("PID_BIRTH_OR_REENTRY")
                if prior is not None:
                    if prior["raw_ids"] != obj["raw_ids"]:
                        kinds.append("MEMBER_CHANGE")
                    if prior["slots"] != obj["slots"]:
                        kinds.append("SLOT_CHANGE")
                    if prior["representative"] != obj["representative"]:
                        kinds.append("REPRESENTATIVE_SWITCH")
                    if prior["public_alias"] != obj["public_alias"]:
                        kinds.append("ALIAS_SWITCH")
                    if not prior["published"] and obj["published"]:
                        kinds.append("PUBLICATION_RESUME")
                    if prior["published"] and not obj["published"]:
                        kinds.append("PUBLICATION_STOP")
                if pid in previous_state and pid not in previous:
                    kinds.append("COAST_RECOVERY_OR_QUALIFICATION_RESUME")
                if kinds:
                    events.append({"route": route, "segment": segment, "scan_s": sec,
                                   "scan_ns": ns, "pid": pid, "event": "|".join(kinds),
                                   "slots": "|".join(map(str, obj["slots"])),
                                   "raw_ids": "|".join(map(str, obj["raw_ids"])),
                                   "rep": obj["representative"], "d_m": obj["d"],
                                   "y_m": obj["y"], "v_mps": obj["v"],
                                   "camera": obj["camera"], "oem": obj["word1"],
                                   "published": obj["published"], "public_alias": obj["public_alias"]})
                timelines.append({"route": route, "segment": segment, "scan_s": sec,
                                  "scan_ns": ns, "slot": "|".join(map(str, obj["slots"])),
                                  "raw_id": "|".join(map(str, obj["raw_ids"])),
                                  "physical_pid": pid, "public_id": obj["public_alias"],
                                  "d_rel_m": obj["d"], "y_rel_m": obj["y"],
                                  "v_rel_mps": obj["v"], "members": "|".join(map(str, obj["raw_ids"])),
                                  "representative": obj["representative"],
                                  "camera": obj["camera_association"], "oem": obj["word1"],
                                  "published": obj["published"], "state": "OBSERVED_QUALIFIED"})
            for pid, state in states.items():
                if pid in current:
                    continue
                timelines.append({"route": route, "segment": segment, "scan_s": sec,
                                  "scan_ns": ns, "physical_pid": pid,
                                  "members": "|".join(map(str, state["raw_ids"])),
                                  "representative": state["representative"],
                                  "published": 0, "state": "INTERNAL_STATE_NO_QUALIFIED_OBJECT"})
                if pid in previous_state and pid in previous:
                    events.append({"route": route, "segment": segment, "scan_s": sec,
                                   "scan_ns": ns, "pid": pid, "event": "OBSERVATION_OR_QUALIFICATION_GAP"})
            previous, previous_state = current, set(states)
        if args.prefix and (route, segment) in {("route26d", 15), ("route28f", 32), ("route264", 6)}:
            for fraction in (1 / 3, 2 / 3):
                # run_segment consumes one carState update at a time; a few
                # updates may not close a scan, so compare the emitted prefix.
                limit = int(len(records) * 2 * fraction)
                prefix = run_prefix(key, limit)
                same = all(signature(a) == signature(b) for a, b in zip(prefix, records))
                prefix_results.append({"route": route, "segment": segment,
                                       "state_limit": limit, "prefix_scans": len(prefix),
                                       "full_scans": len(records), "equal": int(same),
                                       "valid_prefix": int(len(prefix) <= len(records))})
        print(f"{key}: {len(records)} scans", flush=True)
    csv_write(STUDY / "manifests" / "input_inventory.csv", inventory,
              ["route", "segment", "cache_count", "cache_path", "status", "scans"])
    csv_write(STUDY / "tables" / "identity_events.csv", events,
              ["route", "segment", "scan_s", "scan_ns", "pid", "event", "slots", "raw_ids",
               "rep", "d_m", "y_m", "v_mps", "camera", "oem", "published", "public_alias"])
    csv_write(STUDY / "scratch" / "all_qualified_timeline.csv", timelines,
              ["route", "segment", "scan_s", "scan_ns", "slot", "raw_id", "physical_pid",
               "public_id", "d_rel_m", "y_rel_m", "v_rel_mps", "members", "representative",
               "camera", "oem", "published", "state"])
    csv_write(STUDY / "tables" / "prefix_invariance.csv", prefix_results,
              ["route", "segment", "state_limit", "prefix_scans", "full_scans", "equal", "valid_prefix"])


if __name__ == "__main__":
    main()
