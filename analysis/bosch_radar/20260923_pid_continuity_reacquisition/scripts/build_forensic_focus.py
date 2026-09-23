"""Extract reviewable current-HEAD identity windows from replay caches."""

from __future__ import annotations

import csv
import gzip
import json

from replay_identity import CORPUS, STUDY


WINDOWS = (
    ("R264_S6_SPLIT", "route264", 6, 1.2, 7.2, {1000000, 1000037}, "SAME_SURFACE_FRAGMENT"),
    ("R28F_S32_TRUCK", "route28f", 32, 49.5, 54.5, {1000539, 1000540}, "SAME_SURFACE_FRAGMENT"),
    ("R259_S2_HARD_NEGATIVE", "route259", 2, 21.8, 27.8, {1000004, 1000106}, "DIFFERENT_OTHER"),
    ("R280_S7_FRAGMENT", "route280", 7, 20.6, 26.6, {1000015, 1000227}, "SAME_SURFACE_FRAGMENT"),
    ("R28B_S33_FALSE_LEAD", "route28b", 33, 15.5, 21.5, {1000118}, "IDENTITY_UNVERIFIED"),
    ("R28F_S37_FALSE_LEAD", "route28f", 37, 28.0, 34.0, {1000369, 1000002}, "IDENTITY_UNVERIFIED"),
)


def main() -> None:
    rows = []
    for case, route, segment, start, end, pids, gt in WINDOWS:
        key = next(CORPUS.glob(f"{int(route[5:], 16):08x}--*--{segment}.json.gz")).name.removesuffix(".json.gz")
        with gzip.open(STUDY / "scratch" / f"{key}.json.gz", "rt", encoding="utf-8") as inp:
            records = json.load(inp)["records"]
        first_ns = records[0]["scan_ns"]
        for record in records:
            ns = record["scan_ns"]
            sec = (ns - first_ns) / 1e9
            if not start <= sec <= end:
                continue
            qualified = {obj["pid"]: obj for obj in record["objects"]}
            states = {state["pid"]: state for state in record["physical_states"]}
            for pid in sorted(pids):
                obj, state = qualified.get(pid), states.get(pid)
                if obj is None and state is None:
                    continue
                rows.append({"case": case, "route": route, "segment": segment,
                             "scan_s": round(sec, 6), "scan_ns": ns,
                             "raw_slot": "|".join(map(str, obj["slots"])) if obj else "",
                             "raw_track_id": "|".join(map(str, obj["raw_ids"])) if obj else "",
                             "physical_pid": pid, "public_id": obj["public_alias"] if obj else "",
                             "d_rel_m": obj["d"] if obj else "", "y_rel_m": obj["y"] if obj else "",
                             "v_rel_mps": obj["v"] if obj else "",
                             "members": "|".join(map(str, (obj["raw_ids"] if obj else state["raw_ids"]))),
                             "representative": obj["representative"] if obj else state["representative"],
                             "camera_assoc": obj["camera_association"] if obj else "",
                             "oem_word1": obj["word1"] if obj else "",
                             "published": obj["published"] if obj else 0,
                             "state": "OBSERVED_QUALIFIED" if obj else "INTERNAL_ONLY",
                             "prior_gt": gt})
    # Route269 S7: current raw482 alternates between an older group and fresh
    # singleton PIDs. Include all owners of that raw, not one cherry-picked PID.
    key = next(CORPUS.glob("00000269--*--7.json.gz")).name.removesuffix(".json.gz")
    with gzip.open(STUDY / "scratch" / f"{key}.json.gz", "rt", encoding="utf-8") as inp:
        records = json.load(inp)["records"]
    targets = (531256501507, 540736378228, 544125961580)
    first_ns = records[0]["scan_ns"]
    for record in records:
        ns = record["scan_ns"]
        if not any(abs(ns - target) <= 1_500_000_000 for target in targets):
            continue
        for obj in record["objects"]:
            if 482 not in obj["raw_ids"]:
                continue
            rows.append({"case": "R269_S7_GROUP_OWNER", "route": "route269", "segment": 7,
                         "scan_s": round((ns - first_ns) / 1e9, 6), "scan_ns": ns,
                         "raw_slot": "|".join(map(str, obj["slots"])),
                         "raw_track_id": "|".join(map(str, obj["raw_ids"])),
                         "physical_pid": obj["pid"], "public_id": obj["public_alias"],
                         "d_rel_m": obj["d"], "y_rel_m": obj["y"], "v_rel_mps": obj["v"],
                         "members": "|".join(map(str, obj["raw_ids"])),
                         "representative": obj["representative"],
                         "camera_assoc": obj["camera_association"], "oem_word1": obj["word1"],
                         "published": obj["published"], "state": "OBSERVED_QUALIFIED",
                         "prior_gt": "SAME_CONFIRMED_VIDEO"})
    dest = STUDY / "traces" / "forensic_focus.csv"
    with dest.open("w", encoding="utf-8", newline="") as out:
        writer = csv.DictWriter(out, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    print(f"{len(rows)} rows -> {dest}")


if __name__ == "__main__":
    main()
