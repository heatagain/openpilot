"""Replay adjacent available segments with one Bosch provider instance."""

from __future__ import annotations

import csv
import gzip
import json
import sys
from pathlib import Path


ROOT = next(parent for parent in Path(__file__).resolve().parents
            if (parent / "openpilot" / "opendbc_repo").is_dir() and
            (parent / "analysis").is_dir())
STUDY = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "20260921_MRRevo14F_lead_ahead_identity_reacquisition" / "scripts"
sys.path.insert(0, str(SOURCE))
import replay_current_provider as replay  # noqa: E402


SETS = {
    "route26d_s14_s15": ("0000026d--2363789071--14", "0000026d--2363789071--15"),
    "route28f_s35_s37": ("0000028f--bc6e81a93a--35", "0000028f--bc6e81a93a--36",
                           "0000028f--bc6e81a93a--37"),
}


def main() -> None:
    manifest = []
    for name, keys in SETS.items():
        original = replay.load_audit_module
        audit = original()
        pieces = [audit.load_segment(key) for key in keys]
        boundaries = [piece[1][0][0] for piece in pieces]
        combined = tuple([item for piece in pieces for item in piece[i]] for i in range(4))
        audit.load_segment = lambda _key: combined
        replay.load_audit_module = lambda: audit
        try:
            records = replay.run_segment(name)
        finally:
            replay.load_audit_module = original
        output = STUDY / "scratch" / f"{name}.json.gz"
        output.parent.mkdir(parents=True, exist_ok=True)
        with gzip.open(output, "wt", encoding="utf-8") as out:
            json.dump({"segments": keys, "boundaries_ns": boundaries, "records": records}, out,
                      separators=(",", ":"))
        rows = []
        for record in records:
            ns = record["scan_ns"]
            index = max(i for i, start in enumerate(boundaries) if start <= ns)
            if index != len(keys) - 1:
                continue
            segment_s = (ns - boundaries[-1]) / 1e9
            if name == "route26d_s14_s15" and not 11 <= segment_s <= 16:
                continue
            if name == "route28f_s35_s37" and not 29 <= segment_s <= 34:
                continue
            for obj in record["objects"]:
                rows.append({"segment_s": round(segment_s, 6), "scan_ns": ns,
                             "slot": "|".join(map(str, obj["slots"])),
                             "raw_id": "|".join(map(str, obj["raw_ids"])),
                             "physical_pid": obj["pid"], "public_id": obj["public_alias"],
                             "d_rel_m": obj["d"], "y_rel_m": obj["y"], "v_rel_mps": obj["v"],
                             "members": "|".join(map(str, obj["raw_ids"])),
                             "representative": obj["representative"],
                             "camera": obj["camera_association"], "oem": obj["word1"],
                             "published": obj["published"]})
        destination = STUDY / "traces" / f"{name}.csv"
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open("w", encoding="utf-8", newline="") as out:
            fields = ["segment_s", "scan_ns", "slot", "raw_id", "physical_pid", "public_id",
                      "d_rel_m", "y_rel_m", "v_rel_mps", "members", "representative", "camera",
                      "oem", "published"]
            writer = csv.DictWriter(out, fieldnames=fields, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)
        manifest.append({"name": name, "segments": keys, "boundaries_ns": boundaries,
                         "scans": len(records), "focused_rows": len(rows), "trace": str(destination)})
        print(name, len(records), len(rows), flush=True)
    (STUDY / "manifests" / "warm_replay.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
