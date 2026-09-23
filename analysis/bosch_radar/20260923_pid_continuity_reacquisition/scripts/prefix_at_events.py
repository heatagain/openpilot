"""Check current-provider output on event prefixes against full cold replay."""

from __future__ import annotations

import csv
import gzip
import json
from pathlib import Path

from replay_identity import ROOT, STUDY, replay, run_prefix, signature


EVENTS = (
    ("route26d", 15, 14.7),
    ("route28f", 32, 52.2),
    ("route28f", 37, 32.2),
    ("route264", 6, 5.3),
    ("route259", 2, 25.5),
)


def main() -> None:
    rows = []
    corpus = ROOT / "analysis" / "20260919_MRRevo14F_parent_sibling_publication_shadow" / "scratch" / "corpus"
    for route, segment, target_s in EVENTS:
        key = next(corpus.glob(f"{int(route[5:], 16):08x}--*--{segment}.json.gz")).name.removesuffix(".json.gz")
        audit = replay.load_audit_module()
        _can, states, _poses, _models = audit.load_segment(key)
        cutoff_ns = states[0][0] + round(target_s * 1e9)
        limit = next((i for i, state in enumerate(states) if state[0] > cutoff_ns), len(states))
        prefix = run_prefix(key, limit)
        with gzip.open(STUDY / "scratch" / f"{key}.json.gz", "rt", encoding="utf-8") as inp:
            full = json.load(inp)["records"]
        same = len(prefix) <= len(full) and all(signature(a) == signature(b)
                                              for a, b in zip(prefix, full))
        rows.append({"route": route, "segment": segment, "event_s": target_s,
                     "state_limit": limit, "prefix_scans": len(prefix),
                     "full_scans": len(full), "equal": int(same),
                     "last_prefix_ns": prefix[-1]["scan_ns"] if prefix else ""})
        print(route, segment, target_s, len(prefix), same, flush=True)
    dest = STUDY / "tables" / "prefix_event_invariance.csv"
    with dest.open("w", encoding="utf-8", newline="") as out:
        writer = csv.DictWriter(out, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
