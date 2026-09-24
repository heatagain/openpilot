"""Aggregate surface switches, dwell, ping-pong, jump and divergence caches."""
from evaluate_publication_candidates import finalize


if __name__ == "__main__":
  result = finalize()
  print({"routes": result["routes"], "segments": result["segments"],
         "scans": result["completed_scans"], "best_candidate": result["best_candidate"]})
