"""Dedup-aware barrel sighting log.

A sighting is considered a duplicate when another sighting of the same class
lives within `dedup_radius` metres. First-seen sighting "scores"; repeats only
update confidence + last-seen timestamp.

Persisted to CSV so a supervisor restart can pick up where the previous attempt
crashed.
"""
from __future__ import annotations

import csv
import math
import os
import threading
import time
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional


@dataclass
class BarrelEntry:
    barrel_id: int          # monotonic, 1-based
    class_name: str         # "yellow_barrel" | "red_barrel"
    north: float
    east: float
    down: float
    confidence: float
    first_seen: float       # epoch seconds
    last_seen: float
    sightings: int          # incremented on every duplicate hit


class BarrelLog:
    """Thread-safe, file-backed barrel registry."""

    SCORES = {"yellow_barrel": 50, "red_barrel": 100}

    def __init__(
        self,
        csv_path: str,
        dedup_radius: float = 2.0,
        autoload: bool = True,
    ):
        self.csv_path = os.path.abspath(csv_path)
        self.dedup_radius = float(dedup_radius)
        self._lock = threading.Lock()
        self._entries: Dict[int, BarrelEntry] = {}
        self._next_id = 1

        os.makedirs(os.path.dirname(self.csv_path) or ".", exist_ok=True)
        if autoload and os.path.exists(self.csv_path):
            self._load_existing()

    # ---------------- persistence ----------------
    def _load_existing(self) -> None:
        with open(self.csv_path, "r", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                e = BarrelEntry(
                    barrel_id=int(row["barrel_id"]),
                    class_name=row["class_name"],
                    north=float(row["north"]),
                    east=float(row["east"]),
                    down=float(row["down"]),
                    confidence=float(row["confidence"]),
                    first_seen=float(row["first_seen"]),
                    last_seen=float(row["last_seen"]),
                    sightings=int(row["sightings"]),
                )
                self._entries[e.barrel_id] = e
                self._next_id = max(self._next_id, e.barrel_id + 1)

    def _flush(self) -> None:
        tmp = self.csv_path + ".tmp"
        with open(tmp, "w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "barrel_id", "class_name", "north", "east", "down",
                    "confidence", "first_seen", "last_seen", "sightings",
                ],
            )
            writer.writeheader()
            for e in self._entries.values():
                writer.writerow(asdict(e))
        os.replace(tmp, self.csv_path)

    # ---------------- core API ----------------
    def _find_match(self, class_name: str, north: float, east: float) -> Optional[BarrelEntry]:
        """Linear nearest-neighbour. Fine up to ~hundreds of entries."""
        best: Optional[BarrelEntry] = None
        best_d = self.dedup_radius
        for e in self._entries.values():
            if e.class_name != class_name:
                continue
            d = math.hypot(e.north - north, e.east - east)
            if d <= best_d:
                best = e
                best_d = d
        return best

    def add(
        self,
        class_name: str,
        north: float,
        east: float,
        down: float,
        confidence: float,
        ts: Optional[float] = None,
    ) -> "tuple[BarrelEntry, bool]":
        """Register a sighting.

        Returns (entry, is_new). is_new=True only on the first sighting of a
        barrel — the caller can use that to print a score-relevant log line.
        """
        ts = time.time() if ts is None else float(ts)
        with self._lock:
            match = self._find_match(class_name, north, east)
            if match is None:
                entry = BarrelEntry(
                    barrel_id=self._next_id,
                    class_name=class_name,
                    north=float(north),
                    east=float(east),
                    down=float(down),
                    confidence=float(confidence),
                    first_seen=ts,
                    last_seen=ts,
                    sightings=1,
                )
                self._entries[entry.barrel_id] = entry
                self._next_id += 1
                self._flush()
                return entry, True

            # Update existing — running-mean position, keep best confidence
            n = match.sightings
            match.north = (match.north * n + float(north)) / (n + 1)
            match.east  = (match.east  * n + float(east))  / (n + 1)
            match.down  = (match.down  * n + float(down))  / (n + 1)
            match.confidence = max(match.confidence, float(confidence))
            match.last_seen = ts
            match.sightings = n + 1
            self._flush()
            return match, False

    # ---------------- queries ----------------
    def snapshot(self) -> List[BarrelEntry]:
        with self._lock:
            return list(self._entries.values())

    def score(self) -> int:
        with self._lock:
            return sum(self.SCORES.get(e.class_name, 0) for e in self._entries.values())

    def count_by_class(self) -> Dict[str, int]:
        with self._lock:
            counts: Dict[str, int] = {}
            for e in self._entries.values():
                counts[e.class_name] = counts.get(e.class_name, 0) + 1
            return counts


if __name__ == "__main__":
    log = BarrelLog("/tmp/barrels_test.csv", dedup_radius=2.0, autoload=False)
    if os.path.exists(log.csv_path):
        os.remove(log.csv_path)
    log = BarrelLog("/tmp/barrels_test.csv", dedup_radius=2.0)

    e1, new1 = log.add("yellow_barrel", 5.0, 5.0, -1.0, 0.9)
    e2, new2 = log.add("yellow_barrel", 5.3, 5.1, -1.0, 0.95)   # dup
    e3, new3 = log.add("red_barrel",    5.0, 5.0, -3.0, 0.8)    # diff class
    e4, new4 = log.add("yellow_barrel", 20.0, 20.0, -1.0, 0.7)  # new

    print(f"new flags: {new1} {new2} {new3} {new4}  (expect T F T T)")
    print(f"entries: {len(log.snapshot())}  (expect 3)")
    print(f"score: {log.score()}  (expect 50+100+50 = 200)")
    print(f"counts: {log.count_by_class()}")
