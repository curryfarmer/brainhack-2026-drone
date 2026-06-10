# NAV stubbed capabilities — registry

Higher-level ideas explored during the S11 nav build that we deliberately landed
as **stubs** (contract + seam now, implementation deferred — the repo's "stub
first" philosophy). Each is a real `NotImplementedError` that points back at
`module_map.md`; each has tests pinning its contract shape so a future session
fills the body without re-litigating the interface. The **active** mechanism for
each lives alongside and keeps working today.

Conventions every stub here obeys: PURE (stdlib only — bare-venv suite stays
green), typed `NotImplementedError("finals.<module>...: see module_map.md")`,
units in names. Filling one = implement the pure geometry in place, put any
cv2/image/SDK work in `vision/`, add hand-computed tests, flip the module_map row.

---

## 1. Map partition — per-drone "covers a certain area only"

- **Module:** `finals/mission/planning/partition.py`
- **Symbols:** `DroneRegion(drone_id, keep_in_polygon_m)` (contract, real) ·
  `region_to_keep_outs(mine, others)` (**STUB**)
- **Tests:** `finals/tests/test_partition.py` (contract + stub-raises)
- **Idea (user, 2026-06-10):** one overall map (`ArenaMap`), partitioned so each
  drone is confined to its own area.
- **Binding constraint:** drones are POSITION-BLIND ⇒ an area assignment can
  NEVER be closed-loop enforced in flight. A real partition is enforced at PLAN
  time: confine each drone's visibility-graph plan to its region (keep-in) and
  treat the OTHER drones' regions as keep-outs ⇒ inflated corridors disjoint ⇒
  spatial deconfliction BY CONSTRUCTION (inflation margin absorbs drift).
- **Active mechanism instead:** advisory `SectorGuard` (radial wedge from C2,
  dead-reckon estimate, logs a trip, never gates control) + time-staggered
  launch + serialized landing (NAV-8).
- **Fill steps:** (1) add an optional region polygon per drone (arena.json or
  `DroneConfig.zone`), validated loud (NAV-2 pattern); (2) implement
  `region_to_keep_outs` (+ a keep-in clip); (3) `navigate.from_config` passes the
  region + derived keep-outs into `visibility_graph.plan`; (4) test that two
  drones' inflated corridors are disjoint.
- **Why stubbed:** user chose "keep as stub first."

## 2. Map sensing — build/assist the obstacle map instead of hand-measuring it

- **Module:** `finals/mission/planning/map_sensing.py`
- **Symbols:** `position_fix_from_marker(marker_world_m, bearing_deg,
  ground_range_m, drone_yaw_deg)` (**STUB**, lever C) ·
  `keep_outs_from_overhead_corners(corners_by_id)` (**STUB**, lever A)
- **Tests:** `finals/tests/test_map_sensing.py`
- **Idea (user, 2026-06-10):** "what if I can't walk around and measure
  everything — we need dynamic map sensing."
- **Hardware verdict (binding):** full real-time SLAM is INFEASIBLE on this
  airframe — position-blind (no XY to anchor a map), DOWN-looking cam (sees the
  floor, not crates ahead), no cam IMU, depth only ~1.5-3 m ⇒ every observation
  lands in the drifting dead-reckon frame and the map smears.
- **The two real levers:**
  - **C — `position_fix_from_marker`:** a known-coord ArUco decode gives
    range+bearing to a known point ⇒ absolute XY ⇒ RESETS dead-reckon drift.
    Foundational (also sharpens transit + landing) and the enabler for B.
  - **A — `keep_outs_from_overhead_corners`:** operator taps crate corners on ONE
    overhead image (drone hover-high once / phone over the cage) → validated
    `keep_out` polygons. Highest ROI "no tape measure" path.
  - **B — recon mosaic** (described, no signature yet): low pre-flight lawnmower
    scan, detect footprints from above, georeference by dead-reckon ANCHORED with
    C's fixes. Spans modules (recon phase + cv2 in `vision/`) when built.
- **Active mechanism instead:** hand-authored `arena.json` `keep_out`, with the
  already-working "omit `keep_out` ⇒ straight-line transit" fallback (only
  `bounds_m` + `c2` pose required; obstacles/pads/lanes optional).
- **Why stubbed:** user chose "stub the seam first."

---

## Not stubs (related, but already decided)

- **`track_convoy.approach_enabled`** — built, defaults OFF (safe rotate+dwell
  observer); chase is a config flag gated on onsite Gate-E. Not a stub.
- **Omit-`keep_out` straight-line nav** — fully implemented today (empty
  `keep_out` ⇒ planner returns a direct dead-reckon leg). The cheap fallback to
  "no obstacle map." Footgun: no obstacle avoidance ⇒ overfly voids the score, so
  only safe when the straight line is clear.

## Open question parked

- A LOUD warning when `navigate` plans with zero keep-outs ("no obstacle map —
  straight-line transit; verify the path is clear or you void the score"). Cheap
  hardening, not yet built — flagged during the arena discussion.
