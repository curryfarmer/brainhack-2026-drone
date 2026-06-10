# NAV-FIX — adversarial self-review, round 1

Slice: absolute position-fix from known field beacons + the wiring around it.
Branch `nav-fix` off `roboverse-landing` Step-0 (66ee5bb). Files touched:
`mission/planning/map_sensing.py` (position_fix audit + `bounds_from_markers_and_cage`),
`mission/planning/frame.py` (organizer-frame binding L1), `mission/planning/types.py`
(`KNOWN_FIELD_MARKER_IDS` + soft known-id rule in `Marker.from_dict` / `ArenaMap.from_dict`),
`mission/phases/navigate.py` (`marker_id` goal source), `flight/dead_reckon.py`
(`apply_position_fix` opt-in hook), `configs/arenas/field.json` (real-arena artifact),
`tests/test_nav_fix.py` (new), `tests/test_navigate.py` (2 message regexes).

Format: `path:line: <SEV> <tag>: <problem>. <fix>.`

## Findings

- `map_sensing.py:_marker_point: LOW robustness`: a directly-constructed
  `Marker(id, (nan, 0))` bypassed finiteness validation (only raw pairs were
  `_require_point`-checked), so a NaN could slip into `min/max` and silently
  poison the derived bound (`NaN < x` always False). FIXED — `_marker_point` now
  routes BOTH the Marker's `point_m` and raw pairs through `_require_point`.

- `navigate.py:goal-resolution: INFO behaviour-preserving message change`: the
  exactly-one-goal guard messages changed ("BOTH"→"MULTIPLE goal sources",
  "NEITHER"→"sets NO goal") to cover the new third source `marker_id`. The
  CONTRACT (reject 0 / reject >1) is unchanged. Grepped the repo: no production
  code or other test references the old wording (the config separation-guard
  "NEITHER" mentions are unrelated). The 2 pinning tests in `test_navigate.py`
  were updated to the new regexes. ACCEPTED.

- `dead_reckon.py:apply_position_fix: LOW / BY-DESIGN`: the hook does NOT clamp
  the fix to arena bounds — a misdecoded marker could teleport the estimate.
  Clamping would couple the pure DR class to the arena (it has zero arena
  knowledge today). DECISION: keep DR pure; the OPT-IN method's docstring states
  the caller (vision) owns the "only when trustworthy" duty, and
  `position_fix_from_marker` already rejects NaN/neg range upstream. ACCEPTED
  (the floor behaviour is unchanged: nothing calls it by default).

- `types.py:ArenaMap.from_dict strict_marker_ids: INFO ordering`: the strict-flag
  bool check fires before bounds parsing, so a config with BOTH a non-bool flag
  AND malformed bounds reports the flag error first. Loud either way; no silent
  pass. ACCEPTED.

- `map_sensing.py:position_fix_from_marker: AUDIT (not a finding)`: verified the
  sign against `flight/dead_reckon.py` `_integrate_move` (FORWARD: dN=cos θ·d,
  dE=−sin θ·d; psi_NED=−yaw_deg). The implementation `drone = (M_n − r·cos b,
  M_e + r·sin b)` is the correct inverse of the FORWARD map (marker = drone +
  forward-offset at bearing b). Pinned by the existing `test_map_sensing`
  3-4-5 / due-north / due-east fixtures; docstring now cites dead_reckon
  explicitly as the source of truth. CORRECT, no change.

## Mutation kill-check (the 3 named targets + 1 bonus)

- (a) fix sign flipped (`M_n − r·cos b` → `M_n + r·cos b`): `test_map_sensing`
  `test_fix_general_3_4_5_triangle` + `test_fix_marker_due_north_recovers_origin`
  FAIL. KILLED.
- (b) bounds drops contains-all-markers (cage replaces the marker union instead
  of `min/max`-ing with it): `test_nav_fix
  ::test_bounds_cage_smaller_than_markers_still_contains_markers` FAILS (marker
  (4.4,1.35) falls outside the cage-only (0,0,1,1)). KILLED.
- (c) frame axes swapped (`organizer_xy_to_ne` returns (x,y) identity):
  `test_organizer_frame_round_trip_exact`, `..._maps_long_to_north_short_to_east`,
  `..._is_a_swap_not_identity` all FAIL. KILLED.
- bonus — strict known-id guard neutered (`if False and ...`):
  `test_strict_marker_ids_json_flag_opts_in` +
  `test_unknown_marker_id_rejected_when_strict_arg_passed` FAIL. KILLED.

4/4 mutants killed. All reverted clean.

Tests: full suite 1352 green (baseline 1310 + 42 NAV-FIX). Round 2 follows.
