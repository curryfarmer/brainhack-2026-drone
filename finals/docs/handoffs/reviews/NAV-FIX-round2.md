# NAV-FIX — adversarial self-review, round 2 (re-audit + new gaps)

Re-audited the round-1 fixes and swept for new gaps. Format
`path:line: <SEV> <tag>: <problem>. <fix>.`

## Re-audit of round-1 fixes

- `map_sensing.py:_marker_point`: the NaN-bypass fix holds — both a Marker's
  `point_m` and a raw pair now go through `_require_point`, so a
  directly-constructed `Marker(id, (nan, 0))` raises before reaching min/max.
  VERIFIED.

## New gaps found in round 2

- `configs/arenas/field.json: INFO empties-by-design`: ships with empty `pads`
  / `keep_out` (markers-only). CHECKED that NO config references it, so
  `_resolve_arena` never auto-loads it and the config-level pad-validity
  cross-checks (`_validate_landing_pads`) are never run against it. It is a
  standalone gate-D artifact the operator fills (pads/keep-outs surveyed onsite).
  Locked by `test_shipped_field_arena_loads_with_all_5_beacons_in_organizer_frame`
  so a future edit can't silently break the binding / id set. ACCEPTED.

- `types.py:ArenaMap.from_dict strict_marker_ids resolution: VERIFIED no-regress`:
  for an arena WITHOUT the flag and no kwarg, `known_marker_ids` stays None ⇒ NO
  restriction (`test_step0_contracts` ids 11/51 + the e2e id 7 still pass — full
  suite green). The config path `_resolve_arena` calls `from_dict(raw, name=name)`
  (no kwarg), so a flag-bearing arena self-validates via the JSON flag — confirmed
  by loading `field.json` through the same call shape. NO change to existing
  arena loads.

- `bounds_from_markers_and_cage docstring: INFO imprecision`: says the cage
  "normally dominates" — true only when every marker is interior to the cage; a
  marker outside grows the bound (the contains-all property). The word "normally"
  + the explicit union contract + the
  `test_bounds_cage_smaller_than_markers_still_contains_markers` case make the
  exception unambiguous. ACCEPTED (no code change).

- `navigate.py:marker_id lookup: VERIFIED`: `{m.id: m for m in arena.markers}` is
  collision-free because `ArenaMap.from_dict` already enforces unique marker ids
  upstream; an arena with no markers yields an empty dict and the loud "not a
  beacon in this arena (does this arena declare markers?)" message. The int-id +
  bool guard mirrors `Marker.from_dict`.

- `dead_reckon.py:apply_position_fix: VERIFIED purity`: no SDK / numpy / arena
  import; reuses the module's `_require_finite` boundary guard; keeps alt + yaw
  + the DEAD_RECKONING quality (pinned by
  `test_apply_position_fix_keeps_quality_dead_reckoning`). The floor (open-loop)
  behaviour is unchanged — nothing calls it in the default flow.

- Conventions (`tests/test_conventions.py`): green — `map_sensing.py` / `frame.py`
  stay pure (stdlib `math` + `typing` only), no bare `except`, no module-level
  mutable globals (`KNOWN_FIELD_MARKER_IDS` is a frozenset constant).

## Result

No open findings. 4/4 named-mutant kill-check held (re-run after the round-1 +
round-2 edits). Full suite 1352 green (baseline 1310 + 42 NAV-FIX). Clean.
