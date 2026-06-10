# PAD-DETECT round 1 — adversarial review (phases/land_on_pad.py + test_land_on_pad.py + test_replay_e2e.py)

Scope: the `servo_on="pad"` YOLO pad-servo extension to land_on_pad — the
`servo_on`/`pad_classes` tunables + validation, the mode-aware servo-geometry
selectors (`_pad_sightings`/`_servo_candidates`/`_servo_target`/`_pick_target`),
backward-compat with the default "marker" mode, the merge-disjointness with
PAD-VALID's `_valid_sightings`, the WEIGHTS/CLASS contract docs. 1340 tests
green at review time (1310 baseline + 30 new). cv2 4.11.0 / numpy 1.26.4.

Format: `path:line: <SEV> <tag>: <problem>. <fix>.`

## BUGS / correctness
finals/mission/phases/land_on_pad.py:573,653,697: LOW BUG: `step()` computes
`valid = self._servo_candidates(ctx)` and passes it down, but `_step_center` /
`_step_descend` then call `self._servo_target(ctx)` which RECOMPUTES
`_servo_candidates(ctx)` a second time. The two results are identical (ctx is
frozen + the filter is pure/deterministic), so no correctness bug — but it is a
redundant per-tick recompute. KEPT deliberately: the spec mandates the named
`_servo_target(ctx)` seam (the merge boundary PAD-VALID does not touch), and a
double filter of a handful of sightings at ~10 Hz is negligible. Flag-only.

finals/mission/phases/land_on_pad.py:449: LOW BUG: `_pick_target`'s sort key
returns a 4-tuple whose last element is a `str` for markers and a `str` for
pads (or a `list[int]` only under the mutation harness) — if a candidate list
ever MIXED a marker (kind 0) and a pad (kind 1), the `kind` flag (2nd element)
always differs first so the heterogeneous 4th elements are never compared →
no TypeError. In production `_servo_candidates` returns a HOMOGENEOUS list
(all markers OR all pads, never mixed), so the mixed case cannot occur; the
`kind` flag is defensive-only. Verified safe; documented in the docstring.

finals/mission/phases/land_on_pad.py:363: LOW BUG: in "marker" mode a malformed
`pad_classes` (e.g. `[1,2,3]`) is REJECTED even though it is unused, but an
EMPTY `pad_classes` (`[]`) in marker mode is ACCEPTED (stored as `frozenset()`).
Intentional asymmetry: an empty set is the harmless default; a non-string entry
is a structural config bug that would become a landmine if `servo_on` later
flips to "pad". Pinned by `test_marker_mode_ignores_missing_pad_classes` +
`test_marker_mode_rejects_malformed_pad_classes`.

## MISSING / WEAK tests (mutation-targeted)
finals/tests/test_land_on_pad.py: PASS: the 3 spec-named mutants are each KILLED
by a NAMED test (mutation kill-check run, file restored clean afterward):
  (a) `_pad_sightings` drops the `class_name in self.pad_classes` filter (accept
      any yolo class) → `test_servo_target_pad_mode_ignores_non_pad_yolo_class`
      FAILS (+ 2 more: decoy_class_only, pad_plus_decoy). KILLED.
  (b) `_servo_candidates` always returns `_valid_sightings` (silent marker
      fallback in pad mode) → `test_pad_mode_agent_lands_done_on_canned_pad_stream`
      FAILS (+ 3 more). KILLED.
  (c) area-sign flipped (`-area`→`+area`, picks SMALLEST) →
      `test_pad_mode_picks_largest_bbox_deterministically` +
      `test_two_valid_markers_picks_largest_bbox_deterministically` FAIL. KILLED.
  (c') marker tie-break sign flipped (`marker_id`→`-marker_id`) →
      `test_two_valid_markers_equal_area_tie_break_lowest_id` FAILS. KILLED.
  (c'') pad tie-break sign flipped (reverse class_name order) →
      `test_pad_mode_equal_area_tie_break_lowest_class_name` FAILS. KILLED.
  => 5/5 mutants killed.

finals/tests/test_land_on_pad.py: MED TEST (addressed): the pad-mode "lost"
gating (a non-pad yolo class during CENTER counts as LOST) is covered by
`test_pad_mode_decoy_class_only_frame_is_lost`, and pad+decoy-in-same-frame by
`test_pad_mode_pad_plus_decoy_centers_on_pad_only` — mirrors the marker-mode
decoy tests so the pad path has the same anti-decoy guard.

finals/tests/test_replay_e2e.py:108: INFO TEST: the YOLO→Sighting DATA contract
(the canned/ultralytics detector backend → `source="yolo"`, the pad class_name,
`marker_id None`, a real bbox — exactly what `_pad_sightings` filters) is pinned
end-to-end via the EXISTING canned backend in
`test_replay_canned_pad_detection_matches_pad_servo_contract`. This closes the
seam between the (user-trained) weights and the phase without needing a model.

## CODE-OPTIMIZATION PASS
finals/mission/phases/land_on_pad.py:467: LOW OPT [hot path]: `_servo_target`
recomputes `_servo_candidates` (see the LOW BUG above). NOT changed — the named
seam is spec-mandated and the cost is nil. No action.
finals/mission/phases/land_on_pad.py:449: LOW OPT [hot path]: `_pick_target`
rebuilds the `_key` closure each call (≤2 calls/tick) — same micro-alloc note as
the original `max(... lambda ...)`. Do NOT change for readability. Est nil.
finals/mission/phases/land_on_pad.py:573: INFO OPT: the single `step()` mode
fork is a ONE-LINE change (`_valid_sightings`→`_servo_candidates`) that routes
through the new dispatcher; in marker mode `_servo_candidates` is exactly
`_valid_sightings`, so the legacy hot path is unchanged. No budget/fail-loud
gate was touched.

## CONVENTIONS (mechanical)
finals/mission/phases/land_on_pad.py: PASS: no bare `except` / no
`except Exception` added; no top-level numpy or SDK import (still stdlib
enum/math/collections/typing only — the phase stays PURE); no new while-loops;
errors typed (ConfigError in __init__ via the existing `_bad` helper; the
servo_on/pad_classes failures carry WHAT/WHICH(the key+value)/WHY/CHECK); units
in names unchanged; the new VALID_SERVO_MODES is a module-level CONSTANT tuple
(not a mutable global — convention 4 OK). `from_config` flows the two new knobs
through the existing `_zone_kwargs` typo-trap automatically (added to
`_TUNABLES`). DEFAULT `servo_on="marker"` keeps every existing land/nav-e2e test
green (76/76).

## OWNERSHIP / merge boundary (PAD-VALID hand-merge)
finals/mission/phases/land_on_pad.py: PASS: PAD-DETECT's edits are confined to
the SERVO-GEOMETRY methods — `_TUNABLES`, `__init__` (the two new kwargs +
their validation block, appended after `scan_dwell_s`), the new
`_pad_sightings`/`_servo_candidates`/`_servo_target`/`_remember_target` helpers,
`_pick_target` (rewritten for the None-marker_id case), and the target-pick
lines inside `_step_acquire`/`_step_center`/`_step_descend`. `_valid_sightings`
(PAD-VALID's validity predicate) is UNCHANGED and is still the marker-mode
candidate source via `_servo_candidates`. The one shared-method touch is the
single `step()` line `valid = self._servo_candidates(ctx)` (was
`self._valid_sightings(ctx)`) — a trivial hand-merge if PAD-VALID also edits
`step()`.
