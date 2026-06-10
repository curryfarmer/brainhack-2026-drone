# PAD-DETECT round 2 — adversarial re-audit (re-review of round 1 + new gaps)

Re-audit after the round-1 fixes/notes. 1341 tests green (1310 baseline + 31
new). Re-checked every round-1 item, then hunted NEW gaps in the source filter,
the heterogeneous-key sort, the event-message wording, and from_config flow.

Format: `path:line: <SEV> <tag>: <problem>. <fix>.`

## Re-audit of round-1 items
- R1 LOW BUG (double recompute of `_servo_candidates` in `_servo_target`):
  CONFIRMED benign + documented. KEPT — the named `_servo_target(ctx)` seam is
  spec-mandated and the recompute is a pure, deterministic, ≤handful-of-items
  filter at ~10 Hz. No change.
- R1 LOW BUG (heterogeneous 4th sort-key element marker-str vs pad-str):
  RE-VERIFIED. `_servo_candidates` returns a HOMOGENEOUS list every call
  (all-marker in marker mode, all-pad in pad mode), so `_pick_target` never sees
  a mixed list; the leading `kind` flag would prevent a str-vs-other compare even
  if it did. No real path to a TypeError. No change.
- R1 LOW BUG (empty pad_classes accepted in marker mode, malformed rejected):
  RE-VERIFIED intentional + both pinned by tests. No change.

## NEW gaps found this round (all addressed)
finals/mission/phases/land_on_pad.py:430: MED TEST (FIXED this round): the pad
filter is TWO-part (`source=="yolo"` AND `class_name in pad_classes`) but round
1 only pinned the class half. A mutant dropping the `source=="yolo"` clause
(accept any source carrying the pad class — e.g. a future 'pad' colour blob or a
mislabelled 'aruco') SURVIVED. Killed this round by
`test_servo_target_pad_mode_requires_yolo_source` (a 'pad'-source and an
'aruco'-source sighting, both carrying class_name "landing_pad", must NOT be
targets). Mutation re-run: dropping the source clause → that test FAILS. KILLED.
NOTE (scope): a `source=="pad"` COLOUR-blob sighting is the SEPARATE
colour-detector approach (Step-0 `Sighting` source union includes "pad"); this
session's spec is explicitly the YOLO (`source=="yolo"`) servo, so the pad
filter correctly excludes "pad"-source blobs.

finals/mission/phases/land_on_pad.py:592: LOW (verified OK): the acquire-timeout
Fallback message now branches on `servo_on` to name pad_classes vs
valid_marker_ids — checked both branches render finite, sorted, actionable text
(no crash on an empty pad_classes, which is impossible in pad mode anyway). The
"saw N/M of the last frames with a servo target" wording is mode-neutral and
correct for both. No change.

finals/mission/phases/land_on_pad.py:520: LOW (verified OK): the VERIFIED_LANDING
Done message uses `self._target_label or f"marker {self._target_marker_id}"`.
In pad mode `_target_label` is set ("yolo pad 'landing_pad'") so the `or`
fallback is dead in practice; in the degenerate case where is_flying flips False
BEFORE any target was ever locked (immediate-on-deck entry), `_target_label` is
None and `_target_marker_id` is None → "marker None", which is honest (no target
was locked — the legacy text path, unchanged from NAV-6). Acceptable.

## CONVENTIONS (mechanical) — re-scan
finals/mission/phases/land_on_pad.py: PASS (unchanged from R1): pure module,
no bare/blanket except, no top-level numpy/SDK, no new while-loops, typed errors,
module-level CONSTANT (not mutable global), DEFAULT servo_on="marker" keeps the
legacy path byte-identical (1310 baseline tests untouched). The phase still does
NO marker/pad DETECTION itself — it only reads `ctx.sightings` the PerceptionLoop
already published; the YOLO weights remain a DATA artifact (documented contract).

## Mutation kill-check summary (file restored clean after each)
6/6 mutants killed by NAMED tests:
  (a) drop pad_classes filter → test_servo_target_pad_mode_ignores_non_pad_yolo_class
  (b) silent marker fallback in pad mode → test_pad_mode_agent_lands_done_on_canned_pad_stream
  (c) area sign → test_pad_mode_picks_largest_bbox_deterministically
  (c) marker tie-break sign → test_two_valid_markers_equal_area_tie_break_lowest_id
  (c) pad tie-break sign → test_pad_mode_equal_area_tie_break_lowest_class_name
  (d) drop source=="yolo" clause → test_servo_target_pad_mode_requires_yolo_source

## Verdict
CLEAN. No open MED/HIGH findings. Merge-boundary with PAD-VALID is the single
`step()` line + disjoint geometry methods (round-1 OWNERSHIP section). Onsite
deferral: YOLO weights training (the .pt + its pad class label) and gate-F
calibration of the pad-servo k_lateral/commit_alt vs the pad blob — config/data,
not code.
