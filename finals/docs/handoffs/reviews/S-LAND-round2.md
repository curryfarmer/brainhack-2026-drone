# S-LAND round 2 — adversarial review (phases/land_on_pad.py + _servo.py + test_land_on_pad.py)

Scope: re-verify round-1, then independent fresh pass on landing safety invariants,
the descend confirm-window double-count, decoy handling in CENTER/DESCEND, frame_w
axis, fail-loud/conventions, step() hot-path OPT, and test kill-power.

Method: read CURRENT source (land_on_pad.py 1–546, _servo.py 1–206, test 1–608),
types.py (frame_shape=(h,w); Move.distance_cm:int), search.py _zone_kwargs. 63 tests
collected (matches round 1). Mutants reasoned to survival/kill against the exact test
bodies; full suite NOT re-run (per instructions — only specific mutants traced).

Severity counts (this round, fresh + still-open): CRIT 0 / HIGH 0 / MED 5 / LOW 5.
No NEW CRIT or HIGH. All four cardinal sins remain CLOSED (see invariant table).

## Round-1 verification table
| # | Round-1 finding | Verdict | Evidence |
|---|---|---|---|
| R1-1 | `:540` MED BUG: descend confirm window double-counts center-hold frames; "confirm-before-descend" weaker than docstring | CONFIRMED | `_recent` is one shared deque(maxlen=acquire_window_frames); the per-frame hit is appended at `:418` for EVERY substate, never reset on CENTER→DESCEND entry. First descend tick's gate at `:540` reads a window already saturated by acquire+center-hold hits. Precise gap below. |
| R1-2 | `:380` LOW BUG: is_flying None after a Fallback Land never reaches Done | CONFIRMED | `:380` keys terminal Done on `is_flying is False` only; `None` falls through to the `:391` FALLBACK short-circuit which re-emits `Land()` forever. No hover (keeps grounding), but Done depends on the adapter eventually reporting False. Acceptable. |
| R1-3 | `:535` LOW BUG: DESCEND→CENTER re-gate resets _center_streak but not _loss_retries; cumulative (not consecutive) losses can Fallback | CONFIRMED | `:534-535` resets `_center_streak=0` and flips to CENTER; `_loss_retries` is only ever incremented (`:509`) and never reset anywhere in the file. A flickery-but-recovering approach accumulates lifetime losses toward `max_loss_retries`. Conservative-safe (still Lands). |
| R1-4 | `test:341` MED TEST: commit boundary untested at EXACTLY commit_alt_m; mutant `<=`→`<` at `:412` SURVIVES | CONFIRMED | `test_commit_below_alt_lands` uses 0.4, `test_commit_not_triggered_above_floor` uses 1.0, integration crosses via descend (never lands ON 0.5). No `altitude_m == commit_alt_m` assertion. `<=`→`<` survives. |
| R1-5 | `test:212` MED TEST: descend gate untested at EXACTLY descend_persist_frames; mutant `>=`→`>` at `:540` SURVIVES | CONFIRMED | `test_descend_only_when_centered_and_persistently_seen` (descend_persist=2) enters via `_force_descend` with `_recent` full → recent_hits=5≫2; `test_descend_holds...` has hits<persist. No test with recent_hits == descend_persist asserting DOWN. `>=`→`>` survives. |
| R1-6 | `test:284` MED TEST: budget-exceeded tested only in ACQUIRE+CENTER, never in DESCEND | CONFIRMED (TEST gap, NOT a bug) | Budget gate at `:397` is centralized ABOVE the substate dispatch (`:421-426`), so it bounds DESCEND today. But `test_budget_exceeded_in_*` cover only acquire/center; nothing forces PAD_DESCEND then steps past total_budget_s. A future per-substate refactor could regress unguarded. |
| R1-7 | `test:103` MED TEST: decoy-while-valid never-target + decoy-alone-in-CENTER/DESCEND-as-lost unguarded; only acquire-ignores tested | CONFIRMED | Grep of the suite for decoy/non-valid/id 99: only `test_acquire_ignores_non_valid_marker_id` (`:103`, ACQUIRE). No CENTER/DESCEND decoy test. `_valid_sightings` filter at `:319-321` is the only guard; dropping the `marker_id in valid_marker_ids` clause would still pass every test except the acquire one. |
| R1-8 | `test:368` LOW TEST: _pick_target determinism-across-ticks not pinned | CONFIRMED | `test_two_valid_markers_picks_largest_bbox_deterministically` asserts the picked id ONCE (after acquire); `test_two_valid_markers_equal_area_tie_break_lowest_id` calls `_pick_target` once. Neither feeds the same two-pad frame twice asserting a stable pick across ticks. A `max`→`min` or sign-flipped `-marker_id` mutant: see fresh M-3. |
| R1-9 | `test:428` LOW TEST: unreachable-RuntimeError + FALLBACK non-terminal short-circuit not directly asserted | CONFIRMED | `:428-430` RuntimeError is unreachable by construction (all enum members dispatched). `test_fallback_keeps_landing_until_grounded_then_done` DOES exercise the `:391` re-Land path twice → so the FALLBACK short-circuit IS covered; only the defensive RuntimeError is uncovered. Refines round 1: FALLBACK short-circuit is actually tested. |
| R1-10 | `:332`/`:319` LOW OPT [hot path]: per-tick lambda in _pick_target + per-tick list in _valid_sightings | CONFIRMED (no-action) | `_pick_target` (`:328-332`) builds a fresh `key=lambda` each call, up to 2×/tick (CENTER + DESCEND re-gate). `_valid_sightings` (`:319`) builds a fresh list each tick. Both tiny; see fresh OPT note for the one real micro-win. |
| R1-11 | `:104` INFO: servo math correctly delegated to _servo.pixel_offset_to_move, no duplicated lateral/altitude math | CONFIRMED | land_on_pad imports `pixel_offset_to_move` (`:121`) and calls it identically in CENTER (`:487`) and DESCEND (`:529`); the only local math is `_bbox_area` (`:323-326`, area not servo). No duplicated lateral/altitude/clamp math. DRY satisfied. |
| R1-12 | conventions PASS (no bare except, no top-level numpy, no while, typed errors, units, WHAT/WHICH/WHY/CHECK, frame_w=[1] correct, N-of-M `>=` pinned) | CONFIRMED | No `except` anywhere; imports stdlib enum/math/collections/typing + finals only; zero `while`; `frame_shape[1]`=width per types.py `(h,w)` used at `:486` & `:528`; acquire `>=` at `:440` pinned by `test_acquire_on_3_of_5...`. |

## Landing safety invariants (fresh re-derivation) — ALL HOLD
- Budget gate ABOVE every non-terminal substate: YES. `:397` runs before the `:421-426`
  dispatch into ACQUIRE/CENTER/DESCEND. No substate has a private bypass. (R1-6 is the
  missing DESCEND regression test, not a hole.)
- Fallback ALWAYS Lands: YES. `_go_fallback` (`:337-340`) only sets the sub-state + a
  reason; every caller (`:398/405`, `:449/457`, `:511/517`) returns `Land()` that tick,
  and the `:391` short-circuit re-`Land()`s every subsequent tick until grounded. No
  Fallback path returns Hover/Rotate/Move.
- COMMIT→Done only on is_flying False: YES. The depth-floor commit (`:411-413`) emits
  `Land()`, NOT `Done`; Done is reached solely via `:380` (`is_flying is False`). A blind
  commit can never self-declare success without ground contact.
- No accept-invalid-marker: YES. `_valid_sightings` (`:319-321`) filters by
  `marker_id is not None AND in valid_marker_ids`; ACQUIRE additionally requires a
  current-frame valid sighting (`:440 valid and ...`). CENTER/DESCEND consume the same
  filtered `valid`, so a decoy-only frame is `valid=[]` → treated as lost (correct,
  but untested → R1-7).
- No descend-without-persistent-centering: YES, with the documented soft gap (R1-1). A
  DOWN step at `:540-542` requires (a) `valid` non-empty this tick, (b) the servo returned
  None this tick (in-deadband ⇒ centred), AND (c) `_recent_hits() >= descend_persist_frames`.
  All three must hold. The gap is only that (c) is satisfiable by shared center-hold
  history rather than descend-local confirmation — not a centering bypass.
- No unbounded re-acquire: YES. Re-acquire is bounded two ways: the per-phase
  `acquire_timeout_s` wall (`:448`) and `total_budget_s` (`:397`); the mid-descend
  ascend+re-acquire loop is additionally bounded by `max_loss_retries` (`:510`). No
  unbounded loop construct exists (no `while`).

## FRESH findings
finals/mission/phases/land_on_pad.py:540: MED BUG: confirm-before-descend is materially WEAKER than the docstring on the FIRST descend tick — precise characterization of R1-1. CENTER→DESCEND transition (`:497-500`) calls `_step_descend` in the SAME tick, and `_recent` was already saturated by the `center_persist_frames` in-deadband holds + the acquire hits (one shared deque, never reset on entry). So with the test defaults (center_persist=3, descend_persist=2, window=5) the descend gate `_recent_hits() >= 2` is ALWAYS already true the instant CENTER hands off — the "wait N descend-confirm frames before the first DOWN" the docstring (`:62-64`, `:538`) implies is effectively ZERO additional frames whenever center_persist_frames >= descend_persist_frames (the common config). It is a confirmation count shared with centering, not descend-specific. Not a hover/safety risk (bounded by budget; still requires centred + valid this tick). Fix: either (a) reset a descend-local hit counter on PAD_DESCEND entry and gate on it, or (b) amend the docstring at `:62-64`/`:538` to state the window is the shared center+descend confidence and the first descend step fires immediately once centred if the recent window already holds the pad.
finals/mission/phases/land_on_pad.py:509: LOW BUG: `_loss_retries` is monotonic for the whole phase (incremented at `:509`, reset nowhere). A long good descent that occasionally flickers accumulates lifetime losses toward `max_loss_retries` and Fallbacks on cumulative, not consecutive, losses (re-states R1-3 from the source side). Conservative-safe. Fix: reset `_loss_retries = 0` on a clean centred DOWN step at `:541` (a confirmed-good descend proves recovery).
finals/mission/phases/land_on_pad.py:380: LOW BUG: terminal Done keys on `is_flying is False` only; `is_flying is None` after a commanded Land re-enters the `:391` Fallback re-Land forever and never reaches Done (re-states R1-2). No hover. Fix: none required for the no-hover guarantee; document that phase completion depends on the adapter reporting is_flying False after a Land (the MockAdapter does).
finals/mission/phases/land_on_pad.py:328: LOW OPT [hot path]: `_pick_target` builds a fresh `key=lambda s: (self._bbox_area(s), -s.marker_id)` closure on every call, up to 2×/tick (CENTER `:484` and the DESCEND re-gate `:526`). The single real micro-win: hoist the key to a module-level/staticmethod function (it closes over nothing except `self._bbox_area`, which is itself static) so no closure is allocated per tick. Negligible at ~10 Hz with ≤ a handful of sightings; do NOT change if it costs readability. Priority correctness>clarity>perf — this is bottom-priority. Est. impact ~nil.
finals/mission/phases/land_on_pad.py:319: INFO OPT [hot path]: `_valid_sightings` allocates a fresh list each tick — unavoidable (it is the per-tick filtered result the substates consume) and tiny. No f-string is built before any deadband early-return in step() (the only per-tick f-strings are inside Abort/Fallback/Done branches that all terminate the tick — none on the Move/Hover/None happy path). Hot path is clean of premature formatting. No action.
finals/tests/test_land_on_pad.py:212: MED TEST: kill R1-5 — add a test entering PAD_DESCEND with `_recent` carrying EXACTLY `descend_persist_frames` hits (e.g. descend_persist=2, window=5, seed `_recent` with [True, True] then feed one MORE centred valid → set up so recent_hits lands on the gate value) and assert `Move(DOWN)`; pair it with recent_hits == descend_persist-1 asserting Hover. Kills `>= → >`.
finals/tests/test_land_on_pad.py:341: MED TEST: kill R1-4 — add `test_commit_at_exactly_commit_alt_lands` with `altitude_m == commit_alt_m` (0.5) asserting `Land`. Kills `<= → <` at `:412`.
finals/tests/test_land_on_pad.py:284: MED TEST: kill R1-6 — force `p._sub = _SubState.PAD_DESCEND`, seed a healthy `_recent`, then step with `elapsed_s >= total_budget_s` and assert `Land` + `_sub is FALLBACK` + "total landing budget" in reason. Regression-guards the DESCEND budget bound.
finals/tests/test_land_on_pad.py:103: MED TEST: kill R1-7 (two sub-cases). (a) In PAD_CENTER feed ONLY a decoy id and assert it routes to PAD_ACQUIRE (treated as lost), not centered; (b) feed a decoy + a valid pad in the same frame and assert `_target_marker_id` is the VALID id, never the decoy. Currently dropping the `in self.valid_marker_ids` filter at `:319-321` only fails the acquire test.
finals/tests/test_land_on_pad.py:368: LOW TEST: kill R1-8 / fresh M-3 — feed the SAME two-pad frame twice across consecutive ticks and assert the same `_target_marker_id` both ticks; this pins determinism and kills a `max → min` bbox-area mutant or a flipped `-s.marker_id → +s.marker_id` tie-break sign at `:332` (the equal-area test pins the tie-break value once but not against the bbox-area-direction mutant when areas DIFFER across the wrong direction).
finals/tests/test_land_on_pad.py:422: LOW TEST: the DESCEND→CENTER re-gate is tested for a lateral drift (`test_descend_re_gates_centering_after_a_step`), but the `_loss_retries`-never-reset behavior (R1-3 source) is unguarded; if a future fix adds a reset, add a test that a clean DOWN step at `:541` resets `_loss_retries` to 0 so the intended semantics are pinned. (Only add WITH the fix — today the behavior is monotonic by design.)

## Conventions (mechanical) — PASS (unchanged from round 1)
No bare `except` / `except Exception`; stdlib (enum/math/collections/typing) + finals
imports only, numpy/SDK behind no import; zero `while` (no unbounded-loop class — loops
are bounded `for`); errors typed (ConfigError in __init__, Abort/Done/Land Actions in
step, RuntimeError only on the unreachable defensive branch); units in names
(_cm/_m/_deg/_s/_px/_frames); fail-loud WHAT/WHICH(drone_id)/WHY/CHECK present on Abort
(`:350-357`, `:364-368`), Fallback reasons (`:398-404`, `:449-456`, `:511-516`), and the
ConfigError builder (`:173-176`). `frame_w = frame_shape[1]` is the CORRECT width index
(types.py `frame_shape:(h,w)`); used identically in CENTER (`:486`) and DESCEND (`:528`)
— NO x/y swap into the servo. _servo.pixel_offset_to_move re-validates frame_w/altitude/
bounds and hoists `_BBOX_LABELS` off the hot path (`:53-56`) — its own OPT is already done.

## Surviving mutants (named)
- `:412` `alt_m <= self.commit_alt_m` → `<` : SURVIVES (no equality test). Kill via R1-4 test.
- `:540` `_recent_hits() >= descend_persist_frames` → `>` : SURVIVES (no equality test). Kill via R1-5 test.
- `:319-321` drop `marker_id in self.valid_marker_ids` (accept any id) : SURVIVES the CENTER/DESCEND
  decoy path (only the acquire test fails). Kill via R1-7 test.
- `:332` `max` → `min` (or `-s.marker_id` sign flip) : PARTIALLY survives — the equal-area tie-break
  test pins the value once but a differing-area `max→min` across ticks is not asserted. Kill via R1-8/M-3.
- Killed/equivalent: budget-gate removal (caught by acquire/center budget tests, blanket); acquire
  `>=`→`>` (killed by test_acquire_on_3_of_5); the `:428` RuntimeError is EQUIVALENT/unreachable
  (every enum member dispatched) — not worth a test.

## Bottom line
S-LAND is correct and safe to land. The four cardinal sins (accept-invalid /
descend-without-centered / infinite re-acquire / battery-dead hover) are CLOSED and
re-verified against current source. No NEW CRIT/HIGH. The open items are: one design
soft-spot (descend confirm shared with centering — fix or document, MED), two monotonic/
None LOW edge behaviors (conservative-safe), and five MED/LOW test gaps whose mutants
survive today (commit-equality, descend-equality, DESCEND-budget regression guard,
decoy-in-CENTER/DESCEND, pick-target cross-tick determinism). Review only — no fixes applied.
