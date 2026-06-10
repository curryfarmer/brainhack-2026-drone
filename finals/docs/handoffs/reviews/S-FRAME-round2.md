# S-FRAME — ROUND 2 adversarial review (re-audit of round 1 + new findings)

Slice: planning/frame.py + config.py _resolve_arena + types.py ArenaMap.from_dict (+ sample.json,
mock_arena.json). Method: every round-1 mutant claim was RUN in-process against the actual test
assertions (not merely reasoned). Baseline `pytest test_frame test_arena_config` = 44 green.

Bottom line: round 1's TWO HIGH claims split — the discord_to_ned non-45 mutant claim is REFUTED (every
plausible discord_to_ned mutant IS killed by the existing heading-0/90/45/180/-90 grid), the
malformed-arena-through-load_config claim is CONFIRMED (real test gap). NEW: a non-finite `radius_m`
escapes the pad guard (BUG), the in_sector inner-`_wrap180` is genuinely unpinned (a no-wrap mutant
SURVIVES), and pad/keep-out `id` is silently `str()`-coerced.

## VERDICT — round-1 findings

VERDICT config.py:662 (open() OSError unwrapped) MED BUG: CONFIRMED. The `open(resolved,...)` at 657 is
OUTSIDE the try; only `json.load` is wrapped. A stat-passed-but-open-fails path (perms / race / a dir
that isfile rejected via a symlink race) raises raw OSError, breaking the ConfigError contract. Wrap the
open in `except OSError`.
VERDICT types.py:228 (degenerate lane <2 pts silently accepted) MED BUG: CONFIRMED by run — `lanes=[[]]`
and `[[[1,2]]]` both load without raising. Advisory, low blast radius, but a silent-accept of a
malformed field with no WHICH/WHY message.
VERDICT types.py:222-227 (keep-out / lane NOT bounds-checked) LOW BUG: CONFIRMED by run — a keep-out and
a lane wholly outside bounds_m both load. Undocumented asymmetry vs the pad/origin CLOSED-bounds rule.
VERDICT frame.py:120 (bearing exact `==0.0` at-C2 guard undocumented) LOW CONV: CONFIRMED (cosmetic;
consistent with in_sector's guard).
VERDICT frame.py:106-107 (non-45 d_east cos*right mutant SURVIVES) HIGH TEST: REFUTED-because the mutant
`d_east=-sin*fwd-cos*right` is KILLED by test_heading_zero_is_offset_only — at heading 0, cos0=1 so the
right-only cases `(0,4)@0->(0,4)` and `(3,2)@0` fully exercise cos*right (got (0,-4) vs exp (0,4)). Round
1 only weighed the 45deg (right=0) and 90deg (cos90=0) cases and overlooked heading-0. I enumerated all
coefficient-swap / sign / drop mutants of both d_north and d_east: ALL are killed by the existing grid.
No discord_to_ned test gap exists. The proposed heading-30 test is harmless but redundant.
VERDICT frame.py:106 (d_north sin*right sign mutant) MED TEST: REFUTED-because killed by the same grid
(heading-90 `(0,1)->(1,0)` and `(2,3)@90->(4,-1)` pin sin*right on d_north). DUPLICATE of the above.
VERDICT frame.py:122 (bearing diagonal NE test to pin atan2 arg order) MED TEST: REFUTED-because the 4
cardinal asserts ALREADY uniquely pin the atan2 args: of all 16 `atan2(±dn|±de, ±dn|±de)` permutations,
ONLY the correct `atan2(-de,+dn)` survives the cardinals. Worse, round 1's proposed `bearing((1,1)) ==
-45` would NOT discriminate — the transpose mutant `atan2(-dn,de)` ALSO yields -45 at NE, so the proposed
test is non-discriminating AND unnecessary (the mutant it targets is already dead at the cardinals).
VERDICT frame.py:170 (wrapped-boundary delta not pinned at exact half-width) MED TEST: CONFIRMED, and
STRONGER than round 1 stated — not just the `<=`->`<` edge: dropping the INNER `_wrap180` entirely
(`delta=abs(bearing-center)`) SURVIVES the whole suite. test_sector_wraps_across_180 uses center=170 /
bearing=180 (only 10deg off) which needs NO wrap arithmetic, so the wrap is untested. Kill test:
`in_sector(pt@bearing+175, o, center=-170, half_width=30)` must be True (correct=15deg in; no-wrap
mutant=345deg out). (Round 1's center=170/bearing=-160 exact-edge test also kills it; either works.) The
`<=`->`<` operator mutant alone IS killed by test_sector_boundary_is_inclusive.
VERDICT config.py:660-662 (semantic-malformed arena THROUGH load_config untested) HIGH TEST: CONFIRMED.
The two load_config tests are valid-mock + missing-FILE only; no semantic fault is driven through the
config layer. A regression that swallows or re-wraps ArenaMap.from_dict's ConfigError inside
_resolve_arena would pass the entire suite. (Current code DOES propagate correctly — verified — so this
is a pure test gap guarding a real seam.) Add a tmp arena tripping one semantic rule via load_config.
VERDICT config.py:660-661 (invalid-JSON branch untested) MED TEST: CONFIRMED — no test feeds malformed
JSON; grep shows no "invalid JSON" assertion.
VERDICT config.py:646-651 (candidate-path precedence untested) MED TEST: CONFIRMED — only the repo-root
happy path runs; no sibling-vs-repo precedence test exists.
VERDICT types.py:234-239 (c2_heading wrong-TYPE string untested) LOW TEST: CONFIRMED — guard rejects
"90" (verified) but only NaN is tested.
VERDICT types.py:108-111 (polygon_m non-list untested) LOW TEST: CONFIRMED — guard raises on `polygon_m:
5` (verified) but only too-few-vertices is tested.
VERDICT frame.py:95 (non-finite ORIGIN untested) LOW TEST: CONFIRMED — only a length-1 origin is tested;
a NaN/Inf origin component is not.
VERDICT types.py:121-124 (O(n^2) distinct-vertex dedup) LOW OPT: CONFIRMED + REJECT-the-change agreed
(cold path, n<=~8, set would lose first-seen order for the message).
VERDICT config.py:646-651 (3 candidate stats) LOW OPT: CONFIRMED cold-path, no action.
VERDICT frame.py:103-104 (cos/sin per call) LOW OPT: CONFIRMED cold-path, no action.

## NEW findings (round 1 missed)

finals/mission/planning/types.py:152-156: MED BUG: pad `radius_m` guard is `not radius > 0` with NO
math.isfinite check, so `radius_m: Infinity` is ACCEPTED (inf>0 is True) — verified: arena loads with a
non-finite radius. Every OTHER numeric guard in this file (bounds, _point, c2_heading) checks isfinite; a
pad with radius=inf gives NAV-6 an infinite acceptance circle (any sighting "in the hoop"). Add
`or not math.isfinite(radius)` to the guard. (nan is incidentally caught because nan>0 is False; inf is
the leak.)
finals/mission/planning/types.py:131,162: LOW BUG: pad/keep-out `id` is silently coerced via `str()` —
`id: null` -> "None", `id: [1,2]` -> "[1, 2]", `id: 1` -> "1" all load (verified). The contract is a
string id; a non-string id should fail loud naming the field, not stringify. Collision hazard: int `1`
and string `"1"` both become "1". Reject non-str id with a ConfigError, or document the coercion.
finals/mission/planning/types.py:228: LOW BUG: lane points are finite-checked (NaN lane point raises,
verified) but a lane polyline is otherwise unbounded in degeneracy — same class as round1's lane-<2
finding; noting the finite-check DOES fire so only length/bounds escape, not non-finite.
finals/config.py:662: LOW CONV: `_resolve_arena` strips `.json` for the `filename` candidate but passes
the ORIGINAL `name` (possibly "<x>.json") to `ArenaMap.from_dict(raw, name=name)`, so an arena_name with
the extension yields error messages reading `arena 'x.json'`. Pass the stripped basename for clean
WHICH-arena messages.
finals/tests/test_frame.py:122-127: MED TEST: the inner-`_wrap180` kill test is MISSING (see frame.py:170
VERDICT) — add a sector point on the far side of the +-180 seam from center (center=-170, half_width=30,
a point at bearing +175 -> True; +145 -> False) to pin the wrap arithmetic, not just the comparison.
finals/tests/test_arena_config.py: MED TEST: no test pins `radius_m: Infinity` (the new BUG above) — add
a `_mut` pad with radius_m=float('inf') asserting ConfigError match "radius_m".
finals/mission/planning/frame.py:161: LOW TEST: the `>= 180.0` full-circle short-circuit is unpinned, but
the `>=`->`>` mutant is FUNCTIONALLY EQUIVALENT (max wrapped delta is 180, so `delta <= half_width` covers
hw>=180 anyway) — verified survives but harmless. No test needed; note only so a future reader does not
"fix" it.

## OPT (cold path — frame.py + arena parse run ONCE at load; arena <=~20 vertices)

finals/mission/planning/frame.py:103-104: LOW OPT: cos/sin per discord_to_ned call — called once per
landing coordinate. SCALING note only, no action (matches round 1).
finals/mission/planning/types.py:121-124: LOW OPT: O(n^2) distinct-vertex dedup — REJECT the set rewrite
(loses first-seen ORDER needed for the actionable message; n<=~8). Confirmed round 1.
No new hot-path concern: nothing in this slice runs inside step(); all parse/trig is from_config/cold.

## CONVENTIONS (mechanical re-check)

- frame.py imports: stdlib math + typing + finals.errors only; NO top-level numpy/SDK. PASS.
- No bare except in frame.py or the config arena path; both config json branches are typed
  JSONDecodeError. The unwrapped open() at config.py:657 is the one OSError gap (above). PARTIAL.
- No while loops in frame.py or the arena path. PASS.
- No module-level mutable globals introduced. PASS.
- Units in names (_m/_deg/_cm) honored. PASS.
- Every malformed-arena path raises a TYPED ConfigError naming the field EXCEPT: the unreadable-file
  open() (config:657), degenerate/out-of-bounds lanes, out-of-bounds keep-outs, a non-finite radius
  (NEW), and a non-string id silently stringified (NEW). Those are the escapes from the fail-loud bar.
