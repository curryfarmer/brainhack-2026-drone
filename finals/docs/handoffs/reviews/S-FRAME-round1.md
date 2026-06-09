# S-FRAME — ROUND 1 adversarial review

Slice: planning/frame.py + config.py arena parse/validate (+ types.py ArenaMap.from_dict shapes,
sample.json, mock_arena.json). Reviewer lens: frame-convention correctness (NED axes / c2 origin
+heading / CCW+), fail-loud on every malformed arena. Tests: test_frame.py, test_arena_config.py.

Bottom line: frame.py rotation math VERIFIED correct against dead_reckon.py (FORWARD=(cos,-sin),
RIGHT=(sin,cos); CCW+; 0=+north) and the cross-term/sin-sign/transpose mutants ARE killed by
test_heading_90/45. No CRIT. The findings are test gaps + fail-loud/convention nits + cold-path OPT.

## BUG / CONV

finals/config.py:662: MED BUG: `_resolve_arena` opens the resolved arena file with no try/except around the `open()` itself — a file that `os.path.isfile` saw but cannot be opened (permission / race / it is a directory named `<name>.json`) raises a raw OSError, not the loader's ConfigError contract (the JSONDecodeError branch IS wrapped, the open is not). Wrap the `open()` in `except OSError as e: raise ConfigError(f"{resolved}: cannot read arena map — {e}") from e`.
finals/mission/planning/types.py:228: MED BUG: lanes are validated for finite POINTS but a lane polyline is never required to be non-degenerate — a lane of 0 or 1 points (`"lanes": [[]]` or `[[[1,2]]]`) is silently accepted as an empty/one-point polyline. ADVISORY-only so low blast radius, but it is a silent-accept of a malformed field; either reject `< 2` points with an actionable ConfigError or document the no-op as intentional. (No WHICH/WHY message exists for this case today.)
finals/mission/planning/types.py:222-227: LOW BUG: keep-out polygons and lanes are NOT bounds-checked (only pad centers + c2_origin are). A keep-out or lane wholly OUTSIDE bounds_m is silently accepted; NAV-1 would inflate/route around a phantom obstacle off the arena. Likely intentional (keep-outs may legitimately straddle the wall), but it is undocumented asymmetry vs the pad/origin CLOSED-bounds rule — add a one-line comment stating keep-outs are deliberately not bounds-clamped, so a future reader does not read it as a missing check.
finals/mission/planning/frame.py:120: LOW CONV: `bearing_from_c2_deg` uses exact float `== 0.0` to detect the at-C2 degenerate (no defined bearing). Matched by `in_sector`'s identical exact-equality origin guard, so the two are consistent and a point 1e-12 off origin gets a real bearing in BOTH — acceptable, but the exact-equality pair is load-bearing and undocumented at the bearing site. Note it (the in_sector site documents it; this one does not).

## TEST (named mutant + killing test)

finals/mission/planning/frame.py:106-107: HIGH TEST: no test pins discord_to_ned at a heading that is NOT a multiple of 45 AND has BOTH forward and right non-zero. Mutant `d_east = -sin_t*fwd - cos_t*right` (negate the +cos*right cross term) SURVIVES — at the 45-deg test right=0, and the 90-deg combined test (2,3) has cos90=0 so the cos*right term contributes 0 to d_east; nothing exercises cos*right on d_east at a nonzero-cos heading. Add `discord_to_ned((2,3),(0,0),30)` pinned to hand-computed (dN=cos30*2+sin30*3, dE=-sin30*2+cos30*3).
finals/mission/planning/frame.py:106: MED TEST: mutant `d_north = cos_t*fwd - sin_t*right` (negate the +sin*right cross term on d_north) — the only test with right!=0 at a nonzero-sin heading is (0,1)@90 and (2,3)@90, both with cos90=0; at 90 sin*right=1*right so it IS caught there, BUT a sign-only mutant that flips just the d_north cross term at a GENERIC heading is not independently confirmed. The heading-30 both-nonzero test above also kills this.
finals/mission/planning/frame.py:122: MED TEST: bearing_from_c2_deg is only tested at the 4 cardinals + origin; no diagonal (e.g. NE) pins the atan2 arg ORDER/SIGN beyond the axes. Mutant `atan2(d_east, d_north)` (drop the negation) gives east->+90 not -90 and IS killed by the cardinal due-east test — OK. But `atan2(-d_north, d_east)` (transpose args) maps north->+0? Add `bearing_from_c2_deg((1,1),(0,0)) == -45.0` (NE = bearing -45, CCW+) to pin the arg pairing on a diagonal.
finals/mission/planning/frame.py:170: MED TEST: in_sector deadband is pinned closed only at half_width=90 with a point EXACTLY at bearing -90 (test_sector_boundary_is_inclusive). The `<=`->`<` mutant is killed there, good — but the WRAPPED boundary (delta computed through ±180) is never pinned at EXACTLY the half-width. Add an exact-edge case across the wrap: centre 170, half_width 30, a point at bearing exactly -160 (delta exactly 30) asserted True, and bearing -159.999 asserted False.
finals/config.py:660-662: HIGH TEST: no test surfaces a SEMANTICALLY-malformed arena THROUGH load_config (only the missing-file path is tested at the config layer; the semantic rules are tested only directly on ArenaMap.from_dict). A regression that makes `_resolve_arena` swallow or mis-wrap the ArenaMap ConfigError would pass the whole suite. Add a load_config test with a tmp arena JSON that trips one semantic rule (e.g. pad out of bounds) and assert the ConfigError propagates with the field-naming message.
finals/config.py:660-661: MED TEST: `_resolve_arena`'s invalid-JSON branch (`json.JSONDecodeError -> ConfigError "{resolved}: invalid JSON"`) is untested. Add a tmp arena file containing `{bad json` referenced by arena_name and assert ConfigError matches "invalid JSON".
finals/config.py:646-651: MED TEST: the candidate-path PRECEDENCE in `_resolve_arena` (config-dir/arenas, then config-dir, then repo-root finals/configs/arenas) is untested — only the repo-root happy path runs (via mock_arena.json). A reorder that breaks "a profile + its arena travel together" (config-dir first) would not be caught. Add a tmp_path config whose sibling `arenas/<name>.json` differs from the repo sample and assert the SIBLING is loaded.
finals/mission/planning/types.py:234-239: LOW TEST: c2_heading_deg WRONG-TYPE (a string) is untested; only NaN is. Mutant dropping the `isinstance(...,(int,float))` guard would let `"90"` through to `float(heading)` and pass for numeric strings but TypeError on others. Add `_mut(c2_heading_deg="90")` -> ConfigError match "c2_heading_deg".
finals/mission/planning/types.py:108-111: LOW TEST: KeepOut.polygon_m NON-LIST (e.g. `"polygon_m": 5`) raising the dedicated "must be a list of [north_m, east_m] points" message is untested (only too-few-vertices is). Add a non-list polygon_m case.
finals/mission/planning/frame.py:95: LOW TEST: discord_to_ned with a non-finite ORIGIN (NaN/Inf in c2_origin_m) is untested — test_discord_to_ned_bad_origin_raises uses only a length-1 origin. Add `discord_to_ned((1,1),(float('nan'),0.0),0.0)` -> ConfigError match "c2_origin_m" to pin the origin finite-check, not just its arity.

## OPT (cold path — all SCALING notes, none urgent; arena ≤ ~20 vertices, loaded once)

finals/mission/planning/types.py:121-124: LOW OPT: distinct-vertex de-dup is O(n²) (`if p not in distinct` over a list). Cold path, n≤~8 per polygon → negligible. A set would be O(n) but tuples-of-floats hash fine; REJECT as a change because the current form preserves first-seen ORDER for the actionable error message and order does not matter for the set. Note only; do not change.
finals/config.py:646-651: LOW OPT: `_resolve_arena` builds 3 candidate paths and stats each; runs once at load. No action. (Listed only to confirm the cold-path classification — re-statting on every step() would be a bug, but it is not in any step path.)
finals/mission/planning/frame.py:103-104: LOW OPT: cos/sin recomputed per discord_to_ned call; called once per landing coordinate (cold). No action.

## CONVENTIONS (mechanical)

- No bare `except` in frame.py or the config arena path; the two config `except json.JSONDecodeError` are typed (load_config + _resolve_arena). PASS.
- frame.py imports: stdlib `math` + typing + finals.errors only — NO top-level numpy / SDK. PASS (pure-module rule honored).
- No `while` loops in frame.py or the arena path. PASS (no unbounded-loop risk).
- No module-level mutable globals introduced. PASS.
- Units in names (_m/_deg) honored throughout. PASS.
- Every malformed-arena path raises a TYPED ConfigError naming the field; messages carry WHAT/WHICH/WHY/CHECK for bounds/pad/origin/duplicate-id. The lane-degenerate and arena-file-unreadable cases (above) are the two paths that can currently escape that contract.
