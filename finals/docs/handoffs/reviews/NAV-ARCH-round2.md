# NAV-ARCH review — ROUND 2 (re-audit + new gaps)

Re-audit of round-1's fixes + a fresh pass for gaps round 1 missed. Slice
unchanged (visibility_graph.py gate exemption, types.py Gate validation,
navigate.py doc). Suite: 1330 green (no:randomly AND random ordering); 3/3
mutants re-killed after all test churn.

## Re-audit of round-1 findings
- R1 HIGH (gate node removed): CONFIRMED resolved. No gate-specific nodes remain; `git grep approach_/midpoint` in visibility_graph.py is clean. The route through every gate test is carried by start/goal/corner edges via the exemption. test_gates_do_not_change_a_gate_free_arena + the full pre-existing planner/nav/e2e suites prove the no-gate path is unchanged.
- R1 LOW (whole-post excuse / convex caveat): UNCHANGED, documented. Re-verified the convex argument: a straight edge meets a convex polygon in one contiguous sub-segment, so "crosses the doorway span" and "enters the post" are the same entry — no distant double-clip. Concave keep-outs remain out of scope (inflate_polygon's existing caveat).
- R1 LOW (lazy import in types validator): UNCHANGED. Re-verified: `import finals.mission.planning.types` does not pull polygon_tools at module load; the validator import is config-time only; no cycle.

## NEW findings this round
finals/tests/test_visibility_graph_gates.py (round-1 draft): MED TEST (FIXED this round): the first two-arch / off-axis tests asserted `not _hits(block)` on a THROUGH-GATE path. For the SOLID-BLOCK arch model the legal route runs through the block FOOTPRINT via its doorway, so `segment_enters_polygon(real_block)` is correctly True — the assertion was wrong (it encodes the two-POST model, not the solid-block one). Fix: through-gate solid-block tests assert `_crosses_span` (+ goal reached + single heading), NOT block-avoidance; only the DETOUR (no/!fit gate) tests assert `not _hits(block)`. The collision-free CLAIM for a gated arch is "passes through the declared opening", not "avoids the solid footprint".
finals/tests/test_visibility_graph_gates.py (round-2 draft): MED TEST (REMOVED this round): a "one-of-two-arches-too-narrow forces a detour at THAT arch" test was fragile — when one arch is impassable the A* optimum routes AROUND BOTH (cheaper than threading the near doorway then weaving out), so the "near gate used" assertion is geometry-tie-break-dependent, not a real contract. The "full-wall sealed corridor" variant also failed because visibility_graph does NOT clamp to bounds, so detours slip past inflated wall ENDS. The genuine claims it targeted are already covered without fragility: per-gate fit by the single-arch too-narrow + walled-in pair (enclosed pocket: narrow gate -> PlanningError, fitting gate -> succeeds), and composition by test_two_arches_in_series_both_threaded. Removed rather than over-fit to A* tie-breaking.
finals/mission/planning/visibility_graph.py: LOW (noted, no change): the planner is bounds-AGNOSTIC (it never reads arena.bounds_m); a sealed-corridor test cannot rely on walls "reaching the cage edge". This is the EXISTING NAV-1 contract (bounds is a geofence concern owned by guards.py/NAV-8, not the planner) — correct, just a test-design constraint. The "walled-in" tests therefore use a fully ENCLOSED pocket (front+back+left+right), which is the only robust way to force "no way around".
finals/tests/test_nav_e2e.py: ADDED (gap round 1 missed): the FULL [takeoff, navigate, land_on_pad] mission through an arch gate was untested end-to-end via the REAL load_config path — so the from_dict gate VALIDATION + the planner gate EXEMPTION were each unit-tested but never proven to compose on the mission path. test_full_mission_through_an_arch_gate_lands_on_the_pad loads an arena JSON WITH a gate (load_config -> from_dict -> _validate_gate_geometry), builds phases the real way (_build_phases), and chains to a landing over the MockAdapter+DeadReckoner. Closes the seam.

## Mutation kill-check (re-run after all test churn)
- (a) gate-as-free removed -> test_through_gate_is_a_straight_shot_for_a_centred_doorway + test_integration_navigate_flies_through_an_arch_gate + test_full_mission_through_an_arch_gate_lands_on_the_pad all RED. KILLED (3 tests).
- (b) degenerate-span check dropped -> test_from_dict_degenerate_span_is_loud RED (DID NOT RAISE). KILLED.
- (c) clearance fit ignored -> test_too_narrow_gate_does_not_fit_and_still_detours + test_unspecified_clearance_zero_never_fits + test_walled_in_goal_with_no_fitting_gate_still_raises all RED. KILLED (3 tests).
3/3 mutants killed; both source files restored byte-clean (no MUTANT/`if False` markers).

## Verdict
Clean. The gate edge-exemption is the single, robust mechanism (no fragile gate nodes); validation is loud on every malformed shape (degenerate span / outside bounds / not-in-a-keep-out-gap / bad clearance); the no-gate path is byte-for-byte unchanged; the altitude rule is documented (navigate.py + field_markers.md + module_map.md) and is doc-only (no behavioural surface to regress). New tests: 18 (test_visibility_graph_gates.py) + 1 (test_navigate e2e) + 1 (test_nav_e2e full-mission). Suite 1310 -> 1330. 3/3 mutants killed.
