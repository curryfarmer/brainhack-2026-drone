# NAV-ARCH review — ROUND 1

Slice: planning/visibility_graph.py (gate edge exemption + `_fitting_gates` +
`_segments_properly_cross`), planning/types.py (`Gate.from_dict` degenerate-span
guard + `ArenaMap.from_dict` `_validate_gate_geometry`), phases/navigate.py
(gate-altitude docstring), field_markers.md + module_map.md.
Tests: test_visibility_graph_gates.py (NEW, 17), test_navigate.py
(+1 e2e arch), test_step0_contracts.py (2 gate fixtures updated).
Baseline: 1310 green pre-change; 1328 green post (bare venv, no cv2/numpy).

One finding per line: `path:line: SEV tag: problem. fix.`

## BUG / CONV
finals/mission/planning/visibility_graph.py:~184: HIGH BUG (FIXED this round): the first draft added each fitting gate's on-span MIDPOINT (later an offset approach-node pair) as a graph node "so the corner-hugging detour has a waypoint in the slot". The midpoint is COLLINEAR with the span, so no edge to it can ever `_segments_properly_cross` the span (=> never excused => a dead node); it is also routinely BURIED inside an inflated post (verified: a clearance-fitting doorway midpoint sits inside the front block). The offset-pair replacement was likewise buried for any post deeper than the offset and, when not buried, was NOT load-bearing (the dog-leg test routes via inflated CORNERS with the gate node removed — proven by probe). Fix applied: REMOVED all gate-specific nodes; the route THROUGH a gate is carried by the ordinary start/goal/corner edges that the `visible()` exemption now lets cross the opening. Docstring + module_map updated to match. (Simpler, honest, all 17 gate tests + dog-leg still green.)
finals/mission/planning/visibility_graph.py:285: LOW BUG: an edge that properly-crosses a fitting gate's span is excused from the ENTIRE post in `gi.post_idx`, not just the local crossing — so an edge that both crosses the doorway AND clips a DISTANT part of the same post would be wrongly allowed. UNREACHABLE for the convex posts/blocks actually used (a straight segment meets a convex polygon in one contiguous piece, so "crosses the doorway" and "the clip" are the same entry). Same convexity caveat already governs inflate_polygon (S-PLAN round1). Fix: none for convex inputs; if concave keep-outs are ever admitted, intersect the excuse with a locality test (crossing point within clearance_m of the post boundary). Documented as a convexity assumption.
finals/mission/planning/types.py:418: LOW CONV: `_validate_gate_geometry` lazy-imports polygon_tools inside the function (keeps the contracts module stdlib at import time, mirrors the "planners import numpy lazily" convention). The import runs once per `ArenaMap.from_dict` call when gates exist (cold, config-time). Verified no cycle (polygon_tools is leaf stdlib; types.py imports it nowhere else). No action — deliberate.
finals/mission/phases/navigate.py: CONV (clean): the gate-altitude rule is DOC ONLY — navigate issues no vertical Move (confirmed: step() only emits Rotate / Move(FORWARD); the e2e test asserts no Takeoff/Land). The ~1.1 m-ceiling / no-bands reconciliation is documentation, not behaviour, so it cannot regress a test. Correct scope.

## TEST (mutants the suite WOULD / would NOT catch)
finals/mission/planning/visibility_graph.py:285: KILLED — gate-as-free removed (`if False and any(...)`): test_through_gate_is_a_straight_shot_for_a_centred_doorway + test_integration_navigate_flies_through_an_arch_gate both go red (the doorway is no longer crossed => detour). Mutant (a).
finals/mission/planning/types.py:~228: KILLED — degenerate-span check dropped (`if False and endpoints[0]==endpoints[1]`): test_from_dict_degenerate_span_is_loud goes red (DID NOT RAISE). Mutant (b).
finals/mission/planning/visibility_graph.py:246: KILLED — clearance fit ignored (`if False and (clearance<=0 or <min_clear)`): test_too_narrow_gate_does_not_fit_and_still_detours + test_walled_in_goal_with_no_fitting_gate_still_raises both go red (a too-narrow gate would wrongly thread / open the pocket). Mutant (c).
finals/mission/planning/visibility_graph.py:79: MED TEST: `_segments_properly_cross` strictness (the `!= 0` guards that make a collinear/endpoint touch NOT a cross) is implicitly pinned — a mutant relaxing it to `_segments_intersect` semantics would let an edge that merely TOUCHES a gate span endpoint be "excused", but the centred-doorway + off-axis tests still cross PROPERLY so they would pass. Partially covered by test_gate_does_not_excuse_an_unrelated_obstacle (an unrelated obstacle's straight edge does not get a false excuse). Acceptable: the touch-vs-cross distinction is conservative (a relaxation only ever ADMITS more edges through a real gate, never opens an unrelated post — `post_idx` gates that). Noted, no new test required.
finals/mission/planning/types.py:~430: LOW TEST: the span-within-bounds branch of `_validate_gate_geometry` is pinned by test_from_dict_span_outside_bounds_is_loud; the touch-a-keep-out branch by test_from_dict_span_in_no_keepout_gap_is_loud. Both raise distinct messages (matched by `OUTSIDE bounds` / `does not touch ANY keep-out`). Covered.

## OPT (cold path — plan() runs once per phase; gates are <= a handful)
finals/mission/planning/visibility_graph.py:276: LOW OPT: COLD path. `visible(i,j)` recomputes `crossed` (the properly-crossed fitting gates) for every A* edge test; with `fitting` empty this is skipped entirely (the `if fitting else ()` guard), so the no-gate path is byte-for-byte unchanged. With gates, it is O(gates) per edge over a <=~20-node graph — negligible. No action.
finals/mission/planning/types.py:~437: LOW OPT: COLD path. `_validate_gate_geometry` is O(gates * keep_outs) segment-intersection tests at config load. Tiny (handfuls). No action.

## Mechanical convention scan (clean)
- No bare `except` / `except Exception` added (grep clean in both files).
- No top-level numpy/SDK import: visibility_graph stays heapq+math+finals.*; types.py stays stdlib at import time (polygon_tools lazy-imported in the validator only) — convention 8 satisfied, conventions test green (7/7).
- The no-gate behaviour is PROVEN unchanged: test_gates_do_not_change_a_gate_free_arena (identical legs with gates=()) + the full pre-existing test_visibility_graph.py (28) + test_navigate.py + test_nav_e2e.py all green.
- ConfigError raise sites in the gate validators carry WHAT/WHICH(gate id + span)/WHY/CHECK; the fit-drop paths in `_fitting_gates` are SILENT-by-design (a non-fitting gate is not an error, it is just "route around") and documented as fail-CLOSED.
- 3/3 named mutants killed; reverted clean (git diff shows no MUTANT markers).
