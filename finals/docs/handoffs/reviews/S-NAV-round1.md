# S-NAV round 1 — adversarial review

Slice: phases/navigate.py + phases/takeoff.py (tests: test_navigate.py, test_takeoff_phase.py).
Scope: open-loop transit state machine; absolute-heading re-zero; budget/stuck bounds; OPT (step() hot path); conventions.

Context established (calibrates severity): under the real S4 DroneAgent (agent.py:444-451) a failed
flight command calls `_fail()` directly and NEVER re-steps the phase, so `step()` is never re-entered with
`last_action_ok is False`. The navigate/takeoff `last_action_ok is False` Abort branches are therefore the
documented belt-and-suspenders path for a future report-not-fail agent; they are exercised only by hand-built-ctx
tests. Real and correct, but not on the live failure path — keeps the off-by-one-naming findings below MED, not CRIT.

## Findings

finals/mission/phases/navigate.py:298: MED BUG: rotate-cap Abort reports `residual = leg.heading_deg - yaw` UNWRAPPED, but convergence is judged on `wrap180(heading_deg - yaw)` (via bearing_error_to_rotate/_servo). For e.g. heading 180, yaw -170 the message prints "residual ~350 deg > tol" while the true error is -10 deg — a fail-loud WHY that lies and points the operator at the wrong knob. Fix: compute the reported residual with `_servo.wrap180(leg.heading_deg - yaw)` (the same quantity the deadband tests).
finals/mission/phases/navigate.py:322: MED OPT/BUG-risk: `return self.step(ctx)` recurses to skip a 0 cm-rounded leg. Hot path (step() @ tick_hz) but bounded by the finite leg count, so not unbounded; however an all-zero / pathological plan recurses once per leg in ONE tick and re-runs the budget + last_action_ok guards each frame. Acceptable for ≤~20 legs; convert to a `while dist_cm <= 0 and self._leg_idx < len(...)` loop (still bounded) to drop the per-skip stack frame + guard re-eval and make the bound explicit per convention 3. REJECT if the loop would drop the budget/last_action_ok re-checks — keep them above the loop.
finals/mission/phases/navigate.py:124: LOW OPT: `_rot_cap`, `goal_desc`, `_legs` all correctly computed in `__init__`/from_config (cold path) — no per-tick planning/allocation in step(). step() builds no lists/tuples and calls one wrap180/clamp via _servo; the only per-tick allocations are the returned Action dataclasses and Abort f-strings on the error paths (off the happy path). No hot-path opt warranted. [OPT clean]
finals/mission/phases/navigate.py:240: LOW CONV: the `last_action_ok is False` Abort message uses `{ctx.last_action!r}` and includes WHAT/WHICH(drone+leg)/WHY(error)/CHECK — compliant. (No fix; recorded as verified, not a nit.)
finals/mission/phases/navigate.py:297: LOW TEST: the rotate-cap `>` vs `>=` boundary is not pinned. test_non_converging_rotate_aborts_after_bound only asserts `i <= rot_cap+1`, which passes for BOTH `>` (abort on the rot_cap+1-th Rotate) and `>=` (abort on the rot_cap-th). A `>`→`>=` mutant survives. Both are bounded+safe so severity is LOW, but add an assert on the EXACT Rotate count before the Abort (e.g. count Rotates == rot_cap then the next step Aborts).
finals/mission/phases/navigate.py:254: LOW TEST: budget-vs-await_move ordering is unpinned. The budget guard (line 254) sits ABOVE the await_move leg-advance (line 266), so an over-budget final leg Aborts instead of sneaking a Done through. A mutant that moves the await_move advance + the `_leg_idx >= len -> Done` block ABOVE the budget check would emit Done past budget and is NOT caught (test_budget_exceeded_aborts trips budget on a non-final, non-await_move state). Add a test: drive to the final leg's await_move, then step with elapsed > budget and last_action_ok True, assert Abort (not Done).
finals/mission/phases/navigate.py:266: LOW TEST: the await_move→advance happens before the yaw-None guard, so yaw going None on the leg AFTER a successful Move correctly Aborts naming the NEW (unflyable) leg. No test feeds yaw=None mid-plan (after ≥1 successful Move) — test_yaw_none_aborts only hits leg 1. A mutant moving the yaw-None check above the await_move advance would misname the leg; unkilled. Add a mid-plan yaw-None case asserting the leg index in the message.
finals/mission/phases/navigate.py:218: LOW TEST: from_config's empty-plan no-op trap (`if not legs`) is tested only via goal==C2 (test_from_config_degenerate_plan_is_noop_trap). A `not legs`→`legs is None` style weakening (plan never returns None) is unkillable; minor. The `dist_cm<=0`/ZERO-legs distinction is otherwise well covered (test_only_leg_rounds_to_zero_completes_without_moving).
finals/mission/phases/takeoff.py:74: LOW BUG-none: plan is `[Takeoff]` with NO trailing Land — the whole point of the phase; pinned by test_takeoff_then_done_no_land (asserts no Land + HOLDING). step() advances _idx past the Takeoff then Done. No off-by-one (the Takeoff is returned once: _idx 0→1, then _idx>=len→Done). Verified correct.
finals/mission/phases/takeoff.py:88: LOW TEST: takeoff.step() has NO budget/deadline of its own — acceptable (the plan is a single Takeoff; the agent's outer wait_for + safety launch_slot bound it). But there is no test that a SECOND step after Done re-emits Done (Done stability), unlike navigate's test_done_is_stable_after_completion. A mutant that resets `_idx` or re-returns the action would survive. Add a Done-stability assert (step twice past completion).
finals/mission/phases/takeoff.py:64: LOW CONV: `height_cm` validated via search._check_height_cm (int>0, bool/float rejected) on the GROUND in __init__ — a bad height dies at wiring time, not as a mid-air adapter refusal. Compliant with the no-op-trap convention. The `_height_from_band` default (band*100 → else 80) is a documented default, not a silent one hiding a typo (typo keys raise via _zone_kwargs). Verified.
finals/mission/phases/navigate.py:43: LOW CONV: imports are stdlib `math` + finals-internal only; no top-level numpy, no SDK, no bare except, no module-level mutable globals; the only loop-like construct is the bounded recursion at :322 (finite leg count) and the per-leg rotate cap at :297. All four mechanical conventions pass for both files.

## Summary of severity counts
CRIT 0 / HIGH 0 / MED 2 / LOW 10.

No CRIT/HIGH. The state machine is correct on the audited points: leg index advances ONLY after a Move resolves
OK (no early-advance off-by-one — test_failed_move_abort_names_the_current_leg_not_the_next pins it); the per-leg
rotate cap (`ceil(360/max_step)+4`) + the global budget bound both exist (no unbounded spin / no battery-dead hover —
transit Aborts, the agent then safes down via final Land/emergency_land); the re-orient is ABSOLUTE
(wrap180(heading-yaw), re-zeroing creep — test_reorient_re_zeros_drifted_yaw pins it, a relative scheme fails it);
Done is emitted only after the final Move resolves; PositionQuality.NONE respected (yaw-only control, DR is advisory).
The two MEDs are a misleading (unwrapped) residual in a fail-loud message and a per-tick recursion that should be a
bounded loop — neither changes flight behavior.
