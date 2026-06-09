# S-NAV round 2 — adversarial review

Slice: phases/navigate.py + phases/takeoff.py (tests: test_navigate.py, test_takeoff_phase.py).
Scope: open-loop transit state machine; absolute-heading re-zero at PositionQuality.NONE; budget/rotate-cap
bounds; OPT (step() hot path); conventions; mutation-killability of the round-1 MED/LOW findings.

Round-1 context re-checked and CONFIRMED against current source: under S4 DroneAgent a failed flight command
calls `_fail()` and never re-steps the phase, so the `last_action_ok is False` Abort branches in BOTH files are
belt-and-suspenders for a future report-not-fail agent and are exercised only by hand-built-ctx tests. Severity
of off-by-one-naming findings stays MED/LOW (not on the live failure path). I independently confirmed the
servo deadband is INCLUSIVE (`abs(error) <= tol_deg` → None, _servo.py:138) and wrap180 closes the boundary at
+180 (_servo.py:87, `> 180.0`) — both load-bearing for the findings below.

## Round-1 verification table

| Round-1 finding | Verdict | Evidence |
|---|---|---|
| :298 MED BUG — rotate-cap Abort residual `leg.heading_deg - yaw` UNWRAPPED while convergence judged on wrap180 | **CONFIRMED** | navigate.py:298 `residual = leg.heading_deg - yaw` (no wrap); convergence is `bearing_error_to_rotate` → `error = wrap180(target_deg - yaw_deg)` (_servo.py:137). heading 180 / yaw -170: printed `350.0`, true `wrap180(350)=-10.0`. The WHY lies. |
| :322 MED OPT — `return self.step(ctx)` recurses per 0 cm-rounded leg; should be a bounded while | **CONFIRMED** | navigate.py:319-322 unchanged: `if dist_cm <= 0: self._leg_idx += 1; self._substep = "rotate"; return self.step(ctx)`. Bounded by `len(self._legs)`; re-runs the last_action_ok + budget + yaw guards per skip. No flight-behavior change. |
| :124 LOW OPT — `_rot_cap`/`goal_desc`/`_legs` cold-path; step() builds no lists, one wrap180/clamp; only per-tick allocs are returned Action + off-happy-path Abort f-strings → OPT clean | **CONFIRMED** | __init__ (104-137) and from_config (139-228) hold all planning. step() (238-326) allocates only the returned dataclass on each path; the budget/rotate/yaw Abort f-strings are on early-return error paths, not the happy Rotate/Move path. Hot path is clean. |
| :240 LOW CONV — last_action_ok-False Abort names WHAT/WHICH(drone+leg)/WHY(error)/CHECK | **CONFIRMED** | navigate.py:241-246 includes `{ctx.drone_id}`, `leg {idx}/{len}`, `{ctx.last_action!r}`, `({ctx.last_action_error})`, and `CHECK:`. Compliant. |
| :297 LOW TEST — rotate-cap `>` vs `>=` boundary unpinned; mutant survives | **CONFIRMED** | test_non_converging_rotate_aborts_after_bound (test_navigate.py:288) asserts only `i <= rot_cap+1`. Traced max_step_deg=45 → rot_cap=12: `>` aborts at loop i=12 (13th Rotate), `>=` aborts at i=11 (12th Rotate); both satisfy `i<=13`. Mutant SURVIVES. |
| :254 LOW TEST — budget-vs-await_move ordering unpinned; over-budget final-leg-Done mutant survives | **CONFIRMED** | Budget guard (254) sits above await_move advance (266) and the `_leg_idx>=len` Done (270). test_budget_exceeded_aborts (247) trips budget on the FIRST leg pre-Move (goal (0,50), one leg), never at the final await_move. A mutant hoisting the await_move advance + Done above the budget check would emit Done past budget; untested. |
| :266 LOW TEST — mid-plan yaw→None (after ≥1 OK Move) misnaming-leg mutant survives | **CONFIRMED** | test_yaw_none_aborts (387) feeds yaw=None on the FIRST step (leg 1, substep "rotate"). The yaw guard (281) is below the await_move advance (266); a mutant hoisting it would misname the leg only after a Move resolved — that case is untested. |
| :218 LOW TEST — from_config empty-plan trap tested only via goal==C2; `not legs`→`legs is None` weakening unkillable | **CONFIRMED** | test_from_config_degenerate_plan_is_noop_trap (332) sets c2=(2,2), goal=(2,2) → ZERO legs. plan() returns a tuple, never None, so a `not legs`→`legs is None` mutant still raises on a real zero-leg plan only via the `not` form; the weakening is unkillable. Minor (plan never returns None in practice). |
| takeoff :74 LOW BUG-none — plan is `[Takeoff]` with NO Land; advances `_idx` past then Done; no off-by-one | **CONFIRMED** | takeoff.py:74 `self._plan = [Takeoff(height_cm=height_cm)]`; step() (96-102) returns it once (_idx 0→1) then Done. test_takeoff_then_done_no_land (test_takeoff_phase.py:48) asserts no Land + "HOLDING". Correct. |
| takeoff :88 LOW TEST — no Done-stability test (step twice past Done); `_idx` reset / re-return mutant survives | **CONFIRMED** | _drive (test_takeoff_phase.py:31) breaks on the FIRST Done/Abort; no test steps past completion twice. Unlike navigate's test_done_is_stable_after_completion. A mutant re-returning the action or resetting `_idx` survives. |
| takeoff :64 LOW CONV — height_cm validated on the ground (`_check_height_cm`); band default documented, typos raise | **CONFIRMED** | takeoff.py:68 `_check_height_cm("takeoff", height_cm)` in __init__; test_bad_height_dies_on_the_ground (112) pins int>0, rejects bool/float/str. from_config typo guard via `_zone_kwargs` (test_from_config_rejects_typo:93). Compliant. |
| navigate :43 LOW CONV — stdlib math + finals-internal imports only; no top-level numpy/SDK; no bare except; only bounded loops | **CONFIRMED** | navigate.py:43-52 imports `math` + finals.* only; takeoff.py:41-49 same. test_conventions.py FORBIDDEN_SDK_ROOTS (incl. numpy) + no-bare-except scan cover both (neither is in SDK_ALLOWED, so they are scanned). The only loop-likes are the bounded recursion (:322) and the per-leg `_rot_cap` (:297). |

**No round-1 finding is REFUTED or STALE.** Line numbers all match the current source (minor note: round 1 cited
`navigate.py:298/:322` etc. as `~`; the exact current lines are 298 and 322 — unchanged).

## Fresh findings (round 2 independent pass)

finals/mission/phases/navigate.py:298: MED BUG (CONFIRMS + extends round-1 :298): the unwrapped residual is not just cosmetic — the message ALSO juxtaposes `target heading {leg.heading_deg:.1f}` and `yaw {yaw:.1f}` (both raw) with `residual ~{residual} > tol`, so an operator who eyeballs `180 - (-170)` agrees with the lying 350 and concludes the compass is 350 deg off when it is 10 deg off — pointing them at "stuck/oscillating compass" when the real cause is "max_step_deg too small vs tol" (the SECOND clause of the same CHECK). Fix: `residual = wrap180(leg.heading_deg - yaw)` (import wrap180 from _servo alongside bearing_error_to_rotate) — the exact quantity the deadband tests.
finals/mission/phases/navigate.py:288: LOW TEST: the rotate-cap residual is asserted only by substring `"residual" in a.reason` (test_navigate.py:283) — NO test pins its VALUE, so the :298 wrap bug is itself unkillable by the suite (fixing or breaking the wrap changes nothing red). Add a case with heading 180 / constant yaw -170 asserting the reported residual is ~ -10 (wrapped), not ~350. This is the single highest-value new test: it kills the only MED.
finals/mission/phases/navigate.py:319: LOW TEST: the 0 cm-skip uses `dist_cm <= 0` after `int(round(...))`. A `<=`→`<` mutant would emit a 0 cm Move (round(0.4)=0 → Move(0) the adapter refuses) for a sub-0.5 cm leg. test_tiny_final_leg_rounds_to_zero_is_skipped (474) uses 0.4 cm which rounds to 0 and IS skipped — but it asserts `len(moves)==1`, which a `<` mutant breaks only if the 0 cm Move is actually emitted AND counted as a Move (it is). So `<`→`<` is killed, but a `<=`→`==` mutant (skip only EXACTLY 0, still command negative-rounding legs — not reachable since distance_cm>=0) is vacuously safe. Net: the boundary is adequately pinned; recorded as verified, not a defect.
finals/mission/phases/navigate.py:266: LOW BUG-none (correctness verified): leg index advances ONLY in the `await_move` branch (267), reached only after a re-step with last_action_ok not False (the False guard at 240 returns first) — so the index advances ONLY after a Move resolves OK. Done (270) fires only once `_leg_idx >= len`, i.e. after the FINAL leg's await_move advance. test_failed_move_abort_names_the_current_leg_not_the_next (218) pins no early-advance. At PositionQuality.NONE the phase reads ONLY `telemetry.yaw_deg` (280) and never XY/velocity; the re-orient is absolute (wrap180(heading-yaw), 293-294) so it re-zeroes creep — test_reorient_re_zeros_drifted_yaw (165) pins it. No defect; verified.
finals/mission/phases/navigate.py:230: LOW OPT (cold/warm seam, verified): on_enter (230) and the step() first-call anchor (249) both guard on `self._start_elapsed is None`, so a phase entered via on_enter then stepped does NOT re-anchor (no budget-clock reset that would mask an overrun). The hand-built-ctx tests that skip on_enter still get a clock. No double-anchor bug; verified.
finals/mission/phases/navigate.py:254: LOW TEST (extends round-1 :254): add the missing final-leg-over-budget case — drive to the final leg's await_move (last_action_ok True), then step with elapsed > budget; assert Abort, not Done. Without it the "Done sneaks past budget" reorder mutant survives, and a real too-tight budget on the last leg would silently report success instead of failing loud (the convention-3 bound is the load-bearing guarantee here).
finals/mission/phases/takeoff.py:88: LOW TEST (confirms round-1): add a Done-stability assert (step twice past Done returns Done, no Land, no second Takeoff) to match navigate's test_done_is_stable_after_completion. A `_idx` reset or re-emit mutant currently survives; in the live agent a phase is single-shot so the blast radius is small, hence LOW.
finals/mission/phases/navigate.py:319: LOW OPT (NOT worth fixing — recorded): the recursion at :322 could be a `while int(round(self._legs[self._leg_idx].distance_cm)) <= 0` loop to drop the per-skip stack frame + guard re-eval, BUT correctness > performance and the current form re-evaluates the budget/yaw guards on each skip which is harmless (same ctx) and the skip count is bounded by the finite leg list (≤ ~20 legs in any briefed route). REJECT a rewrite unless it provably keeps the budget + last_action_ok + yaw guards ABOVE the loop — round 1's caveat stands. Priority: leave as-is; the MED :298 wrap is the only change that touches behavior-adjacent code.

## Severity counts (round 2 = round-1 re-verified + fresh)

Round-1 findings re-verified: CRIT 0 / HIGH 0 / MED 2 / LOW 10 — ALL CONFIRMED, none refuted/stale.
Fresh round-2 findings: CRIT 0 / HIGH 0 / MED 0 (the :298 fresh note re-grades the SAME round-1 MED, not a new one) / LOW 3 actionable (`:288` residual-value test, `:254` final-leg-over-budget test, takeoff `:88` Done-stability test) + 4 verified-clean records (`:266`, `:230`, `:319` boundary, `:319` recursion-leave-as-is).

## Verdict

The slice is correct on every audited safety point: open-loop transit at PositionQuality.NONE re-zeroes yaw creep
via the ABSOLUTE re-orient (re-confirmed against _servo wrap180/deadband); the leg index advances ONLY after a
Move resolves OK; Done fires ONLY after the final Move; both bounds exist (per-leg `_rot_cap` AND the global
`total_budget_s`), so there is no unbounded spin and no battery-dead hover (transit Aborts → the agent safes
down). Conventions pass mechanically for both files. The ONE behavior-adjacent defect is the round-1 MED at
:298 (unwrapped residual in a fail-loud message), which round 2 confirms is ALSO unkillable by the current
suite (LOW TEST :288) — fixing the wrap and adding the residual-value assertion is the single highest-value
follow-up. The :322 recursion is confirmed bounded and harmless; leave it. No CRIT/HIGH introduced or found.
