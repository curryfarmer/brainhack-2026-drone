# S-DECON round 1 — multi-drone deconfliction (launch slot + SectorGuard + serialized landing + Challenge-2 configs)

Reviewed: guards.py (launch slot + SectorGuard only), mission/agent.py (launch routing), main.py (_build_guards/_build_safety/_build_agents), config.py (deconfliction), configs/landing_real.json, configs/convoy_real.json, tests/test_deconfliction.py, tests/test_landing_config.py, tests/test_convoy_config.py.

## Deadlock / slot-release audit (no CRIT found)
- Each agent holds at most ONE slot at a time: `launch_slot` async-CM scopes ONLY the single Takeoff `_execute` (agent.py:558-561), released in `finally` (guards.py:766-768); the landing slot is acquired later, after launch is released. No hold-and-wait cycle. CONFIRMED deadlock-free.
- launch_slot releases on every path: normal exit, exception (test_launch_slot_released_on_exception), and orchestrator cancel (CancelledError unwinds through the `finally`). The acquire-then-`try`/`finally` window is zero statements wide; `_try_log` is INSIDE the guarded `try`. No leak.
- `asyncio.timeout`+`sem.acquire()` grant-then-cancel race: NOT a leak on CPython 3.11 — Semaphore.acquire hands a granted-then-cancelled permit to the next waiter (`_wake_up_first`). The docstring's choice of `asyncio.timeout` over `wait_for` is sound. No finding.
- land()/launch_slot() are event-driven (semaphore) + bounded retry sleep; no busy-spin.

## Findings

guards.py:513: LOW CONV: `SectorGuard.__init__` annotates `c2_origin_m: Tuple[float, float]` but `Tuple` is NOT imported (line 111-112 imports Callable/Dict/List/Optional/Sequence/TextIO only). Harmless at runtime (PEP 563 string annotation) but `typing.get_type_hints(SectorGuard.__init__)` and annotation-evaluating tooling raise NameError. Fix: add `Tuple` to the typing import.

guards.py:530,538: LOW OPT (hot path): `check()` runs `from finals.mission.planning.frame import bearing_from_c2_deg, in_sector` on EVERY agent tick (~10 Hz x N drones). frame.py imports only finals.errors (no import cycle with guards.py), so this can be a module-level import; current cost is a per-tick `sys.modules` dict lookup (negligible) but it is redundant per-tick work and the lazy form is unnecessary. Fix: hoist both imports to module top. Does NOT conflict with any convention (frame is a pure leaf). REJECT-if-cycle: verified no cycle, so safe.

configs/landing_real.json:29,44,59 + config.py: MED BUG (missing fail-loud): two drones can target the SAME pad and the config loads CLEAN — the bench/real separation guard (config.py:552-566) only checks bands-OR-sectors, never pad-target distinctness, and sectors may be distinct while pad_ids collide. For the LANDING mission the per-drone pad IS the spatial allocation; duplicate targets = two drones land on one physical pad (sequentially, since the landing slot serializes descent, so NOT an in-air collision — hence MED not CRIT — but a guaranteed mission/score failure). Fix: in config.py `_validate` for profile in (bench,real), when every drone has a `zone["navigate"]["pad_id"]`, refuse duplicate pad_ids with an actionable ConfigError (or enforce it in main when arena is present).

tests/test_landing_config.py:90: MED TEST: `test_duplicate_pad_target_is_caught` only asserts the duplicate is OBSERVABLE (`len(set(targets)) < 3`) and explicitly comments "load itself is fine" — it does NOT kill the mutant "two drones aim at one pad". Add the loader/build refusal above + a `pytest.raises(ConfigError, match="pad")` test pointing two drones at one pad. (Pairs with the MED BUG.)

tests/test_landing_config.py: LOW TEST: no test pins that the per-drone advisory sectors are DISTINCT or that each sector is centred on its pad's bearing-from-C2 (the slice intent). Mutant: swap two drones' `sector_deg` (alpha<->bravo) — suite stays green though every drone's advisory wedge now points at the wrong neighbour's space (sectors are advisory, so this is LOW, but the round-1 contract asked for distinct/aligned sectors). Add a test asserting `bearing_from_c2_deg(pad_center, c2_origin)` falls within each drone's wedge.

configs/landing_real.json:29,44,59: LOW TEST: `valid_marker_ids` are PLACEHOLDER ids [10],[11],[12] (flagged in `_comment_marker`) but nothing FAILS if a scored run ships with placeholders. Deferred-to-onsite by design (gate F), but there is no machine check that an obviously-synthetic id set was replaced. Acceptable to defer; noting as the one un-gated placeholder. No code fix required.

config.py:552-566: LOW OPT/CONV (note, not a defect): the multi-drone separation guard accepts sectors-on-all WITHOUT requiring the sectors be distinct or non-overlapping. This is CORRECT per the design (sectors are ADVISORY; TIME slots + open-loop routes are the real separation) — flagging only to confirm it is intentional and that overlap is not silently treated as separation. No fix; REJECT any change that would make overlapping sectors a hard refusal (would break the advisory contract).

## Mutants the suite DOES kill (verified by reading the tests)
- drop launch-slot release in `finally` -> test_launch_slot_serializes_concurrent_takeoffs (len(windows)==3) FAILS.
- remove launch-slot acquire entirely -> same test (overlap assertion) + test_launch_slot_logs_acquire_release FAIL.
- remove the launch_slot_wait_s deadline -> test_launch_slot_wait_is_bounded FAILS (expects FlightTimeout).
- SectorGuard returns LAND_THIS/HOLD instead of ADVISORY -> test_sector_guard_trips_advisory_outside_wedge (`is ADVISORY` + `< HOLD_THIS`) FAILS.
- shared semaphore for launch+landing -> test_launch_and_landing_slots_independent (elapsed<0.2) FAILS.
- disable separation guard (accept neither bands nor sectors) -> test_no_separation_at_all_fails_loud / test_duplicate_band_fails_loud FAIL.
- launch-slot not released on a takeoff failure -> test_one_drone_fails_at_launch_others_finish_exactly_once_emergency (alpha/charlie DONE) FAILS.

## Conventions (mechanical)
- `except Exception` only at guards.py:215 (evaluate_guards) + guards.py:848 (retry ladder) — both whitelisted + documented. No bare `except`.
- No unbounded while in the slice (guards.py:950 bounded by stop event; agent.py none; main.py:793 is _areplay, out of slice, deadline-bounded).
- All awaited ops bounded: launch_slot/land via asyncio.timeout, retries via count+wall-clock, agent commands via outer wait_for. Slot timeouts raise typed FlightTimeout with WHAT/WHICH/WHY(after N s)/CHECK(heartbeat.json).
- No top-level SDK import added; no module-level mutable global added.
