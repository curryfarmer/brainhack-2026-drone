# S-DECON round 2 — adversarial re-review (multi-drone deconfliction + Challenge-2 mission configs)

Scope: finals/mission/orchestrator.py, finals/guards.py (launch slot + landing slot + SectorGuard +
evaluate_guards), finals/config.py (separation guard + arena resolution), finals/mission/agent.py (launch
routing + per-drone guard tick), finals/mission/phases/navigate.py + land_on_pad.py (pad resolution),
finals/configs/{landing_real,convoy_real}.json, finals/configs/arenas/sample.json, and the tests:
test_deconfliction.py, test_landing_config.py, test_convoy_config.py, test_arena_config.py, test_guards.py,
test_orchestrator.py. Python pinned 3.11.9 (verified — the grant-then-cancel analysis is interpreter-correct).

Round 1 baseline: CRIT 0 / HIGH 0 / MED 2 / LOW 5.
Round 2 verdict: CRIT 0 / HIGH 0 / MED 2 / LOW 5. No new CRIT/HIGH. The deadlock-freedom and
slot-release-on-every-path properties are re-confirmed independently (NOT taken on trust).

NOTE on round-1 file paths: round 1 cited `tests/test_landing_config.py:90` etc. Those files exist in THIS
worktree (Grep found them; the Glob mtime index was momentarily stale). The current test surface is RICHER
than round 1 described — test_landing_config.py now ships test_pads_are_distinct_and_valid,
test_nonexistent_pad_id_fails_loud, test_invalid_pad_target_is_caught_by_land_on_pad, test_missing_arena_fails_loud,
test_empty_valid_marker_ids_fails_loud. This narrows but does NOT close the round-1 MED dup-pad gap (below).

---

## Round-1 verification table

| # | Round-1 finding | Verdict | Evidence |
|---|---|---|---|
| R1-MED-1 | Two drones can target the SAME pad; config loads clean (no pad-target distinctness check) | **CONFIRMED** | No `pad_id` distinctness check exists in config.py (`Grep pad_id` -> none), main.py (`_build_phases`/`_build_agents`), or navigate.py. `Navigate.from_config(drone_cfg, cfg)` (navigate.py:140) sees ONE drone + global cfg only — it checks the pad EXISTS in the arena (navigate.py:176-181) but is structurally blind to sibling drones. Building 3 agents that all target one pad succeeds. |
| R1-MED-2 | The matching test only asserts the dup is observable, not that the loader refuses it | **CONFIRMED** | test_landing_config.py:90 `test_duplicate_pad_target_is_caught` ends at `assert len(set(targets)) < 3` (line 103) with the comment "load itself is fine". It mutates a config to dup a pad and asserts only observability. It would still pass if every dup-pad refusal were deleted. |
| R1-LOW-1 | guards.py:513 `SectorGuard.__init__` annotates `Tuple` but `Tuple` not imported | **CONFIRMED** | guards.py:111-112 imports `(TYPE_CHECKING, Callable, Dict, List, Optional, Sequence, TextIO)` — no `Tuple`. Annotation at guards.py:513 `c2_origin_m: Tuple[float, float]`. Harmless at runtime (`from __future__ import annotations` at line 99 = string annotation) but `typing.get_type_hints(SectorGuard.__init__)` raises NameError. |
| R1-LOW-2 | guards.py:530,538 SectorGuard does lazy `from finals.mission.planning.frame import ...` per check() tick | **CONFIRMED** | guards.py:538 imports `bearing_from_c2_deg, in_sector` inside check() on every tick; guards.py:530 in __init__. frame.py (verified) imports only `finals.errors` + stdlib — NO cycle with guards.py, so the hoist to module top is safe. Per-tick cost is a sys.modules dict hit (negligible) — OPT only. |
| R1-LOW-3 | test_landing_config: no test pins per-drone sectors are DISTINCT or pad-bearing-aligned | **CONFIRMED** | `Grep bearing_from_c2 / distinct sector` in test_landing_config.py -> only the `sector_deg is not None` assertion (line 46). Swapping alpha<->bravo `sector_deg` keeps the suite green. Sectors are advisory so LOW. |
| R1-LOW-4 | landing_real.json:29/44/59 valid_marker_ids are PLACEHOLDERS [10],[11],[12]; nothing fails if shipped | **CONFIRMED (defer-by-design)** | Still [10]/[11]/[12] with `_comment_marker` placeholder notes. Deferred to onsite gate F by design; no machine check that a synthetic id set was replaced. No code fix required. |
| R1-LOW-5 (note) | config.py:552-566 separation guard accepts sectors-on-all WITHOUT requiring distinctness; intentional | **CONFIRMED (intentional)** | config.py:552-566 checks `sectors_all = all(d.sector_deg is not None ...)` with no distinctness/overlap test. This is correct per design (sectors ADVISORY; TIME slots + open-loop routes are the real separation). REJECT any change making overlapping sectors a hard refusal. |

### Round-1 deep claims re-verified INDEPENDENTLY (not trusted)

- **Deadlock-freedom — CONFIRMED.** agent.py `_execute` (515-619) handles exactly ONE action per call. The
  launch slot (`async with self._safety.launch_slot(...)`, agent.py:558) wraps ONLY the Takeoff adapter call
  and opens+closes within that single `_execute`. The landing slot is acquired in a SEPARATE `_execute` for
  Land (agent.py:469 -> 575-587 -> safety.land/trip). The two are distinct semaphores (`_slot` vs
  `_launch_slot`, guards.py:682-683). An agent never holds the launch slot while waiting on the landing slot
  (takeoff is strictly sequenced before any land). No hold-and-wait -> no cycle.
- **Slot-release-on-every-path — CONFIRMED.** launch_slot (guards.py:734-768): acquire is in its own
  `try/except TimeoutError` block; the `yield` is inside a separate `try ... finally: sem.release()`
  (766-768). Normal exit, exception, and CancelledError all unwind through that finally. On acquire timeout
  the permit was never obtained (acquire raised, returning its own permit — see next bullet), so nothing
  leaks. land() (guards.py:780-797) has the identical shape (`finally: sem.release()` at 796-797). VERIFIED.
- **asyncio.timeout + sem.acquire() grant-then-cancel race — CONFIRMED no leak on 3.11.9.** Read the actual
  CPython 3.11.9 `asyncio.locks.Semaphore.acquire` source. The granted-then-cancelled path is handled:
  `except CancelledError: if not fut.cancelled(): self._value += 1; self._wake_up_next(); raise` — if the
  future was already granted (set) when the timeout cancel arrives, the permit is RETURNED and re-handed to
  the next waiter. No leak. (Round 1 named the helper `_wake_up_first`; the real method is `_wake_up_next` —
  cosmetic slip, mechanism correct.) The docstring's choice of `asyncio.timeout` over `wait_for` is sound.
- **SectorGuard advisory-only — CONFIRMED.** guards.py:537-560 returns `_trip(TripAction.ADVISORY, ...)`
  only; skips on `position_m is None`; edge-latched (`self._outside`). test_deconfliction.py:234-244 pins
  `trip.action is ADVISORY` AND `trip.action < HOLD_THIS`. Removing the `< HOLD_THIS` invariant would not be
  caught by anything else, but the `is ADVISORY` assertion kills a severity-escalation mutant.

---

## Fresh findings (round 2)

finals/config.py + finals/main.py:295: MED BUG: duplicate navigate.pad_id across drones is accepted (R1-MED-1, re-confirmed). load_config + _build_agents->_build_phases->Navigate.from_config build each drone in isolation; nothing cross-checks pad-target distinctness. Serialized by the landing slot so NOT an in-air collision (hence MED), but a guaranteed mission/score failure (two drones fight for one physical pad). Fix: in config.py `_validate`, for profile in (bench,real) when every drone has zone["navigate"]["pad_id"], refuse duplicate pad_ids with an actionable ConfigError (WHAT=duplicate pad target, WHICH=the colliding drone ids + pad_id, WHY=two drones cannot score one pad, CHECK=zone.navigate.pad_id).

finals/tests/test_landing_config.py:90: MED TEST: test_duplicate_pad_target_is_caught only asserts observability (`len(set(targets)) < 3`), explicitly "load itself is fine" (R1-MED-2, re-confirmed). It does NOT kill the "two drones aim at one pad" mutant. Fix: pair with the config.py guard above and a `pytest.raises(ConfigError, match="pad")` pointing two drones at one pad. NOTE: test_pads_are_distinct_and_valid (line 56) pins distinctness on the SHIPPED config only — it catches a regression in landing_real.json but does NOT prove the loader refuses arbitrary dup configs.

finals/mission/phases/land_on_pad.py + navigate.py:173: LOW BUG: a drone may navigate to a pad whose `valid=false` (red decoy) and the build succeeds clean. Navigate.from_config plans to ANY existing pad regardless of `valid` (navigate.py:182 takes `pads[pad_id].center_m` with no validity check); land_on_pad never receives the arena and cannot cross-check its navigate target's validity. test_invalid_pad_target_is_caught_by_land_on_pad (test_landing_config.py:106) only asserts the pad IS red — it does NOT assert the build refuses it. Runtime symptom: land_on_pad never sees a valid marker -> acquire timeout -> phase failure (so a soft failure, not a collision). Sibling of the dup-pad gap. Fix: same config-time guard — refuse a navigate.pad_id whose arena pad has valid=false (or warn LOUD).

finals/config.py:552: LOW BUG: no check that the arena has >= N valid pads for N drones on a LANDING (pad-target) config. With duplicate + red pad targets both accepted, three drones could be wired against fewer than three green pads and load clean. Same class as the dup-pad gap; the single fix (distinct + valid pad targets) closes it. The sample arena ships 3 green pads for 3 drones so the SHIPPED config is fine — this is a guard against an onsite edit.

finals/config.py:554: LOW BUG (mirror of R1-LOW-5): two drones may carry an IDENTICAL sector_deg and the separation guard accepts it (sectors_all only checks presence, not distinctness). Because sectors are advisory-only this has no real-flight consequence (LOW), but a duplicated wedge means the advisory cross-check is silently wrong for one drone. Acceptable to leave; if touched, warn (never hard-refuse — that would break the advisory contract).

finals/guards.py:111: LOW CONV: `Tuple` is annotated (line 513) but not imported (R1-LOW-1, re-confirmed). Fix: add `Tuple` to the typing import on line 111-112.

finals/guards.py:530,538: LOW OPT (hot path): SectorGuard.check() runs `from finals.mission.planning.frame import bearing_from_c2_deg, in_sector` on every agent tick (R1-LOW-2, re-confirmed; no import cycle — safe to hoist). Fix: module-level import.

finals/tests/test_landing_config.py:46: LOW TEST: no test pins per-drone sectors are DISTINCT or centred on each pad's bearing-from-C2 (R1-LOW-3, re-confirmed). Mutant: swap alpha<->bravo sector_deg — suite stays green. Add `bearing_from_c2_deg(pad_center, c2_origin)` within each drone's wedge.

finals/configs/landing_real.json:29,44,59: LOW TEST (defer): valid_marker_ids placeholders [10]/[11]/[12], no machine check they were replaced (R1-LOW-4, re-confirmed). Deferred to gate F by design. No code fix.

### Time-stagger / liveness (fresh pass — clean)

- The launch corridor stage-gate is `asyncio.Semaphore(1)` (guards.py:730), strictly single-occupancy: it
  can NEVER admit two drones to the shared C2 takeoff zone at once. The "below the ceiling" framing maps to
  this slot (altitude bands are deliberately illegal under the 1.1 m ceiling — see SafetyController
  docstring). Verified single-holder.
- The launch acquire is deadline-bounded by `launch_slot_wait_s` -> FlightTimeout (guards.py:752-761); land
  by `slot_wait_s` (guards.py:786-793). No infinite wait. test_deconfliction.py:133 (launch) and
  test_guards.py:711 (landing) pin the bounded-timeout path.
- No busy-spin: the agent phase loop (agent.py:351) advances `_phase_idx` on Done or awaits a bounded
  command/Wait every iteration over a FINITE phase queue; the orchestrator loop (orchestrator.py:241) waits
  `asyncio.wait(pending, timeout=heartbeat_period_s)` each beat and is bounded by the hard deadline
  (orchestrator.py:252). Both event-driven.

### Config validation fail-loud (fresh pass — clean)

- Unknown keys -> ConfigError naming them + valid keys (config.py:_check_keys 204-208); applied to top,
  detector, guards, every drone, and zone["navigate"] (navigate.py:74-78). No silent drop of unknown keys
  ('_'-prefixed comment keys are intentionally ignored, documented).
- Missing required keys -> ConfigError with the exact key (config.py:209-211).
- The multi-drone separation guard refuses NEITHER-bands-nor-sectors loudly (config.py:556-566); pinned by
  test_convoy_config.py:69 (no-sep) and :59 (dup-band). The arena resolution surfaces semantic arena errors
  THROUGH load_config (test_arena_config.py:237). All ConfigError messages carry WHAT/WHICH/WHY/CHECK.

### Conventions (mechanical — clean)

- `except Exception` only at guards.py:215 (evaluate_guards wrapper) + guards.py:848 (land retry ladder) +
  orchestrator.py:334/415/423/433 (top-loop + reconcile + shutdown + final-heartbeat nets) — all inside
  whitelisted FILES (test_conventions.py EXCEPT_EXCEPTION_WHITELIST: guards.py, orchestrator.py). Every catch
  prints a full traceback via `_net`/inline. No bare `except`.
- No unbounded `while` in the slice: orchestrator.py:241 bounded by all-done OR hard_deadline; agent.py:351
  by the finite phase queue + deadline + stop; the land ladder (guards.py:831) by `range(1, attempts+1)` AND
  the wall-clock window (860-864). AbortListener thread (guards.py:950) bounded by the stop event + one-shot.
- Units in names: `_s` (seconds), `_m` (metres), `_deg`, `_pct`, `_cm`, `_hz` throughout. Consistent.

### Code-optimization pass (orchestrator/agent hot paths — clean except R1-LOW-2)

- Orchestrator tick (1 Hz): per-iteration it builds a `pending` list, one `GuardContext`, and
  evaluate_guards allocates one `trips` list. All necessary (immutable snapshot) and negligible at 1 Hz.
  `_write_heartbeat` is a documented synchronous fsync trade-off at 1 Hz. No busy-spin, await back-pressure
  present (`asyncio.wait(..., timeout=...)`).
- Agent tick (~tick_hz): one GuardContext + one evaluate_guards list per loop (agent.py:372). Necessary.
  Only real OPT is the SectorGuard lazy import (R1-LOW-2 above).

---

## Mutants the suite kills (verified by reading the tests)

- drop launch-slot release in `finally` -> Semaphore(1) never frees -> 2nd/3rd takeoff blocks to the 30 s
  deadline -> FlightTimeout -> agents not DONE -> test_launch_slot_serializes_concurrent_takeoffs
  (`assert all(... DONE)`) AND test_launch_slot_logs_acquire_release FAIL.
- remove launch-slot acquire / share one semaphore for launch+landing -> test_launch_and_landing_slots_independent
  (`elapsed < 0.2`) FAILS; takeoff overlap assertion in test_launch_slot_serializes_concurrent_takeoffs FAILS.
- remove the launch_slot_wait_s deadline -> test_launch_slot_wait_is_bounded (`pytest.raises(FlightTimeout,
  match="launch corridor")`) hangs/FAILS.
- launch-slot not released on a takeoff FAILURE -> test_one_drone_fails_at_launch_others_finish_exactly_once_emergency
  (alpha/charlie DONE) FAILS.
- SectorGuard escalates above ADVISORY -> test_sector_guard_trips_advisory_outside_wedge
  (`is ADVISORY` + `< HOLD_THIS`) FAILS.
- disable the separation guard -> test_no_separation_at_all_fails_loud / test_duplicate_band_fails_loud FAIL.
- arena pad out of bounds / dup pad id / c2 outside bounds -> test_arena_config.py cases FAIL.

## Surviving mutants (NOT killed — the gaps above)

- **two drones target one pad** (dup navigate.pad_id) -> NO test fails; test_duplicate_pad_target_is_caught
  passes either way (asserts only observability). [MED — R1-MED-1/2]
- **a drone navigates to a red (valid=false) pad** -> NO test fails at build time. [LOW]
- **two drones share an identical sector_deg** -> NO test fails (advisory-only). [LOW]
- **swap two drones' sector_deg (alpha<->bravo)** -> suite stays green. [LOW]
