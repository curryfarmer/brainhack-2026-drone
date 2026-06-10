# PAD-VALID round 1 — adversarial review

Scope: NEW `finals/mission/pad_validity.py` (PadValidityMap: cross-drone validity
broadcast + single-owner pad claim, threading.Lock + dict, mirroring ObstacleMap /
ConvoyRegistry); the `_valid_sightings` + `validity_map` constructor seam I OWN in
`phases/land_on_pad.py`; orchestrator heartbeat snapshot; main.py wiring;
`test_pad_validity.py` (46) + `test_pad_validity_e2e.py` (5). Full suite 1361 green
at review time (baseline 1310 + 51 new). 3/3 mutants killed (see end).

Method-ownership for hand-merge with PAD-DETECT: I edited ONLY `__init__`,
`from_config`, `_valid_sightings` in land_on_pad.py. `_valid_sightings` CALLS the
PAD-DETECT-owned `_pick_target` (read-only) — flagged below.

## BUGS / correctness
finals/mission/pad_validity.py:19-32: MED DOC-DRIFT (FIXED this round): the module
VALIDITY docstring said "Last writer wins", contradicting the sticky-invalid rule
the code now implements (record() keeps False on a False→True). A reader would
believe a later valid=True resurrects a red pad. Fix applied: rewrote the block to
state "INVALID IS STICKY (safety-monotone) ... a later valid=True NEVER resurrects
a red pad".
finals/mission/pad_validity.py:106: MED DOC-DRIFT (FIXED this round): class
docstring said "last-writer validity" — same drift. Fixed to "sticky-invalid
validity".
finals/mission/phases/land_on_pad.py:385: LOW BUG (within-spec): `_valid_sightings`
CLAIMS the top pick on every tick it sees a valid pad, BEFORE the N-of-M acquire
gate fires (_step_acquire line 507). So a drone that has merely SIGHTED a pad (but
not yet locked/committed) holds the claim, blocking other drones, even if it later
drifts off and never lands there. This is eager claim-on-sight, not claim-on-
commit. It is SAFE (over-exclusion only ever costs throughput, never causes a
double-land) and matches the spec's "two drones must never chase the SAME valid
pad" (chasing == sighting+servoing). Onsite-deferred decision: claim-on-acquire vs
claim-on-sight, plus whether to add a release() if a drone abandons a pad. Recorded
as a non-goal in the module docstring (no TTL — landing is terminal).
finals/mission/phases/land_on_pad.py:385,509: INFO (hand-merge): `_pick_target`
(PAD-DETECT-owned) is called TWICE per tick — once here to pick the claim target,
once in `_step_acquire` to set `_target_marker_id`. Both run over the same
candidate list, and `_pick_target` is deterministic (largest bbox, lowest-id tie-
break), so claim target == servo target by construction. INVARIANT for hand-merge:
PAD-DETECT must keep `_pick_target` a pure deterministic function of its input list
(no per-call state, no RNG) or claim/servo could disagree. Asserted by
test_pick_target_deterministic_across_ticks.

## MISSING / WEAK tests
finals/tests/test_pad_validity.py: the GIL hides a dropped-lock mutant on a pure
threaded test (a `nullcontext()` lock passed 5/5). Addressed with the `_race_hook()`
seam (pad_validity.py:150) called INSIDE the lock at each check→set boundary, plus
`_ForcedRaceMap` + `threading.Barrier(2, timeout=0.5)` in
test_lock_makes_claim_single_winner_deterministically: dropping the lock lets BOTH
barrier-synced threads pass the check before either sets, so two winners appear
deterministically — the mutant fails every run, not flaky-pass. JUSTIFIED: the hook
stays inside the lock, so it can never itself break mutual exclusion in production
(it is a no-op return None).
finals/tests/test_pad_validity_e2e.py: E2E drives the orchestrator over MockAdapter
with a scripted SightingBus; covers (a) invalid broadcast → other drones never
claim that pad, (b) 3 drones / 1 valid pad → exactly one claims, (c) 2 drones / 2
valid pads → no shared claim, (d) invalid broadcast drops a statically-valid pad for
another drone, (e) an UNREAD valid pad is still landable via the static set
(validity_map present but no prior record). Gap accepted: no E2E that exercises a
real tick-simultaneous claim collision through the orchestrator (the orchestrator
steps agents sequentially per tick, so a true same-instant CAS race only arises
under the threaded unit tests — which DO cover it). Noted, not a hole.

## CONVENTIONS (mechanical)
finals/mission/pad_validity.py: PASS — stdlib only (threading + math +
finals.errors; NO cv2/numpy → bare-venv unit-testable); no module globals; no bare
`except`/`except Exception`; no while-loops; typed error (PadValidityError <
FinalsError) with WHAT/WHICH(beacon/drone)/WHY/CHECK on every malformed input;
bool-vs-int guard on beacon_id and valid and ts (bool is an int subclass — rejected
explicitly); injected clock (ts never read from the map → pure); snapshot() emits
sorted, JSON-serializable copies and never hands out a PadRecord; units in names
(_ts/_s); expected race outcomes returned as bool (claim False), NOT raised.

## land_on_pad.py seam (the static-default invariant)
finals/mission/phases/land_on_pad.py:360: PASS — `validity_map=None` returns
`statically_valid` BEFORE any map call, so every pre-existing land test (which
never passes a map) is byte-for-byte unchanged. Verified: the 63 existing
land_on_pad tests stay green.

## Mutants (kill-check, applied → NAMED test fails → reverted clean; diff -q == backup)
1. drop the lock (record/claim under nullcontext): KILLED by
   test_lock_makes_claim_single_winner_deterministically (got two winners
   ['alpha','bravo'] every run via the barrier seam).
2. claim() always returns True: KILLED by
   test_concurrent_claim_exactly_one_winner_under_threads (two True winners) and
   test_double_claim_refused.
3. record() ignores the sticky-invalid branch (last-writer): KILLED by
   test_invalid_is_sticky_never_upgraded_to_valid AND the E2E
   test_invalid_broadcast_drops_a_statically_valid_pad_for_another_drone.
3/3 mutants killed.
