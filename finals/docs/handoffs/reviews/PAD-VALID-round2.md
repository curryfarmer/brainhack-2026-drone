# PAD-VALID round 2 — adversarial re-audit + new gaps

Re-audit of round 1 (all round-1 findings re-checked) PLUS fresh-eyes pass on the
orchestrator/main wiring, snapshot edge cases, and the `_valid_sightings` ordering.
Full suite still 1361 green. 3/3 mutants killed (unchanged).

## Round-1 findings re-checked
- module/class DOC-DRIFT (sticky-invalid): FIXED and verified — pad_validity.py:19-32
  and :106 now both say sticky-invalid; the smoke `__main__` asserts
  `is_valid(67) is False` after only a False record (no resurrect). RESOLVED.
- eager claim-on-sight (land_on_pad.py:385): RE-CONFIRMED within-spec and safe
  (over-exclusion only costs throughput). Left as the onsite-deferred timing call.
  No code change — adding a release() now would be unrequested scope and risks a
  stranded-claim bug of its own. RESOLVED (documented non-goal).
- `_pick_target` called twice / determinism (hand-merge): RE-CONFIRMED. Pinned by
  test_pick_target_deterministic_across_ticks. RESOLVED.

## NEW findings (round 2)
finals/mission/phases/land_on_pad.py:366-369 / :373-375: INFO ORDERING (verified
correct, no change): the record() broadcast loop runs BEFORE the is_valid() filter,
so within a SINGLE drone's own tick its fresh reads are already in the map when it
filters. This is intended (a drone's own read should inform its own filter), and it
is harmless because record() of a statically-valid id writes True (is_valid → True,
`is not False` keeps it) and a statically-invalid id is never in `statically_valid`
to begin with. The cross-drone protection that matters (another drone's earlier
False) is the one this filter actually acts on. No flapping: a drone never
red-flags its own valid pad. PASS.

finals/mission/pad_validity.py:272-303 / orchestrator.py:500: INFO SNAPSHOT-TS
(verified safe): snapshot(now) calls `_check_ts(now)` and will RAISE on a non-finite
now. The orchestrator passes its own monotonic tick `now` (a finite float from
self._clock()), and the heartbeat is written every tick with that same now used for
the convoys snapshot — so the production caller can never trip it. age_s is None
when read_ts is None (an unread/claim-only beacon), guarding the subtraction. PASS.

finals/mission/pad_validity.py:258-269: LOW (accepted) `claimed_by_other` is
non-mutating and does NOT create a record, but `claim()` DOES get-or-create. So
order matters across the swarm: a drone querying `claimed_by_other` on a never-seen
beacon returns False (correct — nobody owns it) without polluting the map, whereas
the eventual claim() lazily creates the record. Intentional and consistent with
ObstacleMap's query-vs-mutate split. No leak: only the 5 fixed beacon ids ever flow
in, so the dict is bounded regardless. PASS.

finals/main.py:333-335: INFO WIRING (verified): the generalized
`shared_by_param` loop injects a shared object ONLY when (a) it is non-None AND (b)
its param name is in the factory signature. So a phase whose from_config does not
name `validity_map` (every phase except land_on_pad) is untouched, and a
non-landing mission (validity_map=None) never injects. This is the same
signature-checked discipline as navigate's obstacle_map and is covered by the
existing _build_phases tests staying green. PASS.

finals/tests/test_pad_validity.py: NEW GAP CLOSED-OR-ACCEPTED: I re-checked whether
a mutant that swaps `is_valid(...) is not False` to `is True` in `_valid_sightings`
(which would drop UNREAD valid pads) is killed. It IS — by the E2E
test_unread_valid_pad_still_landable_via_static_set, which presents a validity_map
with NO prior record and asserts the pad is still selected. Confirmed this is a
land_on_pad-side guard, complementing the pad_validity unit tests. No new test
needed.

finals/mission/pad_validity.py:190-199: re-audited the sticky-invalid branch for an
ASYMMETRY hole: True-then-False (a green pad later read red) takes the normal
`rec.valid = valid` path (line 197) and correctly flips to False — sticky only
blocks False→True, never True→False, so a real red read always wins. Pinned by
test_record_true_then_false_applies_invalid AND
test_invalid_is_sticky_never_upgraded_to_valid (the two directions). PASS.

## Verdict
No open correctness bugs. The two doc-drift items from round 1 are fixed. All
remaining notes are within-spec, intentional, or onsite-deferred config/timing
decisions (NOT code). 3/3 mutants killed, reverted clean (diff -q vs backup ==
identical). Clean to commit to the PAD-VALID worktree branch (do NOT merge).
