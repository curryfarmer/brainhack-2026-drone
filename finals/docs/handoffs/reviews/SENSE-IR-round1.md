# SENSE-IR round 1 — adversarial review

Scope: ProximityGuard (HULA 4-dir IR -> advisory->LAND ladder) in finals/guards.py;
GuardsConfig proximity_* thresholds + validation in finals/config.py; the OPTIONAL
DepthSource seam (finals/vision/depth.py) + FakeDepthSource; the ProximitySensor seam
(finals/flight/proximity.py); main.py wiring (_build_guards / _build_proximity_fn /
resolve_depth_source_cls / _build_depth / depth lifecycle in _amain); agent.py
proximity_fn plumbing. Tests: test_proximity_guard.py (35), test_depth.py (20),
test_proximity_sensor.py (7). Full suite 1372 green at review time. 3/3 mutants killed.

Format: `path:line: <SEV> <tag>: <problem>. <fix>.`

## BUGS / correctness
finals/guards.py:ProximityGuard.check: INFO BUG-CLEAR: the bad-reading fail-safe
(negative / non-finite / wrong-type -> LAND_THIS) returns on the FIRST bad direction
without scanning the rest. Intentional: a single corrupt IR channel is already a hard
fail-safe, so there is no value continuing the scan; the closest-valid computation is
moot once we LAND. No fix.
finals/config.py:_validate_guards: LOW BUG: proximity_warn_cm/land_cm are validated by
`_num` (> 0, finite) and the land<warn ordering even when proximity_enable is False —
correct (catch a bad value on the ground the moment someone flips enable), but note the
ConfigError fires for a disabled guard with e.g. proximity_land_cm=0. That is the
intended "validate-always" weights-guard philosophy; documented in the diff comment. No
fix — keep loud-on-the-ground.
finals/main.py:_build_proximity_fn: LOW SMELL: the `api=None` parameter is accepted and
threaded from _build_agents but UNUSED today (SyntheticProximitySensor ignores it). It is
the forward seam for the onsite swap to PyhulaxProximitySensor(drone.id, api). Acceptable
as a documented onsite hook, but a reader may flag it as dead. Keep (the docstring states
the swap); a stricter alternative is to drop the param until the onsite gate and add it
then — chose to keep the call-site stable so the onsite change is one line in the builder
body only.

## MISSING / WEAK tests
finals/tests/test_proximity_guard.py: MED TEST (FIXED PRE-REVIEW): mutant (a) `<=`->`>=`
on the land threshold SURVIVES test_proximity_lands_at_hard_stop because that test uses
the boundary value 25.0 (25>=25 still trips). The mutation kill-check is therefore pinned
to test_proximity_land_does_not_latch_on_advisory_state (front=20, expects LAND_THIS;
20>=25 is False so the mutant returns ADVISORY/None -> test FAILS). VERIFIED: mutant
applied, that named test FAILED, reverted clean.
finals/tests/test_proximity_guard.py: LOW TEST: the advisory edge-latch re-arm is covered
(approach -> 1 advisory -> hold silent -> clear -> re-approach -> advisory again), but the
re-arm via the ALL-CLEAR branch (every direction None -> closest_cm is None ->
self._warned=False) versus the re-arm via the >warn_cm branch are two distinct code paths;
confirm both are exercised. Checked: test feeds an all-None reading between approaches AND
a >warn reading in separate tests. Adequate.
finals/tests/test_depth.py: LOW TEST: distance_at out-of-bounds is tested (cx past width,
cy past height, negative) but the ragged-row path (data_m[cy] shorter than width, caught
by the IndexError/TypeError guard) is covered only indirectly via the no-return test. The
try/except around float(data_m[cy][cx]) is defensive for a malformed real backend map; a
direct ragged-map test would pin it. LOW value (FakeDepthSource never produces ragged
maps); noted.

## CODE-OPTIMIZATION PASS
finals/guards.py:ProximityGuard.check: INFO OPT [hot path, ~per-agent-tick]: items_cm()
allocates a fresh 4-tuple of 2-tuples each call and the loop walks all four directions
every tick. Negligible (4 elements, ~10 Hz); the allocation buys the stable iteration
order and the explicit per-direction labels in the trip reason. No change.
finals/vision/depth.py:FakeDepthSource.read: INFO OPT: read() deep-copies the latest map
(`[list(row) for row in data]`) every call to honour the latest-copy contract (a consumer
must never mutate the source's live map). Correct and required; the maps are tiny in
tests. No change.

## CONVENTIONS (mechanical)
finals/guards.py: PASS: no bare except / no except Exception added; ProximityReading is a
frozen dataclass (read-only snapshot, mirrors GuardContext); ProximityGuard holds only the
_warned edge latch as per-instance state (fresh instance per drone, like every Guard); the
constructor uses the shared _check_threshold gate + a loud land<warn ValueError; trip
reasons carry WHAT (IR proximity <dir> at <cm>) / WHICH (gctx.drone_id) / MEASURED-vs-LIMIT
(<cm> vs warn/land) / CHECK (the lane / the wiring); units in names (_cm); math.isfinite
guards every range; no while-loop.
finals/vision/depth.py: PASS: PURE module — numpy is type-only under TYPE_CHECKING; no
top-level cv2/pyrealsense2/numpy; the RealSense backend is a reference COMMENT (rs.* never
imported); FakeDepthSource pacing thread is bounded by stop_event (no unbounded loop); stop
is idempotent and never raises (prints a warning on a stuck thread); start raises
SensorTimeout loudly; distance_at fails to None on out-of-bounds / 0 / non-finite (never a
fabricated range).
finals/flight/proximity.py: PASS: PURE module — no pyhulax import (api injected);
SyntheticProximitySensor default reading=None -> honest skip; PyhulaxProximitySensor.read
raises NotImplementedError pointing at module_map.md + the onsite gate (can never be
silently half-wired); constructor validates drone_id + reading type loudly.
finals/config.py / finals/mission/agent.py / finals/main.py: PASS: proximity_* keys added
to the guards optional-key list AND to _validate_guards (loud bool + _num + ordering);
agent proximity_fn added to the hook-callable validation loop; main builds ProximityGuard
only on proximity_enable and the depth source only on depth_backend != "none" (degrade-
absent — "none" path byte-for-byte unchanged); resolve_depth_source_cls is called in run()
so an unknown depth_backend dies loudly at resolution, before anything arms.
