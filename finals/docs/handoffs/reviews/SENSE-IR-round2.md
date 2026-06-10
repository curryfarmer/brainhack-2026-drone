# SENSE-IR round 2 — adversarial re-audit + new gaps

Re-audit of round 1's findings and a fresh pass for gaps round 1 missed. Full suite 1372
green; 3/3 mutants killed (a: land-threshold flip; b: absent-depth raises; c: ladder swap).

Format: `path:line: <SEV> <tag>: <problem>. <fix>.`

## ROUND 1 FOLLOW-UP (what changed)
finals/vision/depth.py: FIXED: round 1's dead-import note. `SensorError` was imported and
never used -> removed (only `SensorTimeout` is used). The TYPE_CHECKING `import numpy as np`
is kept (documents that a real backend's data_m is a numpy array) and explicitly marked
`# noqa: F401` so it reads as intentional type-only, not an oversight. depth.py re-imports
clean; 20 depth tests still green.
finals/main.py:_build_proximity_fn api=None: KEPT (round 1 LOW SMELL): re-confirmed the
unused `api` param is the documented onsite seam (the docstring states the swap to
PyhulaxProximitySensor(drone.id, api)). Keeping the call-site signature stable means the
onsite change is one line in the builder BODY. Acceptable; no action.

## NEW GAPS (fresh pass)
finals/main.py:_amain depth lifecycle: INFO: re-read the optional depth path. `depth_sources`
is built per (source, perception) pair using source.source_id, started in the SAME
preflight-skipped branch as RGB sources (so bench/real, where preflight owns source.start,
do NOT double-start — depth has no preflight step today so it starts here only on
mock/sitl/replay-style runs where the RGB start loop also runs). Confirmed: the depth start
is INSIDE the `if not _preflight_profile` guard alongside the RGB start, and stop() is
unconditional in finally (idempotent). With depth_backend "none" the list is empty -> the
whole block is a no-op. Correct; no fix. NOTE for a future depth consumer: bench/real would
need a depth start hook in preflight (P6-adjacent) — flagged for that day, out of scope now.
finals/vision/depth.py:_normalize: LOW: a single map whose first ROW is empty (`[[]]`) takes
the `len(first) > 0` False branch -> treated as a list-of-maps, producing `[[]]` as one
0-wide map. Degenerate either way (a 0-row or 0-col map yields width/height 0 and distance_at
always None). Not a real input (FakeDepthSource callers pass real maps); the no-return
default `[[[0.0]]]` covers the empty case. No fix — documented degenerate.
finals/guards.py:ProximityReading: INFO: the dataclass carries `ts` but ProximityGuard.check
never reads it (only the four ranges). Intentional forward-compat: a future
ProximityStaleGuard (mirroring VideoWatchdog) reads ts; the synthetic/pyhulax sensors already
stamp it. No dead-field risk (it is part of the published reading contract). No fix.

## MUTATION KILL-CHECK (re-verified this round)
(a) finals/guards.py: flip `closest_cm <= self._land_cm` -> `>=`. KILLS
    test_proximity_land_does_not_latch_on_advisory_state (front=20 -> expects LAND_THIS;
    20>=25 False -> mutant returns ADVISORY). Applied -> FAILED -> reverted clean.
(b) finals/main.py: resolve_depth_source_cls "none" raises instead of returning None. KILLS
    test_build_depth_none_is_a_clean_noop (depth_backend "none" must NEVER raise). Applied ->
    FAILED with ConfigError -> reverted clean.
(c) finals/guards.py: run the advisory (warn) rung BEFORE the land rung so an advisory masks
    a co-fired land. KILLS test_proximity_ladder_ordering_land_beats_advisory (front=35 +
    left=20 -> expects LAND_THIS; mutant returns ADVISORY). Applied -> FAILED -> reverted
    clean (via git, which required reconstructing the uncommitted guards.py edits — done and
    re-verified: 62 SENSE-IR + full 1372 green).
=> 3/3 mutants killed.

## CONVENTIONS (re-checked, mechanical)
PASS across guards.py / vision/depth.py / flight/proximity.py / config.py / agent.py /
main.py: no bare except / no new except Exception; pure modules keep numpy/cv2/pyrealsense2
top-level-free (TYPE_CHECKING + reference-comment only); every threshold is a config/ctor
param (onsite = tune config, not code); fail-loud reasons carry WHAT/WHICH/WHY/CHECK; the
degrade-absent default (depth "none" / proximity_enable False) leaves the monocular,
non-IR mission path byte-for-byte unchanged; unknown depth_backend dies at run()
resolution before anything arms; the LIVE pyhulax IR read is an explicit NotImplementedError
ONSITE GATE (can never be silently half-wired).

## VERDICT
Clean. No remaining MED/HIGH findings; the LOW/INFO items are documented intentional
choices (api seam, ts forward-field, degenerate empty-map). Ready to commit to the
sense-ir worktree branch (no merge).
