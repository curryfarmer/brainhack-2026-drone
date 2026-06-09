# NAV verification wave — tracking doc + agent prompts

**Branch:** `nav-landing` @ build-complete (NAV-0..8 merged, 1000 green).
**Purpose:** 2 adversarial review rounds + test-hardening + a code-optimization pass per
nav slice, then a full E2E integration suite, then SITL (NAV-9). Built to resist context
rot: every reviewer writes its FULL report to a file under `finals/docs/handoffs/reviews/`
and returns only a compact summary. The orchestrating session holds summaries + triage
decisions, never the verbose reports.

## Why slices (the anti-rot unit)

Nav is reviewed in 6 self-contained slices of 2-3 modules each. A reviewer (or a fresh
Claude session) takes ONE slice, needs only that slice's files + this doc + module_map +
the plan — not the whole repo. Two rounds per slice; round 2 re-audits round-1 findings
(refutes false positives, finds what round 1 missed).

| Slice | Modules | Tests | Review lens (besides the universal bar) |
|---|---|---|---|
| **S-PLAN** | `planning/types.py`, `planning/polygon_tools.py`, `planning/visibility_graph.py`, `errors.py::PlanningError` | test_arena_skeleton, test_polygon_tools, test_visibility_graph | geometry correctness (sign/winding/inflate direction, > vs >=), A* admissibility/termination, **OPT: this is the only genuinely perf-relevant slice** |
| **S-FRAME** | `planning/frame.py`, `config.py` arena parse+validate | test_frame, test_arena_config, sample.json, mock_arena.json | frame-convention correctness (NED axes, c2 origin/heading, CCW+), fail-loud on every malformed arena |
| **S-SERVO** | `phases/_servo.py` | test_servo | sign conventions LOCKED (a left/right or CCW swap MUST fail a test), deadband/clamp boundaries |
| **S-NAV** | `phases/navigate.py`, `phases/takeoff.py` | test_navigate, test_takeoff_phase | open-loop transit state machine, absolute-heading re-zero, budget/stuck bounds, **OPT: step() is per-tick hot path** |
| **S-LAND** | `phases/land_on_pad.py` | test_land_on_pad | ACQUIRE→CENTER→DESCEND→COMMIT machine, never-battery-dead-hover guarantee, loss-recovery bounds, **OPT: step() is per-tick hot path** |
| **S-DECON** | `guards.py` (launch slot + SectorGuard), `agent.py` (launch routing), `main.py` (_build_*), `config.py` deconfliction | test_deconfliction, test_landing_config, test_convoy_config, landing_real.json, convoy_real.json | **concurrency: deadlock-freedom, slot release on every path (finally), deadline bounds**, advisory-not-control sector |

## Universal review bar (every slice, both rounds)

Inherit the binding conventions from `finals/docs/module_map.md`:
- No bare except; `except Exception` only at whitelisted sites. Pure modules: stdlib only
  (numpy is FORBIDDEN top-level in pure modules — TYPE_CHECKING pattern).
- Every blocking/awaited op takes a timeout and raises a TYPED `finals.errors` exception
  whose message names WHAT failed / WHICH drone / WHY / what to CHECK.
- Every while-loop references a deadline / stop-event / iteration bound.
- No module-level mutable globals (except PHASE_REGISTRY).
- Units in names (_cm/_m/_deg/_s). Adapter contract is cm; world frame is m.
- Mission logic MUST be correct at `PositionQuality.NONE` (no horizontal XY ever).
- Phases: `step()` does NO planning/allocation-heavy work — planning happens once in
  `from_config`/`__init__`; `step()` runs at tick_hz.

## CODE-OPTIMIZATION PASS (every slice, tagged `[OPT]`)

For each module assess, in this priority order:

1. **Hot path vs cold path.** `step()` runs every tick (tick_hz, ~10 Hz) → hot. `from_config`
   / `plan()` / `__init__` run ONCE per phase → cold. An inefficiency on a cold path that
   runs once over a ≤~20-vertex arena is almost never worth a readability trade.
2. **Algorithmic complexity of hot paths + planner.** Visibility-graph construction is
   ~O(V²) candidate edges × O(V) segment-vs-polygon tests; flag accidental O(V³)+ or
   redundant re-computation. A* open-set should be a heap, not a linear min-scan. Re-planning
   or polygon re-inflation inside `step()` is a bug, not a micro-opt.
3. **Redundant work / per-tick allocation.** Constants recomputed inside loops; the same
   source re-walked; fresh lists/tuples built every tick in `step()`; repeated `wrap180`/
   trig that could be hoisted.
4. **Hard constraint — DO NOT sacrifice these for perf:** never remove a fail-loud check,
   never obscure a typed error, never drop a while-loop bound, never trade readability for
   micro-gains on a cold path. Correctness > fail-loud clarity > performance. The arena is
   tiny, so most "optimizations" here are SCALING notes, not urgent fixes.

Each `[OPT]` finding states: hot/cold path, current vs proposed complexity, est. impact, and
whether it conflicts with a convention (if so → REJECT it and say why).

## Finding severity + output contract (binding on every reviewer)

Write the FULL report to `finals/docs/handoffs/reviews/<slice>-round<N>.md`, one finding
per line: `path:line: <SEV> <tag>: <problem>. <fix>.` where SEV ∈ {CRIT, HIGH, MED, LOW}
and tag ∈ {BUG, CONV (convention), TEST (missing/weak test), OPT}. No praise, no prose,
skip pure formatting nits. Round 2 adds a `VERDICT:` line per round-1 finding
(CONFIRMED / REFUTED-because / DUPLICATE) plus its own NEW findings.

**Return to the orchestrator ONLY:** the slice name, the report file path, counts per
severity, and the CRIT/HIGH one-liners (≤ ~12 lines total). NOT the full report.

## Triage protocol (orchestrator, between batches)

1. Read the returned compact summaries (cheap).
2. For any CRIT/HIGH, open the disk report for just that slice.
3. Fix in the `nav-landing` worktree; keep `python -m pytest finals/tests -q` green after
   every fix.
4. Record the disposition (fixed / wontfix-because / deferred-to-onsite) in the status
   table below. Mutation kill-check any new/changed test per the playbook.

## Reusable reviewer prompt (paste; fill `<SLICE>`, `<MODULES>`, `<TESTS>`, `<ROUND>`)

```text
You are an adversarial code reviewer for the BrainHack-2026 landing-navigation build,
slice <SLICE>, ROUND <ROUND>. Read ONLY what you need: finals/docs/module_map.md (binding
conventions + status table), finals/docs/handoffs/nav-verification.md (this wave's bar,
severity contract, and the CODE-OPTIMIZATION PASS spec), then the slice files:
  modules: <MODULES>
  tests:   <TESTS>
[ROUND 2 only: also read finals/docs/handoffs/reviews/<SLICE>-round1.md and, for each
 round-1 finding, return a VERDICT line: CONFIRMED / REFUTED-because / DUPLICATE, then add
 your own NEW findings that round 1 missed.]

DO (in priority order):
1. Hunt BUGS that survive the existing tests — wrong sign/convention, off-by-one, an
   unbounded loop, a silent default that hides a typo, a fail-loud message that omits
   WHAT/WHICH/WHY/CHECK, a path that can battery-dead-hover or leave a drone airborne, a
   concurrency hazard (slot not released on a throw, a deadlock cycle, a missing deadline).
2. Find MISSING/WEAK tests: a mutation the suite would NOT catch. Name the exact mutant
   (e.g. "flip > to >= at visibility_graph.py:NN") and the test that should kill it.
3. CODE-OPTIMIZATION PASS exactly as specified in nav-verification.md — tag findings [OPT],
   classify hot/cold path, give complexity + impact, and REJECT any opt that would violate
   a convention (say which). This codebase ranks correctness > fail-loud clarity > perf.
4. Verify the conventions mechanically where you can (grep for bare except, top-level SDK
   imports in pure modules, unbounded while, untyped errors).

OUTPUT CONTRACT (binding): write your FULL report to
finals/docs/handoffs/reviews/<SLICE>-round<ROUND>.md, one finding per line as
`path:line: <SEV> <tag>: <problem>. <fix>.` (SEV ∈ CRIT/HIGH/MED/LOW; tag ∈ BUG/CONV/TEST/OPT).
No praise, no prose, skip formatting nits. Do NOT edit any source or test file — review only.
RETURN to me ONLY: slice name, the report file path, counts per severity, and the CRIT/HIGH
one-liners. Keep your returned message under ~12 lines — the full detail lives in the file.
```

## Integration-suite agent prompt (Task #10, separate from the slice reviewers)

```text
You are building the full end-to-end nav integration test suite for the landing mission.
Read finals/docs/handoffs/nav-verification.md, module_map.md, the plan's "End-to-end
verification strategy" table, and the nav phase/planner/config sources. Build (do not weaken
existing tests) finals/tests/test_nav_e2e.py covering, over the MockAdapter phase-stepping
harness with CANNED Sightings:
  1. Single drone: arena.json -> plan -> navigate (legs execute, absolute-heading re-zero
     survives a yaw-drift fixture) -> land_on_pad (ACQUIRE->CENTER->DESCEND->COMMIT->Done),
     asserting DeadReckoner final pose is within the drift budget of the pad center.
  2. 3-drone deconfliction over the orchestrator: time-staggered launch NEVER has 2 drones
     in a launch/landing corridor at once; serialized landing (2 want to descend -> one
     waits); on one drone FAILED, the others finish and emergency_land fires exactly once.
  3. Failure injection: planner-empty -> Abort/ConfigError; budget exceeded -> blind Land
     (never battery-dead hover); marker lost mid-descend -> bounded ascend+retry then abort.
  4. Convoy still reads all ids: convoy_real.json resolves and sentry_scan still surfaces
     the canned ids (regression guard that the landing work did not break 2B).
Every test fail-loud and deterministic. Run python -m pytest finals/tests -q; report
pass count + any new test file paths. Commit nothing — leave changes staged for review.
```

## Status table (orchestrator updates each batch)

| Slice | R1 done | R2 done | CRIT | HIGH | Disposition | Suite |
|---|---|---|---|---|---|---|
| S-PLAN  | ☐ | ☐ | - | - | | |
| S-FRAME | ☐ | ☐ | - | - | | |
| S-SERVO | ☐ | ☐ | - | - | | |
| S-NAV   | ☐ | ☐ | - | - | | |
| S-LAND  | ☐ | ☐ | - | - | | |
| S-DECON | ☐ | ☐ | - | - | | |
| E2E suite | ☐ | — | - | - | | |

## NAV-9 (SITL, after the table is green) — see the plan's NAV-9 spec; run on `ssh bhvm`.
