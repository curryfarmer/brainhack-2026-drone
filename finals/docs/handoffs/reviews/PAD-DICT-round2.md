# PAD-DICT — adversarial self-review, round 2

Re-audit of round 1 + new-gap hunt. Format unchanged.

## Round 1 re-audit

- aruco.py:135-147 (setattr catch list) — CONFIRMED REFUTED. Re-checked on cv2
  4.11: a whitelisted key always EXISTS (AttributeError impossible), and a bad
  VALUE raises `cv2.error` (str into int field) which IS caught; a float into an
  int field silently truncates (cv2 behavior, not our bug). No realistic failure
  reaches the gap. The catch list is correct for the whitelisted-key contract.

- qr branch ignores marker_dict / aruco_detector_params — CONFIRMED but
  NEUTRALIZED. New evidence (round 2): `config._validate` runs the marker_dict
  membership check AND the aruco_detector_params key whitelist UNCONDITIONALLY
  (regardless of marker_backend), so a `marker_backend:"qr"` config with a bad
  marker_dict OR a typo'd aruco param STILL dies at load. Verified by a live
  load_config probe (both raise ConfigError). The only un-gated path is a direct
  `make_marker_detector("qr", marker_dict=bad)` call, which no production code
  makes. Downgraded to LOW/accepted.

- import-time guards RAISE not assert — CONFIRMED. Verified `python -O -c "import
  finals.vision.aruco"` imports clean and `_resolve_marker_dict('DICT_7X7_1000')`
  returns 15 under -O (the guard is a real `if/raise`, not stripped).

- strict marker_dict membership + params key whitelist at config load —
  CONFIRMED green (the two parametrized loud-tests).

- 13 sim-config pins + 4 real-config 7x7 defaults — CONFIRMED via load_config
  sweep + full suite. `test_dyn_assignment_config` loads two of the pinned
  configs (sitl3_dyn3/5_vision) and stays green, exercising the pins.

- in-test config pins (replay_e2e, vision_wiring) + monkeypatch widening —
  CONFIRMED green.

- DEFAULT_MARKER_DICT single source — CONFIRMED. Mutant (a) re-run against the
  refactored constant still KILLS `test_default_marker_dict_is_the_real_field`.

## New gaps hunted (round 2)

- aruco.py:286 default arg `marker_dict: str = DEFAULT_MARKER_DICT`: LOW OPT:
  using an imported module constant as a default-arg value is evaluated once at
  function-def time — correct + idiomatic; not a mutable-default trap (it's a
  str). No issue. CHECKED, clean.

- preflight.py:235 detail f-string: LOW CONV: the `_p2_perception` detail now
  appends ", dict <name>" ONLY for aruco (the conditional guards the qr case
  where marker_dict is irrelevant). Verified the conditional renders correctly
  for both backends (aruco shows the dict, qr omits it). `data["marker_dict"]`
  is always populated for observability. CHECKED, clean.

- config.py `DEFAULT_MARKER_DICT` self-consistency assert: LOW CONV: a bare
  `assert DEFAULT_MARKER_DICT in VALID_MARKER_DICTS` on two adjacent literals —
  could be stripped under -O, but it is a constant-vs-constant typo guard that
  can only fail if a developer edits one literal and not the tuple; harmless if
  stripped (the strict load-time membership check would then reject the default
  at the first config load anyway). ACCEPTED as a dev typo-guard.

- VALID_ARUCO_PARAM_KEYS coverage: MED→checked: the whitelist excludes the
  AprilTag-only fields and the read/writeDetectorParameters BOUND METHODS (a
  setattr on a bound method would shadow it — correctly excluded). It INCLUDES
  the contrast/threshold/perimeter knobs the low-contrast 7x7 beacons need
  (adaptiveThresh*, minMarkerPerimeterRate, minOtsuStdDev, errorCorrectionRate,
  maxErroneousBitsInBorderRate, perspectiveRemove*). Onsite gate F may want a
  field not in the set; ADDING one is a one-line config.py edit (documented in
  the constant's comment). The import-time guard proves every listed name is a
  real settable DetectorParameters attr on the deployed cv2. CHECKED, clean.

- module docstring line "detect_aruco: cv2.aruco DICT_6X6_250 (the default)":
  MED CONV→FIXED in round 1 (rewrote the docstring to say "CONFIGURABLE
  dictionary ... real-field default DICT_7X7_1000"). Re-verified the docstring no
  longer claims a 6x6 default anywhere. CONFIRMED.

- No depth/SENSE-IR or other Step-0 field touched: CHECKED — only marker_dict +
  aruco_detector_params (the two fields PAD-DICT owns) had their validators
  changed; depth_backend / Sighting.source / FrameStamped.depth /
  ArenaMap.markers untouched (other sessions own them).

## Shared-file note (for hand-merge)
- `finals/config.py`: I added `VALID_MARKER_DICTS`, `VALID_ARUCO_PARAM_KEYS`,
  `DEFAULT_MARKER_DICT` (new module constants) and REPLACED the Step-0
  placeholder marker_dict validator + the aruco_detector_params validator (both
  fields are PAD-DICT-owned per the brief). No other session's fields touched.
- `finals/main.py` `_build_perception` + `finals/preflight.py` `_p2_perception`:
  threaded the two kwargs through the single make_marker_detector call site each.
  Other sessions touching these files edit disjoint hunks.
- `finals/vision/aruco.py`: PAD-DICT owns this file (the dict fix is its core).
- `finals/tests/test_orchestrator.py`: widened ONE monkeypatch lambda signature.

## Mutation kill-check — 3/3 mutants killed
(a) default->6X6 KILLS test_default_marker_dict_is_the_real_field;
(b) resolver returns fixed dict KILLS test_seven_by_seven_field_markers_decode_under_7x7;
(c) params whitelist dropped KILLS test_resolve_detector_params_typo_key_is_loud
    + test_make_marker_detector_threads_params_typo_is_loud.
All reverted clean (diff vs backup == empty).

## Verdict
Clean round. Suite: 1332 passed (cv2) / 1293 passed + 12 skipped (cv2-less).
Conventions green. -O import clean. No open CRIT/HIGH.
