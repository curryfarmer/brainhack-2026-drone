# PAD-DICT — adversarial self-review, round 1

Scope reviewed: `finals/vision/aruco.py` (dict resolver + params whitelist +
threading), `finals/config.py` (VALID_MARKER_DICTS / VALID_ARUCO_PARAM_KEYS /
DEFAULT_MARKER_DICT + strict validation), `finals/main.py` + `finals/preflight.py`
callers, the 13 pinned sim configs, and the new/modified tests.

Format: `path:line: <SEV> <tag>: <problem>. <fix>.`

## Findings

- aruco.py:135-147: MED BUG: `_resolve_detector_params` catches
  `(cv2.error, TypeError, OverflowError)` on `setattr`, but on cv2 4.11 a
  whitelisted-but-WRONG-TYPE value can also raise a bare `ValueError`
  (e.g. a float into an int field on some builds) or `AttributeError` (never,
  since the key is whitelisted). The whitelist guarantees the attr EXISTS, so
  the only realistic failure is a type/range error. Audited: on this cv2,
  setattr of an int field with a str raises `cv2.error`; a float into an int
  field SUCCEEDS (truncates). So no value reaches the gap in practice — but the
  catch list is the contract. Decision: keep as-is; the whitelist makes
  AttributeError impossible and cv2.error covers the type rejection. Documented
  in the docstring. REFUTED as a real bug after audit; left a note.

- make_marker_detector / aruco.py:286,326: MED CONV: when `backend == "qr"`,
  `marker_dict` and `aruco_detector_params` are silently IGNORED (aruco-only
  knobs). A config with `marker_backend:"qr"` + `aruco_detector_params:{...}`
  would drop the tuning with no warning. Matches the existing `save_dir`
  aruco-only behavior + the docstring says "ArUco only", and QR is the
  unconfirmed-dead alternate. Fix deferred: documented as aruco-only; not worth
  a cross-knob guard for a near-dead backend. ACCEPTED as documented limitation.

- aruco.py:87-95: LOW CONV: import-time guards now RAISE ConfigError (not
  assert) so `python -O` cannot strip them. Good. But they run a
  `cv2.aruco.DetectorParameters()` construction at import — a tiny cost paid once
  per process when aruco is imported (already lazy/gated). Acceptable.

- config.py:`_validate` marker_dict block: HIGH TEST→covered: strict
  VALID_MARKER_DICTS membership replaces the Step-0 non-empty-string placeholder.
  A typo ('DICT_7x7_1000'), a non-cv2 name ('DICT_8X8_250'), empty, an int, and
  null all now die at load. Pinned by `test_marker_dict_unknown_is_loud`
  (parametrized x5). CONFIRMED green.

- config.py aruco_detector_params block: HIGH TEST→covered: the per-KEY
  whitelist check is added at config-load (pure, no cv2), so a typo'd
  DetectorParameters field dies on the ground at --dry-run, not as a silent
  no-op mid-flight. Pinned by `test_aruco_detector_params_typo_key_is_loud`.
  The aruco-side `_resolve_detector_params` re-checks (defense in depth, and the
  detector-side backstop for hand-built calls). CONFIRMED.

- Two-layer whitelist (config + aruco): LOW OPT: the param-key whitelist is
  checked twice (config._validate purely, then aruco._resolve_detector_params
  with cv2). Intentional: config is the ground gate for the JSON path; aruco is
  the backstop for `detect_aruco(...)` / direct calls that skip load_config.
  Not redundant. ACCEPTED.

- sim configs (13 pinned): HIGH BUG→avoided: the committed fixtures + sim world
  assets are 6x6; the new default is 7x7, so EVERY config that decodes sim
  assets needs `marker_dict:"DICT_6X6_250"`. Pinned: mock_gazebo, replay,
  sitl, sitl_vision, sitl3_vision, sitl1_followbox1, sitl1_followbox_multi,
  sitl3_dyn3_vision, sitl3_dyn5_vision, sitl1_landing, sitl3_landing,
  sitl3_lanes_vision, sitl3_track_vision. The REAL-field configs (landing_real,
  convoy_real, real, bench) correctly KEEP the 7x7 default. Verified by a
  load_config sweep (real=7x7, sim=6x6). CONFIRMED.

- in-test configs: HIGH BUG→avoided: `test_replay_e2e.write_replay_config` and
  `test_vision_wiring.test_mock_flight_with_replay_frames_end_to_end` build
  configs inline (NOT from disk) and decode the 6x6 fixtures — both now set
  `marker_dict:"DICT_6X6_250"`. `test_vision_aruco.py` legacy decode calls pass
  `marker_dict=SIM_DICT`. CONFIRMED green.

- test_orchestrator.py:420: HIGH BUG→avoided: the `make_marker_detector`
  monkeypatch lambda had signature `(backend, save_dir=None)`; preflight now
  calls it with `marker_dict=` + `aruco_detector_params=`. Widened to
  `(backend, **kw)`. CONFIRMED (test_main_*_preflight_only green).

- DEFAULT_MARKER_DICT single-source: MED OPT→fixed: the function defaults in
  aruco (`detect_aruco`, `make_marker_detector`) and the config field default
  were three copies of the literal "DICT_7X7_1000" — drift risk. Now all
  reference `config.DEFAULT_MARKER_DICT`. Mutant (a) flipping it kills both the
  detector-default test AND the config-default test. FIXED.

## Mutation kill-check (run, named test FAILS, reverted clean)
- (a) DEFAULT_MARKER_DICT / make_marker_detector default flipped to 6X6 ->
  `test_default_marker_dict_is_the_real_field` FAILS. KILLED.
- (b) `_resolve_marker_dict` returns a fixed DICT_6X6_250 ignoring config ->
  `test_seven_by_seven_field_markers_decode_under_7x7` FAILS. KILLED.
- (c) `_resolve_detector_params` whitelist dropped ->
  `test_resolve_detector_params_typo_key_is_loud` +
  `test_make_marker_detector_threads_params_typo_is_loud` FAIL. KILLED.

## Suite
1332 passed with cv2; 1293 passed + 12 skipped with cv2/numpy import-blocked
(the cv2-less competition-laptop bar holds). Conventions test green.
