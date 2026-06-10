# Field ArUco markers — intel + optimisation levers

Intel from the organizers via the user, **2026-06-10**. Captures the STABLE facts
(dictionary, ids, coordinates), the mission mechanism, and where they let us
optimise the pipeline / replace guessed numbers.

**Marker ROLE — RESOLVED (2026-06-10):** each of the 5 markers is a **pad-validity
beacon** sitting NEXT TO a landing pad. The pad itself is a big white **A3 paper with
a logo** (a separate object); the ArUco beside it encodes a number we read to decide
if that pad is valid. Markers and pads are CO-LOCATED but distinct — pad = land
target, ArUco = validity ID. (The earlier "not the pads" was right: they're *beside*
the pads, not the pads.)

## Hard facts (stable)

- **Dictionary: `cv2.aruco.DICT_7X7_1000`** (ArUco, not QR — QR confirmed dead 2026-06-09).
- **5 markers, ids + fixed coordinates** (organizer (x, y) frame, metres; raw-message
  typos resolved: id 45 `1,3`→1.3, id 67 `.1.95`→1.95):

  | id | x (m) | y (m) |
  |----|-------|-------|
  | 11 | 1.35 | 4.40 |
  | 45 | 1.30 | 7.85 |
  | 51 | 4.40 | 4.40 |
  | 67 | 1.95 | 8.70 |
  | 101 | 4.40 | 7.85 |

- These are **STATIC** (fixed coords) ⇒ **NOT the moving RoboMaster convoy**. Whether a
  separate moving-convoy marker set also exists is open.
- Each marker sits **beside a landing pad** (one-to-one with the 5 A3 logo pads); the
  marker is the pad's **validity ID**, not the pad itself.
- **Monocular only — NO depth camera** on the HULA/swarm path (720p RGB, 71° FOV;
  RealSense depth is mapping-challenge hardware, not ours). Range must come from
  monocular cues.
- **Beacon appearance (photo, 2026-06-10):** a 7×7-style ArUco on its OWN white-bordered
  tile (standalone, placed beside the pad), printed **LOW CONTRAST** — gray modules, not
  solid black. cv2.aruco binarizes on contrast, so tune `DetectorParameters`
  (adaptive-threshold window, `minMarkerPerimeterRate`) and confirm decode at gate F; a
  gray-on-white marker is less forgiving than black-on-white.

### Coordinate sanity vs the cage

Cage ≈ **5.3 m × 11.3 m** (narrow × long; user estimate, [arena dims](module_map.md)).
Marker spread: **x ∈ [1.30, 4.40]** ⊂ the ~5.3 m **short** axis; **y ∈ [4.40, 8.70]**
⊂ the ~11.3 m **long** axis. Every marker falls comfortably inside → the coords are
self-consistent with the cage. Markers cluster in the middle of the long axis on one
side — a deliberate layout (pairs share y = 4.40 and y = 7.85).

## Mission mechanism — landing-pad validity (Roboverse)

5 landing pads, each a big white **A3 sheet with an orange-circle logo**, each with a
**corresponding ArUco beacon beside it** (the 5 ids above, at the known coords). The
ArUco decodes to a number.

**Validity is decided at runtime against a dictionary the organizers hand us RIGHT
BEFORE the showcase** (the "safe codes"). Intended flow:

1. **Assign** each pad to its ArUco by **distance** (nearest beacon). Euclidean adjacency
   is the natural metric — each pad's beacon is right beside it; A* path distance is
   overkill unless an obstacle sits between a pad and its own beacon. We have both
   (visibility_graph A* + trivial euclidean).
2. **Read** the ArUco → its number.
3. **Validate**: number ∈ safe-codes dict? This is EXACTLY our existing
   `zone.land_on_pad.valid_marker_ids` knob — the pre-showcase dict drops in there
   last-minute (placeholder → real list). `land_on_pad` already refuses to commit on a
   marker id ∉ valid_marker_ids, so an out-of-dict pad auto-rejects.
4. **Invalid pad → broadcast to all drones** so nobody re-checks or wastes battery on it.

## Landing-pad detection — white-paper-first colour match (no CV model)

**Photo intel (2026-06-10):** the pad logo is the **Roboverse roundel** — dark-red outer
ring, red-orange fill, dark hexagon centre, "ROBOVERSE" text — on a big white A3 sheet.
⚠️ **The arena FLOOR IS ORANGE** (map photo), so you CANNOT threshold "orange" alone —
the floor matches everywhere. The **white A3 paper is the real signal** (bright white
pops hard against the orange floor). User's white-surround instinct is now *mandatory*,
not optional.

Revised recipe (cheap, deterministic, no training):
1. **Find the white A3 rectangle** — high value, low saturation blob on the orange floor.
   The strongest, most lighting-stable cue.
2. **Confirm a red-orange roundel inside it** — segment red+orange (hue near 0–15, high
   saturation), check it's a roughly circular region centred in the white rect. Separates
   a PAD (white + red roundel) from the BEACON tiles (white + gray ArUco, no red) and the
   blue icon cards (white + blue).
3. **Reject distractors** — the same roundel printed on obstacles has **NO white A3**
   around it → fails step 1. (User's key insight, now confirmed critical *because the
   floor itself is orange*.)
4. **Centroid of the white rect / roundel = servo target** — reuses the `_servo`
   pixel-offset-to-Move math, just a different target source than the marker.

- Robust to floor clutter (red arrows, blue tape/cards, black/yellow obstacles, B/W
  ArUco tiles) — none are "white rect + red roundel".
- Cautions: calibrate red/orange AND white HSV vs the real pad and the orange floor at
  gate F — the floor orange and the logo orange may be close in HUE, so lean on
  **saturation/value + the white border**, not hue alone; the down-cam at altitude may
  not frame the whole A3 → test the LOCAL white↔roundel boundary; some floor pads show a
  yellow/green hex outline decal (purpose unconfirmed).

**Two-factor land commit:** (white-A3 + red roundel detected) AND (adjacent beacon id ∈
valid dict). Pad detector says WHERE to land; beacon says WHETHER it's allowed.

## Arena layout + obstacles (map photo, 2026-06-10)

The arena is an **obstacle course on an ORANGE floor** (netted cage), NOT a flat open
search box. Visible elements:

- **Arches / gates** — tall black gate frames with green/yellow hazard striping (the
  "arches"). **Cannot be overflown** (too tall vs our low ceiling) → fly THROUGH the
  opening or route around the legs. ⚠️ A naive 2-D keep-out around an arch would WALL OFF
  a passage that is actually flyable — the planner must model the arch **legs** as
  keep-outs and the **gap** as free (a gate, not a solid block). New nav consideration.
- **Pillars / columns** — tall thin black posts with hazard stripes → simple keep-outs.
- **Low barriers / blocks** — gray/black low walls on the floor → keep-outs.
- **Cones** — yellow/green; small keep-outs.
- **Floor markings to IGNORE**: big **red chevron arrows**, **blue/purple zig-zag tape**,
  small **blue icon cards** (lightning / node symbols — purpose unknown), and **B/W
  square papers that are NOT ArUco**. None decode as markers and none are "white-A3 + red
  roundel", so BOTH detectors ignore them by construction.
- **Pads + beacons** sit on the floor as white tiles (pad = red roundel, beacon = gray
  ArUco), beacon beside pad; some pads show a yellow/green hex outline decal.

Implications: (1) the orange floor makes the **white-paper-first** pad detector mandatory
(above); (2) **arches add a fly-through + likely vertical constraint** our open-loop
horizontal planner does not model yet — flag for the nav rework; (3) hand-mapping the
keep-outs (arch legs, pillars, barriers, cones) from an overhead photo is the realistic
path (the `map_sensing.keep_outs_from_overhead_corners` lever); the 4-dir IR (30–50 cm)
is the only onboard backstop.

### What changes in our code (landing)

- ✅ Validity check — `valid_marker_ids` already models the safe dict.
- ✅ Route-between-obstacles — visibility_graph A* already does this.
- ✅ **Pad locator (YOLO path) — DONE (PAD-DETECT)** — `land_on_pad` can now servo on a
  YOLO pad-class bbox instead of the marker. Set `zone["land_on_pad"]`:
  `{"servo_on": "pad", "pad_classes": ["landing_pad"]}` and point the EXISTING
  `detector` backend at the pad weights (below). Default `servo_on="marker"` keeps the
  legacy beacon-bbox servo, so nothing changes until the knobs are set. (A colour
  orange-on-white detector emitting `source="pad"` is a SEPARATE, still-open path — it
  is NOT picked by the YOLO pad servo.)
- 🟥 **Cross-drone validity sharing is NEW** — a shared pad-validity/claim map in the
  orchestrator (built from sightings, fed to agents) so pads are read ONCE and no two
  drones chase the same/invalid pad. Own session; not done.

### YOLO landing-pad WEIGHTS + CLASS contract (PAD-DETECT — onsite/data, not code)

The pad locator is the EXISTING `ultralytics`/`canned` detector backend running a
USER-TRAINED model (a `.pt` data artifact, like the convoy `best.pt`). The model is NOT
this session's job; the pipeline that consumes it is. Contract the model + config MUST
satisfy for `land_on_pad servo_on="pad"`:

1. **Class** — the model emits ONE class for the landing pad (the white-A3 + orange
   roundel blob). Recommended canonical `class_name`: **`landing_pad`**. If the raw model
   label differs, map it via `detector.class_map` (`{"<raw>": "landing_pad"}`) so the
   published `Sighting.class_name` matches a member of `pad_classes`.
2. **Wiring** — `detector.backend "ultralytics"` + `detector.weights "<pad>.pt"`; the
   PerceptionLoop publishes each detection as
   `Sighting(source="yolo", class_name=<mapped>, marker_id=None, bbox_xyxy=<box>)` (the
   exact shape `land_on_pad._pad_sightings` filters on).
3. **Phase** — `zone["land_on_pad"]: {"servo_on": "pad", "pad_classes": ["landing_pad"]}`;
   the bbox the model emits IS the servo target (tighter/centred boxes land tighter).
4. **Two-factor commit** — the beacon ArUco still drives VALIDITY (PAD-VALID's
   `valid_marker_ids` predicate). Pad detector says WHERE to land; beacon says WHETHER.
5. **Gate F (onsite)** — train/verify the model on the real pad + lighting, then calibrate
   the pad-servo `k_lateral` / `commit_alt_m` against the down-cam pad-blob pixel size.

## ⚠️ Required code change (independent of everything else)

`finals/vision/aruco.py:223` hardcodes `cv2.aruco.DICT_6X6_250`. **A 6×6 detector
cannot read 7×7 markers** — left as-is we detect **nothing** on the real field
(campaign-critical). Fix:

1. Add config knob `marker_dict: str = "DICT_7X7_1000"` (the real default) in
   `config.py`, with a name→`cv2.aruco` constant resolver (loud `ConfigError` on
   unknown, same pattern as `marker_backend`).
2. Thread it through `make_marker_detector(backend, *, marker_dict=..., save_dir=...)`
   and its callers (`main` perception wiring, `preflight.py:229`, `detect_aruco`).
3. **Keep the sim green**: sim assets (`sim/gen_markers.py`, the committed world
   textures, `finals/tests/fixtures` frames) are DICT_6X6_250. Set
   `marker_dict: "DICT_6X6_250"` in the sim vision configs (`sitl*_vision.json`,
   `sitl*_landing.json`) and the cv2-gated fixture tests — OR regenerate those assets
   as 7×7 (more modules ⇒ higher decode-px floor; the 720p real cam absorbs it, the
   640 px sim is tighter). Per-repo bar: full suite green + mutation kill-check before
   commit.

## Optimisation levers (where the known coords pay off)

### L1 — Adopt the organizer (x, y) frame as our canonical arena frame
Our planner/dead-reckoner are frame-agnostic `(north_m, east_m)`. Bind it ONCE to the
organizer axes (proposal: **north ← y (long axis), east ← x (short axis)**; confirm
the compass/origin onsite) and express bounds, keep-outs, and any goals in the SAME
published coords as the markers. Kills a whole class of remap/translation bugs — the
marker coords arrive in this frame for free.

### L2 — Fix the hardcoded bounds (the "how does this help our bounds" answer)
Today `configs/arenas/sample.json` `bounds_m` is a **guess** (12×10, footlength
estimate ~5.3×11.3 — both approximate). The markers give **exact, measured interior
points**, which:
- **Pin scale + orientation + origin** far better than the footlength guess — real
  surveyed metres beat "≈1 ft × 17.5".
- Impose a **hard containment constraint**: `bounds_m` MUST enclose all 5 markers
  (+margin) → east(x) extent ≥ 4.40, north(y) extent ≥ 8.70.
- Let bounds **validation become ground-truth**: after setting the cage rectangle,
  assert all 5 marker coords land inside (they do: 4.40 < ~5.3, 8.70 < ~11.3).

They do **not** alone fix the full rectangle (markers are interior + clustered), so
the cage extent still needs the tape/photos — but origin, axes, and scale are now
anchored. **Recommended:** arena frame = organizer (x, y); `bounds_m` = measured cage
rectangle in that frame (~`[0, 0, 11.3, 5.3]` as `[n_min,e_min,n_max,e_max]`,
onsite-confirmed); markers as interior ground-truth checks. Do NOT blind-scale the
existing sample pads/keep-outs.

### L3 — Absolute position fix → reset open-loop dead-reckon drift (highest nav value)
Our landing nav is **position-blind** open-loop compass DR (`PositionQuality.NONE`),
with ~0.46 m drift bought back only at the very end by the visual servo. A marker at a
**known** world coord, seen by the down-cam, is an **absolute position fix**:

> world_pos ≈ marker_xy − (ground offset of the marker from image centre)

and the ground offset is `pixel_offset × altitude × tan(HFOV/2)/(w/2)` rotated by yaw
— **the `_servo.pixel_offset_to_move` similar-triangles math we already have. No depth
needed** (altitude from telemetry + the 71° FOV). Degenerate, simplest case: centre
the marker under the drone → `world_pos = marker_xy` exactly.

This is the `mission/planning/map_sensing.py::position_fix_from_marker` **stub** — the
5 ids + coords are exactly the anchor table it needs. Implementing it converts
open-loop DR into **drift-corrected** nav: near-true waypoint flight, far tighter
landings, recovery from yaw/scale creep mid-transit. **Gated on** the markers being
floor markers visible looking DOWN at our flight heights (open Q).

## Open questions (need user / cage photos)

1. **Frame origin + axis orientation** — which cage corner is (0,0), which way do +x/+y
   point, where does C2 boot relative to them? (Sets the L1 binding.)
2. **Beacon↔pad offset** — beacon coords are known; where is each A3 pad relative to its
   beacon (spacing + bearing)? Sets the servo target and confirms the distance-assignment.
3. **Floor or elevated?** — down-cam read + position-fix (L3) need beacons + pads visible
   looking DOWN at flight altitude.
4. **Physical sizes** — beacon still ~20 cm? pad A3 (~0.30 × 0.42 m)? Sets decode range,
   the L3 altitude scale, and the expected orange-blob pixel size.
5. **Pre-showcase dict timing** — handed over BEFORE our flight (→ pre-compute pad
   assignment on the ground) or only at showcase (→ in-flight read+share mandatory)?
6. **Separate moving convoy?** — do the 5 RoboMaster robots carry their own marker set,
   or is the mission now pad-validity-only?
7. **Exact orange/white shades + venue lighting** — for HSV calibration; the example pad
   image will pin these.

Trackers: open-questions in [`module_map.md`](module_map.md); cage size in
[`../../docs/finals/README.md`](../../docs/finals/README.md) §1; HULA camera spec §2.0.
