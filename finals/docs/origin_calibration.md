# Origin calibration — Challenge-2A LANDING (gate D)

**Goal:** pin the cage-frame origin + heading so the open-loop dead-reckon
transit reaches the **hardcoded** landing pads.

## What changed (post-simplification, 2026-06-10)

The valid/invalid-pad distinction is **gone**. Every pad is landable; there is
**no per-pad ArUco beacon**. We pick the pad coordinates we want and the drone
flies there open-loop, then the YOLO pad detector (`models/pad_v1.pt`) centres
the touchdown. So calibration is a **physical-placement + open-loop-integration**
problem, not a sensor fix:

- **Origin = a cage CORNER.** `(0,0)` = the corner. `+north` along the LONG wall
  (~11.3 m), `+east` along the SHORT wall (~5.3 m). Bounds, pad centres, and the
  launch point are all measured from that corner, in metres.
- **Heading = aim-the-fleet + a configurable offset** (`heading_offset_deg`,
  default `0.0`). Robust whether HULA yaw is absolute-magnetic or
  relative-to-boot.
- **Rough is fine.** ~0.5 m of dead-reckon drift over a transit is bought back by
  the YOLO pad-centring stage. No survey-grade precision needed.

> Obstacles / keep-outs are the **next** step — origin first. `cage.json` ships
> with `keep_out: []`; `calibrate_origin.py` already accepts obstacles when you
> add them.

## The frame

```
        +north (long wall, ~11.3 m)
          ^
          |     o pad
          |        o pad
          |   * C2 (launch, c2_origin_m)
  (0,0)   +-----------------> +east (short wall, ~5.3 m)
  corner
```

`ArenaMap` fields (in `finals/configs/arenas/cage.json`):

| field | meaning |
|---|---|
| `bounds_m` | `[north_min, east_min, north_max, east_max]` = `[0, 0, LONG_m, SHORT_m]` |
| `c2_origin_m` | `[north_m, east_m]` of the launch spot, measured from the corner (must be inside bounds) |
| `c2_heading_deg` | arena heading the fleet boots facing (deg, CCW+, 0 = +north). **Advisory** (rotates Discord coords); NOT the navigate offset |
| `heading_offset_deg` | **Δ** = the compass-yaw-vs-arena-north misalignment (deg, CCW+). navigate adds it to every leg's Rotate target. Default `0.0` |
| `pads[]` | the chosen landing coords (`center_m`, `radius_m`, `valid:true` for all) |

## Onsite steps (gate D)

1. **Mark the corner + axes.** Pick the cage corner = origin. Tape the `+north`
   (long wall) and `+east` (short wall) directions. Confirm they are square.
2. **Measure the rectangle.** Long wall → `north_max`; short wall → `east_max`.
   Set `bounds_m = [0, 0, north_max, east_max]`.
3. **Measure each pad centre** from the corner → `pads[].center_m` (north, east).
   Pick which pads each drone lands on (these are the `pad_id`s the
   `landing_real.json` navigate zones name: `pad_north` / `pad_se` / `pad_mid`).
4. **Measure the launch spot** from the corner → `c2_origin_m`. Place the drones
   there.
5. **Aim + read yaw (set Δ).** Line every drone up along the SAME boot direction.
   Decide which arena heading that direction is (e.g. `+north` = 0°). Read the
   compass/telemetry yaw it reports while aimed:
   - **Δ = `boot_yaw_reading − arena_heading_aimed`.**
   - If HULA yaw is relative-to-boot **and** you aimed at `+north`, the reading is
     0 → **Δ = 0** (leave it).
   - If the fleet boots facing, say, `+east` (arena heading −90°) but the compass
     reads 0 there, then Δ = `0 − (−90) = +90`.
   - Set `heading_offset_deg = Δ` in `cage.json`.

   > navigate then targets `arena_heading + Δ` so the nose physically points along
   > the arena heading even when the sensor frame is rotated.
6. **Regenerate + check.** Run the helper:
   ```
   python -m finals.tools.calibrate_origin finals/configs/arenas/cage.json \
       --save runs_finals/cage_topdown.png
   ```
   It **validates** the arena through the real loader (bad numbers die here), then
   prints a per-drone **calibration card** — straight-line bearing + distance, and
   every leg's arena heading, the **compass yaw to read after Δ**, and the leg
   distance — plus a top-down PNG + an ASCII map. Eyeball the card vs the cage.
7. **Switch the mission onto the cage + dry-run.** The committed `landing_real.json`
   default is `arena_name: "sample"` (the test-fixture map, kept so the suite stays
   green). Flip it to the real cage, then confirm the mission resolves:
   ```
   # in finals/configs/landing_real.json:  "arena_name": "sample"  ->  "cage"
   python -m finals.main --profile real --config finals/configs/landing_real.json \
       --i-know-this-arms-real-drones --dry-run
   ```
   Expect `[takeoff, navigate, land_on_pad]` per drone, no planner error.
8. **Smoke fly** one drone to one pad. Watch the first Rotate settle on the
   card's compass heading, then the Move distance. Adjust Δ if the nose is off.

## Reading the calibration card

```
[alpha] -> pad 'pad_north' (10.0, 2.65)
    straight line : bearing +0.0 deg (read +0.0 on compass), distance 9.00 m
    planned       : 9 leg(s), path 9.00 m
      leg  1: rotate to arena   +0.0 deg  (compass reads   +0.0)   move  1.00 m
      ...
```

- **bearing** = arena-frame heading C2 → pad; **read … on compass** = the yaw the
  drone should show after Δ.
- **planned** legs subdivide the path so open-loop drift stays under the planner's
  inflation margin (the per-leg re-orient re-zeroes yaw creep each leg).
- An `UNREACHABLE` line means a pad is trapped by a keep-out (only once obstacles
  are added) — fix the coord or the inflation.

## Follow-ups (flagged, not in this step)

- **Obstacles / keep-outs** — the explicit next step; survey crate footprints and
  add them to `cage.json` `keep_out` (the helper re-cards around them).
- **Landing servo flip** — `land_on_pad` `servo_on: "marker" → "pad"` +
  `pad_classes: ["landing_pad"]` + launch `--weights models/pad_v1.pt` (the
  post-simplification mission has no per-pad marker to servo on). Needs the
  `test_nav_e2e` bus rewired to emit YOLO pad sightings.
- **Strip valid/invalid** — drop `LandingPad.valid` + per-drone `valid_marker_ids`
  once every pad is valid.

See `finals/configs/arenas/cage.json`, `finals/tools/calibrate_origin.py`,
`finals/mission/phases/navigate.py` (the offset bake), and
`finals/mission/planning/types.py` (`ArenaMap.heading_offset_deg`).
