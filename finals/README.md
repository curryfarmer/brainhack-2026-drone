# Our Finals Strategy — in plain terms

*BrainHack 2026 finals, swarm challenge. This is the "why" behind the code in
this folder. For the "what to build next", see
[`docs/module_map.md`](docs/module_map.md). For the hardware details, see
[`docs/finals/README.md`](../docs/finals/README.md) in the repo docs tree.*

## The mission

We get **three small HULA drones** and one laptop. Somewhere in the arena, a
**convoy of five RoboMaster ground robots drives around** — our drones have to
find them. The arena also has **valid and invalid landing pads** — where we
land at the end matters. Exact scoring rules arrive with the briefing; our
code is built so those details slot in as config changes, not rewrites.

## The one decision that shapes everything

**All our code runs on the laptop.** The drones connect over Wi-Fi; we send
them commands and they stream video back. We are *not* allowed to assume more
than the drones actually offer, and what they offer is simple:

- Commands like *"take off to 80 cm"*, *"fly forward 100 cm"*, *"turn 90°"*.
  Each command runs to completion, then the drone waits for the next one.
- **There is no "fly to position X,Y"** — the drone doesn't reliably know
  where it is, and neither do we. Everything is designed around that.
- Video comes back over Wi-Fi; all detection runs **on the laptop**, not on
  the drone. The **primary detector is ArUco** (the printed square markers on
  the convoy robots): OpenCV finds the marker *and reads its ID* in a few
  milliseconds per frame, with zero training. A trained object detector
  (YOLO) exists as an optional extra behind config — off by default — for
  spotting robots whose marker isn't readable.

So the mental model is: **one brain (the laptop), three simple bodies.** The
brain runs one loop, ten times a second. Each drone has its own little state
machine ("I'm taking off" → "I'm searching" → "I'm landing"), and the loop
ticks each one in turn. That's also exactly the pattern the organizers'
example code recommends.

## The five pillars

### 1. Build on code that already works — but trust nothing blindly
Our qualifier code flew hundreds of simulated missions; the organizers gave us
official examples for the real drones. We adapt both rather than writing from
scratch. But we **audited the examples line-by-line and found real bugs**
(an infinite wait loop, a thread that dies silently and stops all detection,
and more). Rule: every borrowed piece of code is checked, and the bugs we
fixed are written down in each file's header.

### 2. The same mission code flies in the simulator AND on real drones
We only get **one 2-hour window** with real hardware, but unlimited time in
the qualifier simulator (1 virtual drone). So flight commands go through an
"adapter" — a thin translation layer. One adapter speaks to the real drones
(pyhulax over Wi-Fi), another speaks to the simulator (MAVSDK), a third is a
fake for automated tests. The mission logic above the adapter **cannot tell
the difference** — so everything we test in the sim is a real test of the
real mission.

### 3. Search by sitting still (at first)
Since targets move and the drone can't trust its own position, the default
searcher is a **sentry**: fly up, hover, look, rotate 45°, look again — a
slow 360° scan from a fixed spot. It can't get lost, it can't drift out of
the arena, and a hovering camera takes sharp pictures (best for detection).
A moving convoy will cross a sentry's view repeatedly. The classic "mow the
lawn" sweep exists as an upgrade, but we only switch it on after we measure
(onsite, with a tape measure) how accurately the drones actually fly.

### 4. Write everything down, instantly
Every detection ("drone alpha read marker **17** at 14:02:31, bearing 40°
left, here's the image") is appended to a file **the moment it happens**, and
the file survives crashes. We never deduplicate or merge sightings — the targets
move, so every sighting is potentially worth points. If the program dies
mid-mission, the log up to that second is safe on disk.

### 5. Fail loudly, land safely
A silent bug onsite costs us the campaign, so the code is paranoid:

- **Before takeoff**: a strict checklist (all drones found? batteries? video
  fresh? detector working? log writable?) and a human typing `GO`.
- **In flight**: watchdogs — battery floor, stale telemetry, lost video,
  command timeouts, mission clock. Each knows its response (hover, land this
  drone, land everything).
- **Abort key**: one key press lands everything, on a channel that works even
  if the main loop is stuck. It can only land — never steer.
- **No mid-air surprises between drones**: each drone owns its own altitude
  (1.2 / 1.7 / 2.2 m), and only one drone may be landing at a time.
- A drone that fails **stays failed** — we never auto-restart code that could
  re-arm a real aircraft. The other two carry on.

## How we're building it

In **small batches** — one self-contained session per module, each ending
with passing tests, so no single session has to hold the whole project in its
head and nothing fragile accumulates. The skeleton (interfaces, stubs, config,
tests) is done; the build order and status live in
[`docs/module_map.md`](docs/module_map.md). Roughly: logging → fake drone →
mission engine → safety guards → simulator flight → vision → real-drone
adapter → preflight → briefing-dependent phases.

## The 2-hour hardware window

Strictly ordered, cheapest tests first, with go/no-go gates — so if something
fails at step N, everything before it is already proven:

1. **On the ground** (no flight risk): find the drones on Wi-Fi, read
   batteries, measure video with all three streaming, run the detector on
   real frames, deliberately kill Wi-Fi to see what breaks.
2. **One drone**: take off, hover 10 s, land. Then fly a square and
   tape-measure how far off it lands (this decides sentry vs. lawnmower).
   Test the abort key in the air.
3. **One drone, full mini-mission**: search + live detection + log.
4. **Two, then three drones** hovering together — only after the abort key
   is proven.
5. Dress rehearsal. **No new behavior in the last 25 minutes.**

Every fallback is a *config* change (fewer drones, shorter moves, detection
off), never live code editing.

## What we're still waiting on

The briefing must tell us: what scores points, what the landing pads look
like, the arena size and convoy route, **how big the ArUco markers are**
(size sets how far away we can read them — a ~10 cm marker reads from
roughly 2–4 m on our video stream, which directly decides how low and how
close our drones must fly), and whether an emergency abort key is allowed
mid-run. Each has a safe default already coded; the answers mostly become
config values in [`configs/`](configs/).

## Try it

```bash
pytest finals/tests                              # everything testable without hardware
python -m finals.main --profile mock --dry-run   # see the resolved mission plan
```
