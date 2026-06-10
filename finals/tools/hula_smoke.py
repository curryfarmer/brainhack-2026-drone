r"""hula_smoke.py — OFFLINE, no-flight bring-up smoke for the HULA drone FLEET.

WHY this exists (read before running): the C2-side pyhulax integration
(discovery -> PyhulaxAdapter connect/telemetry/disconnect -> PyhulaxVideoSource
-> ArUco/YOLO) is fully unit-tested against FakeDroneAPI but has NEVER touched
real hardware. The operator joins the drone's Wi-Fi AP to talk to it, which
KILLS the laptop's internet — so Claude cannot drive a live test interactively.
This script is the answer: a self-contained, fail-loud smoke that the operator
runs OFFLINE on the drone Wi-Fi. It logs EVERYTHING to disk, then the operator
switches back to internet and pastes the log back.

GROUND TRUTH (docs/finals/example_code/, the authoritative source):
  - hula_connection.py: the multi-drone Wi-Fi pattern this mirrors —
    Dola().get_all_ips() -> {plane_id: ip}; for EACH drone: DroneAPI().connect(ip)
    -> create_video_stream() -> set_video_stream(True) -> stream.start(); then a
    loop ticks ALL drones (per-drone state machine). One laptop, N concurrent
    video streams + control. THAT is the real challenge, so this smoke runs the
    WHOLE discovered fleet, not one drone.
  - dola.py: the UDP-8668 broadcast discovery (Dola). Pure stdlib, imported here
    directly as ground truth; finals.flight.discovery._parse_packet is the
    audited byte-for-byte copy used as the fallback.

WHAT it does, one capability at a time, ACROSS the fleet (NEVER spins a prop):
  1. env       — record python / numpy / cv2 / pyhulax / ultralytics + weights
  2. discover  — Dola broadcast on UDP 8668 -> every plane_id->IP
  3. connect   — PyhulaxAdapter.connect per drone (battery failsafe)  [power ON]
  4. telemetry — sweep battery / altitude / yaw / is_flying across all drones
  5. video     — LIVE scan: run the camera for --scan-secs, detecting inline
  6. aruco     — ArUco results from the scan (dict-LOCKED by default; --all-dicts
                 for the 6X6-vs-7X7 discovery sweep), per-id frame voting
  7. yolo      — ultralytics results from the scan (LOCAL weights — offline-safe)
  8. teardown  — stop all streams + disconnect all drones            [power OFF]

It REUSES the exact production seams (so the smoke validates the real code path):
finals.flight.pyhulax_adapter, finals.vision.pyhulax_video, and the Dola/
discovery wire format. It does NOT import or edit vision/aruco.py — the ArUco
probe here is a DIAGNOSTIC multi-dict scan (the in-flight detector hardcodes one
dictionary; the point of this smoke is to learn which one the field markers use).

NO flight: connect / read / disconnect only. No takeoff/move/rotate/land/led.

SINGLE DRONE FIRST (default), multidrone later (--all). The fleet machinery is
built and ready; the default smokes ONE drone so bring-up is proven on a single
airframe before loading the laptop+Wi-Fi with N concurrent streams.

Self-test (run while ONLINE, before going to the drone) — no SDK, no hardware:
    python finals\tools\hula_smoke.py --fake           # one fake drone
    python finals\tools\hula_smoke.py --fake --all     # two fake drones (fleet)

Live run (OFFLINE, on the drone Wi-Fi) — 60s live scan by default:
    python finals\tools\hula_smoke.py --ip 192.168.100.1            # 60s scan
    python finals\tools\hula_smoke.py --ip 192.168.100.1 --scan-secs 90
    python finals\tools\hula_smoke.py                               # discover+ONE
    python finals\tools\hula_smoke.py --all                         # fleet (later)

Everything lands in runs\hula_smoke_<timestamp>\ : smoke.log (paste THIS back),
summary.json, and per-drone raw + annotated JPEGs.
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import time
import traceback
from collections import Counter
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Make the repo importable whether launched as `python finals\tools\hula_smoke.py`
# or `python -m finals.tools.hula_smoke` — robust for an operator offline.
_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

# numpy / cv2 / pyhulax / ultralytics / pyrealsense2 are ALL imported lazily
# inside the functions that use them (the package's seam discipline: no SDK at
# module top level — test_conventions.py FORBIDDEN_SDK_ROOTS). This keeps the
# file importable on a bare venv and the conventions scan green.


# ============================================================
# Tee logger — everything to console AND to runs\<ts>\smoke.log
# ============================================================
class _Log:
    """Timestamped tee: prints to the console and appends to the run log file.
    The log file is what the operator pastes back, so every line is stamped."""

    def __init__(self, path: Path):
        self._fh = open(path, "w", encoding="utf-8")
        self.path = path
        self.warnings = 0
        self.errors = 0

    def line(self, msg: str = "") -> None:
        stamp = time.strftime("%H:%M:%S")
        text = f"[{stamp}] {msg}" if msg else ""
        print(text, flush=True)
        self._fh.write(text + "\n")
        self._fh.flush()

    def warn(self, msg: str) -> None:
        self.warnings += 1
        self.line(f"WARN: {msg}")

    def error(self, msg: str) -> None:
        self.errors += 1
        self.line(f"ERROR: {msg}")

    def exc(self, where: str) -> None:
        """Log the active exception with a full traceback (never swallowed)."""
        self.errors += 1
        self.line(f"ERROR in {where}:")
        for ln in traceback.format_exc().rstrip().splitlines():
            self.line(f"    {ln}")

    def rule(self, title: str) -> None:
        self.line("")
        self.line("=" * 64)
        self.line(f"== {title}")
        self.line("=" * 64)

    def close(self) -> None:
        self._fh.close()


# ============================================================
# Stage 1 — environment
# ============================================================
def stage_env(log: _Log, summary: dict, weights: Optional[str]) -> None:
    import numpy as np
    log.rule("STAGE 1/8  env")
    summary["env"] = {}
    log.line(f"python      {sys.version.split()[0]}")
    summary["env"]["python"] = sys.version.split()[0]
    log.line(f"numpy       {np.__version__}")
    summary["env"]["numpy"] = np.__version__
    try:
        import cv2
        log.line(f"cv2         {cv2.__version__}")
        summary["env"]["cv2"] = cv2.__version__
        has_aruco = hasattr(cv2, "aruco")
        log.line(f"cv2.aruco   {'present' if has_aruco else 'MISSING'}")
        summary["env"]["cv2_aruco"] = bool(has_aruco)
    except Exception:
        log.exc("import cv2")
        summary["env"]["cv2"] = None
    try:
        import pyhulax
        ver = getattr(pyhulax, "__version__", "?")
        log.line(f"pyhulax     {ver}")
        summary["env"]["pyhulax"] = str(ver)
    except Exception as e:
        log.warn(f"pyhulax not importable ({type(e).__name__}: {e}) — a LIVE "
                 f"run needs it; install with `pip install pyhulax` while online")
        summary["env"]["pyhulax"] = None
    try:
        import ultralytics
        log.line(f"ultralytics {getattr(ultralytics, '__version__', '?')}")
        summary["env"]["ultralytics"] = getattr(ultralytics, "__version__", "?")
    except Exception as e:
        log.warn(f"ultralytics not importable ({type(e).__name__}: {e}) — YOLO "
                 f"stage will be skipped")
        summary["env"]["ultralytics"] = None
    log.line(f"yolo weights {weights or '(none found — YOLO stage will skip)'}")
    summary["env"]["yolo_weights"] = weights


def _find_weights() -> Optional[str]:
    """Pick a LOCAL YOLO .pt — newest trained model first, then repo-root
    fallbacks. Offline-safe: ultralytics auto-download needs internet the drone
    Wi-Fi does not have, so a local file is REQUIRED or YOLO is skipped."""
    candidates: List[Path] = []
    models_dir = _REPO_ROOT / "models"
    if models_dir.is_dir():
        # models\yolo_<ts>\best.pt — newest by name (timestamped) wins.
        candidates.extend(sorted(models_dir.glob("yolo_*/best.pt"), reverse=True))
    for name in ("best.pt", "yolov10n.pt", "yolo26n.pt"):
        p = _REPO_ROOT / name
        if p.is_file():
            candidates.append(p)
    for c in candidates:
        if c.is_file():
            return str(c)
    return None


# ============================================================
# Stage 2 — discover  (ground truth: dola.Dola; fallback: audited parser)
# ============================================================
def discover_all(log: _Log, summary: dict, secs: float) -> Dict[int, str]:
    """Listen on UDP 8668 for the HULA Dola broadcast and return EVERY
    plane_id->IP heard. PREFERS the official Dola from the example code (ground
    truth); falls back to finals.flight.discovery._parse_packet (its audited
    byte-for-byte copy) if Dola will not import."""
    found = _discover_via_dola(log, secs)
    if found is not None:
        summary["discover"] = {str(k): v for k, v in found.items()}
        return found
    log.warn("official Dola unavailable — using the audited discovery fallback")
    found = _discover_via_parser(log, secs)
    summary["discover"] = {str(k): v for k, v in found.items()}
    return found


def _discover_via_dola(log: _Log, secs: float) -> Optional[Dict[int, str]]:
    ex = _REPO_ROOT / "docs" / "finals" / "example_code"
    if str(ex) not in sys.path:
        sys.path.insert(0, str(ex))
    try:
        from dola import Dola
    except Exception as e:
        log.line(f"(dola import failed: {type(e).__name__}: {e})")
        return None
    try:
        dola = Dola()           # binds UDP 8668
    except OSError as e:
        log.error(f"Dola() could not bind UDP 8668 ({type(e).__name__}: {e}) — "
                  f"another process holds it / firewall / not on drone Wi-Fi")
        return None
    log.line(f"Dola listening on UDP 8668 for {secs:.0f}s "
             f"(join the drone Wi-Fi if you have not)...")
    dola.start()
    try:
        time.sleep(secs)
        info = dola.get_all_plane_info()
    finally:
        dola.stop()
    found: Dict[int, str] = {}
    for pid in sorted(info):
        a = info[pid]
        found[pid] = a["ip"]
        log.line(f"  plane_id={pid}  ip={a['ip']}  sn={a.get('sn')}  "
                 f"wifi_mode={a.get('wifi_mode')}  bind_client={a.get('bind_client')}")
    log.line(f"Dola scan done: {len(found)} drone(s) found")
    return found


def _discover_via_parser(log: _Log, secs: float) -> Dict[int, str]:
    from finals.flight.discovery import _parse_packet, DISCOVERY_PORT
    found: Dict[int, str] = {}
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", DISCOVERY_PORT))
    except OSError as e:
        log.error(f"cannot bind UDP {DISCOVERY_PORT} ({type(e).__name__}: {e})")
        sock.close()
        return found
    deadline = time.monotonic() + secs
    try:
        while time.monotonic() < deadline:
            sock.settimeout(min(0.5, max(0.0, deadline - time.monotonic())))
            try:
                packet, addr = sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError as e:
                log.warn(f"recv failed ({type(e).__name__}: {e})")
                break
            try:
                pinfo = _parse_packet(packet, addr[0])
            except (ValueError, IndexError, UnicodeError):
                continue
            if pinfo is None:
                continue
            pid, ip = pinfo["plane_id"], pinfo["ip"]
            if pid not in found:
                log.line(f"  plane_id={pid}  ip={ip}  sn={pinfo['sn']}")
            found[pid] = ip
    finally:
        try:
            sock.close()
        except OSError:
            pass
    log.line(f"fallback scan done: {len(found)} drone(s) found")
    return found


# ============================================================
# Fake hardware (for --fake self-test; no SDK, no drone)
# ============================================================
class _NumpyFrame:
    def __init__(self, image):
        self._image = image

    def to_rgb(self):
        return self._image


class _NumpyFakeStream:
    """pyhulax VideoStream double that returns a REAL numpy frame (the repo's
    FakeVideoStream returns a channel-list stand-in, unusable for JPEG/decode).
    Drives the WHOLE PyhulaxVideoSource path in --fake mode."""

    def __init__(self, image):
        self._image = image
        self.state = 3            # _STREAM_STATE_RUNNING
        self.frame_count = 0
        self.last_error = None

    def start(self, blocking: bool = False):
        self.frame_count += 1

    def stop(self, timeout=None):
        self.state = 5            # _STREAM_STATE_STOPPED

    @property
    def latest_frame(self):
        self.frame_count += 1
        return _NumpyFrame(self._image)


def _synthetic_marker_frame(marker_id: int, dict_name: str = "DICT_7X7_1000"):
    """640x480 BGR gray frame with a real marker (default DICT_7X7_1000, the field
    dict) in the middle, so the --fake ArUco probe decodes something under the
    DEFAULT dict-lock (a 6X6 synthetic marker would decode as nothing now)."""
    import cv2
    import numpy as np
    img = np.full((480, 640, 3), 128, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    marker = cv2.aruco.generateImageMarker(d, marker_id, 200)
    marker_bgr = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    img[140:340, 220:420] = marker_bgr
    return img


# ============================================================
# A connected drone in the fleet
# ============================================================
class _Drone:
    def __init__(self, drone_id: str, ip: str, adapter, channel_order: str):
        self.id = drone_id
        self.ip = ip
        self.adapter = adapter
        self.channel_order = channel_order
        # Live-scan aggregates (filled by scan_fleet over the whole window).
        self.aruco_ids: dict = {}       # dict_name -> Counter{id: frames_voted}
        self.yolo_classes: dict = {}    # class_name -> peak confidence
        self.frames_seen = 0
        self.fps: Optional[float] = None
        self.saved = 0
        self.last_image = None          # most recent frame (for shape / fallback)


# ============================================================
# Stages 3-9 (the fleet flow) — async because connect() is async
# ============================================================
async def run_fleet(log: _Log, summary: dict, args, outdir: Path,
                    weights: Optional[str]) -> None:
    from finals.flight.pyhulax_adapter import PyhulaxAdapter, FakeDroneAPI

    # ---- resolve targets: list of (drone_id, ip, injected_api_or_None) -----
    log.rule("STAGE 2/8  discover")
    targets: List[Tuple[str, str, object]] = []
    if args.fake:
        # field ids (11, 45) so the default 7X7 lock decodes them cleanly and the
        # allowlist passes (a non-field id would self-flag as a ghost).
        fakes = (("fakeA", 11, 87.0), ("fakeB", 45, 64.0)) if args.all \
            else (("fakeA", 11, 87.0),)
        log.line(f"--fake: injecting {len(fakes)} FakeDroneAPI drone(s)")
        for did, mid, batt in fakes:
            api = FakeDroneAPI(battery_pct=batt, altitude_cm=0.0, yaw_deg=10.0,
                               is_flying=False,
                               video_stream=_NumpyFakeStream(
                                   _synthetic_marker_frame(mid)))
            targets.append((did, "127.0.0.1", api))
    elif args.ip:
        log.line(f"--ip given: skipping discovery, using {args.ip}")
        did = str(args.plane_id) if args.plane_id is not None else "hula"
        targets.append((did, args.ip, None))
    else:
        found = discover_all(log, summary, args.discover_secs)
        if not found:
            log.error("no drones discovered and no --ip given — check Wi-Fi / "
                      "SSID / power / UDP 8668 firewall. Cannot continue.")
            return
        if args.plane_id is not None:
            if args.plane_id not in found:
                log.error(f"plane_id {args.plane_id} not among discovered "
                          f"{sorted(found)} — cannot continue")
                return
            targets.append((str(args.plane_id), found[args.plane_id], None))
        elif args.all:
            for pid in sorted(found):
                targets.append((str(pid), found[pid], None))
            log.line(f"--all: smoking the WHOLE fleet: {[t[0] for t in targets]}")
        else:
            # Single drone first (default): lowest plane_id; multidrone is --all.
            pid = sorted(found)[0]
            targets.append((str(pid), found[pid], None))
            if len(found) > 1:
                log.line(f"single-drone mode (default): smoking plane_id {pid}; "
                         f"also discovered {sorted(found)} — pass --all for the "
                         f"whole fleet, or --plane-id N to pick another")
            else:
                log.line(f"single drone discovered: plane_id {pid}")

    channel_order = "bgr" if args.fake else args.channel_order

    # ---- STAGE 3: connect every drone (power ON) --------------------------
    log.rule(f"STAGE 3/8  connect {len(targets)} drone(s)  [power ON]")
    summary["connect"] = {}
    fleet: List[_Drone] = []
    for did, ip, api in targets:
        try:
            adapter = (PyhulaxAdapter(did, ip=ip, api=api) if api is not None
                       else PyhulaxAdapter(did, ip=ip))
            if api is None:                       # real drone: probe the link first
                _net_probe(log, summary, ip)
            t0 = time.monotonic()
            await adapter.connect(timeout_s=args.connect_timeout)
            log.line(f"  [{did}] connected to {ip} in "
                     f"{time.monotonic() - t0:.1f}s (failsafe on)")
            summary["connect"][did] = {"ok": True, "ip": ip}
            fleet.append(_Drone(did, ip, adapter, channel_order))
        except Exception:
            log.exc(f"connect[{did}]")
            summary["connect"][did] = {"ok": False, "ip": ip}
    if not fleet:
        log.error("no drone connected — nothing to smoke")
        return

    try:
        telemetry_sweep(log, summary, fleet, args.telemetry_secs)
        # Stage 5 is a LIVE scan: it grabs frames for the whole scan window and
        # runs ArUco + YOLO INLINE (logging hits as they happen), so stages 6/7
        # just report what the live scan already found.
        scan_fleet(log, summary, fleet, outdir, args, weights)
        log.rule("STAGE 6/8  aruco (live-scan results, per drone)")
        for dr in fleet:
            _report_aruco(log, summary, dr)
        log.rule("STAGE 7/8  yolo (live-scan results, per drone)")
        if args.no_yolo:
            log.line("skipped (--no-yolo)")
        else:
            for dr in fleet:
                _report_yolo(log, summary, dr)
    finally:
        await _teardown_fleet(log, summary, fleet)


#: pyhulax control port (observed in the connect log: "<ip>:8888"). Discovery is
#: UDP 8668; control/telemetry is this TCP port.
_HULA_CONTROL_PORT = 8888


def _net_probe(log: _Log, summary: dict, ip: str,
               port: int = _HULA_CONTROL_PORT) -> None:
    """Diagnose the link to the drone BEFORE pyhulax connect, so a connect
    failure names the layer: local interface IPs, which NIC the OS routes to the
    drone through (multi-homing catch), and a raw TCP test to the control port
    (separates 'network unreachable' from 'pyhulax handshake / drone busy')."""
    rec: dict = {}
    # All local IPv4s — confirms an interface is actually on the drone subnet.
    try:
        _, _, addrs = socket.gethostbyname_ex(socket.gethostname())
        rec["local_ips"] = addrs
        log.line(f"  net: local IPv4s = {addrs}")
    except OSError as e:
        log.line(f"  net: gethostbyname_ex failed ({type(e).__name__}: {e})")
    # Which source IP the OS would use to reach the drone (no packets sent).
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect((ip, port))
            out_ip = s.getsockname()[0]
        finally:
            s.close()
        rec["route_src_ip"] = out_ip
        same = out_ip.rsplit(".", 1)[0] == ip.rsplit(".", 1)[0]
        verdict = ("same /24, OK" if same else
                   "DIFFERENT subnet: another NIC (ethernet?) is stealing the "
                   "route — disable it")
        log.line(f"  net: route to {ip} leaves via {out_ip} — {verdict}")
    except OSError as e:
        log.line(f"  net: cannot resolve route to {ip} ({type(e).__name__}: {e})")
    # Raw TCP to the control port — does the socket layer even reach it?
    try:
        c = socket.create_connection((ip, port), timeout=3.0)
        c.close()
        rec["tcp_control"] = "open"
        log.line(f"  net: TCP {ip}:{port} OPEN — socket layer reaches the drone. "
                 f"If pyhulax still fails, ANOTHER CLIENT likely holds it (HULA = "
                 f"one controller: close the phone/HulaGo app), or a bind is needed")
    except OSError as e:
        rec["tcp_control"] = f"{type(e).__name__}: {e}"
        log.line(f"  net: TCP {ip}:{port} FAILED ({type(e).__name__}: {e}) — "
                 f"firewall blocking python? wrong NIC? drone busy/off? Try: "
                 f"Test-NetConnection {ip} -Port {port}")
    summary.setdefault("net_probe", {})[ip] = rec


def _fmt(v, unit: str) -> str:
    return "None" if v is None else f"{v:.1f}{unit}"


def telemetry_sweep(log, summary, fleet: List[_Drone], secs: float) -> None:
    """Tick every connected drone for `secs` seconds (the hula_connection.py
    round-robin), logging battery/alt/yaw/flying per drone."""
    log.rule("STAGE 4/8  telemetry (fleet sweep)")
    summary["telemetry"] = {dr.id: {"samples": 0} for dr in fleet}
    end = time.monotonic() + secs
    last = {dr.id: None for dr in fleet}
    while time.monotonic() < end:
        for dr in fleet:
            try:
                t = dr.adapter.telemetry()
            except Exception:
                log.exc(f"telemetry[{dr.id}]")
                continue
            last[dr.id] = t
            summary["telemetry"][dr.id]["samples"] += 1
            log.line(f"  [{dr.id}] batt={_fmt(t.battery_pct,'%')}  "
                     f"alt={_fmt(t.altitude_m,'m')}  "
                     f"yaw={_fmt(t.yaw_deg,'deg')}  flying={t.is_flying}  "
                     f"age={t.age_s():.2f}s")
        time.sleep(1.0)
    for dr in fleet:
        t = last[dr.id]
        if t is None:
            log.error(f"[{dr.id}] no telemetry samples — poller produced nothing")
            continue
        _sanity_telemetry(log, dr.id, t)
        summary["telemetry"][dr.id]["last"] = {
            "battery_pct": t.battery_pct, "altitude_m": t.altitude_m,
            "yaw_deg": t.yaw_deg, "is_flying": t.is_flying}


def _sanity_telemetry(log: _Log, did: str, t) -> None:
    import numpy as np
    if t.battery_pct is None:
        log.warn(f"[{did}] battery_pct is None — getter returned nothing")
    elif not (0.0 < t.battery_pct <= 100.0):
        log.warn(f"[{did}] battery_pct {t.battery_pct} outside (0,100]")
    elif t.battery_pct < 30.0:
        log.warn(f"[{did}] battery LOW ({t.battery_pct:.0f}%) — charge first")
    if t.altitude_m is not None and not np.isfinite(t.altitude_m):
        log.warn(f"[{did}] altitude_m not finite: {t.altitude_m}")
    if t.yaw_deg is not None and not np.isfinite(t.yaw_deg):
        log.warn(f"[{did}] yaw_deg not finite: {t.yaw_deg}")


# Candidate ArUco dictionaries — the field markers are ArUco DICT_7X7_1000 (field
# intel). The real-hardware DOUBLE-DECODE (one marker -> two ids) is cross-DICT
# false-validation: feed a 7X7 marker to a 4x4/5x5/6x6 detector and that dict's
# error-correction snaps it onto a wrong-but-valid id. So the scan LOCKS to one
# dict by default (--aruco-dict); --all-dicts re-enables the discovery sweep.
_ARUCO_DICTS = [
    "DICT_4X4_250", "DICT_5X5_250", "DICT_6X6_250", "DICT_7X7_1000",
    "DICT_APRILTAG_36h11",
]

#: The five fixed-coordinate field markers (DICT_7X7_1000 — see field intel).
#: Ids decoded OUTSIDE this set under the field dict are flagged as suspected
#: ghosts (low-vote mis-decodes) in _report_aruco.
_FIELD_ARUCO_IDS = frozenset({11, 45, 51, 67, 101})


def _build_aruco_detectors(log, only: Optional[str] = None):
    """Build ArUco detectors. `only` (e.g. 'DICT_7X7_1000') builds ONE detector —
    the default field path, which kills the cross-dict double-decode. only=None
    builds all _ARUCO_DICTS (the --all-dicts discovery sweep). Params are tuned
    for the drone cam: sub-pixel corners + a STRICTER errorCorrectionRate (0.4 vs
    the 0.6 default — the exact knob that lets wrong-dict patterns validate)."""
    import cv2
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.errorCorrectionRate = 0.4
    names = _ARUCO_DICTS if only is None else [only]
    detectors = {}
    for name in names:
        const = getattr(cv2.aruco, name, None)
        if const is None:
            log.warn(f"cv2.aruco has no {name} (skipping that dict)")
            continue
        detectors[name] = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(const), params)
    if only is not None and not detectors:
        log.error(f"requested ArUco dict {only!r} unavailable in this cv2 build "
                  f"— falling back to the full discovery sweep")
        return _build_aruco_detectors(log, only=None)
    return detectors


def _touches_border(xyxy, w: int, h: int, margin: int) -> bool:
    """True if the bbox is within `margin` px of any frame edge — a hand/arm
    entering from a corner trips this; a centered landing pad does not. The
    border-reject that keeps a low --yolo-conf from re-introducing the corner FP."""
    x0, y0, x1, y1 = xyxy
    return (x0 <= margin or y0 <= margin
            or x1 >= w - margin or y1 >= h - margin)


def _normalize_for_yolo(image, mode: str):
    """Fight the drone cam's oversaturation BEFORE YOLO. 'gray-world' rescales
    each channel to a common mean (cheap white-balance); 'clahe' equalizes the L
    channel in LAB (local contrast). 'none' returns the frame unchanged. Always
    returns a BGR uint8 array (gray-world/clahe return a fresh copy; the caller's
    frame is never mutated)."""
    if mode == "none":
        return image
    import cv2
    import numpy as np
    if mode == "gray-world":
        out = image.astype(np.float32)
        means = [float(out[:, :, c].mean()) for c in range(3)]
        target = sum(means) / 3.0
        for c in range(3):
            if means[c] > 1e-6:
                out[:, :, c] *= target / means[c]
        return np.clip(out, 0, 255).astype(np.uint8)
    if mode == "clahe":
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        return cv2.cvtColor(cv2.merge((clahe.apply(l), a, b)), cv2.COLOR_LAB2BGR)
    return image


def scan_fleet(log, summary, fleet: List[_Drone], outdir, args, weights) -> None:
    """LIVE scan: start all streams, then for the full --scan-secs window grab
    frames and run ArUco (every frame) + YOLO (throttled) INLINE, logging hits
    AS THEY HAPPEN so the operator gets real-time feedback while moving markers /
    pads through view. A 60-90s stream is thousands of frames, so they are NOT
    all kept — only new-sighting frames + a periodic raw sample are saved,
    bounded by --max-save. Aggregates land on each _Drone for stages 6/7."""
    import cv2
    from finals.vision.pyhulax_video import PyhulaxVideoSource

    log.rule("STAGE 5/8  video — LIVE scan")
    summary["video"] = {}
    by_id = {dr.id: dr for dr in fleet}
    sources = {}
    framedirs = {}
    for dr in fleet:
        api = getattr(dr.adapter, "_api", None)
        if api is None:
            log.error(f"[{dr.id}] adapter has no _api after connect")
            continue
        try:
            src = PyhulaxVideoSource(dr.id, api,
                                     video_channel_order=dr.channel_order)
            src.start(timeout_s=args.video_timeout)
            sources[dr.id] = src
            fd = outdir / dr.id / "frames"
            fd.mkdir(parents=True, exist_ok=True)
            framedirs[dr.id] = fd
            log.line(f"  [{dr.id}] stream started "
                     f"(channel_order={dr.channel_order!r})")
        except Exception:
            log.exc(f"video.start[{dr.id}]")

    detectors = _build_aruco_detectors(
        log, only=(None if args.all_dicts else args.aruco_dict))
    if args.all_dicts:
        log.line(f"ArUco: scanning ALL {len(detectors)} dicts (discovery sweep) — "
                 f"expect cross-dict ghosts")
    else:
        log.line(f"ArUco: LOCKED to {args.aruco_dict} (one detector) — pass "
                 f"--all-dicts for the discovery sweep")
    model = None
    if not args.no_yolo:
        if not weights:
            log.warn("no local YOLO weights — YOLO disabled for the scan (put a "
                     ".pt in models\\ or repo root)")
        else:
            try:
                from ultralytics import YOLO
                model = YOLO(weights)
                log.line(f"YOLO loaded for live scan "
                         f"({len(model.names)} classes, every "
                         f"{args.yolo_period:.1f}s)")
            except Exception:
                log.exc("load ultralytics YOLO")

    scan_secs = max(0.0, args.scan_secs)
    max_save = max(0, args.max_save)
    yolo_period = max(0.0, args.yolo_period)
    sample_period = max(1.0, scan_secs / 12.0)   # ~12 raw samples across window
    if sources and scan_secs > 0:
        log.line(f"  >>> SCANNING {scan_secs:.0f}s — move the ArUco marker / "
                 f"landing pad through each camera's view now (saving up to "
                 f"{max_save} hit/sample frames per drone) <<<")

    start = time.monotonic()
    deadline = start + scan_secs
    st = {did: {"last_yolo_t": 0.0, "last_sample_t": 0.0} for did in sources}
    while sources and time.monotonic() < deadline:
        now = time.monotonic()
        for did, src in sources.items():
            dr = by_id[did]
            try:
                fs = src.get_frame()
            except Exception:
                log.exc(f"get_frame[{did}]")
                continue
            if fs is None:
                continue
            dr.frames_seen += 1
            dr.last_image = fs.image
            elapsed = now - start
            saved_this = False
            # ---- ArUco on EVERY frame (cheap) ----
            gray = cv2.cvtColor(fs.image, cv2.COLOR_BGR2GRAY)
            for name, det in detectors.items():
                corners, ids, _ = det.detectMarkers(gray)
                if ids is None or len(ids) == 0:
                    continue
                id_list = sorted(int(x) for x in ids.flatten())
                counter = dr.aruco_ids.setdefault(name, Counter())
                new = sorted(set(id_list) - set(counter))
                counter.update(id_list)     # per-id frame vote
                if new:     # log only when the id-set GROWS (avoid 23 Hz spam)
                    note = ""
                    if name == "DICT_7X7_1000":
                        ghosts = [i for i in new if i not in _FIELD_ARUCO_IDS]
                        if ghosts:
                            note = (f"  GHOST? {ghosts} not in field ids "
                                    f"{sorted(_FIELD_ARUCO_IDS)}")
                    log.line(f"  [{did}] t={elapsed:5.1f}s  ARUCO {name} -> "
                             f"ids {id_list}  (NEW {new}){note}")
                    if dr.saved < max_save and not saved_this:
                        annot = fs.image.copy()
                        cv2.aruco.drawDetectedMarkers(annot, corners, ids)
                        cv2.imwrite(str(framedirs[did]
                                    / f"aruco_{dr.saved:03d}_t{elapsed:.0f}.jpg"),
                                    annot)
                        dr.saved += 1
                        saved_this = True
            # ---- YOLO throttled (conf knob + border-reject + preproc) ----
            if model is not None and now - st[did]["last_yolo_t"] >= yolo_period:
                st[did]["last_yolo_t"] = now
                infer_img = _normalize_for_yolo(fs.image, args.yolo_preproc)
                try:
                    results = model(infer_img, verbose=False, conf=args.yolo_conf)
                except Exception:
                    log.exc(f"YOLO infer[{did}]")
                    results = []
                h, w = fs.image.shape[:2]
                for r in results:
                    boxes = r.boxes
                    if boxes is None or len(boxes) == 0:
                        continue
                    accepted: List[Tuple[str, float]] = []
                    rejected: List[Tuple[str, float]] = []
                    fresh = False
                    for b in boxes:
                        cn = model.names[int(b.cls[0])]
                        cf = float(b.conf[0])
                        xyxy = [float(v) for v in b.xyxy[0].tolist()]
                        if args.edge_margin > 0 and _touches_border(
                                xyxy, w, h, args.edge_margin):
                            rejected.append((cn, cf))   # hand/arm at the edge
                            continue
                        accepted.append((cn, cf))
                        if cn not in dr.yolo_classes:
                            fresh = True
                        if cf > dr.yolo_classes.get(cn, 0.0):
                            dr.yolo_classes[cn] = cf
                    if accepted:
                        pairs = ", ".join(f"{cn}:{cf:.2f}" for cn, cf in accepted)
                        extra = (f"  (edge-rejected {len(rejected)}: "
                                 f"{', '.join(c for c, _ in rejected)})"
                                 if rejected else "")
                        log.line(f"  [{did}] t={elapsed:5.1f}s  YOLO -> "
                                 f"{pairs}{extra}")
                        # r.plot() draws on the (normalized) frame the model saw,
                        # so the saved JPEG matches inference, not the raw frame.
                        if dr.saved < max_save and (fresh or not saved_this):
                            try:
                                cv2.imwrite(str(framedirs[did]
                                    / f"yolo_{dr.saved:03d}_t{elapsed:.0f}.jpg"),
                                    r.plot())
                                dr.saved += 1
                                saved_this = True
                            except Exception:
                                log.exc(f"save yolo[{did}]")
                    elif rejected:
                        names = ", ".join(f"{cn}:{cf:.2f}" for cn, cf in rejected)
                        log.line(f"  [{did}] t={elapsed:5.1f}s  YOLO edge-rejected "
                                 f"{len(rejected)} border box(es) [{names}] — "
                                 f"likely a hand/arm at the frame edge, ignored")
            # ---- periodic raw sample (proof-of-life even with no detections) ----
            if (not saved_this and dr.saved < max_save
                    and now - st[did]["last_sample_t"] >= sample_period):
                st[did]["last_sample_t"] = now
                try:
                    cv2.imwrite(str(framedirs[did]
                        / f"sample_{dr.saved:03d}_t{elapsed:.0f}.jpg"), fs.image)
                    dr.saved += 1
                except Exception:
                    log.exc(f"save sample[{did}]")
        time.sleep(0.01)

    # ---- stop streams + per-drone summary ----
    for did, src in sources.items():
        dr = by_id[did]
        try:
            healthy = src.healthy
        except Exception:
            healthy = None
        try:
            src.stop()
        except Exception:
            log.exc(f"video.stop[{did}]")
        elapsed = max(1e-6, time.monotonic() - start)
        dr.fps = round(dr.frames_seen / elapsed, 1)
        rec = {"frames_seen": dr.frames_seen, "fps": dr.fps, "saved": dr.saved,
               "channel_order": dr.channel_order, "healthy": healthy}
        if dr.frames_seen == 0:
            log.error(f"[{did}] NO frames in {scan_secs:.0f}s — camera/link/decode")
        elif dr.last_image is not None:
            h, w = dr.last_image.shape[:2]
            means = [round(float(dr.last_image[:, :, c].mean()), 1)
                     for c in range(3)]
            rec["shape"] = [w, h]
            rec["channel_means"] = means
            log.line(f"  [{did}] {dr.frames_seen} frames in {elapsed:.0f}s "
                     f"(~{dr.fps} fps), {dr.saved} saved, {w}x{h}, "
                     f"healthy={healthy}")
        summary["video"][did] = rec


def _report_aruco(log, summary, dr: _Drone) -> None:
    summary.setdefault("aruco", {})
    by_dict = {name: c for name, c in dr.aruco_ids.items() if c}
    # serialize each Counter as {id: frame_votes} for the summary json
    summary["aruco"][dr.id] = {name: {str(i): n for i, n in c.items()}
                               for name, c in by_dict.items()}
    if not by_dict:
        log.line(f"  [{dr.id}] no ArUco decoded during the scan — aim a marker at "
                 f"the lens and re-run, or it uses a dict outside the scanned set "
                 f"(retry with --all-dicts)")
        return
    for name, counter in by_dict.items():
        votes = ", ".join(f"{i}x{n}" for i, n in counter.most_common())
        log.line(f"  [{dr.id}] {name}: votes {votes}")
        if name == "DICT_7X7_1000":
            ghosts = sorted(i for i in counter if i not in _FIELD_ARUCO_IDS)
            if ghosts:
                log.line(f"  [{dr.id}] {name}: ids {ghosts} are NOT field markers "
                         f"{sorted(_FIELD_ARUCO_IDS)} — low-vote ids are mis-"
                         f"decodes (outvoted ghosts); the dominant id is the real "
                         f"marker")
    if len(by_dict) == 1:
        name = next(iter(by_dict))
        top_id, top_n = by_dict[name].most_common(1)[0]
        log.line(f"  [{dr.id}] => field markers decode as {name}, dominant id "
                 f"{top_id} ({top_n} frames) (settles the dict question)")


def _report_yolo(log, summary, dr: _Drone) -> None:
    summary.setdefault("yolo", {})
    classes = {k: round(v, 2) for k, v in dr.yolo_classes.items()}
    summary["yolo"][dr.id] = classes
    if not classes:
        log.line(f"  [{dr.id}] no YOLO detections during the scan")
        return
    pairs = ", ".join(f"{k}(peak {v:.2f})" for k, v in classes.items())
    log.line(f"  [{dr.id}] classes seen: {pairs}")
    if max(classes.values()) < 0.5:
        log.line(f"  [{dr.id}] peak YOLO conf < 0.50 — if the cam looks color-"
                 f"swapped recheck --channel-order (see channel_means above), try "
                 f"--yolo-preproc gray-world; durable fix = fine-tune on the saved "
                 f"frames with hand/background hard negatives (runbook)")


# (periodic depth logger removed on main — depth is the SENSE-IR seam in
# finals/vision/depth.py, unreachable over the Wi-Fi smoke; see module_map.md)


async def _teardown_fleet(log, summary, fleet: List[_Drone]) -> None:
    log.rule("STAGE 8/8  teardown (disconnect all)  [power OFF]")
    summary["final_snapshot"] = {}
    for dr in fleet:
        try:
            await dr.adapter.disconnect()
            log.line(f"  [{dr.id}] disconnected (link released)")
        except Exception:
            log.exc(f"disconnect[{dr.id}]")
        try:
            snap = dr.adapter.telemetry()
            log.line(f"  [{dr.id}] final: batt={_fmt(snap.battery_pct,'%')} "
                     f"alt={_fmt(snap.altitude_m,'m')} "
                     f"yaw={_fmt(snap.yaw_deg,'deg')} flying={snap.is_flying}")
            summary["final_snapshot"][dr.id] = {
                "battery_pct": snap.battery_pct, "altitude_m": snap.altitude_m,
                "yaw_deg": snap.yaw_deg, "is_flying": snap.is_flying}
        except Exception as e:
            log.line(f"  [{dr.id}] no final snapshot ({type(e).__name__}: {e})")


# ============================================================
# main
# ============================================================
def _parse_args(argv) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="OFFLINE no-flight bring-up smoke for the HULA drone fleet "
                    "(discover -> connect -> telemetry -> video -> aruco/yolo -> "
                    "disconnect). Mirrors docs/finals/example_code/hula_connection.py.")
    p.add_argument("--ip", default=None,
                   help="single drone IP; skips discovery (e.g. 192.168.1.50)")
    p.add_argument("--plane-id", type=int, default=None,
                   help="smoke ONLY this plane_id from discovery")
    p.add_argument("--all", action="store_true",
                   help="smoke the WHOLE discovered fleet (default: single drone, "
                        "lowest plane_id — single first, multidrone later)")
    p.add_argument("--discover-secs", type=float, default=15.0,
                   help="seconds to listen for the Dola broadcast (default 15)")
    p.add_argument("--telemetry-secs", type=float, default=6.0,
                   help="seconds to sweep telemetry across the fleet (default 6)")
    p.add_argument("--scan-secs", type=float, default=60.0,
                   help="LIVE scan duration in seconds — how long the camera runs "
                        "while you move ArUco markers / pads through view, "
                        "detecting inline (default 60; try 90 for a long sweep)")
    p.add_argument("--max-save", type=int, default=60,
                   help="cap on saved JPEGs per drone during the scan "
                        "(hit + sample frames; default 60)")
    p.add_argument("--yolo-period", type=float, default=1.5,
                   help="seconds between YOLO inferences during the scan "
                        "(throttle; ArUco runs every frame; default 1.5)")
    # ---- ArUco (fix the real-hardware double-decode) ----
    p.add_argument("--aruco-dict", default="DICT_7X7_1000", choices=_ARUCO_DICTS,
                   help="LOCK ArUco decode to ONE dictionary (default "
                        "DICT_7X7_1000, the field dict) — prevents the cross-dict "
                        "double-decode (one marker -> two ids)")
    p.add_argument("--all-dicts", action="store_true",
                   help="scan ALL candidate dicts (the bring-up discovery sweep) "
                        "instead of locking to --aruco-dict; expect cross-dict "
                        "ghosts in the output")
    # ---- YOLO (oversaturation + hand-in-corner false positive) ----
    p.add_argument("--yolo-conf", type=float, default=0.25,
                   help="YOLO confidence threshold for the scan (default 0.25)")
    p.add_argument("--edge-margin", type=int, default=8,
                   help="reject YOLO boxes within N px of any frame edge — kills "
                        "the hand/arm-in-corner false positive (0 disables; "
                        "default 8)")
    p.add_argument("--yolo-preproc", default="none",
                   choices=("none", "gray-world", "clahe"),
                   help="normalize the frame before YOLO to fight the drone cam's "
                        "oversaturation (default none; try gray-world or clahe)")
    p.add_argument("--connect-timeout", type=float, default=15.0,
                   help="per-drone connect timeout seconds (default 15)")
    p.add_argument("--video-timeout", type=float, default=15.0,
                   help="first-frame timeout seconds (default 15)")
    p.add_argument("--channel-order", default="rgb", choices=("rgb", "bgr"),
                   help="what stream.to_rgb() actually returns (bench-verify; "
                        "default rgb)")
    p.add_argument("--weights", default=None,
                   help="YOLO .pt path (default: auto-detect a local one)")
    p.add_argument("--no-yolo", action="store_true", help="skip the YOLO stage")
    p.add_argument("--out", default=None,
                   help="output dir (default runs\\hula_smoke_<timestamp>)")
    p.add_argument("--fake", action="store_true",
                   help="self-test with TWO FakeDroneAPI drones + synthetic "
                        "marker frames (no SDK, no hardware) — run ONLINE first")
    return p.parse_args(argv)


def main(argv=None) -> int:
    import asyncio
    args = _parse_args(argv)

    ts = time.strftime("%Y%m%dT%H%M%S")
    outdir = Path(args.out) if args.out else _REPO_ROOT / "runs" / f"hula_smoke_{ts}"
    outdir.mkdir(parents=True, exist_ok=True)
    log = _Log(outdir / "smoke.log")
    summary: dict = {"started": ts, "mode": "fake" if args.fake else "live",
                     "outdir": str(outdir)}

    log.rule(f"HULA fleet bring-up smoke  ({'FAKE self-test' if args.fake else 'LIVE'})")
    log.line(f"output dir: {outdir}")
    log.line("NO flight commands are issued (connect/read/disconnect only).")

    weights = args.weights or _find_weights()
    try:
        stage_env(log, summary, weights)
        if sys.platform == "win32":
            asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())
        asyncio.run(run_fleet(log, summary, args, outdir, weights))
    except KeyboardInterrupt:
        log.warn("interrupted by operator (Ctrl-C)")
    except Exception:
        log.exc("main")

    summary["warnings"] = log.warnings
    summary["errors"] = log.errors
    log.rule("RESULT")
    log.line(f"warnings={log.warnings}  errors={log.errors}")
    log.line(f"PASTE BACK: {log.path}")
    log.line(f"summary:    {outdir / 'summary.json'}")
    try:
        (outdir / "summary.json").write_text(
            json.dumps(summary, indent=2), encoding="utf-8")
    except OSError as e:
        log.error(f"could not write summary.json ({e})")
    log.close()
    return 1 if log.errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
