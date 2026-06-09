#!/usr/bin/env python3
"""SIM-3 marker + convoy-robot asset generator (gz-only world assets).

Generates Gazebo-Harmonic model dirs for the SIM-3 convoy world from
`finals/docs/simulation.md` Tier 2 recipes. Two kinds:

  --kind plane : a STATIC textured plane (a bare marker / a landing pad).
                 `<plane><size>S S</size></plane>` at z=0.001, material
                 `<pbr><metal><albedo_map>…png</albedo_map></metal></pbr>`.
  --kind robot : a RoboMaster-ish chassis (~0.4x0.3x0.25 m) carrying that marker
                 as a TOP-FACING visual (welded to the body => moves rigidly),
                 driven by the gz VelocityControl plugin on /model/<name>/cmd_vel.

Marker textures (both supported; the world is reskinned by re-running with --type):
  aruco : cv2.aruco.generateImageMarker(DICT_6X6_250, id, 800) + a >=1-module white
          quiet zone (markers without it detect unreliably).
  qr    : cv2.QRCodeEncoder (payload = the id string) upscaled crisp + a 4-module
          white quiet zone. A true QR needs far more px/module than ArUco, so its
          decode standoff is much shorter — exactly what check_detection.py measures.

Model dir NAMES are type-AGNOSTIC (convoy_robot_<id>, pad_<id>, marker_<id>) so the
world `<include>`s stable names and `--type {aruco|qr}` reskins the textures in place
(the one-key marker_backend switch, sim_sessions.md recap S7). This script is in
`sim/` BY DESIGN — outside the finals conventions/SDK scan — so raw cv2 is allowed.

Runs anywhere cv2 is importable (Windows finals venv or the VM .venv; gz NOT needed
to generate). Fail-loud: bad args / missing cv2 features exit nonzero with WHAT/WHY/CHECK.

Usage:
    # convoy robots (5) + pads (2), ArUco skin
    python sim/gen_markers.py --type aruco --kind robot --ids 7 11 23 42 88 --size-cm 20
    python sim/gen_markers.py --type aruco --kind plane --prefix pad --ids 100 101 --size-cm 40
    # reskin the SAME dirs to QR (world unchanged):
    python sim/gen_markers.py --type qr    --kind robot --ids 7 11 23 42 88 --size-cm 20
    python sim/gen_markers.py --type qr    --kind plane --prefix pad --ids 100 101 --size-cm 40
"""

import argparse
import os
import sys

import numpy as np

try:
    import cv2
except ImportError as exc:  # fail-loud: this script is useless without cv2
    print(
        f"FAIL: cannot import cv2 — WHY: opencv not installed in this interpreter — "
        f"CHECK: pip install opencv-python 'numpy<2'  ({exc})",
        file=sys.stderr,
    )
    sys.exit(2)

# Repo-relative default output: sim/models/ (goes on GZ_SIM_RESOURCE_PATH at run time).
_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.join(_HERE, "models")

ARUCO_SRC_PX = 800          # >=800 px sources avoid texture blur (simulation.md Tier 2)
ARUCO_QUIET_MODULES = 1     # >=1 white module quiet zone (reliable detection)
QR_QUIET_MODULES = 4        # QR spec quiet zone
QR_MODULE_PX = 24           # crisp upscale: px per QR module before the quiet border


# --------------------------------------------------------------------------- #
# texture generation
# --------------------------------------------------------------------------- #
def make_aruco_png(marker_id: int) -> np.ndarray:
    """DICT_6X6_250 marker at >=800 px with a >=1-module white quiet zone (BGR)."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, ARUCO_SRC_PX, borderBits=1)
    # DICT_6X6 markers are 8 modules wide (6 data + 1 black border each side).
    module_px = ARUCO_SRC_PX // 8
    quiet = ARUCO_QUIET_MODULES * module_px
    img = cv2.copyMakeBorder(img, quiet, quiet, quiet, quiet,
                             cv2.BORDER_CONSTANT, value=255)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def make_qr_png(payload: str) -> np.ndarray:
    """QR encoding `payload`, crisp-upscaled with a 4-module white quiet zone (BGR)."""
    try:
        # Force the smallest QR (version 1 = 21 modules) at correction level L: a 1-char
        # id needs no more, and fewer modules => more px/module => a realistic decode floor
        # (organizers would use a minimal QR for a short id, not a high-ECC version-2).
        # The L-level constant moved between cv2 builds — look it up robustly.
        level_l = getattr(cv2, "QRCodeEncoder_CORRECT_LEVEL_L",
                          getattr(cv2, "QRCODE_ENCODER_CORRECT_LEVEL_L", None))
        params = cv2.QRCodeEncoder.Params()
        params.version = 1
        if level_l is not None:
            params.correction_level = level_l
        encoder = cv2.QRCodeEncoder.create(params)
    except AttributeError as exc:
        print(
            f"FAIL: cv2.QRCodeEncoder missing — WHY: this opencv build lacks the QR "
            f"encoder — CHECK: use opencv-contrib-python, or pip install qrcode  ({exc})",
            file=sys.stderr,
        )
        sys.exit(3)
    qr = encoder.encode(payload)                 # NxN uint8, 0/255, no quiet zone
    if qr is None or qr.size == 0:
        print(f"FAIL: QR encode returned empty for payload {payload!r} — "
              f"WHY: unsupported payload — CHECK: use a short ascii id string",
              file=sys.stderr)
        sys.exit(3)
    big = cv2.resize(qr, (qr.shape[1] * QR_MODULE_PX, qr.shape[0] * QR_MODULE_PX),
                     interpolation=cv2.INTER_NEAREST)
    quiet = QR_QUIET_MODULES * QR_MODULE_PX
    big = cv2.copyMakeBorder(big, quiet, quiet, quiet, quiet,
                             cv2.BORDER_CONSTANT, value=255)
    return cv2.cvtColor(big, cv2.COLOR_GRAY2BGR)


def make_marker_png(marker_type: str, marker_id: int) -> np.ndarray:
    if marker_type == "aruco":
        return make_aruco_png(marker_id)
    return make_qr_png(str(marker_id))


# --------------------------------------------------------------------------- #
# SDF / config emission
# --------------------------------------------------------------------------- #
def _model_config(name: str, desc: str) -> str:
    return (
        '<?xml version="1.0"?>\n'
        "<model>\n"
        f"  <name>{name}</name>\n"
        "  <version>1.0</version>\n"
        "  <sdf version=\"1.9\">model.sdf</sdf>\n"
        f"  <description>{desc}</description>\n"
        "</model>\n"
    )


def _marker_material(name: str) -> str:
    """PBR/metal/albedo_map block (Harmonic) — model:// resolves via GZ_SIM_RESOURCE_PATH.

    The texture filename is UNIQUE per model (`<name>.png`, not a shared `marker.png`):
    ogre2's resource manager caches textures by BASENAME, so identical filenames in
    different model dirs collide and every marker renders the first-loaded one. Unique
    names are mandatory for distinct per-marker ids.
    """
    return (
        "        <material>\n"
        "          <diffuse>1 1 1 1</diffuse>\n"
        "          <specular>0 0 0 1</specular>\n"
        "          <pbr>\n"
        "            <metal>\n"
        f"              <albedo_map>model://{name}/materials/textures/{name}.png</albedo_map>\n"
        "              <metalness>0.0</metalness>\n"
        "              <roughness>1.0</roughness>\n"
        "            </metal>\n"
        "          </pbr>\n"
        "        </material>\n"
    )


def plane_sdf(name: str, size_m: float, marker_type: str, marker_id: int) -> str:
    """A STATIC textured plane at z=0.001 (bare marker or landing pad)."""
    return (
        '<?xml version="1.0"?>\n'
        '<sdf version="1.9">\n'
        f'  <model name="{name}">\n'
        f"    <!-- SIM-3 {marker_type} id={marker_id}, {size_m*100:.0f}cm static plane."
        " Generated by sim/gen_markers.py. -->\n"
        "    <static>true</static>\n"
        '    <link name="link">\n'
        "      <pose>0 0 0.001 0 0 0</pose>\n"
        '      <visual name="marker_visual">\n'
        "        <geometry>\n"
        f"          <plane><normal>0 0 1</normal><size>{size_m:.4f} {size_m:.4f}</size></plane>\n"
        "        </geometry>\n"
        f"{_marker_material(name)}"
        "      </visual>\n"
        "    </link>\n"
        "  </model>\n"
        "</sdf>\n"
    )


def robot_sdf(name: str, size_m: float, marker_type: str, marker_id: int) -> str:
    """Chassis (~0.4x0.3x0.25 m) + welded top marker + VelocityControl on cmd_vel.

    The marker is a <visual> on base_link => rigidly fixed (no joint, no nesting,
    no solver cost; only rendered, never simulated). VelocityControl applies the
    bridged ROS Twist (body-frame linear + yaw rate) to the model, kinematically —
    deterministic for the two-run same-ID-set check (no wheel-contact jitter).
    """
    cx, cy, cz = 0.40, 0.30, 0.25                 # chassis box size (m)
    base_z = cz / 2.0 + 0.005                      # bottom 5 mm above ground
    marker_z = cz / 2.0 + 0.0005                   # marker just above the box top face
    return (
        '<?xml version="1.0"?>\n'
        '<sdf version="1.9">\n'
        f'  <model name="{name}">\n'
        f"    <!-- SIM-3 convoy robot carrying {marker_type} id={marker_id}"
        f" ({size_m*100:.0f}cm). Generated by sim/gen_markers.py. -->\n"
        f'    <link name="base_link">\n'
        f"      <pose>0 0 {base_z:.4f} 0 0 0</pose>\n"
        "      <inertial>\n"
        "        <mass>5.0</mass>\n"
        "        <inertia><ixx>0.08</ixx><iyy>0.12</iyy><izz>0.15</izz>"
        "<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>\n"
        "      </inertial>\n"
        '      <collision name="chassis_col">\n'
        f"        <geometry><box><size>{cx} {cy} {cz}</size></box></geometry>\n"
        "      </collision>\n"
        '      <visual name="chassis_vis">\n'
        f"        <geometry><box><size>{cx} {cy} {cz}</size></box></geometry>\n"
        "        <material><diffuse>0.15 0.15 0.18 1</diffuse></material>\n"
        "      </visual>\n"
        '      <visual name="marker_top">\n'
        f"        <pose>0 0 {marker_z:.4f} 0 0 0</pose>\n"
        "        <geometry>\n"
        f"          <plane><normal>0 0 1</normal><size>{size_m:.4f} {size_m:.4f}</size></plane>\n"
        "        </geometry>\n"
        f"{_marker_material(name)}"
        "      </visual>\n"
        "    </link>\n"
        '    <plugin filename="gz-sim-velocity-control-system"\n'
        '            name="gz::sim::systems::VelocityControl">\n'
        f"      <topic>/model/{name}/cmd_vel</topic>\n"
        "      <!-- hold still (no gravity fall) until the rclpy driver sends cmd_vel;\n"
        "           linear is body-frame, so constant linear.x + angular.z => a circle. -->\n"
        "      <initial_linear>0 0 0</initial_linear>\n"
        "      <initial_angular>0 0 0</initial_angular>\n"
        "    </plugin>\n"
        "  </model>\n"
        "</sdf>\n"
    )


# --------------------------------------------------------------------------- #
# driver
# --------------------------------------------------------------------------- #
def write_model(out_dir: str, name: str, sdf_text: str, desc: str,
                png: np.ndarray) -> str:
    model_dir = os.path.join(out_dir, name)
    tex_dir = os.path.join(model_dir, "materials", "textures")
    os.makedirs(tex_dir, exist_ok=True)
    if not cv2.imwrite(os.path.join(tex_dir, f"{name}.png"), png):
        print(f"FAIL: cv2.imwrite failed for {name} — WHY: bad path/permissions — "
              f"CHECK: {tex_dir} writable", file=sys.stderr)
        sys.exit(4)
    with open(os.path.join(model_dir, "model.sdf"), "w", encoding="utf-8", newline="\n") as fh:
        fh.write(sdf_text)
    with open(os.path.join(model_dir, "model.config"), "w", encoding="utf-8", newline="\n") as fh:
        fh.write(_model_config(name, desc))
    return model_dir


def main() -> int:
    ap = argparse.ArgumentParser(description="SIM-3 marker/robot gz asset generator")
    ap.add_argument("--type", choices=["aruco", "qr"], required=True,
                    help="marker texture family")
    ap.add_argument("--kind", choices=["plane", "robot"], default="plane",
                    help="plane = bare marker / pad; robot = chassis + welded marker")
    ap.add_argument("--ids", type=int, nargs="+", required=True, help="marker ids")
    ap.add_argument("--size-cm", type=float, default=20.0,
                    help="marker edge length in cm (default 20; pads use a larger value)")
    ap.add_argument("--prefix", default=None,
                    help="model-dir name prefix; default convoy_robot (robot) / marker (plane)")
    ap.add_argument("--out", default=DEFAULT_OUT, help="output models dir")
    args = ap.parse_args()

    if args.size_cm <= 0:
        print("FAIL: --size-cm must be > 0 — CHECK the value", file=sys.stderr)
        return 2
    size_m = args.size_cm / 100.0
    prefix = args.prefix or ("convoy_robot" if args.kind == "robot" else "marker")

    written = []
    for marker_id in args.ids:
        name = f"{prefix}_{marker_id}"
        png = make_marker_png(args.type, marker_id)
        if args.kind == "robot":
            sdf_text = robot_sdf(name, size_m, args.type, marker_id)
            desc = f"SIM-3 convoy robot, {args.type} id={marker_id}, {args.size_cm:.0f}cm marker"
        else:
            sdf_text = plane_sdf(name, size_m, args.type, marker_id)
            desc = f"SIM-3 {args.type} marker plane id={marker_id}, {args.size_cm:.0f}cm"
        model_dir = write_model(args.out, name, sdf_text, desc, png)
        written.append((name, png.shape, model_dir))

    print(f"gen_markers: wrote {len(written)} {args.kind} model dir(s) "
          f"[{args.type}, {args.size_cm:.0f}cm] under {args.out}")
    for name, shape, _ in written:
        print(f"  {name:<20} texture {shape[1]}x{shape[0]} px")
    return 0


if __name__ == "__main__":
    sys.exit(main())
