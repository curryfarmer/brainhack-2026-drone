#!/usr/bin/env python3
"""SIM-3 convoy driver — rclpy Twist publisher over ros_gz (the SIM-0 ros_gz verdict).

Drives the 5 convoy robots along a DETERMINISTIC, open-loop (no feedback, no random)
velocity schedule. The default schedule is a single constant segment = a circle:
VelocityControl reads body-frame linear.x + yaw-rate angular.z, so a constant
(linear.x, angular.z) traces a circle of radius linear.x / angular.z. Each robot gets
the SAME command; the phase offset that makes them a convoy lives in the spawn poses in
sim/worlds/convoy.sdf. Two runs replay the identical schedule => the same marker-ID set
crosses each camera band (the two-run determinism the SIM-3 smoke asserts).

Path flows ROS -> gz: this node publishes geometry_msgs/Twist on
/model/convoy_robot_<id>/cmd_vel; one ros_gz bridge maps it to gz.msgs.Twist that the
VelocityControl plugins subscribe:

    source /opt/ros/humble/setup.bash
    ros2 run ros_gz_bridge parameter_bridge \\
      /model/convoy_robot_7/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist ...(x5)
    python3 sim/convoy_driver.py --duration-s 60

Runs under the ROS-sourced interpreter (system 3.10), NOT the .venv. This file is in
sim/ BY DESIGN (outside the finals conventions/SDK scan) so raw rclpy is allowed.
Fail-loud: rclpy.init failure (ROS not sourced) exits nonzero with WHAT/WHY/CHECK; the
run is deadline-bounded and stops all robots on clean exit.
"""

import argparse
import sys
import time

DEFAULT_IDS = [7, 11, 23, 42, 88]

# Default = circle: linear.x = LINEAR (body forward), angular.z = ANGULAR (yaw rate).
# radius = LINEAR / ANGULAR. With the convoy.sdf spawn poses this is a 2 m circle through
# the origin tower. A timed waypoint ring can replace _vel_at() (see ROUTE below) — keep it
# a PURE function of elapsed time so two runs stay identical.
ROUTE = None  # None => constant (LINEAR, ANGULAR); else list of (duration_s, v, w) segments


def parse_route(spec: str):
    """Parse a --route spec 'dur,v,w; dur,v,w; ...' into [(dur, v, w), ...].

    Each segment is `duration_s, linear_mps, angular_radps` (body-frame, the
    same VelocityControl inputs vel_at feeds). Segments are separated by ';',
    fields by ','. The car drives segment 1 for its duration, then segment 2,
    ... then the whole list REPEATS (vel_at takes elapsed % period) — so an
    irregular but DETERMINISTIC snaking path (two runs identical, the SIM-3
    determinism contract). Used by the followbox_multi warm-up to weave the
    convoy between crates.

    Fail-loud: a malformed spec raises ValueError naming WHAT was wrong and the
    expected shape — never a silently-dropped segment that would change the
    path between runs. duration must be > 0 (a 0-s segment is dead config that
    would make `elapsed % period` skip it inconsistently)."""
    if not isinstance(spec, str) or not spec.strip():
        raise ValueError(
            "route spec must be a non-empty string 'dur,v,w; dur,v,w; ...' "
            f"(got {spec!r})")
    segments = []
    for idx, raw in enumerate(s for s in spec.split(";") if s.strip()):
        fields = [f.strip() for f in raw.split(",")]
        if len(fields) != 3:
            raise ValueError(
                f"route segment {idx} {raw!r} must have exactly 3 fields "
                f"'duration_s,linear_mps,angular_radps' — got {len(fields)}")
        try:
            dur, v, w = (float(f) for f in fields)
        except ValueError as exc:
            raise ValueError(
                f"route segment {idx} {raw!r} has a non-numeric field — "
                f"expected 'duration_s,linear_mps,angular_radps' ({exc})"
            ) from exc
        if not dur > 0:
            raise ValueError(
                f"route segment {idx} {raw!r}: duration_s must be > 0, got "
                f"{dur} (a zero/negative segment is dead config)")
        segments.append((dur, v, w))
    if not segments:
        raise ValueError(f"route spec {spec!r} parsed to zero segments")
    return segments


def vel_at(elapsed_s: float, linear: float, angular: float, delay_s: float = 0.0):
    # Hold still until delay_s (still a PURE function of elapsed -> two runs
    # identical). S11 uses this so the straight-lane cars stay at their spawns
    # through the drones' EKF settle and only start driving once the scan is
    # live — otherwise they drive out of the nadir footprints before takeoff.
    if elapsed_s < delay_s:
        return 0.0, 0.0
    if ROUTE is None:
        return linear, angular
    period = sum(d for d, _, _ in ROUTE)
    tt = elapsed_s % period
    acc = 0.0
    for dur, v, w in ROUTE:
        if tt < acc + dur:
            return v, w
        acc += dur
    return 0.0, 0.0


def main() -> int:
    ap = argparse.ArgumentParser(description="SIM-3 deterministic convoy driver")
    ap.add_argument("--ids", type=int, nargs="+", default=DEFAULT_IDS)
    ap.add_argument("--rate-hz", type=float, default=20.0)
    ap.add_argument("--duration-s", type=float, default=60.0)
    ap.add_argument("--linear", type=float, default=0.4, help="body-frame forward m/s")
    ap.add_argument("--angular", type=float, default=0.2, help="yaw rate rad/s (radius=lin/ang)")
    ap.add_argument("--delay-s", type=float, default=0.0,
                    help="hold at spawn (zero velocity) for the first N s, then drive")
    ap.add_argument("--route", type=str, default=None,
                    help="irregular path 'dur,v,w; dur,v,w; ...' (body-frame "
                         "duration_s,linear_mps,angular_radps; repeats). "
                         "Overrides --linear/--angular when set.")
    args = ap.parse_args()

    global ROUTE
    if args.route is not None:
        try:
            ROUTE = parse_route(args.route)
        except ValueError as exc:
            print(f"FAIL: --route {args.route!r} — {exc}", file=sys.stderr)
            return 2
        print(f"convoy_driver: route = {len(ROUTE)} segments "
              f"(period {sum(d for d, _, _ in ROUTE):.0f}s): {ROUTE}")

    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
    except Exception as exc:  # noqa: BLE001 - loud, specific
        print(f"FAIL: cannot import rclpy/geometry_msgs — WHY: ROS 2 not sourced — "
              f"CHECK: source /opt/ros/humble/setup.bash  ({exc})", file=sys.stderr)
        return 2

    try:
        rclpy.init()
    except Exception as exc:  # noqa: BLE001
        print(f"FAIL: rclpy.init() — WHY: ROS 2 env broken — "
              f"CHECK: source /opt/ros/humble/setup.bash  ({exc})", file=sys.stderr)
        return 2

    node = Node("sim3_convoy_driver")
    pubs = {i: node.create_publisher(Twist, f"/model/convoy_robot_{i}/cmd_vel", 10)
            for i in args.ids}
    radius = args.linear / args.angular if args.angular else float("inf")
    print(f"convoy_driver: {len(pubs)} robots, linear={args.linear} m/s angular={args.angular} "
          f"rad/s (radius {radius:.2f} m), delay={args.delay_s:.0f}s, "
          f"{args.duration_s:.0f}s @ {args.rate_hz:.0f} Hz")

    period = 1.0 / args.rate_hz
    t_start = time.monotonic()
    deadline = t_start + args.duration_s
    try:
        while time.monotonic() < deadline:                 # deadline-bounded (convention)
            v, w = vel_at(time.monotonic() - t_start, args.linear, args.angular,
                          args.delay_s)
            msg = Twist()
            msg.linear.x = float(v)
            msg.angular.z = float(w)
            for pub in pubs.values():
                pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
        stop = Twist()
        for pub in pubs.values():
            pub.publish(stop)                              # halt convoy on clean exit
        time.sleep(0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(f"convoy_driver: done ({args.duration_s:.0f}s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
