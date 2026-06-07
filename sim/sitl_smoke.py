#!/usr/bin/env python3
"""Raw-MAVSDK environment smoke for SIM-0 ONLY (sim_sessions.md recap S5).

Validates the VM's PX4-SITL multi-instance environment WITHOUT any finals/ code:
connect -> health-ready -> arm -> takeoff ~1.5 m -> altitude >= 1.2 m -> land ->
landed + disarmed -> "PASS instance N". From SIM-1 on every flight goes through
`--profile sitl` (the sim flies the real mission code or it proves nothing) — this
harness is then BANNED for flight validation.

Usage (inside the repo venv, after `bash sim/launch_sitl.sh start N`):
    python sim/sitl_smoke.py --instance 0     # one instance
    python sim/sitl_smoke.py --all 3          # K instances concurrently

Port map: instance N -> MAVLink udpin://0.0.0.0:(14540+N), mavsdk_server gRPC
50051+N (one server per System, spawned by mavsdk itself). Every wait has a hard
deadline and fails with WHAT / WHICH instance / CHECK; exits nonzero on any failure.
"""

import argparse
import asyncio
import sys
import time

from mavsdk import System
from mavsdk.action import ActionError

# Deadlines sized for a 2-vCPU VM running up to 3 lockstep instances: sim time may
# run well below wall-clock (slow != wrong — sim_sessions.md recap S8), so these are
# deliberately generous. Every wait still has a hard deadline (recap S6).
CONNECT_DEADLINE_S = 30.0
HEALTH_DEADLINE_S = 120.0
CLIMB_DEADLINE_S = 90.0
LAND_DEADLINE_S = 180.0
DISARM_DEADLINE_S = 60.0
TAKEOFF_ALT_M = 1.5
ALT_OK_M = 1.2


class SmokeFailure(Exception):
    """One failed smoke step: WHAT failed, WHICH instance, elapsed, WHAT-TO-CHECK."""

    def __init__(self, instance: int, what: str, elapsed_s: float, check: str):
        super().__init__(
            f"FAIL instance {instance}: {what} after {elapsed_s:.1f}s — CHECK: {check}"
        )


async def _await_condition(aiter, predicate, *, instance, what, deadline_s, check):
    """Return the first item of `aiter` satisfying `predicate`, else SmokeFailure."""
    t0 = time.monotonic()

    async def _scan():
        async for item in aiter:
            if predicate(item):
                return item
        raise SmokeFailure(
            instance, f"{what} (telemetry stream ended)", time.monotonic() - t0, check
        )

    try:
        return await asyncio.wait_for(_scan(), timeout=deadline_s)
    except asyncio.TimeoutError:
        raise SmokeFailure(instance, what, time.monotonic() - t0, check) from None


async def _act(coro, *, instance, what, check):
    """Run one action RPC, converting ActionError into a loud SmokeFailure."""
    t0 = time.monotonic()
    try:
        await coro
    except ActionError as exc:
        raise SmokeFailure(
            instance, f"{what} rejected: {exc}", time.monotonic() - t0, check
        ) from exc


async def smoke(instance: int) -> None:
    udp_port = 14540 + instance
    grpc_port = 50051 + instance
    log_hint = f"tail sim/run/px4_{instance}.log"

    drone = System(port=grpc_port)
    await drone.connect(system_address=f"udpin://0.0.0.0:{udp_port}")

    await _await_condition(
        drone.core.connection_state(),
        lambda s: s.is_connected,
        instance=instance,
        what=f"no MAVLink heartbeat on udpin://0.0.0.0:{udp_port}",
        deadline_s=CONNECT_DEADLINE_S,
        check=f"bash sim/launch_sitl.sh status; {log_hint}",
    )
    print(f"instance {instance}: connected on {udp_port} (gRPC {grpc_port})")

    await _await_condition(
        drone.telemetry.health(),
        lambda h: h.is_global_position_ok and h.is_home_position_ok and h.is_armable,
        instance=instance,
        what="health never ready (need global pos + home pos + armable)",
        deadline_s=HEALTH_DEADLINE_S,
        check=f"EKF settles slowly under multi-instance load; {log_hint}",
    )
    print(f"instance {instance}: health ready")

    await _act(
        drone.action.set_takeoff_altitude(TAKEOFF_ALT_M),
        instance=instance,
        what="set_takeoff_altitude",
        check=log_hint,
    )
    await _act(
        drone.action.arm(),
        instance=instance,
        what="arm",
        check=f"vehicle still armed from a previous run? {log_hint}",
    )

    # Past this point a failure may leave the vehicle airborne — best-effort land in
    # the except path so back-to-back runs don't need a manual cleanup.
    try:
        await _act(
            drone.action.takeoff(),
            instance=instance,
            what="takeoff",
            check=log_hint,
        )
        await _await_condition(
            drone.telemetry.position(),
            lambda p: p.relative_altitude_m >= ALT_OK_M,
            instance=instance,
            what=f"altitude never reached {ALT_OK_M} m (target {TAKEOFF_ALT_M} m)",
            deadline_s=CLIMB_DEADLINE_S,
            check=f"low RTF stretches climbs in wall-clock; {log_hint}",
        )
        print(f"instance {instance}: altitude >= {ALT_OK_M} m")

        await _act(
            drone.action.land(), instance=instance, what="land", check=log_hint
        )
        await _await_condition(
            drone.telemetry.in_air(),
            lambda in_air: not in_air,
            instance=instance,
            what="never landed (in_air stayed True)",
            deadline_s=LAND_DEADLINE_S,
            check=log_hint,
        )
        await _await_condition(
            drone.telemetry.armed(),
            lambda armed: not armed,
            instance=instance,
            what="never auto-disarmed after landing",
            deadline_s=DISARM_DEADLINE_S,
            check=f"COM_DISARM_LAND; {log_hint}",
        )
    except SmokeFailure:
        try:
            await asyncio.wait_for(drone.action.land(), timeout=10.0)
            print(
                f"instance {instance}: best-effort land sent after failure",
                file=sys.stderr,
            )
        except Exception:
            pass  # the original SmokeFailure is the story; cleanup is best-effort
        raise

    print(f"PASS instance {instance}")


async def run_all(count: int) -> int:
    results = await asyncio.gather(
        *(smoke(i) for i in range(count)), return_exceptions=True
    )
    failures = [r for r in results if isinstance(r, BaseException)]
    for failure in failures:
        print(failure, file=sys.stderr)
    if failures:
        print(
            f"{len(failures)}/{count} instance(s) FAILED — see messages above",
            file=sys.stderr,
        )
        return 1
    print(f"ALL {count} instances PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="SIM-0 raw-MAVSDK SITL environment smoke (see module docstring)"
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--instance", type=int, help="smoke one instance N")
    group.add_argument("--all", type=int, metavar="K", help="smoke K instances concurrently")
    args = parser.parse_args()

    if args.all is not None:
        if args.all < 1:
            parser.error("--all needs K >= 1")
        return asyncio.run(run_all(args.all))
    try:
        asyncio.run(smoke(args.instance))
    except SmokeFailure as exc:
        print(exc, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
