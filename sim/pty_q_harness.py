#!/usr/bin/env python3
"""pty_q_harness — run a command under a PTY and inject the finals operator-abort
key ('q'+Enter) once the swarm is airborne. For SIM-5 Drill A on the headless VM.

WHY: finals' AbortListener only arms when sys.stdin.isatty() is true. Plain ssh
(or any pipe) gives a non-tty stdin, so finals prints "stdin EOF — abort key
disabled" and the 'q' drill can't run. `expect` is not installed on the VM. This
harness uses pty.fork() so the child's stdin IS a terminal; the parent relays the
child's output to our stdout, watches for the swarm going airborne, then writes
'q\n' to the pty master (= the child's stdin). The child's exit code propagates.

USAGE (via run_vision.sh abort3, but standalone too):
  PYTHONNOUSERSITE=1 python3 sim/pty_q_harness.py \
      --trigger-regex 'offboard active' --trigger-count 3 --fallback-secs 80 -- \
      .venv/bin/python -m finals.main --profile sitl \
      --config finals/configs/sitl3_vision.json --budget 360

Injection fires when the trigger regex has matched --trigger-count times (one
'offboard active' per drone = all airborne) OR after --fallback-secs, whichever
first, plus a short settle so the abort lands mid-scan not mid-takeoff.
"""
import argparse
import os
import pty
import re
import select
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser(description="run a cmd under a pty, inject 'q'")
    ap.add_argument("--trigger-regex", default="offboard active",
                    help="per-drone airborne marker in the child's output")
    ap.add_argument("--trigger-count", type=int, default=3,
                    help="inject once the regex has matched this many times")
    ap.add_argument("--fallback-secs", type=float, default=80.0,
                    help="inject anyway after this long if the trigger never hits")
    ap.add_argument("--settle-after-trigger-secs", type=float, default=4.0,
                    help="wait this long after the trigger before injecting "
                         "(so the abort lands mid-scan, not mid-takeoff)")
    ap.add_argument("cmd", nargs=argparse.REMAINDER,
                    help="-- then the command to run (e.g. -- .venv/bin/python ...)")
    args = ap.parse_args()

    cmd = args.cmd[1:] if args.cmd and args.cmd[0] == "--" else args.cmd
    if not cmd:
        print("FAIL: no command after '--' — WHAT: pty_q_harness needs a child "
              "command to run", file=sys.stderr)
        return 64

    rx = re.compile(args.trigger_regex)
    pid, fd = pty.fork()
    if pid == 0:                       # child: become the command
        os.execvp(cmd[0], cmd)
        os._exit(127)                  # only if execvp failed

    t0 = time.monotonic()
    count = 0
    inject_at = None
    injected = False
    buf = b""
    while True:
        now = time.monotonic()
        if not injected:
            if inject_at is None and (count >= args.trigger_count
                                      or now - t0 >= args.fallback_secs):
                why = (f"trigger x{count}" if count >= args.trigger_count
                       else f"fallback {args.fallback_secs:.0f}s")
                inject_at = now + args.settle_after_trigger_secs
                sys.stdout.write(
                    f"\n[pty_q_harness] swarm airborne ({why}) -> injecting 'q' "
                    f"in {args.settle_after_trigger_secs:.0f}s\n")
                sys.stdout.flush()
            elif inject_at is not None and now >= inject_at:
                os.write(fd, b"q\n")
                injected = True
                sys.stdout.write("[pty_q_harness] injected 'q'+Enter "
                                 "(operator abort -> LAND ALL)\n")
                sys.stdout.flush()

        r, _, _ = select.select([fd], [], [], 0.2)
        if fd in r:
            try:
                data = os.read(fd, 4096)
            except OSError:            # slave closed = child exiting
                break
            if not data:
                break
            os.write(1, data)          # relay raw to our stdout
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if rx.search(line.decode("utf-8", "replace")):
                    count += 1

    _, status = os.waitpid(pid, 0)
    code = os.waitstatus_to_exitcode(status)
    sys.stdout.write(f"[pty_q_harness] child exit code {code}\n")
    sys.stdout.flush()
    return code


if __name__ == "__main__":
    sys.exit(main())
