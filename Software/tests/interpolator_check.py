#!/usr/bin/env python3
"""Property tests for the motion profile, run against the REAL firmware source.

    python3 Software/tests/interpolator_check.py [--cc gcc] [--ssh arm]

WHY THIS EXISTS
    A 32 KB AVR cannot host a test framework, and the interpolator is the code
    that decides how fast the arm moves. It is not something to change and then
    find out about at the bench with a loaded arm.

    So this lifts profileStepC() VERBATIM out of factorylm_arm_controller.ino,
    between its two marker comments, compiles that exact text with a host C
    compiler, and hammers it. Same pattern as nowheel_check.js lifting noWheel()
    out of arm-console.html: the test runs the shipping code, not a
    reimplementation of it that can drift.

    If no local compiler is found it will build over ssh on the Pi, which has
    gcc. Nothing here talks to the board, the bridge, or a servo.

WHAT IT PROVES
    1. A step never exceeds MAX_STEP_C, whatever the elapsed time. That ceiling
       exists so a stalled host cannot produce one huge catch-up write.
    2. Velocity never exceeds the operator's commanded deg/s.
    3. It never steps past the target, in either direction.
    4. It always converges - from any start, any distance, any speed - and does
       not creep forever a few centidegrees short.
    5. It accelerates rather than jumping: the first step of a move is small,
       which is the whole point of the change.
    6. It decelerates into the target rather than stopping dead.
    7. A mid-move SPD reduction is obeyed immediately.
"""

import argparse
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[2]
INO = ROOT / "Software" / "factorylm_arm_controller" / "factorylm_arm_controller.ino"

START = ">>> INTERPOLATOR-PROFILE"
END = "<<< INTERPOLATOR-PROFILE"

HARNESS = r"""
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

/* ---- lifted verbatim from the firmware ---- */
%(lifted)s
/* ---- end lifted ---- */

static int fails = 0;
static void bad(const char *what, long a, long b, long c, long d) {
    printf("  FAIL  %%s  (%%ld %%ld %%ld %%ld)\n", what, a, b, c, d);
    fails++;
}

int main(void) {
    /* 1-4: drive a move to completion from every combination that matters. */
    int dpss[] = {1, 5, 30, 90};
    long dists[] = {1, 5, 50, 200, 1000, 9100, 18000};
    unsigned long els[] = {1, 20, 55, 200};
    int signs[] = {1, -1};

    for (int a = 0; a < 4; a++)
    for (int b = 0; b < 7; b++)
    for (int c = 0; c < 4; c++)
    for (int s = 0; s < 2; s++) {
        uint8_t dps = dpss[a];
        long dist = dists[b];
        unsigned long el = els[c];
        long set = 0, tgt = signs[s] * dist;
        int16_t vel = 0;
        long ticks = 0, maxseen = 0;
        long firststep = -1;

        while (set != tgt) {
            long st = profileStepC(set, tgt, dps, el, &vel);
            long mag = st < 0 ? -st : st;
            if (firststep < 0) firststep = mag;
            if (mag > maxseen) maxseen = mag;

            if (mag > MAX_STEP_C) bad("step exceeded MAX_STEP_C", mag, dps, dist, el);
            if (vel > (long)dps * 100) bad("velocity exceeded dps", vel, dps, dist, el);
            if (vel < 0) bad("velocity went negative", vel, dps, dist, el);

            long before = set;
            set += st;
            /* never step past the target */
            if (signs[s] > 0 && set > tgt) bad("overshot upward", before, set, tgt, dps);
            if (signs[s] < 0 && set < tgt) bad("overshot downward", before, set, tgt, dps);

            if (++ticks > 5000000L) { bad("did not converge", dps, dist, el, signs[s]); break; }
        }
        if (set != tgt) bad("stopped short of target", set, tgt, dps, dist);

        /* 5: it ramps. The first step of a long move at a high speed must be a
           small fraction of the cruise step - that is the jerk that was removed.
           Only meaningful where the ramp has room, i.e. a long move. */
        if (dist >= 1000 && dps >= 30 && el == 20) {
            long cruise = ((long)dps * 100 * (long)el) / 1000;
            if (cruise > MAX_STEP_C) cruise = MAX_STEP_C;
            if (firststep > cruise / 2)
                bad("first step was not a ramp", firststep, cruise, dps, dist);
        }
    }

    /* 6: decelerating into the target - the last steps of a long move must be
       smaller than the middle ones. */
    {
        int16_t vel = 0;
        long set = 0, tgt = 6000;
        long prev = 0, mid = 0, last = 0, n = 0;
        while (set != tgt) {
            long st = profileStepC(set, tgt, 60, 20, &vel);
            set += st; n++;
            if (n == 40) mid = st;
            prev = last; last = st;
        }
        (void)prev;
        if (mid == 0 || last >= mid) bad("did not decelerate into the target", mid, last, n, 0);
    }

    /* 7: lowering SPD mid-move takes effect at once. */
    {
        int16_t vel = 0;
        long set = 0, tgt = 18000;
        /* 60 ticks: past the ~23-tick ramp to 90 deg/s and still well inside the
           cruise, which ends around tick 100. Running to 200 would finish the
           move and park the velocity at 0, which is what this used to do. */
        for (int i = 0; i < 60; i++) set += profileStepC(set, tgt, 90, 20, &vel);
        if (vel <= 3000) bad("never reached the high speed", vel, set, 0, 0);
        long st = profileStepC(set, tgt, 30, 20, &vel);
        (void)st;
        if (vel > 3000) bad("SPD reduction ignored", vel, 3000, 0, 0);
    }

    /* 8: a zero-length move is a no-op and parks the velocity. */
    {
        int16_t vel = 4321;
        long st = profileStepC(500, 500, 30, 20, &vel);
        if (st != 0 || vel != 0) bad("zero-distance move was not a no-op", st, vel, 0, 0);
    }

    if (fails) { printf("INTERPOLATOR_FAIL (%%d)\n", fails); return 1; }
    printf("  PASS  step never exceeds MAX_STEP_C\n");
    printf("  PASS  velocity never exceeds the commanded deg/s\n");
    printf("  PASS  never steps past the target\n");
    printf("  PASS  converges from every distance, speed and tick length\n");
    printf("  PASS  ramps up instead of jumping to full speed\n");
    printf("  PASS  decelerates into the target instead of stopping dead\n");
    printf("  PASS  a mid-move SPD reduction is obeyed at once\n");
    printf("INTERPOLATOR_PASS\n");
    return 0;
}
"""


def lift(src: str) -> str:
    """Pull the marked block out of the .ino, exactly as written."""
    i = src.find(START)
    k = src.find(END)
    if i < 0 or k < 0 or k < i:
        sys.exit("FAIL: the INTERPOLATOR-PROFILE markers are not in %s" % INO)
    block = src[src.index("\n", i) + 1:src.rfind("\n", i, k)]
    if "profileStepC" not in block:
        sys.exit("FAIL: profileStepC is not inside the markers")

    # MAX_STEP_C is declared above the markers, with the other motion constants
    # where it belongs. Lift its REAL declaration rather than restating 200 here:
    # a copy in the test would keep passing after somebody changed the firmware.
    m = re.search(r"^const\s+int16_t\s+MAX_STEP_C\s*=\s*[^;]+;", src, re.M)
    if not m:
        sys.exit("FAIL: could not find the MAX_STEP_C declaration in %s" % INO)
    return m.group(0) + "\n" + block


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cc", default=None, help="host C compiler (default: auto)")
    ap.add_argument("--ssh", default="arm", help="ssh host to build on if there is no local cc")
    args = ap.parse_args()

    src = INO.read_text(encoding="utf-8", errors="replace").replace("\r\n", "\n")
    prog = HARNESS % {"lifted": lift(src)}

    cc = args.cc or shutil.which("gcc") or shutil.which("cc") or shutil.which("clang")
    if cc:
        with tempfile.TemporaryDirectory() as td:
            c = pathlib.Path(td) / "t.c"
            exe = pathlib.Path(td) / "t.exe"
            c.write_text(prog, encoding="utf-8")
            r = subprocess.run([cc, "-O1", "-Wall", "-Wextra", "-Werror",
                                str(c), "-o", str(exe)],
                               capture_output=True, text=True)
            if r.returncode:
                print(r.stdout + r.stderr)
                print("INTERPOLATOR_FAIL (the lifted firmware code did not compile)")
                return 1
            r = subprocess.run([str(exe)], capture_output=True, text=True)
            print(r.stdout + r.stderr, end="")
            return r.returncode

    # No local compiler: build it on the Pi, which has one.
    print("  (no local C compiler - building on %s)" % args.ssh)
    remote = ("cat > /tmp/interp_check.c && gcc -O1 -Wall -Wextra -Werror "
              "/tmp/interp_check.c -o /tmp/interp_check && /tmp/interp_check; "
              "rc=$?; rm -f /tmp/interp_check.c /tmp/interp_check; exit $rc")
    r = subprocess.run(["ssh", "-o", "BatchMode=yes", args.ssh, remote],
                       input=prog, capture_output=True, text=True)
    print(r.stdout + r.stderr, end="")
    if r.returncode:
        print("INTERPOLATOR_FAIL (build or test failed on %s)" % args.ssh)
    return r.returncode


if __name__ == "__main__":
    sys.exit(main())
