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
    8. A REVERSAL brakes through zero instead of flipping sign at speed, and no
       tick changes velocity faster than that joint's acceleration allows. Cases
       1-7 all start at vel = 0, so none of them ever reversed - which is how the
       firmware shipped a reversal that went +30 to -30 deg/s in one 20 ms tick.
    9. Acceleration is per joint and measurably changes the ramp.
   10. Changing acceleration mid-move cannot cause an overshoot - which is what
       makes it safe to accept ACC on a live joint, unlike LIM and MIR.
   11. Slamming acceleration to its gentlest setting DURING a reversal brake -
       reachable by dragging the console's smoothness slider - still converges,
       stays inside the commanded speed, and never emits an away-side step
       larger than MAX_STEP_C.
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

/* The acceleration the pre-existing cases run at. It is the firmware's own boot
   default, so cases 1-8 exercise exactly the behaviour they always did and a red
   result there means the signed-velocity change broke something real. */
#define ACC_TEST 200

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
            long st = profileStepC(set, tgt, dps, ACC_TEST, el, &vel);
            long mag = st < 0 ? -st : st;
            long avel = vel < 0 ? -vel : vel;
            if (firststep < 0) firststep = mag;
            if (mag > maxseen) maxseen = mag;

            if (mag > MAX_STEP_C) bad("step exceeded MAX_STEP_C", mag, dps, dist, el);
            if (avel > (long)dps * 100) bad("velocity exceeded dps", vel, dps, dist, el);
            /* velC IS SIGNED NOW, so "never negative" is no longer the invariant
               - it was an artifact of the old magnitude-only representation, and
               a downward move legitimately carries a negative velocity. The real
               invariant, which is strictly stronger, is that the sign of the
               velocity always agrees with the direction of travel. */
            if (signs[s] > 0 && vel < 0) bad("velocity sign disagrees with +move", vel, dps, dist, el);
            if (signs[s] < 0 && vel > 0) bad("velocity sign disagrees with -move", vel, dps, dist, el);

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
            long st = profileStepC(set, tgt, 60, ACC_TEST, 20, &vel);
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
        for (int i = 0; i < 60; i++) set += profileStepC(set, tgt, 90, ACC_TEST, 20, &vel);
        if (vel <= 3000) bad("never reached the high speed", vel, set, 0, 0);
        long st = profileStepC(set, tgt, 30, ACC_TEST, 20, &vel);
        (void)st;
        if (vel > 3000) bad("SPD reduction ignored", vel, 3000, 0, 0);
    }

    /* 8: a zero-length move is a no-op and parks the velocity. */
    {
        int16_t vel = 4321;
        long st = profileStepC(500, 500, 30, ACC_TEST, 20, &vel);
        if (st != 0 || vel != 0) bad("zero-distance move was not a no-op", st, vel, 0, 0);
    }

    /* 9: THE REVERSAL. This is the case the original 224 never reached, because
       every one of them starts at vel = 0. A joint at cruise whose target moves
       behind it must brake through zero before it travels the other way - never
       flip sign at speed, which lands the arm's whole momentum on the opposite
       gear faces in one tick. Both directions: the bug was symmetric. */
    for (int s = 0; s < 2; s++) {
        int sign = s ? -1 : 1;
        int16_t vel = 0;
        long set = 0, tgt = sign * 9000;

        for (int i = 0; i < 60; i++) set += profileStepC(set, tgt, 30, ACC_TEST, 20, &vel);
        long cruise = vel;
        if ((sign > 0 && cruise < 2900) || (sign < 0 && cruise > -2900))
            bad("never reached cruise before the reversal", cruise, sign, 0, 0);

        /* Flip the target to the far side. Nothing else changes. */
        tgt = -sign * 9000;
        int16_t prev = vel;
        int crossed = 0;
        for (int i = 0; i < 400; i++) {
            long st = profileStepC(set, tgt, 30, ACC_TEST, 20, &vel);
            set += st;

            /* The sign must never invert without standing at zero on the way. */
            if (prev > 0 && vel < 0) bad("velocity flipped +to- without passing zero", prev, vel, sign, i);
            if (prev < 0 && vel > 0) bad("velocity flipped -to+ without passing zero", prev, vel, sign, i);
            if (vel == 0) crossed = 1;

            /* And the change per tick must respect the acceleration limit. At
               20 ms and 200 deg/s^2 that is 400 cd/s, plus one for the dv floor. */
            long dvel = (long)vel - (long)prev;
            if (dvel < 0) dvel = -dvel;
            if (dvel > 401) bad("velocity changed faster than ACC allows", dvel, prev, vel, i);

            prev = vel;
            if (set == tgt) break;
        }
        if (!crossed) bad("velocity never passed through zero on reversal", cruise, vel, sign, 0);
    }

    /* 10: acceleration is PER JOINT and actually does something. A gentle wrist
       value must ramp visibly slower than the brisk shoulder default - this is
       what protects the small printed gear. */
    {
        int16_t vgentle = 0, vbrisk = 0;
        long sg = 0, sb = 0, tgt = 18000;
        for (int i = 0; i < 10; i++) {
            sg += profileStepC(sg, tgt, 90, 60,  20, &vgentle);   /* wrist-ish */
            sb += profileStepC(sb, tgt, 90, 200, 20, &vbrisk);    /* default   */
        }
        if (vgentle >= vbrisk)
            bad("a lower ACC did not ramp more gently", vgentle, vbrisk, 0, 0);
        /* 60 deg/s^2 for 200 ms is about 12 deg/s; 200 gives about 40. */
        if (vgentle > 1500 || vbrisk < 3000)
            bad("ramp rates do not match the requested accelerations", vgentle, vbrisk, 0, 0);
    }

    /* 11: changing ACC mid-move cannot cause an overshoot. ACC is accepted on a
       live joint precisely so smoothness can be tuned by watching the arm, so
       the no-overshoot rule must not depend on the acceleration staying put. */
    {
        int16_t vel = 0;
        long set = 0, tgt = 9000;
        for (int i = 0; i < 30; i++) set += profileStepC(set, tgt, 90, 200, 20, &vel);
        long ticks = 0;
        while (set != tgt) {
            set += profileStepC(set, tgt, 90, 5, 20, &vel);   /* slammed to the gentlest */
            if (set > tgt) { bad("overshot after a mid-move ACC change", set, tgt, vel, 0); break; }
            if (++ticks > 200000L) { bad("did not converge after a mid-move ACC change", set, tgt, vel, 0); break; }
        }
    }

    /* 12: THE SLIDER'S WORST CASE. ACC is accepted on a live joint so smoothness
       can be tuned by watching the arm, which means an operator can drag it to
       the gentlest setting DURING a reversal brake - the one place the required
       braking distance is already at its largest. At 90 deg/s the stopping
       distance goes from about 20 deg at ACC=200 to about 810 deg at ACC=5.

       The firmware cannot leave its envelope regardless, because writeJoint
       clamps at the point of write. What this asserts is the thing the clamp
       does NOT protect: that the profile still converges rather than hanging,
       that velocity stays inside the commanded ceiling, and that the away-side
       step - the only branch where MAX_STEP_C is the sole bound, since there is
       no remaining-distance to clamp against - never exceeds it. */
    {
        int16_t vel = 0;
        long set = 0, tgt = 18000;
        /* 60 ticks: past the ~23-tick ramp to 90 deg/s and still well inside the
           cruise. 200 would FINISH the move and park the velocity at 0, which is
           exactly what the first run of this case did. */
        for (int i = 0; i < 60; i++) set += profileStepC(set, tgt, 90, 200, 20, &vel);
        if (vel < 8900) bad("case 12 never reached cruise", vel, 0, 0, 0);

        tgt = -18000;                     /* reverse... */
        long ticks = 0, worst_away = 0, peak = 0;
        while (set != tgt) {
            /* ...and slam the smoothness slider to its gentlest mid-brake. */
            long st = profileStepC(set, tgt, 90, 5, 20, &vel);
            long mag = st < 0 ? -st : st;
            long avel = vel < 0 ? -vel : vel;
            if (avel > peak) peak = avel;

            if (st > 0 && mag > worst_away) worst_away = mag;   /* still heading away */
            if (mag > MAX_STEP_C) bad("step exceeded MAX_STEP_C during a gentle reversal", mag, st, vel, ticks);
            if (avel > 9000) bad("velocity exceeded 90 deg/s during a gentle reversal", vel, 0, 0, ticks);

            set += st;
            if (set < tgt) { bad("overshot the target during a gentle reversal", set, tgt, vel, ticks); break; }
            if (++ticks > 500000L) { bad("a gentle reversal never converged", set, tgt, vel, ticks); break; }
        }
        if (worst_away > MAX_STEP_C)
            bad("an away-side step exceeded MAX_STEP_C", worst_away, MAX_STEP_C, 0, 0);
        (void)peak;
    }

    if (fails) { printf("INTERPOLATOR_FAIL (%%d)\n", fails); return 1; }
    printf("  PASS  step never exceeds MAX_STEP_C\n");
    printf("  PASS  velocity never exceeds the commanded deg/s\n");
    printf("  PASS  never steps past the target\n");
    printf("  PASS  converges from every distance, speed and tick length\n");
    printf("  PASS  ramps up instead of jumping to full speed\n");
    printf("  PASS  decelerates into the target instead of stopping dead\n");
    printf("  PASS  a mid-move SPD reduction is obeyed at once\n");
    printf("  PASS  a reversal brakes through zero instead of flipping at speed\n");
    printf("  PASS  velocity never changes faster than the joint's ACC allows\n");
    printf("  PASS  per-joint ACC changes the ramp - gentle is measurably gentler\n");
    printf("  PASS  a mid-move ACC change cannot overshoot\n");
    printf("  PASS  slamming ACC to its gentlest mid-reversal still converges\n");
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
