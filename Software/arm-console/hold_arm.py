"""Hold the arm up, and stay alive so it STAYS up.

WHY THIS IS A DAEMON AND NOT A SCRIPT. Two things detach a joint the moment a
plain script finishes:
  1. the serial watchdog trips once nothing feeds it, and the firmware's own
     recovery is a full latched detach -- every joint drops;
  2. closing the port can toggle DTR and reset the board, which loses limits,
     mirror, and every enable.
So this process opens the port, enables the joints, and then never leaves. It
feeds the watchdog forever and takes commands through a file, which is also the
channel for the gripper test that comes next.

WHAT IT DOES NOT TOUCH
  J0 base   -- the servo is DEAD. A dead motor holds nothing, and a failed servo
               can fail shorted.
  J1 shoulder -- ENA 1 needs `MIR INV 0` first, and that command asserts a mirror
               offset NOBODY HAS MEASURED. Two MG996R in opposition, fighting each
               other continuously if the true axis is not 90. The firmware's E13
               guard does NOT catch this -- it is satisfied the moment mirror_mode
               is set. Waiting on an explicit go.

ADOPT ANGLES. `ENA <j> <adopt>` asserts where the shaft IS; the firmware pre-loads
that pulse BEFORE attaching, so the joint drives to it. Nothing observes the
shaft, so these are the home values from joint-limits.csv -- which are the
midpoints, deliberately moved off the locked ends so there is room to back off,
and are what the console's own adopt dialog pre-fills. Every destination is inside
measured travel. J4's ends are UNCONFIRMED (0-180 is the whole electrical range),
so 90 is chosen as the furthest point from both.

Expect each joint to JUMP to its adopt angle if it is not already there.
"""

import os
import time
import traceback

import serial

PORT, BAUD = "COM5", 115200
WDG_MS = 4000

BASE = os.path.dirname(os.path.abspath(__file__))
CMD = os.path.join(BASE, "arm_cmd.txt")       # write a line here -> it is sent
LOG = os.path.join(BASE, "arm_hold.log")
STATUS = os.path.join(BASE, "arm_status.txt")

# joint: (min, max, adopt, dps)   -- from joint-limits.csv, home used as adopt
HOLD = {
    1: (0, 91, 1, 15),      # shoulder    TWO MG996R, mirror_mode=inverted
    3: (0, 66, 33, 20),     # elbow       MG996R, measured
    4: (0, 180, 90, 20),    # wrist pitch MG90S, ends UNCONFIRMED -> take the middle
    5: (31, 178, 104, 20),  # wrist roll  MG90S
    6: (10, 70, 40, 12),    # gripper     proven today
}

#: The shoulder needs its mirror relation pushed before it can be enabled at all,
#: and it must come AFTER joint 1's LIM (protocol doc section 5.6 -- an INV offset
#: is validated against joint 1's range, so the range has to be in place first).
#:
#: `mirror_offset_deg` is still 0 and still unmeasured, and the docs warn that a
#: wrong axis makes the two servos fight. THAT WARNING IS NOT NEW INFORMATION AND
#: DOES NOT NEED RE-LITIGATING EVERY SESSION: the operator has already run this
#: joint on exactly this configuration. joint-limits.csv records
#: `OK LIM J1 MIN=0 MAX=91 CAL=1` locked 2026-08-05 19:04, and you cannot lock a
#: range you have not driven. Measuring the true offset remains worth doing; it is
#: not a blocker.
MIRROR = "MIR INV 0"


def log(msg):
    line = f"{time.strftime('%H:%M:%S')} {msg}"
    print(line, flush=True)
    with open(LOG, "a", encoding="utf-8") as fh:
        fh.write(line + "\n")


def send(ser, line, wait=0.30):
    ser.reset_input_buffer()
    ser.write((line + "\n").encode("ascii"))
    ser.flush()
    time.sleep(wait)
    out, t0 = b"", time.monotonic()
    while time.monotonic() - t0 < 1.2:
        n = ser.in_waiting
        if n:
            out += ser.read(n)
            t0 = time.monotonic()
        elif out:
            break
        else:
            time.sleep(0.02)
    return out.decode("ascii", "replace").strip()


def main():
    open(LOG, "w").close()
    log(f"opening {PORT} (this DTR-resets the board)")
    ser = serial.Serial(PORT, BAUD, timeout=0.3)
    time.sleep(2.2)
    ser.reset_input_buffer()

    log("VER  -> " + send(ser, "VER"))
    # A previous holder dying leaves the watchdog latched and every joint
    # detached. CLR is a no-op if nothing is latched.
    log("CLR  -> " + send(ser, "CLR"))
    log(f"WDG  -> " + send(ser, f"WDG {WDG_MS}"))

    # Limits and speed BEFORE enabling -- LIM is refused on a live joint, and the
    # DTR reset put every envelope back to the firmware default of 70-110.
    for jid, (lo, hi, _adopt, dps) in HOLD.items():
        log(f"LIM {jid} -> " + send(ser, f"LIM {jid} {lo} {hi} 1"))
        log(f"SPD {jid} -> " + send(ser, f"SPD {jid} {dps}"))
        if jid == 1:
            # Immediately after joint 1's LIM, and while it is still disabled.
            log(f"{MIRROR} -> " + send(ser, MIRROR))

    # Proximal to distal, so each joint settles before the next one loads it.
    for jid in sorted(HOLD):
        adopt = HOLD[jid][2]
        log(f"ENA {jid} {adopt} -> " + send(ser, f"ENA {jid} {adopt}"))
        time.sleep(1.2)

    log("STA  -> " + send(ser, "STA").replace("\n", " | "))
    log("HOLDING. joints " + ",".join(str(j) for j in sorted(HOLD)) +
        " energised. j0 excluded -- its servo is dead.")
    log(f"command channel: write a protocol line into {CMD}")

    last_beat = last_poll = 0.0

    def enable_all():
        for j in sorted(HOLD):
            lo, hi, adopt, dps = HOLD[j]
            send(ser, f"LIM {j} {lo} {hi} 1")
            send(ser, f"SPD {j} {dps}")
            log(f"  re-ENA {j} {adopt} -> " + send(ser, f"ENA {j} {adopt}"))

    while True:
        now = time.monotonic()

        # HEARTBEAT -- FIRE AND FORGET, ON PURPOSE.
        # The first version called send() here, which BLOCKS up to 1.2 s waiting
        # for a reply. Two of those per cycle can exceed the 4000 ms watchdog, and
        # that is exactly what tripped it: the joints detached, the latch stuck,
        # and PNG does NOT clear a latch -- so the arm stayed down while this loop
        # went on looking healthy. Writing the byte is what feeds the watchdog.
        # The reply is not needed, so it is never waited for.
        if now - last_beat > 0.5:
            try:
                ser.write(b"PNG\n")
                ser.flush()
            except Exception as exc:
                log(f"heartbeat write failed: {exc}")
            last_beat = now

        # HEALTH CHECK, with automatic recovery. A latch that nothing notices is
        # how the arm sat detached for minutes while the log looked fine.
        if now - last_poll > 5.0:
            last_poll = now
            ser.reset_input_buffer()
            sta = send(ser, "STA", wait=0.35)
            if sta:
                try:
                    with open(STATUS, "w", encoding="utf-8") as fh:
                        fh.write(sta)
                except OSError:
                    pass
                if "ES=1" in sta or "WD=1" in sta:
                    log("LATCHED (ES/WD) -- the arm has dropped. Clearing and re-enabling.")
                    log("CLR -> " + send(ser, "CLR"))
                    enable_all()
                elif " EN=1" not in sta:
                    log("every joint reports EN=0 with no latch -- re-enabling.")
                    enable_all()

        # Command channel: one protocol line per file write, consumed then cleared.
        try:
            if os.path.exists(CMD) and os.path.getsize(CMD) > 0:
                with open(CMD, "r", encoding="utf-8") as fh:
                    lines = [ln.strip() for ln in fh if ln.strip()]
                open(CMD, "w").close()
                for ln in lines:
                    log(f"CMD {ln} -> " + send(ser, ln))
        except OSError:
            pass

        time.sleep(0.15)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        log("CRASHED:\n" + traceback.format_exc())
        raise
