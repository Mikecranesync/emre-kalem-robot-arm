#!/usr/bin/env python3
"""Hold the arm up, and stay alive so it STAYS up.  (Raspberry Pi port.)

WHY THIS IS A DAEMON AND NOT A SCRIPT. Two things detach a joint the moment a
plain script finishes:
  1. the serial watchdog trips once nothing feeds it, and the firmware's own
     recovery is a full latched detach -- every joint drops;
  2. closing the port can toggle DTR and reset the board, which loses limits,
     mirror, and every enable.
So this process opens the port, enables the joints, and then never leaves. It
feeds the watchdog forever and takes commands through a file, which is also the
channel motion_verify.py drives the arm through.

Nothing in software is an emergency stop. `STP` holds with the joints still
driven; `EST` and the watchdog DETACH, and a gravity-loaded arm falls. The rocker
switch and the fuse are the only real stop. Nothing in this file is a safety
device.

WHAT CHANGED FROM THE WINDOWS ORIGINAL (Software/arm-console/hold_arm.py)
  * The Uno is on a Raspberry Pi now, not the laptop. The port default is
    /dev/ttyACM0, not COM5. Everything else about the port is identical: opening
    it still DTR-resets the board and the firmware still keeps nothing across
    that reset.
  * Limits / adopt angles are PARSED FROM joint-limits.csv instead of a
    hand-maintained dict. The original's dict has gone stale and dangerous --
    it carries elbow 0-66 adopt 33 when the real elbow range is 0-30, and wrist
    pitch 0-180 adopt 90 when the real minimum is 33. Commanding a joint into a
    region already measured as unreachable drives it against a mechanical stop
    and stalls it. A file that one human edits after one bench measurement is
    the only thing allowed to say where a joint may go.
  * J0 base is INCLUDED. The original excluded it as a dead servo; that
    diagnosis was wrong. The fault was no 5 V reaching the servos, since fixed,
    and J0 was re-locked 0-180 home 90 on 2026-08-08 on the assembled arm.
  * J1 shoulder is STILL excluded, and now excluded structurally -- see
    build_hold_table().

ADOPT ANGLES. `ENA <j> <adopt>` asserts where the shaft IS; the firmware
pre-loads that pulse BEFORE attaching, so the joint drives to it. Nothing in
this system observes the shaft, so the adopt angle is always a human's estimate.
Here it is `home_deg` from joint-limits.csv -- the value the console's own adopt
dialog pre-fills, deliberately moved off the locked ends by the operator so
there is room to back off. Every destination is therefore inside a range a human
drove to and accepted.

Expect each joint to JUMP to its adopt angle if it is not already there.

CAMERA. This daemon does not touch the camera. It owns the serial port and
nothing else. The Arducam on /dev/video0 is already owned by
~/arm/tools/mjpeg_preview.py, which serves snapshots at
http://127.0.0.1:8781/snapshot -- a SECOND process opening /dev/video0 gets
stale buffered frames, which is indistinguishable from "the joint did not move".
That is motion_verify.py's problem, and its `--camera` source must be that URL.

Usage on the Pi (as a DETACHED process, never a harness background task -- the
first attempt at this was a harness task, it got killed, and the arm fell):

    nohup python3 hold_arm_pi.py --link ~/arm/link > ~/arm/link/daemon.out 2>&1 &
"""

import argparse
import csv
import os
import sys
import time
import traceback

import serial

BAUD = 115200

#: The watchdog window and the heartbeat period are the ORIGINAL's numbers and
#: are deliberately NOT the protocol doc's recommended 1000 / 250 ms. 4000 ms
#: with a 0.5 s beat is an 8x margin, chosen because this loop also does blocking
#: work (the 5 s STA poll, the file channel) between beats. Tightening the window
#: without re-checking that worst-case gap is how the watchdog gets tripped by
#: the very process that exists to feed it.
WDG_MS = 4000
BEAT_S = 0.5
POLL_S = 5.0

#: Joint 1 is the shoulder pair. It is NEVER held by this daemon. See
#: build_hold_table() for the whole reason; it is enforced there, not here.
FORBIDDEN_JOINTS = frozenset({1})

#: Addressable joint ids per SERIAL-PROTOCOL.md section 1. Id 2 is the shoulder
#: pair's second servo and is not addressable by construction -- the firmware
#: answers `ERR E4 <VERB> JOINT=2 RESERVED=shoulder_pair`. A CSV row for it is a
#: malformed file, not a joint we decline to hold.
ADDRESSABLE_JOINTS = frozenset({0, 1, 3, 4, 5, 6})

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

#: Where to look for joint-limits.csv when --limits is not given. First one that
#: exists wins. The last entry is the laptop's checkout, so the CSV parser and
#: its validation can be exercised off-bench without a Pi or a board.
LIMITS_CANDIDATES = (
    os.path.join(SCRIPT_DIR, "joint-limits.csv"),
    os.path.expanduser("~/arm/Software/arm-console/joint-limits.csv"),
    os.path.expanduser("~/emre-kalem-robot-arm/Software/arm-console/joint-limits.csv"),
    "C:/Users/hharp/Documents/emre-kalem-robot-arm/Software/arm-console/joint-limits.csv",
)


def default_limits_path():
    for path in LIMITS_CANDIDATES:
        if os.path.isfile(path):
            return path
    return LIMITS_CANDIDATES[0]


# --------------------------------------------------------------------------
# joint-limits.csv
# --------------------------------------------------------------------------

class LimitsError(Exception):
    """The limits file is unusable. Never partially loaded, never patched up."""


def parse_limits(path):
    """Return {joint_id: dict(name, min, max, home, dps, calibrated)}.

    WHY THE csv MODULE AND NOT str.split(','). The `notes` column is free text
    that a human keeps appending to. It carries no comma TODAY, but the file's
    own header block says fields may be quoted, and one quoted comma turns a
    positional split into a silently shifted row -- a wrong `max_deg` read as a
    `home_deg`. The csv module also looks columns up by NAME, which is what the
    protocol doc requires ("Columns are looked up by name, not by position.
    Unknown columns are ignored. Column order does not matter.").

    WHY THE '#' SKIP. The shipped file opens with ~130 lines of '#' comment
    explaining how to fill it in, BEFORE the real header row that starts
    'joint_id,joint_name,...'. A parser that hands those lines to csv.DictReader
    reads the first comment line as the header and every field comes back None.

    WHY A BAD ROW REJECTS THE WHOLE FILE. Protocol doc section 10: "A half-applied
    limits file is worse than none at all." A partially-loaded file would enable
    some joints against measured limits and others against the firmware's 70-110
    placeholder -- and the operator would have no way to tell which is which.
    """
    try:
        with open(path, "r", encoding="utf-8", newline="") as fh:
            raw = fh.read()
    except OSError as exc:
        raise LimitsError(f"cannot read limits file {path}: {exc}") from exc

    # Keep the original 1-based line numbers so an error names the line the
    # operator will actually see in their editor.
    kept, numbers = [], []
    for lineno, line in enumerate(raw.splitlines(), start=1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        kept.append(line)
        numbers.append(lineno)

    if len(kept) < 2:
        raise LimitsError(f"{path}: no header row and/or no data rows after the '#' block")

    reader = csv.DictReader(kept)
    required = ("joint_id", "min_deg", "max_deg", "home_deg", "max_deg_per_sec")
    missing = [c for c in required if c not in (reader.fieldnames or [])]
    if missing:
        raise LimitsError(
            f"{path}: header row is missing column(s): {', '.join(missing)}\n"
            f"  header seen: {reader.fieldnames}"
        )

    def as_int(row_line, column, value):
        text = (value or "").strip()
        try:
            return int(text)
        except ValueError:
            raise LimitsError(
                f"{path} line {row_line}: {column}={text!r} is not a whole number"
            ) from None

    table = {}
    for offset, row in enumerate(reader):
        # numbers[0] is the header line; data rows start one later.
        line_no = numbers[offset + 1]

        jid = as_int(line_no, "joint_id", row.get("joint_id"))
        if jid == 2:
            raise LimitsError(
                f"{path} line {line_no}: joint_id 2 is RESERVED (shoulder pair's second "
                f"servo, D5). It is never addressable; a row for it means this file "
                f"disagrees with the firmware."
            )
        if jid not in ADDRESSABLE_JOINTS:
            raise LimitsError(f"{path} line {line_no}: joint_id {jid} is not an addressable joint")
        if jid in table:
            raise LimitsError(f"{path} line {line_no}: joint_id {jid} appears twice")

        lo = as_int(line_no, "min_deg", row.get("min_deg"))
        hi = as_int(line_no, "max_deg", row.get("max_deg"))
        home = as_int(line_no, "home_deg", row.get("home_deg"))
        dps = as_int(line_no, "max_deg_per_sec", row.get("max_deg_per_sec"))

        # Each of these mirrors a firmware refusal. Catching them HERE, before
        # the port is ever opened, is the whole point -- see main().
        if not (0 <= lo < hi <= 180):
            raise LimitsError(
                f"{path} line {line_no} (J{jid}): min={lo} max={hi} is outside the servo's "
                f"0..180 electrical range or not min<max (firmware: ERR E10 LIM)"
            )
        if hi - lo < 5:
            raise LimitsError(
                f"{path} line {line_no} (J{jid}): span {hi - lo} is under the firmware's "
                f"5 degree minimum (ERR E10 LIM MINSPAN=5)"
            )
        if not (lo <= home <= hi):
            # Silent-death case: ENA would answer E5, the joint would never
            # attach, and the log would otherwise read as a healthy startup.
            raise LimitsError(
                f"{path} line {line_no} (J{jid}): home_deg={home} is outside {lo}..{hi}. "
                f"home_deg becomes the ENA adopt angle, and the firmware refuses an adopt "
                f"outside MIN..MAX (ERR E5 ENA)"
            )
        if not (1 <= dps <= 90):
            raise LimitsError(
                f"{path} line {line_no} (J{jid}): max_deg_per_sec={dps} is outside 1..90 "
                f"(firmware: ERR E12 SPD)"
            )

        table[jid] = {
            "name": (row.get("joint_name") or f"J{jid}").strip(),
            "min": lo,
            "max": hi,
            "home": home,
            # The CSV's max_deg_per_sec, NOT the original daemon's hand-picked
            # 12-20. That divergence is deliberate: the file is the source of
            # truth, and SPD only governs MOV interpolation -- ENA pre-loads the
            # adopt pulse and attaches, so the adopt jump happens at the servo's
            # own speed whatever DPS says. motion_verify.py sets SPD itself for
            # a run. To slow a joint down, edit the CSV; do not add a flag here.
            "dps": dps,
            "calibrated": (row.get("calibrated") or "").strip().lower() == "yes",
            "pins": (row.get("uno_pins") or "").strip(),
        }

    if not table:
        raise LimitsError(f"{path}: header row present but no data rows")
    return table


def build_hold_table(limits, requested):
    """Decide which joints this daemon will energise, and refuse J1 here.

    THE J1 REFUSAL LIVES AT CONSTRUCTION, NOT AT ARGUMENT PARSING. Everything
    downstream -- the LIM/SPD push, the proximal-to-distal ENA loop, and
    critically enable_all() on the recovery path -- iterates this one dict. If
    the refusal sat in parse_args() instead, a later edit to the recovery path
    could reintroduce joint 1 on the code path that fires UNATTENDED, minutes
    after everyone stopped watching.

    WHY J1 AT ALL. `ENA 1 <adopt>` needs `MIR INV <offset>` pushed first, and
    that command asserts a mirror axis NOBODY HAS MEASURED on this arm.
    mirror_offset_deg is still 0 and still a placeholder. If the true axis is
    96 degrees, the two MG996R servos are commanded 12 degrees apart the entire
    time the joint is enabled: they hold against each other, run hot, and strip
    gears. The firmware's E13 guard does NOT catch this -- it is satisfied the
    moment a mirror mode is set, whatever the offset says.
    """
    if requested is None:
        # Default: every CALIBRATED joint except the forbidden ones. Derived
        # from the file rather than a literal {0,3,4,5,6} so that flipping a row
        # back to calibrated=no drops that joint automatically -- an uncalibrated
        # row means its limits are still a guess, and a guessed envelope is
        # exactly what must not be driven against.
        requested = sorted(j for j, r in limits.items() if r["calibrated"])

    hold, refused = {}, []
    for jid in sorted(set(requested)):
        if jid in FORBIDDEN_JOINTS:
            refused.append(jid)
            continue
        if jid not in limits:
            raise LimitsError(
                f"joint {jid} was requested but has no row in the limits file. "
                f"Rows present: {sorted(limits)}"
            )
        hold[jid] = limits[jid]
    return hold, refused


# --------------------------------------------------------------------------
# serial plumbing
# --------------------------------------------------------------------------

class Daemon:
    def __init__(self, port, link_dir, hold):
        self.port = port
        self.hold = hold
        self.cmd_path = os.path.join(link_dir, "arm_cmd.txt")     # write a line -> it is sent
        self.log_path = os.path.join(link_dir, "arm_hold.log")
        self.status_path = os.path.join(link_dir, "arm_status.txt")
        self.ser = None
        self._boot_lines = []
        self._claimed = False

    def boot(self, msg):
        """Say something BEFORE the log file has been claimed.

        arm_hold.log is truncated per run, but not until this process has
        actually taken the port -- see claim_log(). Until then everything goes to
        stdout and is buffered, so a daemon that refuses to start never destroys
        the log of one that is currently holding the arm.
        """
        line = f"{time.strftime('%H:%M:%S')} {msg}"
        print(line, flush=True)
        self._boot_lines.append(line)

    def claim_log(self):
        """Truncate the log and flush the buffered boot lines into it.

        WHY THE TRUNCATE IS HERE AND NOT AT STARTUP. motion_verify.py addresses
        this file BY BYTE OFFSET -- it records the length before a command and
        tails from there to get that command's own reply, and to check whether
        `LATCHED` / `re-ENA` appeared inside a waypoint window. Truncating it
        out from under a run makes those offsets point past EOF, so the tail
        comes back empty and an intervention the daemon really did perform reads
        as "no intervention". That is a false negative on the one check that
        stops snapped-to-adopt joints being graded as motion.

        So the file is only claimed once this process holds the port
        (exclusive=True), which is the point at which it is provably the only
        daemon and any previous run is provably over. A restart mid-measurement
        still invalidates offsets the harness is holding -- that is unavoidable
        and was never a valid thing to do.
        """
        open(self.log_path, "w").close()
        with open(self.log_path, "a", encoding="utf-8") as fh:
            for line in self._boot_lines:
                fh.write(line + "\n")
        self._claimed = True

    def log(self, msg):
        # Before the log is claimed there may be ANOTHER daemon's log at this
        # path. Never write into it -- a failed startup (a bad port, a lost race
        # for the exclusive lock, a firmware that is not ours) must not inject
        # lines into the file a live measurement run is tailing.
        if not self._claimed:
            self.boot(msg)
            return
        line = f"{time.strftime('%H:%M:%S')} {msg}"
        print(line, flush=True)
        try:
            with open(self.log_path, "a", encoding="utf-8") as fh:
                fh.write(line + "\n")
        except OSError as exc:
            # The log IS the command channel's reply path (motion_verify.py tails
            # it from a byte offset). Losing it is bad, but not a reason to stop
            # feeding the watchdog and drop the arm.
            print(f"!! log write failed: {exc}", flush=True)

    def send(self, line, wait=0.30):
        """Send one line and accumulate the reply. Blocks up to ~1.2 s.

        NEVER call this from the heartbeat. See the loop() comment.
        """
        ser = self.ser
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

    # -- startup ----------------------------------------------------------

    def connect(self):
        self.boot(f"opening {self.port} at {BAUD} -- THIS DTR-RESETS THE BOARD.")
        self.boot("  the firmware keeps nothing across that reset: limits revert to the")
        self.boot("  70-110 default, MIR goes UNKNOWN, CAL goes 0, every joint detaches.")
        self.boot("  that is why LIM and SPD are pushed below before anything is enabled.")

        # exclusive=True is a Linux-specific hazard the Windows original never
        # had to think about. Windows locks a COM port; Linux does NOT lock a tty
        # by default, so a stale daemon left over from a previous session and
        # this one would BOTH write to /dev/ttyACM0. Interleaved bytes produce
        # mangled acks and stolen replies, which reads exactly like "the board
        # acked and nothing moved". exclusive=True turns that into a clean
        # startup failure instead of a mystery.
        # (A TypeError here means pyserial is older than 3.3 -- upgrade it
        # rather than dropping the flag.)
        self.ser = serial.Serial(self.port, BAUD, timeout=0.3, exclusive=True)

        # The port is ours and no other daemon can be holding it, so it is now
        # safe to claim the log. Everything said above lands in the fresh file.
        self.claim_log()

        # Optiboot waits ~1 s for an upload before jumping to the sketch.
        time.sleep(2.2)
        self.ser.reset_input_buffer()

    def handshake(self):
        """Protocol section 9 step 5: refuse to proceed unless this is our firmware.

        The Windows original skipped this because COM5 was a known constant. On
        the Pi it is not: /dev/ttyACM0 is not stable across a replug (it can come
        back as ttyACM1, with something else on ACM0), and there are at least
        three sketches that could be flashed to this board. The single-servo
        bench sketch treats a bare 'a' as ATTACH -- a daemon that starts pushing
        blind at the wrong firmware will one day attach a servo nobody asked for.
        """
        ver = self.send("VER")
        self.log("VER  -> " + ver.replace("\n", " | "))
        if "NAME=FACTORYLM-ARM" not in ver:
            raise RuntimeError(
                f"{self.port} did not identify as FACTORYLM-ARM. Refusing to send anything "
                f"further. What answered VER: {ver!r}"
            )

    def push_state(self):
        # A previous holder dying leaves the watchdog latched and every joint
        # detached. CLR is a no-op if nothing is latched.
        self.log("CLR  -> " + self.send("CLR"))
        self.log(f"WDG  -> " + self.send(f"WDG {WDG_MS}"))

        # Limits and speed BEFORE enabling -- LIM is refused on a live joint
        # (ERR E9 ... STATE=enabled), and the DTR reset above put every envelope
        # back to the firmware default of 70-110.
        #
        # NOTHING IS SENT FOR JOINT 1, AND `MIR` IS DELIBERATELY NEVER SENT.
        # Its absence is load-bearing, not an oversight: with no MIR command the
        # firmware stays at MIR=UNKNOWN after the DTR reset, so `ENA 1 <deg>`
        # answers `ERR E13 ENA JOINT=1 MIR=UNKNOWN` -- a free backstop underneath
        # this daemon's own refusal. The original had `MIRROR = "MIR INV 0"`
        # here. Do not "restore" it: sending it would assert an unmeasured mirror
        # axis and disarm E13 for every other process sharing this port.
        for jid in sorted(self.hold):
            row = self.hold[jid]
            reply = self.send(f"LIM {jid} {row['min']} {row['max']} 1")
            self.log(f"LIM {jid} -> " + reply)
            if "E9" in reply:
                # The board did NOT reset when the port opened, so a joint is
                # still live at limits we did not set. Say so loudly and carry
                # on with the joints that did take.
                #
                # DO NOT "recover" by sending `DIS A`. That detaches everything
                # and a gravity-loaded arm sags. Whether to drop the arm is an
                # operator decision made standing next to it, never one this
                # daemon makes on its own.
                self.log(
                    f"  !! STALE-LIMITS J{jid}: LIM refused (E9) -- the board did not reset "
                    f"on open and this joint is live at limits from a previous session. "
                    f"NOT sending DIS A. Operator: power off, or stop every other owner of "
                    f"{self.port} and restart this daemon."
                )
            self.log(f"SPD {jid} -> " + self.send(f"SPD {jid} {row['dps']}"))

    def enable_initial(self):
        # Proximal to distal (ascending joint id IS proximal to distal on this
        # arm: base, elbow, wrist pitch, wrist roll, gripper), so each joint has
        # settled before the next one loads it. The settle sleep is not cosmetic
        # -- a gravity-loaded joint moves everything distal to it, and enabling
        # the whole arm at once is the most likely brownout on this supply.
        for jid in sorted(self.hold):
            row = self.hold[jid]
            self.log(
                f"ENA {jid} {row['home']} ({row['name']}) -> "
                + self.send(f"ENA {jid} {row['home']}")
            )
            time.sleep(1.2)

    def enable_all(self):
        """Re-push limits, speed and enable for every held joint.

        THE LOG MARKERS BELOW ARE AN API, NOT LOG STYLE. motion_verify.py tails
        arm_hold.log across each waypoint window and does a plain substring test
        for `LATCHED` and `re-ENA` (motion_verify.py, `intervened = ...`). If
        either fires, that waypoint is reported INVALID (daemon re-enabled
        mid-waypoint) and is thrown away rather than measured -- because
        re-sending `ENA <j> <adopt>` SNAPS joints back to their adopt angles,
        which reads exactly like "the joint moved on its own".

        So: emit `re-ENA` per joint, byte for byte, same hyphen, same case.
        Renaming it to `re-enabled` or `RE-ENA` silently stops motion_verify
        flagging intervention and starts it grading snapped-to-adopt joints as
        real motion -- the exact false positive that whole harness exists to
        prevent.

        PRECONDITION: EVERY JOINT IS ALREADY DETACHED. Both callers guarantee it
        -- the latch branch has just sent `CLR` (a latch detaches everything, and
        `CLR` clears the latch without re-attaching), and the other branch only
        fires when no joint reports `EN=1` at all. That is what makes the `LIM`
        below legal: `LIM` is refused on a live joint (`ERR E9 ... STATE=enabled`).

        A PARTIAL detach -- one joint dropped while the others still hold -- is
        NOT detected by either branch and no recovery is attempted. That matches
        the original daemon and is left deliberately alone. If you ever widen the
        health check to catch it, this function becomes wrong: `LIM` on the
        still-live joints will return E9 and the re-enable will half-apply,
        leaving some joints on measured limits and others on the firmware's
        70-110 placeholder. Fix it by disabling that ONE joint before re-LIM-ing
        it. Do NOT reach for `DIS A` -- that detaches everything and a
        gravity-loaded arm sags.
        """
        for jid in sorted(self.hold):
            row = self.hold[jid]
            self.send(f"LIM {jid} {row['min']} {row['max']} 1")
            self.send(f"SPD {jid} {row['dps']}")
            self.log(
                f"  re-ENA {jid} {row['home']} -> " + self.send(f"ENA {jid} {row['home']}")
            )

    # -- the forever loop -------------------------------------------------

    def loop(self):
        last_beat = last_poll = 0.0

        while True:
            now = time.monotonic()

            # HEARTBEAT -- FIRE AND FORGET, ON PURPOSE.
            # The first version of this daemon called send() here, which BLOCKS
            # up to 1.2 s waiting for a reply. Two of those per cycle can exceed
            # the 4000 ms watchdog, and that is exactly what tripped it: the
            # joints detached, the latch stuck, and PNG does NOT clear a latch --
            # so the arm stayed down while this loop went on looking healthy.
            # Writing the byte is what feeds the watchdog. The reply is not
            # needed, so it is never waited for.
            if now - last_beat > BEAT_S:
                try:
                    self.ser.write(b"PNG\n")
                    self.ser.flush()
                except Exception as exc:
                    # Log and keep going. NEVER exit on a write failure: exiting
                    # closes the port, which DTR-resets the board and drops the
                    # arm. A daemon still trying is strictly better than a
                    # daemon that tidied up and left.
                    self.log(f"heartbeat write failed: {exc}")
                last_beat = now

            # HEALTH CHECK, with automatic recovery. A latch that nothing
            # notices is how the arm sat detached for minutes while the log
            # looked fine -- and how a session once reported "the arm is locked"
            # a minute after it had already dropped.
            if now - last_poll > POLL_S:
                last_poll = now
                self.ser.reset_input_buffer()
                sta = self.send("STA", wait=0.35)
                if sta:
                    try:
                        with open(self.status_path, "w", encoding="utf-8") as fh:
                            fh.write(sta)
                    except OSError:
                        pass
                    if "ES=1" in sta or "WD=1" in sta:
                        # `LATCHED` is the other literal motion_verify.py greps
                        # for. Keep it.
                        self.log("LATCHED (ES/WD) -- the arm has dropped. Clearing and re-enabling.")
                        self.log("CLR -> " + self.send("CLR"))
                        self.enable_all()
                    elif " EN=1" not in sta:
                        self.log("every joint reports EN=0 with no latch -- re-enabling.")
                        self.enable_all()

            # Command channel: one protocol line per file write, consumed then
            # cleared. This is a RAW PIPE by contract -- lines are forwarded
            # verbatim, never rewritten, because the operator and
            # motion_verify.py both depend on getting the firmware's own reply
            # back. Hazardous lines are shouted about, not blocked.
            try:
                if os.path.exists(self.cmd_path) and os.path.getsize(self.cmd_path) > 0:
                    with open(self.cmd_path, "r", encoding="utf-8") as fh:
                        lines = [ln.strip() for ln in fh if ln.strip()]
                    open(self.cmd_path, "w").close()
                    for ln in lines:
                        self.warn_if_hazardous(ln)
                        self.log(f"CMD {ln} -> " + self.send(ln))
            except OSError:
                pass

            time.sleep(0.15)

    def warn_if_hazardous(self, line):
        """Shout about the two lines that would undo this daemon's J1 invariant."""
        parts = line.split()
        if not parts:
            return
        verb = parts[0].upper()
        if verb == "MIR":
            self.log(
                "  !! WARNING: a MIR command is about to go out. This daemon never sends one, "
                "so MIR=UNKNOWN has been the firmware's own backstop against enabling the "
                "shoulder (ERR E13). Setting any mirror mode DISARMS that, and "
                "mirror_offset_deg is still 0 and still UNMEASURED on this arm -- a wrong "
                "axis puts two MG996R in continuous opposition and strips gears."
            )
        elif verb == "ENA" and len(parts) > 1 and parts[1] == "1":
            self.log(
                "  !! WARNING: ENA on joint 1 (shoulder pair). This daemon refuses to hold J1 "
                "and will not re-enable it after a latch. The firmware should refuse this "
                "with ERR E13 while MIR=UNKNOWN; if it does not, something else has already "
                "sent MIR."
            )


# --------------------------------------------------------------------------

def parse_joint_list(text):
    out = []
    for tok in text.split(","):
        tok = tok.strip()
        if not tok:
            continue
        try:
            out.append(int(tok))
        except ValueError:
            raise SystemExit(f"--joints: {tok!r} is not a joint id")
    if not out:
        raise SystemExit("--joints: no joint ids given")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Hold the Emre Kalem arm up from a Raspberry Pi and stay alive so it "
                    "STAYS up. Owns the serial port; takes commands through a file.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--port", default="/dev/ttyACM0",
                    help="serial device the Uno is on (default: %(default)s)")
    ap.add_argument("--limits", default=default_limits_path(),
                    help="joint-limits.csv -- the source of truth for every min/max/home/speed "
                         "(default: %(default)s)")
    ap.add_argument("--link", default=SCRIPT_DIR,
                    help="directory for arm_cmd.txt / arm_hold.log / arm_status.txt "
                         "(default: alongside this script)")
    ap.add_argument("--joints", default=None, type=parse_joint_list,
                    help="comma-separated joint ids to hold. Default: every calibrated joint "
                         "in the limits file EXCEPT joint 1. Joint 1 is refused whatever is "
                         "asked for here.")
    args = ap.parse_args(argv)

    link_dir = os.path.abspath(os.path.expanduser(args.link))
    os.makedirs(link_dir, exist_ok=True)

    # ------------------------------------------------------------------
    # PARSE AND VALIDATE THE LIMITS FILE BEFORE THE PORT IS EVER OPENED.
    # Opening the port DTR-resets the board, which detaches every joint and
    # drops a gravity-loaded arm. If a bad CSV killed this process AFTER the
    # open, the arm would be left detached with nothing feeding the watchdog
    # and no owner of the port. Every reason to refuse is found here, while
    # the board is still untouched and whatever was holding the arm still is.
    # ------------------------------------------------------------------
    limits_path = os.path.abspath(os.path.expanduser(args.limits))
    try:
        limits = parse_limits(limits_path)
        hold, refused = build_hold_table(limits, args.joints)
    except LimitsError as exc:
        print(f"REFUSING TO START -- the limits file is unusable:\n  {exc}", file=sys.stderr)
        print("  the board has NOT been touched. Nothing was opened, nothing was enabled.",
              file=sys.stderr)
        return 2

    if not hold:
        print("REFUSING TO START -- no joints left to hold after exclusions.", file=sys.stderr)
        return 2

    daemon = Daemon(args.port, link_dir, hold)

    # boot(), not log() -- the log file is not claimed until the port is open.
    daemon.boot(f"limits file: {limits_path}")
    for jid in sorted(hold):
        row = hold[jid]
        daemon.boot(
            f"  J{jid} {row['name']:<12} {row['pins']:<6} "
            f"{row['min']}-{row['max']} adopt {row['home']} at {row['dps']} deg/s"
        )
    for jid in sorted(refused):
        # Printed every single run, on purpose. The reason is a mechanical
        # hazard, not a policy, and the next person to read this log should not
        # have to go and find out why the shoulder is missing.
        daemon.boot(
            f"  J{jid} REFUSED -- the shoulder pair. `ENA 1` needs `MIR INV <offset>` first, "
            f"and mirror_offset_deg is still 0 and still UNMEASURED on this arm. Two MG996R "
            f"fighting each other through one link strip gears. Measure the offset "
            f"(joint-limits.csv header block) before anyone holds this joint."
        )

    try:
        daemon.connect()
        daemon.handshake()
        daemon.push_state()
        daemon.enable_initial()

        daemon.log("STA  -> " + daemon.send("STA").replace("\n", " | "))
        daemon.log("HOLDING. joints " + ",".join(str(j) for j in sorted(hold)) +
                   " energised. j1 excluded -- unmeasured shoulder mirror offset.")
        daemon.log(f"command channel: write a protocol line into {daemon.cmd_path}")
        daemon.log(f"replies + markers: {daemon.log_path}   (STA every {POLL_S:.0f}s: "
                   f"{daemon.status_path})")
        daemon.log("nothing here is an emergency stop -- the rocker switch and the fuse are.")

        daemon.loop()
    except KeyboardInterrupt:
        # Not a clean shutdown, and it must not pretend to be. Whatever happens
        # next, the port closes, the watchdog stops being fed, and the firmware's
        # recovery is a latched detach: the arm falls.
        daemon.log("INTERRUPTED -- the port is about to close. The watchdog will latch and "
                   "every joint will detach. GO AND CATCH THE ARM.")
        return 130
    except Exception:
        daemon.log("CRASHED:\n" + traceback.format_exc())
        daemon.log("the port is about to close -- the watchdog will latch and the arm will "
                   "drop. GO AND CATCH THE ARM.")
        raise
    return 0


if __name__ == "__main__":
    sys.exit(main())
