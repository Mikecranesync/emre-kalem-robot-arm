---
name: arm-serial-control
description: Use when commanding the Emre Kalem arm over serial — enabling joints, sending MOV/LIM/ENA/SPD, holding the arm up, or debugging a joint that acks but does not move. Trigger on any task touching COM5, hold_arm.py, the servo console, or a protocol reply.
---

# Commanding this arm without dropping it

**Board acks prove the firmware accepted a command. They do not prove a motor
turned.** That distinction cost a whole afternoon on D3, where the firmware
provably drove the joint at 29.2 °/s for 3.5 s and nothing moved — the servo was
dead. Every rule below exists because breaking it already cost a session.

For verifying that a shaft actually turned, see the `arm-motion-verify` skill.
This skill is only about getting commands in and replies out.

---

## 1. A holder DAEMON, never a script

Two independent things detach every joint the moment a plain script exits:

1. **The watchdog latches.** Once nothing feeds it, the firmware's own recovery is
   a full **LATCHED detach** — every joint drops. A gravity-loaded arm falls.
2. **Closing the port can toggle DTR and reset the board.** The firmware keeps
   nothing across that reset: limits, mirror mode and every enable are gone.

So the process that owns the port opens it, enables the joints, and **never
leaves**. It feeds the watchdog forever and takes commands through a file.

`hold_arm.py` is that daemon. It must run as a **detached OS process**, not as a
harness background task — the first attempt was a harness task, it got killed, and
the arm fell.

## 2. The daemon's two bugs. Both fixed, both trivially easy to reintroduce

**(a) The heartbeat must be fire-and-forget.** The first version fed the watchdog
with a `send()` that blocks up to 1.2 s waiting for a reply. Two of those per cycle
exceeds the 4000 ms watchdog. That is exactly what tripped it: joints detached, the
latch stuck, and the arm sat on the bench.

```python
ser.write(b"PNG\n"); ser.flush()   # writing the byte is what feeds the watchdog
```

Never wait for the `PNG` reply. The reply is not needed.

**(b) `PNG` does NOT clear a latch.** Once `ES=1` or `WD=1` is set it stays set, and
the loop will go on answering `STA` cheerfully while every joint is dead. Something
must **actively notice** the latch, send `CLR`, and re-enable:

```python
if "ES=1" in sta or "WD=1" in sta:
    send(ser, "CLR"); enable_all()      # LIM + SPD + ENA, per joint
```

A latch that nothing notices is how the arm sat detached for minutes while the log
looked healthy — and how a session reported "the arm is locked" that had already
dropped a minute earlier.

**Consequence for anyone measuring:** `enable_all()` re-sends `ENA <j> <adopt>`,
which **snaps joints back to their adopt angles**. That reads exactly like "the
joint moved on its own". Scan the log for `LATCHED` / `re-ENA` before trusting any
measurement window.

## 3. One owner per port. The console and the daemon are mutually exclusive

There is exactly one serial port and `GET /rx` is **destructive** — a second poller
steals the console's replies. You cannot run both.

To use the GUI instead: stop the daemon first, then

```powershell
Start-Process "C:\RobotArm\START ARM GUI.bat"
```

**`START ARM GUI.bat` cannot be launched from `cmd`** — `START` is a cmd builtin.
Use PowerShell `Start-Process`. Use the tab it opens and leave the black window
open: **each launch mints a fresh access code, so typing the URL will not work.**

## 4. The file command channel

While the holder runs, this is how you talk to the arm. In the daemon's directory:

| File | Role |
|---|---|
| `arm_cmd.txt` | write ONE protocol line → the daemon sends it |
| `arm_hold.log` | every command and its reply, append-only |
| `arm_status.txt` | full `STA`, rewritten every 5 s |

**CRITICAL SYNCHRONISATION RULE.** `arm_status.txt` is only rewritten on the
daemon's 5 s poll, so immediately after a `MOV` it still holds **pre-move** state —
which says `MOV=0`. Waiting on that returns instantly and you photograph
mid-travel, or worse, call the move settled when it has not started.

Correct primitive: **record `arm_hold.log`'s byte length BEFORE writing the
command, then tail from that offset.** That yields this command's own reply,
synchronously, and works for a `STA` you push through the same channel.

The reference implementation is `Software/tests/reply_cut.py` plus the `Arm`
class in `Software/tests/cycle_poses.py` — `offset()`, `tail(offset)`, `send(line)`,
`sta_row(jid)`, `field(row, key)`. Import it rather than rewriting it:

**Do NOT import it from `motion_verify.py`.** That module imports `cv2` and
`numpy` at the top, so the one-line convenience drags OpenCV into any process
that follows it -- including a bot whose strongest true claim is zero new
dependencies. Two stdlib-only choices, both with the reply-cut fix:

```python
# in-repo tools (bench):
import sys; sys.path.insert(0, "C:/RobotArm/Software/tests")
from cycle_poses import Arm          # + reply_cut.cut_reply / clamped
# anything that must stay stdlib-only:
sys.path.insert(0, "C:/RobotArm/Software/arm-telegram")
from arm_link import ArmLink
```

**Read the reply through `reply_cut.clamped()`, not `"CL=1" in reply`.** The
daemon logs its own heartbeat and 5 s poll into the same file, so a reply can
arrive contaminated or truncated before its `CL=` field -- on 2026-08-07 a live
run produced `OK MOV J1 REQ=88 SET=88 C`, on which a real clamp is invisible.
`clamped()` returns True / False / **None**, and None means unreadable, which is
not a pass.

## 5. Protocol gotchas — hard facts, do not re-derive these

- **`LIM` takes FOUR arguments:** `LIM <j> <min> <max> <cal>`. Three gives `ERR E2`.
- **`STA` takes NONE.** `STA J0` is `ERR E2`. It returns one row per joint plus a
  `SYS` row.
- **Opening the port DTR-resets the board and the firmware keeps nothing.** Limits
  revert to the default **70–110**, `MIR=UNKNOWN`, `CAL=0`. Any script that opens
  the port must push `LIM` itself before enabling anything.
- **J6's 10–70 lies almost entirely BELOW the 70–110 boot default.** After any
  reconnect, everything under 70 **clamps** until `LIM 6 10 70 1` goes out.
- **`STA`'s `SET=` is fiction on a disabled joint.** The firmware seeds every joint
  to 90 at boot. Validity keys off **`EN=`**, never off `SET=`.
- **`ENA <j> <adopt>` DRIVES the joint to `<adopt>`.** The firmware pre-loads that
  pulse *before* attaching. Nothing in this system observes the shaft, so that
  number is always a human's estimate: adopt where the joint *is* and it does not
  move; adopt at midpoint when it has sagged and **it will lift the arm to get
  there**.
- **`MIR INV 0` must go out AFTER joint 1's `LIM`, and while J1 is still disabled.**
  An INV offset is validated against joint 1's range, so the range must be in place
  first. This is an ordering rule, not a hazard gate.
- **`LIM` is refused on a live joint.** Limits and speed go out before `ENA`.

## 6. Read the reply fields, every time

- **`CL=1` on a `MOV` reply means the firmware CLAMPED it** — the joint is *not*
  where you asked. Treat `CL=1` as a failed command, not a warning.
- **`JTO=`** is the joint-timeout flag on each `STA` row. Watch it on any joint that
  might stall against a mechanical stop, especially one whose travel ends are
  unconfirmed.
- **`ES=` / `WD=`** on the `SYS` row are the latches. `WDMS=4000` is the watchdog
  window.

## 7. Proving the firmware is actually driving a pin

Sample `STA` repeatedly through the travel and watch `SET` ramp with `MOV=1`.
Worked example, J6 commanded from 10 to 70 at `SPD 6 6` on 2026-08-06:

```
SET: 13 15 18 21 24 26 31 34 37 39 42 46 49 53 56 59 65 68 70   MOV=1 throughout
~5.9 °/s against a commanded 6
```

Textbook interpolation. **This exonerates the firmware and says nothing whatsoever
about whether the shaft turned** — on that same run the gripper's fingers did not
move at all. Use this to *rule out* software, never to claim motion.

## 8. Per-joint state (as of 2026-08-06 ~19:30)

| J | Joint | Pin | Limits | Home | Notes |
|---|---|---|---|---|---|
| 0 | base | D3 | 29–110 | 64 | **DEAD servo — deliberately excluded.** Replacement ordered. Re-measure J0 after the swap: an MG996R horn seats in ~18° spline steps, so 29–110 will not point where it used to. |
| 1 | shoulder | D4+D5 | 0–91 | 1 | Two MG996R, `MIR=INV OFF=0`. **The offset is unmeasured — this is settled and not a blocker;** the operator has run this joint on exactly this configuration. |
| 3 | elbow | D6 | 0–66 | 33 | MG996R, gravity-loaded through the whole forearm. |
| 4 | wrist pitch | D9 | 0–180 | 90 | MG90S. **Ends UNCONFIRMED** — 0–180 is the entire electrical range and equals the placeholder width, so a mechanical stop may sit well inside it. Stay within ±15 of 90. |
| 5 | wrist roll | D10 | 31–178 | 104 | MG90S. 31 looks like a real found end; 178 is 2° off the electrical ceiling. |
| 6 | gripper | D11 | 10–70 | 40 | **REGRESSED.** Acks and ramps correctly, fingers do not articulate. Operator's diagnosis after checking by hand: *the gear is slipping around the motor shaft.* Mechanical. Do not record it as controllable. |

Canonical sources: `Software/wiring-map.csv`, `Calibration_Notes/calibration-log.csv`
(append-only dated rows — never edit a past observation, append a new one).

## 9. Leave the bench as you found it

Before you finish: **restore any `SPD` you changed**, return every joint you moved
to its home angle, and **confirm with `STA`** rather than assuming. A joint left at
a test waypoint is a trap for the next session, which will read `calibration-log.csv`
homes and find the arm somewhere else.

## 10. Unresolved: the daemon is not in the repo

`hold_arm.py` **is committed**, at `Software/arm-console/hold_arm.py`. It derives
`arm_cmd.txt` / `arm_hold.log` / `arm_status.txt` from `__file__`, so running the
repo copy puts the link files in `Software/arm-console/` -- that is the `--link`
directory. All three names are gitignored. (This section previously said the file
was not committed and lived only in a session scratchpad; that was true when it
was written and stopped being true on 2026-08-06.)

This is the operator's decision to make, not an agent's. Surface it; do not
silently commit the daemon into the repo, and do not assume the path still exists.

## Nothing in software is an emergency stop

`STP` holds with joints driven. `EST` and the watchdog **detach**, and a
gravity-loaded arm falls. **The rocker switch and the fuse are the only real stop.**
Never describe a software command as a safety measure, and never command a joint
while the operator's hand is on it.
