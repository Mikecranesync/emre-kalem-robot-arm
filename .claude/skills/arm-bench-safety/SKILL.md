---
name: arm-bench-safety
description: Use before commanding ANY joint on the Emre Kalem arm, and before reporting that a joint moved — covers the fact that no software command is an emergency stop, the hands-clear protocol after a human reaches into the arm, conservative limits on joints whose travel was never found mechanically, and the evidence discipline that stops "it moved" being asserted when nothing turned.
---

# Bench safety and working discipline — Emre Kalem arm

This is the layer that applies to *physical* work on this arm. It is not about code
quality. Every rule here exists because it was learned the expensive way, and the
"why" is attached to each one so nobody re-derives it by breaking something.

---

## 1. Nothing in software is an emergency stop

Say this plainly and never soften it:

- `STP` **holds** — it stops motion with the joints still driven and still holding
  their angle.
- `EST` and the serial **watchdog** do the opposite: they **detach** every joint.
  The firmware's watchdog recovery is a full *latched* detach.
- A detached, gravity-loaded arm **falls.**

So the thing that sounds like the emergency stop is the thing that drops the arm on
the bench. **The rocker switch and the fuse are the only real stop.**

**Never describe a software command as a safety measure.** Never write "I sent EST
to make it safe". If a human needs the arm to be safe, the power comes off.

## 2. Hands-clear protocol — look, don't assume

**Before commanding any joint after a human has reached into the arm: capture a
camera frame and LOOK at it.** Do not rely on the operator having said "go ahead"
one message earlier.

This is not hypothetical. Tonight the operator reached in to check the gripper,
said to try again — and in the very next captured frame his hand was still cupped
around the claw. A message saying "go ahead" describes a moment that has already
passed; a frame describes now.

**A gripper command must never be sent while fingers are in the claw.** Small
hobby servos are weak, and that is not a reason to relax this. Check the frame,
then command.

Related: the operator standing in shot is also a *measurement* hazard, not just a
safety one — his movement dominates any whole-frame pixel difference. See
`arm-motion-verify` for ROI framing.

## 3. Conservative limits where travel was never found mechanically

A locked limit records an angle somebody **commanded and accepted**. It is not a
measurement of mechanical travel — nothing in this system observes the shaft. Read
every `LIM` with that in mind.

| Joint | What the lock actually tells you | How far to drive it |
|---|---|---|
| **J4 wrist pitch** | `0-180` **equals the servo's whole electrical range and the placeholder width.** A joint locked at exactly that has most likely never been driven to a stop at either end, so a mechanical stop may sit well *inside* it. | **At most ±15° off 90.** |
| **J5 wrist roll** | `31` looks like a real found end. `178` is 2° off the electrical ceiling, so the top end is probably electrical, not mechanical. | Stay off 178. |
| **J0 base** | Dead servo. When the replacement goes on, **re-measure** — an MG996R horn seats on a splined shaft in ~18° steps, so the old 29–110 will not point where it used to. | Do not assume the old envelope. |

**Never drive to a locked end you cannot prove was found mechanically.** Driving
into a region the operator already measured as unreachable pushes the joint against
a stop and stalls it — that exact mistake cost a day when a 0–180 sweep widened
J0's min from 29 back to 0 and handed back 29° of travel that had already been
measured as impossible.

**Gravity-loaded joints move everything distal to them.** The elbow carries the
forearm, wrist and gripper; the shoulder carries the whole arm. Use small steps,
and watch `JTO` on every waypoint — a stall shows up there.

## 4. Read-only, and where the boundary sits

This project is **read-only troubleshooting intelligence.** No control writes.

The arm is a **bench rig**, and driving it is a deliberate bench act by an operator
standing next to it. That is the entire exception. It is **never** a
customer-shipped path, it never runs unattended, and no customer-facing surface
ever opens a serial port to a motor. If something you are writing would put a joint
command on a shipped path, stop — that is a different product with a different
review.

## 5. The arm drifts downward while it is held

Observed with five joints energised: the whole gripper assembly settles slowly
downward. The joints hold their commanded angle and the assembly still sags.

Two consequences, both of which have already produced a wrong claim:

- **Frames captured minutes apart are not comparable.** An aggregate frame
  difference picks up the drift, not the thing being measured. Capture control and
  test frames close together in time.
- **`EN=1` is not proof a joint is holding position.** It reports that the joint is
  attached, not where the shaft is. Likewise `STA`'s `SET=` on a *disabled* joint is
  fiction — the firmware seeds every joint to 90 at boot. Validity keys off `EN=`.

## 6. Evidence discipline

This is where claims go wrong, so it is a rule and not a preference.

- **Ask the operator what he OBSERVED.** He is standing at the bench. Three runs
  once "passed" while nothing physically moved. His eyes are the strongest evidence
  available and the approved exception to every vocabulary rule — a value a human
  actually saw.
- **A board ack is not motion.** `OK MOV J6 REQ=70 SET=70 CL=0` proves the firmware
  accepted a command. It proves nothing about the shaft. That distinction cost a
  whole afternoon on D3, where the firmware provably drove the joint at 29.2 °/s
  for 3.5 s and nothing turned.
- **A `0 px` camera result may mean the capture is dead, not that the arm is
  steady.** Check for distinct frame hashes before believing any null.
- **Take a same-command control frame before believing any null result.** Two
  frames at the *same* command give you the noise floor. Without it you cannot tell
  "nothing moved" from "my measurement cannot see movement".
- **Check `CL=`.** `CL=1` means the firmware clamped the request and the joint is
  not where you asked.

## 7. Where I was wrong — why these rules exist

Kept because each was nearly reported as fact.

1. **Reported the arm was locked when it had already dropped.** True when read,
   false a minute later. The watchdog had latched and nothing was checking for it.
2. **Reported "no change" between gripper open and closed — a false negative.** A
   filtering artifact: threshold 45 plus a 5×5 morphological opening erased the thin
   edge bands a moving finger produces. A same-command control frame is what
   exposed it. This is why the threshold is deliberately plain now.
3. **Said "replug the webcam."** It is the laptop's internal camera. There is
   nothing to replug.
4. **A harness reported `MOVED` for a joint that provably does not articulate —
   twice, two different ways.** First a 70 px edge shimmer against a freakishly
   quiet 2 px noise floor scored 35× and printed MOVED. Then, after an absolute
   floor was added, 519 px of *whole-assembly sag* passed both pixel gates and
   printed MOVED again. Both would have handed the operator a false pass on a
   broken gripper. A pixel difference does not prove a *finger* moved, exactly as a
   board ack does not prove a *motor* turned.

## 8. Power — settled facts, not open questions

- **The fuse question is CLOSED.** The operator has ruled: *"Don't worry about the
  fuse. the power is fine."* Do not raise it again, do not caveat work with it, do
  not add it to a risk list.
- Current supply is a **JCPOWER JC-25-5 (+5 V 5 A)**.
- **`_BENCH_SUPPLY_V = 5.0` is the LABEL on that unit, not a measurement.** Do not
  cite it as a measured voltage.
- The **previous** supply was nominally 6 V and **read 6.62 V** — over spec for the
  MG90S joints (rated 4.8–6.0 V), which is worth knowing when a small servo turns
  out to have failed.

## 9. Camera ownership

The device is `Integrated Camera` (`USB\VID_174F&PID_2469`), the laptop's
**internal** webcam and the only camera on this machine. Other PnP camera entries
belong to the HP Envy printer's scanner. **"Replug it" is never the answer.**

If frames come back frozen — identical MD5s, `max 0` difference between captures —
another application owns the device. Find the holder instead of guessing; Windows
records per-app camera use, and `LastUsedTimeStop = 0` means *in use right now*:

```powershell
$r='HKCU:\SOFTWARE\Microsoft\Windows\CurrentVersion\CapabilityAccessManager\ConsentStore\webcam'
Get-ChildItem $r -Recurse | ForEach-Object {
  $p = Get-ItemProperty $_.PSPath -ErrorAction SilentlyContinue
  if ($p.LastUsedTimeStop -eq 0) { Split-Path $_.PSPath -Leaf } }
```

The holder has been `Microsoft.WindowsCamera` (the operator aiming the Camera app
at the arm). Closing it frees the device immediately — no reboot, no admin.
Restarting `FrameServer` or cycling the PnP device *would* need admin.

## 10. Do not

- ❌ Call any software command an emergency stop, or use one as a safety measure.
- ❌ Command a joint after a human intervention without capturing a frame and
  looking at it first.
- ❌ Send a gripper command with fingers in the claw.
- ❌ Drive J4 beyond ±15° off 90, or to any locked end that was never found
  mechanically.
- ❌ Treat a board ack, an `EN=1`, or a pixel difference as proof of motion.
- ❌ Believe a null result that has no same-command control frame behind it.
- ❌ Re-open the fuse question, or cite `_BENCH_SUPPLY_V` as a measured voltage.
- ❌ Suggest replugging the internal camera.
- ❌ Compare frames captured minutes apart — the arm sags between them.
- ❌ Report that a joint moved without saying what evidence says so.

## Cross-references

- `arm-motion-verify` — how to actually prove a joint moved (ROI framing, control
  frames, the geometry gate and its measured limits).
- `Calibration_Notes/evidence/2026-08-06_motion-verify/` — the worked example,
  including the gripper that acked every command and never articulated.
- `Documentation/RESUME-PROMPT.md` — the 🛑 SETTLED section. Read it before raising
  any concern; it lists the questions that are already closed.
