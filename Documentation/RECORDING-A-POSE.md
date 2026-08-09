# Recording a pose

A pose is a named position you can send the arm back to. Park, start, pick,
handoff — anything you want to return to.

## The short version

1. Double-click **Arm Console** on the desktop. Press **CONNECT**.
2. Pick a joint in the left rail. Press **ENABLE**. Type the angle it is sitting
   at right now, judged by eye, and press CONFIRM.
3. Jog it with the big bar. **Every time you let go, that position is saved as a
   step.** You will see the step appear in the table on the right.
4. Do the same for the other joints, in the order that keeps the claw out of
   trouble.
5. Type a name. Press **SAVE POSE**.

That is it. There is no separate "record" button to remember.

## Why the steps matter

Two positions can both be safe while the straight line between them drives the
claw through the bench, the wire loom, or the base housing. So a pose is not six
numbers — it is **the route you drove, in the order you drove it**.

The steps are that route. They are recorded from the angles the arm was actually
commanded to, at each point where you stopped and looked at it. Nothing is
guessed.

A pose saved with no steps records only where the arm ended up. The console will
show it in the library as **REFERENCE ONLY** and refuse to load it, because
there is no safe way to drive somewhere when nobody recorded how. Above the SAVE
POSE button the console tells you which one you are about to write.

## What the library tells you

| Badge | Means |
|---|---|
| **READY** | It has a recorded route. LOAD it, read the steps, press PLAY. |
| **REFERENCE ONLY** | No route recorded. Kept as history; cannot be driven. |
| **OUT OF RANGE** | A joint's limits have been narrowed since it was saved, so the arm can no longer reach it. Kept as history; re-record it. |

**LOAD** puts the steps in the teach table so you can read them.
**PLAY** then drives them. PLAY on a library row stays dead until you have
loaded that row — the review is the point, not a formality.

## Picking a good park position

Park is where the arm rests with the power off, so it is the one pose worth
being fussy about.

- **Stay a few degrees inside each joint's limits.** A joint parked hard against
  its own end has nowhere to back off to. `joint-limits.csv` does this itself:
  J1's home is 1° and not the locked 0°, J3's is 15° and not 30°. If you save on
  a limit the console says so and still saves it — it is your call, just not one
  to make by accident.
- **Fold the arm over its own base rather than reaching out over the bench.**
- **Leave the wrist alone unless you know its travel.** Driving a wrist blind is
  how the claw gets pushed into the mat.
- **Remember what happens at power-off.** The forearm swings down through the
  base area where the loom and the blue cable sit. Hand under the forearm
  *before* the rocker, every time.

## Getting it back into git

The pose is written into `arm-poses.csv` on the Pi, next to the bridge. Pull it
back into the repository afterwards:

```bash
Software/arm-console/sync-poses.sh
```

Then look at the diff and commit it. The file is **append-only** — an older row
stays true as of its date, so expect additions and never edits. The script
refuses if the bench copy has fewer rows than the repo, because a shrinking
history means something was lost.

## If something goes wrong

- **Nothing moves.** Is the rocker on? Is the joint ENABLEd? Is the e-stop
  latched — the strip across the top says so.
- **RECORD WAYPOINT is greyed out.** The console prints the reason underneath
  it. There are three causes and three different fixes.
- **The angle on screen stops changing while you jog.** That is deliberate. The
  console pauses its polling while you hold the control, and catches up when you
  let go. The number is always what was *commanded* — nothing on this arm
  observes the shaft, so judge it with your eyes.
- **A step you did not want.** Press DELETE on that row before saving.

## The one thing this page cannot do

The console cannot see the arm. `camera_verified` is written as `no` on every
pose it saves, and only a human with the evidence in front of them may change
that. The rocker switch and the inline fuse are the real emergency stop; the
button on screen, the Arduino and this software are not safety devices.
