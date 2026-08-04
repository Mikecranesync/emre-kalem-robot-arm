# Design — Draggable envelope, per-joint joystick, and locking an axis

**Date:** 2026-08-04
**Status:** design, awaiting implementation plan
**Touches:** `Software/factorylm_arm_controller/factorylm_arm_controller.ino`,
`Software/arm-console/arm-console.html`, `Software/arm-console/joint-limits.csv`,
`Documentation/SERIAL-PROTOCOL.md`, `Calibration_Notes/calibration-log.csv`

---

## 1. The problem

Every joint on this arm is clamped to `70–110°` with `calibrated=no`. That is a placeholder,
not this arm's travel, and nobody has measured a single joint. Today the only way to widen a
joint is to edit `joint-limits.csv` in a text editor, save it, press LOAD LIMITS FILE, and pick
the file again — for every step of every joint. Finding one joint's real travel is a dozen
round trips through Notepad.

The single-servo bench sketch is not an escape: it is hard-clamped to the same `70–110`
(`ANGLE_MIN` / `ANGLE_MAX`, `emre_kalem_single_servo_bench_test.ino:30-31`). Nothing in the
project today can drive a joint past 110°.

Three things are needed, and they are one workflow:

1. **Widen the range without leaving the page** — drag the soft stops.
2. **Drive the joint by feel** — hold a control, watch and listen for the mechanical stop.
3. **Record what was found** — turn a felt-out range into measured data with one action.

## 2. Non-goals

- No inverse kinematics, no Cartesian control, no "move the gripper *there*".
- No keyboard, gamepad, or hardware pendant control.
- No joysticking more than one joint at a time.
- No recording joystick motion into waypoints.
- **No webcam / external position measurement.** Deliberately deferred — see §11.

## 3. Why hand-rolled rather than an existing stack

Surveyed 2026-08-04: PAROL6, Arctos Studio, Annin AR3/AR4, 6AR, ROS 2 + MoveIt Servo. All of
them are credible and all of them assume **homing** — limit switches, encoders, or both. AR3
auto-calibrates *because* it has limit switches. MoveIt Servo drives off a joint-state topic
fed by encoders, and its safety features are built on knowing where the arm is.

This arm has no position feedback of any kind. An MG996R takes a pulse and reports nothing.
At power-on nothing can know where the elbow is. The firmware's adopt-before-drive rule — the
operator types the angle the joint is sitting at before it will enable — **is** the homing
procedure for a sensorless arm: a human eye is the encoder. Any of the stacks above would have
to be de-featured back to roughly that, while also losing the safety behaviour written
specifically for this machine (no centre-on-boot, the shoulder pair locked until its mirror is
measured, limits that refuse to be invented).

Remaining work here is a firmware change of a few dozen lines plus a day of console work.
Porting is not competitive with that.

**The honest trigger to revisit:** wanting Cartesian control. At that point the first move is
adding position feedback (§11), not new software.

## 4. The card

```
  J3 · Elbow                                         D6
  ┌───────────────────────────────────────────────────┐
  │  UNCALIBRATED — LIMITS ARE A PLACEHOLDER          │
  │  DISABLED                                         │
  │                                                   │
  │  ENVELOPE   0 ····[|═══════════════|]···· 180     │  ← two handles, live-draggable
  │                   65             125              │
  │                                                   │
  │  JOYSTICK   −  |─────────[▓▓]─────────|  +        │  ← springs back to centre
  │                                                   │
  │  COMMANDED  92°        SPEED  stopped             │
  │  GO TO  [   90 ] °  [GO]                          │  ← exact angle, see §6
  │                                                   │
  │  [ENABLE] [DISABLE]   [LOCK THIS AXIS]   30 °/s   │
  └───────────────────────────────────────────────────┘
```

## 5. The envelope

A two-handle range control showing the joint's soft stops.

| Rule | Behaviour |
|---|---|
| Bounds | `0–180`, left handle strictly below right (firmware already enforces: `E10`) |
| While joint **disabled** | Both handles drag freely |
| While joint **enabled** | Both handles drag, **but neither can be pulled past the joint's current commanded angle** — the handle stops dead and turns red, with a line naming why |
| On release | Console sends `LIM <j> <min> <max> <cal>` |
| Calibrated flag | A drag **never** sets `calibrated`. Dragging is not measuring. Only §7 sets it |
| Shoulder (J1) | A drag re-runs the mirror-image check that file loading already does; a range that would swing the second servo outside `0–180` is refused before it is sent |

**Why the handle stops at the joint.** If the envelope could shrink past where the joint is,
a *limit edit* becomes an *unrequested move* — you tidy a number and a loaded arm swings. That
is precisely what the firmware's existing `E9` refusal was written to prevent
(`factorylm_arm_controller.ino:577`). Blocking the handle preserves that guarantee while still
allowing live editing.

## 6. The joystick

Replaces the current position slider. Speed control, not position.

- Springs back to centre on release.
- **Dead zone** of ±14% of half-travel, so a resting hand cannot creep the joint.
- Beyond the dead zone, deflection maps linearly to speed: `1 °/s` at the dead-zone edge up to
  that joint's own `max_deg_per_sec` from `joint-limits.csv` (`30` today). It never exceeds the
  joint's configured speed. Firmware accepts `1–90` (`E12` outside).
- Runs to the envelope edge and stops there, holding. The envelope is a wall; holding longer
  does nothing.
- Release stops the joint where it is.

**On the wire.** Holding sends `SPD <j> <mapped>` then a single `MOV <j> <envelope edge>`, and
the firmware's interpolator does the walking. A held joystick is near-silent on the link rather
than a command per pixel. `SPD` is re-sent only when the mapped value changes by ≥1 °/s.

`SPD` on an enabled joint is already legal — only `LIM` and `MIR` are refused (`E9`). No change
needed there.

**Release sends `STP <j>`** — see §8.

**Dead-man.** Treated identically to letting go: `pointerup`, `pointercancel`, pointer leaving
the window, `visibilitychange` → hidden, window `blur`, or any bridge/transport error.

**Exact positioning.** Losing the position slider means losing the ability to command a precise
angle, so a small `GO TO [ nn ] °` numeric box replaces it. Clamped to the envelope, sends one
`MOV`. *This is a design decision, not a requirement from the brief — confirm or drop it.*

## 7. Locking an axis

A per-card **LOCK THIS AXIS** button. Enabled only when the joint is enabled and both envelope
handles have been moved from their placeholder values at least once this session.

Pressing it:

1. Sends `LIM <j> <min> <max> 1` — the same limits, now flagged calibrated.
2. Updates the in-memory row: `calibrated=yes`, `home_deg` = the joint's current commanded angle
   (the centre the operator settled on).
3. Marks `joint-limits.csv` dirty and offers it for download, so the values survive a reset.
4. Appends a row to `Calibration_Notes/calibration-log.csv` with the date, joint id, pin,
   min, max, home, and horn state.
5. Flips the card's amber `UNCALIBRATED` badge to green `MEASURED`.

**This button is the only thing in the system that may set `calibrated`.** Nothing infers it.
That keeps the project's one hard rule intact — *limits are data, measured by a human, never
invented*. The amber badge is the only warning that a joint's range is fiction; a system that
could clear it on its own would be lying.

Locking does **not** prevent later re-widening. It records that a human measured this range, not
that the range is now immutable.

## 8. Firmware changes

Two, both narrow.

### 8a. `STP <j>` — stop one joint

`doStp()` currently freezes every enabled joint (`factorylm_arm_controller.ino:769-772`). With
one joint live that is invisible; with three live, releasing one joystick freezes the others
where they stand. Not dangerous — freezing never is — but surprising.

- `STP` with no argument keeps its current meaning exactly: freeze every enabled joint.
- `STP <j>` freezes joint `j` only. `E4` for a bad or reserved id, `E6` if that joint is not
  enabled.

### 8b. `LIM` accepted on an enabled joint, conditionally

`doLimSet()` currently rejects any `LIM` on an enabled joint outright
(`factorylm_arm_controller.ino:577`). New rule:

- Joint **disabled** → unchanged.
- Joint **enabled** → accepted **iff** the joint's current commanded angle `setC` lies inside
  the new `[min, max]`. Otherwise `E9` exactly as today, so nothing that previously failed now
  silently succeeds.
- If a pending move's target `tgtC` falls outside the new range, it is **clamped inward**.
  Clamping can only make a move *shorter*. A limit edit can never create travel.
- `MIR` on an enabled shoulder **stays refused**. Changing the mirror relation on two live
  servos sharing one link is a different order of risk and is out of scope here.

`Documentation/SERIAL-PROTOCOL.md` is updated in the same commit — `E9`'s meaning narrows and
`STP` gains an argument, so the protocol table and the console's `ERR_TEXT` map must move
together or the console will explain the wrong thing.

## 9. Safety invariants — unchanged by this work

- Nothing attached at boot; no centre-on-boot; enabling always requires an explicit adopt angle.
- **A drag never causes motion.** New, and the load-bearing invariant of this design.
- Limits are data, measured by a human. Only the LOCK button sets `calibrated`.
- D4/D5 remain one logical joint; joint 2 remains unaddressable.
- `HOLD` ≠ `E-STOP`. `STP` freezes with joints powered and holding; `EST` detaches and latches.
- The rocker switch and the inline fuse remain the only real emergency stop.
- No word in any new UI copy may imply measurement: **position**, **actual**, **measured** and
  **feedback** stay forbidden while the arm has no sensors. Every angle shown is *commanded*.

## 10. Verification

Every case below is exercised with **zero servos powered**, against the real board, before any
motor is involved. The whole stack is designed to be provable that way.

| # | Case | Expected |
|---|---|---|
| 1 | Envelope drag, joint disabled | `LIM` accepted, card updates |
| 2 | Envelope drag, joint enabled, handle away from joint | `LIM` accepted |
| 3 | Handle dragged toward the joint | stops dead at the commanded angle, turns red, names why |
| 4 | `LIM` sent that excludes `setC` (by hand, bypassing the UI) | `E9` |
| 5 | `LIM` narrowing while a move is pending | `tgtC` clamped inward; no new travel |
| 6 | Joystick at several deflections | `SPD` matches the mapping; never exceeds `max_deg_per_sec` |
| 7 | Joystick held to the envelope edge | stops and holds; further holding does nothing |
| 8 | Release | `STP <j>`; that joint stops, others keep moving |
| 9 | All five dead-man paths | identical to release |
| 10 | Shoulder envelope drag producing an illegal mirror image | refused before it reaches the wire |
| 11 | LOCK on a joint whose handles were never moved | button disabled |
| 12 | LOCK | `LIM … 1`, badge green, CSV offered, calibration-log row appended |
| 13 | Reconnect after LOCK, then load the saved CSV | limits and `calibrated` restored |
| 14 | `STP` with no argument | still freezes every enabled joint |

Then, and only then: one joint, horn off, flag taped to the spline, hand on the rocker.

## 11. Deferred — webcam as an external position monitor

Wanted, and it is the *right* long-term answer, but it is a separate piece of work with its own
design. Recorded here because two decisions above are shaped by it.

The idea: a camera watching the arm, reading fiducial markers on each link, giving a genuine
measurement of where each joint actually is. That is the position feedback this arm lacks — the
thing that would let it verify a commanded angle was reached, catch a slipped horn, and
eventually record real poses for playback.

**Two hooks this design leaves open for it:**

1. **Vocabulary.** The protocol forbids *position*, *actual*, *measured* and *feedback* because
   no such quantity exists. A camera would create one. That rule will need an explicit carve-out
   — camera-measured is a different thing from servo-reported, and conflating them would be
   worse than the current ban. Do not quietly relax the rule in the meantime.
2. **The calibration log.** §7 writes commanded values only. When a camera exists, the log gains
   a measured column beside each commanded one, and the difference between them becomes the
   interesting number. The log's column order should not be treated as frozen.

**What it does not change:** nothing in §5–§8 is provisional on the camera. Soft stops, jogging
and human-verified limits are needed either way, and are the precondition for a camera being
useful — you cannot calibrate a camera against an arm whose joints have no known range.

## 12. Open questions

1. **Keep the `GO TO` numeric box?** (§6) It restores exact positioning without a second slider.
   Cheap either way.
2. **Should LOCK also narrow the envelope to what was actually reached**, rather than recording
   the handles as dragged? Recording the reached extremes is arguably more honest, but it means
   the button behaves differently depending on how far you drove. Currently specified as
   *record the handles*.
