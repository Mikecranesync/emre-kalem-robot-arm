# Design — Draggable envelope, per-joint joystick, and locking an axis

**Date:** 2026-08-04
**Revised:** 2026-08-04 after external review (see §13)
**Status:** design, awaiting implementation
**Touches:** `Software/factorylm_arm_controller/factorylm_arm_controller.ino`,
`Software/arm-console/arm-console.html`, `Software/arm-console/joint-limits.csv`,
`Documentation/SERIAL-PROTOCOL.md`, `Calibration_Notes/calibration-log.csv`,
`Software/tests/protocol_check.py` (new)

---

## 1. The problem

Every joint on this arm is clamped to `70–110°` with `calibrated=no`. That is a placeholder,
not this arm's travel, and nobody has measured a single joint. Today the only way to widen a
joint is to edit `joint-limits.csv` in a text editor, save it, press LOAD LIMITS FILE, and pick
the file again — for every step of every joint.

The single-servo bench sketch is not an escape: it is hard-clamped to the same `70–110`
(`ANGLE_MIN` / `ANGLE_MAX`, `emre_kalem_single_servo_bench_test.ino:30-31`). Nothing in the
project today can drive a joint past 110°.

Three things are needed, and they are one workflow:

1. **Widen the range without leaving the page** — drag the soft stops.
2. **Drive the joint by feel** — hold a control, watch and listen for the mechanical stop.
3. **Record what was found** — turn a felt-out range into recorded, acknowledged data.

## 2. Non-goals

- No inverse kinematics, no Cartesian control.
- No keyboard, gamepad, or hardware pendant control.
- No joysticking more than one joint at a time.
- No recording joystick motion into waypoints.
- **No EEPROM persistence.** The repository has no versioned, checksummed persistence
  mechanism, and adding one is a separate piece of work. The firmware keeps nothing across a
  reset, by design (`SERIAL-PROTOCOL.md §0`).
- **No webcam / external measurement.** Deferred — see §12.

## 3. Vocabulary — enforced throughout

These servos have no feedback of any kind. `Servo.read()` returns the last value written, not a
shaft angle. Therefore, in firmware, console, protocol docs, log files and UI copy:

| Allowed | Banned |
|---|---|
| `commanded`, `target`, `held`, `accepted`, `soft limit` | `actual`, `measured`, `feedback`, `position` |

**No software behaviour in this design may be called an emergency stop.** `STP` and the
joystick timeout are *motion aborts* — they stop interpolation and hold the last commanded
value. They do not remove power and they do not know where the shaft is. The rocker switch and
the inline fuse remain the only emergency stop.

The one approved exception is the console's `MEASURED` badge in §7 — it refers to a human's
measurement of mechanical travel, not to anything the servo reported.

## 4. Why hand-rolled rather than an existing stack

Surveyed 2026-08-04: PAROL6, Arctos Studio, Annin AR3/AR4, 6AR, ROS 2 + MoveIt Servo. All are
credible and all assume **homing** — limit switches, encoders, or both. AR3 auto-calibrates
*because* it has limit switches. MoveIt Servo drives off a joint-state topic fed by encoders,
and its safety features are built on knowing where the arm is.

This arm has none of that. The firmware's adopt-before-drive rule — the operator types the angle
the joint is sitting at before it will enable — **is** the homing procedure for a sensorless
arm. Any of those stacks would have to be de-featured back to roughly that, while losing safety
behaviour written specifically for this machine.

**The honest trigger to revisit:** wanting Cartesian control. At that point the first move is
adding external measurement (§12), not new software.

## 5. The card

```
  J3 · Elbow                                          D6
  ┌────────────────────────────────────────────────────┐
  │  UNCALIBRATED — LIMITS ARE A PLACEHOLDER           │
  │  DISABLED          ENVELOPE: ACKNOWLEDGED          │
  │                                                    │
  │  ENVELOPE   0 ····[|═══════════════|]···· 180      │
  │                   65             125               │
  │                                                    │
  │  JOYSTICK   −  |─────────[▓▓]─────────|  +         │
  │                                                    │
  │  COMMANDED  92°        SPEED  stopped              │
  │  TARGET ANGLE  [   90 ] °  [SEND]                  │
  │                                                    │
  │  [ENABLE] [DISABLE]   [LOCK THIS AXIS]   30 °/s    │
  └────────────────────────────────────────────────────┘
```

## 6. The envelope

A two-handle range control showing the joint's soft stops, in **logical joint space**.

| Rule | Behaviour |
|---|---|
| Bounds | `0–180`, and a **minimum usable span of 5°** — narrower is refused (`E10`) |
| While joint **disabled** | Both handles drag freely |
| While joint **enabled** | Both handles drag, **but neither can be pulled past the joint's current commanded angle** — the handle stops dead, turns red, and names why |
| On release | Console sends one `LIM <j> <min> <max> <cal>` and waits for the reply |
| Calibrated flag | A drag **never** sets it. Only §8 does |
| Shoulder (J1) | A drag re-runs the mirror-image check; a range that would drive the second servo outside `0–180` is refused before it is sent |

### 6a. Limits are enforced in logical space, before any physical transform

`LIM` is applied to the **six-axis logical joint**, and enforcement happens **before**
`degToCmd`, before any calibration offset, before shoulder mirroring, and before any physical
servo write. The shoulder stays one logical joint; the mirrored physical servo is derived from
the already-accepted logical command.

After mirroring and offsets, the resulting physical command is clamped again to the servo's own
absolute bounds. Enforcing only in logical space would let a mirror offset push a physical
servo past its travel while the logical value looked legal.

### 6b. `LIM` is atomic

Validation runs to completion **before any field is written**. A rejected `LIM` leaves the
previous envelope exactly as it was — there is no partially applied envelope, and no state where
`min` has been updated and `max` has not.

Validated: joint id is real and not the reserved id; every argument is a whole number with no
trailing garbage; `min < max`; span ≥ 5°; both inside `0–180`; `cal` is `0` or `1`.

### 6c. Narrowing while the joint is driven

Accepted only if the new range still contains the joint's own commanded value. A range that
would exclude it is refused with `E9`, because applying it would turn a limit edit into an
unrequested move — the operator tidies a number and a loaded arm swings. That is what `E9` was
written to prevent; this narrows the refusal rather than removing it.

If a pending target falls outside the new range it is **clamped inward**. Clamping can only make
a move shorter. **A limit edit can never create travel.**

## 7. The joystick

Replaces the position slider. Speed control, not position.

- Springs back to centre on release.
- **Dead zone** of ±14% of half-travel, so a resting hand cannot creep the joint.
- Beyond it, deflection maps linearly from `1 °/s` to that joint's own `max_deg_per_sec`
  (`30` today). Never exceeds it.
- Runs to the envelope edge and stops there, holding. Holding longer does nothing.
- Release aborts motion and holds the last commanded value.

### 7a. `JOG` — a separate verb, because of the timeout

**Revised after review.** The first draft had the joystick send `SPD` then one `MOV` to the
envelope edge and go quiet. That is elegant and wrong: a board that has heard nothing for two
seconds cannot tell a steady hand from a dead USB cable, and the joint would keep walking to the
edge of an envelope the operator may have just widened to `0–180`.

So joystick motion gets its own verb and its own heartbeat:

```
JOG <j> <-1|0|1>        ->  OK JOG J<j> DIR=<d>
```

- `JOG` sets the joint's target to the envelope edge in that direction and **arms a command-age
  timer** for that joint.
- The console re-sends `JOG` every **200 ms** while the joystick is held.
- If a joint's jog timer is not refreshed within **600 ms** — three missed heartbeats — the
  firmware aborts that joint's motion, holds the last commanded value, and emits
  `EVT JOGTIMEOUT J<j>`.
- `JOG <j> 0` and `STP <j>` both clear the timer. So does `DIS`, `EST`, and any `MOV`.
- **`MOV` does not arm the timer.** A finite move — the `TARGET ANGLE` box, a waypoint — runs to
  completion and must never be cut short by a joystick timeout it did not ask for.

Speed still comes from `SPD`, sent only when the mapped value changes by ≥1 °/s.

**600 ms is chosen against the existing rate**, not picked freely: the console's status poll and
command round-trip both sit comfortably under 200 ms at 115200 baud, so three consecutive
missed heartbeats is a real fault rather than jitter. At the default 30 °/s a 600 ms overrun is
18° of unintended travel — inside a typical envelope, and the reason the timeout is not longer.

### 7b. The timeout is not an emergency stop, and is distinguishable from one

| | What it does | What it does **not** do |
|---|---|---|
| `STP <j>` (operator) | aborts that joint's interpolation, holds the commanded value | remove power; detach; know the shaft angle |
| Jog timeout (fault) | same, plus `EVT JOGTIMEOUT J<j>` | remove power; detach; latch |
| `EST` / `!` | detaches every channel, drives pins low, **latches** | remove power — the rocker does that |

The console must render a jog timeout differently from a stop the operator asked for. A hold the
operator did not request is a symptom.

This is deliberately gentler than the existing serial watchdog (`WDG`), which detaches
everything and latches on host silence. The watchdog is the coarse net for a dead host; the jog
timer is the fine net for a stalled joystick, and holding is the right response because a
detach makes a loaded arm sag.

### 7c. Dead-man on the console side

Treated identically to letting go: `pointerup`, `pointercancel`, pointer leaving the window,
`visibilitychange` → hidden, window `blur`, or any bridge/transport error. The console stops
sending `JOG` and sends `JOG <j> 0`. The firmware timeout is the backstop for the cases the
browser never gets to report.

### 7d. Exact angle entry

Losing the position slider removes the only way to command a precise angle, which calibration
and waypoint work both need. **Resolved: keep it.**

- Labelled **`TARGET ANGLE`**, not "go to".
- Sends only on Enter or an explicit button press. **Never transmits while typing.**
- Clamped and validated against the envelope before sending.
- Shows the board's acknowledgment; a value the firmware clamped is displayed as accepted, not
  as what was typed.

## 8. Locking an axis

A per-card **LOCK THIS AXIS** button, enabled only when the joint is enabled and its envelope
handles have been moved from the placeholder at least once this session.

Pressing it:

1. Sends `LIM <j> <min> <max> 1` and **waits for the acknowledgment**.
2. On acknowledgment: records `calibrated=yes` and `home_deg` = the joint's current commanded
   value, offers the updated `joint-limits.csv`, and appends a calibration-log row.
3. On rejection: changes nothing and shows what the firmware said.

**This is the only code path permitted to set `calibrated`.** Nothing infers it. The amber
`UNCALIBRATED` badge is the only warning that a joint's range is fiction; a system that could
clear it on its own would be lying.

### 8a. What the log row actually claims

**Revised after review.** The row records **accepted commanded soft limits** — the handle
values the firmware acknowledged. It does **not** claim to be the mechanical extremes of the
joint. Nothing in this system can measure those.

Recorded per lock: timestamp, logical joint id, the accepted `min` / `max` / `home`, the
firmware's literal acknowledgment line, firmware version (`FW=` from `VER`), console version,
and a free-text note.

The calibration schema is explicitly **not frozen**: §12's external measurement will add fields
beside these, and the difference between commanded and externally-observed will be the
interesting number. Consumers must tolerate new columns.

## 9. Startup, reconnect, and the acknowledgment gate

**Revised after review.** Opening the serial port resets the board, and the firmware keeps
nothing. The console must never assume its own in-memory or browser-saved limits are live on the
board.

Every joint's envelope carries one of three states, shown on the card:

| State | Meaning |
|---|---|
| `DEFAULT` | the board is running its compiled-in conservative defaults |
| `PENDING` | a `LIM` has been sent and not yet acknowledged |
| `ACKNOWLEDGED` | the firmware acknowledged this exact envelope |

**All motion controls — joystick, `TARGET ANGLE`, `ENABLE` — stay disabled until all six
envelopes are `ACKNOWLEDGED`.** A joystick that can drive against limits the board never
confirmed is the failure this gate exists to prevent.

Firmware boot defaults stay conservative and compiled-in. No EEPROM (§2).

## 10. Safety invariants

- Nothing attached at boot; no centre-on-boot; enabling always requires an explicit adopt value.
- **A drag never causes motion.** Load-bearing.
- **A limit edit never creates travel.** Clamping only ever shortens a move.
- Limits are data. Only §8 sets `calibrated`.
- D4/D5 remain one logical joint; joint 2 remains unaddressable; the mirrored servo is derived
  from the accepted logical command, never commanded directly.
- No software behaviour is called an emergency stop (§3).
- The rocker switch and the inline fuse remain the only emergency stop.

## 11. Verification

### 11a. The protocol harness — non-motion by default

`Software/tests/protocol_check.py` must be **safe to run with servo power on**. Its default mode
enables nothing and moves nothing. Bounded motion tests live behind an explicit `--motion-ok`
flag that first prints a physical-safety acknowledgment.

Default (non-motion) coverage: handshake, help, status; malformed commands; missing arguments;
excess arguments; invalid and reserved joint ids; reversed, equal, and out-of-range limits; span
below the minimum; trailing garbage; overlong input; repeated identical commands; reply grammar
matched exactly; and **state unchanged after every rejected command**.

`--motion-ok` coverage: enable one joint, jog it, confirm the timeout fires and holds, confirm
`STP <j>` stops one joint and leaves others alone, confirm narrowing clamps a pending target.

### 11b. The browser self-test

`?selftest=1` runs from `file://` with no board and no bridge. It must exercise the pure maths,
clamping, deflection mapping, **command string formatting**, **malformed saved CSV data**,
**pointer cancellation**, and **shoulder mirroring**; render each expected/observed failure; and
emit exactly one terminal sentinel — `SELFTEST_PASS` or `SELFTEST_FAIL`. It must complete
deterministically before headless inspection. A wrapper script fails unless `SELFTEST_PASS` is
present and `SELFTEST_FAIL` is absent. Chrome's exit code is not evidence.

### 11c. Bench sequence

1. Servo power **off**: parser and protocol tests.
2. Servo power on, **no load, clear workspace**.
3. One low-risk joint, small increments.
4. Verify stop, timeout, limit rejection, clamping.
5. That joint at each accepted boundary.
6. Reconnect and reset behaviour.
7. **Shoulder pair last**, arm supported, tiny moves.
8. Operator's hand stays on the rocker throughout.

Captured: firmware build output, self-test output, harness output, commands and replies, final
accepted limits, reset/reconnect evidence, shoulder mapping evidence, and every deviation from
this design.

## 12. Deferred — external measurement

An external camera watching fiducial markers on each link would give a genuine observation of
where each joint actually is: the feedback this arm lacks. It is the right long-term answer and
it gets its own design.

Two hooks are left open:

1. **Vocabulary (§3).** The ban on `actual` / `measured` / `feedback` / `position` exists because
   no such quantity exists today. An external observer creates one, and will need an explicit
   carve-out — externally-observed is a different thing from servo-reported, and conflating them
   would be worse than the current ban. Do not relax the rule in the meantime.
2. **The calibration schema (§8a).** Explicitly not frozen. Externally-observed values will sit
   beside the commanded ones.

Nothing in §6–§9 is provisional on it. Soft stops, jogging and human-recorded limits are needed
either way, and are the precondition for a camera being useful at all.

## 13. Review corrections incorporated

| # | Correction | Where |
|---|---|---|
| 1 | `STP` is a motion abort, not an emergency stop; vocabulary enforced | §3, §7b |
| 2 | Board-side jog timeout, distinguishable from an operator stop; `MOV` must not inherit it | §7a, §7b |
| 3 | Limits enforced in logical space before `degToCmd`/offsets/mirroring; atomic `LIM`; minimum span | §6a, §6b |
| 4 | Conservative compiled defaults; acknowledgment gate on reconnect; no EEPROM | §2, §9 |
| 5 | Harness is non-motion by default, `--motion-ok` for bounded motion | §11a |
| 6 | Exact-angle entry kept, labelled `TARGET ANGLE`, send-on-commit only | §7d |
| 7 | `LOCK` records accepted commanded limits, not mechanical extremes; schema not frozen | §8a |
| 8 | Self-test sentinel `SELFTEST_PASS` / `SELFTEST_FAIL`, wider coverage, wrapper script | §11b |

**The one design change, not a wording fix:** correction 2 forced the joystick from
"two lines then silence" to `JOG` + a 200 ms heartbeat (§7a). A command-age timeout cannot
distinguish a steady hand from a dead cable, so the hand has to keep speaking.

## 14. Open questions

None. Both prior open questions are resolved above: exact-angle entry is kept (§7d), and `LOCK`
records the accepted commanded limits rather than reached extremes (§8a).
