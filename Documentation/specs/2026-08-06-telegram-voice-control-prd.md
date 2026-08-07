# PRD — Commanding this arm from Telegram, and later by voice

**Date:** 2026-08-06
**Status:** requirements, awaiting the operator's answers in §17. Nothing built.
**Brief:** one sentence from the operator — *"continue building towards prd vision of controlling
this via telegram and voice commands."* Everything below that is not quoted from an existing
document is an **assumption**, marked as one, and every decision that could go either way is a
question in §17 rather than a guess.
**Touches (proposed):** `Software/arm-telegram/` (new), `Software/tests/` (new offline harness).
**Reads only:** `Software/arm-console/arm-poses.csv`, `Software/tests/motion_verify.py`
(`ArmLink`), `Software/tests/cycle_poses.py`, and the holder daemon's link directory beside
`Software/arm-console/hold_arm.py`. **Nothing here modifies the firmware or `hold_arm.py`.**

---

## 1. The problem, and the doctrine it collides with

The operator wants to send the arm a message and have it move. Reasonable, for a bench rig that
now cycles `pick`↔`storage` cleanly. It also runs straight into the rule this project states most
plainly — `.claude/skills/arm-bench-safety` §4, verbatim:

> The arm is a **bench rig**, and driving it is a deliberate bench act by an operator standing
> next to it. That is the entire exception. It is **never** a customer-shipped path, it never
> runs unattended.

Telegram is, by construction, command from somewhere else. And this machine has no software stop
worth the name: `STP` holds with the joints still driven, while `EST` and the watchdog **detach**
— and a detached, gravity-loaded arm falls. The rocker switch and the fuse are the only real
stop. No pose in `arm-poses.csv` is self-supporting; the holder daemon's torque is the only thing
keeping the arm up.

This document does not resolve that with a warning paragraph. It resolves it structurally: an
**arming contract with a TTL** (§5), **refusal rather than queueing** (§6), a **freshness verdict
on every reply** (§7), and a **vocabulary that cannot lie** (§3).

**Nothing here makes remote command as safe as standing at the bench.** It makes the gap visible
and bounded instead of hidden. §17 Q4 asks whether the bot may run at all when the operator is out
of the room. The recommendation is **no**.

## 2. Non-goals

- **No emergency stop.** Nothing here is one and no copy may call it one (§10).
- **No free-form joint angles in Phase 1** (§9a). Widening is the operator's call (§17 Q2).
- **No second owner of the serial port** — not ever, not as a fallback (§4).
- **No firmware change**, no new verb, no new error code, and **no change to `hold_arm.py`** —
  the bot is a client of the file channel it already exposes.
- **No camera work.** The camera is re-aimed at the ceiling and uncalibrated; §5b is explicit
  that Phase 1 therefore cannot satisfy the hands-clear frame check at all.
- **No inverse kinematics, no Cartesian control, no new poses. No gripper command** — J6 does not
  articulate (gear slipping on the motor shaft). J0 is a dead servo and excluded everywhere.
- **No speech-to-text backend chosen.** Phase 1 is text only (§11).

## 3. Vocabulary for bot replies — stricter here than anywhere else

`SERIAL-PROTOCOL.md` §0 bans `actual`, `measured`, `feedback` and `position` because these servos
have no feedback of any kind: the firmware knows only what it last *commanded*. That ban matters
more in a Telegram reply than in any prior surface, because **the reader cannot glance at the
bench and catch the lie.**

| Allowed in bot copy | Banned |
|---|---|
| "last accepted command was `storage`" | "the arm is at storage" |
| "commanded J3 to 64" | "J3 is at 64" |
| "the board acknowledged `OK MOV J1 REQ=40 SET=40 CL=0`" | "the move completed" |
| "status file is 4 s old; daemon PID 8123 alive" | "the arm is held" |
| "refused — not armed" | "stopped for safety" |

A board ack proves the firmware accepted a command. It does not prove a motor turned — that cost
a whole afternoon on D3, where the firmware provably drove a joint at 29.2 °/s for 3.5 s and
nothing moved. **Every bot reply that reports motion quotes the board's own line and claims
nothing beyond it.**

## 4. Architecture — the bot is a client, never a port owner

```
Telegram --HTTPS long-poll--> arm_telegram.py --writes--------> arm_cmd.txt     \
                                     |  <--byte-offset tail---- arm_hold.log     >  hold_arm.py --> COM5
                                     |  <--mtime + PID--------- arm_status.txt  /     (sole owner)
```

**One owner per port, and it is the daemon.** `GET /rx` is destructive, so a second poller steals
replies; opening the port DTR-resets the board, dropping every limit back to the firmware default
70–110 with `MIR=UNKNOWN` and `CAL=0`. J6's 10–70 lies almost entirely below that default, so a
reset silently clamps it. Therefore:

- **If the daemon is not alive, the bot refuses everything and says so.** It never opens COM5 "to
  help". There is no fallback path and no degraded mode.
- The bot sends one protocol line by writing `arm_cmd.txt`, and reads that command's own reply
  with the **log-offset primitive**: record `arm_hold.log`'s byte length *before* writing, then
  tail from that offset. Import `ArmLink` from `Software/tests/motion_verify.py` — `offset()`,
  `tail()`, `send()`, `sta_row()`, `field()`. Do not reimplement it.
- **A deterministic guard, in the test suite:** the bot module must not import `serial` and must
  not contain the string `COM5`. This repo already keeps guards of that shape — the CSV field-count
  validator that caught a malformed row reading `pass=yes`. Cheap, and it fails loud.

Two firmware facts to respect: **inbound lines cap at 48 characters** including the terminator
(longer is discarded with `ERR E8 LINE`), and **one command is outstanding at a time** — there are
no sequence numbers, the reply is correlated by the echoed verb.

## 5. The presence / arming contract

| | |
|---|---|
| **Opened by** | an explicit `/arm <pose>` from an allowlisted chat id, naming the pose the operator asserts the arm is **currently in** |
| **TTL** | fixed minutes, **operator's choice — §17 Q3**; recommended 5. Motion outside the window is refused; `/stop` is the sole exception (§9) |
| **Closed by** | TTL expiry, `/disarm`, any refusal caused by a stale or dead daemon, or any `LATCHED` / `re-ENA` line in the log (§5c) |
| **On expiry** | **NO MOTION ACCEPTED.** The bot sends nothing to the daemon and nothing to the arm |
| **Extended by** | nothing implicit. Not by activity, not by a completed move. Re-arm deliberately |

### 5a. Expiry must not command the arm — this is the trap

The instinctive implementation of "the window expired, make it safe" is to send `EST` or `DIS`.
**On this arm that drops it.** `EST` and the watchdog detach every joint, and a gravity-loaded
arm falls. So expiry is a state change **in the bot**, not a command to the **arm**:

> On TTL expiry the bot stops accepting motion. It sends nothing. `hold_arm.py` goes on holding
> the arm exactly as it was.

Write it that way in the code and in the copy. Do not "fail safe" by detaching.

### 5b. What arming proves, and what it cannot

**Arming proves:** a message arrived from a chat id on the allowlist, at a known time, and
whoever sent it asserted a starting pose.

**Arming does not prove:** that a human is standing at the bench; that hands are clear of the
claw; that the arm is physically where the declared pose says; or that the phone was in the
operator's hand rather than on a table.

**Telegram cannot prove presence. Nothing in this design changes that.** Arming is a *deliberate
act with a short expiry* — it bounds how long a stale intention stays valid. It is not evidence
of a person.

There is a specific, named gap. `arm-bench-safety` §2 requires **capturing a camera frame and
looking at it** before commanding any joint after a human has reached into the arm — because that
has already happened here: the operator said "go ahead" and in the very next frame his hand was
still cupped around the claw. **The camera is currently aimed at the ceiling, so Phase 1 cannot
satisfy that rule at all.** Arming is a weaker substitute for a rule this repo already wrote and
already learned the hard way. Describe it to the operator that way, not as an equal.

### 5c. A latch voids the declared pose

`hold_arm.py`'s health check clears a latch and re-enables, and `enable_all()` re-sends
`ENA <j> <adopt>` — which **snaps joints back to their adopt angles.** That reads exactly like
"the joint moved on its own", and it means the declared pose is no longer true. So if
`arm_hold.log` gains a `LATCHED` or `re-ENA` line after arming, the bot **disarms, refuses further
motion, and reports the log lines verbatim.** Re-arming needs a fresh `/arm <pose>` — i.e. a human
looking at the arm again.

## 6. Refuse — never queue

A motion request that arrives unarmed, expired, stale, or while another move is in flight is
**refused with a reason**. It is never buffered for later delivery. This project already settled
the identical argument, for jog heartbeats, in `2026-08-04-envelope-joystick-design.md` §7a-ii:

> A heartbeat is only meaningful if it is *fresh*. A queued one asserts, on arrival, that a hand
> was on the control at a moment that has already passed.

A queued **motion** command is worse: it asserts consent that has already expired. Ninety seconds
later the operator may have re-armed for a different purpose, walked away, or put his hand in the
claw — and the queue fires anyway, the arm moving for a reason nobody present can account for. A
refusal costs one retyped message.

**A pose transition takes real time** — the measured round trip is 30.2 s at 12 °/s and 9.8–10.5 s
at 90 °/s, so one direction is roughly 5–15 s. While a transition is in flight the bot is busy,
and a second request is refused rather than stacked.

## 7. Freshness is a first-class requirement, not a nicety

This is the reporting hazard, and it has already nearly ended a session with a false report. From
the 2026-08-06 evening findings §9: the daemon crashed when the bench was powered down, all five
joints detached at that instant, and **`arm_status.txt` was left behind showing `EN=1` on every
joint.** A stale status file reads exactly like a healthy held arm. A Telegram bot is precisely
the surface where that lie does the most damage, because the reader cannot check.

| Verdict | Condition | Motion |
|---|---|---|
| `LIVE` | daemon PID alive **and** `arm_status.txt` mtime inside the freshness window | allowed (if armed) |
| `STALE` | PID alive but the status file has not been rewritten inside the window | **refused** |
| `NO DAEMON` | PID gone, or the link directory is missing | **refused** |

`STALE` **refuses motion.** It does not merely annotate the reply. A daemon that has stopped
rewriting its status file is a daemon whose loop is not running — and that loop's heartbeat is
what feeds the 4000 ms watchdog.

**Freshness window — an assumption, and the reasoning is the interesting part.** `arm_status.txt`
is rewritten on the daemon's **5 s poll**, so it is stale by design even when everything is
healthy; any threshold tighter than 5 s false-alarms constantly. Proposed **15 s** — three poll
periods, so two missed polls are tolerated and a stopped loop is caught inside about a quarter of
a minute. A desk choice, never observed under load: tighten or loosen it against a real run.

### 7a. Never report a move's outcome from `arm_status.txt`

The Telegram-shaped version of the trap in `arm-serial-control` §4. The file is only rewritten on
the 5 s poll, so **immediately after a `MOV` it still holds pre-move state — which says `MOV=0`.**
A bot that reports "done" by reading it right after issuing a move reports a lie that looks
exactly like success. So:

- **Command outcomes come from the log-offset tail of that command's own reply.** Quote the board
  line (`OK MOV J1 REQ=40 SET=40 CL=0`) and check `CL=`. **`CL=1` means the firmware clamped it
  and the joint is not where you asked** — a failed command, not a warning.
- **`arm_status.txt` is only for the periodic "is it still held" question** and for `JTO=` /
  `ES=` / `WD=` surveillance — never for "did my move finish".

## 8. Who may command it

- **An allowlist of Telegram chat ids**, supplied by environment variable, checked on every update
  before anything else — before arming state, before parsing, before everything.
- **Everything else is silently refused.** No reply, no error, no hint that a bot exists. A token
  can leak; a bot that answers strangers tells them what it controls.
- **Silent to the sender, loud in the audit log** (§13): rejected chat id, username, raw text.

## 9. Command vocabulary — Phase 1

**Recommendation: named poses only, plus read-only status and stop.**

| Command | Effect | Armed? |
|---|---|---|
| `/arm <pose>` | open the window, declaring the pose the arm is in now | — |
| `/disarm` | close the window; sends nothing to the arm | — |
| `/status` | freshness verdict + a fresh `STA` through the file channel | no |
| `/go storage` | run the `pick`→`storage` reverse path | **yes** |
| `/go pick` | run the `storage`→`pick` path | **yes** |
| `/stop` | §10 — refuse further motion, send `STP`, report | no — see below |

Everything else — including any message containing a number — is refused with a list of what is
accepted.

**`/stop` is the one command that does not require an armed window**, because a transition takes
5–15 s and can outlive a TTL that expired while it ran — a `/stop` refused for being unarmed would
be refused at exactly the moment it was wanted. It still needs an allowlisted chat id, and is safe
to widen only because it can only ever *reduce* motion. It does not generalise.

### 9a. Why not arbitrary joint angles

`arm-poses.csv` states its own rule in capitals, and that rule is the argument:

> **TO LEAVE A POSE, REVERSE ITS OWN ENTRY PATH.** Do not interpolate between two poses in this
> file and assume the middle is clear.

The endpoints are not the safety property. Two poses can both be safe while the straight line
between them drives the claw through the bench, the wire loom, or the base housing. `entry_path`
is a column precisely because that was learned. **A surface that accepts arbitrary joint angles
can violate that rule trivially** — one number, typed from a phone, by somebody who cannot see the
claw. Named poses whose paths were driven and photographed cannot. Free-form angles are a later
phase behind an explicit widening decision (§17 Q2), not an increment somebody adds because it
seemed convenient.

### 9b. Transitions are directional, and the arm cannot be asked where it is

`pick`'s `entry_path` reads *from storage in 3 phases*, with the reverse spelled out step by step:
lift J1→40 first to get the claw off the mat, neutralise J3/J4/J5 while high, then fold J1→88. It
is not a symmetric interpolation and it is not valid from an arbitrary start.

Nothing in this system observes a shaft, so **the bot cannot know which pose the arm is in.**
Hence the declared pose at arming time — the same contract shape as `ENA <j> <adopt>`, where the
firmware pre-loads a pulse before attaching and the number is *always a human's estimate*. The
bot's copy should say so in those words.

No pose declared → refuse. A transition that does not start from the declared pose → refuse, and
say what was declared. On a completed transition the declared pose becomes the destination — but
only if every waypoint returned `CL=0` and the log is clean of `LATCHED` / `re-ENA`.

**Reuse `Software/tests/cycle_poses.py` for the phase ordering.** It is the artifact that ran 16
clean cycles with zero clamps, zero joint timeouts and zero watchdog latches. Do not re-encode the
phases inside the bot.

### 9c. Speed

`joint-limits.csv` records `max_deg_per_sec=30` per joint. The 40–90 °/s runs exceeded it,
cleanly, and the findings are explicit that this is *"recorded intent, not a firmware limit, and
running above it is a deliberate act."* **A message from a phone is not a deliberate bench act.**
Phase 1 therefore commands at or below the recorded intent — proposed **25 °/s**, about a 19 s
round trip. Raising it is the operator's decision, not a tuning default. The firmware refuses
above 90 anyway (`ERR E12 SPD JOINT=1 REQ=120 MIN=1 MAX=90`).

### 9d. No gripper verb

Both poses carry `j6=70` as a **commanded** value only. The gripper does not articulate — firmware
exonerated, the prong silhouette identical at commanded 10 and 70, the operator's hand-check
diagnosis *the gear is slipping around the motor shaft*. A remote surface must not command a joint
whose mechanical state is unknown, and `arm-bench-safety` forbids a gripper command with fingers
in the claw — which a phone can never rule out. J0 is a dead servo and appears in no pose.

## 10. What `/stop` honestly means

**There is no remote emergency stop. There cannot be one.**

| The bot **can** | The bot **cannot** |
|---|---|
| stop issuing new motion, immediately | remove power |
| refuse every subsequent motion request until re-armed | know where any shaft is |
| send `STP` — aborts interpolation, **holds** with joints driven | make the arm safe to touch |
| report the board's reply verbatim | undo a move already completed |

`/stop` therefore disarms, sends `STP`, quotes the reply, and says in plain words: **"Motion
stopped and further commands refused. This did not remove power. If you need the arm safe, put a
hand under the forearm and use the rocker switch."**

`EST` is on a **forbidden-verb list** for this surface, alongside `DIS`, `CLR`, and anything that
opens the port. `EST` detaches and the arm falls. A remote user cannot catch a falling arm.

## 11. The voice path

**Phase 1 is text only.** Voice is real work, not a flag. Telegram delivers voice notes as
**OGG/Opus** (the documented Bot API format for the `voice` type). Turning that into a pose name
needs a decoder *and* a speech-to-text model — a dependency chain and, for most backends, a model
download. Neither is decided, and this document will not decide it by quietly picking a default.

Phase 1 builds the seam instead: `transcribe(audio_bytes: bytes, mime: str) -> str | None`. One
function. The bot calls it, then feeds the resulting text through **exactly the same allowlist,
arming, freshness and refusal path as a typed message** — voice gets no privileges of its own. The
Phase 1 implementation returns `None` and the bot replies *"voice is not enabled — send a text
command."* The backend is the operator's choice (§17 Q1) and stays unlisted here.

**Fail closed on transcription.** An unrecognised transcription is refused, **never** fuzzy-matched
to the nearest pose name — "pick", "quick" and "stick" are one bad microphone apart, and a
near-miss that moves the arm is exactly the failure a phone-shaped interface invites. Cheap to
state now; expensive to retrofit once somebody has added a similarity score.

## 12. Dependencies, licensing, and the token

**Decision: `python-telegram-bot` is off the list.** It is LGPL-3; this project allows Apache-2.0
or MIT only. The Telegram Bot API is plain HTTPS with JSON, so Phase 1 long-polls `getUpdates`
with **`urllib` from the standard library**. **Phase 1 adds zero new dependencies** — the strongest
true statement available, and the reason to prefer this shape over a client library. The dependency
chain appears only in the voice phase, which is part of why voice is deferred. Follow the precedent
in `Software/lerobot_robot_emre_arm/pyproject.toml`, which **flags** pyserial and numpy as
BSD-3-Clause and off-list rather than passing them silently: anything the voice phase needs gets
the same written note, with its licence named, before it is installed.

**The token is a secret.** Supplied by environment variable (`ARM_TELEGRAM_BOT_TOKEN`; the
allowlist likewise, `ARM_TELEGRAM_ALLOWED_CHAT_IDS`). **Never committed** — not in a file, not as
a default, not in a test fixture. **Never written to the audit log**, and never echoed inside an
error returned to Telegram: a Bot API HTTP failure can carry the URL, and the token is *in* the
URL. Scrub before logging or replying. The bot refuses to start if either variable is missing.

## 13. The audit log

Append-only, one row per decision, **accepted and refused alike** — a refusal history is what
tells the operator whether something is probing the bot. Per row: timestamp; chat id; username if
present; the raw inbound text (or `voice` plus duration); the decision (`accepted` / `refused` +
reason); the exact protocol line sent to the daemon, or none; the daemon's reply verbatim; the
freshness verdict; and the armed state with its remaining TTL. Never the token, never any
environment-variable value.

## 14. Phases, and acceptance criteria that run offline

The bench is disconnected. Every criterion below is checkable with **no hardware**. Preferred
substrate: `Software/arm-sim/` (a firmware simulator + `fake_serial` duck type from a parallel
session — verify its contract first); point the real `hold_arm.py` at it and the real file channel
appears, 5 s poll included. Fallback: a temp dir with the three files and a canned-`OK` consumer.
**No test opens a port** (§4).

### Phase 1 — text, named poses, armed window

- [ ] Motion refused when unarmed; refused after TTL expiry with the **clock monkeypatched, not
      slept**, and the stub records that **no line was written** to `arm_cmd.txt`.
- [ ] A non-allowlisted chat id gets **no reply at all**, and still produces an audit row.
- [ ] `STALE` mtime refuses motion. A dead PID refuses (`NO DAEMON`) and never opens a port.
- [ ] Unknown pose refuses; a numeric joint angle refuses; a transition whose start ≠ the declared
      pose refuses.
- [ ] A second motion request while one is in flight is **refused, not queued** — assert the queue
      is empty afterwards, not merely that ordering held.
- [ ] The log-offset tail returns *this* command's reply and never a previous one — seed the log
      with prior traffic first, then assert.
- [ ] A `LATCHED` / `re-ENA` line mid-session disarms the bot (§5c). A `CL=1` reply is reported as
      a failed command, not success.
- [ ] Reply copy contains none of the banned words (§3) — assert on the strings. One audit row per
      decision, accepts and refuses both. The bot module imports no `serial`, contains no `COM5`.
- [ ] `/stop` is accepted while unarmed, disarms, sends exactly `STP`, never `EST` or `DIS`.

**Phase 1 is not complete until it has been run once at the bench, with the operator present, the
daemon live, and a hand near the rocker.** The offline suite proves the logic. It proves nothing
about the arm.

### Phase 2 — voice, behind the same gates

- [ ] `transcribe()` returning `None` refuses and says voice is not enabled. A transcription that
      does not exactly match a known command refuses; near-misses ("quick", "stick") all refuse.
- [ ] A transcribed command goes through allowlist, arming, freshness and refusal identically to
      typed text — run the same test table twice, once typed, once via a stub transcriber.
- [ ] Any new dependency carries a written licence note before it is installed.

### Phase 3 — free-form angles, only if the operator widens the scope (§17 Q2)

Not specified here. It needs its own design, because it deletes the §9a argument outright and would
need something else put in its place.

## 15. Safety invariants

- Nothing in this feature is an emergency stop, and no copy calls one that.
- **Expiry and refusal never command the arm.** The bot goes quiet; the daemon keeps holding.
- `EST`, `DIS`, `CLR` and anything that opens the port are forbidden verbs here. The daemon is the
  sole owner of COM5, with no fallback.
- Motion only inside an armed window, only from an allowlisted chat id, only on `LIVE`, only along
  a recorded `entry_path`, from the declared pose. `/stop` is the sole window exception (§9).
- Refuse, never queue. No gripper, no J0, no angle outside a recorded pose. Every outcome quotes a
  board line; `CL=1` is a failure. The rocker and the fuse remain the only emergency stop.

## 16. The lessons this design is built on

No bench result is claimed here — nothing in this document has run against hardware, and the arm
was powered down at 21:55. This is the register of **existing** findings that each shaped a
requirement above, so the reasoning is traceable rather than asserted.

| Lesson, and where it was learned | What it forced |
|---|---|
| `arm_status.txt` showed `EN=1` on every joint after the daemon died — a stale file reads exactly like a healthy held arm (evening findings §9) | §7 entire, and `STALE` refusing motion |
| `arm_status.txt` is pre-move immediately after a `MOV`, and says `MOV=0` (`arm-serial-control` §4) | §7a — outcomes come from the log-offset tail, never the status file |
| A queued heartbeat asserts a hand was on the control at a moment already past (joystick design §7a-ii) | §6 — refuse, never queue |
| `EST` and the watchdog detach, and a loaded arm falls (`arm-bench-safety` §1) | §5a — expiry is a bot state change that sends nothing |
| `enable_all()` re-`ENA`s and snaps joints to adopt angles, which reads as spontaneous motion (`arm-serial-control` §2) | §5c — a latch voids the declared pose |
| Two safe poses can have an unsafe straight line between them (`arm-poses.csv` header) | §9a — named poses only |
| The operator said "go ahead" while his hand was still cupped around the claw (`arm-bench-safety` §2) | §5b — arming named as a *weaker substitute*, not an equivalent |
| A board ack is not motion; D3 acked at 29.2 °/s for 3.5 s and nothing turned (and four measurements that night saturated on something that was not the arm) | §3 — replies quote the board line and claim nothing more; §7's verdict is three-way because "no news" is not "healthy" |

## 17. Open questions for the operator

Plain English, one sentence per option. Nothing is built past Phase 1's skeleton until answered.

**Q1 — How should the arm understand a spoken message?**
- Run the speech-to-text on this laptop with a downloaded model — private and free, but it is a large download and needs a decoder installed.
- Send the audio to an online service — no download and usually more accurate, but your voice recordings leave the machine.
- Skip voice entirely for now and stay with typed messages — nothing new to install, and you type instead of talk.
- *Recommendation: skip it for now, and decide once the typed version has actually been used at the bench.*

**Q2 — Should you ever be able to type an exact angle, like "elbow to 50"?**
- Never — only the named poses the arm has already been driven through and photographed.
- Yes, but only for the wrist joints, which carry the least weight.
- Yes, for any joint, with the soft limits as the only guard.
- *Recommendation: never, for now — the rule that keeps this arm safe is "leave a pose the same way you entered it", and a typed number goes straight through that rule.*

**Q3 — How long should the arm stay willing to move after you say you are ready?**
- Two minutes — you must be right there, but one move takes about twenty seconds so you will be re-typing often.
- Five minutes — comfortable for a few moves in a row, and short enough that a forgotten phone goes cold quickly.
- Fifteen minutes — convenient, but a window that long is close to just leaving it switched on.
- *Recommendation: five minutes.*

**Q4 — May the bot accept commands when you are not in the room?**
- No — it only listens while you have told it you are at the bench, and it forgets on its own.
- Yes, but only the read-only status command; movement still needs you to say you are there.
- Yes, movement too.
- *Recommendation: no — nothing holds this arm up except the daemon's torque, no pose is self-supporting, and if the daemon dies the arm falls while the bot happily reports every joint as still on from a file that stopped updating.*

**Q5 — When something is refused or goes wrong, how much detail should come back?**
- One short line explaining why, which is quick to read on a phone.
- One line plus the board's exact reply pasted underneath, which is uglier but is what a diagnosis needs later.
- *Recommendation: paste the exact reply — every debugging session in this project has turned on somebody having the literal line.*
