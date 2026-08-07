# Telegram + Voice Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan phase-by-phase. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Mike types `/arm storage` then `/go pick` from his phone and the arm runs the recorded transition — inside a window he opened deliberately, refusing rather than queueing, quoting the board's own line back, and never once opening COM5.

**Architecture:** Bottom-up, because the bottom is where the lies live. The link client first (it is the only thing that touches the daemon), then a pure policy layer that decides accept-or-refuse with no I/O at all, then the Telegram adapter, then hardware, then voice. Every layer above the link client is testable without a file system clock.

**Tech Stack:** Python 3.11, **standard library only** in the bot package. `Software/arm-sim/` (the protocol simulator) + a fake daemon supply the substrate. pytest 9.0.2 for the harness.

**Spec:** `Documentation/specs/2026-08-06-telegram-voice-control-prd.md`

**Nothing in this plan has run against hardware.** The bench is disconnected, COM5 does not exist, and the camera is aimed at the ceiling. Phases 0–3 and 5 are provable tonight. Phase 4 is not, and is marked so.

---

## Global Constraints

- **The bot package imports nothing outside the standard library.** No `pyserial`, no `cv2`, no `numpy`, no `python-telegram-bot` (LGPL-3, off the Apache-2.0/MIT allow-list — PRD §12). Anything the voice phase needs gets a written licence note *before* install, following `Software/lerobot_robot_emre_arm/pyproject.toml`, which flags pyserial and numpy rather than passing them silently.
- **Forbidden verbs on this surface: `EST`, `DIS`, `CLR`,** and anything that opens the port. `EST` and the watchdog **detach**, and a gravity-loaded arm falls. A remote user cannot catch a falling arm.
- **The daemon is the sole owner of COM5.** No fallback, no degraded mode, no "opening it just to check".
- **Vocabulary** (`SERIAL-PROTOCOL.md` §0, PRD §3): `commanded`, `target`, `accepted`, `last accepted command`. **Never** `actual`, `measured`, `feedback`, `position`, and never "the arm is at X". A board ack proves the firmware accepted a command; it proves nothing about a shaft.
- **No software behaviour is an emergency stop and no copy may call one that.** The rocker and the fuse are the stop.
- **No test opens a port. No test sleeps for real time.** Clocks are injected. A suite that waits out a 5 s poll in wall-clock seconds is a suite nobody runs (`arm-sim/README.md` §9).
- **Inbound protocol lines cap at 48 characters** including the terminator; longer is discarded with `ERR E8 LINE`. One command outstanding at a time — replies correlate by the echoed verb, there are no sequence numbers.
- **Never poll the link faster than 0.12 s.** That is `cycle_poses.py`'s settle interval, and it is the only polling rate this project has evidence for: 16 clean cycles, zero watchdog latches. The daemon's `send()` blocks up to 1.2 s during which it is *not* writing the `PNG` heartbeat, against a 4000 ms watchdog. Do not invent a tighter number.
- Conventional commits. Every commit message ends with:
  ```
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ
  ```
- **Do not push, open a PR, or merge** until Phase 4 is done and only with Mike's explicit go-ahead.

---

## What would make me stop and ask Mike

Stop and report rather than guessing if any of these is true. The first four are already true and are open now.

1. **The armed-window TTL is unanswered (PRD Q17-Q3).** The plan ships 300 s because the PRD recommends five minutes, and the tests parametrise it. If Mike wants two or fifteen it is one env var — but it is his number, not a tuning default.
2. **The speed question below (register finding 5) needs a ruling.** The PRD's 25 °/s is *faster* than the speeds the daemon is currently holding the arm at. Plain English: *the phone would be speeding the arm up, not slowing it down.*
3. **`arm-serial-control` §4 tells the next person to `from motion_verify import ArmLink`, and that now drags OpenCV into any process that does it.** A skill file needs correcting, and correcting a skill is Mike's call.
4. **`arm-serial-control` §10 says `hold_arm.py` is not committed.** It is — `Software/arm-console/hold_arm.py`, tracked. Stale line in a skill.
5. Anything would require **changing `hold_arm.py`** — the PRD forbids it, and a changed daemon must be re-proven at a bench that is disconnected.
6. A phase would need a **new dependency**, for any reason, including voice.
7. The bench arrangement, the camera, or the power state is unclear at Phase 4.
8. A test causes motion that was not expected, or the simulator disagrees with a documented behaviour in a way not already in `arm-sim/README.md` §2.
9. Anyone proposes **free-form joint angles** (PRD Q17-Q2). That is Phase 6, does not exist, and deletes the §9a argument outright.

---

## File Structure

Everything the bot owns lives under `Software/arm-telegram/`, tests included, mirroring `Software/arm-sim/`. Nothing in `Software/tests/` is modified — those are proven bench tools and the bench is down, so tonight is the wrong night to edit them.

| File | Responsibility | Phase |
|---|---|---|
| `Software/arm-telegram/tests/conftest.py` | **new** — puts `arm-sim` and `arm-console` on `sys.path`; the fake-daemon fixture | 0 |
| `Software/arm-telegram/tests/fake_daemon.py` | **new** — a model of `hold_arm.py`'s file channel, replies generated by the real `ArmSim` | 0 |
| `Software/arm-telegram/tests/test_fake_daemon_conformance.py` | **new** — asserts the model matches the real daemon's log format and paths | 0 |
| `Software/arm-telegram/arm_link.py` | **new** — `ArmLink` (stdlib) + freshness verdict + restart detection | 1 |
| `Software/arm-telegram/tests/test_arm_link.py` | **new** — offset tail, verdicts, restart, drift guard vs `motion_verify.ArmLink` | 1 |
| `Software/arm-telegram/policy.py` | **new** — arming, TTL, declared pose, transitions, refusal reasons. **No I/O.** | 2 |
| `Software/arm-telegram/tests/test_policy.py` | **new** — the refusal table; drift guard vs `cycle_poses` phase ordering | 2 |
| `Software/arm-telegram/audit.py` | **new** — append-only decision log | 3 |
| `Software/arm-telegram/voice.py` | **new** — `transcribe(bytes, mime) -> str \| None`, returns `None` | 3 |
| `Software/arm-telegram/bot.py` | **new** — `urllib` long-poll, allowlist, wiring. The only module with a token. | 3 |
| `Software/arm-telegram/tests/test_bot.py` | **new** — allowlist, silence, token scrubbing, voice-through-the-same-gates | 3 |
| `Software/arm-telegram/tests/test_no_serial.py` | **new** — `ast` guard: no `serial` import, no `COM5` literal, anywhere | 3 |
| `Software/arm-telegram/README.md` | **new** — what it does not model, in the `arm-sim` shape | 3 |
| `Documentation/2026-08-06-telegram-bench-log.md` | **new** — the first real board run. Date it the day it runs. | 4 |
| `Software/arm-telegram/tests/test_voice.py` | **new** — fail-closed transcription | 5 |

---

## Where the PRD and the source disagree

Seven places. The PRD is a good document; these are mechanism details that only surface when you open the files. **Where they disagree, the source wins** — same rule as `arm-sim/README.md` §1. Findings 1, 2, 3 and 5 change behaviour.

### 1. `from motion_verify import ArmLink` drags OpenCV into the bot — and so does the pose table

PRD §4 says import `ArmLink` rather than reimplement it, and §9b says reuse `cycle_poses.py` for the phase ordering. Both are right in spirit and both fail in mechanism: `motion_verify.py` and `cycle_poses.py` each `import cv2` and `import numpy` at module top level. A Telegram bot that writes one line to a text file would pull in OpenCV to do it, against a PRD that claims **zero new dependencies** as its strongest true statement.

`arm-serial-control` §4 carries the same instruction as a literal code snippet, so this corrects a **skill file**, not only the PRD.

**Resolution:** lift `ArmLink` verbatim into `Software/arm-telegram/arm_link.py` (stdlib only) and lift `TO_STORAGE` / `TO_PICK` / `PICK` / `STORAGE` / `TRANSIT_J1` into `policy.py`. Do **not** edit the two bench tools — they are proven and cannot be re-proven with the bench down. Instead, **behavioural drift guards**: the tests import both copies (cv2 is installed, so tests may; the bot may not), point them at the same seeded link dir, and assert identical returns and identical constants. Textual comparison would false-fail on whitespace.

### 2. There is no PID, and that is the better signal anyway

PRD §7 gates on "daemon PID alive", and §3's allowed-copy example reads *"status file is 4 s old; daemon PID 8123 alive"*. **`hold_arm.py` writes no PID anywhere** — not to a file, not to the log. It is unimplementable without changing the daemon, which the PRD forbids.

Do not apologise for this. Without a PID, "alive" and "working" collapse into one observable — **is `arm_status.txt` being rewritten** — and that is *stronger*, not weaker. A PID can be alive while the loop is wedged, and a wedged loop is exactly what stops feeding the 4000 ms watchdog. Sharper still: line 161 writes the status file only `if sta:`, so a live process whose `STA` returns empty leaves a stale file with a perfectly healthy PID.

**Resolution:** the verdict is `LIVE` / `STALE` / `NO LINK`, where `NO LINK` means the log or status file is absent. `STALE` and `NO LINK` both refuse motion, so the behaviour is exactly what the PRD specified; only the copy changes. **PRD §3's "daemon PID 8123 alive" example must be struck** — the bot cannot know that and must not claim it. The bot also cannot distinguish crashed from wedged from laptop-asleep. All three read `STALE`, all three refuse, and the reply says which file was how old rather than diagnosing why.

### 3. The daemon truncates its own log at startup — a shrinking log means the arm moved

`hold_arm.py:96` is `open(LOG, "w").close()`. A restarted daemon resets `arm_hold.log` to zero bytes. A bot holding a byte offset across that restart calls `tail(offset)`, seeks past EOF, gets an empty string forever, and **every command hangs to timeout** while the operator watches nothing happen.

It is worse than a hang. A restart re-ran `ENA <j> <adopt>`, so **every joint snapped to its adopt angle** — the same spontaneous-looking motion PRD §5c disarms for.

**Resolution:** the link client records the last offset it saw. `current_size < last_offset` means the daemon restarted → **disarm, refuse, report**. Re-arming needs a fresh `/arm <pose>`, i.e. a human looking at the arm. Testable offline in four lines.

### 4. `ArmLink.send()` returns to end-of-tail, not end-of-reply

`send()` returns `tail.split(marker, 1)[1].strip()` — everything after the marker to the end of the file. The daemon's 5 s health poll can append a line into that window, so a reply quoted verbatim into Telegram can carry an unrelated daemon line.

**Resolution:** cut the reply at the next line matching `^\d\d:\d\d:\d\d ` (the daemon's `log()` timestamp prefix). **Not at the next newline** — `log()` prefixes only the *first* line of a multi-line message, so a `CMD STA -> ` block has untimestamped continuation lines and a naive newline cut would truncate every `STA`. Write that reasoning into the code; somebody will try to simplify it.

### 5. `SPD 25` is faster than the speeds the arm is currently held at

PRD §9c proposes commanding 25 °/s as a cap below the recorded intent of 30. But `hold_arm.py`'s `HOLD` table runs **J1=15, J3=20, J4=20, J5=20, J6=12 °/s**. 25 is above every one of them. The PRD's own principle — *a message from a phone is not a deliberate bench act* — argues against the PRD's own number.

Two further consequences: `enable_all()` re-sends `SPD` on any latch recovery, silently reverting whatever the bot set; and leaving speeds changed violates `arm-serial-control` §9, *restore any `SPD` you changed*.

**Resolution: the bot sends no `SPD` at all.** It inherits the daemon's speeds. Zero state mutation, no revert interaction, no cleanup debt. **Honest trade-off:** at 15–20 °/s a round trip is nearer 30 s than 19 s, which lengthens exposure inside the armed window and makes PRD §9's "a transition can outlive the TTL" *more* likely, not less. That is why `/stop` works unarmed. **This is Mike's call** — see stop-and-ask item 2.

### 6. `arm_cmd.txt` is truncate-on-write, and the bot cannot enforce sole-writer

Both `ArmLink.send()` and the daemon open it `"w"`. Two writers clobber each other silently. If Mike runs `cycle_poses.py` while the bot is armed, commands are lost with no error anywhere.

**Resolution:** the bot cannot prevent this and must not pretend to. It goes in `README.md` and in the reply copy for `/arm`: *one driver at a time.* Named, not fixed.

### 7. `arm-serial-control` §10 is stale

It says `hold_arm.py` "is **not committed**… the consumer is in git and the producer is not." It is committed, at `Software/arm-console/hold_arm.py`. One line to correct in a skill file.

---

## Phase 0 — the substrate: a fake daemon, and proof it resembles the real one

**Files:** create `tests/conftest.py`, `tests/fake_daemon.py`, `tests/test_fake_daemon_conformance.py`

**Why this is first.** Everything downstream is measured against this. If it is wrong, four phases of green tests prove a fiction. It is also the smallest slice that can fail.

**Why not run the real `hold_arm.py` against the simulator.** It would need a monkeypatched `serial` module *and* control of wall-clock time — its loop sleeps 0.15 s and polls every 5 s, so a `STALE` test would take fifteen real seconds. `arm-sim/README.md` §7 describes exactly that monkeypatch recipe and says it was **deliberately not committed**, "not something to leave lying around in a repo that drives motors". Honour that.

**So: `fake_daemon.py` is a model of the daemon, not the daemon.** Say it in its docstring. It reproduces the *file channel* — `arm_cmd.txt` consumed and cleared, `arm_hold.log` appended with `HH:MM:SS CMD <line> -> <reply>`, `arm_status.txt` rewritten on a poll — and it generates every reply by handing the line to a real `ArmSim`, so the wire format is the firmware's and not mine. Time is injected; the "poll" advances only when a test says so.

- [ ] **Step 1: `conftest.py`** puts `Software/arm-sim` and `Software/arm-console` on `sys.path`. `arm_sim` is imported as a top-level module (`from arm_sim import ArmSim`), which is why its own suite runs `cd Software/arm-sim` first. Without this every acceptance command below is aspirational.
- [ ] **Step 2: `fake_daemon.py`** — `pump()` consumes the command file and appends a log line; `poll()` writes the status file; `restart()` truncates the log the way line 96 does; `advance(ms)` moves the injected clock. No threads.
- [ ] **Step 3: conformance.** `import hold_arm` (pyserial is installed and `main()` is guarded), then assert the model against the **real source**: the three filenames from `hold_arm.CMD/LOG/STATUS`, the `HH:MM:SS ` prefix and the `CMD {ln} -> ` marker from `inspect.getsource(hold_arm.log)` and the command-channel block, the 5 s poll constant, and that `main()` truncates `LOG`. A drift guard, in the shape of the CSV field-count validator that caught `pass=yes`.
- [ ] **Step 4: assert `ArmSim` is the reply source** — push `MOV 3 64` through the fake daemon and check the log line carries the firmware's own `OK MOV J3 REQ=64 SET=64 CL=0`, not a canned string.

**Acceptance — simulator, no hardware:**
```bash
cd /c/RobotArm/Software/arm-telegram && python -m pytest -q tests/test_fake_daemon_conformance.py
```

---

## Phase 1 — the link client

**Files:** create `arm_link.py`, `tests/test_arm_link.py`

**Interfaces:** `ArmLink(link_dir)` → `.offset()`, `.tail(off)`, `.send(line, timeout, on_poll=None)`, `.sta_row(jid)`, `.field(row, key)` (all lifted verbatim), plus `.verdict(now) -> "LIVE" | "STALE" | "NO LINK"`, `.restarted() -> bool`, and `.observe() -> LinkState(verdict, restarted, log_window)` — the one object Phase 2 consumes.

- [ ] **Step 1: lift `ArmLink`** from `motion_verify.py`, unchanged in behaviour, stdlib only (register finding 1). Docstring says where it came from and that the two must not drift.
- [ ] **Step 1b: add the `on_poll` hook, or Phase 1's own acceptance command hangs.** `send()` writes the command file then loops `tail()` → `sleep(0.08)` → `tail()` until the marker appears. The Phase 0 fake daemon is synchronous by design, so nothing can append the reply while the test is blocked inside `send()` — it would spin for the full 6 s timeout, `SystemExit`, and break the no-real-sleeps constraint. `on_poll` is called once per loop iteration; the tests wire it to `fake_daemon.pump()` so the reply lands before the first sleep. **It defaults to `None`**, so behaviour with no hook is identical to `motion_verify.ArmLink` and the drift guard stays honest. Threading the fake daemon instead would contradict Phase 0 Step 2; pre-seeding the log instead would skip the write→consume path this phase exists to test.
- [ ] **Step 2: reply cutting** — trim at the next `^\d\d:\d\d:\d\d ` boundary, with finding 4's reasoning in a comment.
- [ ] **Step 3: the freshness verdict** — `LIVE` if `arm_status.txt` mtime is inside the window; `STALE` if not; `NO LINK` if the log or status file is absent. Window **15 s**, and label it what it is: **a desk choice, never observed under load.** The daemon rewrites the status file on a 5 s poll, so it is stale by design and anything tighter than 5 s false-alarms; 15 s tolerates two missed polls. Tighten or loosen it against a real run.
- [ ] **Step 4: restart detection** — `current_size < last_offset` → restarted (finding 3).
- [ ] **Step 5: the drift guard** — import `motion_verify.ArmLink` (tests may; the bot may not), point both at one seeded temp dir, assert identical returns from `offset`/`tail`/`send`/`sta_row`/`field`. **Call both with `on_poll` unset** against a pre-seeded log, or the guard is comparing two different code paths and proves nothing.

**What is proven here and what is not.** Proven: the offset primitive returns *this* command's reply after prior traffic is seeded ahead of it; verdicts flip at the boundary on an injected clock; a shrunken log is caught. Not proven: anything about a real daemon under real load, or about the arm.

**Acceptance — simulator, no hardware:**
```bash
cd /c/RobotArm/Software/arm-telegram && python -m pytest -q tests/test_arm_link.py
```

---

## Phase 2 — the policy layer, with no I/O at all

**Files:** create `policy.py`, `tests/test_policy.py`

**Interfaces:** `decide(state, message, now, link: LinkState) -> Decision(accept: bool, reason: str, lines: list[str], new_state)`. Pure. No files, no sockets, no `time.time()` — the clock is an argument. That is what makes the TTL testable by monkeypatching a number instead of sleeping five minutes. **`LinkState` and not a bare verdict string**, because Step 6 disarms on things a verdict cannot carry: a daemon restart, and a `LATCHED` / `re-ENA` line in the log window. Those are link-layer observations and must be passed in, not re-derived here.

- [ ] **Step 1: the vocabulary** — `/arm <pose>`, `/disarm`, `/status`, `/go storage`, `/go pick`, `/stop`. Everything else refuses with the accepted list, **including any message containing a number** (PRD §9a: two safe poses can have an unsafe straight line between them, and one typed number goes straight through the "leave a pose the way you entered it" rule).
- [ ] **Step 2: the arming contract** — opened only by explicit `/arm <pose>` naming the pose the operator asserts the arm is in *now*; TTL 300 s from `ARM_TELEGRAM_TTL_S`; **extended by nothing** — not activity, not a completed move.
- [ ] **Step 3: expiry sends nothing.** The single most important line in this layer. The instinctive "window expired, make it safe" is `EST` or `DIS`, and on this arm those **detach and drop it**. Expiry is a state change in the bot. `hold_arm.py` goes on holding exactly as before. Assert that the command file is untouched, not merely that a refusal was returned.
- [ ] **Step 4: the transition table**, lifted from `cycle_poses.py` (finding 1) with a constants drift guard. Directional and phase-ordered: `to_storage` lifts J1→40 first to get the claw off the mat, neutralises J3/J4/J5 while high, then folds J1→88. A transition that does not start from the declared pose refuses and says what was declared.
- [ ] **Step 4b: assert every waypoint sits inside `hold_arm.HOLD`'s limits.** `import hold_arm` is already in the Phase 0 conformance test, so the constants are free. Every angle in both transitions is inside 0–91 / 0–66 / 0–180 / 31–178 **today** — a later pose edit that pushes past one produces `CL=1` at the bench, which is exactly the failure Step 7 teaches the bot to report. Two lines here beats finding it with a motor.
- [ ] **Step 5: refuse, never queue.** A second request mid-transition is refused. Assert the queue is **empty**, not that ordering held — a queued motion asserts consent that has already expired.
- [ ] **Step 6: disarming events** — TTL expiry, `/disarm`, `STALE`, `NO LINK`, daemon restart (finding 3), and any `LATCHED` / `re-ENA` in the log window. The latter is `enable_all()` snapping joints to adopt angles, which reads exactly like the arm moving on its own; it voids the declared pose and the log lines are reported verbatim.
- [ ] **Step 7: outcomes come from the reply, never `arm_status.txt`.** The file is rewritten on the 5 s poll, so right after a `MOV` it still holds pre-move state — and says `MOV=0`. A "done" read from it is a lie shaped exactly like success. **`CL=1` is a failed command, not a warning.** The status file is only for "is it still held" and `JTO` / `ES` / `WD` surveillance.
- [ ] **Step 8: `/stop`** — the sole unarmed exception, because a transition takes 5–15 s and can outlive a TTL that expired mid-move; a `/stop` refused for being unarmed is refused at exactly the moment it is wanted. Still needs an allowlisted chat id. Sends exactly `STP`, never `EST` or `DIS`. Assert it: a test greps the emitted lines for the forbidden verbs across every path.
- [ ] **Step 9: no gripper verb, no J0.** J6 acks and does not articulate (the gear slips on the motor shaft), and a phone can never rule out fingers in the claw. J0's servo is dead.

**Acceptance — simulator, no hardware:**
```bash
cd /c/RobotArm/Software/arm-telegram && python -m pytest -q tests/test_policy.py
```

---

## Phase 3 — the Telegram adapter, the audit log, and the voice seam

**Files:** create `bot.py`, `audit.py`, `voice.py`, `tests/test_bot.py`, `tests/test_no_serial.py`, `README.md`

- [ ] **Step 1: long-poll `getUpdates` with `urllib`.** Zero new dependencies (PRD §12). The transport is injectable so tests never touch the network.
- [ ] **Step 2: the token.** `ARM_TELEGRAM_BOT_TOKEN` and `ARM_TELEGRAM_ALLOWED_CHAT_IDS` from the environment; refuse to start if either is missing. **Never committed, never logged, never echoed in an error** — a Bot API HTTP failure carries the URL and the token is *in* the URL. Scrub before logging or replying, and test it by raising a `urllib` error whose message contains a fake token and asserting the token appears in neither the reply nor the audit row.
- [ ] **Step 3: the allowlist as the outermost gate** — before arming state, before parsing, before anything. Everything else gets **no reply at all**: no error, no hint a bot exists. A token can leak; a bot that answers strangers tells them what it controls. Silent to the sender, **loud in the audit log**.
- [ ] **Step 4: `audit.py`** — append-only, one row per decision, accepts and refusals alike, because a refusal history is what reveals probing. Timestamp, chat id, username, raw text, decision and reason, the exact protocol line sent or none, the reply verbatim, the freshness verdict, armed state and remaining TTL. Never the token.
- [ ] **Step 5: `voice.py`** — `transcribe(audio_bytes, mime) -> str | None`, returning `None`, replying *"voice is not enabled — send a text command."* One seam, no backend named.
- [ ] **Step 6: the `ast` import guard.** Parse the **package modules only — `Software/arm-telegram/*.py`, excluding `tests/`** — and assert no `import serial` / `from serial …` node and no `COM5` string literal. **`ast`, not `sys.modules`** — a `sys.modules` check is order-dependent inside a pytest session and can false-pass. The `tests/` exclusion is load-bearing and not laziness: the Phase 0 conformance test deliberately imports `hold_arm`, which imports `serial` and contains the literal `"COM5"`. Widen this guard to `tests/` later and it trips on the test that exists to prove the daemon's format.
- [ ] **Step 7: banned-word assertions** on every reply string the bot can emit (PRD §3). The reader cannot glance at the bench and catch the lie, which is why this ban is stricter here than anywhere else.
- [ ] **Step 8: `README.md` in the `arm-sim` shape** — a "what this does NOT model" section that says plainly: it does not know where the arm is, it cannot prove a human is present, it cannot prevent a second writer to `arm_cmd.txt` (finding 6), and a green suite is evidence about the bot's logic and nothing else.

**Acceptance — simulator, no hardware. The whole suite:**
```bash
cd /c/RobotArm/Software/arm-telegram && python -m pytest -q
```

---

## Phase 4 — FIRST REAL BOARD RUN. Mike at the bench, hand near the rocker.

**Files:** create `Documentation/2026-08-06-telegram-bench-log.md` (date it the day it runs)

**Nothing in Phases 0–3 is evidence about the arm.** The offline suite proves the logic and proves nothing else. This phase is the first time a message from a phone reaches a motor.

**What Mike has to do, before a single command is sent:**

1. **Plug the board in.** COM5 back, servo power arranged as usual, workspace clear.
2. **Put the camera back** on the arm, off the ceiling.
3. **Look at the arm.** Eyes, not a frame, not a status file. Confirm where it actually is and that nothing is in the claw or the path.
4. **Start the daemon and read its log** — confirm the joints adopted and it is holding before the bot is started at all.
5. **Hand near the rocker for every command.** The rocker and the fuse are the only stop; `/stop` is not one and the reply copy says so.
6. **Support the forearm by hand before switching off.** No pose here is self-supporting.

**The camera does not give the bot a hands-clear check, and re-aiming it does not change that.** `arm-bench-safety` §2 requires capturing a frame and *looking at it* before commanding any joint after a human has reached into the arm — because that already happened here: Mike said "go ahead" and in the very next frame his hand was still cupped around the claw. **There is no camera path anywhere in Phases 0–3.** At the bench, *Mike* performs that check with his eyes. The bot never does, in this plan or any phase of it. Arming is a weaker substitute for a rule this repo already wrote and already learned the hard way — not an equivalent.

- [ ] **Step 1: exercise the contract before exercising a motor.** `/go pick` while unarmed → refused. `/status` → a verdict and a fresh `STA`. A non-allowlisted id → silence, with an audit row. Kill the daemon → `NO LINK`, refused, and confirm with `STA` on a separate console that no port was opened.
- [ ] **Step 2: TTL expiry, live.** `/arm storage`, wait it out, `/go pick` → refused. **Confirm from the daemon's own log that nothing was sent** and that the arm did not move. Expiry must be silent on the wire.
- [ ] **Step 3: restart detection, live.** Restart the daemon mid-window (this truncates the log and re-`ENA`s every joint — expect the arm to snap to adopt angles). The bot must disarm and report. **Mike watches the arm during this one.**
- [ ] **Step 4: one transition, watched.** `/arm storage` then `/go pick`. Mike watches the whole run with a hand near the rocker. Then the reverse.
- [ ] **Step 5: `/stop` mid-transition.** Confirm it is accepted, that exactly `STP` went out, that the reply is quoted, and that the arm **held with joints driven** rather than dropping.

**Acceptance evidence — an ack is not a pass.** A board ack proves the firmware accepted a command; it proves nothing about a shaft. That cost a whole afternoon on D3, where the firmware provably drove a joint at 29.2 °/s for 3.5 s and nothing turned. So the bench log records **both**: every board line verbatim, **and Mike's own observation of what the arm did** — his eyes are the strongest evidence available here and the approved exception to every vocabulary rule. The pixel/geometry harness is **not** in use: the camera is uncalibrated after re-aiming, and an uncalibrated ROI has already produced a false `MOVED` twice.

**Write the log as what to record, never as what will happen.** Every step above is a question, not a prediction.

**Acceptance — HARDWARE. Mike present. Two shells:**
```bash
python "C:\RobotArm\Software\arm-console\hold_arm.py"
cd /c/RobotArm/Software/arm-telegram && ARM_TELEGRAM_LINK_DIR=/c/RobotArm/Software/arm-console python bot.py
```

---

## Phase 5 — voice, behind the identical gates

**Files:** create `tests/test_voice.py`; extend `voice.py`

**Blocked on PRD Q17-Q1** — no backend is chosen and this plan does not choose one by quietly picking a default. Telegram delivers voice notes as OGG/Opus, which needs a decoder *and* a speech-to-text model: a dependency chain and, for most backends, a download. Each dependency gets a written licence note, with its licence named, **before** it is installed.

- [ ] **Step 1: fail closed.** An unrecognised transcription is refused and **never** fuzzy-matched to the nearest pose name. "pick", "quick" and "stick" are one bad microphone apart, and a near-miss that moves the arm is exactly the failure a phone-shaped interface invites. Cheap to state now; expensive to retrofit once somebody has added a similarity score.
- [ ] **Step 2: no privileges of its own.** Run the entire Phase 2 refusal table twice — once typed, once through a stub transcriber — and assert identical decisions. Voice is a text source, nothing more.
- [ ] **Step 3: audit rows record `voice` plus duration,** never the audio and never a token.

**Acceptance — simulator/offline, no hardware:**
```bash
cd /c/RobotArm/Software/arm-telegram && python -m pytest -q tests/test_voice.py
```

---

## What is NOT in this plan

- **No customer-shipped path, and no unattended running.** The arm is a bench rig; driving it is a deliberate bench act by an operator standing next to it, and that is the entire exception (`arm-bench-safety` §4). PRD Q17-Q4 asks whether the bot may run when Mike is out of the room and the recommendation is **no**: nothing holds this arm up except the daemon's torque, no pose is self-supporting, and if the daemon dies the arm falls while a status file that stopped updating still reports every joint on.
- **No control writes beyond this bench rig.** No fieldbus, no Modbus, no second port owner, no writes of any kind outside the daemon's file channel.
- **No emergency stop.** There is none and there cannot be one. `/stop` stops issuing motion and sends `STP`, which holds with joints driven. It does not remove power, cannot know a shaft angle, cannot make the arm safe to touch, and cannot undo a completed move.
- **No firmware change, no new verb, no new error code, and no change to `hold_arm.py`.**
- **No free-form joint angles** (PRD Q17-Q2). That is a Phase 6 that does not exist and needs its own design, because it deletes the §9a argument and would need something else put in its place.
- **No camera work, no inverse kinematics, no Cartesian control, no new poses, no gripper verb, no J0.**
- **No speech-to-text backend chosen.**

---

## Self-Review

**PRD coverage.** §3 vocabulary → Global Constraints + Phase 3 Step 7. §4 architecture → Phase 1, Phase 3 Step 6. §5 arming → Phase 2 Steps 2–3; §5a expiry-sends-nothing → Phase 2 Step 3; §5b what arming cannot prove → Phase 4; §5c latch → Phase 2 Step 6. §6 refuse-never-queue → Phase 2 Step 5. §7 freshness → Phase 1 Step 3 (corrected, finding 2); §7a never-from-status → Phase 2 Step 7. §8 allowlist → Phase 3 Step 3. §9 vocabulary → Phase 2 Step 1; §9b transitions → Phase 2 Step 4; §9c speed → **finding 5, changed**; §9d no gripper → Phase 2 Step 9. §10 `/stop` → Phase 2 Step 8. §11 voice → Phase 3 Step 5, Phase 5. §12 licensing and token → Global Constraints, Phase 3 Step 2. §13 audit → Phase 3 Step 4. §14 acceptance → every phase. §15 invariants → Global Constraints. §17 open questions → stop-and-ask 1, 2, 9 and Phase 5.

**Disjointness.** Every phase creates files no other phase creates. Phase 5 is the only phase that extends a file from an earlier one (`voice.py`), and that is the seam it was built as.

**Type consistency.** `ArmLink.verdict(now)` → one of three literal strings, in Phases 1, 2, 3. `ArmLink.observe()` → `LinkState(verdict, restarted, log_window)`, produced in Phase 1 and consumed as `decide()`'s `link` argument in Phase 2 — the *only* channel by which a link-layer fact reaches the pure layer. `decide()` → `Decision(accept, reason, lines, new_state)` in Phases 2, 3, 5; `lines` is the list the bot writes to `arm_cmd.txt` and is **empty on every refusal and on every expiry**, which is the assertion Phase 2 Step 3 makes. `transcribe(bytes, mime)` → `str | None` in Phases 3 and 5, `None` throughout Phase 3. `send(line, timeout, on_poll=None)` → `str` in Phases 1–4; `on_poll` is test-only and unset everywhere in the shipped bot.

**Ordering constraint.** Phase 1 Step 1b's `on_poll` hook is what lets Phase 0's synchronous fake daemon serve Phases 1–3. Build Phase 0 first and Phase 1 without the hook, and every acceptance command from Phase 1 onward hangs for 6 s and then raises `SystemExit` — that is the failure to expect if the phases are done out of order.

**Unproven claims register.** The 15 s freshness window is a desk choice. The fake daemon is a model of `hold_arm.py`, not `hold_arm.py`. The round-trip figures (30.2 s at 12 °/s, 9.8–10.5 s at 90 °/s) are quoted from `arm-poses.csv`, measured before tonight, at speeds the bot will not command. **No bench result is claimed anywhere in this document, and nothing in it has run against hardware.**
