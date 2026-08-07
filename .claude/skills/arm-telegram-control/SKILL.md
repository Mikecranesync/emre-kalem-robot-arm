---
name: arm-telegram-control
description: Use when touching anything under Software/arm-telegram/ — the bot, the policy layer, the link client, the audit log or the voice seam — when adding a command verb, changing the armed window, the TTL or the allowlist, and when reviewing a change to the bot. Covers why remote command collides with this repo's own bench doctrine, why expiry must send nothing, and the daemon-restart trap that got through the first build.
---

# Commanding this arm from a phone — and refusing to

`Software/arm-telegram/` lets Mike type `/arm storage` then `/go pick` and watch a recorded
transition run. Every rule below exists because breaking it already cost something on this bench,
or was caught one probe short of shipping.

**Nothing in this package has run against the arm.** The bench is disconnected, there is no COM5,
and the camera is aimed at the ceiling. The passing suite says a model of a bot talks correctly to
a model of the daemon talking to a model of the firmware. It says nothing about a motor turning.
Never write a sentence that reads as a bench result.

---

## 1. The collision — say it first, and do not soften it

`arm-bench-safety` §4 says the arm is a bench rig, that driving it is a deliberate act by an
operator standing next to it, and that it **never runs unattended.** Telegram is, by construction,
command from somewhere else. No warning paragraph resolves that and neither does this package: the
armed window *bounds* it, and the bound is weaker than the rule it stands in for.

**Arming proves:** a message arrived from a chat id on the allowlist, at a known time, and
whoever sent it asserted a starting pose. **Arming cannot prove:** that a human is at the
bench; that hands are clear of the claw; that the arm is where the declaration says; or that
the phone was in a hand rather than face-down on a table.

`arm-bench-safety` §2 requires **capturing a camera frame and LOOKING at it** before commanding
any joint after a human has reached into the arm — because Mike said "go ahead" and in the very
next frame his hand was still cupped around the claw. The camera is aimed at the ceiling, so
this package **cannot satisfy that rule at all.** Arming is a weaker substitute for a rule this
repo already wrote and already paid for. Say it that way in code, in copy, and to Mike.

## 2. Expiry sends NOTHING to the arm, and one choke point enforces it

The instinctive implementation of "the window expired, make it safe" is `EST` or `DIS`. **On
this arm both DETACH every joint,** and a gravity-loaded arm falls — no pose in `arm-poses.csv`
is self-supporting and a remote user cannot catch one. So expiry is a state change **in the
bot.** `hold_arm.py` goes on holding exactly as it was. `EST`, `DIS` and `CLR` are
`policy.FORBIDDEN_VERBS`; every refusal and every expiry returns `lines == ()`.

The instinct returns one layer down and gets refused the same way: a phase that never settles
(`_abort_unsettled`) sends nothing either — no `STP`, no anything. The bot goes quiet, voids the
declared pose, closes the window, and says the arm may be part-way along the path. **Anyone
adding a "make it safe" path here is making it worse.**

`bot._send()` is the only place this package writes to `arm_cmd.txt`, and it calls
`policy.guard_line()` rather than re-checking verbs locally: a second copy of the allowlist is a
copy that drifts, and the one it drifts from enforces "no gripper, no J0". Whoever adds a verb
needs this — the `STA` the settle loop invents never passes through a `Decision`, so
`policy._guard` never sees it, and an allowlist with a hole in it for the one line the pure layer
does not vet is not an allowlist. `MOV_JOINTS = (1, 3, 4, 5)` is the *wire* half of that rule; the
data half was the only half there was, so a hand-built `Phase` could have put `MOV 6 70` on the
wire. Hardening, not a found bug.

## 3. Refuse, never queue — and the write path is where it leaked

A motion request that is unarmed, expired, stale, or arriving while another move is out is
refused outright. A queued motion command asserts consent that has already expired.

**Assert it by reading `arm_cmd.txt` itself.** `hold_arm.py:177` consumes that file whenever it
is non-empty, so a line written and not yet consumed is a command queued for whichever daemon
reads it next — possibly one that starts tomorrow. Asserting on a reply string, or even on the
daemon's log, cannot see that; `test_no_refusal_ever_leaves_a_line_in_the_command_file` opens it.

**That is also how the rule leaked.** `ArmLink.send()` writes the command file and *then* waits;
when the reply never lands it raises, and the line is still sitting there. `hold_arm.main()`
truncates the LOG on the way up (line 96) and never touches `CMD`. So a `MOV` stranded by a
daemon that died at a phase boundary runs whenever the daemon is next started — no window, no
declared pose, nobody in the room. The guard was on the *decision*, the hole was in the *write*;
`bot._send()` now retracts on any `BaseException`. Two constraints on that retraction: **never a
blind truncate** — clear the file only if it still holds exactly our line, or this package becomes
the second writer it warns everyone about; and **it is not an abort,** which no copy may call it.
It cannot un-send a line a running daemon already read; it closes the "sits there for the next
daemon" hole, the one that fires with nobody present.

## 4. The restart trap — the defect that got through the first build

A daemon restart does two things at once: `hold_arm.py:96` is `open(LOG, "w").close()`, so the
log is **truncated**, and `main()` re-runs `ENA <j> <adopt>`, so **every joint snaps to its adopt
angle.** Mid-transition that means the rest of a recorded path runs against an arm that just
moved on its own. **It looks completely clean, and that is the problem:** every reply collected
is `CL=0` and none is an `ERR`; the latch scan tails from an offset the truncation put past EOF,
so it reads `""`; and **tailing from 0 finds nothing either** — `main()` logs plain `ENA` on the
way up and only `enable_all()` logs `re-ENA`, so a **restart handshake contains no
`LATCH_MARKERS` at all.** Before the fix the rig sent `MOV 3 36`, `MOV 4 140`, `MOV 5 165`,
`MOV 1 8` into an arm whose shoulder had snapped from 40 back to adopt angle 1 — the
wrist-and-elbow phase running with the claw back down on the mat — then replied "the declared
pose is now pick" and left the window open.

**Two independent detectors, and one is not enough.** (1) `link.restarted()` — size shrank **or**
first line changed. Thin: in the test rig against the fake daemon the size went **1595 → 1596**,
so the size half did **not** fire, and the whole check then rests on two daemon starts landing in
different `HH:MM:SS`. (2) **The log must still contain the `CMD <line> -> ` marker we just watched
it log** — exact, needing neither a size nor a clock; it is there by construction and a truncation
removes it. A high-water mark on the size was tried and **rejected**: `high` at the first phase
boundary is still the pre-transition size, so it read `1596 < 1595` = False and let the transition
run. Measured in the rig, not reasoned about.

## 5. Reading the board: the offset tail, the timestamp cut, never the status file

**Record `arm_hold.log`'s byte length BEFORE writing the command, then tail from that offset.**
That yields this command's own reply, synchronously. The reply is then cut at the next daemon
**log-timestamp** boundary, **not** at the next newline: `hold_arm.log()` stamps only the *first*
line of a multi-line message, so a `CMD STA -> ` entry is one record with seven untimestamped
continuation lines. **A newline cut truncates every `STA` to its J0 row and looks exactly like it
worked** — which is why `_cut_reply` carries a comment against being simplified.

**Never report a move's outcome from `arm_status.txt`.** It is rewritten only on the daemon's 5 s
poll, so immediately after a `MOV` it still holds pre-move state — and says `MOV=0`. A "done" read
from it is a lie shaped exactly like success. `finish_motion()` judges a transition from its own
replies and sends nothing, ever. **`CL=1` is a failure, not a warning** — the firmware clamped the
request, the joint is not where you asked, so the arm is part-way along the path, the declared
pose is void and the window closes. And a board ack is not motion: that cost a whole afternoon on
D3, where the firmware provably drove a joint at 29.2 °/s for 3.5 s and nothing turned.

## 6. Freshness: LIVE / STALE / NO LINK, and there is no PID anywhere

`hold_arm.py` writes no PID — not to a file, not to the log — so the bot cannot know whether a
process is alive and **must never claim it.** No loss: a PID can be alive while the loop is wedged,
which is what stops feeding the 4000 ms watchdog. *Is the status file being rewritten* is sharper.

- A **crashed** daemon, a **wedged** one, and a **laptop that went to sleep** all read `STALE`,
  and all three refuse. The reply says which file was how old, not why.
- `NO LINK` means a file is *absent* — in practice, the wrong link directory. A freshly started
  daemon reads `NO LINK` for up to 5 s and that is correct. Do not "fix" it by falling back to the
  log's mtime: the daemon truncates that on the way up, so it reports healthy exactly when it is not.
- **`FRESHNESS_WINDOW_S = 15` is a desk choice, never observed under load.** Three 5 s poll
  periods — two missed polls tolerated, a stopped loop caught inside a quarter of a minute.
  Tighten or loosen it against a real run; never cite it as measured.

Why it exists: on 2026-08-06 the daemon died with the bench power, every joint detached, and
`arm_status.txt` was left showing `EN=1` on all of them. A stale status file reads exactly like a
healthy held arm — and a phone is where that lie does the most damage.

## 7. What may go on the wire, and what may not

- **Named poses only.** `arm-poses.csv` states the rule in capitals: *to leave a pose, reverse its
  own entry path.* The endpoints are not the safety property — two safe poses can have an unsafe
  straight line between them, and one typed number from somebody who cannot see the claw goes
  straight through that rule. So `policy` refuses **any message with a digit**, `/stop 3` included.
- **Phases are the safety property.** Only the joints inside one phase move together, and each
  settles before the next starts. Firing `Decision.lines` back to back *is* the straight line the
  ordering exists to prevent.
- **No gripper verb.** J6 acks and ramps correctly and its fingers do not articulate (the
  operator's hand-check diagnosed a gear slipping on the shaft) — and a phone can never rule
  out fingers in the claw. **Nothing addressing J0:** dead servo, in no pose, stays detached.
- **No `SPD`, ever.** `hold_arm.HOLD` holds J1 at **15** and J3/J4/J5 at **20 °/s**, so the PRD's
  proposed 25 would **speed the arm up**, not slow it down. Raising it is a bench decision.
  Consequence: commanding no speed means the bot **cannot compute a travel time**, which is why
  `SETTLE_TIMEOUT_S` is a flat 25 s ceiling — making that a computed value means commanding `SPD`,
  the forbidden move.

## 8. The token is in the URL

`https://api.telegram.org/bot<token>/sendMessage` — so **any** Bot API error carries the secret: an
`HTTPError`, a traceback, a retry message, anything quoting what it was doing. Scrub before
replying, before logging, and before writing an audit row. Each layer is asserted **on its own**.

## 9. Small rules that each cost something

- **Two clocks.** `observe()` compares a file mtime and needs **wall** time; `decide()` measures a
  TTL and needs **monotonic**. Mixing them is a real bug: monotonic into `observe()` ages the status
  file by the machine's whole uptime and refuses everything.
- **`/stop` cannot interrupt a transition.** The bot is a single long-poll loop, so a mid-move
  `/stop` is not read until the move finishes and the `STP` goes out after the thing it was meant to
  stop. Plan Phase 4 Step 5 cannot pass as written. **The stop mid-move is the rocker switch.**
- **The allowlist is chat ids, not user ids.** A group chat's id is the *group* — allowlist one and
  every member of it can command the arm. Use a private chat.
- **Banned vocabulary, stricter here than anywhere.** `actual`, `measured`, `feedback`, `position`,
  plus "the arm is at", "the arm is held", "the move completed", "stopped for safety", "emergency
  stop". These servos have no feedback of any kind, and the ban bites hardest here: the reader
  cannot glance at the bench and catch the lie.
- **Never fuzzy-match a transcription to a pose.** "pick", "quick" and "stick" are one bad microphone
  apart. `transcribe()` returns the words verbatim or `None`; `policy` refuses the rest.

## 10. Where I was wrong while building this

1. **`except Exception` does not catch `SystemExit`,** and `ArmLink.send()` raises one — a single
   unanswered command would have taken the whole bot down. Fixed in `bot.py` and deliberately
   **not** in `arm_link.py`: that copy has to keep behaving like `motion_verify.ArmLink` or the
   drift guard means nothing.
2. **The token tests passed with the scrub deleted** — every test that mentioned the token fed it
   text an earlier layer had already scrubbed. *A test that mentions the thing is not a test of the
   thing.* Each layer is asserted alone now.
3. **An `ast` guard for the forbidden verbs cannot work.** It tripped on `policy.FORBIDDEN_VERBS`
   itself and would need an exemption for the one file most worth checking. An exempted guard reads
   as coverage while proving nothing.
4. **I assumed a settle timeout would want to send `STP`** — §2's trap, one layer down.

## 11. Do not

- ❌ Call `/stop`, or anything else here, an emergency stop — in code or in copy.
- ❌ Send anything on expiry, on a refusal, or on a settle timeout.
- ❌ Add `EST`, `DIS`, `CLR`, or any verb outside `policy.ALLOWED_VERBS`; command J6 or J0; send `SPD`.
- ❌ Accept a typed angle, or fuzzy-match a spoken word to a pose name.
- ❌ Report a move's outcome from `arm_status.txt`, or treat `CL=1` as a warning.
- ❌ Claim a daemon process is alive. There is no PID anywhere.
- ❌ Import `serial`, name the port, or add a "just to check" open — `tests/test_no_serial.py` fails
  loud. Do not add a dependency either; zero is this package's strongest true statement.
- ❌ Run `cycle_poses.py` while a window is open — two writers on `arm_cmd.txt` clobber each other
  silently, and this package cannot stop it.
- ❌ Poll the link tighter than `SETTLE_POLL_S = 0.12` — `cycle_poses.settle()`'s own clean-run interval.
- ❌ Let the bot run unattended. Nothing holds this arm up but the daemon's torque.

## Cross-references

- `arm-bench-safety` — §1 (nothing in software is an emergency stop), §2 (the capture-a-frame rule
  this surface cannot satisfy), §4 (never unattended).
- `arm-serial-control` — the daemon, the file channel, the protocol replies. **Two stale lines,
  recorded here rather than edited:** §4 says `from motion_verify import ArmLink`, which drags
  OpenCV into the bot — use `Software/arm-telegram/arm_link.py`, a lifted copy held honest by a
  behavioural drift guard. §10 says `hold_arm.py` is not committed; it is, at
  `Software/arm-console/hold_arm.py`. Correcting that skill is Mike's call.
- `arm-motion-verify` — what actually proves a joint moved. Nothing here does.
- `Software/arm-telegram/README.md` — findings 8–12, and what this package cannot do.
- `Documentation/specs/2026-08-06-telegram-voice-control-prd.md` — §5a, §5c, §6, §7a, §15.
