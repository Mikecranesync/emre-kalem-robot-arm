# Telegram + voice session findings — 2026-08-07

Companion to `2026-08-06-EVENING-SESSION-FINDINGS.md`, which ends with the bench powered down
at 21:55 and the arm folded at `storage`. This session ran from there to 01:50 and never
turned it back on.

**Headline: the phone-to-arm path exists as design, simulator and code with 347 passing
tests, an adversarial pass found one real safety defect the green suite did not, and the
bench was disconnected for every minute of it.**

**The caveat governing the whole document: nothing here has run against hardware.** No
COM5, no board, camera aimed at the ceiling. 347 tests say a model of a bot talks correctly
to a model of the daemon talking to a model of the firmware. They say nothing about a motor
turning. Both suites, run for this document:

```
cd Software/arm-sim      && python -m pytest -q   ->   88 passed in 1.34s
cd Software/arm-telegram && python -m pytest -q   ->  259 passed in 51.24s
```

Counts re-run and re-confirmed by the 2026-08-07 audit pass; the wall times are this
machine's and mean nothing. Both READMEs print older counts (`86 passed` in three places,
`217 passed`) — stale as committed, not drifted since, and `git diff HEAD` on both is empty.

---

## 1. What now exists

| Path | What it is for |
|---|---|
| `Documentation/specs/2026-08-06-telegram-voice-control-prd.md` | the requirements, §15's safety invariants, §17's **five** open questions left for Mike rather than guessed past |
| `Documentation/plans/2026-08-06-telegram-voice-plan.md` | six phases, a **nine-item** stop-and-ask list, seven places the PRD and the source disagree |
| `Software/arm-sim/arm_sim.py`, `fake_serial.py` | the simulated firmware, and a duck-typed stand-in for `serial.Serial`. Stdlib, injected clock, no sleeps |
| `Software/arm-sim/tests/`, `README.md` | 88 tests — protocol §12/§13 as literal byte vectors — plus the three doc disagreements and what it does **not** model |
| `Software/arm-telegram/arm_link.py` | the command channel: byte-offset tail, reply cutting, freshness verdict, restart detection |
| `Software/arm-telegram/policy.py` | arming, TTL, the transition table, every refusal. Pure — no files, no sockets, no clock |
| `Software/arm-telegram/bot.py` | the Telegram adapter: `urllib` long-poll, allowlist, settle loop. The only module holding a secret |
| `Software/arm-telegram/audit.py`, `voice.py` | append-only decision log; the `transcribe()` seam, which returns `None` |
| `Software/arm-telegram/tests/`, `README.md` | 259 tests, conformance and adversarial included — `fake_daemon.py` is a **model** of `hold_arm.py`'s file channel whose replies come from a real `ArmSim` — plus what the package does not model |

Nothing under `Software/tests/`, `Software/arm-console/` or the firmware was modified —
proven bench tools, and the bench is down.

---

## 2. The safety argument

Telegram is, by construction, command from somewhere else. `arm-bench-safety` §4 says the
opposite in the plainest sentence this project owns: *"The arm is a bench rig, and driving it is
a deliberate bench act by an operator standing next to it."* Two sentences later it adds *"it
never runs unattended"* — phrased there about customer-shipped paths, but the words are
unqualified and this surface is exactly what they refuse. This machine also has no
software stop worth the name — `STP` holds with the joints still driven, while `EST` and the
watchdog **detach**, and a detached gravity-loaded arm falls.

The resolution is **structural, not a caveat**: motion only inside an armed window opened by
an explicit `/arm <pose>` from an allowlisted chat id, with a TTL nothing implicitly extends.
Refuse, never queue — a queued motion command asserts consent that has already expired. Named
poses only, along `arm-poses.csv`'s own recorded entry paths, because two safe poses can have
an unsafe straight line between them. **Expiry sends nothing to the arm**: the instinctive
"window expired, make it safe" is `EST` or `DIS`, which detach every joint and drop it, so
expiry is a state change in the bot while the daemon goes on holding. `EST`, `DIS` and `CLR`
are denylisted, and two guards raise before `arm_cmd.txt` is touched.

**What arming cannot prove.** It proves a message arrived from an allowlisted chat id at a
known time and that whoever sent it asserted a starting pose. It does not prove a human is at
the bench, that hands are clear of the claw, that the arm is where the declared pose says, or
that the phone was in a hand rather than face-down on a table. `arm-bench-safety` §2 requires
capturing a camera frame and **looking at it** before commanding any joint after a human has
reached into the arm — because that already happened here, with Mike's hand still cupped
around the claw in the frame after he said "go ahead". **There is no camera path anywhere in
this package.** Arming is a *deliberate act with a short expiry* that bounds how long a stale
intention stays valid — a weaker substitute for a rule this repo learned the hard way.

---

## 3. The defect that got through the first build

**A daemon restart at a phase boundary did not stop a transition.** `decide()` already
refused a `/go` when the link had restarted, so a restart *between messages* was always
caught. A restart *during* one was not.

Measured against the fake daemon before the fix — a model, not a bench observation:
`/arm storage`, `/go pick`, restart the daemon after the first phase settled, and the bot
sent `MOV 3 36`, `MOV 4 140`, `MOV 5 165` and `MOV 1 8` into an arm whose shoulder had just
snapped from 40 back to its adopt angle of 1, then replied that the declared pose was now
`pick` and left the window open. The `aim` phase exists to swing the wrist and elbow while
the claw is clear of the mat; it ran with the shoulder folded back down — the exact straight
line between two safe poses that the phase ordering exists to prevent.

**Three conditions had to hold at once for nothing to catch it.**

1. Every reply collected was `CL=0`, none was an `ERR`, so no outcome check fired.
2. The latch scan tails from the offset the transition started at, and a restart
   **truncates** the log — so `tail(start)` seeks past EOF and returns an empty string.
   Plan finding 3 exactly, one layer over.
3. **The least obvious, verified directly rather than assumed: a restart handshake contains
   no latch markers at all.** `hold_arm.main()` logs plain `ENA` on the way up; only
   `enable_all()` logs `re-ENA`, so even tailing from 0 finds nothing to scan for. The
   marker the disarm rule looks for is written by the *recovery* path, never by *startup*.

**The fix is two detectors, and they are not equally strong.**

- `link.restarted()` at each phase boundary — a pure query against the baseline `observe()`
  set at the top of the update, so it is safe to call once per phase and does not consume the
  edge. But it is size-shrink **OR** first-line-changed, and **the size half does not fire
  here** — measured at both baselines, not reasoned about: `1517 → 1521` at the link layer's
  own baseline (`arm_link.py`), `1595 → 1596` at the transition boundary (`bot.py`). That
  detector therefore rests entirely on two daemon starts landing in different `HH:MM:SS`,
  which a startup **floor** of about 8 s makes near-certain, not guaranteed — 2.2 s for the
  port plus 1.2 s after each of five `ENA`s, sleeps alone, before one reply wait is counted.
  A floor, not a measured handshake duration; the real one is longer. It fails **open**.
- **The `vanished` check, which is exact and needs neither a size nor a clock.** The log
  must still contain the `CMD <line> -> ` marker the bot just watched it log — that marker
  is there by construction, because it is how the reply was found. A truncation removes it,
  and its absence is proof the log was replaced.
- A high-water mark on the size was tried **first and rejected with a measurement**:
  `1596 < 1595` is `False`, so it let the transition run.

Both halves fail safe in the over-reporting direction, and over-reporting costs a re-arm — a
human looking at the arm. Pinned by
`test_a_daemon_restart_at_a_phase_boundary_stops_the_transition`, a canary proving the check
does not fire on a healthy transition, and a test naming **which** half carries it, so a
future change leaning on the weaker one is visible rather than assumed.

**The general lesson is the finding, not the bug.** 259 green tests did not catch a path
that drove a simulated arm along the exact trajectory the whole design exists to prevent.
An adversarial pass — a second session going looking, after the build was finished and
reviewed — did, as it did for findings 11 and 12 in `arm-telegram/README.md` and 4 and 5 in
`arm-sim/README.md`. **Reading code you have already read twice does not find these.
Breaking it and re-running does.**

---

## 4. Three doc/firmware disagreements — findings about the EXISTING system

The most reusable thing here: these are about the firmware and `SERIAL-PROTOCOL.md`, not
tonight's new code, and they will bite the next host anyone writes. In all three the firmware
is right and **the doc is stale**. None is fixed; all are in `arm-sim/README.md` §2 with tests.

**`STA` carries a `JTO=` field the worked examples do not show.** The firmware emits it at the
end of every joint line. §4 calls itself "byte for byte" and §12 calls itself "exact bytes";
**neither shows it, and both are stale.** The doc contradicts itself — §3's `JOG` text
correctly promises `JTO` on every `STA` joint line. A host written to §4's field list meets an
unknown field on its first status poll, and the doc's own two literal test vectors would fail
against a real board.

**A watchdog trip emits a second, undocumented `EVT` line.** A real trip produces
`EVT WDOG MS=<n>` *and* `EVT ESTOP SRC=WDG`, because `estopAll()` prints its own `EVT` with
whatever source it was handed. **`SRC=WDG` appears nowhere in the doc** — §6's async-line
list gives only `SRC=CMD` and `SRC=RT`, and §8 describes the trip as one line plus an e-stop.
A host that switches on `SRC=` falls through its own default branch on the one event the
operator most needs explained.

**The e-stop latch gates three verbs, not everything.** `estopLatched` is read by exactly
`doEna`, `doMov` and `doJog`; twelve other verbs answer `OK` on a latched board. Two doc
places read globally and are stale: **§3's `STP` vs `EST` table row** ("everything returns
`E7` until `CLR`") and **§7's `E7` row**. The behaviour is almost certainly correct and
useful — it lets a host push limits, speeds and the mirror into a board that latched before
the host connected. The sharp edge: **`STP <j>` on a latched board answers `E6`, not `E7`**,
because the latch detached the joint, so a host mapping `E7` to "press CLR" gives the
operator no hint at all on that one.

---

## 5. What is not proven

**No hardware, at all** — every number here that looks like a measurement is from an earlier
bench session, or is a measurement of a simulator. Beyond that:

- **`hold_arm.py` writes no PID**, so a crashed daemon, a wedged one and a laptop that went to
  sleep all read `STALE` and all refuse. The reply says which file was how old; it does not
  diagnose why, and must not claim to.
- **The 15 s freshness window is a desk choice, never observed under load** — the status
  file is rewritten on a 5 s poll, so anything tighter false-alarms and 15 s tolerates two
  missed polls. The 25 s settle timeout is a desk choice too, because the bot commands no
  `SPD` and cannot compute a travel time. The settle *predicate* is not: that is
  `cycle_poses.settle()`'s, unchanged, because it ran a full clean speed sweep on 2026-08-06.
  **How many cycles that was is UNVERIFIED and the two sources disagree** — the evening
  findings say 16, `arm-poses.csv` says 11, same date and same six speeds. Both say *clean*,
  which is the part the predicate rests on. The bench is down; nobody can settle it tonight.
- **The bot cannot enforce sole-writer on `arm_cmd.txt`, and does not pretend to.** Both it
  and the daemon open that file `"w"`, so two writers clobber each other silently with no
  error anywhere. One driver at a time. Named, not fixed — fixing it means changing the daemon.
- **`/stop` cannot interrupt a transition.** The bot is a single long-poll loop, inside the
  settle loop while a move runs, so a `/stop` typed mid-move sits in Telegram's queue until
  the transition finishes — offline this produced `motion_complete` and *then* `STP`, 7.9 s
  apart. **The stop mid-move is the rocker switch.** It always was.
- **Angles are guarded by the pose data, not at the wire.** Every waypoint is asserted inside
  `hold_arm.HOLD`'s limits today, but the guard is the recorded pose table, not a clamp on
  the line. Deliberate, with the reasoning left for Mike.

---

## 6. Decisions waiting on Mike

Plain English, one sentence per option. Number 1 ships at five minutes today because the PRD
recommends it — one setting, and Mike's number.

**These six are not the whole open set, and the count moved — read this before assuming it is.**
They are PRD §17 Q1–Q3 plus plan stop-and-ask items 2, 3 and 4 (3 and 4 merged into number 5),
plus number 6, which is new tonight. **PRD Q4** (may the bot accept commands when you are not in
the room) and **Q5** (how much detail comes back on a refusal) are *absent* because the build
followed the PRD's own recommendation on each — no, and paste the board's exact reply. Those two
were the PRD recommending to itself, **not Mike ruling**. Reopen either if you disagree.

1. **How long should the arm stay willing to move after you say you are ready?** Two minutes means you must be right there but you will be re-typing often; five minutes is comfortable for a few moves in a row and a forgotten phone goes cold quickly; fifteen minutes is convenient but is close to just leaving it switched on.
2. **Should you ever be able to type an exact angle, like "elbow to 50"?** Never, and stay with the named poses the arm has already been driven through and photographed; or allow it for the wrist joints only, which carry the least weight; or allow it for any joint with the soft limits as the only guard.
3. **How should the arm understand a spoken message?** Run the speech-to-text on this laptop with a downloaded model, which is private and free but is a big download and needs a decoder installed; or send the audio to an online service, which needs no download but means your voice recordings leave the machine; or skip voice for now and keep typing, which installs nothing.
4. **How fast should the arm move when the phone asks it to?** Leave it as it is, where the phone changes nothing and the arm moves at whatever speed the holder program set (15 to 20 degrees a second); or raise it, which shortens each move but speeds the arm up rather than slowing it down, and is a decision to make standing at the bench.
5. **Should the two out-of-date lines in the `arm-serial-control` skill be corrected?** One tells the next person to import a helper that now drags a large image library into anything that follows it, and one says the holder program is not saved in the project when it is; fixing them is two edits, or leave them and let the next person hit both.
6. **When the holder program is killed, the bot says "the status file is old" rather than "the link is missing" — should that be made sharper?** Leave it, because both answers refuse the move and nothing changes; or correct the bench script, which currently predicts the other answer for that step; or teach the bot to tell the two apart, which is new code on a path that already refuses correctly.

---

## 7. Next — when the board is plugged back in

That is **plan Phase 4**, the first time a message from a phone reaches a motor. Read the
plan's own step list — it is written as what to record, never as what will happen. Before a
single command is sent:

1. **Plug the board in.** COM5 back, servo power arranged as usual, workspace clear.
2. **Put the camera back** on the arm, off the ceiling.
3. **LOOK at the arm** — eyes, not a frame, not a status file. Confirm where it is and that
   nothing is in the claw or the path. Its resting position after the 21:55 detach was never observed.
4. **Start the daemon and read its log** before the bot is started at all.
5. **Hand near the rocker for every command.** It and the fuse are the only stop.
6. **Support the forearm before the rocker goes off.** No pose here is self-supporting.

**Two of the plan's steps cannot pass as written, and knowing that in advance is the point.**
**Step 1** predicts `NO LINK` for a killed daemon; it will read `STALE`, because the three
files stay on disk and the status file's mtime simply ages out — both refuse, so the
behaviour is right and only the prediction is wrong. **Step 5** expects `/stop`
mid-transition to take effect; it cannot, for the reason in §5. Expect a late `STP`, and do
not read it as the bot ignoring you.

**And one case Phase 4 does not cover at all — add it.** Step 3 restarts the daemon
*mid-window*, between messages, which the code caught even before tonight's fix. The case that
exercises the fix is a restart **mid-transition**, at a phase boundary, while a `/go` is in
flight. Expect the arm to snap to its adopt angles at that instant, so Mike watches it with a
hand near the rocker; the bot must stop, send nothing further, disarm and report. That is the
only bench step that tests §3.
