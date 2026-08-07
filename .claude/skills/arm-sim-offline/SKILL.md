---
name: arm-sim-offline
description: Use when touching Software/arm-sim/ — arm_sim.py, fake_serial.py, FakeSerial, or the protocol conformance tests — and whenever writing or testing anything that speaks the arm's serial protocol with the bench disconnected, a host with no COM port, or a protocol question nobody can put a meter on tonight. Trigger again before writing "verified", "works" or "confirmed" on the strength of a green run, because a green simulator run is evidence about the wire format and evidence about nothing else.
---

# Developing against the simulator, with no board on the wire

`Software/arm-sim/` is a model of `factorylm_arm_controller.ino`. It parses the same
bytes, keeps the same state and emits the same lines, so host code can be written and
the protocol regression-tested with no board, no servos and no power.

**It is a model of the FIRMWARE. It is not a model of the ARM.** Everything below either
helps you use it or stops you overclaiming from it; the second job is the bigger one.

---

## 1. The rule this skill exists to enforce

**A green simulator run is not evidence about the arm. It is evidence about the wire
format.**

This repo has already paid for the same mistake twice, in two different disguises:

- **A board ack was not motion.** The firmware provably drove D3 at 29.2 °/s for 3.5 s
  and nothing turned. The servo was dead. That cost an afternoon.
- **A pixel difference was not a finger moving.** The gripper harness printed `MOVED`
  twice, two different ways, for a joint whose fingers provably do not articulate.

A green protocol test is the third disguise, and the most convincing of the three because
it arrives with a number. A passing suite says a model of a host talks correctly to a
model of a firmware. It says nothing whatsoever about a motor turning.

**Never write a sentence that could be read as a bench result.** Say what you exercised.

## 2. Running it, and where the count comes from

```
cd Software/arm-sim && python -m pytest -q
88 passed
```

`README.md` says 86 in three places and the suite reports 88. **Take the count from the run,
never from prose** — including this file. The artefact is evidence; a sentence about it is a claim.

| File | What it is |
|---|---|
| `arm_sim.py` | the simulated firmware. Stdlib only. No threads, no sleeps, no wall clock. |
| `fake_serial.py` | a duck-typed stand-in for `serial.Serial`. Does not import pyserial. |
| `tests/test_protocol_conformance.py` | `SERIAL-PROTOCOL.md` §12 and §13 as literal vectors, plus every gotcha in §4. |

## 3. Pointing a host at it — two shapes, and the difference matters

Both exist in this repo, so both are supported — and **the DTR board reset hangs off
`open()`**, not off construction.

```python
import sys; sys.path.insert(0, "C:/RobotArm/Software/arm-sim")
from fake_serial import FakeSerial

# IMMEDIATE-OPEN -- hold_arm.py's shape. A port argument means open NOW.
ser = FakeSerial("SIM", 115200, timeout=0.3)   # opens AND resets inside __init__
ser.write(b"VER\n"); ser.flush(); print(ser.read(ser.in_waiting))

# DEFERRED-OPEN -- arm-bridge.py's shape. Construct bare, configure, then open.
ser = FakeSerial(port=None)        # nothing opens, nothing resets yet
ser.port = "SIM"
ser.dtr = True; ser.rts = True     # DTR asserted BEFORE open, as the bridge does
ser.open()                         # the board reset happens HERE
```

Getting that backwards is not cosmetic: under an earlier construct-time-only reset, the
bridge's sequence reset the board *before `.port` was assigned*, then raised `AttributeError`
on `.open()`. Two more things you will need:

- **`sim=` an existing board** when you want to prove the reset actually discarded
  something. Against a fresh `ArmSim` the assertion is a tautology.
- **Time is injected.** `ser.sim.advance(ms)` moves it, and a *large* `step_ms` on
  `ArmSim.advance(ms, step_ms=…)` simulates a stalled `loop()` — `advance(200,
  step_ms=200)` hands the interpolator the single 200 ms slice `TICK_CAP_MS` and
  `MAX_STEP_C` exist to bound. **Never `time.sleep()` in a test.**

Use the port name `SIM`: no string in a test or doc should read like an instruction to
open the real port.

## 4. The gotchas it models — this is why you do not hand-roll a stub

A stub you write in ten minutes answers `OK` to everything and teaches your host to expect
a board that does not exist. Each of these has already bitten someone here, or is one
reconnect away:

- **The 70–110 boot default clamps the gripper.** J6's real range, 10–70, lies almost
  entirely below it, so after any reset every J6 command under 70 clamps (`CL=1`) until
  `LIM 6 10 70 1` goes out.
- **`PNG` feeds the watchdog and never clears a latch.** Once `ES=1` or `WD=1` is set it
  stays set. A loop that only pings will answer `STA` cheerfully forever with the arm
  on the bench.
- **`STA` answers cheerfully with every joint detached.** A reply is not a held arm.
  Validity keys off `EN=`, never off `SET=` — the firmware seeds every joint to 90 at boot.
- **A reset discards limits, mirror and enables.** Back to 70–110 with `CAL=0`,
  `MIR=UNKNOWN`, every joint detached. Any host that opens the port must re-push all of it.
- **`ENA <j> <adopt>` DRIVES the joint to `<adopt>`**, pre-loading the pulse before
  attaching. Adopt where the joint is and nothing moves; adopt at midpoint after it has
  sagged and it lifts the arm to get there.
- **`SPD` above 90 is `E12`**, carrying `REQ` / `MIN` / `MAX` keys.
- **`LIM` takes FOUR arguments** (`LIM <j> <min> <max> <cal>`) and **`STA` takes NONE** —
  three args to `LIM`, or any arg to `STA`, is `ERR E2`.

It reproduces quirks rather than tidying them — `ERR E2 MOV N=4` for a five-argument line,
`ERR E5 WDG` with no detail keys, `DIS` on an already-disabled joint answering `OK`. **Do
not "fix" one:** a simulator that improves on the board teaches a host to expect behaviour
the board does not have.

## 5. Three places the protocol doc is stale — the `.ino` wins

Found by the simulator and worth more than it. Anyone reading `SERIAL-PROTOCOL.md` needs
these, and each names the section to correct:

1. **`STA` carries a `JTO=` field the worked examples do not show.** The firmware emits it
   at the end of every joint line, so **§4 ("byte for byte") and §12 ("exact bytes") are
   stale** — their literal vectors would fail against a real board, and a host built to §4's
   field list meets an unknown field on its first poll. §3's `JOG` text has it right, which
   is what proves the other two wrong.
2. **A watchdog trip emits a second, undocumented line:** `EVT WDOG MS=…` then
   `EVT ESTOP SRC=WDG`. **§6's async-line list is missing `SRC=WDG`; §8's "On trip" row
   describes one line where there are two.** A host switching on `SRC=` falls through its
   default branch on the one event the operator most needs explained.
3. **The e-stop latch gates three verbs, not everything.** Only `ENA`, `MOV` and `JOG` read
   it; the other **twelve** — `LIM`, `SPD`, `MIR`, `WDG`, `DIS`, `STA`, `PNG`, `VER`, `HLP`,
   `EST`, `CLR`, bare `STP` — answer `OK` latched, so **§3's `STP`-vs-`EST` row and §7's
   `E7` row both read globally.** Sharp edge: `STP <j>` latched answers `ERR E6 … JOINT=3`,
   not `E7` — a host mapping `E7` to "press CLR" gives the operator no hint at all.

None of these is broken firmware — the firmware is right in all three and the doc is
stale. **Which document to correct is Mike's call.** Fixing the doc costs minutes;
leaving it and relying on the tests costs somebody an afternoon later, because the next
person to write a host reads the doc, not the tests.

## 6. What it does NOT model — read this before believing anything

It knows what the sketch commanded, and nothing else, because the sketch knows nothing else.

- **No servo mechanics.** No torque, no current, no stall, no brownout, no temperature.
  A `MOV` that would strip a gear looks exactly like one that would not.
- **No gravity and no sag.** The real arm settles downward the whole time it is held.
- **Not the slipping J6 gear.** The gripper acked every command on 2026-08-06 and its fingers
  provably did not articulate. This would have acked them too, with a clean `SET=`.
- **Not the dead J0 servo.** J0 is modelled as a working channel because the firmware
  treats it as one. On the bench it is a dead motor that holds nothing.
- **No voltages.** `pin_state()` returns microseconds, never volts. §13's ~0.37 V is
  **arithmetic** (1472 µs per 20 ms = 7.4 % of 5 V), not a logged reading — README §8 item 3.
- **No transmit time and no baud.** `write()` has already arrived when it returns; a real
  `STA` reply is a ~39 ms burst at 115200 (453 **sim** bytes × 10 bits — arithmetic off the
  model, never timed). No line noise, dropped bytes or rival client; framing settings inert.
- **No Optiboot window.** Opening resets instantly, so a host bug in §9's mandatory
  2000 ms wait cannot show up here.
- **`FakeSerial` never blocks and never advances time.** `timeout` is stored and ignored;
  a host loop waiting on wall-clock timeouts busy-spins against it. Drive time explicitly.

### And above all: there is no ISR

`SERIAL-PROTOCOL.md` §13 step 15 is called the most important line in its table because a
`detach()` landing inside the Servo library's interrupt window skips the LOW write and
**can leave a pin driven HIGH forever** — an infinitely long pulse, and every safety
mechanism here routes through detach. `pin_state()` reports the firmware's *intent*: it
detached, then drove the pin LOW.

**A green step-15 test proves the sketch calls the right functions in the right order and
proves nothing whatsoever about a real pin.** §13's meter rows are 2, 9, 10, 15, 18, 25, 26;
only 15, 18 and 26 probe after a detach — 2 and 10 are never-attached baselines. At the bench.

## 7. Building on it — `fake_daemon.py` is the worked example

`Software/arm-telegram/tests/fake_daemon.py` models `hold_arm.py`'s file channel on a real
`ArmSim`. Three rules generalise from it:

- **Generate replies by handing the line to an `ArmSim`.** Never hand-write a canned reply.
  The wire format is then the firmware's, not yours, and stays right when the firmware moves.
- **Conformance-pin your model to the real source** — filenames, log format, poll
  interval, constants. A model nothing pins drifts into fiction quietly.
- **Name every deliberate divergence as a divergence, in the file.** `fake_daemon` does
  not poll on its own, and says so and says why: a stopped loop is exactly the failure
  being tested, so modelling the automatic poll would make it unreachable.

That commit reports 259 passing tests for the Telegram surface — a model of a bot talking
to a model of a daemon talking to a model of a firmware. Three model layers, zero arm.

## 8. Where the simulator's authors were wrong

Not mine — `arm-sim/README.md` §8's register: five entries, three caught while building and
**two only when a second session went looking.** The two carried here are the ones that change
how you *use* it — one from each group, so neither bullet below is "the adversarial pair".

- **"`hold_arm.py` is the only thing that opens the port" was false**, and load-bearing —
  it was the stated justification for how small `FakeSerial`'s surface is. `arm-bridge.py`
  opens the port, drives joints, and uses §3's deferred-open shape. *A confidently-worded
  scope justification is worth checking before the code it justifies.*
- **`MIR INV 90` is refused even though the documented bound is ±90.** Two checks sit on
  that operand and the envelope check bites first, much earlier. *A documented limit is
  not the limit if a second check sits behind it.*

## 9. Do not

- ❌ Say "verified", "working" or "confirmed" about a joint on the strength of a green run.
- ❌ Treat a green §13 step-15 test as evidence that a pin went LOW. Meter, at the bench.
- ❌ Quote a test count from a README. Run the suite.
- ❌ "Fix" a quirk so the simulator is tidier than the board.
- ❌ Change `arm_sim.py` to match the doc where the `.ino` differs. Fix the doc instead.
- ❌ Hand-roll a stub because this looks like overhead — yours will ack a clamped gripper.
- ❌ Use `time.sleep()` in a test, or add a dependency. Standard library only.
- ❌ Write `COM5` in a sample. The port name here is `SIM`.

## Cross-references

- `Software/arm-sim/README.md` — full reference, quirks list, and the independent check
  against `Software/tests/protocol_check.py`.
- `Documentation/SERIAL-PROTOCOL.md` — the spec. §12/§13 are the vectors; §4/§6/§7/§8 are
  the stale sections §5 names.
- `arm-serial-control` — the same protocol against a real board, and the daemon rules.
- `arm-bench-safety` — why an ack is not motion, and why nothing in software is a stop.
- `arm-motion-verify` — how a shaft is actually proven to have turned.
