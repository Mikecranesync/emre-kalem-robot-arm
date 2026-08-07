# arm-sim — a protocol-faithful simulator of `factorylm_arm_controller`

COM5 does not exist tonight. Every host-side thing in this project — the console,
the bridge, `hold_arm.py`, anything that will ever drive a waypoint — is a serial
client with nothing on the other end of the wire. This is the other end.

It parses the same bytes, keeps the same state, and emits the same lines as
`Software/factorylm_arm_controller/factorylm_arm_controller.ino`, so the protocol
can be regression-tested with no board, no servos, no power and no bench.

```
cd Software/arm-sim && python -m pytest -q
86 passed
```

| File | What it is |
|---|---|
| `arm_sim.py` | the simulated firmware. Standard library only. No threads, no sleeps, no wall clock. |
| `fake_serial.py` | a duck-typed stand-in for `serial.Serial`, shaped to what `hold_arm.py` actually calls. Does not import pyserial. |
| `tests/test_protocol_conformance.py` | 86 tests. Sections 12 and 13 of the protocol doc as literal vectors, plus every gotcha below. |

---

## 1. The source of truth is the `.ino`, not the protocol doc

`Documentation/SERIAL-PROTOCOL.md` is the specification and it is a good one.
But where it and the firmware disagree, **this simulator follows the `.ino`**,
because that is what a real board would do. Three such disagreements turned up
while writing this. They are worth more than the simulator is.

Function names in `arm_sim.py` are deliberately the `.ino`'s names — `doSta`,
`doLimSet`, `handleLine`, `writeJoint`, `disableJoint`, `estopAll`. The two files
are meant to be read side by side. **If you change one, diff the other.**

---

## 2. Where the doc and the firmware disagree

### Finding 1 — `STA` carries a `JTO=` field the worked examples do not show

Section 4 calls itself "byte for byte". Section 12 calls itself "exact bytes".
Neither shows `JTO`. The firmware emits it at the end of **every** joint line:

```
doc §4  STA J3 EN=1 SET=95 TGT=110 MIN=70 MAX=110 CAL=1 DPS=30 MOV=1
.ino    STA J3 EN=1 SET=95 TGT=110 MIN=70 MAX=110 CAL=1 DPS=30 MOV=1 JTO=0
```

The doc contradicts itself: section 3's `JOG` text promises `JTO` is "surfaced as
`JTO=<0|1>` on every `STA` joint line", and it is right. Sections 4 and 12 are
stale. **A host written to section 4's field list meets an unknown field on its
very first status poll**, and the two literal test vectors in the doc would fail
against a real board.

Covered by `test_s4_sta_reply_is_byte_for_byte_except_the_missing_JTO_field`,
which reproduces section 4's entire reply and shows exactly where it stops.

### Finding 2 — a watchdog trip emits a second, undocumented `EVT` line

Section 6 enumerates the asynchronous lines a host can see. It lists
`EVT ESTOP SRC=CMD`, `EVT ESTOP SRC=RT` and `EVT WDOG MS=1043`. Section 8 says a
trip produces "`EVT WDOG MS=<elapsed>` then a full e-stop".

A real trip produces **two** lines, because `estopAll()` prints its own `EVT`
with whatever source it was handed:

```
EVT WDOG MS=1001
EVT ESTOP SRC=WDG        <- not in section 6's list
```

`SRC=WDG` appears nowhere in the doc. A host that switches on the `SRC=` value to
decide what to tell the operator falls through its own default branch on the one
event the operator most needs explained.

Covered by `test_watchdog_trips_after_silence_and_detaches_everything`.

### Finding 3 — the e-stop latch gates three verbs, not everything

This is the one with real host consequences.

Section 7's `E7` row reads globally: *"the e-stop / watchdog latch is set — send
`CLR` first"*. In the firmware, `estopLatched` is read by exactly three handlers:
`doEna`, `doMov` and `doJog`. **Everything else answers `OK` on a latched board** —
`LIM`, `SPD`, `MIR`, `WDG`, `DIS`, `DIS A`, `STA`, `PNG`, `VER`, `HLP`, `EST`, and
bare `STP`.

That is almost certainly correct behaviour, and it is useful: it is what would let
a host push limits, speeds and the mirror relation into a board that latched
before the host ever connected, without having to clear the latch first.

One sharp edge falls out: `STP <j>` on a latched board answers `ERR E6 … JOINT=3`
("not enabled"), not `E7`. The latch detached the joint, so the complaint is about
the joint's state. A host that maps `E7` to "press CLR" gives the operator no hint
at all here.

Covered by `test_the_latch_gates_only_ena_mov_and_jog`, which asserts all three
gated verbs and all twelve ungated ones.

### What to do about the three — in plain English

Nothing here is broken firmware. The question is only which document to correct,
and it is Mike's call:

- **Fix the doc, leave the firmware alone** — the firmware behaviour is right in
  all three cases; the doc is just out of date.
- **Fix the doc and add `SRC=WDG` to the async-line list** — same as above, plus
  the one line a host actually needs to switch on.
- **Leave both and rely on this test suite** — cheapest today, but the next person
  to write a host reads the doc, not the tests.

The first two cost minutes. The third costs somebody an afternoon, later.

---

## 3. What the simulator models

Everything the protocol can express, and the state behind it:

- **Every verb**: `VER PNG STA LIM MIR ENA DIS MOV SPD STP JOG EST CLR WDG HLP`,
  plus the raw `!` and `?` bytes intercepted before line assembly.
- **Every error code** section 7 says the firmware can emit — `E1`–`E14`, in the
  firmware's own precedence order, with the exact detail keys.
- **The strict integer parser.** Rejects an empty field, a bare `-`, a leading
  `+`, a float, trailing garbage and overflow. `DIS A` is the one exception,
  matched before parsing is attempted.
- **The 48-character line limit**, including the terminator. 47 content characters
  parse; 48 gets `ERR E8 LINE` and the line is never acted on.
- **Adopt-before-drive.** `ENA <j> <adopt>` pre-loads the pulse *before* attaching,
  so the pin is already at the adopt angle with zero virtual time elapsed. That is
  the whole of landmine 2 in the `.ino` header, and it is directly asserted.
- **The interpolator**, on an injectable clock: elapsed-time integration in
  centidegrees, the 20 ms tick, `TICK_CAP_MS`, and `MAX_STEP_C`.
- **The jog command-age timeout** — silent on the wire, latched as `JTO=1`.
- **The serial watchdog** — including precisely what does and does not feed it.
- **The shoulder mirror**, offset included, and the clamp at the point of write.
- **`millis()` rollover at 2³²**, with the `.ino`'s unsigned-subtraction idiom.
  Two tests cross the wrap. Nobody was going to hold a bench session open for
  49.7 days to check that.
- **Per-pin state** — `('PULSING', microseconds)` or `('LOW', None)` for all seven
  servo pins, which is the only way sections 13's multimeter rows are testable at
  all. Read the next section before you trust it.

---

## 4. What it does NOT model — read this before believing anything

**It is a model of the FIRMWARE. It is not a model of the ARM.** It knows what the
sketch commanded. It knows nothing else, because the sketch knows nothing else.

- **No servo mechanics.** No torque, no current, no stall, no brownout, no
  temperature. A `MOV` that would strip a gear looks exactly like one that would
  not.
- **No gravity and no sag.** The real arm settles downward the whole time it is
  held; five energised joints were observed doing it. Nothing here does.
- **No slipping J6 gear.** The gripper acked every command on 2026-08-06 and its
  fingers provably did not articulate. This simulator would have acked them too,
  cheerfully, and reported a clean `SET=`.
- **No dead J0 servo.** J0 is modelled as a working channel because the firmware
  treats it as one. On the bench it is a dead motor that holds nothing.
- **Not the Servo-library detach race — and this one matters most.**
  `pin_state()` reports the firmware's *intent*: it detached, then drove the pin
  LOW. Section 13 step 15 is called "the most important line in this table"
  because `detach()` landing inside the ISR window can leave a pin driven HIGH
  forever, and every safety mechanism in the system routes through detach. There
  is no ISR here, so **a green step-15 test proves the sketch calls the right
  functions in the right order and proves nothing whatsoever about a real pin.**
  That row still has to be done at the bench with a multimeter.
- **No voltages.** `pin_state()` returns microseconds, never volts. The ~0.37 V
  figure is a *measurement* off a real pin; deriving it here and printing it would
  be manufacturing a bench result.
- **No Optiboot window.** Opening the port resets the board instantly. On real
  hardware the bootloader eats about a second, which is why section 9 mandates a
  2000 ms wait — a host bug in that wait will not show up here.
- **No baud rate and no transmit time.** `write()` has already reached the
  firmware when it returns. The real `STA` reply is a ~36 ms burst at 115200.
- **No `RX_BUDGET`.** The firmware drains at most 32 bytes per `loop()` so a
  pasted block cannot starve the interpolator. With no wall clock there is
  nothing to starve, so modelling it would buy no testable behaviour.
- **No line noise, no dropped bytes, no second client stealing the port.**
- **`close()` does not reset the board.** The reset is modelled at *open*, because
  whether closing asserts DTR is transport-specific — section 9 is explicit that
  transports differ. Both routes into `open()` reset: constructing *with* a port
  opens immediately, `port=None` defers until you call `open()` yourself, which
  is the shape `arm-bridge.py` uses.
- **No `bytesize` / `parity` / `stopbits` / `write_timeout`.** `arm-bridge.py`
  assigns all four before `open()`. Python accepts an assignment to any attribute
  of an ordinary object, so they cost nothing and break nothing — and there is no
  wire here for a framing setting to describe. Whether they should ever become
  real is a scope call for a human, not a bug.
- **`FakeSerial` never blocks and never advances time.** `timeout` is stored and
  ignored. A host loop that waits on wall-clock timeouts will busy-spin against
  it; drive time explicitly with `port.sim.advance(ms)`.

---

## 5. Quirks reproduced deliberately, not fixed

A simulator that quietly improves on the firmware is worse than none, because it
teaches a host to expect behaviour the board does not have.

- **`ERR E2 <VERB> N=4` for a five-argument line.** The parser stops collecting at
  four tokens and then reports the *capped* count. A host that echoes `N` to the
  operator will say "4 arguments" about a line that had five.
- **A stalled loop never makes up the time it lost.** After a slice longer than
  `TICK_CAP_MS`, the firmware sets `lastTickMs = now` and the excess is discarded.
  A stalled board arrives late and never catches up. That is the safe direction,
  and worth knowing before somebody times a move by wall clock.
- **`ERR E5 WDG` carries no detail keys at all.** The known overload the doc
  admits to in section 7. A host cannot even show the operand it refused.
- **`MIR` checks "is joint 1 enabled?" before it checks the mode word**, so
  `MIR SIDEWAYS` on a live joint 1 answers `E9`, not `E11`.
- **The integer parser runs before the joint id is resolved**, so `LIM 2 x 110 0`
  answers `E3 ARG=2` and never mentions that joint 2 is reserved.
- **`DIS` has no state check.** Disabling an already-disabled joint answers `OK`.

---

## 6. Open questions and numeric nits — not findings

- **Line terminator is UNVERIFIED.** This emits `\n`, per section 12's explicit
  "every line ends in `\n` (`0x0A`)". Whether Arduino's `Print::println()` actually
  emits `\r\n` cannot be checked from this repository — the core is not in it, and
  the bench is disconnected. It is one constant (`arm_sim.EOL`) if that is ever
  settled at the bench. Every host in this repo strips its lines, which is why
  nobody would have noticed either way.
- **Section 12 step 3 says "~1524 µs" for 95°. The integer maths gives 1523.**
  `544 + (9500 × 1856) / 18000` truncates to 1523. The `~` makes it non-binding,
  so this is a rounding nit and not a disagreement. The load-bearing figure,
  1472 µs at 90°, is exact and is asserted directly against the value
  `V2-SERVO-POWER-AND-WIRING.md` §6 used to derive the ~0.37 V reading.
- **`Software/tests/protocol_check.py`'s `PEND` markers are stale.** Its own header
  says "a PEND that starts PASSing is a signal that the case needs promoting".
  Run against this simulator it reports `64 passed, 0 pending, 0 failed` — every
  PEND-marked case passes. They cover `STP <j>` (Task 2), `LIM_MIN_SPAN_DEG`
  (Task 3), `LIM` on a driven joint (Task 4), and the `JOG` verb, the `jogActive`
  guard and the `JTO` latch (Task 5). The `.ino` implements all of them. Promoting
  those cases is a separate, one-line-each change and is **not** made here.

---

## 7. Verified against something other than my own reading

The 86 tests here were written from the `.ino` by the same session that wrote the
simulator, which is a closed loop. So it was also pointed at
`Software/tests/protocol_check.py` — a harness written earlier, against the real
board, that knows nothing about any of this.

**64 passed, 0 pending, 0 failed.** Recipe, run by hand and deliberately not
committed (it monkeypatches `serial`, which is not something to leave lying
around in a repo that drives motors):

1. Load `arm_sim.py` and `fake_serial.py` by path.
2. Register a fake `serial` module whose `Serial` is a `FakeSerial` subclass that
   advances the sim clock by 5 ms whenever a read comes back empty, and whose
   `serial.tools.list_ports.comports()` returns `[]`.
3. Replace `time.sleep(s)` with `sim.advance(s * 1000)` so the harness's quiet
   windows pass virtual time instead of real time.
4. `Board("SIM")`, then `non_motion_tests(b)` and `motion_tests(b)`.

That is a genuinely independent check of the wire format, the error strings, the
jog timeout's silence, and the `STP <j>` semantics.

**Re-run from scratch by a later review session: 64 passed, 0 pending, 0 failed.**
The recipe above works verbatim — the clock-advance-on-empty-read hook is needed
only on `readline()`, not on `read()`, and no other deviation was required. This
matters because an uncommitted recipe nobody re-runs is a claim, not evidence.
It is still not committed, for the reason above.

---

## 8. Where I was wrong while building this

Kept because each was nearly written down as fact.

1. **I expected `MIR INV 90` to be accepted, because the doc's bound is ±90.**
   It is refused, at every real joint-1 range, with `MIRROR=out_of_travel`. There
   are *two* checks on that operand and the documented ±90 is not the binding one
   — the envelope check bites first and bites much earlier. The inclusive ±90
   bound is only reachable through `MIR SAME`, which skips the envelope check.
   **A documented limit is not the limit if a second check sits behind it.**
2. **I expected narrowing joint 1's MAX to buy room back for a positive offset.**
   Wrong end. `INV` flips, so it is joint 1's **MIN** that mirrors to the *top* of
   D5's travel. Trimming MAX changes nothing; raising MIN is what buys it. I made
   exactly the mirror-direction mistake that `joint-limits.csv`'s header block
   warns strips gears — on paper, with the arm disconnected, which is the cheap
   place to make it. `test_mir_is_validated_against_joint1_limits_as_they_stand_right_now`
   now asserts both ends so nobody repeats it.
3. **I was about to have `pin_state()` return ~0.37 V.** It would have read like a
   measurement and it would have been arithmetic. It returns microseconds.

### Found by an adversarial review afterwards, not by me

Kept separate because the provenance matters: 1-3 above were caught while
building, these two were not caught at all until a second session went looking
for them. Both are now fixed and asserted.

4. **I wrote that `hold_arm.py` was "the only thing in this repo that opens the
   port and drives joints". It is not, and `arm-bridge.py` is the counter-example
   I should have found** — it is the console's serial bridge, the biggest host
   here, it opens the port (lines 312-326) and `tx()` relays verbatim `ENA`,
   `MOV` and the realtime `!` byte from the browser (line 418). The false claim
   was load-bearing: it was the stated justification for how small `FakeSerial`'s
   surface is. Worse, arm-bridge uses the **deferred-open** pattern — bare
   `Serial()`, then `.port`/`.dtr`/`.rts`, then `.open()` — so under the old
   construct-time-only reset it reset the board *before `.port` was assigned* and
   then raised `AttributeError` on `.open()`. The board reset now hangs off
   `open()`, where the real DTR assertion is. **A confidently-worded scope
   justification is worth checking before the code it justifies.**
5. **A NUL byte on the wire did not truncate the line.** The `.ino` parses the
   buffer as a **C string** — `loop()` writes `line[lineLen] = 0` and `nextTok()`
   stops dead at the first NUL — so `MOV\0 3 95` is a bare `MOV` with no
   arguments (`ERR E2 MOV N=0`). Python strings have no such rule, so the sim
   read it as a five-character verb and answered `ERR E1 VERB TOKEN=MOV\0`. Low
   consequence and never observed on hardware, but it is exactly the class of
   thing this simulator exists to get right: **the `.ino` is C, and C semantics
   are part of the behaviour, not an implementation detail.**

---

## 9. Do not

- ❌ Treat a green run here as evidence about the physical arm. It is evidence
  about the firmware's wire behaviour and nothing else.
- ❌ Treat a green section-13 step 15 as evidence that a pin went low. Bench
  and multimeter only.
- ❌ "Fix" a quirk in section 5 so the simulator is tidier than the board.
- ❌ Change `arm_sim.py` to match `SERIAL-PROTOCOL.md` where the `.ino` differs.
  Fix the doc, or change the firmware and this together.
- ❌ Add a dependency. Standard library only — pyserial and numpy are BSD-3-Clause
  and already carry a flagged exception elsewhere in this repo; there is no reason
  to spread them here.
- ❌ Use `time.sleep()` in a test. Time is injected; a suite that waits on a
  1000 ms watchdog in real seconds is a suite nobody runs.

## Files that go with this one

| File | What it is |
|---|---|
| `Software/factorylm_arm_controller/factorylm_arm_controller.ino` | the firmware this imitates, and the tie-breaker |
| `Documentation/SERIAL-PROTOCOL.md` | the specification, §12 and §13 of which are the test vectors |
| `Software/tests/protocol_check.py` | the same shape of assertions, against the real board |
| `Software/arm-console/hold_arm.py` | where `FakeSerial`'s surface came from |
| `Software/arm-console/joint-limits.csv` | the real locked limits the handshake test pushes |
| `.claude/skills/arm-bench-safety/SKILL.md` | why an ack is not motion, and why none of this is an e-stop |
