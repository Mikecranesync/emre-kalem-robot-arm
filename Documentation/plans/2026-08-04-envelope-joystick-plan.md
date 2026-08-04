# Envelope + Joystick + Lock-an-Axis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the operator widen a joint's soft stops by dragging, drive the joint by feel with a spring-back joystick, and record the range they settled on as accepted, acknowledged data — without leaving the console or editing a CSV by hand.

**Architecture:** Firmware first. `STP <j>` aborts one joint's motion; `LIM` becomes atomic, span-checked, and acceptable on a driven joint when it cannot exclude that joint; `JOG` gives joystick motion its own verb and a board-side command-age timeout. Console after: an acknowledgment gate, then the envelope, joystick, exact-angle entry and lock.

**Tech Stack:** Arduino C++ (AVR, `Servo 1.3.0`), ES5 JavaScript inline in one HTML file, Python 3.11 + pyserial for the firmware harness, headless Chrome for the console harness.

**Spec:** `Documentation/specs/2026-08-04-envelope-joystick-design.md` (revised 2026-08-04 after external review)

## Global Constraints

- **Firmware:** no `String`, no `malloc`, no executable `delay()`, all literals in `F()`. Baseline build is 11650 B flash / 405 B SRAM; ceiling is 32256 B / 2048 B.
- **Console:** ES5 only — `var`, `function`; no arrow functions, `let`, `const`, or template literals. Must stay a single standalone HTML file that works when double-clicked from `file://`.
- **Vocabulary, everywhere — firmware, console, docs, logs, UI copy:** use `commanded`, `target`, `held`, `accepted`, `soft limit`. **Never** `actual`, `measured`, `feedback`, `position`. The one approved exception is the console's `MEASURED` badge, which refers to a human's measurement.
- **No software behaviour may be called an emergency stop.** `STP` and the jog timeout are motion aborts. They do not remove power and do not know the shaft angle. The rocker and the fuse are the emergency stop.
- **A drag never causes motion. A limit edit never creates travel.**
- **`calibrated` is set by exactly one code path** — the LOCK button, Task 11.
- **No EEPROM.** The repo has no versioned, checksummed persistence mechanism.
- Conventional commits. Every commit message ends with:
  ```
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ
  ```
- **Bench safety:** servo power OFF (rocker off, supply unplugged) for every task except Task 13. `protocol_check.py` in its default mode is safe with power on, but there is no reason to take that risk before Task 13.
- **Do not push, open a PR, or merge** until Task 13 and only with the operator's explicit go-ahead.

## Stop Conditions

Stop and report rather than guessing if any of these is true:

- a source file does not match an anchor quoted in this plan;
- `degToCmd`, the logical-to-physical mapping, or the shoulder mirroring is ambiguous;
- a change would break compatibility with the existing protocol;
- the firmware cannot cleanly distinguish jog motion from finite motion;
- compile or upload tooling is unavailable;
- a test causes motion that was not expected;
- the physical stop or power arrangement is unclear;
- unrelated local changes would be overwritten.

## Review Checkpoints

Adversarial review of the actual diff and test output at three points. Fix findings before continuing.

1. After Task 6 — firmware protocol, limits, timeout, harness.
2. After Task 12 — console envelope, joystick, target angle, lock, self-test.
3. Before Task 13 step 7 — powered shoulder testing.

## File Structure

| File | Responsibility | Tasks |
|---|---|---|
| `Software/tests/protocol_check.py` | **new** — non-motion protocol tests by default; `--motion-ok` for bounded motion | 1, 6 |
| `Software/factorylm_arm_controller/factorylm_arm_controller.ino` | `STP <j>`, atomic `LIM`, logical enforcement, `JOG` + timeout | 2–5 |
| `Documentation/SERIAL-PROTOCOL.md` | the contract — `STP` gains an argument, `E9` narrows, `JOG` and `EVT JOGTIMEOUT` are new | 2–5 |
| `Software/arm-console/arm-console.html` | helpers, self-test, ack gate, envelope, joystick, target angle, lock | 7–12 |
| `Software/tests/selftest.sh` | **new** — fails unless `SELFTEST_PASS` present and `SELFTEST_FAIL` absent | 7 |
| `Calibration_Notes/calibration-log.csv` | one row per lock | 11 |

---

## Task 0: Verify every anchor before touching code — ✅ COMPLETE 2026-08-04

> **Findings: `Documentation/2026-08-04-envelope-joystick-baseline.md`.** Ten corrections
> (C1, C2, C4, C7–C13) are already folded into this plan and the spec. Baseline build confirmed
> at **11650 B flash / 405 B SRAM**, `arduino-cli 1.5.1`, `arduino:avr 1.8.8`, `Servo 1.3.0`.
> No stop condition was hit. The steps below are retained as the record of what was checked.

**Files:** Create `Documentation/2026-08-04-envelope-joystick-baseline.md`

No production code changes. This task exists because the plan below quotes line numbers and
signatures, and a plan that contradicts the source is worse than no plan.

- [ ] **Step 1: Read both documents end to end** — the spec and this plan.

- [ ] **Step 2: Confirm each anchor**

```bash
cd /c/RobotArm
grep -n "static void doStp" Software/factorylm_arm_controller/factorylm_arm_controller.ino
grep -n "static void doLimSet" Software/factorylm_arm_controller/factorylm_arm_controller.ino
grep -n "VIS('S','T','P')" Software/factorylm_arm_controller/factorylm_arm_controller.ino
grep -n "jointArg\|intArg\|badArgc\|errJPre\|errJoint\|okPre\|okDone" Software/factorylm_arm_controller/factorylm_arm_controller.ino | head -20
grep -n "function clampi\|function send\|function paintJoint\|function buildCards\|function loadLimitsCsv\|LIM_COLS" Software/arm-console/arm-console.html
```

Record what each actually is. **Where this plan is wrong, correct the plan** — do not preserve an assumption the source disproves.

- [ ] **Step 3: Resolve `degToCmd` exactly**

```bash
grep -n -B3 -A12 "degToCmd" Software/factorylm_arm_controller/factorylm_arm_controller.ino
```

Write down its signature, its units, and whether `setC` / `tgtC` are command units or degrees. Every comparison in Tasks 4 and 5 depends on this. **If it is ambiguous, stop and report** (Stop Conditions).

- [ ] **Step 4: Map logical joints to physical servos**

```bash
grep -n -B5 -A25 "mirror\|MIR_INV\|leftC" Software/factorylm_arm_controller/factorylm_arm_controller.ino | head -60
```

Document: which logical id drives which pin(s); exactly where the mirrored shoulder servo's command is derived; and whether any calibration offset is applied between logical and physical. Task 4 must enforce limits *before* all of it.

- [ ] **Step 5: Characterise the parser and reply grammar**

Record: maximum line length and what happens past it; how tokens are split; whether trailing garbage after a valid number is rejected; the exact shape of `OK` and `ERR` replies; what `EVT` lines exist today; what happens on reconnect (the port reset) to enabled joints, limits, and the watchdog.

- [ ] **Step 6: Record the baseline build**

```bash
"C:\Program Files\Arduino CLI\arduino-cli.exe" compile --fqbn arduino:avr:uno "C:\RobotArm\Software\factorylm_arm_controller"
```

Expected: `0 errors`, `11650 bytes (36%)` flash, `405 bytes (19%)` SRAM. If the numbers differ from those, the tree is not where this plan thinks it is — investigate before continuing.

- [ ] **Step 7: Write the findings up and commit**

```bash
cd /c/RobotArm
git add Documentation/2026-08-04-envelope-joystick-baseline.md
git commit -m "docs: source-verified baseline before the envelope/joystick work

Anchors, degToCmd's real signature and units, the logical-to-physical
mapping, parser and reply grammar, reconnect behaviour, and the baseline
build figures. Any place the plan contradicted the source is corrected in
the plan, not worked around.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 1: The protocol harness — non-motion by default

**Files:** Create `Software/tests/protocol_check.py`

**Interfaces:**
- Produces: `Board.cmd(line) -> list[str]`, `expect(name, reply, needle)`, `expect_unchanged(name, board, probe, needle)`, and a `--motion-ok` flag consumed by Task 6.

**This runs first, before any firmware change**, so it captures today's behaviour and every later task has something to fail against.

- [ ] **Step 1: Write the harness**

```python
"""Protocol tests against the real board.

    python Software/tests/protocol_check.py [--port COM5] [--motion-ok]

DEFAULT MODE IS NON-MOTION. It enables nothing and moves nothing, so it is
safe to run with servo power on. --motion-ok adds bounded motion tests and
prints a physical-safety acknowledgment first.

Nothing here is an emergency stop. STP and the jog timeout abort motion and
hold the last commanded value; they do not remove power.
"""

import argparse
import sys
import time

import serial
from serial.tools import list_ports

BAUD = 115200
FAILURES = []


class Board:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.4)
        time.sleep(2.0)                      # Optiboot holds the bus ~1 s after the reset
        self.ser.reset_input_buffer()

    def cmd(self, line, settle=0.0):
        """Send one line; return reply lines up to and including OK/ERR."""
        self.ser.write((line + "\n").encode("ascii"))
        out, deadline = [], time.time() + 2.0
        while time.time() < deadline:
            raw = self.ser.readline()
            if not raw:
                continue
            text = raw.decode("ascii", "replace").strip()
            if not text or text.startswith(";"):
                continue
            out.append(text)
            if text.startswith("OK") or text.startswith("ERR"):
                break
        if settle:
            time.sleep(settle)
        return out

    def close(self):
        self.ser.close()


def expect(name, reply, needle):
    joined = " | ".join(reply)
    if needle in joined:
        print("  PASS  %s" % name)
    else:
        print("  FAIL  %s\n        wanted %r\n        got    %r" % (name, needle, joined))
        FAILURES.append(name)


def expect_unchanged(name, board, probe, needle):
    """A rejected command must leave state exactly as it was."""
    expect(name, board.cmd(probe), needle)


def find_port():
    for p in list_ports.comports():
        if "2341" in (p.hwid or ""):
            return p.device
    ports = list_ports.comports()
    return ports[0].device if ports else None


def non_motion_tests(b):
    print("\n-- handshake and status --")
    expect("VER identifies the arm controller", b.cmd("VER"), "NAME=FACTORYLM-ARM")
    expect("PNG replies with uptime", b.cmd("PNG"), "OK PNG UP=")
    expect("STA reports six joints", b.cmd("STA"), "OK STA N=6")
    expect("LIM with no args lists limits", b.cmd("LIM"), "LIM J3")
    expect("HLP replies", b.cmd("HLP"), "OK HLP")

    print("\n-- malformed input --")
    expect("unknown verb is refused", b.cmd("XYZ"), "ERR E1")
    expect("a typo does not trigger a stop", b.cmd("MOOV 3 90"), "ERR E1")
    expect("missing arguments", b.cmd("ENA"), "ERR E2")
    expect("too many arguments", b.cmd("PNG 1 2 3"), "ERR E2")
    expect("non-numeric argument", b.cmd("MOV x 90"), "ERR E3")
    expect("trailing garbage after a number", b.cmd("MOV 3 90abc"), "ERR E3")
    expect("overlong line is discarded", b.cmd("MOV 3 " + "9" * 80), "ERR E8")
    expect("blank line is silently ignored", b.cmd("STA"), "OK STA")

    print("\n-- joint ids --")
    expect("joint id out of range", b.cmd("STP 9"), "ERR E4")
    expect("reserved joint id names itself", b.cmd("ENA 2 90"), "RESERVED=shoulder_pair")
    expect("negative joint id", b.cmd("MOV -1 90"), "ERR E4")

    print("\n-- limits validation (joint disabled throughout) --")
    b.cmd("LIM 3 70 110 0")
    expect("reversed limits", b.cmd("LIM 3 120 40 0"), "ERR E10")
    expect("equal limits", b.cmd("LIM 3 90 90 0"), "ERR E10")
    expect("min below absolute bound", b.cmd("LIM 3 -5 110 0"), "ERR E10")
    expect("max above absolute bound", b.cmd("LIM 3 70 200 0"), "ERR E10")
    expect("span below the 5 degree minimum", b.cmd("LIM 3 90 93 0"), "ERR E10")
    expect("bad cal flag", b.cmd("LIM 3 70 110 7"), "ERR E10")
    expect_unchanged("no rejected LIM changed anything",
                     b, "LIM", "LIM J3 MIN=70 MAX=110 CAL=0")
    expect("a legal LIM is accepted", b.cmd("LIM 3 60 120 0"), "OK LIM J3")
    expect_unchanged("the accepted LIM did apply", b, "LIM", "LIM J3 MIN=60 MAX=120")
    b.cmd("LIM 3 70 110 0")

    print("\n-- repeated and idempotent --")
    expect("repeating a LIM is accepted", b.cmd("LIM 3 70 110 0"), "OK LIM J3")
    expect("repeating it again is accepted", b.cmd("LIM 3 70 110 0"), "OK LIM J3")

    print("\n-- stop verbs, nothing enabled --")
    expect("bare STP is accepted with nothing enabled", b.cmd("STP"), "OK STP")
    expect("STP on a disabled joint", b.cmd("STP 3"), "ERR E6")
    expect("MOV on a disabled joint", b.cmd("MOV 3 90"), "ERR E6")
    expect("JOG on a disabled joint", b.cmd("JOG 3 1"), "ERR E6")
    expect("JOG with a bad direction", b.cmd("JOG 3 5"), "ERR")

    print("\n-- shoulder stays locked --")
    expect("shoulder refuses to enable while the mirror is unknown",
           b.cmd("ENA 1 90"), "ERR E13")


def motion_tests(b):
    print("\n-- BOUNDED MOTION (--motion-ok) --")
    b.cmd("LIM 3 70 110 0")
    expect("enable adopts the stated angle", b.cmd("ENA 3 90"), "OK ENA J3")
    expect("STP stops one joint", b.cmd("STP 3"), "OK STP J3")
    expect("LIM that still contains the joint is accepted",
           b.cmd("LIM 3 60 120 0"), "OK LIM J3")
    expect("LIM that would exclude the joint is refused",
           b.cmd("LIM 3 100 120 0"), "ERR E9")
    expect_unchanged("the refused LIM changed nothing",
                     b, "LIM", "LIM J3 MIN=60 MAX=120")
    b.cmd("MOV 3 119")
    b.cmd("LIM 3 60 95 0")
    print("     inspect TGT below: it must be 95 or lower, never 119")
    print("     " + " | ".join(b.cmd("STA")))
    expect("JOG is accepted on an enabled joint", b.cmd("JOG 3 1"), "OK JOG J3 DIR=1")
    print("     holding 1.2 s with no heartbeat -- the timeout should fire")
    time.sleep(1.2)
    expect("jog timeout announced itself", b.cmd("STA"), "OK STA")
    b.cmd("DIS 3")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--motion-ok", action="store_true",
                    help="enable bounded motion tests -- a servo WILL move")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        print("no serial port found")
        return 2

    if args.motion_ok:
        print("=" * 68)
        print(" --motion-ok: ONE JOINT WILL BE ENABLED AND MOVED.")
        print(" Horn off. Nothing in the path. Hand on the rocker switch.")
        print(" The rocker and the fuse are the emergency stop -- not this script.")
        print("=" * 68)
        time.sleep(3.0)

    print("port %s" % port)
    b = Board(port)
    try:
        non_motion_tests(b)
        if args.motion_ok:
            motion_tests(b)
    finally:
        b.cmd("DIS A")
        b.close()

    print("\n%d failure(s)" % len(FAILURES))
    return 1 if FAILURES else 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Run it and record the baseline**

```bash
python "C:\RobotArm\Software\tests\protocol_check.py" --port COM5
```

Expected today: the handshake, malformed-input, joint-id and repeat sections mostly PASS; every `JOG` case FAILS (no such verb, so `ERR E1` not `ERR E6`); `span below the 5 degree minimum` FAILS (no span rule yet); `STP 3` FAILS (`ERR E2`, wrong argument count). **Write the exact output into the baseline doc from Task 0** — this is what Tasks 2–5 are measured against.

- [ ] **Step 3: Commit**

```bash
cd /c/RobotArm
git add Software/tests/protocol_check.py
git commit -m "test: protocol harness, non-motion by default

Talks to the real board over the wire, because a 32 KB AVR cannot host a
test framework. Default mode enables nothing and moves nothing, so it is
safe with servo power on; --motion-ok adds bounded motion tests behind a
printed physical-safety acknowledgment.

Covers handshake, malformed input, missing and excess arguments, bad and
reserved joint ids, reversed/equal/out-of-range/too-narrow limits, trailing
garbage, overlong lines, repeats, and that a rejected command leaves state
untouched.

Committed red: the JOG, STP <j> and minimum-span cases fail against today's
firmware, which is what Tasks 2-5 exist to fix.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 2: Firmware — `STP <j>` aborts one joint

**Files:** Modify `factorylm_arm_controller.ino` (`doStp`, dispatch), `Documentation/SERIAL-PROTOCOL.md`

**Interfaces:** Consumes `jointArg`, `intArg`, `errJPre`, `okPre`, `okDone`, `badArgc`, `j[]`, `NJ`. Produces `STP` (all) and `STP <j>` (one) — Task 9 sends `STP <j>`.

- [ ] **Step 1: Confirm the harness fails here**

```bash
python "C:\RobotArm\Software\tests\protocol_check.py" --port COM5 2>&1 | grep "STP on a disabled joint"
```

Expected: `FAIL` — today `STP 3` returns `ERR E2`.

- [ ] **Step 2: Replace `doStp`**

```cpp
// STP aborts motion.  It is NOT an emergency stop: it cancels the remaining
// interpolation and holds the last COMMANDED value, with the channel still
// driven.  It does not remove power and it cannot know the shaft angle -- the
// rocker switch and the inline fuse are the emergency stop.
//
// Bare STP keeps its original meaning: abort every enabled joint.  STP <j>
// aborts one, so releasing one joystick cannot freeze a joint the operator is
// not touching.
static void doStp() {
  for (uint8_t i = 0; i < NJ; i++) if (j[i].en) { j[i].tgtC = j[i].setC; j[i].jogActive = false; }
  okDone();
}

static void doStpJoint(uint8_t i) {
  if (!j[i].en) { errJPre(F("E6"), i); Serial.println(); return; }
  j[i].tgtC = j[i].setC;
  j[i].jogActive = false;            // an operator stop also disarms the jog timer
  okPre();
  Serial.print(F(" J"));
  Serial.println(i);
}
```

`jogActive` is added to the joint record in Task 5. **Until Task 5 lands, omit both `jogActive` lines** — add them back as part of Task 5, Step 3.

**Correction C7 (Task 0):** the first draft used `jogMs = 0` as the "not jogging" sentinel. `millis()` returns exactly `0` once per ~49.7 days, so a jog armed on that tick would never time out. A separate `bool` costs 7 bytes of SRAM and removes the case entirely.

- [ ] **Step 3: Widen the dispatch** (replaces the single `STP` line found in Task 0, Step 2)

```cpp
  if (VIS('S','T','P')) {
    if (tokc == 0) { doStp(); return; }
    if (tokc != 1) { badArgc(); return; }
    if (!intArg(0, &a0)) return;
    if (!jointArg(a0, &id)) return;
    doStpJoint(id);
    return;
  }
```

- [ ] **Step 4: Compile**

```bash
"C:\Program Files\Arduino CLI\arduino-cli.exe" compile --fqbn arduino:avr:uno "C:\RobotArm\Software\factorylm_arm_controller"
```

Expected: `0 errors`. Record flash/SRAM.

- [ ] **Step 5: Upload, servo power OFF**

```bash
"C:\Program Files\Arduino CLI\arduino-cli.exe" upload -p COM5 --fqbn arduino:avr:uno --verify "C:\RobotArm\Software\factorylm_arm_controller"
```

- [ ] **Step 6: Re-run the harness**

```bash
python "C:\RobotArm\Software\tests\protocol_check.py" --port COM5
```

Expected: the two `STP` cases now PASS. `JOG` and span cases still FAIL.

- [ ] **Step 7: Update the protocol doc** — replace the `STP` verb-table row with:

```markdown
| `STP` | — | `OK STP` | Abort motion on every enabled joint; hold the last commanded value. Joints stay driven. **Not an emergency stop.** |
| `STP` | `<j>` | `OK STP J<j>` | Abort motion on one joint only. `E4` bad/reserved id, `E6` not enabled. |
```

And add, under the verb table:

> **`STP` is a motion abort, not an emergency stop.** It cancels remaining interpolation and
> holds the last *commanded* value. It does not remove power, does not detach, and cannot know
> the shaft angle. `EST` detaches and latches; the rocker switch and the inline fuse are the
> only emergency stop.

- [ ] **Step 8: Commit**

```bash
cd /c/RobotArm
git add Software/factorylm_arm_controller/factorylm_arm_controller.ino Documentation/SERIAL-PROTOCOL.md
git commit -m "feat(fw): STP takes an optional joint id

Bare STP keeps its exact meaning. STP <j> aborts one joint, so releasing one
joystick will not freeze a joint the operator is not touching.

Documents STP as a motion abort rather than an emergency stop: it cancels
interpolation and holds the last commanded value, with the channel still
driven. It cannot remove power and cannot know the shaft angle.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 3: Firmware — `LIM` becomes atomic, span-checked, and strict

**Files:** Modify `factorylm_arm_controller.ino` (`doLimSet`), `Documentation/SERIAL-PROTOCOL.md`

**Interfaces:** Produces a `LIM` that validates completely before writing anything. Task 4 adds the enabled-joint case on top.

- [ ] **Step 1: Confirm the harness fails**

```bash
python "C:\RobotArm\Software\tests\protocol_check.py" --port COM5 2>&1 | grep "span below"
```

Expected: `FAIL` — no minimum-span rule exists yet.

- [ ] **Step 2: Add the span constant** beside the other limit constants:

```cpp
// A soft-limit span narrower than this is refused.  Below about 5 degrees the
// envelope is too tight to jog inside usefully, and a slip of a handle would
// pin a joint against its own limits with no room to back off.
const uint8_t LIM_MIN_SPAN_DEG = 5;
```

- [ ] **Step 3: Rewrite `doLimSet`'s validation so nothing is written until every check passes**

```cpp
// LIM j min max cal -- ATOMIC.  Every check runs before ANY field is written,
// so a rejected LIM leaves the previous envelope exactly as it was.  There is
// no state in which min has been updated and max has not.
//
// Limits are enforced in LOGICAL joint space, before the centidegree
// calibration offset, and before shoulder mirroring.  The mirrored physical
// servo is derived from the accepted logical command, never commanded directly.
//
// CAL is set explicitly from the argument, never inferred: a file still holding
// the defaults must stay flagged uncalibrated.
//
// E10, not E5.  E5 is documented as "adopt angle outside this joint's MIN..MAX";
// a bad min/max pair or a bad cal flag is a different failure with a different
// remedy, and sharing the code made the console's plain-English message wrong.
static void doLimSet(uint8_t i, int32_t mn, int32_t mx, int32_t cal) {
  if (mn < 0 || mx > 180 || mn >= mx) {
    errJPre(F("E10"), i);
    Serial.print(F(" REQMIN="));  Serial.print(mn);
    Serial.print(F(" REQMAX="));  Serial.print(mx);
    Serial.println(F(" LIMIT=0..180 MIN<MAX"));
    return;
  }
  if ((mx - mn) < (int32_t)LIM_MIN_SPAN_DEG) {
    errJPre(F("E10"), i);
    Serial.print(F(" REQMIN="));  Serial.print(mn);
    Serial.print(F(" REQMAX="));  Serial.print(mx);
    Serial.print(F(" MINSPAN=")); Serial.println(LIM_MIN_SPAN_DEG);
    return;
  }
  if (cal != 0 && cal != 1) {
    errJPre(F("E10"), i);
    Serial.print(F(" REQCAL="));
    Serial.println(cal);
    return;
  }
  // ---- every check has passed; only now is anything written ----
  j[i].minD = (uint8_t)mn;
  j[i].maxD = (uint8_t)mx;
  j[i].cal  = (cal == 1);
```

Keep the existing `okPre()` reply block below unchanged.

- [ ] **Step 4: Verify the parser rejects trailing garbage — VERIFY ONLY, no change expected**

**Correction C4 (Task 0):** `parseInt` (`:280-293`) already rejects any non-digit, an empty field, a bare `-`, and overflow past `100000`. It is deliberately neither `atoi` (returns 0 on garbage, indistinguishable from a real 0) nor `sscanf` (~1.5 KB of AVR stdio). The harness case `MOV 3 90abc → ERR E3` should pass without any firmware change.

Confirm it does. If it somehow does not, stop and report — that would mean the file is not the one Task 0 read.

- [ ] **Step 5: Compile, upload with power OFF, re-run**

```bash
"C:\Program Files\Arduino CLI\arduino-cli.exe" compile --fqbn arduino:avr:uno "C:\RobotArm\Software\factorylm_arm_controller"
"C:\Program Files\Arduino CLI\arduino-cli.exe" upload -p COM5 --fqbn arduino:avr:uno --verify "C:\RobotArm\Software\factorylm_arm_controller"
python "C:\RobotArm\Software\tests\protocol_check.py" --port COM5
```

Expected: every limits-validation case PASSes, including `no rejected LIM changed anything`.

- [ ] **Step 6: Update the protocol doc** — add to the `LIM` prose:

> `LIM` is **atomic**: every argument is validated before any field is written, so a rejected
> `LIM` leaves the previous envelope untouched. The minimum accepted span is **5°**
> (`ERR E10 … MINSPAN=5`). Limits are enforced in logical joint space, before the centidegree
> conversion, before calibration offsets and before shoulder mirroring.

- [ ] **Step 7: Commit**

```bash
cd /c/RobotArm
git add Software/factorylm_arm_controller/factorylm_arm_controller.ino Documentation/SERIAL-PROTOCOL.md
git commit -m "feat(fw): LIM is atomic, span-checked and strict

Every argument is validated before any field is written, so a rejected LIM
leaves the previous envelope exactly as it was -- no half-applied envelope
with a new min and an old max. Adds a 5 degree minimum span: below that
there is no room to jog inside the envelope or back off a limit.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 4: Firmware — `LIM` on a driven joint, enforced in logical space

**Files:** Modify `factorylm_arm_controller.ino` (`doLimSet`), `Documentation/SERIAL-PROTOCOL.md`

**Interfaces:** Produces `LIM` accepted on an enabled joint when the new range still contains that joint's commanded value. Task 8 relies on it for live dragging.

- [ ] **Step 1: Confirm the motion harness fails**

```bash
python "C:\RobotArm\Software\tests\protocol_check.py" --port COM5 --motion-ok
```

Servo power OFF for this run — the joint enables and the firmware emits pulses, but with no servo connected nothing moves. Expected: `LIM that still contains the joint is accepted` FAILS with `ERR E9`.

- [ ] **Step 2: Add the enabled-joint branch** — insert immediately after the `cal` check in `doLimSet`, still before any write:

```cpp
  // On a DRIVEN joint the new range is accepted only if it still contains that
  // joint's own commanded value.  A range that would EXCLUDE it is refused,
  // because applying it would turn a limit edit into an unrequested move -- the
  // operator tidies a number and a loaded arm swings.  That is exactly what E9
  // was written to prevent; this narrows the refusal rather than removing it.
  //
  // Comparison happens in CENTIDEGREES, because setC/tgtC are centidegrees and
  // mn/mx are whole degrees.  Degrees -> centidegrees is *100, the same inline
  // conversion clampToLimits, enableJoint and doLimSet already use.
  int16_t loC = 0, hiC = 0;
  if (j[i].en) {
    loC = (int16_t)mn * 100;
    hiC = (int16_t)mx * 100;
    if (j[i].setC < loC || j[i].setC > hiC) {
      errJPre(F("E9"), i);
      Serial.println(F(" STATE=enabled"));
      return;
    }
  }
```

And after the three assignments, before `okPre()`:

```cpp
  // A pending target outside the new range is clamped INWARD.  Clamping can only
  // make a move shorter, so a limit edit can never create travel.
  if (j[i].en) {
    if (j[i].tgtC < loC) j[i].tgtC = loC;
    if (j[i].tgtC > hiC) j[i].tgtC = hiC;
  }
```

- [ ] **Step 3: Confirm the physical clamp after mirroring — VERIFY ONLY, no change expected**

**Correction C2 (Task 0):** this is already structural and must not be rebuilt.

- `writeJoint()` (`:415`) clamps `setC` via `clampToLimits()` **first**, then computes
  `mirrorC(setC)` for D5.
- `mirrorC()` (`:400`) clamps its own output to `[0, 18000]` centidegrees, using an `int32`
  intermediate so `18000 + 2*offset` cannot overflow.
- `enableJoint()` (`:470`) takes the same path.
- `doMir()` (`:664-677`) refuses an `INV` offset whose image of joint 1's whole `MIN..MAX` would
  fall outside `0..180`.

Confirm all four still hold after your change and say so in the commit. **Adding another clamp
here would be duplicate machinery, which is exactly the drift the firmware's "one clamp on the
write path" comment warns against.**

- [ ] **Step 4: Compile, upload with power OFF, re-run both modes**

```bash
"C:\Program Files\Arduino CLI\arduino-cli.exe" compile --fqbn arduino:avr:uno "C:\RobotArm\Software\factorylm_arm_controller"
"C:\Program Files\Arduino CLI\arduino-cli.exe" upload -p COM5 --fqbn arduino:avr:uno --verify "C:\RobotArm\Software\factorylm_arm_controller"
python "C:\RobotArm\Software\tests\protocol_check.py" --port COM5
python "C:\RobotArm\Software\tests\protocol_check.py" --port COM5 --motion-ok
```

Expected: the `LIM`-on-enabled cases PASS. Read the printed `STA` line by eye: after `LIM 3 60 95`, `TGT` must be `95` or lower, never `119`.

- [ ] **Step 5: Update the protocol doc** — replace the `E9` row with:

```markdown
| `E9` | `MODE` | `ERR E9 <VERB> JOINT=<j> STATE=enabled` | `MIR` while joint 1 is enabled, or a `LIM` whose new range would not contain that joint's own commanded value |
```

- [ ] **Step 6: Commit**

```bash
cd /c/RobotArm
git add Software/factorylm_arm_controller/factorylm_arm_controller.ino Documentation/SERIAL-PROTOCOL.md
git commit -m "feat(fw): accept LIM on a driven joint when it cannot exclude the joint

Live envelope editing needs LIM to work on a driven joint. E9 is narrowed,
not removed: a range that would EXCLUDE the joint's commanded value is still
refused, because applying it would turn a limit edit into an unrequested
move. A pending target outside the new range is clamped inward, and clamping
can only shorten a move, never create travel.

Comparison is done in centidegrees (deg * 100) -- setC is not a degree.
MIR on an enabled shoulder stays refused.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 5: Firmware — `JOG` and the command-age timeout

**Files:** Modify `factorylm_arm_controller.ino` (joint record, `doJog`, dispatch, main loop, `doStpJoint`), `Documentation/SERIAL-PROTOCOL.md`

**Interfaces:** Produces `JOG <j> <-1|0|1>` → `OK JOG J<j> DIR=<d>`, and `EVT JOGTIMEOUT J<j>`. Task 9 sends `JOG` on a 200 ms heartbeat.

**Why this exists:** a board that has heard nothing for two seconds cannot tell a steady hand from a dead USB cable. Without this, a dropped link during a hold leaves the joint walking to the edge of an envelope the operator may have just widened to `0–180`.

- [ ] **Step 1: Confirm the harness fails** — `JOG on a disabled joint` returns `ERR E1` today.

- [ ] **Step 2: Add the constants**

```cpp
// Jog heartbeat.  The console re-sends JOG every JOG_BEAT_MS while the joystick
// is held; four consecutive misses abort that joint's motion and hold.
//
// Derived from the MEASURED command path, not guessed (see the Task 0 baseline
// doc, section 9).  The console is a strict one-command-in-flight FIFO already
// carrying PNG every 250 ms and STA every 250 ms; adding JOG makes a three-way
// rotation, each serialised, each quantised by the bridge's 60 ms /rx poll, with
// a STA reply costing ~36 ms of transmit on its own.  Realistic rotation is
// 180-300 ms, so a 600 ms timeout would have left only 2-3 beats of margin in the
// GOOD case and a single queue hiccup could false-trip a jog still being held.
//
// 1000 ms is still 4x tighter than the existing 4000 ms WDG watchdog, and at the
// default 30 deg/s a 1000 ms overrun is 30 degrees -- bounded by the envelope in
// any case.
const uint16_t JOG_BEAT_MS    = 250;
const uint16_t JOG_TIMEOUT_MS = 1000;
```

- [ ] **Step 3: Add the jog fields to the joint record**

```cpp
  bool     jogActive;   // true only between JOG <j> +/-1 and whatever ends it
  uint32_t jogMs;       // millis() of the last refresh; meaningful only when jogActive
```

Initialise both in `setup()`'s joint loop (`jogActive = false; jogMs = 0;`). Then restore the two `jogActive = false;` lines in `doStp` and `doStpJoint` that Task 2 Step 2 deferred.

**Correction C7:** do **not** use `jogMs == 0` as the sentinel — `millis()` is exactly `0` once per ~49.7 days, and a jog armed on that tick would never time out. The separate flag costs 7 bytes of SRAM against 1643 free.

- [ ] **Step 4: Write `doJog`**

```cpp
// JOG j dir.  dir is -1, 0 or +1.  Sets the target to the envelope edge in that
// direction and ARMS a per-joint command-age timer.  dir 0 aborts the jog and
// holds, exactly like STP <j>.
//
// JOG is a separate verb from MOV on purpose: a finite move -- the target-angle
// box, a waypoint -- must run to completion and must never be cut short by a
// timeout it did not ask for.  Only JOG arms the timer; MOV clears it.
static void doJog(uint8_t i, int32_t dir) {
  if (!j[i].en) { errJPre(F("E6"), i); Serial.println(); return; }
  if (dir < -1 || dir > 1) {
    errJPre(F("E14"), i);
    Serial.print(F(" REQDIR="));
    Serial.println(dir);
    return;
  }
  if (dir == 0) {
    j[i].tgtC      = j[i].setC;
    j[i].jogActive = false;
  } else {
    /* Centidegrees: minD/maxD are whole degrees, tgtC is centidegrees. */
    j[i].tgtC      = (int16_t)(dir > 0 ? j[i].maxD : j[i].minD) * 100;
    j[i].jogActive = true;
    j[i].jogMs     = millis();
  }
  okPre();
  Serial.print(F(" J"));   Serial.print(i);
  Serial.print(F(" DIR=")); Serial.println(dir);
}
```

- [ ] **Step 5: Dispatch it** — beside the other joint-argument verbs:

```cpp
  if (VIS('J','O','G')) {
    if (tokc != 2) { badArgc(); return; }
    if (!intArg(0, &a0) || !intArg(1, &a1)) return;
    if (!jointArg(a0, &id)) return;
    doJog(id, a1);
    return;
  }
```

- [ ] **Step 6: Clear the flag wherever motion is otherwise decided**

Task 0 §13 enumerated every existing write to `tgtC`. The complete rule set:

| Event | `jogActive` |
|---|---|
| `JOG i ±1` | **set**, `jogMs = millis()` |
| `JOG i 0` | cleared |
| `STP` (bare) | cleared for **every** joint |
| `STP i` | cleared for joint `i` |
| `MOV i deg` | cleared for joint `i` — **a finite move must not inherit the timeout** |
| `SPD i dps` | untouched — speed changes mid-jog are normal |
| `LIM i …` accepted on a driven joint | untouched; the target is clamped and the jog continues inside the new envelope |
| `DIS i` / `DIS A` | cleared (via `disableJoint`) |
| `EST` / `!` / watchdog trip | cleared for every joint (via `estopAll` → `disableJoint`) |
| timeout fires | cleared for that joint |
| reset | cleared by `setup()` |

Put the clear in `disableJoint()` (`:444`) rather than in each caller — it is already the single
choke point for `DIS`, `EST` and the watchdog, and it already does `tgtC = setC` for exactly this
class of reason.

Then grep every write to `tgtC` and confirm each one either arms or clears the flag deliberately:

```bash
grep -n "tgtC =" Software/factorylm_arm_controller/factorylm_arm_controller.ino
```

- [ ] **Step 7: Enforce the timeout in the main loop**, beside the existing interpolator tick:

```cpp
  // Command-age timeout.  A jog that stops being refreshed aborts and HOLDS --
  // it does not detach and it does not latch.  Detaching would make a loaded arm
  // sag, which is worse than holding.  This is deliberately gentler than the
  // serial watchdog (WDG), which is the coarse net for a dead host.
  // Rollover-safe unsigned subtraction, the same idiom the interpolator tick and
  // the serial watchdog already use.  jogActive -- never "jogMs != 0" -- is what
  // says a jog is running.
  for (uint8_t i = 0; i < NJ; i++) {
    if (j[i].en && j[i].jogActive && (uint32_t)(now - j[i].jogMs) > JOG_TIMEOUT_MS) {
      j[i].tgtC      = j[i].setC;
      j[i].jogActive = false;
      Serial.print(F("EVT JOGTIMEOUT J"));
      Serial.println(i);
    }
  }
```

`now` is already computed at `:1071`; reuse it rather than calling `millis()` a second time.

- [ ] **Step 8: Compile, upload with power OFF, run both harness modes**

Expected: `JOG on a disabled joint` → `ERR E6`; `JOG with a bad direction` → `ERR E14`; in `--motion-ok`, `JOG 3 1` is accepted and an `EVT JOGTIMEOUT J3` line appears within ~1000 ms of the heartbeat stopping.

- [ ] **Step 9: Update the protocol doc**

Add to the verb table:

```markdown
| `JOG` | `<j> <-1\|0\|1>` | `OK JOG J<j> DIR=<d>` | Jog toward the envelope edge and arm the command-age timer. `0` aborts and holds. Must be refreshed every 250 ms. |
```

Add to the error table:

```markdown
| `E14` | `JOGDIR` | `ERR E14 JOG JOINT=<j> REQDIR=<d>` | jog direction outside -1..+1 |
```

Add to the events list:

```markdown
| `EVT JOGTIMEOUT J<j>` | a jog was not refreshed within 1000 ms; that joint's motion was aborted and it is holding its last commanded value. Not a latch, not a detach, not an emergency stop. |
```

And a prose block:

> **Jog heartbeat.** `JOG` arms a per-joint command-age timer. The host must re-send `JOG` every
> **250 ms** while the operator holds the control. Four consecutive misses (**1000 ms**) abort
> that joint's motion and hold the last commanded value, announced by `EVT JOGTIMEOUT`. `MOV`
> does **not** arm the timer — a finite move runs to completion. A timeout hold is
> distinguishable from an operator `STP` by the event line, and the host should show it
> differently: a hold nobody asked for is a symptom.
>
> This is deliberately gentler, and four times tighter, than the `WDG` serial watchdog: `WDG`
> detaches every joint and latches after 4000 ms of host silence, and a detached gravity-loaded
> arm sags. The jog timer holds instead, because a stalled joystick is not a dead host.

- [ ] **Step 10: Commit**

```bash
cd /c/RobotArm
git add Software/factorylm_arm_controller/factorylm_arm_controller.ino Documentation/SERIAL-PROTOCOL.md
git commit -m "feat(fw): JOG verb with a per-joint command-age timeout

A board that has heard nothing for two seconds cannot tell a steady hand
from a dead USB cable. Without this, a dropped link mid-hold leaves the
joint walking to the edge of an envelope the operator may have just widened
to 0-180.

JOG arms a 1000 ms timer refreshed by a 250 ms heartbeat. On expiry the joint
aborts motion and HOLDS its last commanded value, announced by EVT
JOGTIMEOUT -- it does not detach and does not latch, because detaching makes
a loaded arm sag. Deliberately gentler than the WDG serial watchdog, which
is the coarse net for a dead host.

JOG is separate from MOV so that finite motion cannot inherit the timeout.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 6: Harness — bounded motion coverage, and REVIEW CHECKPOINT 1

**Files:** Modify `Software/tests/protocol_check.py`

- [ ] **Step 1: Extend `motion_tests`** with the cases the firmware can now satisfy: `STP <j>` stops one joint while another keeps its target; a jog refreshed on a 200 ms beat does **not** time out; a jog abandoned does; `MOV` issued during a jog clears the timer and runs to completion without a timeout.

```python
    print("\n-- heartbeat keeps a jog alive --")
    b.cmd("ENA 3 90")
    b.cmd("JOG 3 1")
    for _ in range(6):
        time.sleep(0.2)
        b.cmd("JOG 3 1")
    expect("a refreshed jog did not time out", b.cmd("STA"), "OK STA")
    print("     no EVT JOGTIMEOUT should appear above")
    b.cmd("JOG 3 0")

    print("\n-- a finite move does not inherit the jog timeout --")
    b.cmd("MOV 3 95")
    time.sleep(1.0)
    expect("finite move survived past the jog timeout", b.cmd("STA"), "OK STA")
    print("     no EVT JOGTIMEOUT should appear above")
    b.cmd("DIS 3")
```

- [ ] **Step 2: Run both modes, servo power OFF** — expect `0 failure(s)`.

- [ ] **Step 3: REVIEW CHECKPOINT 1**

Read the whole firmware diff so far: `git diff main -- Software/factorylm_arm_controller/`. Check specifically — is every write to `tgtC` accounted for in the jog-timer logic? Is `doLimSet` genuinely atomic on every path? Does any new string say "emergency stop"? Is the flash/SRAM still inside budget? Fix findings before continuing.

- [ ] **Step 4: Commit**

```bash
cd /c/RobotArm
git add Software/tests/protocol_check.py
git commit -m "test: bounded motion coverage for STP <j>, jog heartbeat and timeout

Covers the two cases that matter most: a refreshed jog must not time out,
and a finite MOV must not inherit the jog timeout.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 7: Console — pure helpers, the self-test and its wrapper

**Files:** Modify `Software/arm-console/arm-console.html`; create `Software/tests/selftest.sh`

**Interfaces:** Produces `joySpeed(u, dpsMax)` → `{dir, dps}`; `envClampHandle(which, want, j)` → `{value, blocked}`; `mirrorImageOk(mn, mx, mode, off)` → `{ok, lo, hi}`; `fmtJog(id, dir)` / `fmtLim(id, mn, mx, cal)` → command strings; `JOY_DEAD` = `0.14`; `LIM_MIN_SPAN` = `5`.

- [ ] **Step 1: Write the self-test first**, before the closing `</script>`:

```javascript
/* ?selftest=1 runs the pure helpers against known values and writes exactly one
   terminal sentinel -- SELFTEST_PASS or SELFTEST_FAIL -- plus a line per
   failure. Runs from file:// with no board and no bridge. Verified by
   Software/tests/selftest.sh; Chrome's exit code is not evidence. */
function runSelfTest(){
  var fails = [];
  function eq(name, got, want){
    if (JSON.stringify(got) !== JSON.stringify(want)) {
      fails.push(name + "\n    expected " + JSON.stringify(want) + "\n    observed " + JSON.stringify(got));
    }
  }
  var jOn  = { min:70, max:110, set:90, en:true };
  var jOff = { min:70, max:110, set:90, en:false };

  eq("centre is dead",             joySpeed(0, 30),    { dir:0, dps:0 });
  eq("inside dead zone is dead",   joySpeed(0.10, 30), { dir:0, dps:0 });
  eq("just past dead zone crawls", joySpeed(0.15, 30), { dir:1, dps:1 });
  eq("full right is max dps",      joySpeed(1, 30),    { dir:1, dps:30 });
  eq("full left is max dps",       joySpeed(-1, 30),   { dir:-1, dps:30 });
  eq("dps ceiling is per joint",   joySpeed(1, 12),    { dir:1, dps:12 });

  eq("min handle free below joint", envClampHandle("min", 80, jOn),   { value:80,  blocked:false });
  eq("min handle blocked at joint", envClampHandle("min", 95, jOn),   { value:90,  blocked:true  });
  eq("max handle free above joint", envClampHandle("max", 130, jOn),  { value:130, blocked:false });
  eq("max handle blocked at joint", envClampHandle("max", 85, jOn),   { value:90,  blocked:true  });
  eq("min handle floors at 0",      envClampHandle("min", -20, jOn),  { value:0,   blocked:false });
  eq("max handle ceils at 180",     envClampHandle("max", 200, jOn),  { value:180, blocked:false });
  eq("disabled joint does not block", envClampHandle("min", 95, jOff),{ value:95,  blocked:false });

  eq("mirror legal at offset 0", mirrorImageOk(70, 110, "inverted", 0),  { ok:true,  lo:70,   hi:110 });
  eq("mirror illegal when wide", mirrorImageOk(10, 170, "inverted", 20), { ok:false, lo:-130, hi:30  });
  eq("mirror irrelevant if same",mirrorImageOk(0, 180, "same", 0),       { ok:true,  lo:0,    hi:180 });

  eq("jog command formatting",   fmtJog(3, 1),              "JOG 3 1");
  eq("jog stop formatting",      fmtJog(3, 0),              "JOG 3 0");
  eq("lim command formatting",   fmtLim(3, 60, 120, false), "LIM 3 60 120 0");
  eq("lim carries the cal flag", fmtLim(3, 60, 120, true),  "LIM 3 60 120 1");

  /* Malformed saved data must never reach the board. */
  eq("reversed saved row rejected", limRowOk({min:120, max:40}),  false);
  eq("too narrow saved row rejected", limRowOk({min:90, max:93}), false);
  eq("out of range saved row rejected", limRowOk({min:-5, max:110}), false);
  eq("sane saved row accepted",     limRowOk({min:60, max:120}),  true);

  /* Pointer cancellation must land in the same state as letting go. */
  var released = [];
  var fake = { releaseJoy: function(){ released.push("released"); } };
  simulatePointerCancel(fake);
  eq("pointer cancel releases the joystick", released, ["released"]);

  var out = document.createElement("pre");
  out.id = "selftest";
  out.textContent = fails.length
    ? ("SELFTEST_FAIL (" + fails.length + ")\n" + fails.join("\n"))
    : "SELFTEST_PASS";
  document.body.appendChild(out);
}
if (/[?&]selftest=1/.test(location.search)) runSelfTest();
```

- [ ] **Step 2: Write the wrapper** — `Software/tests/selftest.sh`:

```bash
#!/usr/bin/env bash
# Fails unless SELFTEST_PASS is present AND SELFTEST_FAIL is absent.
# Chrome's exit code is not evidence -- a page that throws before rendering
# still exits 0, which is exactly the failure this guards against.
set -u
CHROME="${CHROME:-/c/Program Files/Google/Chrome/Application/chrome.exe}"
PAGE="file:///C:/RobotArm/Software/arm-console/arm-console.html?selftest=1"

DOM="$("$CHROME" --headless=new --disable-gpu --virtual-time-budget=5000 --dump-dom "$PAGE" 2>/dev/null)"

if grep -q "SELFTEST_FAIL" <<<"$DOM"; then
  echo "SELFTEST FAILED:"
  sed -n '/SELFTEST_FAIL/,/<\/pre>/p' <<<"$DOM"
  exit 1
fi
if ! grep -q "SELFTEST_PASS" <<<"$DOM"; then
  echo "SELFTEST DID NOT RUN - no sentinel in the DOM."
  echo "The page probably threw before rendering. Open it in a browser."
  exit 1
fi
echo "SELFTEST_PASS"
```

`--virtual-time-budget=5000` is what makes it deterministic — the DOM is dumped after the page's timers settle, not at an arbitrary moment.

- [ ] **Step 3: Run it and watch it fail**

```bash
bash "C:\RobotArm\Software\tests\selftest.sh"
```

Expected: `SELFTEST DID NOT RUN` — none of the helpers exist yet.

- [ ] **Step 4: Write the helpers**, after `clampi`:

```javascript
var JOY_DEAD = 0.14;
var LIM_MIN_SPAN = 5;          /* must match LIM_MIN_SPAN_DEG in the firmware */

/* Joystick deflection -> speed. u is -1..+1. Inside the dead zone the joint is
   still, so a resting hand cannot creep it. Beyond it, deflection maps linearly
   from 1 deg/s to this joint's own max_deg_per_sec -- never past it. */
function joySpeed(u, dpsMax){
  var mag = Math.abs(u);
  if (mag <= JOY_DEAD) return { dir:0, dps:0 };
  var k = (mag - JOY_DEAD) / (1 - JOY_DEAD);
  return { dir: u > 0 ? 1 : -1, dps: clampi(Math.round(1 + k * (dpsMax - 1)), 1, dpsMax) };
}

/* An envelope handle may never be dragged past the joint's own commanded value
   while that joint is driven -- squeezing the envelope past the joint would make
   the arm move because someone tidied a number. The firmware refuses such a LIM
   with E9; this stops it ever being sent. */
function envClampHandle(which, want, j){
  var v = Math.round(want), blocked = false;
  if (which === "min") {
    v = clampi(v, 0, j.max - LIM_MIN_SPAN);
    if (j.en && v > j.set) { v = Math.floor(j.set); blocked = true; }
  } else {
    v = clampi(v, j.min + LIM_MIN_SPAN, 180);
    if (j.en && v < j.set) { v = Math.ceil(j.set); blocked = true; }
  }
  return { value: v, blocked: blocked };
}

/* Under mirror_mode=inverted the firmware drives the pair's second servo to
   (180 + 2*offset) - angle. If any part of joint 1's travel maps outside 0-180,
   two MG996Rs would fight through one link. */
function mirrorImageOk(mn, mx, mode, off){
  if (mode !== "inverted") return { ok:true, lo:mn, hi:mx };
  var lo = (180 + 2*off) - mx, hi = (180 + 2*off) - mn;
  return { ok: (lo >= 0 && hi <= 180), lo:lo, hi:hi };
}

/* Command formatting lives in one place so the self-test can check the exact
   bytes that go on the wire. */
function fmtJog(id, dir){ return "JOG " + id + " " + dir; }
function fmtLim(id, mn, mx, cal){ return "LIM " + id + " " + mn + " " + mx + " " + (cal ? 1 : 0); }

/* A saved CSV row is untrusted input. Same rules the firmware enforces, checked
   before anything is sent, so a hand-edited file cannot put the board in a state
   the console then misrenders. */
function limRowOk(r){
  var mn = Number(r.min), mx = Number(r.max);
  if (!isFinite(mn) || !isFinite(mx)) return false;
  if (mn < 0 || mx > 180) return false;
  if (mn >= mx) return false;
  return (mx - mn) >= LIM_MIN_SPAN;
}

/* Pointer cancellation, window blur and tab-hide all land here, so the
   self-test can exercise the same path the browser uses. */
function simulatePointerCancel(j){ if (j && j.releaseJoy) j.releaseJoy(); }
```

- [ ] **Step 5: Run the wrapper** — expect `SELFTEST_PASS`.

- [ ] **Step 6: Commit**

```bash
cd /c/RobotArm
git add Software/arm-console/arm-console.html Software/tests/selftest.sh
git commit -m "test(console): pure helpers with a headless self-test and a real gate

joySpeed, envClampHandle, mirrorImageOk, command formatters, and a validator
for untrusted saved CSV rows. ?selftest=1 runs them from file:// with no
board and no bridge, emitting exactly one sentinel.

selftest.sh fails unless SELFTEST_PASS is present and SELFTEST_FAIL absent.
Chrome's exit code is not evidence: a page that throws before rendering
still exits 0, which is the failure the wrapper exists to catch.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 8: Console — the envelope and the acknowledgment gate

**Files:** Modify `Software/arm-console/arm-console.html` — CSS, card markup in `buildCards()`, `j.dom`, `paintJoint`, `pushState`

**Interfaces:** Consumes `envClampHandle`, `fmtLim`, `limRowOk`, `send`, `notice`, `plainErr`, `paintJoint`. Produces `sendLim(id)` → Promise, `envState(id)` → `"DEFAULT"|"PENDING"|"ACKNOWLEDGED"`, `allEnvelopesAcked()` → Boolean.

- [ ] **Step 1: CSS** — append to the `<style>` block:

```css
.env{position:relative;height:40px;margin:10px 2px 0}
.envtrack{position:absolute;top:15px;left:0;right:0;height:6px;background:var(--panel-3);border-radius:3px}
.envband{position:absolute;top:15px;height:6px;background:var(--move);opacity:.55;border-radius:3px}
.envjoint{position:absolute;top:7px;width:2px;height:22px;background:var(--ok)}
.envh{position:absolute;top:5px;width:14px;height:26px;margin-left:-7px;background:var(--panel-2);
      border:2px solid var(--move);border-radius:4px;cursor:ew-resize;touch-action:none}
.envh.blocked{border-color:var(--fault);background:var(--fault-bg)}
.envnums{display:flex;justify-content:space-between;font-family:var(--mono);font-size:12px;
         color:var(--move);margin:2px 2px 0}
.envmsg{color:var(--warn);font-size:12px;min-height:16px;margin:2px 2px 0}
.ackpill{font-size:10px;letter-spacing:.6px;border-radius:999px;padding:2px 8px;margin-left:6px}
.ack-default{background:var(--grey-bg);border:1px solid var(--grey);color:var(--dim)}
.ack-pending{background:var(--warn-bg);border:1px solid var(--warn);color:var(--warn)}
.ack-ok{background:var(--ok-bg);border:1px solid var(--ok);color:var(--ok)}
```

- [ ] **Step 2: Markup** — insert before the joystick row in `buildCards()`:

```javascript
      '<div class="lbl">ENVELOPE &mdash; the soft stops' +
        '<span class="ackpill js-ack">DEFAULT</span></div>' +
      '<div class="env js-env">' +
        '<div class="envtrack"></div>' +
        '<div class="envband js-envband"></div>' +
        '<div class="envjoint js-envjoint"></div>' +
        '<div class="envh js-hmin"></div>' +
        '<div class="envh js-hmax"></div>' +
      '</div>' +
      '<div class="envnums"><span class="js-envmin">70&deg;</span><span class="js-envmax">110&deg;</span></div>' +
      '<div class="envmsg js-envmsg"></div>' +
```

- [ ] **Step 3: `j.dom` additions**

```javascript
      env:q(".js-env"), envBand:q(".js-envband"), envJoint:q(".js-envjoint"),
      hmin:q(".js-hmin"), hmax:q(".js-hmax"), ack:q(".js-ack"),
      envMin:q(".js-envmin"), envMax:q(".js-envmax"), envMsg:q(".js-envmsg"),
```

Add `envTouched:false, ackMin:null, ackMax:null, ackPending:false` to the joint record literal.

- [ ] **Step 4: The acknowledgment gate**

```javascript
/* The firmware keeps NOTHING across a reset, and opening the port resets it.
   The console must never assume its own in-memory limits are live on the board.
   Motion stays disabled until every joint's envelope has been acknowledged by
   the firmware -- a joystick driving against limits the board never confirmed is
   exactly the failure this gate prevents. */
function envState(id){
  var j = J[id];
  if (j.ackPending) return "PENDING";
  if (j.ackMin === j.min && j.ackMax === j.max) return "ACKNOWLEDGED";
  return "DEFAULT";
}

function allEnvelopesAcked(){
  for (var k = 0; k < JOINT_DEFS.length; k++) {
    if (envState(JOINT_DEFS[k].id) !== "ACKNOWLEDGED") return false;
  }
  return true;
}

/* One LIM for one joint, and the acknowledgment recorded. cal is passed through
   unchanged -- dragging is not a measurement, so it must never set the flag.
   Only the LOCK button does that. */
function sendLim(id){
  var j = J[id];
  if (connState !== "on") return Promise.resolve(false);
  if (!limRowOk({ min:j.min, max:j.max })) {
    notice("bad", "J" + id + ": " + j.min + "-" + j.max + "\u00b0 is not a legal envelope. " +
                  "Limits sit inside 0-180 and must be at least " + LIM_MIN_SPAN + "\u00b0 apart.");
    return Promise.resolve(false);
  }
  j.ackPending = true; paintJoint(id);
  return send(fmtLim(id, j.min, j.max, j.cal))
    .then(function(){
      j.ackMin = j.min; j.ackMax = j.max; j.ackPending = false;
      paintJoint(id); paintAll();
      return true;
    })
    .catch(function(e){
      j.ackPending = false; paintJoint(id); paintAll();
      notice("warn", plainErr(e.message, { id:id, name:J[id].def.name }));
      return false;
    });
}
```

In `pushState()`, clear `ackMin`/`ackMax` for every joint **before** the `LIM` loop, and route each `LIM` through `sendLim` so the acknowledgment is recorded. A reconnect must start from `DEFAULT`.

- [ ] **Step 5: Wire the drag**

```javascript
    var envDrag = null;
    function envDegFromX(clientX){
      var r = j.dom.env.getBoundingClientRect();
      return (clientX - r.left) / r.width * 180;
    }
    function envMove(e){
      var res = envClampHandle(envDrag, envDegFromX(e.clientX), j);
      if (d.id === 1) {
        var chk = mirrorImageOk(envDrag === "min" ? res.value : j.min,
                                envDrag === "max" ? res.value : j.max,
                                mirrorMode, mirrorOffset);
        if (!chk.ok) {
          j.dom.envMsg.textContent = "That range would drive the shoulder's second servo to " +
            chk.lo + "\u2013" + chk.hi + "\u00b0, outside 0\u2013180. Two servos would fight through one link.";
          return;
        }
      }
      if (envDrag === "min") j.min = res.value; else j.max = res.value;
      j.dom[envDrag === "min" ? "hmin" : "hmax"].classList.toggle("blocked", res.blocked);
      j.dom.envMsg.textContent = res.blocked
        ? "The joint is here. Disable it to bring the envelope in further."
        : "";
      paintJoint(d.id);
    }
    function envUp(){
      window.removeEventListener("pointermove", envMove);
      window.removeEventListener("pointerup", envUp);
      window.removeEventListener("pointercancel", envUp);
      j.dom.hmin.classList.remove("blocked");
      j.dom.hmax.classList.remove("blocked");
      j.dom.envMsg.textContent = "";
      envDrag = null;
      j.envTouched = true;
      sendLim(d.id);
    }
    function envDown(which){
      return function(e){
        e.preventDefault();
        envDrag = which;
        window.addEventListener("pointermove", envMove);
        window.addEventListener("pointerup", envUp);
        window.addEventListener("pointercancel", envUp);
      };
    }
    j.dom.hmin.addEventListener("pointerdown", envDown("min"));
    j.dom.hmax.addEventListener("pointerdown", envDown("max"));
```

- [ ] **Step 6: Paint** — add to `paintJoint`:

```javascript
  if (D.env) {
    var a = j.min / 180 * 100, b = j.max / 180 * 100;
    D.hmin.style.left = a + "%";
    D.hmax.style.left = b + "%";
    D.envBand.style.left = a + "%";
    D.envBand.style.width = (b - a) + "%";
    D.envJoint.style.left = (clampi(j.set, 0, 180) / 180 * 100) + "%";
    D.envJoint.style.display = j.en ? "block" : "none";
    D.envMin.textContent = j.min + "\u00b0";
    D.envMax.textContent = j.max + "\u00b0";
    var stt = envState(id);
    D.ack.textContent = stt;
    D.ack.className = "ackpill js-ack " +
      (stt === "ACKNOWLEDGED" ? "ack-ok" : stt === "PENDING" ? "ack-pending" : "ack-default");
  }
```

**Correction C10 (Task 0):** an earlier draft invented a `d0(j)` helper. `paintJoint(id)` (`:1527`) already takes `id` as its parameter and opens with `var j = J[id], d = j.def, D = j.dom;`. Use `id` directly.

And gate motion in `paintAll`:

```javascript
  var gate = allEnvelopesAcked();
  JOINT_DEFS.forEach(function(dd){
    var jj = J[dd.id];
    if (jj.dom.enable) jj.dom.enable.disabled = !gate;
    if (jj.dom.joy)    jj.dom.joy.style.opacity = gate ? "1" : "0.4";
  });
  if (!gate) {
    notice("warn", "Motion is disabled until every joint's envelope has been acknowledged by the controller.", 6000);
  }
```

- [ ] **Step 7: Verify** — `bash Software/tests/selftest.sh` still `SELFTEST_PASS`. Then bridge up, servo power OFF: on connect every card should go `DEFAULT` → `PENDING` → `ACKNOWLEDGED`, and ENABLE should be disabled until the last one lands. Drag J3's handles and confirm the pill returns to `ACKNOWLEDGED` after each release.

- [ ] **Step 8: Commit**

```bash
cd /c/RobotArm
git add Software/arm-console/arm-console.html
git commit -m "feat(console): draggable envelope with an acknowledgment gate

Two handles set the soft stops and push one LIM on release. While the joint
is driven a handle stops dead at its commanded value rather than squeezing
past it, so a drag can never cause motion. Dragging never touches the
calibrated flag.

Motion stays disabled until every envelope is ACKNOWLEDGED by the firmware.
Opening the port resets the board and the firmware keeps nothing, so the
console must never assume its own limits are live -- a joystick driving
against limits the board never confirmed is the failure this prevents.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 9: Console — the joystick, with the heartbeat

**Files:** Modify `Software/arm-console/arm-console.html` — CSS, markup, `j.dom`, the slider listeners, `paintJoint`

**Interfaces:** Consumes `joySpeed`, `fmtJog`, `JOY_DEAD`, `allEnvelopesAcked`. Produces `joyRelease(id)`, `joyReleaseAll()`.

- [ ] **Step 1: CSS**

```css
.joy{position:relative;height:44px;margin:10px 2px 0;touch-action:none}
.joytrack{position:absolute;top:17px;left:0;right:0;height:8px;background:var(--panel-3);border-radius:4px}
.joydead{position:absolute;top:17px;height:8px;background:var(--line);border-radius:4px}
.joyfill{position:absolute;top:17px;height:8px;background:var(--move);border-radius:4px}
.joyh{position:absolute;top:7px;left:50%;width:34px;height:28px;margin-left:-17px;background:var(--panel-2);
      border:2px solid var(--dim);border-radius:6px;cursor:grab}
.joyh.live{border-color:var(--move);background:var(--move-bg);cursor:grabbing}
```

- [ ] **Step 2: Replace the position-slider markup** with:

```javascript
      '<div class="lbl">JOYSTICK &mdash; hold to move, let go to stop</div>' +
      '<div class="joy js-joy">' +
        '<div class="joytrack"></div>' +
        '<div class="joydead js-joydead"></div>' +
        '<div class="joyfill js-joyfill"></div>' +
        '<div class="joyh js-joyh"></div>' +
      '</div>' +
      '<div class="scale"><span class="js-lo">70&deg;</span><span class="js-hi">110&deg;</span></div>' +
```

Keep `js-lo` / `js-hi` — `paintJoint` already writes them.

- [ ] **Step 3: `j.dom`** — remove `slider:q(".js-slider"),`, add:

```javascript
      joy:q(".js-joy"), joyH:q(".js-joyh"), joyFill:q(".js-joyfill"), joyDead:q(".js-joydead"),
```

- [ ] **Step 4: Replace the five slider listeners**

```javascript
    var joyHeld = false, joyLastDps = 0, joyLastDir = 0, joyBeat = null;
    var JOY_BEAT_MS = 250;      /* must match the firmware's JOG_BEAT_MS */

    function joyU(clientX){
      var r = j.dom.joy.getBoundingClientRect();
      return clampi(((clientX - r.left) / r.width - 0.5) * 2, -1, 1);
    }
    function joyApply(u){
      var s = joySpeed(u, j.dps);
      j.dom.joyH.style.left = (50 + u * 50) + "%";
      if (s.dps) {
        j.dom.joyFill.style.left  = (u > 0 ? 50 : 50 + u * 50) + "%";
        j.dom.joyFill.style.width = (Math.abs(u) * 50) + "%";
      } else {
        j.dom.joyFill.style.width = "0%";
      }
      if (connState !== "on" || !j.en) return;
      if (s.dps && s.dps !== joyLastDps) { joyLastDps = s.dps; send("SPD " + d.id + " " + s.dps); }
      if (s.dir !== joyLastDir) { joyLastDir = s.dir; send(fmtJog(d.id, s.dir)); }
    }
    /* The heartbeat is not optional. The firmware aborts a jog it has not heard
       about for 1000 ms, because a board that has heard nothing cannot tell a
       steady hand from a dead cable.

       Correction C9 (Task 0): the outbox is a strict one-in-flight FIFO already
       carrying PNG and STA every 250 ms. If the queue backs up, an un-coalesced
       heartbeat ACCUMULATES and then REPLAYS -- stale MOTION commands, which is
       far worse than a stale PNG. So the beat is skipped whenever anything is
       still queued: a backed-up queue IS a stalled host, which is precisely what
       the firmware timeout exists to catch. Letting it fire is the honest
       outcome, not a bug to paper over. */
    function joyBeatTick(){
      if (!joyHeld || !joyLastDir) return;
      if (outboxDepth() > 0) return;
      if (connState === "on" && j.en) send(fmtJog(d.id, joyLastDir));
    }
    function joyDown(e){
      e.preventDefault();
      if (!allEnvelopesAcked()) { notice("warn", "Wait for every envelope to be acknowledged."); return; }
      if (!j.en) { notice("warn", "Enable J" + d.id + " before moving it."); return; }
      joyHeld = true; joyLastDps = 0; joyLastDir = 0;
      j.dom.joyH.classList.add("live");
      window.addEventListener("pointermove", joyMove);
      window.addEventListener("pointerup", joyReleaseLocal);
      window.addEventListener("pointercancel", joyReleaseLocal);
      joyBeat = setInterval(joyBeatTick, JOY_BEAT_MS);
      joyApply(joyU(e.clientX));
    }
    function joyMove(e){ if (joyHeld) joyApply(joyU(e.clientX)); }
    function joyReleaseLocal(){
      if (!joyHeld) return;
      joyHeld = false;
      if (joyBeat) { clearInterval(joyBeat); joyBeat = null; }
      window.removeEventListener("pointermove", joyMove);
      window.removeEventListener("pointerup", joyReleaseLocal);
      window.removeEventListener("pointercancel", joyReleaseLocal);
      j.dom.joyH.classList.remove("live");
      j.dom.joyH.style.left = "50%";
      j.dom.joyFill.style.width = "0%";
      joyLastDps = 0; joyLastDir = 0;
      if (connState === "on" && j.en) send(fmtJog(d.id, 0));
    }
    j.releaseJoy = joyReleaseLocal;
    j.dom.joy.addEventListener("pointerdown", joyDown);
    j.dom.joyDead.style.left  = (50 - JOY_DEAD * 50) + "%";
    j.dom.joyDead.style.width = (JOY_DEAD * 100) + "%";
```

- [ ] **Step 4b: Expose the outbox depth**

`outbox` is a module-level array in the transport section. Add beside `trimOutbox()`:

```javascript
/* Read-only view of the queue depth, so the jog heartbeat can decline to add a
   motion command to a backlog. */
function outboxDepth(){ return outbox.length; }
```

Also extend `trimOutbox()`'s coalescing comment to name `JOG`: it collapses `PNG` and `STA` only,
and `JOG` must never reach it because Step 4 skips the beat while anything is queued. **If a
future change makes `JOG` reachable there, it must coalesce newest-per-joint — replaying a stale
motion command is a different class of bug from replaying a stale ping.**

- [ ] **Step 5: Global dead-man**, near `sendHold()`:

```javascript
/* Letting go, losing the pointer, losing the window, the tab going to the
   background, and the transport failing are all the same event: stop. The
   firmware's jog timeout is the backstop for the cases the browser never gets
   to report at all. */
function joyRelease(id){ if (J[id] && J[id].releaseJoy) J[id].releaseJoy(); }
function joyReleaseAll(){ JOINT_DEFS.forEach(function(dd){ joyRelease(dd.id); }); }
window.addEventListener("blur", joyReleaseAll);
document.addEventListener("visibilitychange", function(){ if (document.hidden) joyReleaseAll(); });
```

Also call `joyReleaseAll()` wherever the console already handles a dropped connection.

- [ ] **Step 6: Render a jog timeout distinctly**

Wherever `EVT` lines are handled, add:

```javascript
  if (/^EVT JOGTIMEOUT J(\d+)/.test(line)) {
    var jid = line.match(/^EVT JOGTIMEOUT J(\d+)/)[1];
    joyRelease(Number(jid));
    notice("bad", "J" + jid + " stopped on its own: the controller did not hear from this page " +
                  "for a second and aborted the move. The joint is holding, still powered. " +
                  "Check the USB cable before jogging again.", 12000);
  }
```

A hold nobody asked for is a symptom, and must not look like a normal stop.

- [ ] **Step 7: Remove the dead slider line in `paintJoint`** (`if (!j.dragging) D.slider.value = ...`). Leave `j.wantMove` — Task 10 refills it and `:1167` still drains it.

- [ ] **Step 8: Verify** — `selftest.sh` green. Then, servo power OFF, bridge up, J3 enabled: hold the joystick and confirm `JOG 3 1` repeats about five times a second and no `EVT JOGTIMEOUT` appears. Release, confirm `JOG 3 0`. Hold, then pull the USB — confirm the firmware emits `EVT JOGTIMEOUT` on reconnect-free inspection via the harness.

- [ ] **Step 9: Commit**

```bash
cd /c/RobotArm
git add Software/arm-console/arm-console.html
git commit -m "feat(console): spring-back joystick on a 200 ms JOG heartbeat

Deflection sets speed, capped at the joint's own max_deg_per_sec. The
heartbeat is mandatory: the firmware aborts a jog it has not heard about for
1000 ms, because it cannot distinguish a steady hand from a dead cable.

Pointer loss, window blur, tab hide and transport failure are all treated as
letting go. EVT JOGTIMEOUT is rendered as a fault rather than a normal stop
-- a hold nobody asked for is a symptom.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 10: Console — `TARGET ANGLE`

**Files:** Modify `Software/arm-console/arm-console.html`

**Resolved by review:** keep it. Calibration and waypoint testing both need repeatable commanded values.

- [ ] **Step 1: Markup** — after the scale row:

```javascript
      '<div class="gotorow">TARGET ANGLE <input type="number" class="js-target" step="1">' +
        '<button class="btn small js-send">SEND</button>' +
        '<span class="small js-targetack"></span></div>' +
```

- [ ] **Step 2: `j.dom`:** `target:q(".js-target"), sendBtn:q(".js-send"), targetAck:q(".js-targetack"),`

- [ ] **Step 3: Wire it — commit-only, never while typing**

```javascript
    /* Sends on Enter or the button, never on input. A box that transmits while
       you type turns a four-keystroke number into four moves, the first three
       of them wrong. */
    function sendTarget(){
      if (!allEnvelopesAcked()) { notice("warn", "Wait for every envelope to be acknowledged."); return; }
      if (!j.en) { notice("warn", "Enable J" + d.id + " first."); return; }
      var raw = Number(j.dom.target.value);
      if (!isFinite(raw)) { j.dom.targetAck.textContent = "not a number"; return; }
      var v = clampi(Math.round(raw), j.min, j.max);
      j.dom.target.value = v;
      j.dom.targetAck.textContent = "sending " + v + "\u00b0\u2026";
      send("MOV " + d.id + " " + v)
        .then(function(reply){
          /* Show what the controller ACCEPTED, not what was typed. */
          var m = String(reply).match(/SET=(-?\d+)/);
          j.dom.targetAck.textContent = "accepted " + (m ? m[1] : v) + "\u00b0";
        })
        .catch(function(e){
          j.dom.targetAck.textContent = "refused";
          notice("warn", plainErr(e.message, d));
        });
    }
    j.dom.sendBtn.onclick = sendTarget;
    j.dom.target.addEventListener("keydown", function(e){ if (e.key === "Enter") sendTarget(); });
```

- [ ] **Step 4: Verify** — servo power OFF, J3 enabled, envelope 60–120: type `105`, Enter, confirm `accepted 105°` and `STA` shows `TGT=105`. Type `500`, Enter, confirm the box rewrites to `120` and the acknowledgment shows what the board accepted. Type `9`, `9`, `9` slowly and confirm **nothing is sent** until Enter.

- [ ] **Step 5: Commit**

```bash
cd /c/RobotArm
git add Software/arm-console/arm-console.html
git commit -m "feat(console): TARGET ANGLE box for exact commanded values

The joystick replaced the position slider, removing the only way to command
a precise angle -- which calibration and waypoint testing both need.

Sends on Enter or the button only, never while typing: a box that transmits
per keystroke turns a four-digit number into four moves, three of them
wrong. Clamped to the envelope before sending, and it displays what the
controller accepted rather than what was typed.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 11: Console — `LOCK THIS AXIS` records accepted limits

**Files:** Modify `Software/arm-console/arm-console.html`, `Calibration_Notes/calibration-log.csv`

**Resolved by review:** LOCK records the **accepted commanded soft limits** — the handle values the firmware acknowledged. It does not claim mechanical extremes. Nothing here can observe those.

- [ ] **Step 1: Extend the calibration log's header** so the new fields have somewhere to live, appended to the existing columns rather than reordering them:

```
accepted_min_deg,accepted_max_deg,accepted_home_deg,firmware_ack,fw_version,console_version,lock_note
```

Add a comment line above the header:

```
# Columns are APPEND-ONLY. External-measurement fields will be added beside the
# commanded ones later; nothing may assume this row shape is final.
```

- [ ] **Step 2: Markup** — inside `cardactions`, after DISABLE:

```javascript
        '<button class="btn small js-lock" disabled>LOCK THIS AXIS</button>' +
```

`j.dom`: `lock:q(".js-lock"),`

- [ ] **Step 3: Implement**

```javascript
/* The ONLY code path allowed to set the calibrated flag. Nothing infers it.
   The amber UNCALIBRATED badge is the only warning that a joint's range is a
   placeholder, so a system that could clear it on its own would be lying.

   What this records is the ACCEPTED COMMANDED soft limits -- the handle values
   the firmware acknowledged. It is NOT a claim about the joint's mechanical
   extremes; nothing in this system can observe those. */
function lockAxis(id){
  var j = J[id];
  if (!j.en)         { notice("warn", "Enable J" + id + " and settle it where you want its centre first."); return; }
  if (!j.envTouched) { notice("warn", "Move J" + id + "'s envelope handles first. Locking the placeholder would record a range nobody chose."); return; }

  var prevCal = j.cal;
  j.cal  = true;
  j.home = Math.round(j.set);
  sendLim(id).then(function(ok){
    if (!ok) { j.cal = prevCal; paintJoint(id); return; }
    notice("ok", "J" + id + " " + j.def.name + " locked at " + j.min + "\u2013" + j.max +
                 "\u00b0, centre " + j.home + "\u00b0 \u2014 accepted by the controller. " +
                 "Save the limits file to keep it past a reset.", 9000);
    downloadCalibrationRow(id);
    paintJoint(id);
  });
}

/* Correction C11: p2() is a NESTED function inside another handler (:2025), not
   a global, so it cannot be called from here. Two digits, inline. */
function pad2(n){ return (n < 10 ? "0" : "") + n; }

function downloadCalibrationRow(id){
  var j = J[id], d = j.def, now = new Date();
  var stamp = now.getFullYear() + "-" + pad2(now.getMonth()+1) + "-" + pad2(now.getDate()) +
              " " + pad2(now.getHours()) + ":" + pad2(now.getMinutes());
  var row = [stamp, id, d.name, d.pins,
             j.min, j.max, j.home,
             '"OK LIM J' + id + ' MIN=' + j.min + ' MAX=' + j.max + ' CAL=1"',
             fwVersion || "unknown",
             CONSOLE_VERSION,
             '"accepted commanded soft limits - not mechanical extremes"'].join(",");
  var url = URL.createObjectURL(new Blob([row + "\n"], { type:"text/csv" }));
  var a = document.createElement("a");
  a.href = url;
  a.download = "calibration-row-J" + id + ".csv";
  a.click();
  URL.revokeObjectURL(url);
}
```

**Correction C12 (Task 0): `sysInfo` does not exist, and the firmware version is currently thrown away.** The handshake parses the `VER` reply into a local `v` and checks `v.NAME` against `EXPECT_NAME` only (`:1040`, `:1087`); `v.FW` is discarded. `sys` (`:555`) holds `{es, wd, mir, uncal}` and no version.

So this task must first **capture it**. Add near the other module-level state:

```javascript
var fwVersion = "";                    /* FW= from the VER handshake; "" until connected */
var CONSOLE_VERSION = "1.1.0";         /* bump with the console, recorded in every lock row */
```

and in **both** places that validate the handshake, immediately after the `NAME` check passes:

```javascript
      fwVersion = v.FW || "";
```

Clear it wherever the connection is torn down, next to the existing `staFreshMs = 0;` resets, so a stale version can never be written into a calibration row after a disconnect.

- [ ] **Step 4: SAVE LIMITS FILE**, beside `LOAD LIMITS FILE`:

```javascript
      '<button class="btn small" id="saveLim">SAVE LIMITS FILE</button>' +
```

```javascript
/* Writes the current in-memory limits back out in the column order the loader
   requires, so the file round-trips. The firmware keeps nothing across a reset;
   this file is the only place an accepted envelope survives. */
function downloadLimitsCsv(){
  var lines = [LIM_COLS.join(",")];
  JOINT_DEFS.forEach(function(dd){
    var jj = J[dd.id];
    lines.push([dd.id, dd.name, dd.pins, jj.min, jj.max, jj.home, jj.dps,
                jj.cal ? "yes" : "no", dd.id === 1 ? mirrorMode : "",
                (jj.notes || "").replace(/,/g, ";"), dd.id === 1 ? mirrorOffset : 0].join(","));
  });
  var url = URL.createObjectURL(new Blob([lines.join("\n") + "\n"], { type:"text/csv" }));
  var a = document.createElement("a");
  a.href = url; a.download = "joint-limits.csv"; a.click();
  URL.revokeObjectURL(url);
}
el("saveLim").onclick = downloadLimitsCsv;
```

- [ ] **Step 5: Paint the button and badge** — in `paintJoint`:

```javascript
  if (D.lock) D.lock.disabled = !(j.en && j.envTouched && envState(id) === "ACKNOWLEDGED");
  if (D.cal) {
    D.cal.textContent = j.cal ? "MEASURED" : "UNCALIBRATED \u2014 LIMITS ARE A PLACEHOLDER";
    D.cal.className = "pill " + (j.cal ? "ok" : "uncal") + " js-cal";
  }
```

- [ ] **Step 6: Verify the round trip** — servo power OFF: enable J3, drag to 60–130, LOCK, SAVE LIMITS FILE. Replace `Software/arm-console/joint-limits.csv` with the download, reconnect (which resets the board), and confirm J3 returns 60–130 `ACKNOWLEDGED` with a green badge and `LIM` on the board reporting `MIN=60 MAX=130 CAL=1`. Confirm the calibration row says *accepted commanded soft limits*.

- [ ] **Step 7: Commit**

```bash
cd /c/RobotArm
git add Software/arm-console/arm-console.html Calibration_Notes/calibration-log.csv
git commit -m "feat(console): LOCK THIS AXIS records accepted commanded soft limits

The only code path permitted to set the calibrated flag. It sends LIM with
cal=1, WAITS for the acknowledgment, and only then records the range, the
settled centre, the firmware's literal ack, and both version strings.

What it records is explicitly the accepted commanded soft limits, not the
joint's mechanical extremes -- nothing in this system can observe those. The
calibration log's columns are marked append-only so external-measurement
fields can be added beside the commanded ones later.

Disabled until the operator has moved that joint's handles and the envelope
is acknowledged, so the placeholder can never be locked in as if it were
chosen.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 12: Console — share the shoulder check, and REVIEW CHECKPOINT 2

**Files:** Modify `Software/arm-console/arm-console.html`

- [ ] **Step 1: Replace the inline mirror arithmetic in `loadLimitsCsv`** with a call to `mirrorImageOk` (Task 7), keeping the existing error text. The drag path (Task 8) already calls it; this removes the duplicate so the two cannot drift.

- [ ] **Step 2: Route file loading through `limRowOk`** so a hand-edited CSV cannot stage a row the firmware will reject.

- [ ] **Step 3: `bash Software/tests/selftest.sh`** — `SELFTEST_PASS`.

- [ ] **Step 4: REVIEW CHECKPOINT 2**

Read the whole console diff: `git diff main -- Software/arm-console/`. Check: is there any remaining path that sends motion before `allEnvelopesAcked()`? Does any UI string use a banned word from Global Constraints? Is `j.cal` written anywhere except `lockAxis` and the CSV loader? Does every joystick exit path clear the heartbeat interval? Fix findings before continuing.

- [ ] **Step 5: Commit**

```bash
cd /c/RobotArm
git add Software/arm-console/arm-console.html
git commit -m "fix(console): one shoulder mirror check and one row validator

The mirror arithmetic lived inside loadLimitsCsv, so dragging J1's envelope
could stage a range the loader would have refused. Both paths now share
mirrorImageOk, and file loading runs the same limRowOk validation the drag
path uses.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 13: Bench verification, then the PR

**Files:** Create `Documentation/2026-08-04-envelope-joystick-bench-log.md`

- [ ] **Step 1: Both harnesses, servo power OFF**

```bash
python "C:\RobotArm\Software\tests\protocol_check.py" --port COM5
python "C:\RobotArm\Software\tests\protocol_check.py" --port COM5 --motion-ok
bash "C:\RobotArm\Software\tests\selftest.sh"
```

Expected: `0 failure(s)` twice and `SELFTEST_PASS`.

- [ ] **Step 2: Servo power on, no load, clear workspace.** Confirm 4.9–5.1 V at the servo end before connecting anything.

- [ ] **Step 3: One low-risk joint — wrist roll, D10.** The only servo whose type is documented and the lightest loaded. Horn off, flag taped to the spline, hand on the rocker. Small increments only.

- [ ] **Step 4: Verify the four behaviours that matter** — release stops it; the jog timeout fires and holds when the cable is pulled mid-hold; a `LIM` that would exclude the joint is refused; a narrowing `LIM` clamps a pending target. Record the commands and replies verbatim.

- [ ] **Step 5: That joint at each accepted boundary** — jog to the envelope min, then the max, confirming it stops and holds at each.

- [ ] **Step 6: Reconnect and reset** — close the tab, reopen, confirm every card starts `DEFAULT`, motion is gated until all six are `ACKNOWLEDGED`, and nothing was assumed from the previous session.

- [ ] **Step 7: REVIEW CHECKPOINT 3, then the shoulder** — arm supported, tiny moves, shoulder pair last. Record the mapping evidence: which physical servo moved for a given logical command, and whether the mirrored one tracked correctly.

- [ ] **Step 8: Write the bench log** — build output, self-test output, harness output, commands and replies, final accepted limits per joint, reset/reconnect evidence, shoulder mapping evidence, and every deviation from the design with the reason.

- [ ] **Step 9: Commit, push, PR — only with explicit approval**

```bash
cd /c/RobotArm
git remote -v                     # confirm emre-kalem-robot-arm before anything else
git add Documentation/2026-08-04-envelope-joystick-bench-log.md
git commit -m "docs: bench log for the envelope + joystick work

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
git push -u origin feat/envelope-joystick
git log origin/feat/envelope-joystick --oneline -5
gh pr create --base main --head feat/envelope-joystick --title "feat: draggable envelope, per-joint joystick, lock-an-axis"
```

The PR body must state: flash/SRAM before and after, both harness results, self-test result, which checks were done unpowered versus powered, per-joint bench results, unresolved risks, and every deviation from the design.

- [ ] **Step 10: Final report** — commits created; files changed; design corrections made; exact commands run and their results; firmware build evidence; self-test evidence; bench results by joint; unresolved risks; deviations and why; branch status; and one of **`READY FOR DRAFT PR`**, **`NEEDS FIXES`**, or **`BLOCKED`**.

---

## Self-Review

**Spec coverage.** §3 vocabulary → Global Constraints, enforced at both review checkpoints. §5 card → Tasks 8–11. §6 envelope → Task 8; §6a logical enforcement → Task 4 Steps 2–3; §6b atomicity → Task 3; §6c narrowing → Task 4. §7 joystick → Task 9; §7a `JOG` → Task 5; §7b timeout semantics → Task 5 Steps 7, 9 and Task 9 Step 6; §7c dead-man → Task 9 Step 5; §7d target angle → Task 10. §8 lock → Task 11; §8a claim wording and schema → Task 11 Steps 1, 3. §9 ack gate → Task 8 Step 4. §10 invariants → Global Constraints. §11a harness → Tasks 1, 6; §11b self-test → Task 7; §11c bench sequence → Task 13. §12 deferred → no task, correctly. §14 — no open questions remain.

**Placeholder scan.** No TBDs. Every code step carries real code. **Task 0 has run** — all three of the previously-deferred unknowns are now resolved against source, and the four invented identifiers it exposed (`degToCmd`, `d0(j)`, global `p2()`, `sysInfo.fw`) are corrected in place rather than left as instructions to figure out later. Every remaining mention of `degToCmd` in this document is either a Task 0 step that went looking for it or a correction note recording that it does not exist.

**Type consistency.** `joySpeed` → `{dir, dps}` in Tasks 7, 9. `envClampHandle` → `{value, blocked}` in Tasks 7, 8. `mirrorImageOk` → `{ok, lo, hi}` in Tasks 7, 8, 12. `limRowOk` → Boolean in Tasks 7, 8, 12. `sendLim(id)` → `Promise<Boolean>` in Tasks 8, 11 — note it resolves `false` rather than rejecting, which is why `lockAxis` checks the value and rolls `j.cal` back. `fmtJog`/`fmtLim` → String in Tasks 7, 8, 9. `JOY_DEAD` and `LIM_MIN_SPAN` are defined once in Task 7; `LIM_MIN_SPAN` must equal the firmware's `LIM_MIN_SPAN_DEG` from Task 3, and Review Checkpoint 2 should confirm it.

**Ordering constraint.** Task 2 writes `jogMs` lines that Task 5 creates the field for. Task 2 Step 2 says to omit them and Task 5 Step 3 says to restore them — if the tasks are executed out of order, that is the compile break to expect.
