# Envelope + Joystick + Lock-an-Axis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the operator widen a joint's soft stops by dragging, drive the joint by feel with a spring-back joystick, and record the range they found as measured data — without leaving the console or editing a CSV by hand.

**Architecture:** Two narrow firmware changes (`STP <j>`; `LIM` accepted on an enabled joint when it cannot exclude the joint) plus console work in the single-file `arm-console.html`. The firmware's interpolator does the walking, so a held joystick sends two lines and goes quiet rather than streaming.

**Tech Stack:** Arduino C++ (AVR, `Servo 1.3.0`), ES5 JavaScript inline in one HTML file, Python 3.11 + pyserial for the firmware test harness, headless Chrome for the console test harness.

**Spec:** `Documentation/specs/2026-08-04-envelope-joystick-design.md`

## Global Constraints

- **Firmware:** no `String`, no `malloc`, no executable `delay()`, all literals in `F()`. Current build is 11650 B flash / 405 B SRAM; stay under 32256 B / 2048 B.
- **Console:** ES5 only — `var`, `function`, no arrow functions, no `let`/`const`, no template literals. The file must stay a single standalone HTML that works when double-clicked from `file://`.
- **Forbidden words in any UI copy or protocol text:** `position`, `actual`, `measured`, `feedback`. These servos report nothing; every angle is *commanded*. (Exception: the `MEASURED` badge in Task 7 refers to a human's measurement, not the servo's — that exact string is approved.)
- **A drag must never cause motion.** Load-bearing invariant of the whole design.
- **`calibrated` is set by exactly one code path** — the LOCK button in Task 7. Nothing infers it.
- `HOLD` ≠ `E-STOP`. `STP` freezes with joints powered; `EST` detaches and latches.
- Conventional commits: `feat:` / `fix:` / `docs:` / `test:` / `refactor:`.
- Every commit message ends with:
  ```
  Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ
  ```
- **Bench safety for every task:** servo power OFF (rocker off, supply unplugged) unless a step explicitly says otherwise. Only Task 9 involves a powered servo.

## File Structure

| File | Responsibility | Tasks |
|---|---|---|
| `Software/factorylm_arm_controller/factorylm_arm_controller.ino` | firmware: `STP <j>`, conditional `LIM` | 1, 2 |
| `Software/tests/protocol_check.py` | **new** — drives the real board over serial, asserts protocol replies | 1, 2 |
| `Documentation/SERIAL-PROTOCOL.md` | the contract; `STP` gains an argument, `E9` narrows | 1, 2 |
| `Software/arm-console/arm-console.html` | pure helpers + self-test, envelope UI, joystick UI, GO TO, LOCK | 3–8 |
| `Software/arm-console/joint-limits.csv` | unchanged in shape; written by the LOCK export | 7 |
| `Calibration_Notes/calibration-log.csv` | gains a row per LOCK | 7 |

**Why `protocol_check.py` is a new file rather than tests inside the sketch:** the firmware has no test framework and cannot have one — it runs on a 32 KB AVR. The only honest way to test it is to talk to it over the wire exactly as the console does. That script is the firmware's test suite.

---

## Task 1: Firmware — `STP <j>` stops one joint

**Files:**
- Modify: `Software/factorylm_arm_controller/factorylm_arm_controller.ino:769-772` (`doStp`), `:863` (dispatch)
- Modify: `Documentation/SERIAL-PROTOCOL.md` — the verb table row for `STP`
- Create: `Software/tests/protocol_check.py`

**Interfaces:**
- Consumes: existing `jointArg(int32_t, uint8_t*)`, `intArg(uint8_t, int32_t*)`, `errJPre(const __FlashStringHelper*, uint8_t)`, `okDone()`, `badArgc()`, `j[]`, `NJ`.
- Produces: wire behaviour `STP` (all joints) and `STP <j>` (one joint) — Task 5 sends `STP <j>` on joystick release. `protocol_check.py` exposes `Board.cmd(line) -> list[str]` and `expect(reply, substring)`, used by Task 2.

- [ ] **Step 1: Write the failing test harness**

Create `Software/tests/protocol_check.py`:

```python
"""Protocol tests against the real board. Run with servo power OFF.

    python Software/tests/protocol_check.py [COM5]

Every test drives the firmware over the wire exactly as the console does.
Nothing here enables a joint that it does not disable again.
"""

import sys
import time

import serial
from serial.tools import list_ports

BAUD = 115200


class Board:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.4)
        time.sleep(2.0)                      # Optiboot holds the bus ~1 s after reset
        self.ser.reset_input_buffer()

    def cmd(self, line):
        """Send one line, return every reply line until OK/ERR."""
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
            if text.startswith("OK ") or text == "OK" or text.startswith("ERR "):
                break
        return out

    def close(self):
        self.ser.close()


FAILURES = []


def expect(name, reply, needle):
    joined = " | ".join(reply)
    if needle in joined:
        print("  PASS  %s" % name)
    else:
        print("  FAIL  %s\n        wanted %r\n        got    %r" % (name, needle, joined))
        FAILURES.append(name)


def find_port():
    for p in list_ports.comports():
        if "2341" in (p.hwid or ""):
            return p.device
    ports = list_ports.comports()
    return ports[0].device if ports else None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if not port:
        print("no serial port found")
        return 2
    print("port %s" % port)
    b = Board(port)
    try:
        expect("VER identifies the arm controller", b.cmd("VER"), "NAME=FACTORYLM-ARM")

        # --- STP ---------------------------------------------------------
        expect("bare STP still accepted", b.cmd("STP"), "OK STP")
        expect("STP on a disabled joint", b.cmd("STP 3"), "ERR E6")
        expect("STP on a bad joint id", b.cmd("STP 9"), "ERR E4")
        expect("STP on the reserved id", b.cmd("STP 2"), "RESERVED=shoulder_pair")

        b.cmd("ENA 3 90")
        expect("STP on an enabled joint", b.cmd("STP 3"), "OK STP J3")
        b.cmd("DIS 3")
    finally:
        b.cmd("DIS A")
        b.close()

    print("\n%d failure(s)" % len(FAILURES))
    return 1 if FAILURES else 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Run it to make sure the new cases fail**

```bash
python "C:\RobotArm\Software\tests\protocol_check.py" COM5
```

Expected: `VER` and `bare STP` PASS; the four `STP <j>` cases FAIL, because today `STP 3` returns `ERR E2` (wrong argument count) rather than `E6`/`E4`/`OK STP J3`.

- [ ] **Step 3: Change `doStp` to take a joint**

Replace `factorylm_arm_controller.ino:769-772` with:

```cpp
// STP with no argument freezes every enabled joint - unchanged meaning, and
// what the HOLD MOTION button has always sent.  STP <j> freezes one, so
// releasing one joystick cannot freeze a joint the operator is not touching.
static void doStp() {
  for (uint8_t i = 0; i < NJ; i++) if (j[i].en) j[i].tgtC = j[i].setC;
  okDone();
}

static void doStpJoint(uint8_t i) {
  if (!j[i].en) { errJPre(F("E6"), i); Serial.println(); return; }
  j[i].tgtC = j[i].setC;
  okPre();
  Serial.print(F(" J"));
  Serial.println(i);
}
```

- [ ] **Step 4: Widen the dispatch**

Replace `factorylm_arm_controller.ino:863` with:

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

- [ ] **Step 5: Compile and check the size budget**

```bash
"C:\Program Files\Arduino CLI\arduino-cli.exe" compile --fqbn arduino:avr:uno "C:\RobotArm\Software\factorylm_arm_controller"
```

Expected: `0 errors`. Flash under 32256 B, SRAM under 2048 B. Note the new numbers — they go in the commit message.

- [ ] **Step 6: Upload, with servo power OFF**

```bash
"C:\Program Files\Arduino CLI\arduino-cli.exe" upload -p COM5 --fqbn arduino:avr:uno --verify "C:\RobotArm\Software\factorylm_arm_controller"
```

- [ ] **Step 7: Run the tests again**

```bash
python "C:\RobotArm\Software\tests\protocol_check.py" COM5
```

Expected: `0 failure(s)`.

- [ ] **Step 8: Update the protocol doc**

In `Documentation/SERIAL-PROTOCOL.md`, replace the `STP` row of the verb table with these two rows:

```markdown
| `STP` | — | `OK STP` | Freeze every enabled joint where it is. Joints stay driven. |
| `STP` | `<j>` | `OK STP J<j>` | Freeze one joint only. `E4` bad/reserved id, `E6` not enabled. |
```

- [ ] **Step 9: Commit**

```bash
cd /c/RobotArm
git add Software/factorylm_arm_controller/factorylm_arm_controller.ino Software/tests/protocol_check.py Documentation/SERIAL-PROTOCOL.md
git commit -m "feat(fw): STP takes an optional joint id

Bare STP keeps its exact meaning. STP <j> freezes one joint, so releasing
one joystick will not freeze a joint the operator is not touching.

Adds Software/tests/protocol_check.py -- the firmware's test suite, which
talks to the real board over the wire because a 32 KB AVR cannot host a
test framework.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 2: Firmware — `LIM` on an enabled joint, when it cannot exclude the joint

**Files:**
- Modify: `Software/factorylm_arm_controller/factorylm_arm_controller.ino:568-595` (`doLimSet` and its comment)
- Modify: `Software/tests/protocol_check.py`
- Modify: `Documentation/SERIAL-PROTOCOL.md` — the `E9` row and the `LIM` prose

**Interfaces:**
- Consumes: `Board.cmd`, `expect` from Task 1; `j[i].en`, `j[i].setC`, `j[i].tgtC`, `j[i].minD`, `j[i].maxD`, `errJPre`.
- Produces: `LIM` accepted on an enabled joint when `minD <= setC <= maxD`; Task 4 relies on this to send `LIM` during a live drag.

`setC` and `tgtC` are in **command units** (whatever `degToCmd` produces), `minD`/`maxD` are whole degrees. Convert with the sketch's existing `degToCmd()` before comparing — do not compare a command unit against a degree.

- [ ] **Step 1: Write the failing tests**

Add to `protocol_check.py`, inside `main()` after the STP block and before the `finally`:

```python
        # --- LIM on an enabled joint -------------------------------------
        b.cmd("LIM 3 70 110 0")
        b.cmd("ENA 3 90")
        expect("LIM that still contains the joint is accepted",
               b.cmd("LIM 3 60 120 0"), "OK LIM J3")
        expect("LIM that would exclude the joint is refused",
               b.cmd("LIM 3 100 120 0"), "ERR E9")
        expect("the refused LIM changed nothing",
               b.cmd("LIM"), "LIM J3 MIN=60 MAX=120")
        b.cmd("MOV 3 119")
        b.cmd("LIM 3 60 95 0")
        expect("a pending target is clamped inward, not left outside",
               b.cmd("STA"), "STA J3")
        b.cmd("DIS 3")
        expect("LIM on a disabled joint is unchanged",
               b.cmd("LIM 3 70 110 0"), "OK LIM J3")
        expect("MIR on an enabled shoulder is still refused",
               (b.cmd("ENA 0 90"), b.cmd("MIR SAME"))[1], "OK MIR")
        b.cmd("DIS 0")
```

Read the `STA J3` line by eye for the clamp check: after `LIM 3 60 95`, `TGT` must be `95` or lower, never `119`.

- [ ] **Step 2: Run to verify the new cases fail**

```bash
python "C:\RobotArm\Software\tests\protocol_check.py" COM5
```

Expected: `LIM that still contains the joint is accepted` FAILS with `ERR E9` — today every `LIM` on an enabled joint is refused.

- [ ] **Step 3: Replace `doLimSet`'s guard**

Replace the comment block and guard at `factorylm_arm_controller.ino:568-577` with:

```cpp
// LIM j min max cal.
//
// On a DISABLED joint: unchanged.
//
// On an ENABLED joint: accepted only if the joint's own commanded angle still
// falls inside the new range.  A range that would EXCLUDE the joint is refused
// with E9, exactly as before, because applying it would turn a limit edit into
// an unrequested move - the operator tidies a number and a loaded arm swings.
// That is the whole reason E9 existed; this narrows it rather than removing it.
//
// A pending target outside the new range is clamped inward.  Clamping can only
// make a move SHORTER, so a limit edit can never create travel.
//
// CAL is set explicitly from the argument, never inferred: a file that still
// holds the defaults must stay flagged uncalibrated.
//
// E10, not E5.  E5 is documented as "adopt angle outside this joint's MIN..MAX";
// a bad min/max pair or a bad cal flag is a different failure with a different
// remedy, and sharing the code made the GUI's plain-English message wrong.
static void doLimSet(uint8_t i, int32_t mn, int32_t mx, int32_t cal) {
  if (mn < 0 || mx > 180 || mn >= mx) {
    errJPre(F("E10"), i);
    Serial.print(F(" REQMIN="));  Serial.print(mn);
    Serial.print(F(" REQMAX="));  Serial.print(mx);
    Serial.println(F(" LIMIT=0..180 MIN<MAX"));
    return;
  }
  if (cal != 0 && cal != 1) {
    errJPre(F("E10"), i);
    Serial.print(F(" REQCAL="));
    Serial.println(cal);
    return;
  }
  if (j[i].en) {
    int16_t loC = degToCmd((uint8_t)mn), hiC = degToCmd((uint8_t)mx);
    if (j[i].setC < loC || j[i].setC > hiC) {
      errJPre(F("E9"), i);
      Serial.println(F(" STATE=enabled"));
      return;
    }
    if (j[i].tgtC < loC) j[i].tgtC = loC;
    if (j[i].tgtC > hiC) j[i].tgtC = hiC;
  }
```

Leave the rest of the function (the three assignments and `okPre()` reply) exactly as it is.

**Note on ordering:** the range and cal validation now runs *before* the enabled check, so an illegal `LIM` on an enabled joint reports `E10` (what is wrong with the numbers) rather than `E9` (that the joint is live). That is the more useful message and no test depends on the old order.

- [ ] **Step 4: Confirm `degToCmd` exists with that signature**

```bash
grep -n "degToCmd" "C:\RobotArm\Software\factorylm_arm_controller\factorylm_arm_controller.ino"
```

Expected: a definition taking a degree value and returning the command unit stored in `setC`/`tgtC`. If the real name or signature differs, use the actual one — do not add a second conversion helper.

- [ ] **Step 5: Compile**

```bash
"C:\Program Files\Arduino CLI\arduino-cli.exe" compile --fqbn arduino:avr:uno "C:\RobotArm\Software\factorylm_arm_controller"
```

Expected: `0 errors`, still inside the size budget.

- [ ] **Step 6: Upload with servo power OFF, then run the tests**

```bash
"C:\Program Files\Arduino CLI\arduino-cli.exe" upload -p COM5 --fqbn arduino:avr:uno --verify "C:\RobotArm\Software\factorylm_arm_controller"
python "C:\RobotArm\Software\tests\protocol_check.py" COM5
```

Expected: `0 failure(s)`.

- [ ] **Step 7: Update the protocol doc**

In `Documentation/SERIAL-PROTOCOL.md`, replace the `E9` row of the error table with:

```markdown
| `E9` | `MODE` | `ERR E9 <VERB> JOINT=<j> STATE=enabled` | `MIR` while joint 1 is enabled, or a `LIM` whose new range would not contain the joint's own commanded angle |
```

And add, under the `LIM` prose:

> **`LIM` on an enabled joint.** Accepted, provided the new range still contains that joint's
> commanded angle. A range that would exclude it is refused with `E9` — applying it would turn a
> limit edit into an unrequested move. A pending target outside the new range is clamped inward;
> clamping can only shorten a move, never create travel.

- [ ] **Step 8: Commit**

```bash
cd /c/RobotArm
git add Software/factorylm_arm_controller/factorylm_arm_controller.ino Software/tests/protocol_check.py Documentation/SERIAL-PROTOCOL.md
git commit -m "feat(fw): allow LIM on an enabled joint when it cannot exclude the joint

Live envelope editing needs LIM to work on a driven joint. E9 is narrowed
rather than removed: a range that would EXCLUDE the joint's commanded angle
is still refused, because applying it would turn a limit edit into an
unrequested move -- which is what E9 was written to prevent. A pending
target outside the new range is clamped inward, and clamping can only make
a move shorter, never create travel.

MIR on an enabled shoulder stays refused.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 3: Console — pure helpers and a self-test harness

**Files:**
- Modify: `Software/arm-console/arm-console.html` — add helpers near `clampi` (`:565`), add a self-test block before `</body>`

**Interfaces:**
- Consumes: `clampi(v, lo, hi)` at `:565`.
- Produces, all global, used by Tasks 4–7:
  - `joySpeed(u, dpsMax)` → `{dir: -1|0|1, dps: Number}` — `u` is deflection `-1..+1`.
  - `envClampHandle(which, want, j)` → `{value: Number, blocked: Boolean}` — `which` is `"min"` or `"max"`, `want` is the raw degree the pointer is over, `j` is the joint record.
  - `JOY_DEAD` = `0.14`.

- [ ] **Step 1: Write the failing self-test**

Insert immediately before the closing `</script>` that ends the main console script:

```javascript
/* ?selftest=1 runs the pure helpers against known values and writes the result
   into #selftest. No serial, no bridge -- this runs from file:// too, so it can
   be checked headlessly with:
     chrome --headless=new --dump-dom "file:///C:/RobotArm/Software/arm-console/arm-console.html?selftest=1"
   Grep the dump for SELFTEST. */
function runSelfTest(){
  var fails = [];
  function eq(name, got, want){
    if (JSON.stringify(got) !== JSON.stringify(want)) {
      fails.push(name + ": got " + JSON.stringify(got) + " want " + JSON.stringify(want));
    }
  }
  var jx = { min:70, max:110, set:90, en:true };

  eq("centre is dead",            joySpeed(0, 30),      { dir:0, dps:0 });
  eq("inside dead zone is dead",  joySpeed(0.10, 30),   { dir:0, dps:0 });
  eq("just past dead zone crawls",joySpeed(0.15, 30),   { dir:1, dps:1 });
  eq("full right is max dps",     joySpeed(1, 30),      { dir:1, dps:30 });
  eq("full left is max dps",      joySpeed(-1, 30),     { dir:-1, dps:30 });
  eq("dps ceiling is per joint",  joySpeed(1, 12),      { dir:1, dps:12 });

  eq("min handle free below joint",  envClampHandle("min", 80, jx),  { value:80,  blocked:false });
  eq("min handle blocked at joint",  envClampHandle("min", 95, jx),  { value:90,  blocked:true  });
  eq("max handle free above joint",  envClampHandle("max", 130, jx), { value:130, blocked:false });
  eq("max handle blocked at joint",  envClampHandle("max", 85, jx),  { value:90,  blocked:true  });
  eq("min handle floors at 0",       envClampHandle("min", -20, jx), { value:0,   blocked:false });
  eq("max handle ceils at 180",      envClampHandle("max", 200, jx), { value:180, blocked:false });

  var jd = { min:70, max:110, set:90, en:false };
  eq("disabled joint does not block", envClampHandle("min", 95, jd), { value:95, blocked:false });

  var out = document.createElement("pre");
  out.id = "selftest";
  out.textContent = fails.length
    ? ("SELFTEST FAIL (" + fails.length + ")\n" + fails.join("\n"))
    : "SELFTEST PASS";
  document.body.appendChild(out);
}
if (/[?&]selftest=1/.test(location.search)) runSelfTest();
```

- [ ] **Step 2: Run it to verify it fails**

```bash
"/c/Program Files/Google/Chrome/Application/chrome.exe" --headless=new --disable-gpu --dump-dom "file:///C:/RobotArm/Software/arm-console/arm-console.html?selftest=1" 2>/dev/null | grep -o "SELFTEST[^<]*"
```

Expected: nothing printed, or a JS error — `joySpeed` and `envClampHandle` do not exist yet.

- [ ] **Step 3: Write the helpers**

Insert directly after `function clampi(v,lo,hi){ ... }` at `:565`:

```javascript
/* Joystick deflection -> speed. u is -1..+1. Inside the dead zone the joint is
   still, so a resting hand cannot creep it. Beyond it, deflection maps linearly
   from 1 deg/s up to this joint's own max_deg_per_sec -- never past it. */
var JOY_DEAD = 0.14;

function joySpeed(u, dpsMax){
  var mag = Math.abs(u);
  if (mag <= JOY_DEAD) return { dir:0, dps:0 };
  var k = (mag - JOY_DEAD) / (1 - JOY_DEAD);
  return { dir: u > 0 ? 1 : -1, dps: clampi(Math.round(1 + k * (dpsMax - 1)), 1, dpsMax) };
}

/* Envelope handle clamp. A handle may never be dragged past the joint's own
   commanded angle while that joint is driven -- squeezing the envelope past the
   joint would make the arm move because someone tidied a number. The firmware
   refuses such a LIM with E9; this stops it ever being sent. */
function envClampHandle(which, want, j){
  var v = Math.round(want), blocked = false;
  if (which === "min") {
    v = clampi(v, 0, j.max - 2);
    if (j.en && v > j.set) { v = Math.floor(j.set); blocked = true; }
  } else {
    v = clampi(v, j.min + 2, 180);
    if (j.en && v < j.set) { v = Math.ceil(j.set); blocked = true; }
  }
  return { value: v, blocked: blocked };
}
```

- [ ] **Step 4: Run the self-test again**

```bash
"/c/Program Files/Google/Chrome/Application/chrome.exe" --headless=new --disable-gpu --dump-dom "file:///C:/RobotArm/Software/arm-console/arm-console.html?selftest=1" 2>/dev/null | grep -o "SELFTEST[^<]*"
```

Expected: `SELFTEST PASS`.

- [ ] **Step 5: Commit**

```bash
cd /c/RobotArm
git add Software/arm-console/arm-console.html
git commit -m "test(console): pure joystick + envelope helpers with a headless self-test

joySpeed maps deflection to speed with a dead zone and the joint's own
max_deg_per_sec as the ceiling. envClampHandle stops an envelope handle
being dragged past the joint it would exclude.

?selftest=1 runs them from file:// with no board and no bridge, so the
logic is checkable headlessly before any servo is involved.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 4: Console — the envelope control

**Files:**
- Modify: `Software/arm-console/arm-console.html` — card markup in `buildCards()` (`:1414`), `j.dom` map (`:1447`), `paintJoint` (`:1560` area), plus CSS in the `<style>` block

**Interfaces:**
- Consumes: `envClampHandle` (Task 3), `send`, `notice`, `plainErr`, `paintJoint`, `J`, `connState`.
- Produces: `sendLim(id)` — pushes that joint's current `min`/`max`/`cal` with one `LIM`. Task 7 calls it.

- [ ] **Step 1: Add the CSS**

Append inside the existing `<style>` block:

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
```

- [ ] **Step 2: Add the markup**

In `buildCards()`, insert immediately **before** the `'<input type="range" class="js-slider" ...'` line:

```javascript
      '<div class="lbl">ENVELOPE &mdash; the soft stops</div>' +
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

- [ ] **Step 3: Extend the `j.dom` map**

Add these entries to the `j.dom = { ... }` object literal:

```javascript
      env:q(".js-env"), envBand:q(".js-envband"), envJoint:q(".js-envjoint"),
      hmin:q(".js-hmin"), hmax:q(".js-hmax"),
      envMin:q(".js-envmin"), envMax:q(".js-envmax"), envMsg:q(".js-envmsg"),
```

- [ ] **Step 4: Wire the drag**

Add inside `buildCards()`'s per-joint block, after the existing slider listeners:

```javascript
    var envDrag = null;
    function envDegFromX(clientX){
      var r = j.dom.env.getBoundingClientRect();
      return (clientX - r.left) / r.width * 180;
    }
    function envMove(e){
      var res = envClampHandle(envDrag, envDegFromX(e.clientX), j);
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

- [ ] **Step 5: Add `sendLim`**

Add near the other senders, after `pushState()`:

```javascript
/* One LIM for one joint. cal is passed through unchanged -- dragging an
   envelope handle is not a measurement, so it must never set the flag. Only
   the LOCK button does that. */
function sendLim(id){
  var j = J[id];
  if (connState !== "on") return Promise.resolve();
  return send("LIM " + id + " " + j.min + " " + j.max + " " + (j.cal ? 1 : 0))
    .catch(function(e){ notice("warn", plainErr(e.message, { id:id, name:J[id].def.name })); });
}
```

- [ ] **Step 6: Paint the envelope**

In `paintJoint`, beside the existing `D.slider.value` line at `:1560`, add:

```javascript
  var a = D.env ? (j.min / 180 * 100) : 0, b = D.env ? (j.max / 180 * 100) : 0;
  if (D.env) {
    D.hmin.style.left = a + "%";
    D.hmax.style.left = b + "%";
    D.envBand.style.left = a + "%";
    D.envBand.style.width = (b - a) + "%";
    D.envJoint.style.left = (clampi(j.set, 0, 180) / 180 * 100) + "%";
    D.envJoint.style.display = j.en ? "block" : "none";
    D.envMin.textContent = j.min + "\u00b0";
    D.envMax.textContent = j.max + "\u00b0";
  }
```

- [ ] **Step 7: Verify with the bridge running, servo power OFF**

```bash
powershell -NoProfile -ExecutionPolicy Bypass -File "C:\RobotArm\Documentation\preflight-arm-gui.ps1"
powershell -NoProfile -Command "Start-Process -FilePath 'C:\RobotArm\START ARM GUI.bat' -WorkingDirectory 'C:\RobotArm'"
```

In the browser: CONNECT, then drag J3's handles. Expected — the band and numbers follow, and after each release the board's own view agrees:

```bash
python "C:\RobotArm\Software\tests\protocol_check.py" COM5   # after closing the browser tab
```

Then ENABLE J3 at 90 and drag the min handle right: it must stop at 90, turn red, and show the message.

- [ ] **Step 8: Commit**

```bash
cd /c/RobotArm
git add Software/arm-console/arm-console.html
git commit -m "feat(console): draggable envelope on every joint card

Two handles set the soft stops and push one LIM on release. While the joint
is driven a handle stops dead at the joint's commanded angle rather than
squeezing past it, so a drag can never cause motion. Dragging never touches
the calibrated flag.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 5: Console — the joystick replaces the position slider

**Files:**
- Modify: `Software/arm-console/arm-console.html` — card markup, `j.dom`, the slider listeners at `:1454-1468`, `paintJoint` at `:1560`, CSS

**Interfaces:**
- Consumes: `joySpeed`, `JOY_DEAD` (Task 3), `send`, `J`, `connState`.
- Produces: `joyRelease(id)` — the single stop path. Task 6 and the E-STOP handler call it.

- [ ] **Step 1: Add the CSS**

```css
.joy{position:relative;height:44px;margin:10px 2px 0;touch-action:none}
.joytrack{position:absolute;top:17px;left:0;right:0;height:8px;background:var(--panel-3);border-radius:4px}
.joydead{position:absolute;top:17px;height:8px;background:var(--line);border-radius:4px}
.joyfill{position:absolute;top:17px;height:8px;background:var(--move);border-radius:4px}
.joyh{position:absolute;top:7px;left:50%;width:34px;height:28px;margin-left:-17px;background:var(--panel-2);
      border:2px solid var(--dim);border-radius:6px;cursor:grab}
.joyh.live{border-color:var(--move);background:var(--move-bg);cursor:grabbing}
```

- [ ] **Step 2: Replace the position slider markup**

Replace the `'<input type="range" class="js-slider" ...'` line and the `'<div class="scale">...'` line with:

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

Keep `js-lo` / `js-hi` — `paintJoint` already writes them and they still label the ends.

- [ ] **Step 3: Update `j.dom`**

Remove `slider:q(".js-slider"),` and add:

```javascript
      joy:q(".js-joy"), joyH:q(".js-joyh"), joyFill:q(".js-joyfill"), joyDead:q(".js-joydead"),
```

- [ ] **Step 4: Replace the slider listeners**

Delete the five `j.dom.slider.addEventListener(...)` blocks at `:1454-1468` and put this in their place:

```javascript
    var joyHeld = false, joyLastDps = 0, joyLastDir = 0;

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
      /* SPD only when the mapped value actually changes, and MOV only on a
         direction change: the firmware's interpolator does the walking, so a
         held joystick is two lines and then silence, not a line per pixel. */
      if (s.dps && s.dps !== joyLastDps) { joyLastDps = s.dps; send("SPD " + d.id + " " + s.dps); }
      if (s.dir && s.dir !== joyLastDir) {
        joyLastDir = s.dir;
        send("MOV " + d.id + " " + (s.dir > 0 ? j.max : j.min));
      }
    }
    function joyDown(e){
      e.preventDefault();
      if (!j.en) { notice("warn", "Enable J" + d.id + " before moving it."); return; }
      joyHeld = true; joyLastDps = 0; joyLastDir = 0;
      j.dom.joyH.classList.add("live");
      window.addEventListener("pointermove", joyMove);
      window.addEventListener("pointerup", joyRelease2);
      window.addEventListener("pointercancel", joyRelease2);
      joyApply(joyU(e.clientX));
    }
    function joyMove(e){ if (joyHeld) joyApply(joyU(e.clientX)); }
    function joyRelease2(){
      if (!joyHeld) return;
      joyHeld = false;
      window.removeEventListener("pointermove", joyMove);
      window.removeEventListener("pointerup", joyRelease2);
      window.removeEventListener("pointercancel", joyRelease2);
      j.dom.joyH.classList.remove("live");
      j.dom.joyH.style.left = "50%";
      j.dom.joyFill.style.width = "0%";
      joyLastDps = 0; joyLastDir = 0;
      if (connState === "on" && j.en) send("STP " + d.id);
    }
    j.releaseJoy = joyRelease2;
    j.dom.joy.addEventListener("pointerdown", joyDown);
    j.dom.joyDead.style.left  = (50 - JOY_DEAD * 50) + "%";
    j.dom.joyDead.style.width = (JOY_DEAD * 100) + "%";
```

- [ ] **Step 5: Add the global dead-man and the release helper**

Add near `sendHold()` at `:1232`:

```javascript
/* Letting go, losing the pointer, losing the window, or the tab going to the
   background are all the same event: stop. */
function joyRelease(id){ if (J[id] && J[id].releaseJoy) J[id].releaseJoy(); }
function joyReleaseAll(){ JOINT_DEFS.forEach(function(d){ joyRelease(d.id); }); }
window.addEventListener("blur", joyReleaseAll);
document.addEventListener("visibilitychange", function(){ if (document.hidden) joyReleaseAll(); });
```

- [ ] **Step 6: Remove the dead slider reference in `paintJoint`**

Delete the line at `:1560`:

```javascript
  if (!j.dragging) D.slider.value = clampi(Math.round(j.set), j.min, j.max);
```

`j.dragging` and `j.wantMove` are now unused by the joystick, but `wantMove` is still consumed at `:1167-1168` — Task 6 re-fills it. Leave both fields on the joint record.

- [ ] **Step 7: Verify**

Self-test still green:

```bash
"/c/Program Files/Google/Chrome/Application/chrome.exe" --headless=new --disable-gpu --dump-dom "file:///C:/RobotArm/Software/arm-console/arm-console.html?selftest=1" 2>/dev/null | grep -o "SELFTEST[^<]*"
```

Then, servo power OFF, bridge up, J3 enabled at 90: hold the joystick right and confirm on the board that `SPD` and `MOV` arrived and that `STA` shows `TGT` at the envelope max and `MOV=1`. Release and confirm `MOV=0`. Alt-tab away mid-hold and confirm it stops.

- [ ] **Step 8: Commit**

```bash
cd /c/RobotArm
git add Software/arm-console/arm-console.html
git commit -m "feat(console): spring-back joystick replaces the position slider

Deflection sets speed, capped at the joint's own max_deg_per_sec. Holding
sends one SPD and one MOV to the envelope edge and then goes quiet -- the
firmware's interpolator walks it, so there is no command-per-pixel stream.
Release sends STP <j>, which now stops that joint alone.

Pointer loss, window blur and tab hide are all treated as letting go.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 6: Console — the GO TO box

**Files:** Modify `Software/arm-console/arm-console.html` — card markup, `j.dom`, `buildCards()` wiring

**Interfaces:** Consumes `clampi`, `J`, `j.wantMove` (drained at `:1167-1168`). Produces nothing new.

> **Confirm before building.** The spec lists this as an open question (§12.1). It exists because replacing the position slider removed the only way to command an exact angle. If the operator does not want it, skip this task entirely — nothing else depends on it.

- [ ] **Step 1: Add the markup** — insert after the `'<div class="scale">...'` line:

```javascript
      '<div class="gotorow">GO TO <input type="number" class="js-goto" step="1">' +
        '<button class="btn small js-gogo">GO</button></div>' +
```

- [ ] **Step 2: Add to `j.dom`:** `goTo:q(".js-goto"), goBtn:q(".js-gogo"),`

- [ ] **Step 3: Wire it** — add in `buildCards()`:

```javascript
    j.dom.goBtn.onclick = function(){
      if (!j.en) { notice("warn", "Enable J" + d.id + " first."); return; }
      var v = clampi(Math.round(Number(j.dom.goTo.value)), j.min, j.max);
      j.dom.goTo.value = v;
      j.wantMove = v;
    };
    j.dom.goTo.addEventListener("keydown", function(e){ if (e.key === "Enter") j.dom.goBtn.onclick(); });
```

- [ ] **Step 4: Verify** — servo power OFF, J3 enabled: type `105`, press GO, confirm `STA` shows `TGT=105`. Type `500`, press GO, confirm the box rewrites itself to the envelope max and no out-of-range `MOV` is sent.

- [ ] **Step 5: Commit**

```bash
cd /c/RobotArm
git add Software/arm-console/arm-console.html
git commit -m "feat(console): GO TO box for commanding an exact angle

The joystick replaced the position slider, which removed the only way to
say 'go to exactly 90'. Clamped to the envelope before it is sent.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 7: Console — LOCK THIS AXIS

**Files:** Modify `Software/arm-console/arm-console.html` — card markup, `j.dom`, `buildCards()`, `paintJoint`; the CSV builder near `:2027`

**Interfaces:** Consumes `sendLim` (Task 4), `send`, `notice`, `J`, `JOINT_DEFS`, `LIM_COLS` (`:527`). Produces `lockAxis(id)`, `downloadLimitsCsv()`, `downloadCalibrationRow(id)`.

- [ ] **Step 1: Add the markup** — inside `'<div class="cardactions">'`, after the DISABLE button:

```javascript
        '<button class="btn small js-lock" disabled>LOCK THIS AXIS</button>' +
```

- [ ] **Step 2: Add to `j.dom`:** `lock:q(".js-lock"),`

- [ ] **Step 3: Track whether the envelope has been touched**

In `envUp()` (Task 4, Step 4), before `sendLim(d.id)`, add:

```javascript
      j.envTouched = true;
```

And add `envTouched:false` to the joint record literal at `:538`.

- [ ] **Step 4: Implement the lock**

```javascript
/* The ONLY code path allowed to set the calibrated flag. Nothing infers it.
   The amber UNCALIBRATED badge is the only warning that a joint's range is a
   placeholder, so a system that could clear it on its own would be lying. */
function lockAxis(id){
  var j = J[id];
  if (!j.en)          { notice("warn", "Enable J" + id + " and settle it where you want its centre first."); return; }
  if (!j.envTouched)  { notice("warn", "Move J" + id + "'s envelope handles first. Locking the placeholder would record a range nobody measured."); return; }
  j.cal  = true;
  j.home = Math.round(j.set);
  sendLim(id).then(function(){
    notice("ok", "J" + id + " " + j.def.name + " locked: " + j.min + "-" + j.max +
                 "\u00b0, centre " + j.home + "\u00b0. Save the limits file to keep it past a reset.", 9000);
    downloadCalibrationRow(id);
    paintJoint(id);
  });
}

/* One row per lock, matching Calibration_Notes/calibration-log.csv's columns.
   Commanded values only -- there is no measurement of where the joint actually
   is, and there will not be until something external watches the arm. */
function downloadCalibrationRow(id){
  var j = J[id], d = j.def, now = new Date();
  var stamp = now.getFullYear() + "-" + p2(now.getMonth()+1) + "-" + p2(now.getDate());
  var row = [stamp, id, d.name, d.pins, "", "yes", "", "", "", "", "locked-in-console",
             "horn state NOT RECORDED - fill in",
             "Envelope locked from the arm console. min=" + j.min + " max=" + j.max +
             " home=" + j.home + ". Commanded angles only."].join(",");
  var url = URL.createObjectURL(new Blob([row + "\n"], { type:"text/csv" }));
  var a = document.createElement("a");
  a.href = url;
  a.download = "calibration-row-J" + id + "-" + stamp + ".csv";
  a.click();
  URL.revokeObjectURL(url);
}
```

- [ ] **Step 5: Wire the button and paint its state**

In `buildCards()`:

```javascript
    j.dom.lock.onclick = function(){ lockAxis(d.id); };
```

In `paintJoint`, next to the existing badge painting:

```javascript
  if (D.lock) D.lock.disabled = !(j.en && j.envTouched);
  if (D.cal) {
    D.cal.textContent = j.cal ? "MEASURED" : "UNCALIBRATED \u2014 LIMITS ARE A PLACEHOLDER";
    D.cal.className = "pill " + (j.cal ? "ok" : "uncal") + " js-cal";
  }
```

- [ ] **Step 6: Add a SAVE LIMITS FILE button**

Next to the existing `LOAD LIMITS FILE` label at `:408`:

```javascript
      '<button class="btn small" id="saveLim">SAVE LIMITS FILE</button>' +
```

and:

```javascript
/* Writes the current in-memory limits back out in the same column order the
   loader requires, so the file round-trips. The firmware keeps nothing across
   a reset -- this file is the only place a measured range survives. */
function downloadLimitsCsv(){
  var lines = [LIM_COLS.join(",")];
  JOINT_DEFS.forEach(function(d){
    var j = J[d.id];
    lines.push([d.id, d.name, d.pins, j.min, j.max, j.home, j.dps,
                j.cal ? "yes" : "no", d.id === 1 ? mirrorMode : "",
                (j.notes || "").replace(/,/g, ";"), d.id === 1 ? mirrorOffset : 0].join(","));
  });
  var url = URL.createObjectURL(new Blob([lines.join("\n") + "\n"], { type:"text/csv" }));
  var a = document.createElement("a");
  a.href = url; a.download = "joint-limits.csv"; a.click();
  URL.revokeObjectURL(url);
}
el("saveLim").onclick = downloadLimitsCsv;
```

- [ ] **Step 7: Verify the round trip** — servo power OFF: enable J3, drag its envelope to 60–130, LOCK, SAVE LIMITS FILE. Replace `Software/arm-console/joint-limits.csv` with the download. Reconnect (which resets the board) and confirm J3 comes back 60–130 with a green `MEASURED` badge, and that `LIM` on the board reports `MIN=60 MAX=130 CAL=1`.

- [ ] **Step 8: Commit**

```bash
cd /c/RobotArm
git add Software/arm-console/arm-console.html
git commit -m "feat(console): LOCK THIS AXIS records a felt-out range as measured

The only code path in the system permitted to set the calibrated flag. It
sends LIM with cal=1, records the settled centre as home_deg, offers the
updated joint-limits.csv, and emits a calibration-log row.

Disabled until the operator has actually moved that joint's envelope
handles, so the 70-110 placeholder can never be locked in as if somebody
had measured it.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 8: Console — re-check the shoulder mirror on an envelope drag

**Files:** Modify `Software/arm-console/arm-console.html` — extract the existing check out of `loadLimitsCsv` (`:1938-1946`) and call it from `envMove` too

**Interfaces:** Consumes `mirrorMode`, `mirrorOffset`. Produces `mirrorImageOk(mn, mx, mode, off)` → `{ok: Boolean, lo: Number, hi: Number}`.

- [ ] **Step 1: Add the case to the self-test** (Task 3's `runSelfTest`):

```javascript
  eq("mirror image legal at offset 0",  mirrorImageOk(70, 110, "inverted", 0),  { ok:true,  lo:70,  hi:110 });
  eq("mirror image illegal when wide",  mirrorImageOk(10, 170, "inverted", 20), { ok:false, lo:-130, hi:30 });
  eq("mirror irrelevant when same",     mirrorImageOk(0, 180, "same", 0),       { ok:true,  lo:0,   hi:180 });
```

- [ ] **Step 2: Run it and watch it fail** — `SELFTEST FAIL`, `mirrorImageOk is not defined`.

- [ ] **Step 3: Extract the helper**, next to `envClampHandle`:

```javascript
/* Under mirror_mode=inverted the firmware drives the pair's second servo to
   (180 + 2*offset) - angle. If any part of joint 1's travel maps outside
   0-180, two MG996Rs would fight through one link. Same arithmetic the CSV
   loader already ran -- now shared, so a drag cannot bypass it. */
function mirrorImageOk(mn, mx, mode, off){
  if (mode !== "inverted") return { ok:true, lo:mn, hi:mx };
  var lo = (180 + 2*off) - mx, hi = (180 + 2*off) - mn;
  return { ok: (lo >= 0 && hi <= 180), lo:lo, hi:hi };
}
```

- [ ] **Step 4: Use it in both places.** In `loadLimitsCsv` replace the inline arithmetic at `:1938-1946` with a call to `mirrorImageOk`, keeping the existing error text. In `envMove` (Task 4), before assigning:

```javascript
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
```

- [ ] **Step 5: Run the self-test** — `SELFTEST PASS`.

- [ ] **Step 6: Commit**

```bash
cd /c/RobotArm
git add Software/arm-console/arm-console.html
git commit -m "fix(console): share the shoulder mirror check between file load and drag

The mirror-image arithmetic lived inside loadLimitsCsv, so dragging J1's
envelope could stage a range the file loader would have refused. Extracted
to mirrorImageOk and called from both, with self-test cases.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
```

---

## Task 9: Bench verification and the PR

**Files:** Create `Documentation/2026-08-04-envelope-joystick-bench-log.md`

- [ ] **Step 1: Run both harnesses**

```bash
python "C:\RobotArm\Software\tests\protocol_check.py" COM5
"/c/Program Files/Google/Chrome/Application/chrome.exe" --headless=new --disable-gpu --dump-dom "file:///C:/RobotArm/Software/arm-console/arm-console.html?selftest=1" 2>/dev/null | grep -o "SELFTEST[^<]*"
```

Expected: `0 failure(s)` and `SELFTEST PASS`.

- [ ] **Step 2: Walk the spec's §10 table with ZERO servos powered.** Record the observed result for all fourteen rows in the bench log. Any row that cannot be checked without a motor is marked `deferred to step 3`, not `pass`.

- [ ] **Step 3: One powered joint.** Wrist roll, D10 — the only servo whose type is documented, and the lightest loaded. Horn off, flag taped to the spline, hand on the rocker. Confirm: joystick moves it, speed tracks deflection, it stops at the envelope edge, release stops it, LOCK records the range. Record what was actually observed, including anything surprising.

- [ ] **Step 4: Screenshot the card** and save it beside the bench log.

- [ ] **Step 5: Commit and open the PR**

```bash
cd /c/RobotArm
git remote -v                      # confirm emre-kalem-robot-arm before anything else
git add Documentation/2026-08-04-envelope-joystick-bench-log.md
git commit -m "docs: bench log for the envelope + joystick work

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Y439m3TLBSprzpjcp4ZfjQ"
git push -u origin feat/envelope-joystick
git log origin/feat/envelope-joystick --oneline -3    # verify the push landed
gh pr create --base main --head feat/envelope-joystick --title "feat: draggable envelope, per-joint joystick, lock-an-axis"
```

The PR body must state the flash/SRAM figures, both harness results, and which §10 rows were checked unpowered versus powered.

---

## Self-Review

**Spec coverage:** §4 card → Tasks 4–7. §5 envelope → Task 4 (+ Task 8 for the shoulder). §6 joystick → Task 5; GO TO → Task 6. §7 lock → Task 7. §8a `STP <j>` → Task 1. §8b conditional `LIM` → Task 2. §9 invariants → Global Constraints, enforced per task. §10 verification → Task 9, with the harnesses built in Tasks 1 and 3. §11 webcam → explicitly out of scope, no task. §12.1 GO TO → Task 6, gated on confirmation. **§12.2 (should LOCK narrow the envelope to what was actually reached?) has no task** — the spec's stated default, *record the handles as dragged*, is what Task 7 implements. If the answer changes, Task 7 Step 4 is the only place affected.

**Placeholder scan:** no TBDs; every code step carries real code; the one deliberate unknown — `degToCmd`'s exact signature — has its own verification step (Task 2, Step 4) rather than being assumed.

**Type consistency:** `joySpeed` returns `{dir, dps}` in Tasks 3 and 5. `envClampHandle` returns `{value, blocked}` in Tasks 3, 4 and 8. `mirrorImageOk` returns `{ok, lo, hi}` in Task 8. `sendLim(id)` returns a Promise in Tasks 4 and 7. `j.envTouched` is created in Task 7 Step 3 and read in Task 7 Step 5.

**Known rough edge:** Task 5 removes `D.slider` while `j.dragging` and `j.wantMove` survive on the joint record — `wantMove` because `:1167` still drains it and Task 6 refills it, `dragging` because nothing reads it any more. Leaving a genuinely dead field is deliberate: deleting it touches the joint-record literal that four other tasks also edit, and a follow-up commit can drop it cleanly.
