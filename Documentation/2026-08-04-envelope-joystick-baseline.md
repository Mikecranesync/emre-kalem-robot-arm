# Task 0 — Source-verified baseline

**Date:** 2026-08-04
**Branch:** `feat/envelope-joystick`
**Commit verified against:** `b6a5eee0ff3c9602271e516c3d99bbd495037fbb`
**Working tree at time of verification:** clean
**Scope:** read-only verification. No production code was modified.

---

## 1. Identity

| | |
|---|---|
| Firmware | `Software/factorylm_arm_controller/factorylm_arm_controller.ino` — 1113 lines<br/>`sha256 c110322b3fc6cd46c014a3d5315404a3c326b71b506b6a2f7e04443569e52d04` |
| Console | `Software/arm-console/arm-console.html` — 2228 lines<br/>`sha256 390630e6693d0727d66a768307531dbf4bc5e0100c016d004de269b8dde11cf0` |
| Bridge | `Software/arm-console/arm-bridge.py` — 861 lines |
| Protocol | `Documentation/SERIAL-PROTOCOL.md` — 971 lines |
| Board | genuine Arduino Uno R3, `VID:PID 2341:0043`, `COM5` |

## 2. Toolchain and build baseline

```
arduino-cli 1.5.1 (commit 01f3d4f2b, 2026-06-05)
arduino:avr 1.8.8
Servo 1.3.0
```

```
Sketch uses 11650 bytes (36%) of program storage space. Maximum is 32256 bytes.
Global variables use 405 bytes (19%) of dynamic memory, leaving 1643 bytes. Maximum is 2048 bytes.
exit 0
```

Matches the figures the plan assumed. **Headroom: 20606 B flash, 1643 B SRAM.**

**No other test suite exists yet.** `protocol_check.py` and `selftest.sh` are created by Tasks 1 and 7. The only current automated check is the compile above.

## 3. Anchor verification

| Plan quotes | Reality | Verdict |
|---|---|---|
| `doStp()` at `:769-772` | `:769-772`, loops `i < NJ`, sets `tgtC = setC` on enabled joints | ✅ |
| `doLimSet` at `:576`, `E9` guard at `:577` | exact | ✅ |
| `STP` dispatch at `:863` | exact, `tokc != 0 → badArgc()` | ✅ |
| `jointArg(int32_t, uint8_t*)` | `:829`; rejects `<0`, `>= NJ`, **and `RESERVED_ID`**, emits `errJoint` itself | ✅ |
| `intArg(uint8_t, int32_t*)` | `:838`; emits `E3` itself | ✅ |
| `badArgc()` | `:847`; emits `ERR E2 <VERB> N=<tokc>` | ✅ |
| `errJPre` / `errJ` / `errJoint` / `okPre` / `okDone` / `errPlain` | all exist as quoted | ✅ |
| `clampi`, `send`, `paintJoint`, `buildCards`, `loadLimitsCsv`, `LIM_COLS`, `mirrorMode`, `mirrorOffset`, `connState` | all exist | ✅ |
| **`degToCmd(...)`** | **DOES NOT EXIST** | ❌ **C1** |
| **`d0(j)` inside `paintJoint`** | **INVENTED** — `paintJoint(id)` already has `id` in scope | ❌ **C10** |
| **`p2(n)` as a global** | **nested** inside another function at `:2025` | ❌ **C11** |
| **`sysInfo.fw`** | **DOES NOT EXIST** — no `sysInfo` anywhere | ❌ **C12** |

## 4. Units and conversion — correction C1

There is no `degToCmd`. The real conversions are:

```cpp
static uint16_t degCToUs(int16_t c);   // centidegrees -> microseconds (544 + c*1856/18000)
static int16_t  degOf(int16_t c);      // centidegrees -> whole degrees, rounded ((c+50)/100)
```

| Quantity | Type | Units |
|---|---|---|
| `j[i].setC`, `j[i].tgtC` | `int16_t` | **centidegrees** (`DEF_SET_C = 9000` = 90.00°) |
| `j[i].minD`, `j[i].maxD` | `uint8_t` | **whole degrees** |
| `mirrorOffsetC` | `int16_t` | **centidegrees** |
| everything on the wire | — | **whole degrees** |

Degrees → centidegrees is `* 100`, written inline. Existing precedents: `clampToLimits` (`:378-379`) uses `minD * 100`; `doLimSet` (`:600`) uses `mn * 100`; `enableJoint` (`:477`) uses `adoptDeg * 100`.

**Consequence for Task 4:** `degToCmd((uint8_t)mn)` must be `(int16_t)mn * 100`.

### Clamping already in place

```cpp
static int16_t clampToLimits(uint8_t i, int16_t c);   // :377, clamps to [minD*100, maxD*100]
```

Called from `writeJoint` (`:418`) and `enableJoint` (`:477`) — the two functions that write a pulse width. The firmware's own comment: *"the per-joint envelope is a STRUCTURAL property of the write path and not a lucky property of four careful callers."*

## 5. Logical → physical mapping

```cpp
const uint8_t PIN_A[NJ] = {   3,   4, NO_PIN,      6,      9,     10,     11 };
const uint8_t PIN_B[NJ] = { NO_PIN, 5, NO_PIN, NO_PIN, NO_PIN, NO_PIN, NO_PIN };
```

`NJ = 7` slots, `RESERVED_ID = 2`, `NO_PIN = 255`, `NUM_DIGITAL_PINS = 20` on the Uno.

| Logical id | Pin(s) | Servo object | Addressable |
|---|---|---|---|
| 0 Base | D3 | `sA[0]` | yes |
| 1 Shoulder | **D4 + D5** | `sA[1]` + `sB` | yes — one logical joint |
| 2 | none | — | **no** — `jointArg` rejects it |
| 3 Elbow | D6 | `sA[3]` | yes |
| 4 Wrist pitch | D9 | `sA[4]` | yes |
| 5 Wrist roll | D10 | `sA[5]` | yes |
| 6 Gripper | D11 | `sA[6]` | yes |

**Six addressable logical joints, seven physical servos.** Slot 2 is allocated but pinless; every pin access is guarded on `PIN_A[i] >= NUM_DIGITAL_PINS`, so a loop that forgets to skip slot 2 still cannot drive anything. `j[2].en` can never become true, because `enableJoint` returns before setting it.

## 6. Shoulder mirroring and offset order — correction C2

```cpp
static int16_t mirrorC(int16_t leftC) {          // :400
  if (mirMode != MIR_INV) return leftC;
  int32_t r = 18000 + 2L*mirrorOffsetC - leftC;
  if (r < 0) r = 0;  if (r > 18000) r = 18000;
  return (int16_t)r;
}
```

Order inside `writeJoint` (`:415-437`):

1. `setC = clampToLimits(i, setC)` — **logical clamp first**
2. `uA = degCToUs(setC)` and `uB = degCToUs(mirrorC(setC))` — both converted **before** the critical section
3. `noInterrupts()` → both `writeMicroseconds` → `interrupts()` — the pair updates atomically so the Timer1 ISR cannot pulse one shoulder servo at a new angle while the other still holds the old one

`enableJoint` (`:481-492`) does the same, and attaches the pair inside `noInterrupts()` for the same reason.

**The spec's §6a requirement — enforce limits in logical space before mirroring, then clamp the physical result — is already satisfied by construction.** `clampToLimits` runs first; `mirrorC` clamps its own output to `[0, 18000]`. `doMir` additionally refuses an `INV` offset whose image of joint 1's whole `MIN..MAX` would leave `0..180` (`:664-677`).

**Task 4 has nothing to add here. It must verify and not disturb it.** The spec should say so rather than describing existing behaviour as new work.

## 7. Parser behaviour — correction C4

| Case | Behaviour | Evidence |
|---|---|---|
| Missing arguments | `ERR E2 <VERB> N=<count>` | `badArgc()` `:847`, per-verb `tokc` checks |
| Excess arguments | same; `>4` tokens caught at `:965` | `handleLine` |
| **Trailing garbage** | **already rejected** — `parseInt` returns false on *any* non-digit | `:280-293` |
| Non-numeric | `ERR E3 <VERB> …` | `intArg` `:838` |
| Overflow | `parseInt` rejects `> 100000` | `:288` |
| Overlong line | `LINE_MAX = 48`; sets `lineOver`, discards through the terminator, emits `ERR E8 LINE` and **never acts on a truncated line** | `:1063-1068`, `:1050-1053` |
| Unknown verb | verb must be exactly 3 chars `A–Z` → `ERR E1 VERB TOKEN=<v>` | `:951-957` |
| Blank line | silently ignored, so CRLF cannot double-fire | `:938`, `:1049` |
| Case | `upcase()` on the verb and on `MIR`'s mode word | `:940`, `:636` |
| `!` and `?` | intercepted **before** line assembly, never queue behind a partial line, and **do not feed the watchdog** | `:1043-1044`, `:969-980` |

**Consequence for Task 3 Step 4:** the conditional "fix `intArg` if it accepts `90abc`" is unnecessary. `parseInt` already rejects it. The step becomes verify-only.

**Error-code space:** `E1`–`E13` are in use (`E8` is emitted as the whole literal `ERR E8 LINE` at `:1053`). **`E14` is free**, as the plan assumed.

## 8. Interpolation, replacement and queueing — answers the review's question directly

```cpp
TICK_MS = 20; TICK_CAP_MS = 200; MAX_STEP_C = 200;   // :177-188
```

Every 20 ms, for each enabled joint:

```
maxStep = dps * elapsed_ms / 10          (centidegrees; elapsed capped at 200 ms)
maxStep = min(maxStep, MAX_STEP_C=200)   (2.00 degrees per write, hard)
setC   += clamp(tgtC - setC, ±maxStep)
writeJoint(i) if it moved
```

**There is no command queue in the firmware.** `tgtC` is a single, per-joint, freely replaceable target. `MOV` overwrites it; `STP` sets `tgtC = setC`; `LIM` on a disabled joint drags both inward; `disableJoint` sets `tgtC = setC`.

> **Review question — "can JOG be one replaceable active state rather than queued MOV operations?"**
> **Yes, natively.** The firmware already models exactly one replaceable active target per joint. `JOG` needs to add only an *arming flag plus a timestamp*, not a queue. **All queueing risk is on the console side** (§9, correction C9).

Elapsed-time integration means a stall produces a proportionally larger next step, so `deg/s` is honoured across the ~36 ms transmit burst of a `STA` reply. `MAX_STEP_C` then bounds the resulting *step*, which is what actually moves the arm.

## 9. Console command path — corrections C8, C9

```
RX_POLL_MS = 60      PING_MS = 250       STATUS_MS = 250     MOVE_PUMP_MS = 120
REPLY_TIMEOUT = 4000 WATCHDOG_MS = 4000  OUTBOX_MAX = 6      DESYNC_LIMIT = 2
```

`pumpOutbox()` (`:818`) sends **only when `liveAwaiters() === 0`**. One command in flight at a time, strictly FIFO, each awaiting its `OK`/`ERR` terminator.

> **Review question — "can browser commands arrive concurrently or out of order?"**
> **Not from the console.** Single-in-flight FIFO makes reordering structurally impossible. The one bypass is `sendBang()` (`:845`), which writes the raw `!` outside the queue — by design, and safe because the firmware intercepts `!` before line assembly.

**C9 — `trimOutbox` coalesces only `PNG` and `STA`** (`:782-800`). A `JOG` heartbeat is neither, so a backed-up queue would **accumulate and then replay stale `JOG` commands** — a *motion* command replaying, which is materially worse than a stale `PNG`. `JOG` must be added to the coalescing rule, newest-per-joint.

**C8 — the heartbeat budget is tighter than the plan assumed.** The link already carries `PNG` every 250 ms and `STA` every 250 ms. Adding `JOG` makes a three-command rotation, each serialised, each quantised by the bridge's 60 ms `/rx` poll, with a `STA` reply costing ~36 ms of transmit on its own. Realistic rotation: **180–300 ms**.

A 200 ms heartbeat against a 600 ms timeout therefore has only ~2–3 missed-beat margin *in the good case*, and a single queue hiccup could false-trip a jog the operator is still holding. **Recommend heartbeat 250 ms / timeout 1000 ms** — still 4× tighter than the existing 4000 ms `WDG`, and at 30 °/s a 1000 ms overrun is 30°, bounded by the envelope in any case.

**Background tabs:** the console already documents (`:463-487`) that Chrome and Edge clamp `setInterval` in a background tab to roughly once per minute. A jog heartbeat **will** die when the tab is hidden. That is the correct outcome — motion stops and holds — and the console already raises a persistent banner on hide. The jog timeout is *gentler* than the existing watchdog, which detaches everything and makes a loaded arm sag.

## 10. Reply grammar

Zero or more data lines, then **exactly one terminator**:

```
OK <VERB> [key=value ...]
ERR <Ecode> <VERB> [JOINT=<i>] [key=value ...]
```

Asynchronous lines that are **not** terminators: `RDY …` (boot, once), `EVT …`, and `;` comments. Host rule is "accumulate until `OK` or `ERR`".

Verified shapes:

```
OK VER NAME=FACTORYLM-ARM PROTO=1.0 FW=1.0.0 JOINTS=6 BUILD=20260801
OK PNG UP=<ms>
STA J<i> EN=<0|1> SET=<deg> TGT=<deg> MIN=<deg> MAX=<deg> CAL=<0|1> DPS=<n> MOV=<0|1>
SYS ES=<0|1> WD=<0|1> WDMS=<n> MIR=<SAME|INV|UNKNOWN> UP=<ms> UNCAL=<n>
OK STA N=6
LIM J<i> MIN=<deg> MAX=<deg> CAL=<0|1>
OK LIM J<i> MIN=<deg> MAX=<deg> CAL=<0|1>
OK MOV J<i> REQ=<deg> SET=<deg> CL=<0|1>
OK ENA J<i> ADOPT=<deg>
OK MIR MODE=<name> OFF=<deg>
EVT ESTOP SRC=<CMD|RT|WDG>
EVT WDOG MS=<elapsed>
```

`SET=` means **two different things** and the console already handles the distinction (`pumpMoves`, `:1170-1177`): in `OK MOV` it is the *accepted target after clamping*; in `STA` it is the *current commanded angle*. `STA` is the only writer of `j.set`.

## 11. `millis()` wraparound — correction C7

Every existing comparison uses unsigned subtraction and is rollover-safe:

```cpp
if ((uint32_t)(now - lastTickMs) >= TICK_MS)      // :1078
uint32_t el = (uint32_t)(now - lastRxMs);         // :1106
```

The plan's jog check `(uint32_t)(nowMs - j[i].jogMs) > JOG_TIMEOUT_MS` matches that idiom and is safe.

**But the plan's sentinel is not.** It uses `jogMs == 0` to mean "not jogging". `millis()` returns exactly `0` once per ~49.7 days, so a jog armed on that tick would be treated as disarmed and never time out. Low probability, trivially avoided: **use a separate `bool jogActive` and only consult `jogMs` when it is true.** Cost is 7 bytes of SRAM.

## 12. Bridge concurrency — new finding C13

```python
from http.server import ThreadingHTTPServer      # :117
class BridgeServer(ThreadingHTTPServer):         # :454
    daemon_threads = True                        # :460
```

`Link.tx()` (`:396-407`) takes `self._lock` **only to read `self._ser`**, releases it, and then calls `ser.write()` **outside the lock**.

Two concurrent `/tx` requests can therefore interleave bytes mid-line on the serial link. The console cannot trigger this on its own — its single-in-flight FIFO serialises everything — but a **second browser tab holding the same token**, or any other client, could. The result would be a corrupted line, which the firmware would reject as `E1`/`E3`, or worse, silently reassemble into a different valid command.

Not exploited by anything today, and **out of scope for this feature**. Recorded as a risk. The fix is one line — hold the lock across the write — and belongs in its own commit.

## 13. State-clearing rules, as they exist today

| Event | `setC` | `tgtC` | `en` | latch |
|---|---|---|---|---|
| `ENA i adopt` | ← `clampToLimits(adopt*100)` | ← same | true | — |
| `MOV i deg` | untouched | ← `clamp(deg)*100` | — | refused if latched (`E7`) |
| `STP` | untouched | ← `setC`, every enabled joint | — | not a latch |
| `SPD i dps` | untouched | untouched | — | — |
| `LIM i …` (disabled only) | dragged into the new envelope `:600-602` | ← `setC` | — | — |
| `DIS i` / `DIS A` | untouched | ← `setC` (`:463`) | false | — |
| `EST` / `!` / watchdog | untouched | ← `setC` via `disableJoint` | false, all | **latched** |
| `CLR` | untouched | untouched | — | cleared, no precondition |
| **reset / reconnect** | ← `DEF_SET_C` 9000 | ← 9000 | false, all | cleared |
| reset also resets | `minD/maxD` → 70/110, `dps` → 30, `cal` → false, `wdgMs` → 0, `mirMode` → UNKNOWN | | | |

**Reset wipes everything.** This is why the spec's §9 acknowledgment gate is necessary rather than defensive.

**Rules `JOG` must add** (for Task 5):

| Event | `jogActive` |
|---|---|
| `JOG i ±1` | set true, `jogMs = millis()` |
| `JOG i 0` | cleared |
| `STP` (bare) | cleared **for every joint** |
| `STP i` | cleared for joint `i` |
| `MOV i` | cleared for joint `i` — a finite move must not inherit the timeout |
| `LIM i` accepted on a driven joint | left alone; the target is clamped, the jog continues within the new envelope |
| `DIS i` / `DIS A` | cleared |
| `EST` / `!` / watchdog | cleared for every joint |
| timeout fires | cleared for that joint, `tgtC = setC`, `EVT JOGTIMEOUT J<i>` |
| reset | cleared by initialisation |

## 14. Corrections applied to the documents

| # | Correction | Document change |
|---|---|---|
| C1 | `degToCmd` does not exist; `setC`/`tgtC` are centidegrees | plan Task 4 uses `(int16_t)mn * 100` |
| C2 | logical-clamp-then-mirror already structural | spec §6a reworded to verify-and-preserve; plan Task 4 Step 3 likewise |
| C4 | parser already rejects trailing garbage | plan Task 3 Step 4 becomes verify-only |
| C7 | `jogMs == 0` sentinel collides with `millis() == 0` | plan Task 5 uses a separate `jogActive` bool |
| C8 | heartbeat budget tighter than assumed | spec §7a and plan Task 5/9: **250 ms beat, 1000 ms timeout** |
| C9 | `trimOutbox` coalesces only PNG/STA | plan Task 9 must add JOG coalescing |
| C10 | `d0(j)` invented | plan uses `id`, already in scope |
| C11 | `p2()` is nested, not global | plan Task 11 formats the timestamp inline |
| C12 | `sysInfo.fw` does not exist | plan Task 11 stores `FW=` at handshake first |
| C13 | bridge `tx()` writes outside the lock | recorded as an out-of-scope risk |

## 15. Unresolved questions

1. **Is a 1000 ms jog timeout the right number?** Derived from the measured command rotation, not from bench data — nothing has ever jogged this arm. Revisit after Task 13 step 4 with real round-trip figures.
2. **Should `JOG` coalescing keep newest-per-joint or drop the heartbeat entirely when the outbox is non-empty?** Dropping is simpler and arguably more honest — a backed-up queue *is* a stalled host, which is what the timeout exists to catch. Decide in Task 9.
3. **Does the `LIM`-on-driven-joint change interact with an active jog?** Proposed: the target is clamped inward and the jog continues within the new envelope. Needs an explicit harness case in Task 6.
4. **Bridge lock scope (C13)** — fix now in its own commit, or after this feature? Recommend after; it is untriggered by any current client.

## 16. Recommendation

**READY TO PROCEED TO TASK 1**, with the ten document corrections above applied.

No stop condition was hit. Every anchor either verified or was corrected in the documents. `degToCmd` — the plan's one flagged unknown — turned out not to exist at all, which is exactly why this task ran before any code was written.

The most valuable finding is not a defect: **the firmware already models one replaceable target per joint with no queue**, so `JOG` is a small addition rather than a new motion subsystem. The genuine risks are on the console side — heartbeat budget (C8) and `JOG` accumulating in a backed-up outbox (C9).
