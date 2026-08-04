# FactoryLM Arm Serial Protocol v1.0 — Reference

The wire format between `factorylm_arm_controller.ino` (Arduino Uno) and whatever is
driving it — the arm console GUI, the localhost bridge, or a human typing into the
Arduino IDE's Serial Monitor.

**115200 baud, 8N1, line-oriented ASCII.** Every command is human-typable. That is a hard
requirement, not a nicety: it makes the firmware provable with no GUI, and the GUI
debuggable with no firmware.

---

## 0. What this is — and what it is not

This is a supervised hobby bench procedure, the same as everything else in this repo.

> **The real emergency stop is the KCD1 rocker switch and the inline fuse.**
> The on-screen E-STOP button, the `!` byte, the `EST` command, the serial watchdog, this
> firmware, and the Arduino itself are **conveniences, not safety devices.** A browser tab,
> a USB cable, or a Python process can all die mid-move. The only stop you can rely on is
> removing servo power.

Two more things stated up front because they shape everything below:

- **These servos have no position feedback of any kind.** The firmware knows only what it
  last *commanded*. `Servo.read()` returns the last commanded value, not the physical
  angle. The words **position**, **actual**, **measured**, and **feedback** do not appear
  in this protocol and must not appear in any UI built on it. Every angle on the wire is a
  **commanded** angle.
- **The firmware retains nothing across a reset.** No EEPROM, deliberately. Opening the
  serial port resets the board, and it comes back detached, uncalibrated, at defaults.
  The host is responsible for pushing state back in (§4).

---

## 1. The joint map

Joint IDs are the **same indices as `Software/wiring-map.csv`**, so this project has
exactly one numbering system. That matters: `Calibration_Notes/calibration-log.csv`
already contains a correction caused by index/label confusion (index 0 printed "Base",
but the servo physically on D3 was the gripper). A second numbering space would
manufacture that bug again.

| ID | Name | Uno pin(s) | Servo | Type source |
|----|------|-----------|-------|-------------|
| 0 | Base | D3 | MG996R | INFERRED |
| 1 | **Shoulder (PAIR)** | **D4 + D5** | 2× MG996R | INFERRED |
| 2 | **RESERVED — never addressable** | — | — | — |
| 3 | Elbow | D6 | MG996R | INFERRED |
| 4 | Wrist pitch | D9 | MG90S | INFERRED |
| 5 | Wrist roll | D10 | MG90S | **DOC-CONFIRMED** |
| 6 | Gripper | D11 | MG90S | INFERRED |

**Six addressable joints: 0, 1, 3, 4, 5, 6.** Seven physical servos.

### Why ID 2 is reserved

D5 is the *second* servo of the shoulder pair. Joint 1 drives D4 and D5 together as one
logical joint, mirrored, from a single command. **D5 is not independently commandable
from the wire, at all, by construction.**

ID 2 is not silently skipped — it is kept and made to fail loudly. Every command
addressing it returns:

```
ERR E4 <VERB> JOINT=2 RESERVED=shoulder_pair
```

So the gap explains itself the first time someone types `ENA 2`. Two MG996Rs fighting
through one shoulder link is the highest-consequence mechanical failure available on
this arm; the protocol removes the ability to express it rather than warning about it.

---

## 2. Framing and grammar

### Host → firmware

```
VERB[ arg]*\n
```

- `\r` is ignored, so any Serial Monitor line-ending setting works.
- Verbs are 3 letters, accepted case-insensitively, always **echoed uppercase**.
- Arguments are space-separated **signed decimal integers**. There are **no floats
  anywhere on the wire.** Centidegrees and microseconds exist only inside the firmware.
- **Every angle field on the wire is integer degrees.**
- Maximum inbound line: **48 characters including the terminator.** A longer line is
  discarded through the next `\n` and answered `ERR E8 LINE`. A truncated line is never
  acted on.
- The integer parser is strict: any non-digit character, an empty field, or an overflow is
  `ERR E3 NUM`. It does not use `atoi` (which returns 0 for garbage, indistinguishable
  from a real 0). **`DIS A` is the single exception** — the literal `A` is matched before
  integer parsing is attempted.

### Firmware → host: the ACK / NAK contract

This is the whole read-loop contract, and it has no special cases:

> **Every command produces zero or more DATA lines, then EXACTLY ONE terminator line:
> either `OK …` or `ERR …`.**
>
> The host accumulates lines until it sees `OK` or `ERR`. That is the entire algorithm.

- **ACK:** `OK <VERB> [key=value ...]`
- **NAK:** `ERR <Ecode> <VERB> [key=value ...]`

The echoed verb is what correlates a reply to its command. **There are no sequence
numbers** — the host keeps exactly **one command outstanding** and waits for its
terminator before sending the next.

**Asynchronous lines never terminate anything** and are identified by their first token:

| First token | Meaning |
|---|---|
| `RDY` | boot banner, emitted once at the end of `setup()` |
| `EVT` | an event happened (e-stop fired, watchdog tripped) |
| `STA` / `SYS` / `LIM` | data lines belonging to a `STA` or `LIM` reply |
| `;` | free-text comment / help line — always ignorable |

A host that treats `EVT` or `;` as a terminator will hang. A host that ignores `EVT` will
miss an e-stop it did not initiate.

---

## 3. Command table

| Command | Arguments | Terminator on success | What it does |
|---|---|---|---|
| `VER` | — | `OK VER NAME=… PROTO=… FW=… JOINTS=… BUILD=…` | Identify. The handshake gate. |
| `PNG` | — | `OK PNG UP=<ms>` | Cheapest possible watchdog feed. |
| `STA` | — | `OK STA N=6` | Full state dump. Also feeds the watchdog. |
| `LIM` | — | `OK LIM N=6` | List all limits. |
| `LIM` | `<j> <min> <max> <cal>` | `OK LIM J<j> MIN=… MAX=… CAL=…` | Set one joint's limits. Joint must be **disabled**. |
| `MIR` | `<SAME\|INV\|UNKNOWN> [offset_deg]` | `OK MIR MODE=… OFF=…` | Set the shoulder mirror relation, and for `INV` the axis it mirrors about. Joint 1 must be **disabled**. |
| `ENA` | `<j> <adopt_deg>` | `OK ENA J<j> ADOPT=<deg>` | Adopt-and-enable, fused into one command. |
| `DIS` | `<j>` | `OK DIS J<j>` | Detach one joint and pull its pin(s) LOW. |
| `DIS` | `A` | `OK DIS ALL` | Same, every joint. Not a latch. |
| `MOV` | `<j> <deg>` | `OK MOV J<j> REQ=… SET=… CL=…` | Set the target. Non-blocking. |
| `SPD` | `<j> <dps>` | `OK SPD J<j> DPS=…` | Slew rate, 1–90 degrees per second. |
| `STP` | — | `OK STP` | Abort motion on every enabled joint; hold the last commanded value. Joints stay driven. **Not an emergency stop.** |
| `STP` | `<j>` | `OK STP J<j>` | Abort motion on one joint only. `E4` bad/reserved id, `E6` not enabled. |
| `EST` | — | `OK EST` | E-STOP: detach everything, drive all pins LOW, **latch**. |
| `CLR` | — | `OK CLR` | Clear the e-stop / watchdog latch. |
| `WDG` | `<ms>` | `OK WDG MS=…` | Serial watchdog timeout. `0` = off (the boot default). |
| `HLP` | — | `OK HLP` | Help, as `;` comment lines. |

### The commands that need more than a table row

#### `STP` is a motion abort, not an emergency stop

`STP` cancels the remaining interpolation and holds the last **commanded** value. The channel
stays driven. It does not remove power, does not detach, and cannot know the shaft angle — these
servos report nothing.

`EST` and the serial watchdog are a different thing: they detach every channel and **latch**, and
a de-energised gravity-loaded arm sags. **The rocker switch and the inline fuse are the only
emergency stop.**

Bare `STP` aborts every enabled joint. `STP <j>` aborts one — so a host that releases one control
cannot freeze a joint the operator is not touching. With a single joint enabled the two forms are
indistinguishable; with several live, the difference is the whole point.

#### `ENA <j> <adopt_deg>` — adopt-before-drive

`adopt_deg` is **the operator's by-eye estimate of where that joint is right now.** The
firmware pre-loads exactly that pulse width *before* it attaches, so the very first pulse
the servo ever receives equals its current position and **nothing jumps**.

This is fused into a single command on purpose. There is no bare "enable", no "home", and
no centre-on-boot, so *enabling without adopting is not expressible.* With the arm being
reassembled at unknown joint positions, a servo snapping to 90° is the single
highest-risk event in this project — and it would fire on every enable, not just at boot.

Rejections: `E4` (bad or reserved id), `E7` (latched), `E5` (adopt outside this joint's
MIN..MAX), `E6` (already enabled), `E13` (joint 1 while `MIR=UNKNOWN`).

#### `LIM <j> <min> <max> <cal>` — atomic, and span-checked

Every argument is validated **before any field is written**, so a rejected `LIM` leaves the
previous envelope exactly as it was. There is no reachable state in which `min` has been updated
and `max` has not.

The minimum accepted span is **5°** — `ERR E10 … MINSPAN=5`. Below that the envelope is too tight
to jog inside usefully, and a slipped handle would pin a joint against its own limits with no room
to back off.

Limits are enforced in **logical joint space**. The physical write path applies them before any
mirroring or pulse-width conversion, so the shoulder's second servo is derived from an
already-clamped logical command and is never commanded directly.

#### `MOV <j> <deg>` — clamps and reports; never silently

Out-of-range moves are **accepted, clamped, and flagged** rather than rejected. This
matches the single-servo bench sketch's existing `[limit] … Clamped.` behaviour, so the
operator's mental model carries over.

```
OK MOV J3 REQ=95  SET=95  CL=0      accepted as asked
OK MOV J3 REQ=130 SET=110 CL=1      ACCEPTED BUT CLAMPED
```

This is only acceptable because **`CL=1` is never silent** and both the requested and the
applied angle are on the wire. Any UI built on this protocol **must** surface `CL=1`
visibly. A console that quietly snaps the slider back turns clamp-and-report into exactly
the hazard clamping is accused of being.

`MOV` is **non-blocking**: it sets the target and returns immediately. The interpolator
walks the commanded angle toward it at `DPS` degrees per second. Poll `STA` and watch
`MOV=` to know when it arrived.

#### `STP` vs `EST` — know which one you want

| | `STP` | `EST` (and `!`) |
|---|---|---|
| Targets | frozen at current commanded angle | frozen |
| Servos | **stay attached and holding** | **detached, every pin driven LOW** |
| Latches? | no — the next `MOV` works immediately | **yes** — everything returns `E7` until `CLR` |
| Arm behaviour under gravity | holds | **sags** |

`EST` detaching rather than holding is the deliberate choice. Be honest about the
consequence: a de-energised gravity-loaded arm falls to wherever gravity puts it. That is
why recovery always routes back through `ENA <j> <adopt_deg>` with a **fresh** by-eye
estimate. The staleness after an e-stop is mechanical, not a lost setpoint — the
firmware's numbers survive fine, the arm's real position does not.

#### `MIR <mode> [offset_deg]` — the mirror axis is data too

`offset_deg` is **optional and defaults to 0**, so `MIR INV` on its own still means exactly
what it always meant. It is a signed whole number of degrees, accepted in **±90**.

It only does anything in `INV` mode. There, D5 is commanded as the mirror image of D4 about
**90 + offset_deg degrees**, not about a hardcoded 90:

```
right_centidegrees = (18000 + 2 × offset_centidegrees) − left_centidegrees
```

then clamped into `0..18000` before it reaches the servo. With `offset_deg = 0` that is the
old `18000 − left`. In `SAME` mode the offset is stored but unused: D5 gets D4's angle.

**Why this is an argument and not a constant.** "INV mirrors about exactly 90.00" is an
assumption about how two horns happened to land on a splined shaft. This project refuses
that class of assumption about travel limits, and the mirror axis is the same kind of
unknown: a real offset of 6° left at 0 means the two shoulder servos fight each other by
12° the entire time joint 1 is driven. Measure it (`joint-limits.csv` header block), do not
guess it.

**Every `MIR` command sets the offset** — omitting it sets 0. An offset can never outlive
the mode it arrived with.

`MIR` validates the offset against joint 1's *current* `MIN..MAX` before accepting it. If
either end of that travel would mirror outside `0..180`, the whole command is rejected:

```
ERR E11 MIR OFF=40 MIN=70 MAX=110 MIRROR=out_of_travel
```

That refusal is the point. Without it, some reachable D4 angle would silently clamp D5 and
the pair would fight — discovered with the arm loaded, which is exactly when it strips
gears. Set `LIM 1 …` first, then `MIR`.

**The offset is not reported in `STA`.** `SYS` carries `MIR=SAME|INV|UNKNOWN` only. The
only place the current offset appears on the wire is the `OK MIR … OFF=…` reply. A host
that wants to display it must remember what it sent.

#### `LIM` and `MIR` are refused while the joint is live

Both return `ERR E9 <VERB> JOINT=<j> STATE=enabled` if the affected joint is enabled (`MIR`
always names `JOINT=1`). You cannot move the goalposts underneath a driven joint. Disable
it, change the limits, re-enable with a fresh adopt angle.

`CAL` is set **explicitly** from the `LIM` argument, never inferred. A limits file that
still holds the shipped 70-110 defaults must stay flagged uncalibrated — that flag is the
only thing standing between an operator and a placeholder they believe.

---

## 4. Worked `STA` reply — byte for byte

`STA` emits one `STA` line per joint in ascending ID order, then exactly one `SYS` line,
then the terminator. Field order is exactly as shown.

```
STA J0 EN=0 SET=90 TGT=90 MIN=70 MAX=110 CAL=0 DPS=30 MOV=0
STA J1 EN=0 SET=90 TGT=90 MIN=70 MAX=110 CAL=0 DPS=30 MOV=0
STA J3 EN=1 SET=95 TGT=110 MIN=70 MAX=110 CAL=1 DPS=30 MOV=1
STA J4 EN=0 SET=90 TGT=90 MIN=70 MAX=110 CAL=0 DPS=30 MOV=0
STA J5 EN=0 SET=90 TGT=90 MIN=70 MAX=110 CAL=0 DPS=30 MOV=0
STA J6 EN=0 SET=90 TGT=90 MIN=70 MAX=110 CAL=0 DPS=30 MOV=0
SYS ES=0 WD=0 WDMS=1000 MIR=UNKNOWN UP=48211 UNCAL=5
OK STA N=6
```

There is no `STA J2` line, ever. `N=6` counts addressable joints.

| Field | Meaning |
|---|---|
| `EN` | 1 = attached and driven, 0 = detached |
| `SET` | the angle currently being **commanded**, degrees. Not a measurement. |
| `TGT` | where the interpolator is walking `SET` to |
| `MIN` `MAX` | this joint's limits, degrees |
| `CAL` | 1 = a human measured these limits. 0 = still the 70-110 placeholder. |
| `DPS` | slew rate, degrees per second |
| `MOV` | 1 while `SET != TGT` |

| `SYS` field | Meaning |
|---|---|
| `ES` | 1 = the e-stop / watchdog latch is set. Every `MOV` and `ENA` will return `E7`. |
| `WD` | 1 if the latch currently set was caused by the **serial watchdog** rather than an explicit `EST` / `!`. Cleared by `CLR` along with `ES`. Purely informational — it tells the operator whether the host lost contact or somebody pressed the button. |
| `WDMS` | configured watchdog timeout in ms. `0` = watchdog disabled. |
| `MIR` | `SAME`, `INV`, or `UNKNOWN` — the shoulder mirror relation |
| `UP` | `millis()` since boot |
| `UNCAL` | count of addressable joints with `CAL=0` |

The example above shows `UNCAL=5` because J3 has `CAL=1`. **With the shipped
`joint-limits.csv`, which flags every joint uncalibrated, a real first run reports
`UNCAL=6` and six amber cards.** That is correct, and it is the intended first impression.

---

## 5. Raw single bytes — no newline, no queueing

Two characters are intercepted in the read loop **before line assembly**, so they can
never queue up behind a half-typed line:

| Byte | Hex | Effect |
|---|---|---|
| `!` | `0x21` | **E-STOP.** Emits `EVT ESTOP SRC=RT` then `OK EST`. Identical effect to `EST`. |
| `?` | `0x3F` | Status. Byte-identical output to `STA`. |

Neither takes a newline. Latency is bounded by one `loop()` iteration.

**Neither feeds the serial watchdog**, and that is deliberate — see §8. They are intercepted
*before* line assembly, so they never reach the parser, and the watchdog's whole job is to
notice a host that has stopped speaking the protocol. A host that heartbeats with bare `?`
bytes gets status back and is still declared dead on schedule. **Heartbeat with `PNG\n`.**

**This only works because the firmware contains no `delay()` anywhere.** Any blocking call
silently defeats the realtime e-stop. Note that the single-servo bench sketch *does* use
`delay(1200)` and `delay(1000)` in its `w` and `m` commands — those behaviours must not be
carried into the controller. This belongs on the review checklist for every commit to the
sketch, because a blocking call is invisible right up until the moment it matters.

---

## 6. Asynchronous lines

```
RDY NAME=FACTORYLM-ARM PROTO=1.0 FW=1.0.0
EVT ESTOP SRC=CMD
EVT ESTOP SRC=RT
EVT WDOG MS=1043
; free text comment
```

`RDY` is emitted once at the end of `setup()`. **The host must not wait for it** — see §8.
In practice the connect sequence's flush usually discards it. Log it if you see it.

`EVT` lines are the only way a host learns about a stop it did not initiate. Handle them.

---

## 7. Error codes

Format: `ERR <Ecode> <VERB> [key=value ...]`

Each code means exactly one thing. Where a code once covered several unrelated failures the
GUI could only print a vague message, or a wrong one — a rejected slew rate was being
explained to the operator as "that starting angle is outside this joint's limits". So the
overloaded cases were split out into `E10`, `E11` and `E12`.

**This table is the emitted format, verb for verb and key for key.** `<VERB>` is the
uppercased three-letter verb that caused it.

| Code | Mnemonic | Emitted as | Cause |
|---|---|---|---|
| `E1` | `VERB` | `ERR E1 VERB TOKEN=<token>` | unknown or malformed verb |
| `E2` | `ARGS` | `ERR E2 <VERB> N=<count>` | wrong number of arguments |
| `E3` | `NUM` | `ERR E3 <VERB> ARG=<n>` | argument `n` is not an integer (1-based) |
| `E4` | `JOINT` | `ERR E4 <VERB> JOINT=<id>` | bad joint id. Id 2 also gets ` RESERVED=shoulder_pair` |
| `E5` | `RANGE` | `ERR E5 ENA JOINT=<j> REQ=… MIN=… MAX=…` | adopt angle outside this joint's `MIN..MAX` |
| `E6` | `STATE` | `ERR E6 <VERB> JOINT=<j>` | `MOV` on a disabled joint, or `ENA` on an enabled one |
| `E7` | `ESTOP` | `ERR E7 <VERB> JOINT=<j>` | the e-stop / watchdog latch is set — send `CLR` first |
| `E8` | `LINE` | `ERR E8 LINE` | inbound line exceeded 48 chars; discarded, not acted on |
| `E9` | `MODE` | `ERR E9 <VERB> JOINT=<j> STATE=enabled` | `LIM` or `MIR` while that joint is enabled |
| `E10` | `LIMITS` | `ERR E10 LIM JOINT=<j> …` | the `LIM` operands themselves are illegal |
| `E11` | `MIRARG` | `ERR E11 MIR …` | the `MIR` mode word or offset is illegal |
| `E12` | `SPEED` | `ERR E12 SPD JOINT=<j> REQ=… MIN=1 MAX=90` | slew rate outside 1–90 °/s |
| `E13` | `MIRROR` | `ERR E13 ENA JOINT=1 MIR=UNKNOWN` | tried to enable joint 1 while the mirror is unknown |

`E8` is the one error that does **not** echo a verb: the line was refused *before* it was
read to the end, so there is no verb in it that could be trusted.

The three de-overloaded codes carry the operand that was refused, which is what makes them
worth having:

```
ERR E10 LIM JOINT=3 REQMIN=120 REQMAX=40 LIMIT=0..180 MIN<MAX
ERR E10 LIM JOINT=3 REQCAL=7
ERR E11 MIR MODE=BACKWARDS
ERR E11 MIR OFF=200 LIMIT=+/-90
ERR E11 MIR OFF=40 MIN=70 MAX=110 MIRROR=out_of_travel
ERR E12 SPD JOINT=3 REQ=500 MIN=1 MAX=90
```

More examples, exactly as they appear on the wire:

```
ERR E4 ENA JOINT=2 RESERVED=shoulder_pair
ERR E13 ENA JOINT=1 MIR=UNKNOWN
ERR E7 MOV JOINT=3
ERR E9 LIM JOINT=3 STATE=enabled
ERR E5 ENA JOINT=3 REQ=140 MIN=70 MAX=110
ERR E1 VERB TOKEN=FOO
ERR E2 MOV N=1
ERR E3 MOV ARG=2
```

> **Known overload still outstanding:** `WDG` with a timeout outside `0` / `200`–`10000`
> answers a bare `ERR E5 WDG`, reusing the adopt-angle code. Every other operand-range
> failure now has its own code. Treat `E5` on a verb other than `ENA` as "that number was
> out of range", not as anything to do with joint travel.

**Garbage input NAKs. It never triggers an e-stop.** An e-stop that fires on a typo trains
the operator to ignore e-stops, which is worse than having none.

---

## 8. The watchdog contract

The serial watchdog exists to answer one question: *what happens to a driven arm when the
host dies?*

| | |
|---|---|
| **Command** | `WDG <ms>` |
| **Boot default** | `0` — **disabled** |
| **Legal values** | `0`, or `200`–`10000` |
| **Recommended** | `1000`, pushed by the host at connect |
| **Armed only when** | `WDMS != 0` **AND** at least one joint is enabled |
| **Fed by** | **a complete, well-formed command line that reached a handler — nothing else** |
| **On trip** | `EVT WDOG MS=<elapsed>` then a full e-stop: every channel detached, every pin driven LOW, latched |
| **Recovery** | `CLR`, then `ENA <j> <fresh adopt angle>` per joint |

Three details that are load-bearing:

1. **Raw bytes do not feed it — including `!` and `?`.** The timer is reset in exactly one
   place: after a line has arrived complete, assembled without overflowing, held a legal
   three-letter verb and an acceptable argument count, and been handed to a handler. A
   wedged host spraying garbage, a stuck-high UART line, or a host looping on the bare `?`
   status byte would otherwise feed the watchdog forever while the arm stays driven — the
   exact failure it exists to catch.

   Note the boundary precisely: the line must be **well-formed**, not **accepted**. A
   syntactically valid command that the firmware then refuses — `MOV` on a disabled joint,
   an out-of-range `SPD`, even an unknown three-letter verb — *does* feed the watchdog. The
   host is demonstrably alive and speaking the protocol, which is the only question the
   watchdog asks. Only a line rejected *before* dispatch fails to feed it: one longer than
   48 characters (`E8`), a verb that is not three letters, or more than four arguments.
2. **It only arms when something is actually driven.** A human working at the Arduino
   Serial Monitor with nothing enabled is never tripped by a watchdog they did not ask
   for.
3. **The comparison is rollover-safe** — `(uint32_t)(millis() - lastRx) > wdgMs`, never
   `millis() > lastRx + wdgMs`. `millis()` wraps at about 49.7 days.

The host's side of the contract: send `PNG` every **250 ms** on its own timer. That is a
4× margin against a 1000 ms timeout. `STA` also feeds the watchdog, so the 250 ms status
poll and the playback loop both reinforce it — but do not *rely* on that. Keep the `PNG`
timer independent, so a stalled UI render cannot stop the heartbeat.

---

## 9. Connect handshake — mandatory, both transports

1. **Open the port at 115200.** Opening it asserts DTR, which **resets a genuine Uno**.
2. **Wait 2000 ms.** Optiboot 4.4 waits about a second for an upload before jumping to the
   sketch.
3. **Flush and discard** everything sitting in the receive buffer.
4. **Send `VER`.** Retry up to 3 times at 500 ms intervals.
5. **Send nothing else until the reply contains exactly `NAME=FACTORYLM-ARM`.** On a
   mismatch or a timeout: refuse to connect and display what you actually found.
6. **Push state.** The firmware retains nothing across the reset in step 1. For each joint
   in ascending order send `LIM j min max cal` then `SPD j dps`; then
   `MIR <mode> [offset_deg]`; then `WDG 1000`.

   **`MIR` must come after joint 1's `LIM`.** An `INV` offset is validated against joint
   1's limits *as the firmware currently holds them*, so sending `MIR` first checks it
   against the 70-110 default and can accept an offset that the real limits would refuse
   (or refuse one they would allow). Ascending joint order already does this — joint 1's
   `LIM` goes out long before `MIR` — but do not "optimise" the order.
7. **Start the timers:** `PNG` every 250 ms, `STA` every 250 ms.

### Why step 5 is not boilerplate

COM ports shift on this machine, and there are at least three sketches that could be
flashed to this board. The single-servo bench sketch interprets a bare `a` as *attach* and
a bare digit as *select this pin*. A console that starts sending blind will one day drive
the wrong firmware — and that firmware will happily attach a servo.

### Why to query, never to listen

Do **not** passively wait for the `RDY` banner. DTR is handled differently by every
transport — PowerShell asserts it, pyserial exposes it as a flag, Chrome's Web Serial
handles signals its own way. A banner-waiting connect handler works on one transport and
hangs on another. Always ask.

### If step 6 returns `E9`

`LIM` is refused while a joint is enabled. If the state push hits `ERR E9 LIM … STATE=enabled`,
the board did **not** reset when the port opened — some transports do not assert DTR the
same way — so joints may still be live from a previous session at limits you did not set.
**Surface that to the operator rather than proceeding with stale limits.** `DIS A`, then
retry the push.

---

## 10. `joint-limits.csv` — column reference

Location: `Software/arm-console/joint-limits.csv`. Loaded by the console through a normal
file picker, in both transports.

| Column | Legal values | Goes where |
|---|---|---|
| `joint_id` | `0`, `1`, `3`, `4`, `5`, `6` | the `<j>` argument |
| `joint_name` | free text, no comma | display only |
| `uno_pins` | e.g. `D3`, `D4+D5` | display only — the firmware owns the real pin map |
| `min_deg` | `0`–`180` integer, `< max_deg` | `LIM` arg 2 |
| `max_deg` | `0`–`180` integer, `> min_deg` | `LIM` arg 3 |
| `home_deg` | integer between `min_deg` and `max_deg` | **GUI only — see below** |
| `max_deg_per_sec` | `1`–`90` integer | `SPD` arg 2 |
| `calibrated` | `yes` / `no` | `LIM` arg 4 |
| `mirror_mode` | `same` / `inverted` / `unknown`, or empty | `MIR` arg 1 |
| `notes` | free text, **no comma** | display only |
| `mirror_offset_deg` | signed whole degrees, `-90`–`90`; empty means `0` | `MIR` arg 2 |

### The CSV-to-wire vocabulary map

The file and the wire deliberately use different spellings. Translate them here and
nowhere else:

| CSV | Wire |
|---|---|
| `calibrated=no` | `CAL=0` (4th arg of `LIM`) |
| `calibrated=yes` | `CAL=1` |
| `mirror_mode=same` | `MIR SAME` |
| `mirror_mode=inverted` | **`MIR INV`** — not `INVERTED` |
| `mirror_mode=unknown` *or empty* | `MIR UNKNOWN` |
| `mirror_offset_deg=0` *or empty* | the trailing `0` in `MIR INV 0` — or just omit it |
| `mirror_offset_deg=6` | `MIR INV 6` |

`mirror_mode` and `mirror_offset_deg` are only read from the **joint 1** row. They are
ignored everywhere else, and the shipped file carries `mirror_offset_deg=0` on every row so
the column is never ragged.

`mirror_offset_deg` is a **placeholder at 0, exactly like the 70-110 travel limits.** It is
the number of degrees joint 1's mirror axis sits away from 90, it has never been measured on
this arm, and the procedure for measuring it is in the `joint-limits.csv` header block. A
guessed offset is worse than none, because it looks deliberate.

### `home_deg` never causes motion

`home_deg` is **not sent to the firmware.** There is no `HOM` command and no homing
behaviour anywhere in this system. Its only job is to pre-fill the number box in the
console's ENABLE prompt so the operator has a starting point to correct. The operator
still confirms it, and the confirmed number becomes `ENA <j> <adopt_deg>`.

### Parser rules

These apply identically to the limits file and the waypoints file:

- **Lines whose first non-whitespace character is `#` are skipped. Blank lines are
  skipped.** The shipped `joint-limits.csv` opens with a `#` block explaining how to fill
  it in. Every instruction in that block is repeated in the `notes` column of the relevant
  row, so nothing is lost if a parser drops it — but a parser that treats a `#` line as a
  data row will reject the whole file.
- A header row is **required**. Columns are looked up **by name**, not by position.
  Unknown columns are ignored. Column order does not matter.
- Comparisons are case-insensitive and whitespace-trimmed.
- **No field may contain a comma.** The shipped file uses semicolons and dashes instead.
- A malformed row — bad integer, `min >= max`, `min < 0`, `max > 180`, unknown
  `joint_id`, `joint_id 2`, or a `mirror_offset_deg` that is not a whole number in
  `-90..90` — **rejects the whole file**, with a visible message naming the row number and
  the problem. Never partially load. A half-applied limits file is worse than none at all.

  The offset belongs in that list even though the firmware checks it too. If the loader
  passes `200` through, the file appears to load cleanly and the failure surfaces later as
  `ERR E11` in the middle of the connect-time state push — a half-pushed board, which is
  the exact failure mode "never partially load" exists to prevent. Reject it at the file.
  Note the loader can only range-check it; whether the offset is *correct* is a bench
  measurement no parser can verify.
- **If no file loads, every joint stays at 70-110 with `CAL=0` and its amber
  UNCALIBRATED badge.** That is the correct, safe, honest fallback — so a rejected file is
  loud and annoying, never dangerous.
- If a loaded file widens a joint past 70-110, show a one-line informational notice
  (`J0 Base widened to 45-135 from the file`). Do not block it. **A human editing that CSV
  is the calibration act.**

---

## 11. Waypoints CSV — column reference

```
step,label,base_deg,shoulder_deg,elbow_deg,wrist_pitch_deg,wrist_roll_deg,gripper_deg,dwell_ms
```

| Column | Joint | Meaning |
|---|---|---|
| `step` | — | ordinal, ascending |
| `label` | — | free text, no comma |
| `base_deg` | J0 (D3) | integer degrees, or `-` |
| `shoulder_deg` | J1 (D4+D5) | integer degrees, or `-` |
| `elbow_deg` | J3 (D6) | integer degrees, or `-` |
| `wrist_pitch_deg` | J4 (D9) | integer degrees, or `-` |
| `wrist_roll_deg` | J5 (D10) | integer degrees, or `-` |
| `gripper_deg` | J6 (D11) | integer degrees, or `-` |
| `dwell_ms` | — | milliseconds to hold after the step completes |

**An empty cell or `-` means "leave this joint alone"** — no `MOV` is sent for it. That is
how a waypoint that only closes the gripper stays a one-joint move.

**Six columns, one per logical joint.** There is no separate shoulder-left and
shoulder-right column, because there is no way to command them separately.

Playback lives entirely in the host. The firmware stays dumb: per step the host sends one
`MOV` per non-`-` joint, waits for each `OK`, then polls `STA` until every commanded joint
reports `MOV=0`, then waits `dwell_ms`. An e-stop aborts between any two operations.

> `Software/conveyor-waypoints-template.csv` is the older 7-physical-column format and is
> superseded by this one. It contains zero rows, so there is nothing to migrate.

---

## 12. Worked session — exact bytes

`>` is host → firmware. `<` is firmware → host. Every line ends in `\n` (`0x0A`) unless
stated otherwise. **Servo power is OFF for this entire session** — see §13.

### Step 0 — open the port

Open COM*n* at 115200 8N1. DTR asserts and the Uno resets. Wait **2000 ms**. Flush the
receive buffer. The `RDY` banner is usually discarded by that flush — that is fine and
expected.

### Step 1 — identify (the gate)

```
>  VER\n
      56 45 52 0A
<  OK VER NAME=FACTORYLM-ARM PROTO=1.0 FW=1.0.0 JOINTS=6 BUILD=20260801\n
```

`NAME=FACTORYLM-ARM` present → proceed. Anything else → refuse to connect.

### Step 2 — push state (14 commands)

```
>  LIM 0 70 110 0\n
<  OK LIM J0 MIN=70 MAX=110 CAL=0\n
>  SPD 0 30\n
<  OK SPD J0 DPS=30\n

>  LIM 1 70 110 0\n
<  OK LIM J1 MIN=70 MAX=110 CAL=0\n
>  SPD 1 30\n
<  OK SPD J1 DPS=30\n

      ... the same LIM / SPD pair for joints 3, 4, 5, 6, in ascending order ...

>  MIR UNKNOWN\n
<  OK MIR MODE=UNKNOWN OFF=0\n
>  WDG 1000\n
<  OK WDG MS=1000\n
```

The offset argument was omitted, so it defaulted to 0 and the reply says so. Once the
shoulder has been measured that line becomes `MIR INV 6` and answers
`OK MIR MODE=INV OFF=6`.

Then start the two timers. The heartbeat looks like this, four times a second, forever:

```
>  PNG\n
<  OK PNG UP=2431\n
```

### Step 3 — enable the elbow

The operator looks at the arm, judges the elbow to be at about 95°, and types it in.

```
>  ENA 3 95\n
<  OK ENA J3 ADOPT=95\n
```

The firmware **must** pre-load the adopt pulse width (~1524 µs here) *before* calling
`attach()`, so D6 begins pulsing at exactly the adopted angle. Nothing jumps. A bare
`attach()` would command ~1500 µs centre on the very next frame — that is the
snap-to-centre this whole design exists to prevent, and it would fire on every enable, not
just at boot. With servo power off nothing moves either way — but
D6 is now emitting real pulses, which is what §13 measures.

### Step 4 — move it

```
>  MOV 3 110\n
<  OK MOV J3 REQ=110 SET=110 CL=0\n
```

`MOV` returned immediately. The interpolator is now walking 95 → 110 at 30 °/s, about
half a second. Polling mid-move:

```
>  STA\n
<  STA J0 EN=0 SET=90 TGT=90 MIN=70 MAX=110 CAL=0 DPS=30 MOV=0\n
<  STA J1 EN=0 SET=90 TGT=90 MIN=70 MAX=110 CAL=0 DPS=30 MOV=0\n
<  STA J3 EN=1 SET=103 TGT=110 MIN=70 MAX=110 CAL=0 DPS=30 MOV=1\n
<  STA J4 EN=0 SET=90 TGT=90 MIN=70 MAX=110 CAL=0 DPS=30 MOV=0\n
<  STA J5 EN=0 SET=90 TGT=90 MIN=70 MAX=110 CAL=0 DPS=30 MOV=0\n
<  STA J6 EN=0 SET=90 TGT=90 MIN=70 MAX=110 CAL=0 DPS=30 MOV=0\n
<  SYS ES=0 WD=0 WDMS=1000 MIR=UNKNOWN UP=9884 UNCAL=6\n
<  OK STA N=6\n
```

`MOV=1`, and `SET` is partway. Half a second later the same poll shows
`STA J3 EN=1 SET=110 TGT=110 … MOV=0`. That transition is how playback knows a step
finished.

### Step 5 — ask for something out of range

```
>  MOV 3 130\n
<  OK MOV J3 REQ=130 SET=110 CL=1\n
```

`OK`, not `ERR` — the command was accepted. But `REQ=130 SET=110 CL=1` says plainly that
110 is what will actually be commanded. **The console must show this**, not swallow it.

### Step 6 — e-stop, the realtime way

One byte. No newline. It does not queue behind anything.

```
>  !
      21
<  EVT ESTOP SRC=RT\n
<  OK EST\n
```

Every channel is now detached and every signal pin is driven LOW. The latch is set. Note
that `EVT` arrived *before* the terminator — that is normal, and it is why `EVT` must
never be treated as a terminator.

### Step 7 — the latch means what it says

```
>  MOV 3 100\n
<  ERR E7 MOV JOINT=3\n
>  ENA 3 95\n
<  ERR E7 ENA JOINT=3\n
```

### Step 8 — recover deliberately

```
>  CLR\n
<  OK CLR\n
```

**Now go and look at the arm.** It was de-energised while latched; under load it has
sagged. The adopt angle you type next must be a **fresh** by-eye estimate, not the number
from before the stop.

```
>  ENA 3 88\n
<  OK ENA J3 ADOPT=88\n
```

### Other replies worth recognising

```
>  ENA 2 90\n
<  ERR E4 ENA JOINT=2 RESERVED=shoulder_pair\n

>  ENA 1 90\n
<  ERR E13 ENA JOINT=1 MIR=UNKNOWN\n

>  MOV 4 100\n
<  ERR E6 MOV JOINT=4\n                        (joint 4 is not enabled)

>  ENA 3 140\n
<  ERR E5 ENA JOINT=3 REQ=140 MIN=70 MAX=110\n

>  SPD 3 500\n
<  ERR E12 SPD JOINT=3 REQ=500 MIN=1 MAX=90\n

>  MIR SIDEWAYS\n
<  ERR E11 MIR MODE=SIDEWAYS\n

>  MIR INV 40\n
<  ERR E11 MIR OFF=40 MIN=70 MAX=110 MIRROR=out_of_travel\n
                                             (mirroring 70..110 about 130 lands outside 0..180)

>  MIR INV 5\n
<  OK MIR MODE=INV OFF=5\n

>  DIS A\n
<  OK DIS ALL\n
```

---

## 13. The zero-power acceptance test

**Every part of this system is provable with no servo connected and no servo power.** That
is deliberate: the only supply on hand is a 6.62 V 700 mA adapter, which runs one unloaded
servo and cannot begin to drive an assembled arm. Do all of this first.

Setup: Uno on USB. **No servos plugged in. Servo supply disconnected. KCD1 rocker OFF.**
A multimeter on DC volts.

| # | Do this | Expect exactly |
|---|---|---|
| 1 | Upload `factorylm_arm_controller`. Open Serial Monitor at 115200. | The `;` banner, then `RDY NAME=FACTORYLM-ARM PROTO=1.0 FW=1.0.0` |
| 2 | Probe D3, D4, D5, D6, D9, D10, D11 against GND | **~0 V on all seven.** Nothing is attached at boot. |
| 3 | Type `VER` | `OK VER NAME=FACTORYLM-ARM PROTO=1.0 FW=1.0.0 JOINTS=6 BUILD=20260801` |
| 4 | Type `STA` | 6 `STA` lines (no `J2`), one `SYS` line with `UNCAL=6`, then `OK STA N=6` |
| 5 | Type `ENA 2 90` | `ERR E4 ENA JOINT=2 RESERVED=shoulder_pair` |
| 6 | Type `ENA 1 90` | `ERR E13 ENA JOINT=1 MIR=UNKNOWN` |
| 7 | Type `MOV 3 100` | `ERR E6 MOV JOINT=3` — disabled joints refuse to move |
| 8 | Type `ENA 3 90` | `OK ENA J3 ADOPT=90` |
| 9 | **Probe D6 against GND** | **~0.37 V.** Pulses are being generated. |
| 10 | Probe D3, D4, D5, D9, D10, D11 | **still ~0 V.** Only the joint you enabled is driven. |
| 11 | Type `MOV 3 110`, then `STA` a few times | `OK MOV J3 REQ=110 SET=110 CL=0`, then `SET` climbing 90→110 with `MOV=1`, settling at `MOV=0` |
| 12 | Type `MOV 3 130` | `OK MOV J3 REQ=130 SET=110 CL=1` — **clamped and said so** |
| 13 | Type `MOV 3 0` | `OK MOV J3 REQ=0 SET=70 CL=1` |
| 14 | Type `!` (no Enter) | `EVT ESTOP SRC=RT` then `OK EST` |
| 15 | **Probe D6 again** | **~0 V.** This is the detach-then-drive-LOW check — see below. |
| 16 | Type `MOV 3 100` | `ERR E7 MOV JOINT=3` |
| 17 | Type `CLR`, then `ENA 3 90`, then `DIS A` | `OK CLR` / `OK ENA J3 ADOPT=90` / `OK DIS ALL` |
| 18 | Probe all seven pins | **~0 V.** |
| 19 | Type `FOO` | `ERR E1 VERB TOKEN=FOO` — and **no e-stop** |
| 20 | Type a 60-character line of junk | `ERR E8 LINE` — and no action taken |
| 21 | Type `MIR SIDEWAYS` | `ERR E11 MIR MODE=SIDEWAYS` — the mode word is checked |
| 22 | Type `MIR INV 200` | `ERR E11 MIR OFF=200 LIMIT=+/-90` |
| 23 | Type `MIR INV 40` | `ERR E11 MIR OFF=40 MIN=70 MAX=110 MIRROR=out_of_travel` — at the default 70-110 limits, mirroring about 130° would put D5 at 190° |
| 24 | Type `MIR INV 5`, then `STA` | `OK MIR MODE=INV OFF=5`, then `SYS … MIR=INV …`. **`SYS` shows the mode but never the offset** |
| 25 | Type `ENA 1 90`, then **probe D4 and D5** | `OK ENA J1 ADOPT=90`, then **~0.37 V on D4 and ~0.39 V on D5** — both pulsing, neither at 0 V or 5 V. D5 is not identical to D4: at `OFF=5` it is commanded to 100°, which is the mirror doing its job |
| 26 | Type `DIS 1`, then probe D4 and D5 | `OK DIS J1`, then **~0 V on both.** Never one and not the other |
| 27 | Type `MIR UNKNOWN`, then `ENA 1 90` | `OK MIR MODE=UNKNOWN OFF=0`, then `ERR E13 ENA JOINT=1 MIR=UNKNOWN` — the lock comes back |

### Why step 15 is the most important line in this table

The ~0.37 V figure comes from `V2-SERVO-POWER-AND-WIRING.md` §6: a ~1472 µs pulse every
20 ms is a 7.4 % duty cycle, and 7.4 % of 5 V is 0.37 V. A cheap meter averaging 50 Hz PWM
wanders either side of that — **the useful distinction is "about a third of a volt" vs
"zero" vs "about five".**

**"About five" at step 15 is a failure, and it is the failure that matters most.** The
Arduino Servo library's interrupt handler drives a channel's pin HIGH when its slot comes
up and only drives it LOW at the *next* interrupt, gated on an `isActive` flag that
`detach()` clears immediately. If a `detach()` lands inside that window, the LOW write is
skipped and **the pin stays driven HIGH forever.** A continuously-high signal is an
infinitely long pulse: servos hold, hunt, or slew to an endpoint and stall at full current.

Every safety mechanism in this system routes through detach — boot, per-joint disable, the
watchdog, the e-stop, aborting playback. The firmware's fix is that `detach()` is *always*
immediately followed by `pinMode(OUTPUT)` + `digitalWrite(LOW)`. **This bug is silent and
intermittent: it will pass casual testing and fail during a real e-stop with the arm
loaded.** Steps 2, 10, 15 and 18 are the only place it gets caught. Do not skip them.

Only once every row above passes should a single unloaded servo be connected — and even
then, one joint at a time, per `HOW-TO-TEST-EACH-SERVO.md`.

---

## 14. Calibrating a joint, and the shoulder mirror

### A joint

Use the existing single-servo bench sketch — not this controller — with the horn off and
the joint free. Follow `HOW-TO-TEST-EACH-SERVO.md`. Find where the joint actually stops
each way, back off a few degrees from each end, write those two numbers into
`joint-limits.csv` with `calibrated=yes`, log the same numbers in
`Calibration_Notes/calibration-log.csv`, and reload the file in the console.

Do not set `calibrated=yes` on a row you did not measure. The amber badge is the only
thing telling the operator that 70-110 is a guess.

### The shoulder mirror — this one blocks joint 1 entirely

Joint 1 drives D4 and D5 as one logical joint. The firmware rate-limits the *logical*
joint and mirrors at the single point of write, so the two servos can never transiently
disagree. But it needs to know **which relation** to apply:

| `mirror_mode` | Wire | What D5 is commanded |
|---|---|---|
| `same` | `MIR SAME` | the same pulse width as D4 |
| `inverted` | `MIR INV [offset]` | mirrored about `90 + offset` — at offset 0, D4 at 70° means D5 at 110° |

**Both are plausible, and which one is correct depends on how the horns were physically
fitted — which nobody recorded.** Getting it backwards puts two MG996Rs in opposition
through one link the instant both are enabled, and strips gears.

So there is **no default.** `MIR` boots to `UNKNOWN` and `ENA 1 <deg>` returns
`ERR E13 ENA JOINT=1 MIR=UNKNOWN` until a human decides.

To decide it, use the single-servo bench sketch, which already documents centring D4 and
D5 **separately and never paired**. Work out whether the two servos must receive the same
angle or opposite angles to move the shoulder link the same way. Then set `mirror_mode` in
the joint 1 row of `joint-limits.csv` and reload.

### If it is `inverted`, there is a second number to measure

`inverted` alone still assumes the two servos mirror about **exactly 90°**, and that is an
assumption about how two horns landed on a splined shaft. An MG996R spline steps in whole
teeth roughly 18° apart, so a perfectly symmetrical fit is luck, not the default.

If the real mirror axis is 96° and `mirror_offset_deg` is left at 0, the two shoulder
servos are commanded 12° apart from the truth **the whole time joint 1 is enabled** — they
hold against each other, run hot, and eventually strip a gear. Nothing on screen shows it,
because neither servo reports anything back.

Measure it the same way, with the linkage unbolted and D4 and D5 driven separately: find
the commanded angle `a` for D4 and `b` for D5 at which the two output shafts point the same
physical way, then

```
mirror_offset_deg = (a + b) / 2 − 90
```

rounded to a whole degree. Both at 90 gives 0. `a=84`, `b=108` gives 6. Write it in the
joint 1 row, log how you measured it in `Calibration_Notes/calibration-log.csv`, and reload.

Leave it at 0 until it has actually been measured. **A guessed offset is worse than no
offset, because it looks deliberate.**

---

## 15. Known limits — stated plainly

**No position feedback.** These servos have none. Every `SET` on the wire is a *commanded*
angle. With a marginal supply the servo may not have got there at all, and the operator
must never be led to believe otherwise.

**The AVR hardware watchdog is deliberately omitted in v1.** It is safe on this board —
Optiboot 4.4 reads and zeroes `MCUSR` and disables the WDT before the sketch starts, so
there is no reset loop. It is left out because the serial watchdog already covers the
failure mode we care about (the host dies), and because a WDT reset drops every pin to
INPUT, which means a gravity-loaded arm goes limp and *falls*, and the adopted positions
are lost. Revisit once the arm is assembled and it is clear whether a limp-arm failsafe or
a held-pose failsafe is the lesser evil.

**Opening the Arduino Serial Monitor while the console or the bridge holds the port steals
the port and DTR-resets the board — mid-move if there is a move in progress.** This will
happen during debugging. Close the console first.

**The present 6.62 V supply is out of spec for the three MG90S units** — wrist pitch (D9),
wrist roll (D10), gripper (D11), all rated 4.8–6.0 V. Do not use it on those three even
for single-servo bring-up. It also cannot drive an assembled arm at all: a reassembled
7-servo arm holding its own weight needs 3–5 A and this adapter supplies 700 mA.

**Six of seven servo types are inferred from the BOM.** Only wrist roll (D10) is
doc-confirmed. Nothing in the firmware varies by servo type, and nothing should until
that is confirmed — if a channel assumed to be MG996R is actually an MG90S, the current
supply is already over its rated maximum.

**Default slew rate is 30 °/s across the board and has never been tested against a loaded
joint.** It crosses the whole default 70-110 envelope in about 1.3 s. Rate limiting is a
*power* feature, not a comfort feature: a bare step command makes a servo drive at full
speed, and an MG996R can pull up to 2.5 A doing it. Seven simultaneous steps is the most
likely cause of a brownout even on the incoming 3–5 A supply. Raise it per joint only
after the real supply is in place and current draw has actually been measured.

**Power doctrine is unchanged and non-negotiable.** USB powers the Uno; a separate
regulated supply powers the servos; **only the grounds join.** Nothing from the supply's
positive side touches Uno `5V`, `VIN`, or the barrel jack. The missing common ground was
the root cause of the entire earlier no-motion episode — one wire, any Arduino `GND` to
the supply negative. Keep the 470–1000 µF capacitor near the servos, and keep servo
current off the breadboard.

---

## Files that go with this one

| File | What it is |
|---|---|
| `Software/factorylm_arm_controller/factorylm_arm_controller.ino` | the firmware that speaks this protocol |
| `Software/arm-console/arm-console.html` | the host console |
| `Software/arm-console/arm-bridge.py` | the localhost serial bridge |
| `Software/arm-console/joint-limits.csv` | the limits data (§10) |
| `Software/wiring-map.csv` | the pin/index map the joint IDs come from |
| `Software/emre_kalem_single_servo_bench_test/` | the proven single-servo sketch — the calibration tool |
| `Documentation/HOW-TO-TEST-EACH-SERVO.md` | the per-servo bench procedure |
| `Documentation/V2-SERVO-POWER-AND-WIRING.md` | power architecture and the 0.37 V measurement |
| `Calibration_Notes/calibration-log.csv` | where measured results go |

`factorylm_arm_controller` is **ours**. The vendor never shipped
`emre_kalem_arm_calibrate.ino` or `emre_kalem_arm_uno_controller.ino`; those filenames
must not be reused, because we would be claiming provenance we do not have.
