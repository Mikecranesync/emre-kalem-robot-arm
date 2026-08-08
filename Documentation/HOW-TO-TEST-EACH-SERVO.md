# How To Test Each Servo — Bench Card

Plain-English procedure for `emre_kalem_single_servo_bench_test`.
One servo at a time, every time.

---

## The one rule that explains the numbers

**The number you type picks a PIN, not a motor.**

| Type | Drives pin | Which joint it's meant to be | Servo type |
|---|---|---|---|
| `0` | D3 | Base | MG996R |
| `1` | D4 | Shoulder left | MG996R |
| `2` | D5 | Shoulder right | MG996R |
| `3` | D10 | Elbow | MG996R |
| `4` | D6 | Wrist pitch | MG90S |
| `5` | D9 | Wrist roll | MG90S |
| `6` | D11 | Gripper / claw | MG90S |

The sketch prints "Base" when you type `0` because that's what D3 is *supposed* to be.
If you plug a different motor into D3, it still says "Base". It's a label, not a
detection.

**Easiest habit: wire each servo to its own pin from the table.** Then the names on
screen match reality and your log stays honest.

---

## Voltage — read this before choosing which servos to test

Your supply measures **6.62 V**.

| Servo | Safe range | On 6.62 V |
|---|---|---|
| MG996R — base, both shoulders, elbow (D3, D4, D5, D10) | 4.8 – 7.2 V | **fine, test these now** |
| MG90S — wrist pitch, wrist roll, gripper (D6, D9, D11) | 4.8 – **6.0 V** | **over spec — wait** |

So right now you can properly test **four of the seven**. The three small ones need a
buck converter set to 5.00 V, or a regulated 5 V supply, first.

---

## Do this once

1. Open Arduino IDE.
2. **File → Open** →
   `C:\RobotArm\Software\emre_kalem_single_servo_bench_test\emre_kalem_single_servo_bench_test.ino`
3. Check the top bar says **Arduino Uno** and **COM5**.
4. Click **Upload** (the right-arrow). Wait for "Done uploading."
5. Click the **magnifying-glass icon** to open Serial Monitor.
6. Set the speed dropdown to **115200**.

You should see the banner and the command list. If you see nonsense characters, the
speed is wrong — that's the only cause.

You only upload once. It stays on the board.

---

## Then repeat this for each servo

### Step 1 — wire it, with the power OFF

Unplug the supply. Then:

```
Servo YELLOW  →  the servo's pin from the table above
Servo RED     →  supply +
Servo BROWN   →  supply −
Arduino GND   →  supply −        ← the common ground. NEVER leave this out.
```

That last wire is what stops the servo working if you forget it. No shared ground means
the servo cannot read the signal at all — it sits perfectly still and draws nothing,
which looks exactly like a dead servo. That cost us most of a session.

**Take the horn off**, or unbolt the arm part, so the shaft spins free.

Nothing from the supply's positive side goes near the Arduino — not `5V`, not `VIN`, not
the barrel jack.

### Step 2 — power on and centre it

In Serial Monitor, type these one at a time:

| Type | What happens |
|---|---|
| the pin's number | selects it — screen confirms which pin |
| `c` | stores centre, 90° |
| `a` | **signal starts here.** Servo turns to centre and stops |

Watch that first move. A servo that turns to a position and goes **quiet** is correct.

### Step 3 — the small moves (this is the actual calibration)

| Type | What happens |
|---|---|
| `+` | one degree one way |
| `-` | one degree back |
| `]` | five degrees one way |
| `[` | five degrees back |

Do `+` `-` first. Only move on to `]` `[` once the one-degree test is clean.

**One degree is very hard to see.** Tape a toothpick or a paper flag to the shaft — it
gives you a pointer without adding any load.

### Step 4 — if you genuinely can't tell whether it moved

| Type | What happens |
|---|---|
| `w` | 90 → 110 → 70 → 90, holding over a second at each |

That's ±20°, the widest the sketch allows. Unmistakable even on a bare shaft. It's a
*detection* test, not a calibration test — use it to answer "is this thing alive?", then
go back to `+` `-`.

### Step 5 — stop and move on

| Type | What happens |
|---|---|
| `d` | detaches — signal off, servo no longer driven |

**Always `d` before you touch a wire.** Then unplug the supply, rewire the next servo,
and start again at Step 1.

---

## Full command list

```
0..6   select a pin
c      store centre 90
a      attach — signal starts
+  -   one degree each way
]  [   five degrees each way
w      wide test 90-110-70-90  (big, visible)
m      hold centre 60 s then auto-detach  (frees both hands for a meter)
d      detach — signal off
s      show status
h      help
```

Built-in limits you cannot type your way past:

- Every angle clamped to **70–110°**. `0` and `180` are impossible.
- Signal **detached at power-up**. Nothing is driven until you type `a`.
- Selecting a different pin **auto-detaches** the current one.

---

## What good and bad look like

| What you see | Verdict |
|---|---|
| Turns to centre, goes quiet, small moves both ways, returns to the same spot | **Pass.** Log it |
| Turns to centre but you can't see ±1° | Normal. Add a toothpick, or use `w` |
| Nothing at all | Wiring. Check the common ground first, then the signal pin, then the plug isn't reversed |
| Buzzes and won't stop | It's fighting something — horn still on, or a hard stop. `d` immediately |
| Gets hot, or the wires get warm | **Power off now.** Something is drawing far too much |
| Board resets / Serial Monitor reprints the banner | Brownout — the supply can't keep up |

Abort at any time: type `d`, or pull the supply. The Arduino stays on USB either way, so
you never lose the Serial Monitor.

---

## Suggested order

Lightest duty first, so mistakes are cheap:

1. **Elbow — D10** (`3`)
2. **Base — D3** (`0`)
3. **Shoulder left — D4** (`1`)
4. **Shoulder right — D5** (`2`)

Then, **after** you have 5 V:

5. Wrist roll — D9 (`5`)
6. Wrist pitch — D6 (`4`)
7. Gripper — D11 (`6`)

### Shoulders: a hard rule

Centre **D4 and D5 separately**. Never drive them as a pair with this sketch — it has no
mirror logic, and two servos fighting each other across one joint will strip gears. The
paired test needs the real controller sketch, which the vendor hasn't supplied.

---

## Log every one

Fill in `Calibration_Notes/calibration-log.csv` as you go: which pin, whether it reached
centre, whether ±1° and ±5° were clean, any noise or heat, pass or fail.

That file is what tells you months from now which joints were trustworthy and which were
marginal. Don't skip it.

---

## Before anything is bolted back on

Two things still outstanding, both from `V2-SERVO-POWER-AND-WIRING.md`:

- **Fit the 470–1000 µF capacitor** across the servo supply, close to the servo.
- **Get servo power off the breadboard.** Run `+` and `−` from the screw terminals
  straight to the servo. Breadboard contacts aren't rated for loaded servo current, and
  a poor contact produces intermittent faults that look exactly like a failing motor.

Unloaded bench testing on the breadboard is fine. Loaded operation is not.
