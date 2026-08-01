# V2 — Servo Power & Wiring, Rebuilt From Sources

> ## ✅ RESOLVED 2026-08-01 — root cause was a MISSING COMMON GROUND
>
> The base servo now moves. The fault was **no wire between the Arduino's `GND` and the
> 6.6 V supply's negative rail.**
>
> Everything else had been correct the whole time: the servo had power, and D3 was
> emitting valid ~1472 µs centre pulses. But a servo signal is a voltage measured
> *relative to ground*. With no shared zero, the servo had no way to interpret the
> pulses — so it sat still and drew nothing. That is exactly why 21 attach events across
> 7 pins produced no current draw and no brownout: the silence was the symptom.
>
> **Fix: one wire, any Arduino `GND` pin → the supply's negative rail.**
>
> Confirmed working with the servo on the external 6.6 V supply and the Arduino on USB.
> No board reset during the test, which is the correct behaviour for this architecture.
>
> This is the single most common servo wiring mistake there is. Everything below still
> stands — especially the capacitor (§4a) and the breadboard warning (§4b).

Written 2026-08-01 after stepping back and checking our setup against Arduino's own
documentation and the servo datasheets, instead of against my assumptions.

**Headline: the software was never the problem. The power architecture is.**

---

## 1. What the evidence already ruled out

| Layer | Status | Evidence |
|---|---|---|
| Laptop / IDE / compiler | Good | IDE 2.3.10, AVR 1.8.8, Servo 1.3.0, clean compiles |
| Board | Good | Genuine Uno, `1E 95 0F`, bootloader FW 4.4 |
| Upload path | Good | Flash written **and read back byte-compared**, twice |
| Sketch logic | Good | 21 attach events across 7 pins, every state transition correct |
| Pulse width | **Good — see below** | This was the last plausible software cause. It is eliminated |

### Why pulse width is not the fault

The Arduino Servo library's `attach(pin)` defaults to **544 µs at 0°** and **2400 µs at
180°** ([official API docs](https://github.com/arduino-libraries/Servo/blob/master/docs/api.md)).
So `write(90)` produces:

```
544 + (90/180) x (2400 - 544)  =  1472 us
```

A ~1472 µs pulse is a valid centre command for **any** hobby servo — MG996R, MG90S, SG90,
all of them. The library's own docs note servos "often respond to values between 700 and
2300", and 1472 sits comfortably in the middle.

If the servo had power and signal, it would have moved. It did not. Therefore the fault is
upstream of the code.

**Conclusion: do not rewrite the sketch.** It was correct. Two diagnostic commands were
added (§5), nothing else changed.

---

## 2. The one thing that explains every observation

Across three test runs — 21 separate attach events on 7 different pins — **the board never
browned out, never reset, and showed no sign of current draw.**

An MG996R snapping to centre is a violent current event. Arduino's guidance and every
datasheet agree it can pull up to **2.5 A stalled**. If that servo had been connected to
the Arduino's 5 V pin and driven to centre, the outcome would have been dramatic: a
brownout, a reset, or a tripped USB fuse. We saw none of it, 21 times.

**Silence like that is not a servo failing to respond. It is a servo that is not
electrically present.**

---

## 3. The power architecture — corrected, and now sourced

This is no longer a judgement call. Two independent sources say the same thing:

- **Your walkthrough:** "No servo red wire to the Arduino 5V pin."
- **Arduino / servo documentation:** the Arduino 5 V pin supplies roughly **500 mA total**.
  An MG996R draws up to **2500 mA** stalled. Running one from the 5 V pin "will cause
  voltage drops, erratic behaviour, and potentially damage your board."

I earlier offered a shortcut — one **MG90S** (a small servo, ~100–300 mA unloaded) powered
from the 5 V pin. That remains defensible for a single small servo doing tiny moves. **It
was never valid for the base servo, which is an MG996R.** Powering a big servo from the
board is off the table.

### The correct chain

```
   USB  ─────────────────────────►  ARDUINO         (logic only)
                                       │
                                       ├── D3 ──────► servo YELLOW   (signal)
                                       │
                                       └── GND ──┐
                                                 │   ← THE ONLY LINK
   5-6 V SUPPLY  ── + ──[ FUSE 1A ]──[ KCD1 ]────┼──► servo RED
                  ── − ───────────────────────────┴──► servo BROWN
                          │        │
                          └─[ 470-1000 uF ]─┘       ← across + and −, near the servo
```

Three rules, and they are the whole thing:

1. **USB powers the Arduino. The separate supply powers the servo.** Never both.
2. **The grounds must be joined.** Without a shared ground reference the servo cannot
   interpret the signal pulses at all — the signal is measured *relative to ground*, and
   with no common ground there is no reference. A servo with power but no common ground
   sits dead still, exactly like one with no power.
3. **Nothing from the supply's positive side touches the Arduino.** Not `5V`, not `VIN`,
   not the barrel jack.

---

## 4. Two things missing from everything I wrote before

### 4a. The capacitor — add one

**470–1000 µF electrolytic across the servo supply, physically close to the servo.**

Standard practice, and I omitted it. A servo starting to move yanks current far faster
than a wall supply can respond. The capacitor is a local reservoir that covers that first
few milliseconds, which is exactly when brownouts and resets happen. With a 700 mA supply
driving a servo rated for far more, it matters more, not less.

Watch polarity — electrolytics are polarised. The stripe is the negative side.

### 4b. The breadboard is not a power path

Your walkthrough is explicit: do not use "a solderless breadboard carrying MG995/MG996R
current."

In the bench photo, the blue screw-terminal blocks are plugged **into** the breadboard.
That's a sensible connector on the wrong path — the servo current still has to cross the
breadboard's spring contacts, which are rated around 1 A at best and degrade with use. A
poor contact there produces intermittent, maddening faults that look exactly like a
failing servo.

**Signal wires through the breadboard: fine. Servo power through the breadboard: no.**
Run servo `+` and `−` from the screw terminals straight to the servo.

Also worth knowing: on many mini breadboards the long `+`/`−` rails are **split in the
middle**, so the top half is not connected to the bottom half. That alone has cost people
entire evenings.

---

## 5. Sketch changes (additions only — no rewrite)

Two commands added. Both require `a` first, so "nothing is driven until you type `a`"
still holds.

| Command | What it does |
|---|---|
| `w` | **Wide visibility test.** 90 → 110 → 70 → 90, holding 1.2 s at each. That's ±20°, the full clamp range — unmistakable even on a bare spline |
| `m` | **Measure mode.** Holds a steady 90° pulse for 60 s, then auto-detaches. Frees both hands for the multimeter. Any keypress aborts |

Why `w` exists: the ±1° and ±5° tests we ran are the correct **safety** ladder, but they
are poor **detection** tests. With the horn removed — which is right for safety — there is
no pointer on the shaft, and 5° of bare spline rotation is nearly invisible. We may have
been moving that servo all along and unable to see it.

**Free fix: tape a toothpick or a paper flag to the output spline.** Restores a visible
pointer without restoring any load.

Sketch is now 8682 bytes (26% of flash). Compiles clean.

---

## 6. The measurement that ends the guessing

With the servo wired and the sketch loaded, in the IDE's Serial Monitor at 115200:

```
0    select base (D3)
c    store centre
a    attach
m    hold 60 s
```

Then probe with the meter:

| Probe | Expect | Meaning if wrong |
|---|---|---|
| Signal pin (D3) ↔ GND | **~0.37 V DC** | 0 V = no pulses on that pin. ~5 V = stuck high |
| Servo plug RED ↔ BROWN | **~5–6 V DC** | 0 V = the servo has no power. This is the prime suspect |

Where 0.37 V comes from: a 1472 µs pulse repeated every 20 ms is a 7.4% duty cycle, and
7.4% of 5 V is 0.37 V. A cheap meter averaging 50 Hz PWM will wander a bit — the useful
distinction is **"about a third of a volt" vs "zero" vs "about five"**, not the exact
figure.

---

## 7. Shopping list

| Item | Why | Rough cost |
|---|---|---|
| Female barrel jack, screw terminals, 5.5 × 2.1 mm | To use the 6 V supply without cutting its cable | $2 |
| 470–1000 µF electrolytic capacitor, 10 V+ | Absorbs the startup current spike | $1 |
| Inline fuse holder + 1 A fuses | Required by the walkthrough | $3 |
| **Later:** regulated 5 V, 3–5 A supply | The 700 mA unit will never drive multiple servos | $15–25 |

The 6.62 V unit stays usable for **one MG996R**, unloaded (in spec: 4.8–7.2 V). It is
**out of spec for the MG90S** (4.8–6.0 V) and cannot drive several servos at once.

---

## Sources

- [Arduino Servo library API — attach() defaults 544/2400 µs](https://github.com/arduino-libraries/Servo/blob/master/docs/api.md)
- [Arduino Forum — Servo.attach min/max defaults](https://forum.arduino.cc/t/servo-attach-min-max-defaults-confusing/165639)
- [How To Mechatronics — servo control and power](https://howtomechatronics.com/how-it-works/how-servo-motors-work-how-to-control-servos-using-arduino/)
- [MG996R specifications and wiring guidance](https://componentindex.net/components/mg996r/)
- [MG90S specifications](https://docs.cirkitdesigner.com/component/1e4b6629-da03-44b1-bcdd-d6dab0bbc368/servo-motor-mg90s)
- `Emre_Kalem_Arduino_Uno_Beginner_Walkthrough.docx` — the project's own safety procedure
