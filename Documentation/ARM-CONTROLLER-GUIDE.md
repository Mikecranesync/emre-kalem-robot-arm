# FactoryLM Arm Console — User Guide

Plain-English guide to the arm controller program and the on-screen console on
Windows and macOS.
Companion to `SERIAL-PROTOCOL.md` (the technical version).

---

## 1. Read this first: the power supply you have will not run the arm

**Your supply is 6.62 volts at 700 milliamps. That is not enough to move the
assembled arm. Do not try.**

Here is the arithmetic, in plain terms.

- 700 milliamps is 0.7 amps. That is the most current that supply can push.
- One MG996R motor, working hard, can pull **2.5 amps on its own**.
- A rebuilt arm has **seven motors** all holding the arm's own weight up against
  gravity at the same time.

So you need roughly **3 to 5 amps**, not 0.7. That is not a small shortfall. It is
about five times short.

**What you need to buy:** a *regulated* 5-volt supply rated **3 amps or more**.
"Regulated" means it holds a steady 5 volts instead of sagging when something pulls
hard on it. A phone charger is not good enough. Expect to pay $15 to $25.

### What happens if you try anyway

The supply will sag under load. When the voltage sags, the Arduino resets, the motors
twitch, and the whole thing looks like a software fault when it is really a power
fault. That is a frustrating afternoon and a real chance of stripping a plastic gear.

### But the wait is not wasted — here is the good part

**Everything in this guide works right now with zero motors powered.**

Put the program on the Arduino, start the console, connect, enable a joint, drag a
slider, record a pose, play it back. The Arduino answers every command and the screen
shows every number, whether or not any motor is plugged in.

That is not a toy exercise. It is the real proof that the whole chain works:

```
your browser  ->  the bridge  ->  the USB cable  ->  the Arduino  ->  (motors, later)
```

If you prove all of that with the rocker switch OFF, then the day your new supply
arrives, the only thing left to test is the electrical side. One unknown at a time is
how you avoid breaking things.

**So: work through this entire guide with servo power OFF first.** That is the
intended way to use it, not a workaround.

---

## 2. Three of the seven motors cannot take 6.62 volts even for a bench test

Motors have a maximum voltage. Going over it damages them — usually slowly, so you do
not notice until one starts misbehaving.

| Motor type | Where it is | Safe voltage | On your 6.62 V supply |
|---|---|---|---|
| MG996R (the big ones) | Base D3, Shoulder D4, Shoulder D5, Elbow D6 | 4.8 – 7.2 V | **Fine** |
| MG90S (the small ones) | Wrist pitch D9, Wrist roll D10, Gripper D11 | 4.8 – **6.0 V** | **Too high — do not use** |

6.62 is above 6.0. So the three small motors — wrist pitch, wrist roll and gripper —
must not be run from that supply at all, not even for a quick single-motor test.

They need either the new 5-volt supply, or a **buck converter** (a small board that
takes a higher voltage in and puts a lower voltage out) set to exactly 5.00 volts and
checked with a meter before anything is connected.

### One honest caveat about that table

The motor types come from the **parts list**, not from anyone reading the label on
each motor. Only one is confirmed by documentation: wrist roll on D10 is an MG90S.
The other six are educated guesses recorded as `INFERRED` in
`Software\wiring-map.csv`, and there is already a warning about exactly this in
`Calibration_Notes\calibration-log.csv`.

Why that matters: if a motor you believe is an MG996R is actually an MG90S, then
6.62 volts is **already over its limit**. If you can read the label on a motor, do,
and correct `wiring-map.csv`. Until then, treat 6.62 V as a bench-only voltage for the
four big joints and nothing else.

---

## 3. Putting the program on the Arduino

The Arduino only holds one program at a time. Right now it holds the old single-motor
bench test. You are going to replace that with the arm controller.

Arduino calls a program a **sketch**. Same thing.

### Before you start

1. **Servo power OFF.** Rocker switch off, or unplug the supply.
2. **Close the arm console.** If the `FactoryLM Arm GUI` terminal window is open, click
   it and press `Ctrl-C`, then close it. Only one program at a time can use the USB
   port, and the console holds it.

### Upload it

1. Open **Arduino IDE**. Arduino IDE and Arduino CLI run on macOS as well as Windows;
   the board, sketch, baud rate, and protocol are the same on both hosts.
2. **File → Open**, and open this file:
   `Software/factorylm_arm_controller/factorylm_arm_controller.ino` inside your
   local checkout.
3. Look at the bar across the top. It should say **Arduino Uno** and a port.
   Pick the port that identifies the Arduino. On Windows it looks like `COM5`; on
   macOS it normally looks like `/dev/cu.usbmodem...` or `/dev/cu.wchusbserial...`.
   Do not hard-code the name — it can change when you unplug and replug the board.
4. Click the **right-arrow button** (Upload).
5. Wait for **"Done uploading."**

### Check it took

Still in Arduino IDE, click the **magnifying-glass icon** to open Serial Monitor, and
set the speed dropdown to **115200**. You should see a line like this:

```
RDY NAME=FACTORYLM-ARM PROTO=1.0 FW=1.0.0
```

`RDY` means ready. `NAME=FACTORYLM-ARM` is how the console recognises that the right
program is on the board.

**Then close Serial Monitor.** Leaving it open blocks the console from using the port.

### If you would rather type commands

```text
# Windows
"C:\Program Files\Arduino CLI\arduino-cli.exe" board list
"C:\Program Files\Arduino CLI\arduino-cli.exe" compile --fqbn arduino:avr:uno "C:\RobotArm\Software\factorylm_arm_controller"
"C:\Program Files\Arduino CLI\arduino-cli.exe" upload -p COM4 --fqbn arduino:avr:uno "C:\RobotArm\Software\factorylm_arm_controller"

# macOS/Linux — run from the repository root
arduino-cli board list
arduino-cli compile --fqbn arduino:avr:uno Software/factorylm_arm_controller
arduino-cli upload -p /dev/cu.usbmodemXXXX --fqbn arduino:avr:uno Software/factorylm_arm_controller
```

Replace the example port with the one reported by `board list`.

### What the board does when it starts

Nothing. On purpose.

Every motor signal is switched off, and stays off until you specifically enable a joint
from the screen. No motor is ever driven at power-up. There is no "go to home position"
step, and there never will be — see section 8 for why that would be dangerous.

---

## 4. Starting the console

### The normal way

On Windows, double-click:

```
C:\RobotArm\START ARM GUI.bat
```

On macOS, double-click `START ARM GUI.command` in the repository folder, or run
`./START ARM GUI.command` from Terminal.

Two things happen.

1. A **terminal window** opens. This is the *bridge* — a small helper program that passes
   messages between your web browser and the USB cable. **Leave it open.** Closing it
   kills the connection.
2. Your **browser opens** at `http://127.0.0.1:8770/`. That address means "this
   computer, nobody else". Nothing is on the internet. Nothing leaves the computer.

If the browser does not open by itself, type that address in yourself.

### The long code in the address bar

Once the page loads, the address will have grown a long jumble on the end, like:

```
http://127.0.0.1:8770/?t=hcUsJz701BF2oB4v_USFRO1I
```

That is normal. It is a one-time access code, and the terminal window prints it too.

**Why it exists:** "this computer only" turns out not to be as private as it sounds. Any
web page you have open in any tab can quietly send messages to a program running on your own
laptop. Without the code, an ordinary web page could have found the bridge and started
moving the arm while you were reading something else, with nothing on screen to tell you.
The code is the bridge's way of checking that a request really came from the arm console
page and not from some other page that went looking.

Three things follow from it, and all three are normal:

- **The code is different every time the bridge starts.** It is never saved anywhere.
- **An old tab stops working after you restart the bridge.** Press F5 on it, or open the
  link from the terminal window again.
- **Plain `http://127.0.0.1:8770/` still works.** The bridge just sends you along to the
  proper address with the code attached.

None of this makes the arm safe to stand next to. It stops stray software reaching the USB
port. The rocker switch is still the only real stop.

**None of it applies to the no-Python route below** — opening `arm-console.html` straight
from the folder does not use the bridge at all.

### Connect to the board

1. In the box at the top, open the port list.
2. Choose the entry that identifies **Arduino Uno**. On macOS the entry normally
   has a `/dev/cu.*` name rather than a COM number; do not hard-code it.
3. Press **CONNECT**.

Opening the port restarts the Arduino. That is normal and takes about two seconds.

### What "connected and working" looks like

- The chip at top left turns **green** and says `CONNECTED`.
- Next to it you see `FACTORYLM-ARM FW 1.0.0`.
- Six joint cards appear.
- Every card has an **amber badge saying UNCALIBRATED** and an amber stripe down its
  left edge.
- Every card says **DISABLED**.

**Six amber cards is correct.** It is the screen telling you the truth: nobody has
measured how far these joints can actually travel yet, so it is using a placeholder
range of 70 to 110 degrees. Section 7 is how you fix that. An interface that showed
green here would be lying to you.

### The six cards

| Card | Joint | Arduino pin(s) | Motor |
|---|---|---|---|
| **J0** | Base — turns left and right | D3 | MG996R |
| **J1** | Shoulder — **both motors together** | D4 + D5 | two MG996R |
| **J3** | Elbow | D6 | MG996R |
| **J4** | Wrist pitch — tilts up and down | D9 | MG90S |
| **J5** | Wrist roll — twists | D10 | MG90S |
| **J6** | Gripper / claw | D11 | MG90S |

**There is no card number 2, and that is deliberate.** Number 2 is the shoulder's
second motor. The two shoulder motors drive the same joint through the same linkage,
so they are only ever commanded together, as J1. There is no way to move one shoulder
motor on its own — not from the screen, not by typing, not at all. If two motors ever
pushed different directions on that one joint they would strip their gears against
each other in seconds.

### If Python is not installed

There is a second way in that installs nothing:

1. Open `Software/arm-console/` in the checkout.
2. Double-click **`arm-console.html`**
3. Press **CHOOSE PORT** and pick the Arduino Uno

This works in **Google Chrome and Microsoft Edge only**. Firefox cannot do it.
Everything else in this guide is identical.

---

## 5. Moving a joint, and stopping it

### First, what the numbers on screen actually mean

The screen shows the angle the software **told** the motor to go to. It says
`commanded`, not `position`.

These motors send nothing back. They have no way to report where they really are. If a
motor is jammed, unpowered, or fighting something, the number on screen will still show
the angle it was told to go to. Believe your eyes over the screen, always.

### Enabling a joint — and the one mistake to avoid

A joint does nothing until you **enable** it. Enabling means "start sending signals to
this motor".

Press **ENABLE** on a card. A small box appears asking for an angle, with the word
**"the angle this joint is at RIGHT NOW"**.

**Read that carefully. It is asking where the joint already is — not where you want it
to go.**

Look at the physical arm and estimate the angle by eye. If the elbow looks like it is
sitting near the middle of its travel, type 90. If it is clearly over towards one end,
type 105, or 75, or whatever your eye says.

Why it matters: the very first signal the motor receives will be exactly the number you
typed. Tell the truth and the motor holds still — it is already there. **Type 90 out of
habit on a joint that is actually sitting at 130 and the motor will snap 40 degrees in
an instant**, dragging whatever is bolted to it. On a reassembled arm that is the single
most damaging thing you can do from this screen.

Guessing a little wrong is fine — you will see a small twitch. Guessing wildly wrong is
not. When unsure, look again before you press Confirm.

Press **CONFIRM**. The card turns green and says `ENABLED`, and the slider wakes up.

### Moving it

Drag the slider. The joint moves smoothly toward the new angle rather than snapping —
speed is limited to 30 degrees per second by default.

That speed limit is a power feature, not a comfort feature. An unlimited motor drives
at full speed and pulls its maximum current. Slowing it down is what stops six motors
all lunging at once and dragging the supply voltage down.

You can change it in the small **deg/s** box on the card. Anything from 1 to 90. Leave
it at 30 until the arm actually moves under load and you have watched it.

### If you ask for something out of range

The card flashes amber and prints something like:

```
CLAMPED: asked 130 degrees, limit is 110
```

The joint went to 110, not 130. Nothing was ignored silently — it is telling you it
overruled you and by how much. If you see this a lot, the real answer is section 7:
measure the joint's true range and write it down.

### Stopping

Four levels, gentlest first.

| Button | What it does | When |
|---|---|---|
| **HOLD MOTION** (top) | Freezes every joint where it is. **Motors stay powered and keep holding** | Something is moving and you want it to stop moving *now*, without the arm dropping |
| **DISABLE** (on a card) | Stops driving that one joint | Finished with it |
| **DISABLE ALL** (amber, top) | Stops driving every joint | Taking a break, about to touch wiring |
| **E-STOP** (big red, top) | Cuts the signal to everything instantly and locks it out | Something is going wrong |

### HOLD MOTION versus E-STOP — the difference that matters at the bench

These two buttons both stop the arm, and they stop it in completely different ways. On a
bare bench with no motor power you cannot tell them apart. On a rebuilt arm holding its own
weight, the difference is whether the arm stays where it is or falls.

| | **HOLD MOTION** | **E-STOP** |
|---|---|---|
| What stops | The movement. Every joint freezes at the angle it had reached | Everything |
| The motors | **Stay powered. Still holding.** | **Signal cut. They stop holding** |
| A loaded arm | **Stays put** | **Goes limp and sags under gravity** |
| Getting going again | Just move a slider. Nothing to clear, nothing to re-enable | Press CLEAR E-STOP, then re-enable every joint one at a time with a fresh "where is it now" angle |
| Locks anything out? | No | **Yes, until you clear it** |

Plain version: **HOLD MOTION is "stop moving". E-STOP is "let go".**

So if the arm is heading somewhere you did not intend, **HOLD MOTION is usually the button
you actually want** — it halts the move and the arm keeps holding its position while you
think. E-STOP is for when something is genuinely wrong and you want everything
de-energised, accepting that a loaded arm will sag when it happens.

That is also why HOLD MOTION is not a safety device in any sense. It only stops the arm
going somewhere new; the motors are still live and still pulling current. If you need the
arm to be *safe to put your hands near*, that is the rocker switch, every time.

### The red E-STOP button

It is the biggest thing on the page and it is always visible, even when you scroll.

Pressing it cuts the signal to every motor at once and **latches** — meaning nothing
will move again, no matter what you click, until you press **CLEAR E-STOP**. After
clearing, every joint is disabled again and you re-enable them one at a time, each with
a fresh "where is it right now" angle.

**Understand what "cuts the signal" means physically.** A motor with no signal stops
holding. On a reassembled arm carrying its own weight, that means the arm **goes limp
and sags** under gravity. It does not lock in place.

So: support the arm, or have it resting on something, before you rely on that button.

That is also why re-enabling always asks for the angle again. After a stop, the arm has
physically moved. The software's old numbers are stale — not because it forgot, but
because the metal moved while it was not looking.

> **The rocker switch and the inline fuse are the real emergency stop.**
> The red button, the Arduino, the bridge and this browser page are conveniences. A
> browser tab can freeze, a USB cable can fall out, a laptop can sleep. Killing the
> power is the only stop that cannot fail. Keep a hand near that switch whenever the
> arm is powered.

### Why the shoulder card will not let you enable it yet

J1 shows `mirror: unknown` and its ENABLE button is greyed out.

The two shoulder motors face opposite ways on the joint. To move the shoulder up, one
may need to turn clockwise while the other turns counter-clockwise — or they may both
need to turn the same way. **It depends on which way round the plastic horns were
fitted when the arm was built, and nobody wrote that down.**

Guessing has one bad outcome: get it backwards and both motors push against each other
through one linkage, at full torque, until something breaks.

So the software refuses. Section 7 tells you how to find the answer in about ten
minutes. This is the design working, not a bug to route around.

---

## 6. Teach and playback — recording poses

This is how you make the arm repeat a sequence without programming anything.

The idea: pose the arm by hand with the sliders, press a button to save that pose, do
it again for the next pose, then press play. Each saved pose is called a **waypoint**.

### Recording

1. Enable the joints you want to control.
2. Drag the sliders until the arm is in the pose you want.
3. Press **RECORD WAYPOINT**.

A new row appears in the table with the current angle of every **enabled** joint.

Joints that are **disabled** get a `-` in that row. A dash means "leave this joint
alone at this step". That is genuinely useful: if you only enable the gripper, your
waypoint says "close the gripper, do not touch anything else".

4. Type a name in the row's **label** box — "pick up", "lift clear", "release". Future
   you will thank present you.
5. Set **dwell ms** — how long to wait at that pose before moving on, in thousandths of
   a second. 1000 means one second.

Repeat for each pose in your sequence.

### Playing it back

| Button | What it does |
|---|---|
| **GO** (on a row) | Move to that one pose |
| **PLAY** | Run every row from the top |
| **loop** (tick box) | Keep repeating until stopped |
| **STOP PLAYBACK** | Stop between steps |
| **E-STOP** | Stop immediately, everywhere |

During playback the console moves each joint, waits for it to arrive, waits the dwell
time, then moves to the next row.

If a joint fails to arrive within 15 seconds the whole sequence aborts and tells you.
That usually means a motor is stalled, unpowered, or fighting a hard stop — go and look
at it before pressing anything else.

### Saving and loading

- **SAVE CSV** writes the table to a file, normally into your Downloads folder, named
  with today's date. CSV is a plain-text spreadsheet file — Excel opens it, so does
  Notepad.
- **LOAD CSV** reads one back in. It is the browser's ordinary file-picker.

Keep your saved sequences somewhere sensible inside the repository, such as
`Software/arm-console/`.

### A safety note about playback

Playback obeys the joint limits, and nothing else. It has no idea whether the path
between two poses swings the arm through the table, through a clamp, or through your
hand.

**Always play a new sequence once with servo power OFF first.** Watch the numbers move
on screen. Then play it again at low deg/s with power on and a hand on the rocker
switch.

---

## 7. Calibration — finding each joint's real limits

### What a limit is, and why yours are fake right now

Every joint has a range it can physically travel before it hits its own plastic
end-stop. A motor pushed past that stop does not stop pushing. It stalls, pulls its
maximum current, gets hot, and grinds its gears.

The controller refuses to command any angle outside each joint's stored range. That
range starts as a placeholder of **70 to 110 degrees** — deliberately narrow, and
deliberately marked **UNCALIBRATED** in amber on every card.

That placeholder is not a measurement. It is a "we do not know yet" flag that happens
to be safe. Nobody has measured this arm. Exactly one motor has ever been moved on this
project, unloaded, on a bench.

**Never widen a limit because you think it should be wider. Measure it.**

### How to measure one joint

Use the old single-motor bench sketch for this. It is the proven tool for it, and the
full procedure is in `HOW-TO-TEST-EACH-SERVO.md`. Short version:

1. **Servo power off.** Unbolt that joint's arm section, or take the horn off, so the
   shaft turns freely with nothing attached.
2. Wire that one motor only. Remember the **common ground**: an Arduino `GND` pin to
   the supply's negative. Without it the motor sits dead still and looks broken.
3. Load `emre_kalem_single_servo_bench_test` and centre the motor at 90.
4. Move outward five degrees at a time. Watch and listen. Stop the moment you hear
   strain, see the linkage tighten, or feel a hard stop.
5. **Come back 5 degrees from wherever you stopped.** That is your limit — with margin,
   not right on the edge.
6. Do the same in the other direction.
7. Write both numbers down.

Tape a toothpick to the shaft first. One degree of bare metal spline is nearly
invisible.

### Where to write it down

Open this file in Notepad or Excel:

```
C:\RobotArm\Software\arm-console\joint-limits.csv
```

The columns are:

| Column | What to put |
|---|---|
| `joint_id` | 0, 1, 3, 4, 5, 6 — do not change these |
| `joint_name` | the name — do not change |
| `uno_pins` | the pin — do not change |
| `min_deg` | the low limit you measured |
| `max_deg` | the high limit you measured |
| `home_deg` | a comfortable resting angle inside that range |
| `max_deg_per_sec` | how fast, 1 to 90 |
| `calibrated` | `yes` once you have really measured it, otherwise `no` |
| `mirror_mode` | shoulder row only — see below |
| `notes` | anything you want to remember |
| `mirror_offset_deg` | shoulder row only, and only if the mirror turns out to be `inverted` — see below. **Leave it at 0 until you have measured it** |

Save the file. Then in the console press **LOAD LIMITS FILE** and choose it. The card
updates and the amber badge turns green.

**Typing `yes` does not calibrate anything.** The bench measurement is the calibration.
That column only records that you did it. Marking a joint calibrated when you have not
measured it removes the one warning that was protecting you.

Also add a row to `Calibration_Notes\calibration-log.csv` while it is fresh — which
pin, what you saw, any noise or heat, pass or fail. In six months that log is the only
thing that tells you which joints were solid and which were marginal.

### If the file will not load

If **any single row** is wrong, the whole file is rejected and the console tells you
which row and what is wrong with it. It will not load half a file.

That is intentional. A half-applied limits file is more dangerous than no file, because
some joints would silently keep old values while you believe you loaded new ones.

Common causes: a typo in a number, `min_deg` bigger than `max_deg`, a value below 0 or
above 180, or a row for joint 2 (which does not exist — see section 4).

### The shoulder mirror — the one extra step for J1

You need to find out whether the two shoulder motors turn the **same** way or
**opposite** ways to move the joint one direction.

1. Servo power off. Unbolt the shoulder linkage so each motor turns something free.
2. Using the single-motor bench sketch, select **D4** on its own. Centre it. Move it
   five degrees. **Note which way the output turns.**
3. Detach. Now select **D5** on its own. Centre it. Move it five degrees the same
   direction in software. **Note which way that output turns.**
4. Compare:
   - Both turned the same way to move the joint the same way → `mirror_mode` is
     **`same`**
   - They had to turn opposite ways → `mirror_mode` is **`inverted`**
5. Put that word in the `mirror_mode` column of the shoulder row in
   `joint-limits.csv`, save, and press **LOAD LIMITS FILE**.

The J1 card's ENABLE button now works. Until it is set to `same` or `inverted`, J1 is
locked.

**Never drive D4 and D5 together on the bench sketch.** That sketch has no mirror logic
at all. Only the arm controller knows how to pair them, and only once you have told it
which way.

### If the answer was `inverted`, there is one more number to measure

`inverted` means the two motors turn opposite ways. The obvious assumption is that they
mirror around the exact middle — 90 degrees. That assumption is probably wrong, and the
column `mirror_offset_deg` is where you correct it.

Here is why it is probably wrong. The plastic horn does not slide onto the motor shaft at
any angle you like. The shaft has teeth, and the horn drops into one of them — on the big
MG996R motors the teeth are about **18 degrees apart**. So whoever built this arm fitted
each horn to the nearest tooth. Getting both sides landing at exactly the same place is
luck, not the normal outcome.

**What goes wrong if you ignore it.** Suppose the true middle is really 96 degrees and you
leave the offset at 0. Then every single time you enable the shoulder, the two motors are
being told to sit 12 degrees apart from where they agree — so they push against each other,
constantly, the whole time the joint is on. They will hold, get warm, draw more current than
they should, and in time strip a gear. And nothing on the screen will show it, because these
motors report nothing back.

**How to measure it** — same setup as the mirror test above, linkage unbolted, one motor at
a time on the bench sketch, never both:

1. Drive **D4** on its own until its output points in some clear physical direction — say,
   straight up. Write down the number you commanded. Call it **a**.
2. Detach. Drive **D5** on its own until its output points **the same physical way**. Write
   down that number. Call it **b**.
3. Add them, halve the result, and take 90 off:

   ```
   mirror_offset_deg = (a + b) / 2 - 90
   ```

   Round to a whole number. If both were 90, the answer is 0 and you were lucky. If a was 84
   and b was 108: (84 + 108) / 2 = 96, minus 90, so **6**.
4. Put that number in the `mirror_offset_deg` column of the shoulder row, save, and press
   **LOAD LIMITS FILE**.
5. Write down in `Calibration_Notes\calibration-log.csv` what you did and what you saw.

**Leave it at 0 until you have actually done this.** A made-up offset is worse than no
offset at all, because the file then looks like somebody measured it. Same rule as the
travel limits: measure, do not guess.

The console may also refuse an offset outright, saying the mirror would land outside the
motor's travel. That is not a bug — it means that offset combined with the shoulder's
min/max would ask the second motor to go somewhere it cannot reach. Fix the shoulder's
limits first, then set the offset.

---

## 8. The first power-on after the arm is rebuilt — the riskiest moment

Everything before this was with motors unpowered or unbolted. This is different, and it
deserves its own procedure.

### Why this moment is different

- **Every motor is at an angle nobody knows.** They were turned by hand during assembly.
- **The horns and arm sections are bolted on.** A motor that twitches now swings a whole
  limb, not a bare shaft.
- **Gravity is loading the joints.** The shoulder and elbow are holding weight up.
- **All seven motors want current at once.** This is where a weak supply browns out.

The controller is written so it never centres anything by itself. That protection only
works if you also do not ask it to move six joints at once on the first try.

### Do it in this order

1. **Support the arm.** Rest it on a box, clamp it, or hold it. Assume it will sag
   the moment power is cut — because it will.
2. **Clear the area.** Nothing fragile, nothing in the sweep, no fingers in the path.
3. **Servo power OFF.** Rocker off. Confirm by eye, not by memory.
4. **Start the console and connect to the board.** With no servo power at all, prove
   the software works: green CONNECTED chip, six cards, correct firmware name.
5. **Load your limits file** and check the calibrated joints show green.
6. **Walk the arm through by hand** and write down, on paper, roughly what angle each
   joint is sitting at. This is the list you will type into the ENABLE boxes.
7. **Put one hand on the rocker switch. Leave it there.** Everything from here happens
   one-handed.
8. **Turn servo power on.** Nothing should move. Nothing is being driven yet — no joint
   is enabled. If anything twitches or hums at this point, **hit the rocker switch** and
   go find the wiring fault before continuing.
9. **Enable exactly ONE joint.** Start with the lightest one — the gripper (J6) or wrist
   roll (J5). Type the angle you wrote down for it in step 6.
10. **It should not move.** You told it where it already is. A small settle is normal. A
    lunge means your angle estimate was wrong — hit the rocker.
11. **Make one small move.** Drop the speed to 10 deg/s. Nudge the slider a few degrees.
    Watch it. Bring it back.
12. **DISABLE that joint before you enable the next one.**
13. Repeat for the next joint. Lightest to heaviest: gripper, wrist roll, wrist pitch,
    elbow, base, and the shoulder last.

### The rules for that first session

- **One joint enabled at a time. Never all six.** If something is wrong, one motor
  fighting is a nuisance; six is a broken arm.
- **Heaviest joints last.** The shoulder holds the most weight and has two motors. It
  has the most to go wrong and it goes wrong hardest.
- **Hand on the switch, every second the arm is powered.**
- **If the board resets** — the console drops out, the cards go grey — that is a
  brownout. The supply could not keep up. Stop. That is section 9.
- **Joint 1 stays locked** until you have done the mirror test in section 7. Do not
  work around it. It is locked because that is the failure that breaks gears.

Once each joint has moved on its own, cleanly, at low speed, then you can start
enabling two at a time. There is no hurry.

---

## 9. Troubleshooting

### The port is busy, or Connect fails

**Almost always: something else already has the USB port.** Only one program can use it
at a time.

Check and close, in this order:

1. **Arduino IDE's Serial Monitor.** This is the usual one. Close the Serial Monitor
   panel, not just the IDE window.
2. **A second bridge window.** If you started the platform launcher twice, there
   are two. Close the extra one.
3. **A second browser tab** with the console open and connected.

Then press **CONNECT** again.

> **While the console is connected, do not open Serial Monitor.** It steals the port
> *and* restarts the Arduino, which drops every joint mid-move. If the arm is powered
> and holding weight when you do that, it sags. This will catch you once — try to make
> it a time when the arm is on the bench and unpowered.

### No board in the list

- **Check the USB cable is in both ends.** Some cheap cables are charge-only and carry
  no data at all. If a cable has never worked, try another one.
- **Press REFRESH** next to the port list. The operating system may update its list
  lazily after you plug the board in.
  (If you opened `arm-console.html` directly in Chrome instead of using the launcher,
  that button says **CHOOSE PORT** and opens Chrome's own picker.)
- **Use the genuine Arduino Uno.** The clone board cannot be programmed on this laptop
  — Windows installed a broken driver for it. The write-up is in
  `USB-SERIAL-DIAGNOSIS.md`. It is not your fault and neither board is damaged.
- **Do not go looking for a hard-coded port name.** Windows may show COM4/COM5 while
  macOS shows `/dev/cu.*`, and either system can change the name after a replug. Pick
  the entry whose text identifies the Arduino instead.

### It connects but refuses, saying the wrong program is on the board

The console checks the board's name before it will send anything. If it says the name
does not match, the Arduino is still running the **old single-motor bench sketch**.

Go back to section 3 and upload `factorylm_arm_controller`.

This check exists on purpose. The old sketch reads a single letter `a` as "start
driving". A console sending commands blind to the wrong program would eventually make
something move that you did not ask to move.

### A joint will not move

Work down this list in order. It is roughly most-likely-first.

1. **Is the joint enabled?** A grey card and a dead slider means no signals are being
   sent. Press ENABLE.
2. **Is the e-stop latched?** If the top bar is red, everything is locked out. Press
   **CLEAR E-STOP**, then re-enable the joint.
3. **Is servo power on?** The rocker switch, and the supply actually plugged in. The
   Arduino runs happily on USB alone and the screen looks completely normal with no
   servo power at all — that is by design, and it is also the easiest thing in the world
   to forget.
4. **Is the common ground connected?** One wire from any Arduino `GND` pin to the
   supply's negative rail. **Without it a motor sits perfectly still and draws nothing,
   which looks exactly like a dead motor.** This exact fault cost this project most of a
   session. It is the first thing to check on any wiring you just changed.
5. **Is the signal wire on the right pin?** J0 is D3, J1 is D4 and D5, J3 is D6, J4 is
   D9, J5 is D10, J6 is D11.
6. **Is the motor plug the right way round?** Yellow is signal, red is positive, brown
   or black is negative.
7. **Is it mechanically jammed?** The screen will happily show a commanded angle while
   the motor is stalled against a stop. A stalled motor gets hot fast. If any motor or
   wire feels warm, **cut power now**.

### The arm jerks, twitches, or the board keeps resetting

Almost always **power**, not software.

| What you see | What it usually means | What to do |
|---|---|---|
| Board resets, console disconnects, cards go grey | **Brownout.** The supply collapsed under load | Fewer joints at once, lower deg/s, and get the 3–5 A supply |
| One joint jerks instead of gliding | deg/s set too high for that joint's load | Drop it to 10–15 in the card's deg/s box |
| Twitching that comes and goes with no pattern | Servo current running through breadboard contacts | Run servo `+` and `−` from the screw terminals **straight to the motor**. Breadboard springs are about 1 A at best and get worse with use |
| A hard kick every time a joint starts moving | No reservoir capacitor | Fit the 470–1000 µF capacitor across the servo supply, close to the motors. Watch polarity — the stripe is negative |
| Buzzing that never settles | The motor is fighting something | DISABLE that joint immediately. Then look for a hard stop, a tight linkage, or — on the shoulder — a wrong `mirror_mode` |
| Everything jerks whenever several joints move together | Seven motors, 0.7 amps | This is section 1. Nothing in software fixes it |

The full power wiring, with sources, is in `V2-SERVO-POWER-AND-WIRING.md`.

### The launcher says "the pyserial package is missing"

`pyserial` is the small add-on that lets Python talk to a USB port. Without it the
bridge cannot open the Arduino.

The launcher prints the exact one-line command to fix it. **Copy that line exactly** —
type it into the same terminal window and press Enter, then close the window and run
the platform launcher again.

Why "exactly" matters: this laptop has **four different Pythons installed**, and only
one of them has pyserial. The launcher checks each one and uses whichever can actually
talk to the port, so the fix line it prints is aimed at the right Python. A pip command
you found somewhere else may install it into a Python the launcher is not using, and
nothing will change.

If you would rather not deal with Python at all, skip it entirely: double-click
`Software\arm-console\arm-console.html` and use the console from Chrome or Edge.

### The browser page is blank or says it cannot connect

The bridge is not running. Look for the `FactoryLM Arm GUI` terminal window. If it is not
there, run the platform launcher again and read what it prints — it explains its
own failures in plain English and never closes without telling you why.

### The port list is empty right after restarting the bridge — and the Arduino is definitely plugged in

Symptoms: the page says **"No serial ports found"** or nothing appears in the port list,
pressing REFRESH changes nothing, and the terminal window is open and looks healthy. Some
buttons may also complain about an "access code".

**You are looking at a tab left over from a previous run of the bridge.** Each time the
bridge starts it makes a new one-time code (section 4), and the old tab is still showing the
old one, so the bridge refuses it. The refusal reaches the page as an empty port list, which
looks exactly like an unplugged Arduino — so check this *before* going hunting for a cable.

**Press F5 on that tab.** It will pick up the new code by itself. If that does not do it,
close the tab and click the link printed in the terminal window.

This is worth recognising because it looks alarming and is completely harmless — the same
thing happens any time you restart the bridge without closing the browser. Nothing is
broken, nothing was sent to the board, and no setting needs changing.

### The board keeps its own state, so check it after any of this

Whenever the console has been disconnected and reconnected — a restarted bridge, a reloaded
tab, a replugged cable — the Arduino restarted too, and **it remembers nothing**. Every
joint comes back disabled, uncalibrated, at the 70-110 placeholder, with the shoulder
mirror unknown. That is by design. Reload your limits file and re-enable joints one at a
time with fresh angles, exactly as you did the first time.

---

## The short version

**Today, with no motor power at all:** upload `factorylm_arm_controller`, run
the platform launcher, connect, enable a joint, drag the slider, record a couple of
waypoints, play them back. That proves the whole chain end to end.

**Next:** buy the regulated 5 V, 3–5 A supply. Fit the capacitor. Get servo current off
the breadboard.

**Then:** measure each joint's real travel one at a time with the single-motor bench
sketch, write it into `joint-limits.csv`, and do the shoulder mirror test.

**Finally:** the first powered session — arm supported, one joint at a time, lightest
first, hand on the rocker switch.

---

## Where else to look

| File | What it covers |
|---|---|
| `SERIAL-PROTOCOL.md` | The full command list and error codes — the technical reference |
| `HOW-TO-TEST-EACH-SERVO.md` | Bench-testing one motor at a time, in detail |
| `V2-SERVO-POWER-AND-WIRING.md` | The correct power wiring, with sources |
| `USB-SERIAL-DIAGNOSIS.md` | Why the clone Uno will not program on this laptop |
| `NEXT-STEPS-PLAIN-ENGLISH.md` | The earlier plain-English walkthrough |
| `Calibration_Notes/calibration-log.csv` | The running record of what you measured |

Any time you want to confirm nothing has broken on the laptop side, double-click
`CHECK SETUP.bat` on Windows or `CHECK SETUP.command` on macOS. Green all the way down
means the tools are fine.

---

## Last thing

This is a supervised hobby bench procedure, not an industrial safety standard.

**The Arduino, the firmware, the bridge, the browser page and the red E-STOP button are
not safety devices.** They are conveniences that make a hobby arm easier to control. Any
one of them can freeze, crash, or lose its USB cable at the worst moment.

The rocker switch and the inline fuse are the real stop. Keep the arm supported so
gravity cannot drop a joint, keep your hands out of the sweep, and keep one hand on
that switch.
