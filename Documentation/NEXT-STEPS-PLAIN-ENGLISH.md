# What To Do Next — Plain English

Written 2026-08-01, after the laptop and board were set up and tested.
Companion to `SETUP-STATUS.md` (the technical version).

---

## Where you are right now

The laptop and the board are finished. They talk to each other, and you proved it — the
small light on the board is blinking because we put a program there and it ran.

Nothing has moved yet. No motor has had power. That is on purpose.

Three parts to this project:

1. **The computer side** — finished.
2. **The program side** — ready, not loaded yet. That is next.
3. **The electrical side** — not ready. Parts are missing. Nothing physical moves until
   that is built.

**Use the genuine Arduino Uno (on COM5). The other board — the clone — cannot be
programmed on this laptop.** Reason is written up in `USB-SERIAL-DIAGNOSIS.md`. It is
not your fault and not a broken board; Windows installed a faulty USB driver for it.

---

## Step 1 — Put the arm test program on the board

The board currently holds the blinking-light program. You are going to replace it with
the program that talks to the arm's motors.

1. Open **Arduino IDE**.
2. Click **File → Open**.
3. Go to `C:\RobotArm\Software\emre_kalem_single_servo_bench_test\` and open the file
   ending in `.ino`. That is the program.
4. Check the top of the window says **Arduino Uno** and **COM5**. It should already.
5. Click the **right-arrow button** at the top left. That copies the program onto the
   board.
6. Wait for **"Done uploading."**

**Before you click anything: the black rocker switch stays OFF, and no motor is
connected to power.** Loading a program is safe with everything off — the program is
written so the board sends no signals to any motor until you specifically tell it to.

---

## Step 2 — Open the text window

The board can print messages to your screen and take typed commands. That window is
called the **Serial Monitor**.

1. Click the **magnifying-glass icon** at the top right of Arduino IDE.
2. A panel opens at the bottom.
3. Find the speed dropdown in that panel and set it to **115200**.

You should see a welcome message, safety reminders, and a list of commands.

If you see nonsense symbols instead of words, the speed is wrong. That is the only thing
that causes it.

---

## Step 3 — Practise with nothing plugged in

Do this before any motor is anywhere near the board. It costs nothing and teaches you
the sequence while nothing can move.

Type these one at a time and read what comes back:

| Type | What it does |
|---|---|
| `5` | Picks the wrist-roll motor — the smallest, gentlest one |
| `c` | Sets the target to the middle position, 90 degrees |
| `a` | Tells the board to start sending signals |
| `+` | Move one degree one way |
| `-` | Move one degree back |
| `d` | Stop sending signals |

Type `h` any time for the full list. Type `s` to see what the program currently thinks
is going on.

With no motor connected, nothing happens physically — you just watch the messages. That
is the point. When a real motor is attached later, you will already know the rhythm
instead of learning it while something can swing.

Two safety rules are built into the program and you cannot type your way past them:

- It will never go below 70 degrees or above 110. Straight up (0) and straight over
  (180) are impossible. A 3D-printed joint can hit its own physical stop long before the
  motor reaches those numbers.
- When the board powers up it sends nothing. It stays silent until you type `a`.

---

## Step 4 — What you still need before anything moves

This is the real blocker, and it is parts, not software.

A motor cannot run off the USB cable. USB does not supply enough current, and trying it
can make the board reset or damage it. The motors need their own power supply.

You need:

- A **5-volt power supply** — a regulated one, not a random phone charger
- A **fuse** in the positive wire
- The **KCD1 rocker switch**, so you can cut power instantly with one hand
- **Proper insulated connectors** — screw terminals or lever connectors, not twisted
  bare wire and not alligator clips
- A **multimeter**, to confirm it really is putting out 5 volts the right way round
  before anything is connected

The wiring rule, and this one really matters:

> **USB goes to the board. The 5-volt supply goes to the motor.
> The only wire they share is ground.**

Never run the 5-volt supply's positive wire into the board's `5V` pin, its `VIN` pin, or
its round power socket. That is the mistake that kills boards.

When you get there, start with the **wrist-roll motor on pin D9**, with the arm part
unbolted from it so it spins freely. Smallest motor, least to push against — the safest
thing to get wrong first.

---

## Step 5 — Two program files are missing

The download contains only the 3D-printing files for the plastic parts. Two programs the
instruction booklet keeps referring to were never in it:

- `emre_kalem_arm_calibrate.ino`
- `emre_kalem_arm_uno_controller.ino`

Substitutes were deliberately not written. The booklet tells you to use "the actual
calibration program's commands" — a made-up program carrying that name would let you
believe you were following the designer's tested procedure when you were not. That is a
safety problem, not an inconvenience.

Ask whoever supplied the arm for those two files. The test program you have covers the
whole one-motor-at-a-time stage on its own — the booklet says so itself.

---

## The short version

**Now:** load the test program, open the text window at 115200, practise the commands
with nothing connected.

**Then:** gather the 5-volt supply, fuse, switch, connectors, and multimeter.

**After that:** one motor, unbolted, centred, tiny movements, one hand on the switch.

Any time you want to confirm nothing has broken, double-click
`C:\RobotArm\CHECK SETUP.bat`. Green all the way down means you are fine.

---

## One last thing

This is a hobby bench procedure, not an industrial safety standard. The board, the
program, the rocker switch, and the software stop are **not** safety devices. Keep the
arm supported so gravity cannot drop a joint, keep your hand away from the moving parts,
and keep your other hand at the switch.
