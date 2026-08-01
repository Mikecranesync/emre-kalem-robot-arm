/*
 * emre_kalem_single_servo_bench_test
 * ----------------------------------
 * Safe, limited ONE-SERVO bench test for the Emre Kalem 6-axis arm on an Arduino Uno.
 *
 * Written to the contract in "Emre Kalem Robot Arm - Arduino Uno Beginner Walkthrough"
 * (Phase 2, section "Included backup single-servo bench test"):
 *
 *   - Commands are limited to 70 deg - 110 deg. Nothing outside that range is possible.
 *   - The servo signal starts DETACHED. No pulses leave the Uno until you type 'a'.
 *   - Only one servo may be attached at a time.
 *   - It does NOT test the paired shoulder as a pair. Center D4 and D5 separately.
 *   - Serial Monitor: 115200 baud. Any line-ending setting works (CR/LF are ignored).
 *
 * POWER RULES (from the walkthrough - do not violate):
 *   USB powers the Uno.  An external regulated 5 V supply powers the servo.
 *   ONLY THE GROUNDS ARE JOINED.
 *   Never connect external +5 V or a servo red wire to Uno 5V, VIN, or the barrel jack.
 *
 * This is a supervised hobby-robot bench procedure, not a safety-rated commissioning
 * tool. This sketch, the rocker switch, and the Uno are NOT safety devices.
 */

#include <Servo.h>

// ---------------------------------------------------------------------------
// Hard safety limits. Do not widen these to "test the full range".
// The advertised servo range is not the safe mechanical range of a printed joint.
// ---------------------------------------------------------------------------
const int ANGLE_MIN    = 70;   // degrees - hard clamp
const int ANGLE_MAX    = 110;  // degrees - hard clamp
const int ANGLE_CENTER = 90;   // degrees - the only start position
const int STEP_SMALL   = 1;    // '+' / '-'
const int STEP_LARGE   = 5;    // ']' / '['

// ---------------------------------------------------------------------------
// Servo map. Matches wiring-map.csv and the walkthrough pin table.
// Select a servo by typing its index digit (0-6).
// ---------------------------------------------------------------------------
const int SERVO_COUNT = 7;
const int  SERVO_PIN[SERVO_COUNT]  = {   3,             4,               5,                6,      9,             10,           11        };
const char* SERVO_NAME[SERVO_COUNT] = { "Base",        "Shoulder left", "Shoulder right", "Elbow", "Wrist pitch", "Wrist roll", "Gripper" };

Servo servo;

int selectedIndex = -1;              // -1 = nothing selected yet
int targetAngle   = ANGLE_CENTER;    // last commanded angle
bool centerStored = false;           // set by 'c'
bool attached     = false;           // true only between 'a' and 'd'

// ---------------------------------------------------------------------------

void printBanner() {
  Serial.println();
  Serial.println(F("========================================================"));
  Serial.println(F(" EMRE KALEM ARM - SINGLE SERVO BENCH TEST"));
  Serial.println(F("========================================================"));
  Serial.print  (F(" Angle limit: "));
  Serial.print(ANGLE_MIN); Serial.print(F(" - ")); Serial.print(ANGLE_MAX);
  Serial.println(F(" degrees. Center = 90."));
  Serial.println(F(" Signal starts DETACHED. No pulses until you type 'a'."));
  Serial.println(F(" One servo at a time. Shoulder pair is NOT tested paired."));
  Serial.println(F("--------------------------------------------------------"));
  Serial.println(F(" BEFORE YOU TYPE ANYTHING:"));
  Serial.println(F("   KCD1 rocker OFF. External 5 V supply OFF."));
  Serial.println(F("   Servo red  -> external +5 V (switched + fused) ONLY"));
  Serial.println(F("   Servo brown/black -> external GND"));
  Serial.println(F("   External GND -> an Arduino GND pin (common ground)"));
  Serial.println(F("   Servo signal -> its Arduino digital pin"));
  Serial.println(F("   NEVER connect external +5 V to Uno 5V / VIN / barrel jack."));
  printHelp();
}

void printHelp() {
  Serial.println(F("--------------------------------------------------------"));
  Serial.println(F(" COMMANDS (type the character, Enter optional)"));
  Serial.println(F("   0..6  select servo:"));
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.print(F("           ")); Serial.print(i);
    Serial.print(F("  ")); Serial.print(SERVO_NAME[i]);
    Serial.print(F("  (D")); Serial.print(SERVO_PIN[i]); Serial.println(F(")"));
  }
  Serial.println(F("   c     store the center command: 90 degrees"));
  Serial.println(F("   a     attach the signal and output 90 (servo power still OFF)"));
  Serial.println(F("   +     move +1 degree      -   move -1 degree"));
  Serial.println(F("   ]     move +5 degrees     [   move -5 degrees"));
  Serial.println(F("   d     DETACH the signal (do this before rewiring)"));
  Serial.println(F("   s     show status         h / ?  show this help"));
  Serial.println(F("   --- diagnostics (only work after 'a') ---"));
  Serial.println(F("   w     WIDE visibility test: 90 -> 110 -> 70 -> 90"));
  Serial.println(F("         Big obvious moves. Use when you cannot tell if it moved."));
  Serial.println(F("   m     MEASURE mode: hold 90 for 60 s, then auto-detach."));
  Serial.println(F("         Frees both hands to probe with a multimeter."));
  Serial.println(F("         Any keypress aborts early."));
  Serial.println(F("--------------------------------------------------------"));
  Serial.println(F(" NORMAL ORDER: 5 -> c -> a -> [rocker ON] -> + -> - -> d"));
  Serial.println(F(" Only use ] and [ after the +/- one-degree test passes."));
  Serial.println();
}

void printStatus() {
  Serial.print(F("[status] servo="));
  if (selectedIndex < 0) {
    Serial.print(F("none"));
  } else {
    Serial.print(SERVO_NAME[selectedIndex]);
    Serial.print(F(" (D")); Serial.print(SERVO_PIN[selectedIndex]); Serial.print(F(")"));
  }
  Serial.print(F("  signal="));
  Serial.print(attached ? F("ATTACHED") : F("detached"));
  Serial.print(F("  center_stored="));
  Serial.print(centerStored ? F("yes") : F("no"));
  Serial.print(F("  angle="));
  Serial.println(targetAngle);
}

// Clamp to the hard limits. This is the safety net - every write goes through it.
int clampAngle(int a) {
  if (a < ANGLE_MIN) return ANGLE_MIN;
  if (a > ANGLE_MAX) return ANGLE_MAX;
  return a;
}

void detachSignal(const __FlashStringHelper* why) {
  if (attached) {
    servo.detach();
    attached = false;
    Serial.print(F("[ok] signal DETACHED. "));
    Serial.println(why);
  } else {
    Serial.println(F("[info] signal was already detached."));
  }
}

void selectServo(int index) {
  if (attached) {
    // Never leave a servo driven while the operator moves to another one.
    detachSignal(F("(auto-detached because you selected a different servo)"));
  }
  selectedIndex = index;
  centerStored  = false;
  targetAngle   = ANGLE_CENTER;
  Serial.print(F("[ok] selected "));
  Serial.print(SERVO_NAME[index]);
  Serial.print(F(" on D"));
  Serial.println(SERVO_PIN[index]);
  Serial.println(F("     next: 'c' to store center 90, then 'a' to attach."));
}

void storeCenter() {
  if (selectedIndex < 0) {
    Serial.println(F("[stop] select a servo first (type 0..6)."));
    return;
  }
  targetAngle  = ANGLE_CENTER;
  centerStored = true;
  Serial.println(F("[ok] center command stored: 90 degrees."));
  Serial.println(F("     next: 'a' to attach. Servo power should still be OFF."));
}

void attachSignal() {
  if (selectedIndex < 0) {
    Serial.println(F("[stop] select a servo first (type 0..6)."));
    return;
  }
  if (!centerStored) {
    Serial.println(F("[stop] store the center first (type 'c')."));
    return;
  }
  if (attached) {
    Serial.println(F("[info] already attached. Type 'd' first if you rewired."));
    return;
  }
  targetAngle = ANGLE_CENTER;
  servo.attach(SERVO_PIN[selectedIndex]);
  servo.write(targetAngle);
  attached = true;
  Serial.print(F("[ok] signal ATTACHED on D"));
  Serial.print(SERVO_PIN[selectedIndex]);
  Serial.println(F(", output 90 degrees."));
  Serial.println(F("     Stand clear. Keep one hand at the rocker."));
  Serial.println(F("     Now turn the KCD1 rocker ON. The servo moves to center."));
}

void moveBy(int delta) {
  if (!attached) {
    Serial.println(F("[stop] signal is detached. Type 'a' to attach first."));
    return;
  }
  int requested = targetAngle + delta;
  int applied   = clampAngle(requested);
  if (applied != requested) {
    Serial.print(F("[limit] "));
    Serial.print(requested);
    Serial.print(F(" is outside "));
    Serial.print(ANGLE_MIN); Serial.print(F("-")); Serial.print(ANGLE_MAX);
    Serial.println(F(". Clamped."));
  }
  targetAngle = applied;
  servo.write(targetAngle);
  Serial.print(F("[ok] angle = "));
  Serial.println(targetAngle);
}

// Big, unmistakable moves inside the 70-110 clamp. The +/-1 and +/-5 tests are the
// right SAFETY ladder but they are useless as a DETECTION test: on a bare output
// spline with the horn removed, five degrees is close to invisible. This answers
// the question "is this servo moving at all?" and nothing else.
void wideVisibilityTest() {
  if (!attached) {
    Serial.println(F("[stop] signal is detached. Type 'a' to attach first."));
    return;
  }
  Serial.println(F("[wide] LOOK AT THE OUTPUT SHAFT NOW."));
  Serial.println(F("       Tape a toothpick to it if there is no horn fitted -"));
  Serial.println(F("       a bare spline is very hard to read."));

  const int steps[] = { ANGLE_MAX, ANGLE_CENTER, ANGLE_MIN, ANGLE_CENTER };
  for (uint8_t i = 0; i < 4; i++) {
    targetAngle = clampAngle(steps[i]);
    servo.write(targetAngle);
    Serial.print(F("[wide] angle = "));
    Serial.println(targetAngle);
    delay(1200);
  }
  Serial.println(F("[wide] done. Still attached. Type 'd' to stop the signal."));
}

// Hold a steady centre pulse so the operator can probe with a multimeter using
// both hands. Auto-detaches so the servo is never left driven by accident.
// Expected reading on the signal pin vs GND: about 0.37 V DC
// (1472 us pulse / 20000 us frame * 5 V). A cheap meter averaging 50 Hz PWM
// will wander either side of that - the useful distinction is
// "roughly a third of a volt" vs "zero" vs "about five".
void holdForMeasurement() {
  if (!attached) {
    Serial.println(F("[stop] signal is detached. Type 'a' to attach first."));
    return;
  }
  targetAngle = ANGLE_CENTER;
  servo.write(targetAngle);

  Serial.println(F("[measure] holding 90 degrees for 60 seconds."));
  Serial.print  (F("[measure] probe D"));
  Serial.print(SERVO_PIN[selectedIndex]);
  Serial.println(F(" against GND, meter on DC volts."));
  Serial.println(F("[measure]   ~0.37 V  = pulses are being generated (good)"));
  Serial.println(F("[measure]   0 V      = no signal on this pin"));
  Serial.println(F("[measure]   ~5 V     = pin stuck high, not pulsing"));
  Serial.println(F("[measure] Also probe the SERVO PLUG red-to-brown: expect ~5 V."));
  Serial.println(F("[measure] Press any key to stop early."));

  for (int8_t s = 60; s > 0; s--) {
    if (Serial.available() > 0) {
      while (Serial.available() > 0) { Serial.read(); }
      Serial.println(F("[measure] aborted by keypress."));
      break;
    }
    if (s % 10 == 0) {
      Serial.print(F("[measure] "));
      Serial.print(s);
      Serial.println(F(" s remaining..."));
    }
    delay(1000);
  }

  detachSignal(F("(measure mode finished - auto-detached)"));
}

void handleCommand(char ch) {
  if (ch >= '0' && ch <= '6') { selectServo(ch - '0'); return; }

  switch (ch) {
    case 'c': case 'C': storeCenter();           break;
    case 'a': case 'A': attachSignal();          break;
    case '+':           moveBy(+STEP_SMALL);     break;
    case '-':           moveBy(-STEP_SMALL);     break;
    case ']':           moveBy(+STEP_LARGE);     break;
    case '[':           moveBy(-STEP_LARGE);     break;
    case 'd': case 'D': detachSignal(F("Safe to turn the rocker OFF and rewire.")); break;
    case 's': case 'S': printStatus();           break;
    case 'w': case 'W': wideVisibilityTest();    break;
    case 'm': case 'M': holdForMeasurement();    break;
    case 'h': case 'H': case '?': printHelp();   break;
    case '7': case '8': case '9':
      Serial.println(F("[stop] no such servo. Valid indexes are 0..6."));
      break;
    default:
      Serial.print(F("[stop] unknown command '"));
      Serial.print(ch);
      Serial.println(F("'. Type 'h' for help."));
      break;
  }
}

void setup() {
  // Serial Monitor baud rate. Set Serial Monitor to this exact number: 115200
  Serial.begin(115200);
  // Do NOT attach here. The signal must be detached at power-up.
  while (!Serial && millis() < 3000) { /* wait briefly for the Serial Monitor */ }
  printBanner();
  printStatus();
}

void loop() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r' || ch == '\n' || ch == ' ' || ch == '\t') continue;  // ignore line endings
    handleCommand(ch);
  }
}
