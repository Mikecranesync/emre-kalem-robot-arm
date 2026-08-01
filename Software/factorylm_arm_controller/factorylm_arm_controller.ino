/*
 * factorylm_arm_controller
 * =============================================================================
 * Multi-joint serial controller for the Emre Kalem 6-axis arm on an Arduino Uno.
 * FactoryLM Arm Serial Protocol v1.0.  115200 8N1, line-oriented ASCII.
 *
 * THIS FIRMWARE IS OURS.  The vendor never shipped a controller sketch.  This is
 * a direct extension of emre_kalem_single_servo_bench_test.ino and it keeps every
 * one of that sketch's safety invariants:
 *
 *   - Every servo signal starts DETACHED.  Nothing is driven until commanded.
 *   - Every write goes through one clamp.  Per-joint limits are enforced there.
 *   - Changing what a joint is doing always detaches first.
 *   - Limits are DATA, never guesses.  They default to 70-110 degrees and stay
 *     flagged UNCALIBRATED until a human measures the real travel and loads it.
 *
 * -----------------------------------------------------------------------------
 * POWER RULES  (from the walkthrough and V2-SERVO-POWER-AND-WIRING.md)
 * -----------------------------------------------------------------------------
 *   USB powers the Uno.  A separate regulated supply powers the servos.
 *   ONLY THE GROUNDS ARE JOINED.
 *   Never connect external +5 V or a servo red wire to Uno 5V, VIN, or the
 *   barrel jack.
 *
 *   The missing common ground was the root cause of the entire 2026-08-01
 *   no-motion episode.  One wire: any Arduino GND to the supply negative.
 *
 *   The present 6.62 V adapter is OUT OF SPEC for the three MG90S units
 *   (D9 wrist pitch, D10 wrist roll, D11 gripper - rated 4.8-6.0 V), and at
 *   700 mA it cannot drive an assembled arm at all.  That needs 3-5 A.
 *
 * -----------------------------------------------------------------------------
 * THE REAL E-STOP IS THE KCD1 ROCKER AND THE INLINE FUSE.
 * -----------------------------------------------------------------------------
 *   This sketch, the EST command, the '!' byte, the serial watchdog and the Uno
 *   are conveniences.  They are NOT safety devices.  A USB cable, a browser tab
 *   or a host process can all die mid-move.  The only real stop is removing
 *   servo power.  This is a supervised hobby-robot bench procedure, not a
 *   safety-rated system.
 *
 * -----------------------------------------------------------------------------
 * JOINT IDs - six addressable, ID 2 permanently reserved
 * -----------------------------------------------------------------------------
 *   IDs match the index column of Software/wiring-map.csv so this project has
 *   exactly ONE numbering system.  calibration-log.csv already carries a
 *   correction caused by index/label confusion; a second numbering space would
 *   recreate that bug.
 *
 *     0  Base            D3        MG996R  (inferred)
 *     1  Shoulder PAIR   D4 + D5   2x MG996R (inferred) - ONE logical joint
 *     2  RESERVED        -         never addressable
 *     3  Elbow           D6        MG996R  (inferred)
 *     4  Wrist pitch     D9        MG90S   (inferred)
 *     5  Wrist roll      D10       MG90S   (DOC-CONFIRMED)
 *     6  Gripper         D11       MG90S   (inferred)
 *
 *   ID 2 is the shoulder pair's second servo (D5).  It is NOT commandable from
 *   the wire - every command naming joint 2 returns
 *   "ERR E4 <VERB> JOINT=2 RESERVED=shoulder_pair".  The gap is deliberate and
 *   self-documenting: it explains itself every time somebody types "ENA 2".
 *   Two MG996Rs fighting through one shoulder link is the highest-consequence
 *   mechanical failure available on this arm, so D5 is only ever driven as the
 *   mirror of D4, at the single point of write.
 *
 * -----------------------------------------------------------------------------
 * TWO Servo-LIBRARY LANDMINES THIS SKETCH IS BUILT AROUND
 * -----------------------------------------------------------------------------
 * Both verified in the installed Servo 1.3.0 source, src/avr/Servo.cpp.
 *
 * 1. detach() ALONE CAN LATCH A PIN HIGH FOREVER.
 *    The ISR drives a channel's pin HIGH when its slot becomes current and only
 *    drives it LOW at the NEXT interrupt, gated on Pin.isActive (lines 57-58).
 *    detach() clears isActive immediately (line 257).  A detach landing inside
 *    that window skips the LOW write and the pin sits driven HIGH forever - an
 *    infinitely long pulse.  Servos hold, hunt, or slew to an endpoint and stall
 *    at maximum current.  Upstream bug arduino-libraries/Servo#160.
 *
 *    EVERY safety mechanism here routes through detach, so there is exactly ONE
 *    way to stop driving a joint: disableJoint(), which always follows detach()
 *    with pinMode(OUTPUT) + digitalWrite(LOW).  Never call detach() anywhere
 *    else.  This failure is silent and intermittent - it will pass casual
 *    testing and fail during a real e-stop with the arm loaded.  Confirm it once
 *    at the bench with the multimeter procedure in V2-SERVO-POWER-AND-WIRING.md
 *    section 6: a healthy signal pin reads ~0.37 V, a stuck-high pin reads ~5 V.
 *
 * 2. attach() COMMANDS ~1500 us (CENTRE) ON THE VERY NEXT FRAME.
 *    The constructor sets ticks = usToTicks(1500) (line 227) and attach() never
 *    touches it.  A bare attach(pin) therefore snaps the joint to centre within
 *    one 20 ms frame - and that fires on EVERY per-joint enable, not just boot.
 *    With the arm being reassembled at unknown joint positions this is the
 *    single highest-risk event in the project.
 *
 *    So: enableJoint() always pre-loads the adopt pulse with writeMicroseconds()
 *    BEFORE attach().  That is legal - writeMicroseconds is guarded only by
 *    "channel < MAX_SERVOS" (line 279) and servoIndex is assigned in the
 *    constructor.  The pre-attach clamp uses this->min / this->max, which the
 *    constructor never sets; for FILE-SCOPE objects those are zero-initialised,
 *    giving a 544..2400 us clamp - wider than anything we ever command.  That is
 *    why the Servo objects MUST stay at file scope and why we always call the
 *    plain attach(pin) form, never the 3-argument one.
 *
 *    There is NO home command, NO centre-on-boot and NO centre-on-enable.
 *    "Centre it slowly" is not an acceptable compromise: slow motion into an
 *    unknown mechanical stop still stalls the servo at full current.
 *
 * -----------------------------------------------------------------------------
 * WHY THERE IS NO AVR HARDWARE WATCHDOG (avr/wdt.h) IN v1
 * -----------------------------------------------------------------------------
 *   It would be safe on this board - the installed Optiboot 4.4 reads and zeroes
 *   MCUSR and calls watchdogConfig(WATCHDOG_OFF) before jumping to the sketch,
 *   so there is no reset loop.  It is omitted anyway because:
 *     (a) the SERIAL watchdog below already covers the failure mode we care
 *         about, which is "the host died while a joint was driven";
 *     (b) a WDT reset drops every pin to INPUT, so a gravity-loaded arm goes
 *         limp and FALLS, and the adopted position is lost;
 *     (c) this sketch has no dynamic allocation and no delay(), so the lockup
 *         surface it would protect is very small;
 *     (d) simplicity is an explicit requirement.
 *   Revisit once the arm is assembled and loaded, when it is clear whether a
 *   limp-arm failsafe or a held-pose failsafe is the lesser evil.
 *
 * -----------------------------------------------------------------------------
 * NO delay() ANYWHERE.  This is a hard precondition, not a style note.
 * -----------------------------------------------------------------------------
 *   The realtime '!' e-stop byte is silently defeated by any blocking call.  The
 *   bench sketch's delay(1200)/delay(1000) patterns are deliberately NOT carried
 *   over.  Everything here is a millis()-based state machine.  Put this on the
 *   review checklist for every commit: a blocking call is invisible until the
 *   moment it matters.
 *
 * These servos have NO position feedback of any kind.  Every angle this firmware
 * reports is what it COMMANDED, never what the joint actually did.
 */

#include <Servo.h>
#include <avr/pgmspace.h>
#include <string.h>

// Eight Servo objects are constructed (seven slots + the shoulder's second
// servo).  If the library could not hand out that many channels, servoIndex
// would be INVALID_SERVO and writeMicroseconds would silently do nothing -
// a silent failure on a machine that moves.  Fail at compile time instead.
//
// This has to be static_assert, not #if: MAX_SERVOS expands to
// (_Nbr_16timers * SERVOS_PER_TIMER) and _Nbr_16timers is an ENUM, which the
// preprocessor cannot see - it would silently evaluate to 0 and always fire.
// On the Uno (ATmega328P) _Nbr_16timers == 1, so MAX_SERVOS == 12.
static_assert(MAX_SERVOS >= 8, "Servo library MAX_SERVOS < 8 - this sketch needs 8 channels.");

// =============================================================================
// CONFIGURATION
// =============================================================================

const uint8_t NJ          = 7;     // joint slots, indexed by JOINT ID 0..6
const uint8_t RESERVED_ID = 2;     // the shoulder pair's second servo - not addressable
const uint8_t NO_PIN      = 255;   // sentinel; see the guard note below

// Pin table, indexed by joint id.  Slot 2 is allocated but has no pin.
//
// Every function that touches a pin guards on "pin >= NUM_DIGITAL_PINS" rather
// than on "i == RESERVED_ID".  Making the guard structural means a future loop
// that forgets to skip slot 2 still cannot drive anything.  Do not index
// PIN_A / PIN_B anywhere except behind that guard.
const uint8_t PIN_A[NJ] = {   3,   4, NO_PIN,      6,      9,     10,     11 };
const uint8_t PIN_B[NJ] = { NO_PIN, 5, NO_PIN, NO_PIN, NO_PIN, NO_PIN, NO_PIN };

// Boot defaults.  Deliberately narrow and deliberately flagged uncalibrated.
// Exactly ONE provisional, unloaded calibration row exists in this whole
// project, so we do not know any joint's real mechanical travel.
const uint8_t DEF_MIN_DEG =  70;
const uint8_t DEF_MAX_DEG = 110;
const int16_t DEF_SET_C   = 9000;  // 90.00 degrees, in centidegrees
const uint8_t DEF_DPS     =  30;   // degrees per second

const uint8_t  SPD_MIN_DPS =  1,     SPD_MAX_DPS  = 90;
const uint16_t WDG_MIN_MS  = 200,    WDG_MAX_MS   = 10000;
const uint8_t  TICK_MS     = 20;     // interpolator period
const uint8_t  TICK_CAP_MS = 200;    // largest elapsed slice honoured after a stall

// Hard ceiling on how far ONE interpolator write may move a joint, whatever the
// elapsed time was.  TICK_CAP_MS caps ELAPSED TIME, which is NOT the same thing:
// at 90 deg/s a 200 ms slice is still an 18 degree jump out of a single write.
//
// 200 centidegrees = 2.00 degrees is exactly one nominal 20 ms tick at
// SPD_MAX_DPS (90 * 20 / 10 = 180 cd), so no legal speed is throttled - only the
// post-stall catch-up burst is.  A 1.00 degree cap would have silently limited
// every speed above 50 deg/s, which is inside the legal 1..90 range.
const int16_t MAX_STEP_C = 200;

// Largest mirror offset accepted on the wire, degrees.  Bounds mirrorOffsetC so
// the int16 store and the 18000 + 2*offset arithmetic cannot overflow before the
// envelope check below gets a chance to run.
const int16_t MIR_OFF_MAX_DEG = 90;
const uint8_t  LINE_MAX    = 48;     // inbound line buffer including terminator
const uint8_t  RX_BUDGET   = 32;     // bytes drained per loop(), so a paste cannot
                                     // starve the interpolator

// Mirror relation for the shoulder pair.  UNKNOWN is the only honest default.
const uint8_t MIR_UNKNOWN = 0;
const uint8_t MIR_SAME    = 1;
const uint8_t MIR_INV     = 2;

// =============================================================================
// STATE
// =============================================================================

// FILE SCOPE IS REQUIRED - see landmine 2 in the header comment.
Servo sA[NJ];   // primary servo per joint
Servo sB;       // joint 1's second servo, on D5

struct Joint {
  int16_t setC;   // angle commanded right now, centidegrees
  int16_t tgtC;   // where the interpolator is walking to, centidegrees
  uint8_t minD;   // limit, degrees - DATA, loaded from joint-limits.csv
  uint8_t maxD;   // limit, degrees
  uint8_t dps;    // slew rate, degrees per second
  bool    en;     // true only between ENA and DIS
  bool    cal;    // false = these limits are a placeholder, not a measurement
};
Joint j[NJ];

uint8_t  mirMode      = MIR_UNKNOWN;

// The INV mirror axis, as an offset from 90.00 degrees, in centidegrees.
// "Mirror about exactly 90.00" was an unstated assumption: it is only true if
// both shoulder horns were fitted symmetrically about the same reference, which
// nothing in this project recorded.  The axis is DATA now, defaulting to 0 (the
// old behaviour) and loaded from joint-limits.csv column mirror_offset_deg.
int16_t  mirrorOffsetC = 0;

bool     estopLatched = false;
bool     wdgTripped   = false;   // the CURRENT latch came from the watchdog, not the operator
uint16_t wdgMs        = 0;       // 0 = disabled (the boot default)
uint32_t lastRxMs     = 0;       // fed ONLY from handleLine(), after dispatch()
uint32_t lastTickMs   = 0;

char    line[LINE_MAX];
uint8_t lineLen  = 0;
bool    lineOver = false;

char  g_verb[4] = {0, 0, 0, 0};  // uppercased verb, echoed on every reply
char* tokv[4];                   // argument tokens
uint8_t tokc = 0;

// =============================================================================
// SMALL HELPERS
// =============================================================================

// Centidegrees -> microseconds, integer only.  AVR has no FPU and software
// float drags in 1-2 KB of flash.  Matches the library's own 544/2400 mapping:
// 9000 (90.00 deg) -> 1472 us, which is the figure the V2 doc measured.
static uint16_t degCToUs(int16_t c) {
  return (uint16_t)(544 + (((int32_t)c * 1856) / 18000));
}

// Centidegrees -> whole degrees for the wire.  Every angle on the wire is an
// integer degree; centidegrees exist only in here.
static int16_t degOf(int16_t c) {
  return (int16_t)((c + 50) / 100);
}

static const __FlashStringHelper* mirName() {
  if (mirMode == MIR_SAME) return F("SAME");
  if (mirMode == MIR_INV)  return F("INV");
  return F("UNKNOWN");
}

static bool anyEnabled() {
  for (uint8_t i = 0; i < NJ; i++) if (j[i].en) return true;
  return false;
}

static void upcase(char* s) {
  while (*s) { if (*s >= 'a' && *s <= 'z') *s -= 32; s++; }
}

// Strict integer parser.  NOT atoi: atoi returns 0 on garbage, which is
// indistinguishable from a real 0.  NOT sscanf: that drags in ~1.5 KB of AVR
// stdio.  Rejects an empty field, any non-digit, and overflow.
static bool parseInt(const char* s, int32_t* out) {
  if (*s == 0) return false;
  bool neg = false;
  if (*s == '-') { neg = true; s++; if (*s == 0) return false; }
  int32_t v = 0;
  while (*s) {
    if (*s < '0' || *s > '9') return false;
    v = v * 10 + (int32_t)(*s - '0');
    if (v > 100000L) return false;             // far beyond any legal argument
    s++;
  }
  *out = neg ? -v : v;
  return true;
}

// Split off the next whitespace-delimited token, NUL-terminating it in place.
static char* nextTok(char** p) {
  char* s = *p;
  while (*s == ' ' || *s == '\t') s++;
  if (*s == 0) { *p = s; return NULL; }
  char* start = s;
  while (*s && *s != ' ' && *s != '\t') s++;
  if (*s) { *s = 0; s++; }
  *p = s;
  return start;
}

// =============================================================================
// REPLY HELPERS
//
// Every command emits zero or more DATA lines then EXACTLY ONE terminator,
// "OK ..." or "ERR ...".  The host read loop is therefore "accumulate lines
// until OK or ERR", with no special cases.
// =============================================================================

// okPre() does NOT feed the serial watchdog.  It used to, and that quietly
// contradicted the invariant documented here: the raw '?' byte calls doSta() ->
// okPre() without ever passing through line assembly or the parser, so a stuck
// UART line or a wedged host spraying '?' fed the watchdog forever while the arm
// stayed driven.  The single feed now lives in handleLine(), after dispatch().
static void okPre() {
  Serial.print(F("OK "));
  Serial.print(g_verb);
}

static void okDone() {
  okPre();
  Serial.println();
}

static void errPre(const __FlashStringHelper* code) {
  Serial.print(F("ERR "));
  Serial.print(code);
  Serial.print(' ');
}

// "ERR <code> <VERB>" with nothing after it.  Only failures that genuinely have
// no joint and no operand carry no detail; SERIAL-PROTOCOL.md section 7 promises
// "ERR <Ecode> <VERB> [key=value ...]" and its acceptance test quotes the keys.
static void errPlain(const __FlashStringHelper* code) {
  errPre(code);
  Serial.println(g_verb);
}

// "ERR <code> <VERB> JOINT=<i>" with the line left OPEN, so the caller can append
// its own key=value detail before println().  errJ() is the closed form.
static void errJPre(const __FlashStringHelper* code, uint8_t i) {
  errPre(code);
  Serial.print(g_verb);
  Serial.print(F(" JOINT="));
  Serial.print(i);
}

static void errJ(const __FlashStringHelper* code, uint8_t i) {
  errJPre(code, i);
  Serial.println();
}

static void errJoint(int32_t id) {
  errPre(F("E4"));
  Serial.print(g_verb);
  Serial.print(F(" JOINT="));
  Serial.print(id);
  if (id == (int32_t)RESERVED_ID) Serial.print(F(" RESERVED=shoulder_pair"));
  Serial.println();
}

// =============================================================================
// MOTION PRIMITIVES
//
// These four functions are the only code allowed to touch a servo.  All pin
// access is behind the NUM_DIGITAL_PINS guard.
// =============================================================================

// THE clamp the header promises.  Every angle that reaches a servo goes through
// this, so the per-joint envelope is a STRUCTURAL property of the write path and
// not a lucky property of four careful callers.
static int16_t clampToLimits(uint8_t i, int16_t c) {
  int16_t lo = (int16_t)j[i].minD * 100;
  int16_t hi = (int16_t)j[i].maxD * 100;
  if (c < lo) return lo;
  if (c > hi) return hi;
  return c;
}

// THE ONE piece of shoulder mirror arithmetic.  Both write paths (writeJoint and
// the enableJoint pre-load) call this; there is no second copy to drift.
//
// SAME: the horns were fitted the same way round, so D5 takes D4's pulse.
// INV : they oppose, so D5 is D4 mirrored about the axis (90.00 + offset).
//       rightC = (18000 + 2*mirrorOffsetC) - leftC.  With offset 0 that is the
//       old 18000 - leftC exactly, so nothing changes for a symmetric build.
//
// The clamp to [0, 18000] is not decoration.  doMir() proves the offset keeps
// joint 1's whole MIN..MAX envelope legal before storing it, and neither LIM nor
// MIR is accepted while joint 1 is enabled - but D5 is the servo nobody can see
// on STA, so the last line of defence lives at the point of write, not upstream.
//
// int32 intermediate: 18000 + 2*mirrorOffsetC overflows int16 at offsets this
// firmware is otherwise willing to parse.
static int16_t mirrorC(int16_t leftC) {
  if (mirMode != MIR_INV) return leftC;
  int32_t r = (int32_t)18000 + 2L * (int32_t)mirrorOffsetC - (int32_t)leftC;
  if (r < 0)     r = 0;
  if (r > 18000) r = 18000;
  return (int16_t)r;
}

// Rate-limit the LOGICAL joint, then mirror at the single point of write.
// Never run two independent interpolators on the pair: independent limiting
// lets them transiently desynchronise, which is the fighting case.
//
// Always writeMicroseconds, never write(degrees): the library's integer degree
// map quantises to ~10.3 us per degree, so a slow move sits still and then
// jumps a whole degree - visible and audible stair-stepping.
static void writeJoint(uint8_t i) {
  if (PIN_A[i] >= NUM_DIGITAL_PINS) return;

  j[i].setC = clampToLimits(i, j[i].setC);

  if (i == 1) {
    // Convert BOTH pulse widths before entering the critical section - degCToUs
    // is a 32-bit multiply and divide, and the ISR must not wait on it.
    uint16_t uA = degCToUs(j[1].setC);
    uint16_t uB = degCToUs(mirrorC(j[1].setC));
    // Update the pair atomically.  Between two bare writeMicroseconds calls the
    // Timer1 ISR can fire and pulse one shoulder servo at the NEW angle while the
    // other still holds the PREVIOUS one, for a whole 20 ms frame.  Two MG996Rs
    // disagreeing through one link is the fighting case this sketch exists to
    // prevent - same reasoning as the paired attach/detach below.
    noInterrupts();
    sA[1].writeMicroseconds((int)uA);
    sB.writeMicroseconds((int)uB);
    interrupts();
  } else {
    sA[i].writeMicroseconds((int)degCToUs(j[i].setC));
  }
}

// THE ONLY WAY TO STOP DRIVING A JOINT.  Idempotent - safe on a joint that was
// never attached.  detach() then drive the pin LOW; see landmine 1.
//
// LOW, never pinMode(INPUT): high-Z leaves the signal line floating and able to
// pick up noise.
static void disableJoint(uint8_t i) {
  if (PIN_A[i] >= NUM_DIGITAL_PINS) return;

  if (i == 1 && PIN_B[1] < NUM_DIGITAL_PINS) {
    // Detach both shoulder servos with interrupts held off, so the ISR cannot
    // pulse one of them alone in the gap.  There must be no reachable state
    // where one shoulder servo is driven and the other is not.
    noInterrupts();
    sA[1].detach();
    sB.detach();
    interrupts();
    pinMode(PIN_A[1], OUTPUT);  digitalWrite(PIN_A[1], LOW);
    pinMode(PIN_B[1], OUTPUT);  digitalWrite(PIN_B[1], LOW);
  } else {
    sA[i].detach();
    pinMode(PIN_A[i], OUTPUT);  digitalWrite(PIN_A[i], LOW);
  }

  j[i].en   = false;
  j[i].tgtC = j[i].setC;   // no stale target survives to the next enable
}

// Adopt-before-drive.  adoptDeg is the operator's by-eye estimate of where the
// joint IS right now.  Pre-load that pulse width BEFORE attach() so the very
// first pulse the servo ever sees equals its current position and nothing
// jumps.  See landmine 2.
static void enableJoint(uint8_t i, int16_t adoptDeg) {
  if (PIN_A[i] >= NUM_DIGITAL_PINS) return;

  // Same clamp as writeJoint, for the same reason: this function ALSO writes a
  // pulse width, so the header's "every write goes through one clamp" has to be
  // true here too.  doEna already rejects an out-of-range adopt angle, so this
  // is structural rather than corrective - which is exactly the point.
  int16_t c = clampToLimits(i, (int16_t)(adoptDeg * 100));
  j[i].setC = c;
  j[i].tgtC = c;

  if (i == 1 && PIN_B[1] < NUM_DIGITAL_PINS) {
    int16_t r = mirrorC(c);
    // BOTH pre-loads before EITHER attach.  One shoulder servo driving against
    // a dead-weight geartrain is the worst reachable configuration on this arm.
    sA[1].writeMicroseconds((int)degCToUs(c));
    sB.writeMicroseconds((int)degCToUs(r));
    // Attach the pair atomically: between two bare attach() calls the Timer1
    // ISR can fire and pulse D4 alone for up to one 20 ms frame.
    noInterrupts();
    sA[1].attach(PIN_A[1]);
    sB.attach(PIN_B[1]);
    interrupts();
  } else {
    sA[i].writeMicroseconds((int)degCToUs(c));
    sA[i].attach(PIN_A[i]);
  }

  j[i].en = true;
}

// ONE e-stop function, called from exactly two places: the EST / '!' handler
// and the watchdog trip.  That guarantees no path can latch while leaving en
// true anywhere - which is what makes CLR safe with no precondition.
//
// Be honest about what detaching means: a de-energised gravity-loaded arm SAGS.
// Recovery always routes back through "ENA j <adopt_deg>" because the staleness
// is MECHANICAL, not a lost setpoint.
static void estopAll(const __FlashStringHelper* src, bool byWatchdog) {
  for (uint8_t i = 0; i < NJ; i++) disableJoint(i);
  estopLatched = true;
  wdgTripped   = byWatchdog;
  Serial.print(F("EVT ESTOP SRC="));
  Serial.println(src);
}

// =============================================================================
// COMMAND HANDLERS
// =============================================================================

static void doVer() {
  okPre();
  Serial.println(F(" NAME=FACTORYLM-ARM PROTO=1.0 FW=1.0.0 JOINTS=6 BUILD=20260801"));
}

static void doPng() {
  okPre();
  Serial.print(F(" UP="));
  Serial.println(millis());
}

static void doSta() {
  uint8_t uncal = 0;
  for (uint8_t i = 0; i < NJ; i++) {
    if (i == RESERVED_ID) continue;
    if (!j[i].cal) uncal++;
    Serial.print(F("STA J"));      Serial.print(i);
    Serial.print(F(" EN="));       Serial.print(j[i].en ? 1 : 0);
    Serial.print(F(" SET="));      Serial.print(degOf(j[i].setC));
    Serial.print(F(" TGT="));      Serial.print(degOf(j[i].tgtC));
    Serial.print(F(" MIN="));      Serial.print(j[i].minD);
    Serial.print(F(" MAX="));      Serial.print(j[i].maxD);
    Serial.print(F(" CAL="));      Serial.print(j[i].cal ? 1 : 0);
    Serial.print(F(" DPS="));      Serial.print(j[i].dps);
    Serial.print(F(" MOV="));      Serial.println((j[i].en && j[i].setC != j[i].tgtC) ? 1 : 0);
  }
  Serial.print(F("SYS ES="));   Serial.print(estopLatched ? 1 : 0);
  Serial.print(F(" WD="));      Serial.print(wdgTripped ? 1 : 0);
  Serial.print(F(" WDMS="));    Serial.print(wdgMs);
  Serial.print(F(" MIR="));     Serial.print(mirName());
  Serial.print(F(" UP="));      Serial.print(millis());
  Serial.print(F(" UNCAL="));   Serial.println(uncal);
  okPre();
  Serial.println(F(" N=6"));
}

static void doLimList() {
  for (uint8_t i = 0; i < NJ; i++) {
    if (i == RESERVED_ID) continue;
    Serial.print(F("LIM J"));  Serial.print(i);
    Serial.print(F(" MIN="));  Serial.print(j[i].minD);
    Serial.print(F(" MAX="));  Serial.print(j[i].maxD);
    Serial.print(F(" CAL="));  Serial.println(j[i].cal ? 1 : 0);
  }
  okPre();
  Serial.println(F(" N=6"));
}

// LIM j min max cal.  Rejected unless the joint is DISABLED - you cannot move
// the goalposts under a live joint.  CAL is set explicitly from the argument,
// never inferred: a file that still holds the defaults must stay flagged
// uncalibrated.
//
// E10, not E5.  E5 is documented as "adopt angle outside this joint's MIN..MAX";
// a bad min/max pair or a bad cal flag is a different failure with a different
// remedy, and sharing the code made the GUI's plain-English message wrong.
static void doLimSet(uint8_t i, int32_t mn, int32_t mx, int32_t cal) {
  if (j[i].en) { errJPre(F("E9"), i); Serial.println(F(" STATE=enabled")); return; }

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

  j[i].minD = (uint8_t)mn;
  j[i].maxD = (uint8_t)mx;
  j[i].cal  = (cal == 1);

  // Keep the reported commanded angle inside the new envelope.  The joint is
  // disabled, so this only tidies what STA displays; ENA overwrites setC with
  // the operator's adopt angle anyway.
  int16_t lo = (int16_t)mn * 100, hi = (int16_t)mx * 100;
  if (j[i].setC < lo) j[i].setC = lo;
  if (j[i].setC > hi) j[i].setC = hi;
  j[i].tgtC = j[i].setC;

  okPre();
  Serial.print(F(" J"));      Serial.print(i);
  Serial.print(F(" MIN="));   Serial.print(j[i].minD);
  Serial.print(F(" MAX="));   Serial.print(j[i].maxD);
  Serial.print(F(" CAL="));   Serial.println(j[i].cal ? 1 : 0);
}

// The mirror relation is DATA, not a constant.  Both "same pulse" and "mirror
// about centre" are plausible and depend on how the horns were physically
// fitted, which is unrecorded.  Getting it backwards makes two MG996Rs fight
// through one link the instant both are enabled.  So there is no default:
// determine it at the bench with the single-servo sketch, which already
// documents centring D4 and D5 separately.
//
// MIR <SAME|INV|UNKNOWN> [offset_deg]
//
// The OFFSET is the same argument as the mode: data, not a constant.  "INV means
// mirror about exactly 90.00" is an assumption about how the two horns were
// splined, and this project refused to make assumptions of exactly that class
// about travel limits.  offset_deg is optional and defaults to 0, which is the
// old hardcoded behaviour, so every existing MIR line still means what it meant.
//
// EVERY MIR command sets the offset (0 when omitted), so an offset can never
// outlive the mode it was given with, and the envelope check below therefore
// only has to run for the mode being set right now.
//
// E11, not E5.  A bad mode word or a bad offset is not "adopt angle out of
// range", which is what the GUI tells the operator when it sees E5.
static void doMir(char* modeTok, int32_t offDeg) {
  if (j[1].en) { errJPre(F("E9"), 1); Serial.println(F(" STATE=enabled")); return; }

  upcase(modeTok);
  uint8_t m;
  if      (strcmp_P(modeTok, PSTR("SAME"))    == 0) m = MIR_SAME;
  else if (strcmp_P(modeTok, PSTR("INV"))     == 0) m = MIR_INV;
  else if (strcmp_P(modeTok, PSTR("UNKNOWN")) == 0) m = MIR_UNKNOWN;
  else {
    errPre(F("E11"));
    Serial.print(g_verb);
    Serial.print(F(" MODE="));
    Serial.println(modeTok);
    return;
  }

  // Unconditional: it bounds the int16 store and the 18000 + 2*offset maths.
  if (offDeg < -(int32_t)MIR_OFF_MAX_DEG || offDeg > (int32_t)MIR_OFF_MAX_DEG) {
    errPre(F("E11"));
    Serial.print(g_verb);
    Serial.print(F(" OFF="));    Serial.print(offDeg);
    Serial.print(F(" LIMIT=+/-")); Serial.println(MIR_OFF_MAX_DEG);
    return;
  }

  int32_t offC = offDeg * 100L;

  // Only INV uses the offset, so only INV is checked against the envelope.  The
  // whole of joint 1's legal travel must MIRROR to a legal servo angle: if it
  // does not, some reachable D4 angle would silently clamp D5 and the pair would
  // fight.  Refuse the offset instead of discovering that with the arm loaded.
  if (m == MIR_INV) {
    int32_t axis  = 18000L + 2L * offC;
    int32_t imgLo = axis - (int32_t)j[1].maxD * 100L;   // image of MAX
    int32_t imgHi = axis - (int32_t)j[1].minD * 100L;   // image of MIN
    if (imgLo < 0 || imgHi > 18000L) {
      errPre(F("E11"));
      Serial.print(g_verb);
      Serial.print(F(" OFF="));  Serial.print(offDeg);
      Serial.print(F(" MIN="));  Serial.print(j[1].minD);
      Serial.print(F(" MAX="));  Serial.print(j[1].maxD);
      Serial.println(F(" MIRROR=out_of_travel"));
      return;
    }
  }

  mirMode       = m;
  mirrorOffsetC = (int16_t)offC;

  okPre();
  Serial.print(F(" MODE="));  Serial.print(mirName());
  Serial.print(F(" OFF="));   Serial.println(offDeg);
}

static void doEna(uint8_t i, int32_t adopt) {
  if (estopLatched) { errJ(F("E7"), i); return; }
  if (adopt < (int32_t)j[i].minD || adopt > (int32_t)j[i].maxD) {
    errJPre(F("E5"), i);
    Serial.print(F(" REQ="));  Serial.print(adopt);
    Serial.print(F(" MIN="));  Serial.print(j[i].minD);
    Serial.print(F(" MAX="));  Serial.println(j[i].maxD);
    return;
  }
  if (j[i].en) { errJ(F("E6"), i); return; }
  if (i == 1 && mirMode == MIR_UNKNOWN) {
    errPre(F("E13"));
    Serial.print(g_verb);
    Serial.println(F(" JOINT=1 MIR=UNKNOWN"));
    return;
  }
  enableJoint(i, (int16_t)adopt);
  okPre();
  Serial.print(F(" J"));       Serial.print(i);
  Serial.print(F(" ADOPT="));  Serial.println(adopt);
}

static void doDis(uint8_t i) {
  disableJoint(i);
  okPre();
  Serial.print(F(" J"));
  Serial.println(i);
}

static void doDisAll() {
  for (uint8_t i = 0; i < NJ; i++) disableJoint(i);
  okPre();
  Serial.println(F(" ALL"));
}

// Non-blocking: sets the target only; the interpolator walks setC toward it.
//
// Out-of-range CLAMPS AND REPORTS rather than rejecting, matching the bench
// sketch's "[limit] ... Clamped." behaviour so the operator's mental model
// carries over.  That is only acceptable because CL=1 is never silent and both
// REQ and SET are on the wire.
static void doMov(uint8_t i, int32_t deg) {
  if (estopLatched) { errJ(F("E7"), i); return; }
  if (!j[i].en)     { errJ(F("E6"), i); return; }

  int32_t applied = deg;
  uint8_t cl = 0;
  if (applied < (int32_t)j[i].minD) { applied = (int32_t)j[i].minD; cl = 1; }
  if (applied > (int32_t)j[i].maxD) { applied = (int32_t)j[i].maxD; cl = 1; }
  j[i].tgtC = (int16_t)(applied * 100);

  okPre();
  Serial.print(F(" J"));     Serial.print(i);
  Serial.print(F(" REQ="));  Serial.print(deg);
  Serial.print(F(" SET="));  Serial.print(applied);
  Serial.print(F(" CL="));   Serial.println(cl);
}

// Rate limiting is a POWER feature, not a comfort feature.  A bare step command
// makes the servo drive at full speed, so an MG996R draws up to ~2.5 A.  Seven
// simultaneous steps is the most likely cause of a brownout even on the
// incoming 3-5 A supply.  Raise dps only after that supply is in place and the
// current draw has actually been measured.
//
// E12, not E5 - a slew rate outside 1..90 has nothing to do with an adopt angle
// outside a joint's travel, which is what E5 is documented to mean.
static void doSpd(uint8_t i, int32_t dps) {
  if (dps < (int32_t)SPD_MIN_DPS || dps > (int32_t)SPD_MAX_DPS) {
    errJPre(F("E12"), i);
    Serial.print(F(" REQ="));  Serial.print(dps);
    Serial.print(F(" MIN="));  Serial.print(SPD_MIN_DPS);
    Serial.print(F(" MAX="));  Serial.println(SPD_MAX_DPS);
    return;
  }
  j[i].dps = (uint8_t)dps;
  okPre();
  Serial.print(F(" J"));     Serial.print(i);
  Serial.print(F(" DPS="));  Serial.println(j[i].dps);
}

// Freeze every target at its current commanded angle.  Joints stay ATTACHED and
// holding.  NOT latching - a following MOV works immediately.
static void doStp() {
  for (uint8_t i = 0; i < NJ; i++) if (j[i].en) j[i].tgtC = j[i].setC;
  okDone();
}

static void doEst(const __FlashStringHelper* src) {
  estopAll(src, false);
  okDone();
}

static void doClr() {
  // No precondition: estopAll() has already cleared every enabled flag, and ENA
  // requires an explicit adopt angle, so there is no path back to a driven
  // joint without a deliberate operator action.
  estopLatched = false;
  wdgTripped   = false;
  okDone();
}

static void doWdg(int32_t ms) {
  if (ms != 0 && (ms < (int32_t)WDG_MIN_MS || ms > (int32_t)WDG_MAX_MS)) {
    errPlain(F("E5"));
    return;
  }
  wdgMs = (uint16_t)ms;
  okPre();
  Serial.print(F(" MS="));
  Serial.println(wdgMs);
}

// Kept deliberately short.  At 115200 every line of help is ~0.5 ms of BLOCKING
// transmit, during which a queued '!' waits and the interpolator does not tick.
static void doHlp() {
  Serial.println(F("; FACTORYLM-ARM protocol 1.0  115200 8N1  line ends CR or LF"));
  Serial.println(F("; joints 0=base D3  1=shoulder D4+D5 pair  3=elbow D6"));
  Serial.println(F(";        4=wristpitch D9  5=wristroll D10  6=gripper D11"));
  Serial.println(F(";        2 is RESERVED - the pair's 2nd servo, never addressable"));
  Serial.println(F("; VER / PNG / STA (or ?)   identify / ping / full status"));
  Serial.println(F("; LIM                      list limits"));
  Serial.println(F("; LIM j min max cal        set limits; joint must be DISABLED"));
  Serial.println(F("; MIR mode [off_deg]       SAME|INV|UNKNOWN; j1 DISABLED"));
  Serial.println(F(";                          INV mirrors about 90+off_deg, off=0 default"));
  Serial.println(F("; ENA j adopt_deg          attach; 1st pulse = adopt, nothing jumps"));
  Serial.println(F("; DIS j   |   DIS A        detach one / detach all"));
  Serial.println(F("; MOV j deg                set target; clamped, reply shows CL=1"));
  Serial.println(F("; SPD j dps                1..90 deg/s"));
  Serial.println(F("; STP                      freeze targets, stay attached"));
  Serial.println(F("; EST (or !)               E-STOP: detach all + latch.  CLR clears"));
  Serial.println(F("; WDG ms                   0=off else 200..10000; host silence=stop"));
  Serial.println(F("; SYS WD=1 means the latch came from the watchdog, not the operator"));
  Serial.println(F("; angles are COMMANDED degrees - these servos have no feedback"));
  okDone();
}

// =============================================================================
// LINE PARSING AND DISPATCH
// =============================================================================

// Resolve a joint argument.  Emits the error itself and returns false on
// failure, so callers stay flat.
static bool jointArg(int32_t id, uint8_t* out) {
  if (id < 0 || id >= (int32_t)NJ || id == (int32_t)RESERVED_ID) {
    errJoint(id);
    return false;
  }
  *out = (uint8_t)id;
  return true;
}

static bool intArg(uint8_t idx, int32_t* out) {
  if (parseInt(tokv[idx], out)) return true;
  errPre(F("E3"));
  Serial.print(g_verb);
  Serial.print(F(" ARG="));
  Serial.println(idx + 1);
  return false;
}

static void badArgc() {
  errPre(F("E2"));
  Serial.print(g_verb);
  Serial.print(F(" N="));
  Serial.println(tokc);
}

#define VIS(a, b, c) (g_verb[0] == (a) && g_verb[1] == (b) && g_verb[2] == (c))

static void dispatch() {
  int32_t a0, a1, a2, a3;
  uint8_t id;

  if (VIS('V','E','R')) { if (tokc != 0) { badArgc(); return; } doVer(); return; }
  if (VIS('P','N','G')) { if (tokc != 0) { badArgc(); return; } doPng(); return; }
  if (VIS('S','T','A')) { if (tokc != 0) { badArgc(); return; } doSta(); return; }
  if (VIS('S','T','P')) { if (tokc != 0) { badArgc(); return; } doStp(); return; }
  if (VIS('H','L','P')) { if (tokc != 0) { badArgc(); return; } doHlp(); return; }
  if (VIS('C','L','R')) { if (tokc != 0) { badArgc(); return; } doClr(); return; }
  if (VIS('E','S','T')) { if (tokc != 0) { badArgc(); return; } doEst(F("CMD")); return; }

  if (VIS('L','I','M')) {
    if (tokc == 0) { doLimList(); return; }
    if (tokc != 4) { badArgc(); return; }
    if (!intArg(0, &a0) || !intArg(1, &a1) || !intArg(2, &a2) || !intArg(3, &a3)) return;
    if (!jointArg(a0, &id)) return;
    doLimSet(id, a1, a2, a3);
    return;
  }

  // MIR <mode> [offset_deg].  The offset is OPTIONAL and defaults to 0, so the
  // one-argument form every existing host sends keeps working unchanged.
  if (VIS('M','I','R')) {
    if (tokc != 1 && tokc != 2) { badArgc(); return; }
    a0 = 0;
    if (tokc == 2 && !intArg(1, &a0)) return;
    doMir(tokv[0], a0);
    return;
  }

  if (VIS('E','N','A')) {
    if (tokc != 2) { badArgc(); return; }
    if (!intArg(0, &a0) || !intArg(1, &a1)) return;
    if (!jointArg(a0, &id)) return;
    doEna(id, a1);
    return;
  }

  if (VIS('D','I','S')) {
    if (tokc != 1) { badArgc(); return; }
    upcase(tokv[0]);
    if (tokv[0][0] == 'A' && tokv[0][1] == 0) { doDisAll(); return; }
    if (!intArg(0, &a0)) return;
    if (!jointArg(a0, &id)) return;
    doDis(id);
    return;
  }

  if (VIS('M','O','V')) {
    if (tokc != 2) { badArgc(); return; }
    if (!intArg(0, &a0) || !intArg(1, &a1)) return;
    if (!jointArg(a0, &id)) return;
    doMov(id, a1);
    return;
  }

  if (VIS('S','P','D')) {
    if (tokc != 2) { badArgc(); return; }
    if (!intArg(0, &a0) || !intArg(1, &a1)) return;
    if (!jointArg(a0, &id)) return;
    doSpd(id, a1);
    return;
  }

  if (VIS('W','D','G')) {
    if (tokc != 1) { badArgc(); return; }
    if (!intArg(0, &a0)) return;
    doWdg(a0);
    return;
  }

  // Unknown verb.  Garbage input NAKs.  It must NEVER trigger an e-stop - an
  // e-stop that fires on a typo trains the operator to ignore e-stops.
  errPre(F("E1"));
  Serial.print(F("VERB TOKEN="));
  Serial.println(g_verb);
}

static void handleLine() {
  char* p = line;
  char* v = nextTok(&p);
  if (v == NULL) return;   // blank line - silent, so a CRLF cannot NAK

  upcase(v);

  // Verbs are exactly three letters.  Copy into g_verb so every reply echoes it
  // uppercase, and so the raw '!' / '?' handlers can set it too.
  uint8_t vl = 0;
  while (v[vl] && vl < 4) vl++;
  g_verb[0] = v[0] ? v[0] : '?';
  g_verb[1] = (vl > 1) ? v[1] : 0;
  g_verb[2] = (vl > 2) ? v[2] : 0;
  g_verb[3] = 0;

  if (vl != 3 || v[0] < 'A' || v[0] > 'Z' || v[1] < 'A' || v[1] > 'Z'
              || v[2] < 'A' || v[2] > 'Z') {
    errPre(F("E1"));
    Serial.print(F("VERB TOKEN="));
    Serial.println(v);
    return;
  }

  tokc = 0;
  while (tokc < 4) {
    char* t = nextTok(&p);
    if (t == NULL) break;
    tokv[tokc++] = t;
  }
  if (nextTok(&p) != NULL) { badArgc(); return; }   // more than 4 arguments

  dispatch();

  // THE ONLY PLACE THE SERIAL WATCHDOG IS FED (setup() aside, which just seeds
  // it).  Reaching here means a complete line arrived, assembled without
  // overflowing, held a legal three-letter verb and an acceptable argument count,
  // and was handed to a real handler.  Nothing weaker counts as "the host is
  // alive": raw bytes do not, an over-long line does not, and in particular the
  // realtime '!' and '?' bytes do not - they bypass line assembly entirely, so a
  // stuck UART line or a host wedged in a '?' loop can no longer hold the
  // watchdog open while the arm stays driven.
  //
  // HOSTS: heartbeat with a TERMINATED LINE COMMAND (PNG\n is the cheapest).
  // A bare '?' still returns status, but it will NOT feed the watchdog.
  lastRxMs = millis();
}

static void setVerb(char a, char b, char c) {
  g_verb[0] = a; g_verb[1] = b; g_verb[2] = c; g_verb[3] = 0;
}

// =============================================================================
// SETUP / LOOP
// =============================================================================

void setup() {
  Serial.begin(115200);

  // Drive every signal pin LOW before anything else.  This emits no pulses, so
  // nothing moves, and it is strictly better than leaving them floating.
  for (uint8_t i = 0; i < NJ; i++) {
    if (PIN_A[i] < NUM_DIGITAL_PINS) { pinMode(PIN_A[i], OUTPUT); digitalWrite(PIN_A[i], LOW); }
    if (PIN_B[i] < NUM_DIGITAL_PINS) { pinMode(PIN_B[i], OUTPUT); digitalWrite(PIN_B[i], LOW); }
  }

  for (uint8_t i = 0; i < NJ; i++) {
    j[i].setC = DEF_SET_C;
    j[i].tgtC = DEF_SET_C;
    j[i].minD = DEF_MIN_DEG;
    j[i].maxD = DEF_MAX_DEG;
    j[i].dps  = DEF_DPS;
    j[i].en   = false;
    j[i].cal  = false;
  }

  lastRxMs   = millis();
  lastTickMs = lastRxMs;

  Serial.println();
  Serial.println(F("; ==========================================================="));
  Serial.println(F(";  FACTORYLM ARM CONTROLLER - Emre Kalem 6-axis arm, Uno"));
  Serial.println(F("; ==========================================================="));
  Serial.println(F(";  NOTHING IS ATTACHED. No servo is driven until you send ENA."));
  Serial.println(F(";  Limits default to 70-110 deg and are flagged UNCALIBRATED."));
  Serial.println(F(";  Joint 1 (shoulder D4+D5) is LOCKED until MIR is set."));
  Serial.println(F(";  Joint id 2 is RESERVED - the pair's 2nd servo, not addressable."));
  Serial.println(F(";"));
  Serial.println(F(";  POWER: USB powers the Uno. A separate supply powers the servos."));
  Serial.println(F(";         ONLY THE GROUNDS ARE JOINED. Never feed external +5 V"));
  Serial.println(F(";         into Uno 5V, VIN or the barrel jack."));
  Serial.println(F(";"));
  Serial.println(F(";  THE ROCKER SWITCH AND THE INLINE FUSE ARE THE REAL E-STOP."));
  Serial.println(F(";  This firmware is NOT a safety device. Type HLP for commands."));
  Serial.println(F("; ==========================================================="));
  Serial.println(F("RDY NAME=FACTORYLM-ARM PROTO=1.0 FW=1.0.0"));
}

void loop() {
  // ---- serial input -------------------------------------------------------
  // '!' and '?' are intercepted BEFORE line assembly, so an e-stop never queues
  // behind a half-typed line.  Latency is bounded by one loop() iteration -
  // which only holds because there is no delay() anywhere in this sketch.
  uint8_t budget = RX_BUDGET;
  while (Serial.available() > 0 && budget > 0) {
    budget--;
    char ch = (char)Serial.read();

    if (ch == '!') { setVerb('E','S','T'); doEst(F("RT")); continue; }
    if (ch == '?') { setVerb('S','T','A'); doSta();        continue; }

    // Both CR and LF terminate.  The Serial Monitor's line-ending setting is
    // therefore irrelevant, exactly as in the single-servo bench sketch.  An
    // empty buffer is dropped silently, so CRLF fires once, not twice.
    if (ch == '\r' || ch == '\n') {
      if (lineOver) {
        // The verb slot carries the mnemonic here: there is no trustworthy verb
        // in a line we refused to finish reading.
        Serial.println(F("ERR E8 LINE"));
      } else if (lineLen > 0) {
        line[lineLen] = 0;
        handleLine();
      }
      lineLen  = 0;
      lineOver = false;
      continue;
    }

    if (lineLen < (LINE_MAX - 1)) {
      line[lineLen++] = ch;
    } else {
      lineOver = true;   // discard through the next terminator; NEVER act on a
                         // truncated line
    }
  }

  uint32_t now = millis();

  // ---- interpolator -------------------------------------------------------
  // Elapsed-time integration, not a fixed step per tick.  A stall caused by a
  // blocking Serial.print produces a proportionally larger next step, which
  // still respects deg/s exactly.  That is what makes the ~36 ms transmit burst
  // of a STA reply harmless.
  if ((uint32_t)(now - lastTickMs) >= TICK_MS) {
    uint32_t el = (uint32_t)(now - lastTickMs);
    if (el > TICK_CAP_MS) el = TICK_CAP_MS;
    lastTickMs = now;

    for (uint8_t i = 0; i < NJ; i++) {
      if (!j[i].en) continue;
      int32_t maxStep = ((int32_t)j[i].dps * (int32_t)el) / 10;   // centidegrees
      if (maxStep < 1) maxStep = 1;
      // TICK_CAP_MS bounds the elapsed SLICE; this bounds the resulting STEP,
      // which is the thing that actually moves the arm.  Without it a 200 ms
      // stall at 90 deg/s still commands an 18 degree jump in one write.
      if (maxStep > (int32_t)MAX_STEP_C) maxStep = MAX_STEP_C;
      int32_t d = (int32_t)j[i].tgtC - (int32_t)j[i].setC;
      if (d >  maxStep) d =  maxStep;
      if (d < -maxStep) d = -maxStep;
      if (d != 0) {
        j[i].setC += (int16_t)d;
        writeJoint(i);
      }
    }
  }

  // ---- serial watchdog ----------------------------------------------------
  // Armed only when a timeout is set AND at least one joint is enabled, so a
  // hand operator at the Serial Monitor is never tripped by a watchdog they did
  // not ask for.  Comparison is rollover-safe.
  if (wdgMs != 0 && !estopLatched && anyEnabled()) {
    uint32_t el = (uint32_t)(now - lastRxMs);
    if (el > (uint32_t)wdgMs) {
      Serial.print(F("EVT WDOG MS="));
      Serial.println(el);
      estopAll(F("WDG"), true);
    }
  }
}
