"""A protocol-faithful simulator of factorylm_arm_controller.ino.

WHY THIS EXISTS
    COM5 does not exist tonight and the bench is disconnected. Every host-side
    thing in this project -- the console, the bridge, hold_arm.py, any future
    playback -- is a serial client with nothing to talk to. This module is the
    other end of the wire: it parses the same bytes, keeps the same state, and
    emits the same lines, so host code can be exercised and the protocol can be
    regression-tested with no board, no servos and no power.

    It is NOT a model of the arm. It models the FIRMWARE. Read the "WHAT THIS
    DOES NOT MODEL" section of README.md before you believe anything it tells
    you about a joint.

THE SOURCE OF TRUTH IS THE .ino, NOT THE PROTOCOL DOC
    Documentation/SERIAL-PROTOCOL.md is the specification and it is very good,
    but where it and factorylm_arm_controller.ino disagree, THIS FILE FOLLOWS
    THE .ino -- because that is what a real board would do. Three such
    disagreements were found while writing this and every one of them is
    recorded in README.md. They are worth more than the simulator.

    Function names below are deliberately the .ino's names (doSta, doLimSet,
    handleLine, writeJoint, disableJoint...) so the two files can be read side
    by side. If you change one, diff the other.

TIME IS INJECTED AND NEVER REAL
    There is no thread, no sleep and no wall clock anywhere in here. A test
    moves time by calling advance(). That is what makes the interpolator, the
    jog command-age timeout and the serial watchdog testable at all: all three
    are millis() state machines, and a test that waited on wall time for a
    1000 ms watchdog would be a test nobody runs.

NO POSITION FEEDBACK, HERE EITHER
    Every angle this simulator reports is a COMMANDED angle, exactly as on the
    real board. It cannot tell you where a shaft is because the firmware cannot
    either. The words "position", "actual", "measured" and "feedback" do not
    appear on the wire and must not appear in anything built on this.
"""

from __future__ import annotations

# =============================================================================
# CONFIGURATION -- every constant here is lifted from the .ino, same names
# =============================================================================

NJ = 7  # joint slots, indexed by JOINT ID 0..6
RESERVED_ID = 2  # the shoulder pair's second servo - not addressable

#: Pin table indexed by joint id. Slot 2 is allocated and pinless. The .ino
#: guards every pin access on "pin >= NUM_DIGITAL_PINS" rather than on
#: "i == RESERVED_ID", so a loop that forgets to skip slot 2 still cannot drive
#: anything. `None` here is that guard.
PIN_A: tuple[int | None, ...] = (3, 4, None, 6, 9, 10, 11)
PIN_B: tuple[int | None, ...] = (None, 5, None, None, None, None, None)

ALL_PINS = (3, 4, 5, 6, 9, 10, 11)

DEF_MIN_DEG = 70
DEF_MAX_DEG = 110
DEF_SET_C = 9000  # 90.00 degrees, in centidegrees
DEF_DPS = 30

LIM_MIN_SPAN_DEG = 5
SPD_MIN_DPS, SPD_MAX_DPS = 1, 90
WDG_MIN_MS, WDG_MAX_MS = 200, 10000

JOG_BEAT_MS = 250
JOG_TIMEOUT_MS = 1000
TICK_MS = 20
TICK_CAP_MS = 200
MAX_STEP_C = 200

MIR_OFF_MAX_DEG = 90
LINE_MAX = 48

MIR_UNKNOWN, MIR_SAME, MIR_INV = 0, 1, 2

#: LINE TERMINATOR. SERIAL-PROTOCOL.md section 12 states every line ends in
#: `\n` (0x0A) and that is what this emits. Whether the real board emits CRLF
#: instead is UNVERIFIED -- Arduino's Print::println() lives in the core, which
#: is not in this repository, and the bench is disconnected so it cannot be
#: observed. It is one constant if that is ever settled. Every host in this
#: repo strips its lines, which is why nobody would have noticed either way.
EOL = "\n"

U32 = 0xFFFFFFFF


# =============================================================================
# SMALL HELPERS -- C semantics, not Python semantics
# =============================================================================

def _idiv(a: int, b: int) -> int:
    """Integer division that TRUNCATES TOWARD ZERO, like C.

    Python's // floors, so -1 // 100 is -1 where C gives 0. Every division in
    the .ino is C division; using // would be a silent one-count difference the
    day a negative ever reaches degOf().
    """
    q = abs(a) // abs(b)
    return q if (a < 0) == (b < 0) else -q


def _u32sub(a: int, b: int) -> int:
    """(uint32_t)(a - b) -- the rollover-safe idiom the .ino uses everywhere.

    millis() wraps at about 49.7 days. The firmware never writes
    "millis() > last + timeout"; it writes "(uint32_t)(millis() - last) >
    timeout", which is correct across the wrap. Modelling the wrap here is what
    makes that testable without waiting seven weeks.
    """
    return (a - b) & U32


def degCToUs(c: int) -> int:
    """Centidegrees -> microseconds. Integer only, exactly as the .ino.

    9000 (90.00 deg) -> 1472 us, which is the figure
    V2-SERVO-POWER-AND-WIRING.md section 6 measured and the 0.37 V duty-cycle
    arithmetic is built on.
    """
    return 544 + _idiv(c * 1856, 18000)


def degOf(c: int) -> int:
    """Centidegrees -> whole degrees for the wire. Centidegrees never leave."""
    return _idiv(c + 50, 100)


def _upcase(s: str) -> str:
    """Uppercase a-z ONLY, like the .ino's upcase().

    str.upper() also maps non-ASCII, which would let a byte the firmware would
    have rejected slip through the verb check looking legal.
    """
    return "".join(chr(ord(ch) - 32) if "a" <= ch <= "z" else ch for ch in s)


def parseInt(s: str) -> int | None:
    """The .ino's strict parser. Returns None where the firmware returns false.

    NOT atoi: atoi returns 0 on garbage, which is indistinguishable from a real
    0 -- the whole reason this exists. Rejects an empty field, a bare "-", any
    non-digit, a leading "+", and any magnitude above 100000.
    """
    if s == "":
        return None
    neg = False
    if s[0] == "-":
        neg = True
        s = s[1:]
        if s == "":
            return None
    v = 0
    for ch in s:
        if ch < "0" or ch > "9":
            return None
        v = v * 10 + (ord(ch) - 48)
        if v > 100000:  # far beyond any legal argument
            return None
    return -v if neg else v


# =============================================================================
# CLOCK
# =============================================================================

class VirtualClock:
    """millis(), under the test's control. Wraps at 2**32 like the AVR's."""

    def __init__(self, start_ms: int = 0) -> None:
        self._ms = start_ms & U32

    def millis(self) -> int:
        return self._ms

    def advance(self, ms: int) -> None:
        if ms < 0:
            raise ValueError("time does not run backwards")
        self._ms = (self._ms + ms) & U32

    def set(self, ms: int) -> None:
        """Jump the clock. Use BEFORE seeding a board, not during a session --
        a mid-session jump looks to every timer exactly like real elapsed time,
        which is correct behaviour and almost never what a test meant."""
        self._ms = ms & U32


# =============================================================================
# STATE
# =============================================================================

class Joint:
    __slots__ = ("setC", "tgtC", "minD", "maxD", "dps", "en", "cal",
                 "jogActive", "jogTimedOut", "jogMs")

    def __init__(self) -> None:
        self.setC = DEF_SET_C
        self.tgtC = DEF_SET_C
        self.minD = DEF_MIN_DEG
        self.maxD = DEF_MAX_DEG
        self.dps = DEF_DPS
        self.en = False
        self.cal = False
        # jogActive is the ONLY thing that says a jog is running -- never
        # "jogMs != 0", because millis() is exactly 0 once every ~49.7 days.
        self.jogActive = False
        self.jogTimedOut = False
        self.jogMs = 0


class ArmSim:
    """The simulated controller. Bytes in, bytes out, plus an injected clock."""

    def __init__(self, clock: VirtualClock | None = None) -> None:
        self.clock = clock if clock is not None else VirtualClock()
        self._out = bytearray()
        self._partial = ""  # the line Serial.print() has started but not ended
        self.j = [Joint() for _ in range(NJ)]
        # Per-pin model: True = the Servo channel is attached and pulsing.
        # _pulse_us is the last pulse width written, attached or not (the
        # library stores it either way; the pin only carries it when attached).
        self._attached = {p: False for p in ALL_PINS}
        self._pulse_us = {p: 0 for p in ALL_PINS}
        self.reset()

    # -- boot ---------------------------------------------------------------

    def reset(self) -> None:
        """setup(). THE FIRMWARE RETAINS NOTHING ACROSS THIS.

        Opening the serial port asserts DTR and resets a genuine Uno, so this
        runs on every connect. Limits go back to 70-110 with CAL=0, the mirror
        goes back to UNKNOWN, every joint detaches, the watchdog turns off. A
        host that does not push its state back in is driving a board at
        defaults it did not choose.
        """
        for p in ALL_PINS:
            self._attached[p] = False
            self._pulse_us[p] = 0
        for i in range(NJ):
            self.j[i] = Joint()
        self.mirMode = MIR_UNKNOWN
        self.mirrorOffsetC = 0
        self.estopLatched = False
        self.wdgTripped = False
        self.wdgMs = 0
        self.lastRxMs = self.millis()
        self.lastTickMs = self.lastRxMs
        self.line = ""
        self.lineOver = False
        self.g_verb = ""
        self._partial = ""

        self._pln()
        self._pln("; ===========================================================")
        self._pln(";  FACTORYLM ARM CONTROLLER - Emre Kalem 6-axis arm, Uno")
        self._pln("; ===========================================================")
        self._pln(";  NOTHING IS ATTACHED. No servo is driven until you send ENA.")
        self._pln(";  Limits default to 70-110 deg and are flagged UNCALIBRATED.")
        self._pln(";  Joint 1 (shoulder D4+D5) is LOCKED until MIR is set.")
        self._pln(";  Joint id 2 is RESERVED - the pair's 2nd servo, not addressable.")
        self._pln(";")
        self._pln(";  POWER: USB powers the Uno. A separate supply powers the servos.")
        self._pln(";         ONLY THE GROUNDS ARE JOINED. Never feed external +5 V")
        self._pln(";         into Uno 5V, VIN or the barrel jack.")
        self._pln(";")
        self._pln(";  THE ROCKER SWITCH AND THE INLINE FUSE ARE THE REAL E-STOP.")
        self._pln(";  This firmware is NOT a safety device. Type HLP for commands.")
        self._pln("; ===========================================================")
        self._pln("RDY NAME=FACTORYLM-ARM PROTO=1.0 FW=1.0.0")

    def millis(self) -> int:
        return self.clock.millis()

    # -- the host's side of the wire ---------------------------------------

    def out_waiting(self) -> int:
        return len(self._out)

    def read_out(self, n: int | None = None) -> bytes:
        if n is None or n >= len(self._out):
            data, self._out = bytes(self._out), bytearray()
            return data
        data, self._out = bytes(self._out[:n]), self._out[n:]
        return data

    def unread(self, data: bytes) -> None:
        """Push bytes back to the FRONT of the outbound buffer.

        Only for a reader that took more than it wanted -- FakeSerial.readline()
        has to look for a terminator before it knows how much is one line. The
        firmware itself never calls this; there is no un-transmit on a UART.
        """
        self._out = bytearray(data) + self._out

    def discard_out(self) -> None:
        self._out = bytearray()

    def pin_state(self, pin: int) -> tuple[str, int | None]:
        """('PULSING', microseconds) or ('LOW', None) for one Uno pin.

        THIS IS THE FIRMWARE'S INTENT, NOT A VOLTAGE. It says the sketch
        attached the channel and wrote that pulse width, or that it detached
        and drove the pin LOW. It CANNOT reproduce the Servo-library race that
        section 13 step 15 of the protocol doc exists to catch -- see README.
        """
        if pin not in self._attached:
            raise KeyError(f"pin D{pin} is not a servo pin on this board")
        if self._attached[pin]:
            return ("PULSING", self._pulse_us[pin])
        return ("LOW", None)

    # -- Serial.print / println --------------------------------------------

    def _p(self, text: str = "") -> None:
        self._partial += text

    def _pln(self, text: str = "") -> None:
        self._out += (self._partial + text + EOL).encode("latin-1")
        self._partial = ""

    def _kv(self, key: str, value: object) -> None:
        """One `key=value` field, mid-line.

        The .ino writes these as `Serial.print(F(" MIN=")); Serial.print(...)`
        pairs, column-aligned, one pair per wire field. Keeping one CALL per
        wire field here preserves that line-for-line correspondence -- which is
        the point, because the two files have to be diffable by eye.
        """
        self._p(key)
        self._p(str(value))

    def _kvln(self, key: str, value: object) -> None:
        """The last `key=value` field on a line, then the terminator."""
        self._p(key)
        self._pln(str(value))

    # =====================================================================
    # REPLY HELPERS -- zero or more DATA lines, then EXACTLY ONE terminator
    # =====================================================================

    def _okPre(self) -> None:
        self._p("OK ")
        self._p(self.g_verb)

    def _okDone(self) -> None:
        self._okPre()
        self._pln()

    def _errPre(self, code: str) -> None:
        self._p("ERR ")
        self._p(code)
        self._p(" ")

    def _errPlain(self, code: str) -> None:
        self._errPre(code)
        self._pln(self.g_verb)

    def _errJPre(self, code: str, i: int) -> None:
        """Line left OPEN so the caller can append its own key=value detail."""
        self._errPre(code)
        self._p(self.g_verb)
        self._p(" JOINT=")
        self._p(str(i))

    def _errJ(self, code: str, i: int) -> None:
        self._errJPre(code, i)
        self._pln()

    def _errJoint(self, jid: int) -> None:
        self._errPre("E4")
        self._p(self.g_verb)
        self._p(" JOINT=")
        self._p(str(jid))
        if jid == RESERVED_ID:
            self._p(" RESERVED=shoulder_pair")
        self._pln()

    # =====================================================================
    # MOTION PRIMITIVES -- the only code allowed to touch a servo
    # =====================================================================

    def clampToLimits(self, i: int, c: int) -> int:
        """THE clamp. Every angle that reaches a servo goes through this, so
        the per-joint envelope is a structural property of the write path and
        not a lucky property of four careful callers."""
        lo = self.j[i].minD * 100
        hi = self.j[i].maxD * 100
        if c < lo:
            return lo
        if c > hi:
            return hi
        return c

    def mirrorC(self, leftC: int) -> int:
        """THE ONE piece of shoulder mirror arithmetic. Both write paths call
        it; there is no second copy to drift.

        INV: rightC = (18000 + 2*mirrorOffsetC) - leftC, clamped to 0..18000.
        The clamp is the last line of defence -- D5 is the servo nobody can see
        on STA.
        """
        if self.mirMode != MIR_INV:
            return leftC
        r = 18000 + 2 * self.mirrorOffsetC - leftC
        if r < 0:
            r = 0
        if r > 18000:
            r = 18000
        return r

    def writeJoint(self, i: int) -> None:
        """Rate-limit the LOGICAL joint, then mirror at the single point of
        write. Never two independent interpolators on the pair -- independent
        limiting lets them transiently desynchronise, which is the fighting
        case."""
        if PIN_A[i] is None:
            return
        self.j[i].setC = self.clampToLimits(i, self.j[i].setC)
        if i == 1:
            self._writeMicroseconds(PIN_A[1], degCToUs(self.j[1].setC))
            self._writeMicroseconds(PIN_B[1], degCToUs(self.mirrorC(self.j[1].setC)))
        else:
            self._writeMicroseconds(PIN_A[i], degCToUs(self.j[i].setC))

    def _writeMicroseconds(self, pin: int | None, us: int) -> None:
        if pin is None:
            return
        self._pulse_us[pin] = us

    def disableJoint(self, i: int) -> None:
        """THE ONLY WAY TO STOP DRIVING A JOINT. Idempotent.

        detach() then drive the pin LOW, in one operation, because a detach()
        that is not immediately followed by a LOW write can leave the pin
        driven HIGH forever (Servo library race, upstream Servo#160). There is
        deliberately no other code path in this file that clears _attached.
        """
        if PIN_A[i] is None:
            return
        for pin in (PIN_A[i], PIN_B[i]):
            if pin is not None:
                self._attached[pin] = False  # detach()
                self._pulse_us[pin] = 0      # pinMode(OUTPUT) + digitalWrite(LOW)

        self.j[i].en = False
        self.j[i].tgtC = self.j[i].setC  # no stale target survives to next enable
        # The single choke point for DIS, DIS A, EST, '!' and the watchdog.
        self.j[i].jogActive = False
        self.j[i].jogTimedOut = False

    def enableJoint(self, i: int, adoptDeg: int) -> None:
        """Adopt-before-drive. The pulse is PRE-LOADED BEFORE attach, so the
        very first pulse the servo ever sees equals the adopted angle and
        nothing jumps. A bare attach() commands ~1500 us centre on the next
        frame -- that snap is what this whole design exists to prevent, and it
        would fire on every enable, not just at boot."""
        if PIN_A[i] is None:
            return
        c = self.clampToLimits(i, adoptDeg * 100)
        self.j[i].setC = c
        self.j[i].tgtC = c
        self.j[i].jogActive = False
        self.j[i].jogTimedOut = False

        if i == 1 and PIN_B[1] is not None:
            r = self.mirrorC(c)
            self._writeMicroseconds(PIN_A[1], degCToUs(c))   # BOTH pre-loads
            self._writeMicroseconds(PIN_B[1], degCToUs(r))   # before EITHER attach
            self._attached[PIN_A[1]] = True
            self._attached[PIN_B[1]] = True
        else:
            self._writeMicroseconds(PIN_A[i], degCToUs(c))
            self._attached[PIN_A[i]] = True

        self.j[i].en = True

    def estopAll(self, src: str, byWatchdog: bool) -> None:
        """ONE e-stop function, two callers: EST / '!' and the watchdog trip.
        No path can latch while leaving en true anywhere, which is what makes
        CLR safe with no precondition.

        A de-energised gravity-loaded arm SAGS. Recovery always routes back
        through ENA <j> <fresh adopt angle> because the staleness is
        MECHANICAL, not a lost setpoint.
        """
        for i in range(NJ):
            self.disableJoint(i)
        self.estopLatched = True
        self.wdgTripped = byWatchdog
        self._p("EVT ESTOP SRC=")
        self._pln(src)

    def _anyEnabled(self) -> bool:
        return any(self.j[i].en for i in range(NJ))

    # =====================================================================
    # COMMAND HANDLERS
    # =====================================================================

    def doVer(self) -> None:
        self._okPre()
        self._pln(" NAME=FACTORYLM-ARM PROTO=1.0 FW=1.0.0 JOINTS=6 BUILD=20260801")

    def doPng(self) -> None:
        self._okPre()
        self._p(" UP=")
        self._pln(str(self.millis()))

    def doSta(self) -> None:
        uncal = 0
        for i in range(NJ):
            if i == RESERVED_ID:
                continue
            if not self.j[i].cal:
                uncal += 1
            jt = self.j[i]
            self._kv("STA J", i)
            self._kv(" EN=", 1 if jt.en else 0)
            self._kv(" SET=", degOf(jt.setC))
            self._kv(" TGT=", degOf(jt.tgtC))
            self._kv(" MIN=", jt.minD)
            self._kv(" MAX=", jt.maxD)
            self._kv(" CAL=", 1 if jt.cal else 0)
            self._kv(" DPS=", jt.dps)
            self._kv(" MOV=", 1 if (jt.en and jt.setC != jt.tgtC) else 0)
            # JTO is emitted by the firmware and is ABSENT from the worked STA
            # replies in SERIAL-PROTOCOL.md sections 4 and 12. See README
            # "Where the doc and the firmware disagree", finding 1.
            self._kvln(" JTO=", 1 if jt.jogTimedOut else 0)
        self._kv("SYS ES=", 1 if self.estopLatched else 0)
        self._kv(" WD=", 1 if self.wdgTripped else 0)
        self._kv(" WDMS=", self.wdgMs)
        self._kv(" MIR=", self._mirName())
        self._kv(" UP=", self.millis())
        self._kvln(" UNCAL=", uncal)
        self._okPre()
        self._pln(" N=6")

    def _mirName(self) -> str:
        if self.mirMode == MIR_SAME:
            return "SAME"
        if self.mirMode == MIR_INV:
            return "INV"
        return "UNKNOWN"

    def doLimList(self) -> None:
        for i in range(NJ):
            if i == RESERVED_ID:
                continue
            self._kv("LIM J", i)
            self._kv(" MIN=", self.j[i].minD)
            self._kv(" MAX=", self.j[i].maxD)
            self._kvln(" CAL=", 1 if self.j[i].cal else 0)
        self._okPre()
        self._pln(" N=6")

    def doLimSet(self, i: int, mn: int, mx: int, cal: int) -> None:
        """ATOMIC. Every check runs before ANY field is written, so a rejected
        LIM leaves the previous envelope exactly as it was. There is no
        reachable state in which min has been updated and max has not."""
        # A joint the operator is physically holding does not get its envelope
        # moved underneath it. Same E9 code, DIFFERENT detail key -- the
        # remedies differ ("let go" vs "disable the joint").
        if self.j[i].jogActive:
            self._errJPre("E9", i)
            self._pln(" STATE=jogging")
            return

        if mn < 0 or mx > 180 or mn >= mx:
            self._errJPre("E10", i)
            self._kv(" REQMIN=", mn)
            self._kv(" REQMAX=", mx)
            self._pln(" LIMIT=0..180 MIN<MAX")
            return
        if (mx - mn) < LIM_MIN_SPAN_DEG:
            self._errJPre("E10", i)
            self._kv(" REQMIN=", mn)
            self._kv(" REQMAX=", mx)
            self._kvln(" MINSPAN=", LIM_MIN_SPAN_DEG)
            return
        if cal not in (0, 1):
            self._errJPre("E10", i)
            self._p(" REQCAL=")
            self._pln(str(cal))
            return

        lo, hi = mn * 100, mx * 100

        # On a DRIVEN joint the new range is accepted only if it still contains
        # that joint's own commanded value. Applying a range that excludes it
        # would turn a limit edit into an unrequested move.
        if self.j[i].en and (self.j[i].setC < lo or self.j[i].setC > hi):
            self._errJPre("E9", i)
            self._pln(" STATE=enabled")
            return

        # ---- every check has passed; only now is anything written ----
        self.j[i].minD = mn
        self.j[i].maxD = mx
        self.j[i].cal = (cal == 1)

        if self.j[i].en:
            # A pending target outside the new range is clamped INWARD.
            # Clamping can only make a move SHORTER, so a limit edit can never
            # create travel.
            if self.j[i].tgtC < lo:
                self.j[i].tgtC = lo
            if self.j[i].tgtC > hi:
                self.j[i].tgtC = hi
        else:
            if self.j[i].setC < lo:
                self.j[i].setC = lo
            if self.j[i].setC > hi:
                self.j[i].setC = hi
            self.j[i].tgtC = self.j[i].setC

        self._okPre()
        self._kv(" J", i)
        self._kv(" MIN=", self.j[i].minD)
        self._kv(" MAX=", self.j[i].maxD)
        self._kvln(" CAL=", 1 if self.j[i].cal else 0)

    def doMir(self, modeTok: str, offDeg: int) -> None:
        """MIR <SAME|INV|UNKNOWN> [offset_deg].

        EVERY MIR command sets the offset (0 when omitted), so an offset can
        never outlive the mode it arrived with. The joint-1-enabled check runs
        FIRST -- before the mode word is even looked at.
        """
        if self.j[1].en:
            self._errJPre("E9", 1)
            self._pln(" STATE=enabled")
            return

        modeTok = _upcase(modeTok)
        if modeTok == "SAME":
            m = MIR_SAME
        elif modeTok == "INV":
            m = MIR_INV
        elif modeTok == "UNKNOWN":
            m = MIR_UNKNOWN
        else:
            self._errPre("E11")
            self._p(self.g_verb)
            self._p(" MODE=")
            self._pln(modeTok)
            return

        # Unconditional: it bounds the store and the 18000 + 2*offset maths.
        if offDeg < -MIR_OFF_MAX_DEG or offDeg > MIR_OFF_MAX_DEG:
            self._errPre("E11")
            self._p(self.g_verb)
            self._kv(" OFF=", offDeg)
            self._kvln(" LIMIT=+/-", MIR_OFF_MAX_DEG)
            return

        offC = offDeg * 100

        # Only INV uses the offset, so only INV is checked against the
        # envelope. The whole of joint 1's legal travel must MIRROR to a legal
        # servo angle; if it does not, some reachable D4 angle would silently
        # clamp D5 and the pair would fight.
        if m == MIR_INV:
            axis = 18000 + 2 * offC
            imgLo = axis - self.j[1].maxD * 100  # image of MAX
            imgHi = axis - self.j[1].minD * 100  # image of MIN
            if imgLo < 0 or imgHi > 18000:
                self._errPre("E11")
                self._p(self.g_verb)
                self._kv(" OFF=", offDeg)
                self._kv(" MIN=", self.j[1].minD)
                self._kv(" MAX=", self.j[1].maxD)
                self._pln(" MIRROR=out_of_travel")
                return

        self.mirMode = m
        self.mirrorOffsetC = offC

        self._okPre()
        self._kv(" MODE=", self._mirName())
        self._kvln(" OFF=", offDeg)

    def doEna(self, i: int, adopt: int) -> None:
        if self.estopLatched:
            self._errJ("E7", i)
            return
        if adopt < self.j[i].minD or adopt > self.j[i].maxD:
            self._errJPre("E5", i)
            self._kv(" REQ=", adopt)
            self._kv(" MIN=", self.j[i].minD)
            self._kvln(" MAX=", self.j[i].maxD)
            return
        if self.j[i].en:
            self._errJ("E6", i)
            return
        if i == 1 and self.mirMode == MIR_UNKNOWN:
            self._errPre("E13")
            self._p(self.g_verb)
            self._pln(" JOINT=1 MIR=UNKNOWN")
            return
        self.enableJoint(i, adopt)
        self._okPre()
        self._kv(" J", i)
        self._kvln(" ADOPT=", adopt)

    def doDis(self, i: int) -> None:
        self.disableJoint(i)
        self._okPre()
        self._p(" J")
        self._pln(str(i))

    def doDisAll(self) -> None:
        for i in range(NJ):
            self.disableJoint(i)
        self._okPre()
        self._pln(" ALL")

    def doMov(self, i: int, deg: int) -> None:
        """Non-blocking: sets the target only. Out-of-range CLAMPS AND REPORTS
        rather than rejecting -- acceptable only because CL=1 is never silent
        and both REQ and SET are on the wire."""
        if self.estopLatched:
            self._errJ("E7", i)
            return
        if not self.j[i].en:
            self._errJ("E6", i)
            return

        applied = deg
        cl = 0
        if applied < self.j[i].minD:
            applied = self.j[i].minD
            cl = 1
        if applied > self.j[i].maxD:
            applied = self.j[i].maxD
            cl = 1
        self.j[i].tgtC = applied * 100

        # A finite move ENDS the jog and does not inherit its timeout.
        self.j[i].jogActive = False
        self.j[i].jogTimedOut = False

        self._okPre()
        self._kv(" J", i)
        self._kv(" REQ=", deg)
        self._kv(" SET=", applied)
        self._kvln(" CL=", cl)

    def doSpd(self, i: int, dps: int) -> None:
        if dps < SPD_MIN_DPS or dps > SPD_MAX_DPS:
            self._errJPre("E12", i)
            self._kv(" REQ=", dps)
            self._kv(" MIN=", SPD_MIN_DPS)
            self._kvln(" MAX=", SPD_MAX_DPS)
            return
        self.j[i].dps = dps
        self._okPre()
        self._kv(" J", i)
        self._kvln(" DPS=", self.j[i].dps)

    def doStp(self) -> None:
        """Abort motion on every ENABLED joint and hold. NOT an emergency stop:
        the channels stay driven. Not latching."""
        for i in range(NJ):
            if not self.j[i].en:
                continue
            self.j[i].tgtC = self.j[i].setC
            self.j[i].jogActive = False
            self.j[i].jogTimedOut = False
        self._okDone()

    def doStpJoint(self, i: int) -> None:
        if not self.j[i].en:
            self._errJ("E6", i)
            return
        self.j[i].tgtC = self.j[i].setC
        self.j[i].jogActive = False
        self.j[i].jogTimedOut = False
        self._okPre()
        self._p(" J")
        self._pln(str(i))

    def doJog(self, i: int, dirn: int) -> None:
        """Order mirrors doEna: latch first, then the ARGUMENT, then joint
        state. A direction outside -1..+1 is malformed whatever the joint is
        doing, and reporting E6 for it would send the operator to look at the
        wrong thing."""
        if self.estopLatched:
            self._errJ("E7", i)
            return
        if dirn < -1 or dirn > 1:
            self._errJPre("E14", i)
            self._p(" REQDIR=")
            self._pln(str(dirn))
            return
        if not self.j[i].en:
            self._errJ("E6", i)
            return

        self.j[i].jogTimedOut = False

        if dirn == 0:
            self.j[i].tgtC = self.j[i].setC
            self.j[i].jogActive = False
        else:
            self.j[i].tgtC = (self.j[i].maxD if dirn > 0 else self.j[i].minD) * 100
            self.j[i].jogActive = True
            self.j[i].jogMs = self.millis()

        self._okPre()
        self._kv(" J", i)
        self._kvln(" DIR=", dirn)

    def doEst(self, src: str) -> None:
        self.estopAll(src, False)
        self._okDone()

    def doClr(self) -> None:
        """No precondition: estopAll() has already cleared every enabled flag,
        and ENA requires an explicit adopt angle, so there is no path back to a
        driven joint without a deliberate operator action."""
        self.estopLatched = False
        self.wdgTripped = False
        for i in range(NJ):
            self.j[i].jogTimedOut = False
        self._okDone()

    def doWdg(self, ms: int) -> None:
        # KNOWN OVERLOAD, documented in SERIAL-PROTOCOL.md section 7: this
        # reuses E5, the adopt-angle code, for an out-of-range timeout. It is
        # the one operand-range failure that never got its own code.
        if ms != 0 and (ms < WDG_MIN_MS or ms > WDG_MAX_MS):
            self._errPlain("E5")
            return
        self.wdgMs = ms
        self._okPre()
        self._p(" MS=")
        self._pln(str(self.wdgMs))

    def doHlp(self) -> None:
        for text in (
            "; FACTORYLM-ARM protocol 1.0  115200 8N1  line ends CR or LF",
            "; joints 0=base D3  1=shoulder D4+D5 pair  3=elbow D6",
            ";        4=wristpitch D9  5=wristroll D10  6=gripper D11",
            ";        2 is RESERVED - the pair's 2nd servo, never addressable",
            "; VER / PNG / STA (or ?)   identify / ping / full status",
            "; LIM                      list limits",
            "; LIM j min max cal        set limits; joint must be DISABLED",
            "; MIR mode [off_deg]       SAME|INV|UNKNOWN; j1 DISABLED",
            ";                          INV mirrors about 90+off_deg, off=0 default",
            "; ENA j adopt_deg          attach; 1st pulse = adopt, nothing jumps",
            "; DIS j   |   DIS A        detach one / detach all",
            "; MOV j deg                set target; clamped, reply shows CL=1",
            "; SPD j dps                1..90 deg/s",
            "; STP [j]                  abort motion, hold, stay attached",
        ):
            self._pln(text)
        self._p("; JOG j -1|0|1             jog to envelope edge; resend every ")
        self._p(str(JOG_BEAT_MS))
        self._pln(" ms")
        for text in (
            "; EST (or !)               E-STOP: detach all + latch.  CLR clears",
            "; WDG ms                   0=off else 200..10000; host silence=stop",
            "; SYS WD=1 means the latch came from the watchdog, not the operator",
            "; angles are COMMANDED degrees - these servos have no feedback",
        ):
            self._pln(text)
        self._okDone()

    # =====================================================================
    # LINE PARSING AND DISPATCH
    # =====================================================================

    def _jointArg(self, jid: int) -> int | None:
        """Emits the error itself and returns None on failure, so callers stay
        flat -- exactly like the .ino's jointArg()."""
        if jid < 0 or jid >= NJ or jid == RESERVED_ID:
            self._errJoint(jid)
            return None
        return jid

    def _intArg(self, idx: int) -> int | None:
        v = parseInt(self.tokv[idx])
        if v is not None:
            return v
        self._errPre("E3")
        self._p(self.g_verb)
        self._p(" ARG=")
        self._pln(str(idx + 1))
        return None

    def _badArgc(self) -> None:
        self._errPre("E2")
        self._p(self.g_verb)
        self._p(" N=")
        self._pln(str(self.tokc))

    def dispatch(self) -> None:
        v = self.g_verb
        tokc = self.tokc

        if v in ("VER", "PNG", "STA", "HLP", "CLR", "EST"):
            if tokc != 0:
                self._badArgc()
                return
            {"VER": self.doVer, "PNG": self.doPng, "STA": self.doSta,
             "HLP": self.doHlp, "CLR": self.doClr,
             "EST": lambda: self.doEst("CMD")}[v]()
            return

        if v == "STP":
            if tokc == 0:
                self.doStp()
                return
            if tokc != 1:
                self._badArgc()
                return
            a0 = self._intArg(0)
            if a0 is None:
                return
            i = self._jointArg(a0)
            if i is None:
                return
            self.doStpJoint(i)
            return

        if v == "LIM":
            if tokc == 0:
                self.doLimList()
                return
            if tokc != 4:
                self._badArgc()
                return
            args = []
            for k in range(4):
                a = self._intArg(k)
                if a is None:
                    return
                args.append(a)
            i = self._jointArg(args[0])
            if i is None:
                return
            self.doLimSet(i, args[1], args[2], args[3])
            return

        if v == "MIR":
            # The offset is OPTIONAL and defaults to 0, so the one-argument
            # form every existing host sends keeps working unchanged.
            if tokc not in (1, 2):
                self._badArgc()
                return
            a0 = 0
            if tokc == 2:
                a0 = self._intArg(1)
                if a0 is None:
                    return
            self.doMir(self.tokv[0], a0)
            return

        if v == "DIS":
            if tokc != 1:
                self._badArgc()
                return
            self.tokv[0] = _upcase(self.tokv[0])
            # The literal 'A' is matched BEFORE integer parsing is attempted --
            # the single exception to the strict parser.
            if self.tokv[0] == "A":
                self.doDisAll()
                return
            a0 = self._intArg(0)
            if a0 is None:
                return
            i = self._jointArg(a0)
            if i is None:
                return
            self.doDis(i)
            return

        if v in ("ENA", "JOG", "MOV", "SPD"):
            if tokc != 2:
                self._badArgc()
                return
            a0 = self._intArg(0)
            if a0 is None:
                return
            a1 = self._intArg(1)
            if a1 is None:
                return
            i = self._jointArg(a0)
            if i is None:
                return
            {"ENA": self.doEna, "JOG": self.doJog,
             "MOV": self.doMov, "SPD": self.doSpd}[v](i, a1)
            return

        if v == "WDG":
            if tokc != 1:
                self._badArgc()
                return
            a0 = self._intArg(0)
            if a0 is None:
                return
            self.doWdg(a0)
            return

        # Unknown verb. Garbage input NAKs. It must NEVER trigger an e-stop --
        # an e-stop that fires on a typo trains the operator to ignore e-stops.
        self._errPre("E1")
        self._p("VERB TOKEN=")
        self._pln(self.g_verb)

    def handleLine(self) -> None:
        # The .ino parses `line` as a C STRING: loop() writes line[lineLen] = 0
        # and nextTok() stops dead at the first NUL. So a NUL that arrived ON
        # THE WIRE -- line noise, a wedged host, a half-initialised buffer --
        # TRUNCATES the line; it never sits inside a token.
        #
        # Python strings do not end at a NUL, so without this the byte becomes
        # an ordinary character and changes the verdict: "MOV\0 3 95" reads as a
        # five-character verb (E1 VERB) where the board sees a bare MOV with no
        # arguments (E2 MOV N=0). Found by an adversarial review, not at the
        # bench -- the bench is disconnected and this has never been observed on
        # real hardware.
        text = self.line.split("\x00", 1)[0]
        toks = text.replace("\t", " ").split(" ")
        toks = [t for t in toks if t != ""]
        if not toks:
            return  # blank line - silent, so a CRLF cannot NAK

        v = _upcase(toks[0])

        # Verbs are exactly three letters. g_verb is the first three characters
        # so every reply echoes it uppercase.
        vl = min(len(v), 4)
        self.g_verb = v[:3]

        if vl != 3 or not all("A" <= ch <= "Z" for ch in v[:3]):
            self._errPre("E1")
            self._p("VERB TOKEN=")
            self._pln(v)
            return

        rest = toks[1:]
        self.tokv = rest[:4]
        self.tokc = len(self.tokv)
        if len(rest) > 4:
            # More than four arguments. NOTE the reported count is the CAPPED
            # tokc (4), not the real argument count -- see README, "Quirks
            # reproduced deliberately".
            self._badArgc()
            return

        self.dispatch()

        # THE ONLY PLACE THE SERIAL WATCHDOG IS FED. Reaching here means a
        # complete line arrived, assembled without overflowing, held a legal
        # three-letter verb and an acceptable argument count, and was handed to
        # a real handler. A line the firmware then REFUSED still feeds it: the
        # host is demonstrably alive and speaking the protocol, which is the
        # only question the watchdog asks.
        self.lastRxMs = self.millis()

    # =====================================================================
    # SETUP / LOOP
    # =====================================================================

    def feed(self, data: bytes) -> None:
        """The rx section of loop(), then the rest of one loop() iteration.

        '!' and '?' are intercepted BEFORE line assembly, so an e-stop can
        never queue behind a half-typed line -- and, deliberately, neither of
        them feeds the serial watchdog.
        """
        if isinstance(data, str):
            raise TypeError("feed() takes bytes, not str -- this is a wire")
        for byte in data:
            ch = chr(byte)

            if ch == "!":
                self.g_verb = "EST"
                self.doEst("RT")
                continue
            if ch == "?":
                self.g_verb = "STA"
                self.doSta()
                continue

            # Both CR and LF terminate, so the Serial Monitor's line-ending
            # setting is irrelevant. An empty buffer is dropped silently, so
            # CRLF fires once, not twice.
            if ch in ("\r", "\n"):
                if self.lineOver:
                    # There is no trustworthy verb in a line we refused to
                    # finish reading, so E8 is the one error with no verb.
                    self._pln("ERR E8 LINE")
                elif self.line:
                    self.handleLine()
                self.line = ""
                self.lineOver = False
                continue

            if len(self.line) < (LINE_MAX - 1):
                self.line += ch
            else:
                self.lineOver = True  # discard through the next terminator

        self._loop()

    def _loop(self) -> None:
        """The non-rx sections of loop(), IN THE .ino's ORDER.

        interpolator -> jog timeout -> watchdog. The order is observable: the
        jog timeout writes tgtC and the watchdog can fire in the same iteration
        and wipe every joint, so running them the other way round changes what
        a following STA reports.
        """
        now = self.millis()

        # ---- interpolator ----
        # Elapsed-time integration, not a fixed step per tick. A stall produces
        # a proportionally larger next step, which still respects deg/s exactly.
        if _u32sub(now, self.lastTickMs) >= TICK_MS:
            el = _u32sub(now, self.lastTickMs)
            if el > TICK_CAP_MS:
                el = TICK_CAP_MS
            self.lastTickMs = now

            for i in range(NJ):
                if not self.j[i].en:
                    continue
                maxStep = _idiv(self.j[i].dps * el, 10)  # centidegrees
                if maxStep < 1:
                    maxStep = 1
                # TICK_CAP_MS bounds the elapsed SLICE; this bounds the
                # resulting STEP, which is the thing that actually moves the arm.
                if maxStep > MAX_STEP_C:
                    maxStep = MAX_STEP_C
                d = self.j[i].tgtC - self.j[i].setC
                if d > maxStep:
                    d = maxStep
                if d < -maxStep:
                    d = -maxStep
                if d != 0:
                    self.j[i].setC += d
                    self.writeJoint(i)

        # ---- jog command-age timeout ----
        # SILENT ON PURPOSE. Nothing is printed. The condition that fires it is
        # "the host stopped talking to us", so writing into that channel buys
        # nothing and adds an unsolicited line to a strict
        # accumulate-until-terminator reader. The state is LATCHED instead and
        # surfaced on STA as JTO=1. It HOLDS; it does not detach.
        for i in range(NJ):
            jt = self.j[i]
            if jt.en and jt.jogActive and _u32sub(now, jt.jogMs) > JOG_TIMEOUT_MS:
                jt.tgtC = jt.setC
                jt.jogActive = False
                jt.jogTimedOut = True

        # ---- serial watchdog ----
        # Armed only when a timeout is set AND at least one joint is enabled,
        # so a human at the Serial Monitor with nothing enabled is never
        # tripped by a watchdog they did not ask for.
        if self.wdgMs != 0 and not self.estopLatched and self._anyEnabled():
            el = _u32sub(now, self.lastRxMs)
            if el > self.wdgMs:
                self._p("EVT WDOG MS=")
                self._pln(str(el))
                self.estopAll("WDG", True)

    def advance(self, ms: int, step_ms: int = 1) -> None:
        """Run the board forward `ms` milliseconds of virtual time.

        `step_ms` is how much time passes between loop() iterations. The
        default of 1 is the realistic case -- a real loop() runs thousands of
        times a second. A LARGE step_ms is how you simulate a STALL: after
        advance(200, step_ms=200) the interpolator sees a single 200 ms slice,
        which is the case TICK_CAP_MS and MAX_STEP_C exist to bound.
        """
        if ms < 0:
            raise ValueError("time does not run backwards")
        if step_ms < 1:
            raise ValueError("step_ms must be at least 1 ms")
        remaining = ms
        while remaining > 0:
            slice_ms = min(step_ms, remaining)
            self.clock.advance(slice_ms)
            remaining -= slice_ms
            self._loop()

    def poll(self) -> None:
        """One loop() iteration with no time passing."""
        self._loop()
