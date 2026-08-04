"""Protocol tests for factorylm_arm_controller, run against the real board.

    python Software/tests/protocol_check.py [--port COM5] [--motion-ok]

WHY THIS EXISTS
    A 32 KB AVR cannot host a test framework, so the only honest way to test
    this firmware is to talk to it over the wire exactly as the console does.
    This script is the firmware's test suite.

DEFAULT MODE IS NON-MOTION
    It enables nothing and moves nothing, so it is safe to run with servo power
    on. --motion-ok adds bounded motion tests and prints a physical-safety
    acknowledgment first.

    Nothing here is an emergency stop. STP and the jog timeout abort motion and
    hold the last commanded value; they do not remove power. The rocker switch
    and the inline fuse are the emergency stop.

ONE TRANSMITTING CLIENT AT A TIME
    Close the arm console's browser tab before running this. The bridge writes
    to the serial port outside its own lock on a threading HTTP server, so two
    clients transmitting at once can interleave bytes mid-line.

THREE OUTCOMES, NOT TWO
    PASS  the firmware behaved as the protocol says.
    PEND  a case for a feature that is not implemented yet. Each PEND names the
          task that will turn it green. A PEND is not a failure and does not
          affect the exit code -- but a PEND that starts PASSing is a signal
          that the case needs promoting.
    FAIL  the firmware is wrong, or the harness is.

    Exit 0 = no FAILs (PENDs are fine).  1 = at least one FAIL.
    2 = the harness itself could not run (no port, port busy, board silent).
"""

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - environment problem, not a test result
    sys.stderr.write(
        "pyserial is not installed for this interpreter.\n"
        "On this bench machine use:  \"C:\\Program Files\\Python311\\python.exe\"\n"
    )
    sys.exit(2)

BAUD = 115200
REPLY_TIMEOUT_S = 2.0
BOOT_WAIT_S = 2.0          # Optiboot 4.4 holds the bus ~1 s after the DTR reset

PASSES = []
PENDING = []
FAILURES = []


class HarnessError(Exception):
    """The harness could not run. Distinct from a firmware failure."""


class Board:
    def __init__(self, port):
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.4)
        except Exception as exc:
            raise HarnessError("could not open %s: %s" % (port, exc))
        time.sleep(BOOT_WAIT_S)
        self.ser.reset_input_buffer()

    def cmd(self, line):
        """Send one line; return reply lines up to and including OK/ERR.

        Comment lines (';') are dropped. A command that draws no reply at all
        returns [] after REPLY_TIMEOUT_S -- that is a real result for the
        blank-line case, not a hang.
        """
        self.ser.write((line + "\n").encode("ascii"))
        out, deadline = [], time.time() + REPLY_TIMEOUT_S
        while time.time() < deadline:
            raw = self.ser.readline()
            if not raw:
                continue
            text = raw.decode("ascii", "replace").strip()
            if not text or text.startswith(";"):
                continue
            out.append(text)
            if text.startswith("OK") or text.startswith("ERR"):
                break
        return out

    def raw(self, data):
        self.ser.write(data)

    def quiet_for(self, seconds):
        """Return anything the board volunteered during a silent window."""
        self.ser.reset_input_buffer()
        time.sleep(seconds)
        return self.ser.read(4096).decode("ascii", "replace").strip()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def _record(name, ok, joined, needle, pending):
    if ok:
        PASSES.append(name)
        print("  PASS  %s" % name)
    elif pending:
        PENDING.append((name, pending))
        print("  PEND  %s\n        not implemented yet -- %s" % (name, pending))
    else:
        FAILURES.append(name)
        print("  FAIL  %s\n        wanted %r\n        got    %r" % (name, needle, joined))


def expect(name, reply, needle, pending=None):
    joined = " | ".join(reply)
    _record(name, needle in joined, joined, needle, pending)


def expect_absent(name, reply, needle, pending=None):
    joined = " | ".join(reply)
    _record(name, needle not in joined, joined, "NOT " + needle, pending)


def find_port():
    for p in list_ports.comports():
        if "2341" in (p.hwid or ""):
            return p.device
    ports = list_ports.comports()
    return ports[0].device if ports else None


# ---------------------------------------------------------------------------
# Non-motion tests. Nothing here enables a joint.
# ---------------------------------------------------------------------------

def non_motion_tests(b):
    print("\n-- handshake and status --")
    ver = b.cmd("VER")
    if not ver:
        raise HarnessError("the board did not answer VER at all -- wrong sketch, "
                           "wrong baud, or the port is held by another program")
    expect("VER identifies the arm controller", ver, "NAME=FACTORYLM-ARM")
    expect("PNG replies with uptime", b.cmd("PNG"), "OK PNG UP=")
    expect("STA reports six joints", b.cmd("STA"), "OK STA N=6")
    expect("STA carries a per-joint line", b.cmd("STA"), "STA J3 EN=")
    expect("SYS line reports the mirror state", b.cmd("STA"), "MIR=")
    expect("LIM with no arguments lists the limits", b.cmd("LIM"), "LIM J3 MIN=")
    expect("HLP replies", b.cmd("HLP"), "OK HLP")

    print("\n-- malformed input --")
    expect("unknown verb is refused", b.cmd("XYZ"), "ERR E1")
    expect("a four-letter verb is refused", b.cmd("MOOV 3 90"), "ERR E1")
    expect("a typo does not trigger a stop", b.cmd("XYZ"), "VERB TOKEN=")
    expect("missing arguments", b.cmd("ENA"), "ERR E2")
    expect("too many arguments", b.cmd("PNG 1 2 3"), "ERR E2")
    expect("non-numeric argument", b.cmd("MOV x 90"), "ERR E3")
    expect("trailing garbage after a number", b.cmd("MOV 3 90abc"), "ERR E3")
    expect("a bare minus sign is not a number", b.cmd("MOV 3 -"), "ERR E3")
    expect("overlong line is discarded", b.cmd("MOV 3 " + "9" * 80), "ERR E8")

    # A blank line must be silently ignored, so a CRLF terminator cannot NAK.
    # Sent raw: cmd() would wait REPLY_TIMEOUT_S for a reply that must not come.
    b.raw(b"\n")
    time.sleep(0.1)
    expect("a blank line is ignored and the link still works", b.cmd("PNG"), "OK PNG")

    print("\n-- joint ids --")
    expect("reserved joint id names itself", b.cmd("ENA 2 90"), "RESERVED=shoulder_pair")
    expect("joint id above range", b.cmd("MOV 9 90"), "ERR E4")
    expect("negative joint id", b.cmd("MOV -1 90"), "ERR E4")

    print("\n-- limits validation, joint disabled throughout --")
    b.cmd("LIM 3 70 110 0")
    expect("reversed limits", b.cmd("LIM 3 120 40 0"), "ERR E10")
    expect("equal limits", b.cmd("LIM 3 90 90 0"), "ERR E10")
    expect("min below the absolute bound", b.cmd("LIM 3 -5 110 0"), "ERR E10")
    expect("max above the absolute bound", b.cmd("LIM 3 70 200 0"), "ERR E10")
    expect("bad cal flag", b.cmd("LIM 3 70 110 7"), "ERR E10")
    expect("no rejected LIM changed anything",
           b.cmd("LIM"), "LIM J3 MIN=70 MAX=110 CAL=0")

    expect("a legal LIM is accepted", b.cmd("LIM 3 60 120 0"), "OK LIM J3")
    expect("the accepted LIM did apply", b.cmd("LIM"), "LIM J3 MIN=60 MAX=120")
    expect("repeating a LIM is accepted", b.cmd("LIM 3 60 120 0"), "OK LIM J3")
    expect("repeating it again is accepted", b.cmd("LIM 3 60 120 0"), "OK LIM J3")
    b.cmd("LIM 3 70 110 0")

    # Runs last among the limits cases and restores state itself: today this
    # LIM is ACCEPTED, so leaving it in place would corrupt every check above.
    expect("span below the 5 degree minimum is refused",
           b.cmd("LIM 3 90 93 0"), "MINSPAN",
           pending="Task 3 adds LIM_MIN_SPAN_DEG")
    b.cmd("LIM 3 70 110 0")
    expect("limits are back at the default after the span case",
           b.cmd("LIM"), "LIM J3 MIN=70 MAX=110")

    print("\n-- stop verbs, nothing enabled --")
    expect("bare STP is accepted with nothing enabled", b.cmd("STP"), "OK STP")
    expect("STP on a disabled joint", b.cmd("STP 3"), "ERR E6",
           pending="Task 2 gives STP an optional joint id")
    expect("STP on a bad joint id", b.cmd("STP 9"), "ERR E4",
           pending="Task 2 gives STP an optional joint id")
    expect("STP on the reserved id", b.cmd("STP 2"), "RESERVED=shoulder_pair",
           pending="Task 2 gives STP an optional joint id")

    print("\n-- motion verbs are refused while disabled --")
    expect("MOV on a disabled joint", b.cmd("MOV 3 90"), "ERR E6")
    expect("JOG on a disabled joint", b.cmd("JOG 3 1"), "ERR E6",
           pending="Task 5 adds the JOG verb")
    expect("JOG with a bad direction", b.cmd("JOG 3 5"), "ERR E14",
           pending="Task 5 adds the JOG verb")

    print("\n-- jog timeout latch is visible on STA --")
    expect("STA carries the JTO field", b.cmd("STA"), "JTO=",
           pending="Task 5 surfaces the jog-timeout latch")

    print("\n-- the shoulder stays locked --")
    expect("shoulder refuses to enable while the mirror is unknown",
           b.cmd("ENA 1 90"), "ERR E13")
    expect("mirror mode is still unknown", b.cmd("STA"), "MIR=UNKNOWN")

    print("\n-- nothing above enabled anything --")
    expect("J3 is still disabled", b.cmd("STA"), "STA J3 EN=0")
    expect("J0 is still disabled", b.cmd("STA"), "STA J0 EN=0")
    expect("the e-stop is not latched", b.cmd("STA"), "ES=0")


# ---------------------------------------------------------------------------
# Bounded motion tests. Only with --motion-ok.
# ---------------------------------------------------------------------------

def motion_tests(b):
    print("\n-- BOUNDED MOTION (--motion-ok) --")
    b.cmd("LIM 3 70 110 0")
    expect("enable adopts the stated angle", b.cmd("ENA 3 90"), "OK ENA J3")
    expect("STP stops one joint", b.cmd("STP 3"), "OK STP J3",
           pending="Task 2 gives STP an optional joint id")

    expect("LIM that still contains the joint is accepted",
           b.cmd("LIM 3 60 120 0"), "OK LIM J3",
           pending="Task 4 allows LIM on a driven joint")
    expect("LIM that would exclude the joint is refused",
           b.cmd("LIM 3 100 120 0"), "ERR E9")

    print("\n-- jog --")
    expect("JOG is accepted on an enabled joint", b.cmd("JOG 3 1"), "OK JOG J3 DIR=1",
           pending="Task 5 adds the JOG verb")
    expect("LIM is refused while that joint is jogging",
           b.cmd("LIM 3 70 100 0"), "STATE=jogging",
           pending="Task 5 adds the jogActive guard")

    print("     going quiet for 1.4 s -- the timeout must fire SILENTLY")
    stray = b.quiet_for(1.4)
    if stray:
        FAILURES.append("the jog timeout must not emit anything")
        print("  FAIL  the jog timeout emitted an unsolicited line\n        got %r" % stray)
    else:
        PASSES.append("the jog timeout was silent on the wire")
        print("  PASS  the jog timeout was silent on the wire")

    expect("the timeout latched JTO on that joint", b.cmd("STA"), "JTO=1",
           pending="Task 5 adds the jog-timeout latch")
    expect("a deliberate command clears the latch", b.cmd("JOG 3 0"), "OK JOG J3 DIR=0",
           pending="Task 5 adds the JOG verb")
    expect("JTO is clear again", b.cmd("STA"), "JTO=0",
           pending="Task 5 adds the jog-timeout latch")

    print("\n-- a finite move must not inherit the jog timeout --")
    b.cmd("MOV 3 95")
    time.sleep(1.4)
    expect_absent("finite move survived past the jog timeout window",
                  b.cmd("STA"), "JTO=1",
                  pending="Task 5 adds the jog-timeout latch")

    b.cmd("DIS 3")
    expect("the joint is disabled again", b.cmd("STA"), "STA J3 EN=0")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default=None, help="serial port; auto-detected if omitted")
    ap.add_argument("--motion-ok", action="store_true",
                    help="enable bounded motion tests -- a servo WILL move")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.stderr.write("HARNESS ERROR: no serial port found. Plug the board in with a "
                         "DATA cable.\n")
        return 2

    if args.motion_ok:
        print("=" * 70)
        print(" --motion-ok: ONE JOINT WILL BE ENABLED AND MOVED.")
        print(" Horn off. Nothing in the path. Hand on the rocker switch.")
        print(" The rocker and the fuse are the emergency stop -- not this script.")
        print("=" * 70)
        time.sleep(3.0)

    print("port %s @ %d" % (port, BAUD))
    print("mode: %s" % ("BOUNDED MOTION" if args.motion_ok else "non-motion (safe)"))

    b = None
    try:
        b = Board(port)
        non_motion_tests(b)
        if args.motion_ok:
            motion_tests(b)
    except HarnessError as exc:
        sys.stderr.write("\nHARNESS ERROR: %s\n" % exc)
        return 2
    except Exception as exc:  # noqa: BLE001 - any crash here is a harness fault
        sys.stderr.write("\nHARNESS ERROR: %s: %s\n" % (type(exc).__name__, exc))
        return 2
    finally:
        if b is not None:
            try:
                b.cmd("DIS A")
            except Exception:
                pass
            b.close()

    print("\n" + "=" * 70)
    print(" %d passed, %d pending, %d failed" % (len(PASSES), len(PENDING), len(FAILURES)))
    if PENDING:
        print("\n PENDING -- expected red until the named task lands:")
        for name, why in PENDING:
            print("   - %s\n       %s" % (name, why))
    if FAILURES:
        print("\n FAILED -- the firmware is wrong, or this harness is:")
        for name in FAILURES:
            print("   - %s" % name)
    print("=" * 70)
    return 1 if FAILURES else 0


if __name__ == "__main__":
    sys.exit(main())
