"""Conformance suite for the FactoryLM arm serial protocol, run against the sim.

WHAT THIS IS
    `Documentation/SERIAL-PROTOCOL.md` section 12 ("Worked session -- exact
    bytes") and section 13 ("The zero-power acceptance test") are literal test
    vectors. Somebody wrote them down expecting them to be checked. Until
    tonight the only way to check them was to have a board, a USB cable and a
    multimeter. This file checks them with none of those.

    It is the offline twin of `Software/tests/protocol_check.py`, which runs the
    same shape of assertions against the real board. The two are deliberately
    written the same way -- send a line, accumulate until OK or ERR, assert on
    the strings -- so an expectation proven here can be re-pointed at hardware
    by swapping the transport and nothing else.

THE ONE RULE FOR THIS FILE
    ASSERTIONS GO THROUGH BYTES IN AND BYTES OUT. No test reaches into
    `sim.j[3].setC`. If a behaviour has no representation on the wire it is not
    a protocol behaviour and does not belong here. The two exceptions are the
    injected clock and `pin_state()`, neither of which has a wire representation
    by definition -- on a real board `pin_state` is a multimeter probe.

WHERE THE DOC AND THE FIRMWARE DISAGREE
    The .ino wins, every time, because that is what a real board would do. Each
    such case is asserted the .ino's way with a comment naming the stale doc
    section on the line itself. README.md summarises them; the comment here is
    where the finding actually lives.

NOTHING HERE TOUCHES HARDWARE
    No serial port is opened, no servo is driven, no wall-clock second passes.
    Every timer in the firmware is exercised by moving a virtual clock.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

# ---------------------------------------------------------------------------
# Loading the two modules by file path.
#
# Same motivation as Software/lerobot_robot_emre_arm/tests/_repo_files.py: the
# suite must not depend on how pytest happened to be invoked. `python -m pytest`
# from Software/arm-sim puts the CWD on sys.path and a plain `import arm_sim`
# would work; a bare `pytest` from the repo root does not, and the suite would
# fail to collect for a reason that has nothing to do with the firmware.
#
# UNLIKE _repo_files.py there is no synthetic package name here, and that is a
# deliberate difference rather than a missed convention. That trick exists to
# stop a real `__init__.py` (which imports lerobot) from ever executing. There
# is no package and no `__init__.py` in arm-sim, and `fake_serial.py` does
# `from arm_sim import ArmSim` -- so the modules must be registered under their
# own names or that import would load a second copy of the firmware state.
# ---------------------------------------------------------------------------

SIM_DIR = Path(__file__).resolve().parent.parent


def _load(name: str):
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, SIM_DIR / f"{name}.py")
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not build an import spec for {name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module  # registered BEFORE exec so fake_serial resolves
    spec.loader.exec_module(module)
    return module


arm_sim = _load("arm_sim")
fake_serial = _load("fake_serial")

ArmSim = arm_sim.ArmSim
VirtualClock = arm_sim.VirtualClock
FakeSerial = fake_serial.FakeSerial
PortClosedError = fake_serial.PortClosedError

#: Every servo pin on the Uno, in the .ino's own order. Sections 2, 10 and 18
#: of the acceptance test probe exactly these seven.
ALL_PINS = (3, 4, 5, 6, 9, 10, 11)


# ---------------------------------------------------------------------------
# Transport helpers -- shaped like protocol_check.Board so the assertions port
# ---------------------------------------------------------------------------

def read_reply(port) -> list[str]:
    """Accumulate lines until OK or ERR. That is the entire host algorithm.

    `;` comments and blank lines are dropped, exactly as protocol_check.py does.
    An `EVT` line is NOT a terminator and is returned as part of the reply --
    a host that treated it as one would hang.
    """
    out: list[str] = []
    while True:
        raw = port.readline()
        if not raw:
            return out
        text = raw.decode("latin-1").strip()
        if not text or text.startswith(";"):
            continue
        out.append(text)
        if text.startswith("OK") or text.startswith("ERR"):
            return out


def cmd(port, line: str) -> list[str]:
    port.write((line + "\n").encode("ascii"))
    return read_reply(port)


def one(port, line: str) -> str:
    """The single terminator line, for commands that emit no data lines."""
    reply = cmd(port, line)
    assert reply, f"{line!r} drew no reply at all"
    return reply[-1]


def sta(port) -> dict:
    """STA as {joint_id: {KEY: value}} plus {'SYS': {...}}.

    Lets a test assert on one joint's field instead of a substring of the whole
    dump -- the difference between "J3 stopped" and "some line somewhere
    contained TGT=90".
    """
    out: dict = {}
    for line in cmd(port, "STA"):
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "STA" and len(parts) > 1 and parts[1].startswith("J"):
            out[int(parts[1][1:])] = dict(p.split("=", 1) for p in parts[2:] if "=" in p)
        elif parts[0] == "SYS":
            out["SYS"] = dict(p.split("=", 1) for p in parts[1:] if "=" in p)
    return out


@pytest.fixture
def port():
    """A freshly opened board with the boot banner flushed.

    That flush is not tidiness -- it is step 3 of the mandatory connect
    handshake in SERIAL-PROTOCOL.md section 9.
    """
    p = FakeSerial("SIM", 115200, timeout=0.3)
    p.reset_input_buffer()
    return p


def push_defaults(port, joint: int, lo: int, hi: int, cal: int = 0, dps: int = 30):
    assert one(port, f"LIM {joint} {lo} {hi} {cal}").startswith("OK LIM")
    assert one(port, f"SPD {joint} {dps}").startswith("OK SPD")


# ===========================================================================
# SECTION 12 -- the worked session, exact bytes
# ===========================================================================

def test_s12_step0_opening_the_port_emits_the_banner_then_rdy():
    """Section 12 step 0 / acceptance row 1. The flush usually eats this."""
    p = FakeSerial("SIM", 115200, timeout=0.3)
    banner = p.read(4096).decode("latin-1").splitlines()
    assert banner[-1] == "RDY NAME=FACTORYLM-ARM PROTO=1.0 FW=1.0.0"
    assert any(ln.startswith(";  NOTHING IS ATTACHED.") for ln in banner)
    assert any("THE ROCKER SWITCH AND THE INLINE FUSE ARE THE REAL E-STOP" in ln
               for ln in banner)
    # RDY is emitted once and never again. Section 9 is explicit that a host
    # must QUERY with VER rather than wait for this line.
    assert p.in_waiting == 0


def test_s12_step1_ver_is_the_gate(port):
    assert one(port, "VER") == (
        "OK VER NAME=FACTORYLM-ARM PROTO=1.0 FW=1.0.0 JOINTS=6 BUILD=20260801"
    )


def test_s12_step2_the_state_push_is_fourteen_commands(port):
    """Six LIM/SPD pairs in ascending order, then MIR, then WDG."""
    sent = 0
    for jid in (0, 1, 3, 4, 5, 6):
        assert one(port, f"LIM {jid} 70 110 0") == f"OK LIM J{jid} MIN=70 MAX=110 CAL=0"
        assert one(port, f"SPD {jid} 30") == f"OK SPD J{jid} DPS=30"
        sent += 2
    # The offset argument is omitted, so it defaults to 0 and the reply says so.
    assert one(port, "MIR UNKNOWN") == "OK MIR MODE=UNKNOWN OFF=0"
    assert one(port, "WDG 1000") == "OK WDG MS=1000"
    sent += 2
    assert sent == 14

    png = one(port, "PNG")
    # UP= is millis(); section 12's `UP=2431` is illustrative, not reproducible.
    assert png.startswith("OK PNG UP=")
    assert png.split("UP=")[1].isdigit()


def test_s12_step3_ena_preloads_the_adopt_pulse_before_attach(port):
    """The pulse is loaded BEFORE attach, so the first pulse the servo ever
    sees equals the adopted angle. Section 12 quotes ~1524 us for 95 degrees;
    the .ino's integer maths gives 1523 (see README, numeric nits)."""
    assert port.sim.pin_state(6) == ("LOW", None)
    before = port.sim.millis()

    assert one(port, "ENA 3 95") == "OK ENA J3 ADOPT=95"

    # No virtual time has passed, so the interpolator has not run once. The
    # pin is already at the adopt angle because it was PRE-LOADED, not walked
    # to. That is the whole of landmine 2 in the .ino header.
    assert port.sim.millis() == before
    assert port.sim.pin_state(6) == ("PULSING", arm_sim.degCToUs(9500))
    assert arm_sim.degCToUs(9500) == 1523

    snap = sta(port)
    assert snap[3]["EN"] == "1"
    assert snap[3]["SET"] == "95"
    assert snap[3]["TGT"] == "95"
    assert snap[3]["MOV"] == "0"


def test_s12_step4_mov_is_nonblocking_and_sta_shows_it_walking(port):
    assert one(port, "ENA 3 95") == "OK ENA J3 ADOPT=95"
    assert one(port, "MOV 3 110") == "OK MOV J3 REQ=110 SET=110 CL=0"

    # MOV returned immediately: SET has not moved and MOV=1 already.
    snap = sta(port)
    assert snap[3]["SET"] == "95"
    assert snap[3]["TGT"] == "110"
    assert snap[3]["MOV"] == "1"

    # Section 12 shows SET=103 mid-move. That exact figure depends on when the
    # host happened to poll, so it is not asserted -- the walking IS.
    port.sim.advance(200)
    mid = sta(port)
    assert 95 < int(mid[3]["SET"]) < 110
    assert mid[3]["MOV"] == "1"

    port.sim.advance(600)
    done = sta(port)
    assert done[3]["SET"] == "110"
    assert done[3]["MOV"] == "0"


def test_s12_step5_out_of_range_mov_is_accepted_clamped_and_flagged(port):
    one(port, "ENA 3 95")
    assert one(port, "MOV 3 130") == "OK MOV J3 REQ=130 SET=110 CL=1"


def test_s12_step6_bang_byte_estops_and_evt_precedes_the_terminator(port):
    one(port, "ENA 3 95")
    port.write(b"!")  # one byte, no newline
    reply = read_reply(port)
    assert reply == ["EVT ESTOP SRC=RT", "OK EST"]
    # Every channel detached and every signal pin driven LOW.
    assert all(port.sim.pin_state(pin) == ("LOW", None) for pin in ALL_PINS)


def test_s12_step7_the_latch_means_what_it_says(port):
    one(port, "ENA 3 95")
    port.write(b"!")
    read_reply(port)
    assert one(port, "MOV 3 100") == "ERR E7 MOV JOINT=3"
    assert one(port, "ENA 3 95") == "ERR E7 ENA JOINT=3"


def test_s12_step8_recover_deliberately_with_a_fresh_adopt_angle(port):
    one(port, "ENA 3 95")
    port.write(b"!")
    read_reply(port)
    assert one(port, "CLR") == "OK CLR"
    # The staleness after an e-stop is MECHANICAL. 88, not the 95 from before.
    assert one(port, "ENA 3 88") == "OK ENA J3 ADOPT=88"


def test_s12_other_replies_worth_recognising(port):
    assert one(port, "ENA 2 90") == "ERR E4 ENA JOINT=2 RESERVED=shoulder_pair"
    assert one(port, "ENA 1 90") == "ERR E13 ENA JOINT=1 MIR=UNKNOWN"
    assert one(port, "MOV 4 100") == "ERR E6 MOV JOINT=4"
    assert one(port, "ENA 3 140") == "ERR E5 ENA JOINT=3 REQ=140 MIN=70 MAX=110"
    assert one(port, "SPD 3 500") == "ERR E12 SPD JOINT=3 REQ=500 MIN=1 MAX=90"
    assert one(port, "MIR SIDEWAYS") == "ERR E11 MIR MODE=SIDEWAYS"
    assert one(port, "MIR INV 40") == (
        "ERR E11 MIR OFF=40 MIN=70 MAX=110 MIRROR=out_of_travel"
    )
    assert one(port, "MIR INV 5") == "OK MIR MODE=INV OFF=5"
    assert one(port, "DIS A") == "OK DIS ALL"


# ===========================================================================
# SECTION 4 -- the worked STA reply, byte for byte
# ===========================================================================

def test_s4_sta_reply_is_byte_for_byte_except_the_missing_JTO_field(port):
    """Reproduces section 4's worked reply exactly, and shows what is stale.

    DOC/FIRMWARE DISAGREEMENT 1. Section 4 calls itself "byte for byte" and
    section 12 calls itself "exact bytes"; neither carries the JTO field. The
    .ino's doSta() emits `JTO=<0|1>` at the end of every joint line, and
    section 3's JOG text promises it does ("surfaced as JTO=<0|1> on every STA
    joint line"). The two halves of the doc contradict each other; the .ino
    settles it. Any host built to the section 4 field list will meet a field
    it does not know about on the very first poll.
    """
    one(port, "LIM 3 70 110 1")   # section 4's example has CAL=1 on J3 only
    one(port, "WDG 1000")
    one(port, "ENA 3 95")
    one(port, "MOV 3 110")

    lines = cmd(port, "STA")

    for jid in (0, 1, 4, 5, 6):
        assert f"STA J{jid} EN=0 SET=90 TGT=90 MIN=70 MAX=110 CAL=0 DPS=30 MOV=0 JTO=0" in lines
        #                                        section 4 stops here ^^^^^^^

    assert "STA J3 EN=1 SET=95 TGT=110 MIN=70 MAX=110 CAL=1 DPS=30 MOV=1 JTO=0" in lines
    #                                                   section 4 stops here ^^^^^^

    sys_line = next(ln for ln in lines if ln.startswith("SYS "))
    fields = dict(tok.split("=", 1) for tok in sys_line.split()[1:])
    assert fields["ES"] == "0"
    assert fields["WD"] == "0"
    assert fields["WDMS"] == "1000"
    assert fields["MIR"] == "UNKNOWN"
    assert fields["UNCAL"] == "5"   # J3 is calibrated, the other five are not
    assert fields["UP"].isdigit()   # section 4's UP=48211 is illustrative
    assert list(fields) == ["ES", "WD", "WDMS", "MIR", "UP", "UNCAL"]

    assert lines[-1] == "OK STA N=6"


def test_there_is_never_a_STA_J2_line(port):
    """`N=6` counts ADDRESSABLE joints. Seven physical servos, six of them
    commandable, and the gap is deliberate."""
    for setup in ([], ["ENA 3 90"], ["ENA 3 90", "EST"]):
        for line in setup:
            cmd(port, line)
        lines = cmd(port, "STA")
        assert not any(ln.startswith("STA J2") for ln in lines)
        assert len([ln for ln in lines if ln.startswith("STA J")]) == 6
        assert lines[-1] == "OK STA N=6"
        cmd(port, "CLR")
        cmd(port, "DIS A")

    lim = cmd(port, "LIM")
    assert not any(ln.startswith("LIM J2") for ln in lim)
    assert lim[-1] == "OK LIM N=6"


# ===========================================================================
# SECTION 13 -- the zero-power acceptance test, row by row
# ===========================================================================

def test_s13_rows_2_to_10_nothing_is_driven_until_it_is_asked_for(port):
    # Row 2: nothing is attached at boot.
    assert all(port.sim.pin_state(pin) == ("LOW", None) for pin in ALL_PINS)

    # Row 3
    assert "NAME=FACTORYLM-ARM" in one(port, "VER")

    # Row 4
    lines = cmd(port, "STA")
    assert len([ln for ln in lines if ln.startswith("STA J")]) == 6
    assert "UNCAL=6" in next(ln for ln in lines if ln.startswith("SYS "))
    assert lines[-1] == "OK STA N=6"

    # Rows 5, 6, 7
    assert one(port, "ENA 2 90") == "ERR E4 ENA JOINT=2 RESERVED=shoulder_pair"
    assert one(port, "ENA 1 90") == "ERR E13 ENA JOINT=1 MIR=UNKNOWN"
    assert one(port, "MOV 3 100") == "ERR E6 MOV JOINT=3"

    # Row 8, 9: D6 is now emitting real pulses. ~0.37 V on a meter; 1472 us
    # here, which is the figure V2-SERVO-POWER-AND-WIRING.md section 6 used to
    # derive that voltage in the first place.
    assert one(port, "ENA 3 90") == "OK ENA J3 ADOPT=90"
    assert port.sim.pin_state(6) == ("PULSING", 1472)

    # Row 10: only the joint you enabled is driven.
    for pin in (3, 4, 5, 9, 10, 11):
        assert port.sim.pin_state(pin) == ("LOW", None)


def test_s13_rows_11_to_13_clamp_and_report_never_silently(port):
    one(port, "ENA 3 90")

    # Row 11
    assert one(port, "MOV 3 110") == "OK MOV J3 REQ=110 SET=110 CL=0"
    seen = []
    for _ in range(4):
        port.sim.advance(200)
        snap = sta(port)
        seen.append((int(snap[3]["SET"]), snap[3]["MOV"]))
    assert seen[0][1] == "1"                      # still climbing
    assert [s for s, _ in seen] == sorted(s for s, _ in seen)   # monotone
    assert seen[-1] == (110, "0")                 # settled

    # Rows 12 and 13
    assert one(port, "MOV 3 130") == "OK MOV J3 REQ=130 SET=110 CL=1"
    assert one(port, "MOV 3 0") == "OK MOV J3 REQ=0 SET=70 CL=1"


def test_s13_rows_14_to_18_estop_detaches_and_latches(port):
    one(port, "ENA 3 90")

    # Row 14
    port.write(b"!")
    assert read_reply(port) == ["EVT ESTOP SRC=RT", "OK EST"]

    # Row 15 -- "the most important line in this table". See README: the sim
    # asserts the firmware's INTENT (detach then drive LOW). It structurally
    # cannot reproduce the Servo-library race this row exists to catch.
    assert port.sim.pin_state(6) == ("LOW", None)

    # Row 16
    assert one(port, "MOV 3 100") == "ERR E7 MOV JOINT=3"

    # Row 17
    assert one(port, "CLR") == "OK CLR"
    assert one(port, "ENA 3 90") == "OK ENA J3 ADOPT=90"
    assert one(port, "DIS A") == "OK DIS ALL"

    # Row 18
    assert all(port.sim.pin_state(pin) == ("LOW", None) for pin in ALL_PINS)


def test_s13_rows_19_and_20_garbage_naks_and_never_estops(port):
    # Row 19
    assert one(port, "FOO") == "ERR E1 VERB TOKEN=FOO"
    assert sta(port)["SYS"]["ES"] == "0"

    # Row 20: a 60-character line of junk. Discarded through the next
    # terminator and NEVER acted on.
    assert one(port, "X" * 60) == "ERR E8 LINE"
    assert sta(port)["SYS"]["ES"] == "0"
    # The board is still perfectly usable afterwards.
    assert one(port, "PNG").startswith("OK PNG UP=")


def test_s13_rows_21_to_24_the_mirror_arguments_are_checked(port):
    assert one(port, "MIR SIDEWAYS") == "ERR E11 MIR MODE=SIDEWAYS"
    assert one(port, "MIR INV 200") == "ERR E11 MIR OFF=200 LIMIT=+/-90"
    assert one(port, "MIR INV 40") == (
        "ERR E11 MIR OFF=40 MIN=70 MAX=110 MIRROR=out_of_travel"
    )
    assert one(port, "MIR INV 5") == "OK MIR MODE=INV OFF=5"

    sys_line = next(ln for ln in cmd(port, "STA") if ln.startswith("SYS "))
    assert "MIR=INV" in sys_line
    # Row 24: SYS shows the mode but NEVER the offset. A host that wants to
    # display it must remember what it sent.
    assert "OFF" not in sys_line


def test_s13_rows_25_to_27_the_shoulder_pair_moves_as_one(port):
    one(port, "MIR INV 5")

    # Row 25: both pulsing, neither at LOW, and D5 is NOT identical to D4 --
    # at OFF=5 it is commanded to 100 degrees, which is the mirror doing its job.
    assert one(port, "ENA 1 90") == "OK ENA J1 ADOPT=90"
    assert port.sim.pin_state(4) == ("PULSING", arm_sim.degCToUs(9000))
    assert port.sim.pin_state(5) == ("PULSING", arm_sim.degCToUs(10000))
    assert port.sim.pin_state(4) != port.sim.pin_state(5)

    # Row 26: never one and not the other.
    assert one(port, "DIS 1") == "OK DIS J1"
    assert port.sim.pin_state(4) == ("LOW", None)
    assert port.sim.pin_state(5) == ("LOW", None)

    # Row 27: the lock comes back.
    assert one(port, "MIR UNKNOWN") == "OK MIR MODE=UNKNOWN OFF=0"
    assert one(port, "ENA 1 90") == "ERR E13 ENA JOINT=1 MIR=UNKNOWN"


# ===========================================================================
# GOTCHA -- argument counts. LIM takes FOUR. STA takes NONE.
# ===========================================================================

def test_lim_takes_four_arguments_and_three_is_e2(port):
    assert one(port, "LIM 3 70 110") == "ERR E2 LIM N=3"
    assert one(port, "LIM 3 70") == "ERR E2 LIM N=2"
    assert one(port, "LIM 3") == "ERR E2 LIM N=1"
    # Zero arguments is the LIST form, not an error.
    assert cmd(port, "LIM")[-1] == "OK LIM N=6"
    assert one(port, "LIM 3 70 110 0") == "OK LIM J3 MIN=70 MAX=110 CAL=0"
    # Nothing above changed the envelope.
    assert "LIM J3 MIN=70 MAX=110 CAL=0" in cmd(port, "LIM")


def test_sta_takes_no_arguments_at_all(port):
    assert one(port, "STA J0") == "ERR E2 STA N=1"
    assert one(port, "STA 0") == "ERR E2 STA N=1"
    assert one(port, "STA J0 J1") == "ERR E2 STA N=2"
    # And the argument-free forms of the other bare verbs.
    assert one(port, "PNG 1 2 3") == "ERR E2 PNG N=3"
    assert one(port, "VER 1") == "ERR E2 VER N=1"
    assert one(port, "CLR 1") == "ERR E2 CLR N=1"
    assert one(port, "EST 1") == "ERR E2 EST N=1"


def test_more_than_four_arguments_reports_the_capped_count(port):
    """QUIRK REPRODUCED, NOT FIXED. The .ino stops collecting at four tokens,
    then reports `N=<tokc>` -- which is 4, not the real argument count. A host
    that echoes N back to the operator will say "4 arguments" about a line
    that had five."""
    assert one(port, "LIM 3 70 110 0 9") == "ERR E2 LIM N=4"
    assert one(port, "MOV 3 90 1 2 3") == "ERR E2 MOV N=4"


# ===========================================================================
# GOTCHA -- opening the port DTR-resets the board. It keeps NOTHING.
# ===========================================================================

def test_opening_the_port_resets_the_board_and_loses_every_pushed_value():
    """The assertion is only worth anything against state we pushed OURSELVES,
    so the same ArmSim is reopened. A second FakeSerial with a fresh ArmSim
    would be trivially at defaults and would prove nothing."""
    sim = ArmSim()
    first = FakeSerial("SIM", 115200, timeout=0.3, sim=sim)
    first.reset_input_buffer()

    assert one(first, "LIM 6 10 70 1") == "OK LIM J6 MIN=10 MAX=70 CAL=1"
    assert one(first, "MIR INV 0") == "OK MIR MODE=INV OFF=0"
    assert one(first, "WDG 1000") == "OK WDG MS=1000"
    assert one(first, "SPD 3 12") == "OK SPD J3 DPS=12"
    assert one(first, "ENA 3 95") == "OK ENA J3 ADOPT=95"
    assert first.sim.pin_state(6) == ("PULSING", arm_sim.degCToUs(9500))
    first.close()

    second = FakeSerial("SIM", 115200, timeout=0.3, sim=sim)
    second.reset_input_buffer()

    lim = cmd(second, "LIM")
    for jid in (0, 1, 3, 4, 5, 6):
        assert f"LIM J{jid} MIN=70 MAX=110 CAL=0" in lim   # limits: gone
    snap = sta(second)
    assert snap["SYS"]["MIR"] == "UNKNOWN"    # mirror relation: gone
    assert snap["SYS"]["WDMS"] == "0"         # watchdog: back to disabled
    assert snap["SYS"]["UNCAL"] == "6"        # every CAL flag: back to 0
    assert snap["SYS"]["ES"] == "0"
    for jid in (0, 1, 3, 4, 5, 6):
        assert snap[jid]["EN"] == "0"         # every joint: detached
        assert snap[jid]["DPS"] == "30"       # every slew rate: back to default
    assert all(second.sim.pin_state(pin) == ("LOW", None) for pin in ALL_PINS)


# ===========================================================================
# GOTCHA -- joint 6's real range lies BELOW the 70-110 boot default
# ===========================================================================

def test_joint6_real_range_lies_below_the_boot_default(port):
    """J6 gripper is locked at 10-70 in joint-limits.csv. The firmware boots at
    70-110. The two overlap at exactly one degree. This is the single most
    likely way a caller gets silently wrong behaviour after a reset -- and it
    bites at ENABLE, before any MOV is even attempted."""
    # A perfectly sane adopt angle from the CSV's own home column is REFUSED.
    assert one(port, "ENA 6 40") == "ERR E5 ENA JOINT=6 REQ=40 MIN=70 MAX=110"

    # Enable inside the default instead, and every real gripper angle clamps.
    assert one(port, "ENA 6 90") == "OK ENA J6 ADOPT=90"
    assert one(port, "MOV 6 40") == "OK MOV J6 REQ=40 SET=70 CL=1"   # closed-ish
    assert one(port, "MOV 6 10") == "OK MOV J6 REQ=10 SET=70 CL=1"   # closed
    assert one(port, "MOV 6 70") == "OK MOV J6 REQ=70 SET=70 CL=0"   # the one degree

    # Push the measured limits and it all works. This is what the connect
    # handshake's state push (section 9 step 6) is FOR.
    assert one(port, "DIS 6") == "OK DIS J6"
    assert one(port, "LIM 6 10 70 1") == "OK LIM J6 MIN=10 MAX=70 CAL=1"
    assert one(port, "ENA 6 40") == "OK ENA J6 ADOPT=40"
    assert one(port, "MOV 6 10") == "OK MOV J6 REQ=10 SET=10 CL=0"


def test_a_clamped_move_never_reaches_the_request(port):
    """CL=1 is not advisory. The joint stops at the limit and stays there --
    which is why a UI that snaps the slider back silently is a hazard."""
    one(port, "ENA 3 90")
    assert one(port, "MOV 3 130") == "OK MOV J3 REQ=130 SET=110 CL=1"
    port.sim.advance(3000)
    snap = sta(port)
    assert snap[3]["SET"] == "110"
    assert snap[3]["TGT"] == "110"
    assert snap[3]["MOV"] == "0"
    assert port.sim.pin_state(6) == ("PULSING", arm_sim.degCToUs(11000))


# ===========================================================================
# GOTCHA -- SET on a disabled joint is fiction
# ===========================================================================

def test_set_on_a_disabled_joint_is_fiction(port):
    """The firmware seeds every joint to 90 at boot. J0's servo is DEAD and has
    never been driven this session, and STA still reports SET=90 for it.
    Validity keys off EN=."""
    snap = sta(port)
    for jid in (0, 1, 3, 4, 5, 6):
        assert snap[jid]["EN"] == "0"
        assert snap[jid]["SET"] == "90"
        assert snap[jid]["TGT"] == "90"
        assert snap[jid]["MOV"] == "0"
    assert all(port.sim.pin_state(pin) == ("LOW", None) for pin in ALL_PINS)

    # It stays fiction after a LIM tidies it into a new envelope: SET moves to
    # 70 with nothing attached and no pulse anywhere.
    assert one(port, "LIM 6 10 70 1") == "OK LIM J6 MIN=10 MAX=70 CAL=1"
    assert sta(port)[6]["SET"] == "70"
    assert port.sim.pin_state(11) == ("LOW", None)


# ===========================================================================
# GOTCHA -- the watchdog, and what does and does not feed it
# ===========================================================================

def test_watchdog_trips_after_silence_and_detaches_everything(port):
    one(port, "WDG 1000")
    one(port, "ENA 3 90")

    port.sim.advance(1000)          # the comparison is STRICTLY greater
    assert port.in_waiting == 0, "the watchdog fired one millisecond early"

    port.sim.advance(1)
    volunteered = port.sim.read_out().decode("latin-1").splitlines()
    assert volunteered[0] == "EVT WDOG MS=1001"
    # DOC/FIRMWARE DISAGREEMENT 2. Section 6 enumerates the asynchronous lines
    # a host can see and lists only `EVT ESTOP SRC=CMD` and `EVT ESTOP SRC=RT`.
    # estopAll() emits its own EVT with whatever source it was given, so a
    # watchdog trip produces a SECOND, undocumented line.
    assert volunteered[1] == "EVT ESTOP SRC=WDG"

    snap = sta(port)
    assert snap["SYS"]["ES"] == "1"
    assert snap["SYS"]["WD"] == "1"     # the latch came from the watchdog
    for jid in (0, 1, 3, 4, 5, 6):
        assert snap[jid]["EN"] == "0"
    assert all(port.sim.pin_state(pin) == ("LOW", None) for pin in ALL_PINS)


def test_watchdog_only_arms_when_something_is_actually_driven(port):
    """A human at the Serial Monitor with nothing enabled is never tripped by a
    watchdog they did not ask for."""
    one(port, "WDG 1000")
    port.sim.advance(10_000)
    assert port.in_waiting == 0
    assert sta(port)["SYS"]["ES"] == "0"


def test_png_feeds_the_watchdog(port):
    one(port, "WDG 1000")
    one(port, "ENA 3 90")
    for _ in range(8):
        port.sim.advance(900)
        assert one(port, "PNG").startswith("OK PNG UP=")
    assert sta(port)["SYS"]["ES"] == "0"
    assert sta(port)[3]["EN"] == "1"


def test_raw_bytes_do_not_feed_the_watchdog(port):
    """`?` returns status and the host is still declared dead on schedule.
    HEARTBEAT WITH PNG, NOT WITH `?`."""
    one(port, "WDG 1000")
    one(port, "ENA 3 90")
    port.sim.advance(900)

    port.write(b"?")
    assert read_reply(port)[-1] == "OK STA N=6"   # status came back...

    port.sim.advance(200)                          # ...and it died anyway
    assert sta(port)["SYS"]["WD"] == "1"


def test_an_overlong_line_does_not_feed_the_watchdog(port):
    """It is rejected BEFORE dispatch, so it never counts as the host speaking
    the protocol."""
    one(port, "WDG 1000")
    one(port, "ENA 3 90")
    port.sim.advance(900)
    assert one(port, "X" * 60) == "ERR E8 LINE"
    port.sim.advance(200)
    assert sta(port)["SYS"]["WD"] == "1"


def test_a_refused_but_wellformed_command_does_feed_the_watchdog(port):
    """The line must be WELL-FORMED, not ACCEPTED. A host that is demonstrably
    alive and speaking the protocol is the only question the watchdog asks --
    even an unknown three-letter verb counts."""
    one(port, "WDG 1000")
    one(port, "ENA 3 90")
    for refused in ("MOV 4 100", "SPD 3 500", "FOO"):
        port.sim.advance(900)
        assert one(port, refused).startswith("ERR")
    port.sim.advance(900)
    assert sta(port)["SYS"]["ES"] == "0"
    assert sta(port)[3]["EN"] == "1"


def test_a_verb_that_is_not_three_letters_does_not_feed_the_watchdog(port):
    """The other side of the same boundary: E1 raised by the LINE handler
    (wrong verb length) never reaches dispatch, so it never feeds."""
    one(port, "WDG 1000")
    one(port, "ENA 3 90")
    port.sim.advance(900)
    assert one(port, "MOOV 3 90") == "ERR E1 VERB TOKEN=MOOV"
    port.sim.advance(200)
    assert sta(port)["SYS"]["WD"] == "1"


def test_png_feeds_the_watchdog_but_never_clears_a_latch(port):
    """This is how the arm sat detached for minutes while the log looked fine:
    the heartbeat kept succeeding and nothing checked ES/WD."""
    one(port, "WDG 1000")
    one(port, "ENA 3 90")
    port.sim.advance(1500)
    assert sta(port)["SYS"]["ES"] == "1"

    for _ in range(10):
        assert one(port, "PNG").startswith("OK PNG UP=")
    snap = sta(port)
    assert snap["SYS"]["ES"] == "1"     # still latched
    assert snap["SYS"]["WD"] == "1"
    assert snap[3]["EN"] == "0"         # still detached
    assert one(port, "MOV 3 100") == "ERR E7 MOV JOINT=3"

    # Only CLR clears it, and recovery still needs a fresh adopt angle.
    assert one(port, "CLR") == "OK CLR"
    assert sta(port)["SYS"]["ES"] == "0"
    assert one(port, "ENA 3 88") == "OK ENA J3 ADOPT=88"


def test_a_latched_board_answers_sta_cheerfully_while_every_joint_is_detached(port):
    one(port, "MIR INV 0")
    one(port, "WDG 1000")
    for jid in (0, 1, 3, 4, 5, 6):
        assert one(port, f"ENA {jid} 90") == f"OK ENA J{jid} ADOPT=90"
    port.sim.advance(1500)

    lines = cmd(port, "STA")
    assert lines[-1] == "OK STA N=6"          # a perfectly healthy-looking reply
    assert len([ln for ln in lines if ln.startswith("STA J")]) == 6
    snap = sta(port)
    assert all(snap[jid]["EN"] == "0" for jid in (0, 1, 3, 4, 5, 6))
    assert snap["SYS"]["ES"] == "1"
    # Every joint still reports a SET. None of it means anything -- the arm has
    # been de-energised and a gravity-loaded arm sags while it is.
    assert all(snap[jid]["SET"] == "90" for jid in (0, 1, 3, 4, 5, 6))


def test_the_latch_gates_only_ena_mov_and_jog(port):
    """DOC/FIRMWARE DISAGREEMENT 3. Section 7's E7 row reads globally -- "the
    e-stop / watchdog latch is set, send CLR first". In the .ino `estopLatched`
    is read by exactly three handlers. Everything else answers OK while the
    board is latched, which is what makes a connect-time state push work
    against a board that latched before the host arrived."""
    one(port, "ENA 3 90")
    port.write(b"!")
    read_reply(port)
    assert sta(port)["SYS"]["ES"] == "1"

    # Gated -- these three, and only these three.
    assert one(port, "ENA 3 90") == "ERR E7 ENA JOINT=3"
    assert one(port, "MOV 3 90") == "ERR E7 MOV JOINT=3"
    assert one(port, "JOG 3 1") == "ERR E7 JOG JOINT=3"

    # NOT gated. Every one of these answers OK on a latched board.
    assert one(port, "STP") == "OK STP"
    assert one(port, "LIM 3 60 120 0") == "OK LIM J3 MIN=60 MAX=120 CAL=0"
    assert one(port, "SPD 3 45") == "OK SPD J3 DPS=45"
    assert one(port, "WDG 2000") == "OK WDG MS=2000"
    assert one(port, "MIR SAME") == "OK MIR MODE=SAME OFF=0"
    assert one(port, "DIS 3") == "OK DIS J3"
    assert one(port, "DIS A") == "OK DIS ALL"
    assert one(port, "PNG").startswith("OK PNG UP=")
    assert cmd(port, "STA")[-1] == "OK STA N=6"
    assert one(port, "HLP") == "OK HLP"
    assert one(port, "VER").startswith("OK VER")
    assert one(port, "EST") == "OK EST"          # re-latching is allowed

    # And STP on one joint answers E6, not E7 -- the latch detached it, so the
    # complaint is "not enabled". A host mapping E7 to "press CLR" gets no such
    # hint here.
    assert one(port, "STP 3") == "ERR E6 STP JOINT=3"

    assert sta(port)["SYS"]["ES"] == "1"          # never accidentally cleared


# ===========================================================================
# GOTCHA -- motion takes TIME, at the configured deg/s
# ===========================================================================

def test_motion_takes_time_at_the_configured_dps(port):
    """30 deg/s for 100 ms is 3 degrees. Exactly, not approximately: the
    interpolator integrates elapsed time in centidegrees and the 20 ms tick
    divides 100 evenly."""
    one(port, "SPD 3 30")
    one(port, "ENA 3 90")
    one(port, "MOV 3 110")

    port.sim.advance(100)
    assert sta(port)[3]["SET"] == "93"
    port.sim.advance(100)
    assert sta(port)[3]["SET"] == "96"
    port.sim.advance(100)
    assert sta(port)[3]["SET"] == "99"

    # 20 degrees at 30 deg/s is 667 ms of travel, so it is NOT there yet at 600.
    port.sim.advance(300)
    assert sta(port)[3]["MOV"] == "1"
    port.sim.advance(200)
    assert sta(port)[3] == dict(sta(port)[3], SET="110", MOV="0")


def test_a_faster_slew_rate_arrives_sooner(port):
    """Same distance, three speeds, and the ordering is what SPD means."""
    arrivals = {}
    for dps in (10, 30, 90):
        cmd(port, "DIS A")
        one(port, f"SPD 3 {dps}")
        one(port, "ENA 3 70")
        one(port, "MOV 3 110")
        elapsed = 0
        while sta(port)[3]["MOV"] == "1" and elapsed < 10_000:
            port.sim.advance(20)
            elapsed += 20
        arrivals[dps] = elapsed
    assert arrivals[90] < arrivals[30] < arrivals[10]
    # 40 degrees at 10 deg/s is 4 s; the tick quantises upward, never downward.
    assert 4000 <= arrivals[10] <= 4040


def test_speed_above_ninety_is_e12_and_the_bounds_are_inclusive(port):
    assert one(port, "SPD 3 500") == "ERR E12 SPD JOINT=3 REQ=500 MIN=1 MAX=90"
    assert one(port, "SPD 3 91") == "ERR E12 SPD JOINT=3 REQ=91 MIN=1 MAX=90"
    assert one(port, "SPD 3 0") == "ERR E12 SPD JOINT=3 REQ=0 MIN=1 MAX=90"
    assert one(port, "SPD 3 -5") == "ERR E12 SPD JOINT=3 REQ=-5 MIN=1 MAX=90"
    assert one(port, "SPD 3 1") == "OK SPD J3 DPS=1"
    assert one(port, "SPD 3 90") == "OK SPD J3 DPS=90"
    # A refused SPD changed nothing.
    assert one(port, "SPD 3 999") == "ERR E12 SPD JOINT=3 REQ=999 MIN=1 MAX=90"
    assert sta(port)[3]["DPS"] == "90"


def test_a_stalled_loop_cannot_produce_an_unbounded_jump(port):
    """TICK_CAP_MS bounds the elapsed SLICE; MAX_STEP_C bounds the resulting
    STEP, which is the thing that actually moves the arm. At 90 deg/s a 200 ms
    stall would otherwise command 18 degrees out of a single write."""
    one(port, "SPD 3 90")
    one(port, "ENA 3 70")
    one(port, "MOV 3 110")

    # One loop() iteration that saw 200 ms elapse: the step is capped at
    # MAX_STEP_C = 200 centidegrees = 2.00 degrees.
    port.sim.advance(200, step_ms=200)
    assert sta(port)[3]["SET"] == "72"

    # And no legal speed is throttled by that cap in normal running: at 90
    # deg/s a nominal 20 ms tick is 180 centidegrees, which is under it.
    port.sim.advance(20)
    assert sta(port)[3]["SET"] == "74"   # 72.00 + 1.80 -> 73.80 -> rounds to 74


def test_the_interpolator_does_not_make_up_time_lost_to_a_stall(port):
    """REPRODUCED, NOT FIXED. After a capped slice the .ino sets lastTickMs to
    now, so the elapsed time beyond TICK_CAP_MS is discarded rather than
    carried. A stalled board arrives LATE and never catches up -- which is the
    safe direction, and worth knowing before somebody times a move by wall
    clock."""
    one(port, "SPD 3 30")
    one(port, "ENA 3 70")
    one(port, "MOV 3 110")

    port.sim.advance(1000, step_ms=1000)     # one enormous stalled iteration
    # 1000 ms at 30 deg/s would be 30 degrees. The cap allows 2.00.
    assert sta(port)[3]["SET"] == "72"


# ===========================================================================
# GOTCHA -- reserved joint id 2
# ===========================================================================

@pytest.mark.parametrize("line,verb", [
    ("ENA 2 90", "ENA"),
    ("MOV 2 90", "MOV"),
    ("SPD 2 30", "SPD"),
    ("JOG 2 1", "JOG"),
    ("DIS 2", "DIS"),
    ("STP 2", "STP"),
    ("LIM 2 70 110 0", "LIM"),
])
def test_reserved_joint_2_is_never_addressable_by_any_verb(port, line, verb):
    """The gap is not silently skipped -- it is kept and made to fail loudly, so
    it explains itself the first time somebody types `ENA 2`."""
    assert one(port, line) == f"ERR E4 {verb} JOINT=2 RESERVED=shoulder_pair"


def test_only_joint_2_gets_the_reserved_suffix(port):
    assert one(port, "MOV 9 90") == "ERR E4 MOV JOINT=9"
    assert one(port, "MOV -1 90") == "ERR E4 MOV JOINT=-1"
    assert one(port, "MOV 7 90") == "ERR E4 MOV JOINT=7"


# ===========================================================================
# GOTCHA -- MIR must come after joint 1's LIM, and while joint 1 is disabled
# ===========================================================================

def test_mir_is_refused_while_joint1_is_enabled(port):
    one(port, "MIR SAME")
    one(port, "ENA 1 90")
    assert one(port, "MIR INV 0") == "ERR E9 MIR JOINT=1 STATE=enabled"
    # The enabled check runs BEFORE the mode word is even looked at, so a
    # nonsense mode on a live joint reports E9 and not E11.
    assert one(port, "MIR SIDEWAYS") == "ERR E9 MIR JOINT=1 STATE=enabled"
    # Nothing changed.
    assert next(ln for ln in cmd(port, "STA") if ln.startswith("SYS ")).count("MIR=SAME") == 1

    assert one(port, "DIS 1") == "OK DIS J1"
    assert one(port, "MIR INV 0") == "OK MIR MODE=INV OFF=0"


def test_mir_is_validated_against_joint1_limits_as_they_stand_right_now(port):
    """SERIAL-PROTOCOL.md section 9 step 6: MIR must come AFTER joint 1's LIM.
    This is that warning made concrete with the arm's real locked range."""
    # At the 70-110 boot default an offset of 1 is comfortably legal.
    assert one(port, "MIR INV 1") == "OK MIR MODE=INV OFF=1"

    # Joint 1 is really locked at 0-91 (joint-limits.csv). Its mirror image at
    # offset 0 is 89-180 -- legal, with ZERO degrees to spare at the top.
    assert one(port, "LIM 1 0 91 1") == "OK LIM J1 MIN=0 MAX=91 CAL=1"
    assert one(port, "MIR INV 0") == "OK MIR MODE=INV OFF=0"
    # So ANY positive offset now pushes the image outside 0-180 and is refused.
    assert one(port, "MIR INV 1") == (
        "ERR E11 MIR OFF=1 MIN=0 MAX=91 MIRROR=out_of_travel"
    )

    # Measure a non-zero offset on this arm and joint 1's range MUST narrow --
    # and note WHICH END. INV flips, so it is joint 1's MIN that mirrors to the
    # top of D5's travel. Trimming MAX does nothing at all here:
    assert one(port, "LIM 1 0 85 1") == "OK LIM J1 MIN=0 MAX=85 CAL=1"
    assert one(port, "MIR INV 1") == (
        "ERR E11 MIR OFF=1 MIN=0 MAX=85 MIRROR=out_of_travel"
    )
    # Raising MIN by the same 2 degrees the offset costs is what buys it back.
    assert one(port, "LIM 1 2 91 1") == "OK LIM J1 MIN=2 MAX=91 CAL=1"
    assert one(port, "MIR INV 1") == "OK MIR MODE=INV OFF=1"


def test_pushing_mir_before_lim_leaves_an_offset_nobody_checked(port):
    """The failure section 9 step 6 says not to 'optimise' into existence.

    MIR first is validated against the 70-110 DEFAULT and accepted. LIM then
    widens joint 1 underneath it, and the offset is now sitting on an envelope
    it was never checked against. The firmware does not re-check; the mirror
    arithmetic silently CLAMPS at the point of write instead, and the two
    shoulder servos are commanded into opposition with nothing on the wire
    saying so.
    """
    assert one(port, "MIR INV 20") == "OK MIR MODE=INV OFF=20"   # ok at 70-110
    assert one(port, "LIM 1 0 91 1") == "OK LIM J1 MIN=0 MAX=91 CAL=1"

    # The image of joint 1's new minimum is 18000 + 4000 - 0 = 22000 cd, well
    # outside 0..18000. Enable at that minimum and watch it clamp.
    assert one(port, "ENA 1 0") == "OK ENA J1 ADOPT=0"
    assert port.sim.pin_state(4) == ("PULSING", arm_sim.degCToUs(0))       # 544 us
    assert port.sim.pin_state(5) == ("PULSING", arm_sim.degCToUs(18000))   # 2400 us
    # Two MG996Rs, one link, commanded to opposite ends. NOTHING on the wire
    # reports this -- D5 does not appear in STA at all. Push LIM first.
    assert not any("JOINT=1" in ln for ln in cmd(port, "STA"))


# ===========================================================================
# The full error table of section 7
# ===========================================================================

def test_e1_unknown_or_malformed_verb(port):
    assert one(port, "FOO") == "ERR E1 VERB TOKEN=FOO"
    assert one(port, "foo") == "ERR E1 VERB TOKEN=FOO"      # echoed uppercase
    assert one(port, "MOOV 3 90") == "ERR E1 VERB TOKEN=MOOV"
    assert one(port, "AB") == "ERR E1 VERB TOKEN=AB"
    assert one(port, "M0V 3 90") == "ERR E1 VERB TOKEN=M0V"  # a digit is not A-Z


def test_e2_wrong_number_of_arguments(port):
    assert one(port, "ENA") == "ERR E2 ENA N=0"
    assert one(port, "ENA 3") == "ERR E2 ENA N=1"
    assert one(port, "ENA 3 90 1") == "ERR E2 ENA N=3"
    assert one(port, "WDG") == "ERR E2 WDG N=0"
    assert one(port, "MIR") == "ERR E2 MIR N=0"
    assert one(port, "MIR INV 0 0") == "ERR E2 MIR N=3"
    assert one(port, "DIS") == "ERR E2 DIS N=0"
    assert one(port, "STP 3 4") == "ERR E2 STP N=2"


def test_e3_argument_is_not_an_integer_one_based(port):
    assert one(port, "MOV x 90") == "ERR E3 MOV ARG=1"
    assert one(port, "MOV 3 90abc") == "ERR E3 MOV ARG=2"
    assert one(port, "MOV 3 -") == "ERR E3 MOV ARG=2"
    assert one(port, "MOV 3 +90") == "ERR E3 MOV ARG=2"      # no leading plus
    assert one(port, "MOV 3 9.5") == "ERR E3 MOV ARG=2"      # no floats, anywhere
    assert one(port, "MOV 3 999999") == "ERR E3 MOV ARG=2"   # overflow guard
    assert one(port, "LIM 3 70 110 x") == "ERR E3 LIM ARG=4"
    assert one(port, "DIS B") == "ERR E3 DIS ARG=1"          # only 'A' is special
    # The integer parser runs BEFORE the joint id is resolved, so a bad number
    # on a reserved joint reports E3 and not E4.
    assert one(port, "LIM 2 x 110 0") == "ERR E3 LIM ARG=2"


def test_e4_bad_joint_id(port):
    assert one(port, "ENA 9 90") == "ERR E4 ENA JOINT=9"
    assert one(port, "ENA 2 90") == "ERR E4 ENA JOINT=2 RESERVED=shoulder_pair"


def test_e5_adopt_angle_outside_this_joints_travel(port):
    assert one(port, "ENA 3 140") == "ERR E5 ENA JOINT=3 REQ=140 MIN=70 MAX=110"
    assert one(port, "ENA 3 69") == "ERR E5 ENA JOINT=3 REQ=69 MIN=70 MAX=110"
    assert one(port, "ENA 3 70") == "OK ENA J3 ADOPT=70"     # inclusive


def test_e5_is_still_overloaded_for_a_bad_watchdog_timeout(port):
    """KNOWN OVERLOAD, and the doc says so. Every other operand-range failure
    got its own code; WDG kept E5. Treat E5 on a verb other than ENA as "that
    number was out of range", never as anything to do with joint travel. Note
    it carries NO detail keys at all, so a host cannot even show the operand."""
    assert one(port, "WDG 50") == "ERR E5 WDG"
    assert one(port, "WDG 199") == "ERR E5 WDG"
    assert one(port, "WDG 10001") == "ERR E5 WDG"
    assert one(port, "WDG -1") == "ERR E5 WDG"
    assert one(port, "WDG 200") == "OK WDG MS=200"
    assert one(port, "WDG 10000") == "OK WDG MS=10000"
    assert one(port, "WDG 0") == "OK WDG MS=0"               # 0 = off, the default


def test_e6_wrong_state_for_this_verb(port):
    assert one(port, "MOV 3 90") == "ERR E6 MOV JOINT=3"     # disabled
    assert one(port, "JOG 3 1") == "ERR E6 JOG JOINT=3"      # disabled
    assert one(port, "STP 3") == "ERR E6 STP JOINT=3"        # disabled
    assert one(port, "ENA 3 90") == "OK ENA J3 ADOPT=90"
    assert one(port, "ENA 3 90") == "ERR E6 ENA JOINT=3"     # already enabled
    # DIS has no state check at all: disabling a disabled joint is fine.
    assert one(port, "DIS 4") == "OK DIS J4"


def test_e7_the_latch(port):
    one(port, "ENA 3 90")
    one(port, "EST")
    assert one(port, "MOV 3 90") == "ERR E7 MOV JOINT=3"
    assert one(port, "ENA 3 90") == "ERR E7 ENA JOINT=3"
    assert one(port, "JOG 3 1") == "ERR E7 JOG JOINT=3"


def test_e8_the_one_error_that_echoes_no_verb(port):
    """The line was refused BEFORE it was read to the end, so there is no verb
    in it that could be trusted. 47 content characters plus the terminator is
    the documented 48-character maximum and is accepted."""
    assert one(port, "PNG" + " 1" * 30) == "ERR E8 LINE"

    ok_line = "MOV 3 " + "9" * 41            # 47 characters
    assert len(ok_line) == 47
    assert one(port, ok_line) == "ERR E3 MOV ARG=2"   # parsed, then refused

    too_long = "MOV 3 " + "9" * 42           # 48 characters
    assert len(too_long) == 48
    assert one(port, too_long) == "ERR E8 LINE"


def test_e9_lim_and_mir_will_not_move_the_goalposts(port):
    one(port, "ENA 3 90")
    # A range that still contains the joint's own commanded value is accepted.
    assert one(port, "LIM 3 60 120 0") == "OK LIM J3 MIN=60 MAX=120 CAL=0"
    # A range that would EXCLUDE it is refused -- applying it would turn a
    # limit edit into an unrequested move.
    assert one(port, "LIM 3 100 120 0") == "ERR E9 LIM JOINT=3 STATE=enabled"
    assert sta(port)[3]["MIN"] == "60"      # and nothing was written

    # STATE=jogging is the same code with a different remedy: let go of the
    # control, rather than disable the joint.
    assert one(port, "JOG 3 1") == "OK JOG J3 DIR=1"
    assert one(port, "LIM 3 70 100 0") == "ERR E9 LIM JOINT=3 STATE=jogging"
    assert sta(port)[3]["MIN"] == "60"


def test_a_limit_edit_can_never_create_travel(port):
    """Section 3: a pending target outside the new range is clamped INWARD.

    Clamping can only make a move SHORTER. That is the whole safety argument for
    accepting a LIM on a driven joint at all -- the operator tidies a number and
    a loaded arm must not swing further than it was already going to.
    """
    one(port, "LIM 3 60 120 0")
    one(port, "SPD 3 1")               # slow, so the move is still in flight
    one(port, "ENA 3 90")
    one(port, "MOV 3 120")
    assert sta(port)[3]["TGT"] == "120"

    assert one(port, "LIM 3 60 100 0") == "OK LIM J3 MIN=60 MAX=100 CAL=0"
    snap = sta(port)
    assert snap[3]["TGT"] == "100"     # pulled IN, never pushed out
    assert snap[3]["MOV"] == "1"       # still travelling, just less far
    assert int(snap[3]["SET"]) < 100

    # And the joint stops where the edited limit says, not where the MOV asked.
    port.sim.advance(20_000)
    assert sta(port)[3]["SET"] == "100"
    assert sta(port)[3]["MOV"] == "0"


def test_e10_the_lim_operands_themselves_are_illegal(port):
    assert one(port, "LIM 3 120 40 0") == (
        "ERR E10 LIM JOINT=3 REQMIN=120 REQMAX=40 LIMIT=0..180 MIN<MAX"
    )
    assert one(port, "LIM 3 90 90 0") == (
        "ERR E10 LIM JOINT=3 REQMIN=90 REQMAX=90 LIMIT=0..180 MIN<MAX"
    )
    assert one(port, "LIM 3 -5 110 0") == (
        "ERR E10 LIM JOINT=3 REQMIN=-5 REQMAX=110 LIMIT=0..180 MIN<MAX"
    )
    assert one(port, "LIM 3 70 200 0") == (
        "ERR E10 LIM JOINT=3 REQMIN=70 REQMAX=200 LIMIT=0..180 MIN<MAX"
    )
    assert one(port, "LIM 3 90 93 0") == (
        "ERR E10 LIM JOINT=3 REQMIN=90 REQMAX=93 MINSPAN=5"
    )
    assert one(port, "LIM 3 90 95 0") == "OK LIM J3 MIN=90 MAX=95 CAL=0"   # 5 is legal
    assert one(port, "LIM 3 70 110 7") == "ERR E10 LIM JOINT=3 REQCAL=7"
    # LIM is ATOMIC: not one of those rejections wrote a field.
    assert "LIM J3 MIN=90 MAX=95 CAL=0" in cmd(port, "LIM")


def test_e11_the_mir_mode_word_or_offset_is_illegal(port):
    assert one(port, "MIR BACKWARDS") == "ERR E11 MIR MODE=BACKWARDS"
    assert one(port, "MIR INVERTED") == "ERR E11 MIR MODE=INVERTED"  # wire says INV
    assert one(port, "MIR INV 200") == "ERR E11 MIR OFF=200 LIMIT=+/-90"
    assert one(port, "MIR INV -200") == "ERR E11 MIR OFF=-200 LIMIT=+/-90"
    assert one(port, "MIR INV 40") == (
        "ERR E11 MIR OFF=40 MIN=70 MAX=110 MIRROR=out_of_travel"
    )
    assert one(port, "MIR SAME 40") == "OK MIR MODE=SAME OFF=40"   # SAME ignores it

    # The +/-90 bound is a STORE bound and it is checked BEFORE the envelope,
    # which is the only way to see it: at any real joint-1 range an INV offset
    # anywhere near 90 fails the envelope check first. So the inclusive bound
    # is provable only through SAME, which skips that second check.
    assert one(port, "MIR INV 91") == "ERR E11 MIR OFF=91 LIMIT=+/-90"
    assert one(port, "MIR INV 90") == (
        "ERR E11 MIR OFF=90 MIN=70 MAX=110 MIRROR=out_of_travel"
    )
    assert one(port, "MIR SAME 90") == "OK MIR MODE=SAME OFF=90"    # inclusive
    assert one(port, "MIR SAME -90") == "OK MIR MODE=SAME OFF=-90"  # inclusive
    assert one(port, "mir same") == "OK MIR MODE=SAME OFF=0"       # case-insensitive


def test_e12_slew_rate_outside_one_to_ninety(port):
    assert one(port, "SPD 3 500") == "ERR E12 SPD JOINT=3 REQ=500 MIN=1 MAX=90"


def test_e13_joint_1_is_locked_until_a_human_decides_the_mirror(port):
    """Getting the mirror relation backwards puts two MG996Rs in opposition
    through one link the instant both are enabled, and strips gears. So there
    is no default."""
    assert one(port, "ENA 1 90") == "ERR E13 ENA JOINT=1 MIR=UNKNOWN"
    assert one(port, "MIR SAME") == "OK MIR MODE=SAME OFF=0"
    assert one(port, "ENA 1 90") == "OK ENA J1 ADOPT=90"
    # SAME means D5 gets D4's angle exactly.
    assert port.sim.pin_state(4) == port.sim.pin_state(5)


def test_e14_jog_direction_outside_minus_one_to_plus_one(port):
    """The direction is checked BEFORE joint state, because a bad direction is
    malformed whatever the joint is doing -- reporting E6 for it would send the
    operator to look at the wrong thing."""
    assert one(port, "JOG 3 5") == "ERR E14 JOG JOINT=3 REQDIR=5"    # disabled
    one(port, "ENA 3 90")
    assert one(port, "JOG 3 5") == "ERR E14 JOG JOINT=3 REQDIR=5"
    assert one(port, "JOG 3 -2") == "ERR E14 JOG JOINT=3 REQDIR=-2"
    assert one(port, "JOG 3 -1") == "OK JOG J3 DIR=-1"
    assert one(port, "JOG 3 0") == "OK JOG J3 DIR=0"
    assert one(port, "JOG 3 1") == "OK JOG J3 DIR=1"


# ===========================================================================
# The jog command-age timeout -- silent, and latched
# ===========================================================================

def test_the_jog_timeout_is_silent_on_the_wire_and_latches_JTO(port):
    """Nothing is emitted when a jog times out. The condition that fires it is,
    by definition, the host having stopped listening."""
    one(port, "ENA 3 90")
    assert one(port, "JOG 3 1") == "OK JOG J3 DIR=1"
    assert sta(port)[3]["TGT"] == "110"     # target is the envelope edge

    port.sim.advance(1000)                  # strictly greater, so not yet
    assert port.in_waiting == 0
    assert sta(port)[3]["JTO"] == "0"

    port.sim.advance(1)
    assert port.in_waiting == 0, "the jog timeout must not emit anything"
    snap = sta(port)
    assert snap[3]["JTO"] == "1"            # the only way to find out
    assert snap[3]["EN"] == "1"             # it HOLDS -- it does not detach
    assert snap[3]["MOV"] == "0"
    assert snap[3]["SET"] == snap[3]["TGT"]

    # Cleared by any deliberate operator action addressed to that joint, never
    # by the passage of time.
    assert one(port, "JOG 3 0") == "OK JOG J3 DIR=0"
    assert sta(port)[3]["JTO"] == "0"


def test_a_refreshed_jog_does_not_time_out(port):
    one(port, "SPD 3 1")           # slow, so it does not simply arrive
    one(port, "ENA 3 90")
    one(port, "JOG 3 1")
    for _ in range(8):
        port.sim.advance(250)
        assert one(port, "JOG 3 1") == "OK JOG J3 DIR=1"
    assert sta(port)[3]["JTO"] == "0"


def test_a_finite_move_never_inherits_the_jog_timeout(port):
    """MOV deliberately does not arm the timer. A finite move must run to
    completion and must never be cut short by a timeout it did not ask for."""
    one(port, "SPD 3 1")
    one(port, "ENA 3 90")
    one(port, "MOV 3 110")
    port.sim.advance(5000)
    snap = sta(port)
    assert snap[3]["JTO"] == "0"
    assert snap[3]["EN"] == "1"

    # And a MOV issued mid-jog clears a latch the jog left behind.
    one(port, "JOG 3 1")
    port.sim.advance(1500)
    assert sta(port)[3]["JTO"] == "1"
    one(port, "MOV 3 95")
    assert sta(port)[3]["JTO"] == "0"


def test_stp_on_one_joint_leaves_the_others_alone(port):
    """With a single joint enabled the two STP forms are indistinguishable;
    with several live, the difference is the whole point."""
    one(port, "LIM 0 60 120 0")
    one(port, "LIM 3 60 120 0")
    one(port, "SPD 0 1")
    one(port, "SPD 3 1")
    one(port, "ENA 0 90")
    one(port, "ENA 3 90")
    one(port, "MOV 0 120")
    one(port, "MOV 3 120")

    assert one(port, "STP 3") == "OK STP J3"
    snap = sta(port)
    assert snap[3]["MOV"] == "0"       # stopped, still driven
    assert snap[3]["EN"] == "1"
    assert snap[0]["MOV"] == "1"       # untouched, still going
    assert snap[0]["TGT"] == "120"

    assert one(port, "STP") == "OK STP"
    assert sta(port)[0]["MOV"] == "0"
    assert sta(port)[0]["EN"] == "1"   # STP is NOT an emergency stop


# ===========================================================================
# Raw single bytes -- intercepted BEFORE line assembly
# ===========================================================================

def test_the_status_byte_is_intercepted_mid_line(port):
    """`?` is read before line assembly, so it cannot queue behind a half-typed
    line -- and the half-typed line survives it."""
    one(port, "ENA 3 90")
    port.write(b"MOV 3 9")          # deliberately unterminated
    assert port.in_waiting == 0

    port.write(b"?")
    status = read_reply(port)
    assert status[-1] == "OK STA N=6"

    port.write(b"0\n")              # ...and the line completes normally
    assert read_reply(port) == ["OK MOV J3 REQ=90 SET=90 CL=0"]


def test_the_estop_byte_is_intercepted_mid_line(port):
    """Latency is bounded by one loop() iteration, which only holds because
    there is no delay() anywhere in the sketch."""
    one(port, "ENA 3 90")
    port.write(b"MOV 3 1")
    port.write(b"!")
    assert read_reply(port) == ["EVT ESTOP SRC=RT", "OK EST"]
    assert all(port.sim.pin_state(pin) == ("LOW", None) for pin in ALL_PINS)

    port.write(b"10\n")             # the queued line lands on a latched board
    assert read_reply(port) == ["ERR E7 MOV JOINT=3"]


def test_a_blank_line_is_silently_ignored_so_crlf_cannot_nak(port):
    port.write(b"\n")
    assert port.in_waiting == 0
    port.write(b"   \n")
    assert port.in_waiting == 0
    port.write(b"\r\n")             # CRLF fires once, not twice
    assert port.in_waiting == 0
    assert one(port, "PNG").startswith("OK PNG UP=")


def test_cr_terminates_a_line_exactly_like_lf(port):
    """The Serial Monitor's line-ending setting is irrelevant, deliberately."""
    port.write(b"VER\r")
    assert read_reply(port)[-1].startswith("OK VER")
    port.write(b"VER\r\n")
    assert read_reply(port)[-1].startswith("OK VER")
    assert port.in_waiting == 0


def test_a_nul_byte_truncates_the_line_the_way_a_c_string_does(port):
    """The .ino parses the buffer as a C string, so a NUL ENDS the line there.

    `loop()` writes `line[lineLen] = 0` and `nextTok()` stops dead at the first
    NUL, so a NUL arriving on the wire -- line noise, a wedged host -- discards
    everything after it rather than living inside a token. Python strings have
    no such rule, which is why this needed asserting: the natural reading gives
    a five-character verb (E1) where the board sees a bare MOV with no
    arguments (E2 N=0).

    Caught by an adversarial review, NOT at the bench. This is a claim about
    what the C would do with that byte; it is not evidence that any NUL has
    ever arrived on this arm's wire.
    """
    port.write(b"MOV\x00 3 95\n")
    assert read_reply(port)[-1] == "ERR E2 MOV N=0"

    # Truncation applies to the verb token itself, not only to the arguments.
    port.write(b"MO\x00V\n")
    assert read_reply(port)[-1] == "ERR E1 VERB TOKEN=MO"

    # A leading NUL makes the whole line blank: silent, no NAK, like any other
    # empty line -- so a stuck-low byte cannot NAK-storm a host.
    port.write(b"\x00MOV 3 95\n")
    assert port.in_waiting == 0


def test_verbs_and_dis_a_are_case_insensitive_and_echoed_uppercase(port):
    assert one(port, "ver").startswith("OK VER ")
    assert one(port, "PnG").startswith("OK PNG ")
    assert one(port, "dis a") == "OK DIS ALL"
    assert one(port, "dis A") == "OK DIS ALL"


def test_hlp_is_all_comment_lines_then_one_terminator(port):
    port.write(b"HLP\n")
    raw = port.sim.read_out().decode("latin-1").splitlines()
    assert raw[-1] == "OK HLP"
    assert all(ln.startswith(";") for ln in raw[:-1])
    assert any("2 is RESERVED" in ln for ln in raw)
    assert any("resend every 250 ms" in ln for ln in raw)
    assert any("no feedback" in ln for ln in raw)


# ===========================================================================
# millis() rollover -- the invariant that is otherwise untestable
# ===========================================================================

def test_the_watchdog_survives_the_millis_rollover():
    """millis() wraps at about 49.7 days. The .ino never writes
    `millis() > last + timeout`; it writes `(uint32_t)(millis() - last) >
    timeout`, which is correct across the wrap. Nobody is going to hold a bench
    session open for seven weeks to check that, so it is checked here."""
    clock = VirtualClock(start_ms=0xFFFFFFFF - 500)
    p = FakeSerial("SIM", 115200, timeout=0.3, sim=ArmSim(clock))
    p.reset_input_buffer()

    one(p, "WDG 1000")
    one(p, "ENA 3 90")

    p.sim.advance(1000)                         # crosses the wrap
    assert p.sim.millis() < 600, "the clock did not actually wrap"
    assert p.in_waiting == 0, "the watchdog tripped early across the rollover"

    p.sim.advance(1)
    assert p.sim.read_out().decode("latin-1").splitlines()[0] == "EVT WDOG MS=1001"


def test_the_jog_timeout_survives_the_millis_rollover():
    clock = VirtualClock(start_ms=0xFFFFFFFF - 200)
    p = FakeSerial("SIM", 115200, timeout=0.3, sim=ArmSim(clock))
    p.reset_input_buffer()

    one(p, "SPD 3 1")
    one(p, "ENA 3 90")
    one(p, "JOG 3 1")
    p.sim.advance(1000)
    assert sta(p)[3]["JTO"] == "0"
    p.sim.advance(1)
    assert sta(p)[3]["JTO"] == "1"


# ===========================================================================
# The pulse-width arithmetic, cross-checked against a third source
# ===========================================================================

def test_degrees_to_microseconds_matches_the_documented_measurement(port):
    """1472 us at 90 degrees is not an arbitrary constant: it is the figure
    V2-SERVO-POWER-AND-WIRING.md section 6 uses to derive the ~0.37 V a healthy
    signal pin reads on a meter (1472/20000 = 7.4 % duty, 7.4 % of 5 V)."""
    assert arm_sim.degCToUs(9000) == 1472
    assert arm_sim.degCToUs(0) == 544        # the library's own floor
    assert arm_sim.degCToUs(18000) == 2400   # and its ceiling
    # Integer only. No floats anywhere, on the wire or under it.
    assert all(isinstance(arm_sim.degCToUs(c), int) for c in (0, 9000, 18000))


def test_angles_on_the_wire_are_always_whole_degrees(port):
    """Centidegrees exist only inside the firmware."""
    one(port, "SPD 3 1")
    one(port, "ENA 3 90")
    one(port, "MOV 3 110")
    for _ in range(20):
        port.sim.advance(37)          # deliberately not a multiple of the tick
        snap = sta(port)
        assert snap[3]["SET"].lstrip("-").isdigit()
        assert snap[3]["TGT"].lstrip("-").isdigit()


# ===========================================================================
# FakeSerial -- the surface hold_arm.py actually uses
# ===========================================================================

def test_fake_serial_exposes_the_surface_hold_arm_uses():
    p = FakeSerial("COM5", 115200, timeout=0.3)
    assert p.port == "COM5"
    assert p.baudrate == 115200
    assert p.timeout == 0.3
    assert p.is_open

    assert p.in_waiting > 0                 # the boot banner is sitting there
    p.reset_input_buffer()
    assert p.in_waiting == 0

    assert p.write(b"VER\n") == 4
    p.flush()
    assert p.in_waiting > 0
    chunk = p.read(2)
    assert chunk == b"OK"
    rest = p.read(p.in_waiting)
    assert rest.endswith(b"BUILD=20260801\n")

    assert p.read(64) == b""                # never blocks; nothing is ready
    assert p.readline() == b""

    p.close()
    assert not p.is_open
    p.close()                               # idempotent, like pyserial's


def test_fake_serial_supports_the_deferred_open_arm_bridge_actually_uses():
    """`arm-bridge.py` -- the console's serial bridge, and the host that drives
    the most joints in this repo -- does NOT construct with a port.

    It builds a bare `serial.Serial()`, sets `.port`, `.dtr` and `.rts`, and
    only then calls `.open()` (arm-bridge.py lines 312-326). An earlier version
    of `fake_serial.py` claimed hold_arm.py was the only host that opens the
    port, modelled the board reset at CONSTRUCTION, and had no `open()` at all
    -- so this exact sequence reset the board before `.port` was assigned and
    then raised AttributeError. The reset now hangs off `open()`, where the
    real DTR assertion is.
    """
    p = FakeSerial(port=None, sim=ArmSim(VirtualClock(0)))
    assert not p.is_open                    # no port given: configured, not open
    with pytest.raises(PortClosedError):
        p.write(b"VER\n")                   # and it will not pretend otherwise

    p.port = "COM5"
    p.dtr = True
    p.rts = True
    p.open()
    assert p.is_open
    assert p.in_waiting > 0                 # the reset banner, exactly as DTR

    p.reset_input_buffer()
    p.write(b"VER\n")
    assert p.read(p.in_waiting).startswith(b"OK VER NAME=FACTORYLM-ARM")

    with pytest.raises(PortClosedError):
        p.open()                            # a second open is a second DTR reset

    # Re-opening after a close DOES reset again -- that is the hazard, not a bug.
    p.write(b"LIM 3 10 70 1\n")
    p.reset_input_buffer()
    p.close()
    p.open()
    p.reset_input_buffer()
    p.write(b"LIM\n")
    listing = p.read(p.in_waiting).decode("latin-1")
    assert "LIM J3 MIN=70 MAX=110 CAL=0" in listing   # the push is gone


def test_fake_serial_refuses_a_str_and_refuses_a_closed_port():
    p = FakeSerial()
    with pytest.raises(TypeError):
        p.write("VER\n")                    # pyserial raises here too
    p.close()
    for call in (lambda: p.write(b"VER\n"),
                 lambda: p.read(1),
                 lambda: p.readline(),
                 lambda: p.reset_input_buffer(),
                 lambda: p.in_waiting):
        with pytest.raises(PortClosedError):
            call()


def test_readline_leaves_an_incomplete_line_alone():
    """A truncated line that looks complete is exactly the failure the
    firmware's own E8 handling refuses to allow; the host side should not
    invent one either."""
    p = FakeSerial()
    p.reset_input_buffer()
    partial = b"OK VER NAME=FACT"           # a reply cut off mid-flight
    p.sim.unread(partial)
    assert p.readline() == b""
    assert p.in_waiting == len(partial)     # still all there, untouched

    p.sim.unread(b"ORYLM-ARM\n")            # NOTE: unread pushes to the FRONT
    assert p.readline() == b"ORYLM-ARM\n"
    assert p.readline() == b""              # the fragment is still waiting
    assert p.in_waiting == len(partial)


# ===========================================================================
# The connect handshake of section 9, end to end
# ===========================================================================

def test_the_full_connect_handshake_from_section_9(port):
    """Steps 4 to 7, with this arm's REAL locked limits from joint-limits.csv.

    Note the order: joint 1's LIM goes out long before MIR, which is what makes
    the mirror offset check meaningful. Ascending joint order already does this
    -- section 9 says do not 'optimise' it, and this is why."""
    real_limits = {
        0: (29, 110, 30),    # Base
        1: (0, 91, 30),      # Shoulder pair
        3: (0, 66, 30),      # Elbow
        4: (0, 180, 30),     # Wrist pitch -- locked at the full electrical range
        5: (31, 178, 30),    # Wrist roll
        6: (10, 70, 30),     # Gripper
    }
    assert "NAME=FACTORYLM-ARM" in one(port, "VER")
    for jid, (lo, hi, dps) in real_limits.items():
        assert one(port, f"LIM {jid} {lo} {hi} 1") == f"OK LIM J{jid} MIN={lo} MAX={hi} CAL=1"
        assert one(port, f"SPD {jid} {dps}") == f"OK SPD J{jid} DPS={dps}"
    assert one(port, "MIR INV 0") == "OK MIR MODE=INV OFF=0"
    assert one(port, "WDG 1000") == "OK WDG MS=1000"

    snap = sta(port)
    assert snap["SYS"]["UNCAL"] == "0"
    assert snap["SYS"]["MIR"] == "INV"
    assert snap["SYS"]["WDMS"] == "1000"
    for jid, (lo, hi, _dps) in real_limits.items():
        assert snap[jid]["MIN"] == str(lo)
        assert snap[jid]["MAX"] == str(hi)
        assert snap[jid]["CAL"] == "1"

    # Now the heartbeat keeps it alive indefinitely with joints driven.
    assert one(port, "ENA 6 40") == "OK ENA J6 ADOPT=40"
    for _ in range(20):
        port.sim.advance(250)
        assert one(port, "PNG").startswith("OK PNG UP=")
    assert sta(port)["SYS"]["ES"] == "0"
    assert sta(port)[6]["EN"] == "1"


def test_a_state_push_against_a_board_that_did_not_reset_reports_e9(port):
    """Section 9's "If step 6 returns E9": some transports do not assert DTR
    the same way, so joints may still be live from a previous session at limits
    the host did not set. That must be surfaced, not pushed through."""
    one(port, "ENA 3 90")                   # a joint left live by "the last session"
    assert one(port, "LIM 3 0 66 1") == "ERR E9 LIM JOINT=3 STATE=enabled"
    # The documented remedy.
    assert one(port, "DIS A") == "OK DIS ALL"
    assert one(port, "LIM 3 0 66 1") == "OK LIM J3 MIN=0 MAX=66 CAL=1"
