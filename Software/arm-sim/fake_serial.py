"""A duck-typed stand-in for `serial.Serial`, wired to `arm_sim.ArmSim`.

WHY A DUCK TYPE AND NOT A SUBCLASS
    Subclassing `serial.Serial` would make this module import pyserial, and
    pyserial is BSD-3-Clause -- NOT on this project's Apache-2.0/MIT allow-list.
    It is already an unavoidable dependency of `arm-console/arm-bridge.py` and
    `tests/protocol_check.py` and is FLAGGED as an exception in
    `lerobot_robot_emre_arm/pyproject.toml`. There is no reason to spread it
    further: this file is stdlib-only and nothing here needs the real class.
    Following that precedent means flagging, never passing silently.

WHERE THE SURFACE CAME FROM -- read, not guessed
    THREE hosts in this repo open the serial port. An earlier version of this
    docstring said `hold_arm.py` was "the only thing in this repo that opens the
    port and drives joints". THAT WAS FALSE, and it was load-bearing: it was the
    stated justification for how small this surface is. An adversarial review
    caught it. The correction is worth more than the method it added.

    1. `Software/arm-console/hold_arm.py` -- the holder daemon. Uses exactly:

           serial.Serial(PORT, BAUD, timeout=0.3)
           .reset_input_buffer()   .write(bytes)   .flush()
           .in_waiting             .read(n)

    2. `Software/tests/protocol_check.py` -- the firmware's own test harness,
       written earlier against the real board. Constructs the same way and adds
       `.readline()` and `.close()`. Implemented deliberately, not silently: a
       surface that stops one method short of the repo's own firmware harness is
       a surface that gets patched in a hurry later. (It runs: see README
       section 7.)

    3. `Software/arm-console/arm-bridge.py` -- the console's serial bridge, and
       the biggest host here. It DOES open the port and it DOES drive joints:
       `tx()` relays verbatim ASCII from the browser, including `ENA`, `MOV` and
       the bare realtime `!` byte. It uses the DEFERRED-OPEN pattern, which is a
       different shape from the other two:

           ser = serial.Serial()            # constructed with NO port
           ser.port = device                # ...configured...
           ser.dtr = True ; ser.rts = True  # DTR asserted BEFORE open
           ser.open()                       # ...and only NOW does it open

       `open()` is therefore implemented, and the board reset is wired to
       `open()` rather than to construction -- see below.

    `Software/tests/motion_verify.py` CONTRIBUTES NOTHING to this list, and
    that is worth stating rather than leaving as an absence: it never opens a
    serial port at all. It talks to the holder daemon through
    `arm_cmd.txt` / `arm_hold.log`, because the daemon must keep owning COM5 --
    the watchdog latches and every joint detaches the moment nothing feeds it,
    and closing the port DTR-resets the board.

    NOT MODELLED, on purpose: `bytesize`, `parity`, `stopbits` and
    `write_timeout`. arm-bridge.py assigns all four, and Python accepts an
    assignment to any attribute of an ordinary object, so they cost nothing and
    break nothing. There is no wire here for a framing setting to describe.
    Whether they should ever become real is a scope call, not a bug.

OPENING THE PORT RESETS THE BOARD, AND THAT IS THE POINT
    `open()` calls `ArmSim.reset()`, exactly like DTR on a genuine Uno. Limits
    go back to 70-110 with CAL=0, the mirror goes back to UNKNOWN, every joint
    detaches. Pass `sim=` an existing board to prove that against state you
    pushed yourself -- with a fresh `ArmSim` the assertion is a tautology.

    Constructing with a `port` opens immediately, which is pyserial's own rule:
    a port argument means open now, `port=None` means configure now and open
    later. Both routes run the same reset, because on real hardware both end in
    the same DTR assertion. Getting this backwards is not cosmetic -- under the
    old construct-time-only reset, arm-bridge.py's deferred sequence reset the
    board BEFORE `.port` had even been assigned, and then raised AttributeError
    on `.open()`.

THIS NEVER BLOCKS AND NEVER MOVES TIME
    `timeout` is stored because the real constructor takes it, and is otherwise
    ignored: there is no wire, so there is nothing to wait for. `read()` and
    `readline()` return what is buffered right now. Virtual time only moves
    when a test calls `port.sim.advance(...)`. A host loop that relies on
    wall-clock timeouts will therefore busy-spin against this object -- which
    is a limitation, not a bug, and is listed as such in README.md.
"""

from __future__ import annotations

from arm_sim import ArmSim


class PortClosedError(Exception):
    """Raised on I/O after close().

    NOT pyserial's `PortNotOpenError` -- importing pyserial to borrow its
    exception class is the dependency this module exists to avoid. Code that
    catches the pyserial class by name will not catch this one.
    """


class FakeSerial:
    """`serial.Serial`-shaped handle onto a simulated arm controller."""

    def __init__(
        self,
        port: str | None = "SIM",
        baudrate: int = 115200,
        timeout: float | None = None,
        *,
        sim: ArmSim | None = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_open = False

        # DTR/RTS are stored because arm-bridge.py asserts both BEFORE open().
        # dtr is not decoration: asserting it is what resets a genuine Uno, and
        # that reset is the single most consequential thing this class models.
        self.dtr = True
        self.rts = True

        self.sim = sim if sim is not None else ArmSim()

        # pyserial's rule: a port argument means open NOW; port=None means
        # configure now and call open() later.
        if port is not None:
            self.open()

    # -- guards -------------------------------------------------------------

    def _require_open(self) -> None:
        if not self.is_open:
            raise PortClosedError(f"{self.port} is closed")

    # -- open / close -------------------------------------------------------

    def open(self) -> None:
        """Open the port -- WHICH RESETS THE BOARD.

        Anything the previous owner of this port never read is gone, then the
        board resets and writes its banner. The host's 2000 ms wait and flush
        (SERIAL-PROTOCOL.md section 9) is what normally discards that banner;
        this class does NOT model the Optiboot window, so the reset here is
        instant.
        """
        if self.is_open:
            # pyserial raises SerialException("Port is already open.") here.
            # Not silently ignored: a double open on a real board is a second
            # DTR reset, which would silently discard limits, mirror and every
            # enable the host had pushed.
            raise PortClosedError(f"{self.port} is already open")
        if self.port is None:
            raise PortClosedError("no port configured -- set .port before open()")
        self.sim.discard_out()
        self.sim.reset()
        self.is_open = True

    # -- the surface hold_arm.py uses --------------------------------------

    def write(self, data: bytes) -> int:
        """Send bytes to the board. Returns the count, like pyserial."""
        self._require_open()
        if isinstance(data, str):
            # pyserial raises TypeError here too. Worth keeping: a str that
            # silently encoded would hide a real bug in a host.
            raise TypeError("write() takes bytes, not str")
        data = bytes(data)
        self.sim.feed(data)
        return len(data)

    def flush(self) -> None:
        """No-op. Nothing is buffered on the transmit side -- `write()` has
        already reached the simulated firmware by the time it returns."""
        self._require_open()

    def read(self, size: int = 1) -> bytes:
        """Up to `size` bytes. Never blocks; returns b'' when nothing is ready."""
        self._require_open()
        if size <= 0:
            return b""
        return self.sim.read_out(size)

    @property
    def in_waiting(self) -> int:
        self._require_open()
        return self.sim.out_waiting()

    def reset_input_buffer(self) -> None:
        self._require_open()
        self.sim.discard_out()

    def close(self) -> None:
        """Idempotent, like pyserial's.

        On a real board this can toggle DTR and reset the controller, losing
        limits, mirror and every enable. That reset is modelled at OPEN rather
        than here, because whether closing asserts DTR is transport-specific --
        SERIAL-PROTOCOL.md section 9 is explicit that transports differ, and
        section 15 records this as a real hazard. So closing this object stops
        the handle and leaves the simulated board exactly as it was; a
        subsequent open is what resets it.
        """
        self.is_open = False

    # -- one method past that surface, on purpose (see the module docstring) --

    def readline(self) -> bytes:
        """One complete line INCLUDING its terminator, or b'' if none is ready.

        pyserial would return a partial line when its timeout expired. This has
        no timeout to expire, so an incomplete line is left in the buffer
        untouched rather than handed over as if it were finished -- a truncated
        line that looks complete is exactly the failure mode the firmware's own
        E8 handling refuses to allow.
        """
        self._require_open()
        buffered = self.sim.read_out()          # take it all...
        idx = buffered.find(b"\n")
        if idx < 0:
            self.sim.unread(buffered)           # ...and put it all back
            return b""
        line, rest = buffered[:idx + 1], buffered[idx + 1:]
        self.sim.unread(rest)
        return line
