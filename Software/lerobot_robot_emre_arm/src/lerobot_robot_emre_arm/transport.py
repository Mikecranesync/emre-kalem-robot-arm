"""The serial link to factorylm_arm_controller. One writer, one command in flight.

    !!  NOT YET RUN AGAINST HARDWARE  !!
    Every line here is derived from Documentation/SERIAL-PROTOCOL.md section 9,
    the firmware source, and the shape of Software/tests/protocol_check.py -- but
    the arm cannot currently be assembled (the bench supply is ~700 mA and cannot
    hold it), so none of this has spoken to a board. Treat first bring-up as
    debugging, not verification. Anything below that is a timing claim is wire
    arithmetic from byte counts, never a measurement.

WHY THIS DOES NOT GO THROUGH arm-bridge.py
    The bridge is a working localhost HTTP-to-serial pipe and it would have been
    the obvious thing to reuse. Three reasons not to, in increasing order of how
    load-bearing they are:

      1. `GET /rx` is DESTRUCTIVE. Its drain() returns the buffered lines and
         then clears the deque, so two pollers steal each other's replies -- and
         a LeRobot get_observation() loop is a poller.
      2. Defect C13: the bridge's tx() writes to the port outside its own lock on
         a threading HTTP server, so two concurrent writes can interleave
         mid-line. Already documented in protocol_check.py's own docstring.
      3. The bridge deliberately never generates the heartbeat, on the grounds
         that a proxy outliving its client turns the watchdog into decoration.
         That reasoning is correct and it means the adapter must own PNG anyway
         -- which leaves the bridge with nothing to contribute.

    Consequence, and it is operational rather than technical: THE ARM CONSOLE AND
    THIS ADAPTER ARE MUTUALLY EXCLUSIVE. Close the console tab (and the bridge)
    before `lerobot-record`. On Windows the OS enforces this for us, since COM
    opens are exclusive -- see PortBusy below.

REPLY CORRELATION IS BY ECHOED VERB, AND THERE ARE NO SEQUENCE NUMBERS
    That is the structural reason only one client may transmit: with no
    correlation id, a second writer's replies are indistinguishable from ours.
    Serialising writes would NOT be sufficient; attribution is what breaks.

THE `SET=` COLLISION -- PARSE BY CONTEXT, NOT BY KEY
    In a MOV acknowledgement, `SET=` is the ACCEPTED TARGET after clamping.
    In a STA line, `SET=` is the interpolator's CURRENT commanded output and
    `TGT=` is the goal. Same key, two different quantities. A flat key=value
    parser applied uniformly will silently record targets as positions.
"""

from __future__ import annotations

import logging
import socket
import sys
import threading
import time
from dataclasses import dataclass, field

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover -- environment problem, not a logic error
    sys.stderr.write(
        "pyserial is not installed for this interpreter.\n"
        "Install it with:  python3 -m pip install --user pyserial\n"
    )
    raise

log = logging.getLogger("lerobot_robot_emre_arm.transport")

BAUD = 115200
BOOT_WAIT_S = 2.0        # Optiboot 4.4 holds the bus ~1 s after the DTR reset
REPLY_TIMEOUT_S = 2.0
VER_TRIES = 3
VER_INTERVAL_S = 0.5
FIRMWARE_NAME = "NAME=FACTORYLM-ARM"

#: The bridge binds 127.0.0.1:8770 and nothing else. Probing it is a token-free
#: way to turn a cryptic PermissionError into an instruction.
BRIDGE_PROBE = ("127.0.0.1", 8770)


class PortBusy(RuntimeError):
    """Plain English only -- an operator reads this, never a traceback."""


class LinkError(RuntimeError):
    """The board answered, but not with what the protocol says it should."""


class CommandError(RuntimeError):
    """The board replied ERR E<n>. Carries the parsed reply."""

    def __init__(self, reply: "Reply") -> None:
        super().__init__(reply.terminator)
        self.reply = reply


# ---------------------------------------------------------------------------
# Replies
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Reply:
    """Zero or more DATA lines, then exactly one OK/ERR terminator.

    That is the entire read-loop contract and it has no special cases. `EVT`,
    `RDY` and `;` comment lines are asynchronous and are NOT terminators -- a
    host that treats EVT as one hangs, and a host that discards EVT misses a stop
    it did not initiate.
    """

    verb: str
    data: tuple[str, ...]
    terminator: str
    ok: bool
    events: tuple[str, ...] = field(default=())

    @property
    def error_code(self) -> str | None:
        if self.ok:
            return None
        parts = self.terminator.split()
        return parts[1] if len(parts) > 1 else "E?"

    def fields(self) -> dict[str, str]:
        """key=value pairs from the terminator line only.

        Deliberately does NOT touch the data lines: STA's per-joint lines each
        carry their own SET=/TGT= and folding them together would collide.
        """
        out: dict[str, str] = {}
        for token in self.terminator.split()[1:]:
            if "=" in token:
                k, v = token.split("=", 1)
                out[k] = v
        return out


def parse_sta(reply: Reply) -> dict[str | int, dict[str, str]]:
    """Turn a STA reply into {joint_id: {KEY: value}, 'SYS': {...}}.

    Same shape as protocol_check.py's sta(), so the two agree about what a
    status snapshot looks like.
    """
    out: dict[str | int, dict[str, str]] = {}
    for line in reply.data:
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "STA" and len(parts) > 1 and parts[1].startswith("J"):
            jid = int(parts[1][1:])
            out[jid] = dict(p.split("=", 1) for p in parts[2:] if "=" in p)
        elif parts[0] == "SYS":
            out["SYS"] = dict(p.split("=", 1) for p in parts[1:] if "=" in p)
    return out


# ---------------------------------------------------------------------------
# Port discovery
# ---------------------------------------------------------------------------

def _rank(port) -> int:
    text = ((port.hwid or "") + " " + (port.description or "")).upper()
    if "VID:PID=2341" in text or "VID:PID=2A03" in text:
        return 3   # genuine Arduino
    if "VID:PID=1A86" in text or "VID:PID=0403" in text or "VID:PID=10C4" in text:
        return 2   # CH340 / FTDI / CP210x clone
    if any(w in text for w in ("ARDUINO", "CH340", "USB SERIAL", "USB-SERIAL")):
        return 1
    return 0


def find_port() -> str | None:
    """Best-looking Arduino port, or None when the choice is ambiguous.

    Refusing to guess is the point: picking the wrong port opens some other
    device and, worse, could reach a board running the single-servo bench sketch,
    which reads a bare 'a' as ATTACH.
    """
    ports = list(list_ports.comports())
    if not ports:
        return None
    best = max(_rank(p) for p in ports)
    candidates = [p for p in ports if _rank(p) == best]
    if len(candidates) == 1:
        return candidates[0].device
    if best == 0 and len(ports) == 1:
        return ports[0].device
    return None


def bridge_is_running() -> bool:
    """True when something is listening on the arm bridge's port."""
    s = socket.socket()
    s.settimeout(0.25)
    try:
        s.connect(BRIDGE_PROBE)
        return True
    except OSError:
        return False
    finally:
        s.close()


# ---------------------------------------------------------------------------
# The link
# ---------------------------------------------------------------------------

class SerialLink:
    """Owns the port outright. One transmitter, one command in flight."""

    def __init__(
        self,
        port: str | None = None,
        *,
        watchdog_ms: int = 1000,
        heartbeat_s: float = 0.25,
        idle_disable_s: float = 2.0,
    ) -> None:
        self.port = port
        self.watchdog_ms = watchdog_ms
        self.heartbeat_s = heartbeat_s
        self.idle_disable_s = idle_disable_s

        self._ser: serial.Serial | None = None
        self._io = threading.Lock()          # serialises command round-trips
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []

        # DELIBERATELY NOT UPDATED BY command(). The idle watcher must measure
        # activity of the CONTROL LOOP, not activity on the wire -- and the
        # heartbeat is activity on the wire. Feeding this from command() would
        # let the heartbeat refresh it every 250 ms, so the idle timeout could
        # never elapse and `DIS A` would be unreachable. That would leave the one
        # failure this watcher exists to catch -- a wedged loop in a live process,
        # invisible to the firmware watchdog because the heartbeat keeps it fed --
        # completely uncovered.
        self._last_loop_activity = time.monotonic()
        self._idle_parked = False
        self.firmware_version: str | None = None
        self.events: list[str] = []

    # -- lifecycle ---------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def open(self) -> None:
        """Steps 1-5 of the connect handshake. Pushes no state, enables nothing."""
        if self.is_open:
            return

        if bridge_is_running():
            raise PortBusy(
                "The arm bridge is already running, which means the console owns "
                "the USB port. Close the FactoryLM Arm GUI window (and its browser "
                "tab), then start again. Only one program can talk to the board at "
                "a time -- the wire protocol has no way to tell two clients' "
                "replies apart."
            )

        device = self.port or find_port()
        if device is None:
            raise PortBusy(
                "Could not identify the Arduino's serial port, or more than one "
                "candidate looked equally likely. Pass the port explicitly "
                "(EmreArmConfig.port='COM5' on Windows, '/dev/cu.usbmodem...' on "
                "macOS). Guessing is not safe here: another sketch on another "
                "board would happily attach a servo."
            )

        try:
            # exclusive=True is the POSIX half of single-ownership. On Windows the
            # OS already makes COM opens exclusive, so it is redundant there.
            # UNVERIFIED on macOS/Linux -- confirm TIOCEXCL actually takes effect
            # before relying on it on Bravo or Charlie.
            self._ser = serial.Serial(device, BAUD, timeout=0.4, exclusive=True)
        except PermissionError as exc:
            raise PortBusy(
                f"Could not open {device}. Almost always this means the Arduino "
                "IDE's Serial Monitor, the arm console, or another Python session "
                "is still holding the port. Close it and try again. "
                f"({exc})"
            ) from exc
        except OSError as exc:
            raise PortBusy(f"Could not open {device}: {exc}") from exc

        self.port = device

        # Opening the port asserted DTR, which resets a genuine Uno. The firmware
        # keeps NOTHING across that reset: every joint detached, limits back to
        # 70-110, CAL=0, MIR=UNKNOWN, watchdog off.
        time.sleep(BOOT_WAIT_S)
        self._ser.reset_input_buffer()

        # Never wait passively for the RDY banner -- DTR is handled differently by
        # every transport, so a banner-waiting handler works on one and hangs on
        # another. Always ask.
        for attempt in range(VER_TRIES):
            try:
                reply = self._command_locked("VER")
            except (LinkError, CommandError):
                reply = None
            if reply is not None and FIRMWARE_NAME in reply.terminator:
                self.firmware_version = reply.fields().get("FW")
                break
            if attempt < VER_TRIES - 1:
                time.sleep(VER_INTERVAL_S)
        else:
            found = reply.terminator if reply is not None else "(no reply)"
            self.close()
            raise LinkError(
                f"The board on {device} did not identify as {FIRMWARE_NAME}. "
                f"It said: {found!r}. Refusing to send anything else. At least "
                "three sketches could be on this board, and the single-servo "
                "bench sketch reads a bare 'a' as ATTACH and a bare digit as "
                "'select this pin' -- sending blind would eventually drive the "
                "wrong firmware into attaching a servo."
            )

    def close(self) -> None:
        self.stop_background()
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    # -- commands ----------------------------------------------------------

    def note_loop_activity(self) -> None:
        """Called by the CONTROL LOOP -- get_observation() and send_action() only.

        Never called from command(), and never from the heartbeat. See the
        comment on `_last_loop_activity`.
        """
        self._last_loop_activity = time.monotonic()
        if self._idle_parked:
            # The loop is running again. The joints are detached and the arm has
            # almost certainly sagged, so re-enabling is the caller's decision and
            # needs a FRESH adopt angle -- clearing this flag only stops the
            # watcher from issuing DIS A over and over at a detached arm.
            log.warning(
                "control loop resumed after an idle park. Every joint is "
                "DETACHED and the arm has probably sagged; nothing is driven "
                "until you ENA each joint with a newly estimated adopt angle."
            )
            self._idle_parked = False

    def command(self, line: str, timeout_s: float = REPLY_TIMEOUT_S) -> Reply:
        """Send one line, read to its terminator. Raises CommandError on ERR."""
        with self._io:
            reply = self._command_locked(line, timeout_s)
        if not reply.ok:
            raise CommandError(reply)
        return reply

    def try_command(self, line: str, timeout_s: float = REPLY_TIMEOUT_S) -> Reply:
        """Like command(), but returns the ERR reply instead of raising."""
        try:
            return self.command(line, timeout_s)
        except CommandError as exc:
            return exc.reply

    def _command_locked(self, line: str, timeout_s: float = REPLY_TIMEOUT_S) -> Reply:
        if self._ser is None:
            raise LinkError("the serial port is not open")

        verb = line.split()[0] if line.split() else ""
        self._ser.reset_input_buffer()
        self._ser.write((line + "\n").encode("ascii"))
        self._ser.flush()

        data: list[str] = []
        events: list[str] = []
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            raw = self._ser.readline()
            if not raw:
                continue
            text = raw.decode("ascii", errors="replace").strip()
            if not text or text.startswith(";"):
                continue          # comment / banner noise

            if text.startswith("EVT") or text.startswith("RDY"):
                # Asynchronous. NOT a terminator -- a host that treats EVT as one
                # hangs. Captured and surfaced rather than dropped.
                #
                # BUT ONLY IF IT ARRIVES DURING A ROUND TRIP. The
                # reset_input_buffer() above discards anything sitting in the
                # buffer between commands, and with a 250 ms heartbeat that
                # window is short. So EVT is NOT a reliable way to learn about a
                # stop we did not initiate. The reliable way is the latch on
                # STA's SYS row (ES=, WD=), which EmreArm.get_observation()
                # checks on every single tick.
                events.append(text)
                self.events.append(text)
                log.warning("board event: %s", text)
                continue

            if text.startswith("OK ") or text.startswith("ERR "):
                ok = text.startswith("OK ")
                echoed = text.split()[1] if len(text.split()) > 1 else ""
                if ok and verb and echoed != verb:
                    # Smoke detector, NOT a mutex: two concurrent MOVs both echo
                    # "OK MOV". It only catches cross-VERB interleave. Prevention
                    # is exclusive ownership of the port.
                    raise LinkError(
                        f"sent {verb!r} but the board acknowledged {echoed!r}. "
                        "Another program is almost certainly transmitting on this "
                        "port. Close the arm console and any other session."
                    )
                return Reply(verb=verb, data=tuple(data), terminator=text,
                             ok=ok, events=tuple(events))

            data.append(text)

        raise LinkError(
            f"no terminator for {line!r} within {timeout_s}s. Every command "
            "produces exactly one OK/ERR line; silence means the board reset, the "
            "cable came out, or something else is reading this port."
        )

    # -- background: heartbeat and idle park -------------------------------

    def start_background(self) -> None:
        """Start the heartbeat and the idle watcher.

        WHY A HEARTBEAT THREAD HERE, when arm-bridge.py argues against one:
        there, the bridge is a SEPARATE PROCESS from the browser, so a
        bridge-generated heartbeat would outlive a dead GUI and mask it. Here the
        control loop and this thread share one process, so the thread dies when
        the process dies and the watchdog still catches process death and SIGKILL.

        What a heartbeat alone canNOT catch is a WEDGED loop inside a live
        process -- it would feed the watchdog forever while the arm stayed driven,
        which is the exact failure the watchdog exists to prevent. The idle
        watcher covers that, at the only layer that can see the loop.
        """
        if self._threads:
            return
        self._stop.clear()
        for target, name in ((self._heartbeat_loop, "emre-arm-png"),
                             (self._idle_loop, "emre-arm-idle")):
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()
            self._threads.append(t)

    def stop_background(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2.0)
        self._threads.clear()

    def _heartbeat_loop(self) -> None:
        # PNG, never the realtime '?' byte. The watchdog is fed in exactly one
        # place -- after a complete, terminated line reached a handler. '!' and
        # '?' are intercepted before line assembly and deliberately do NOT feed
        # it, so a host that polls with '?' because it is cheaper gets
        # watchdog-tripped mid-motion by its own heartbeat.
        while not self._stop.wait(self.heartbeat_s):
            if not self.is_open:
                continue
            try:
                self.try_command("PNG", timeout_s=1.0)
            except (LinkError, OSError) as exc:
                log.error("heartbeat failed: %s", exc)

    def _idle_loop(self) -> None:
        while not self._stop.wait(0.25):
            if not self.is_open or self._idle_parked:
                continue
            if time.monotonic() - self._last_loop_activity < self.idle_disable_s:
                continue
            log.error(
                "no get_observation()/send_action() call for %.1fs -- parking the "
                "arm with DIS A. THE ARM WILL SAG: this de-energises every joint. "
                "Recovery needs ENA <j> <FRESH adopt angle> per joint, because "
                "the staleness is mechanical.",
                self.idle_disable_s,
            )
            self._idle_parked = True
            try:
                self.try_command("DIS A")
            except (LinkError, OSError) as exc:
                log.error("idle park failed: %s", exc)

    # -- shutdown ----------------------------------------------------------

    def park_and_close(self) -> None:
        """STP, then DIS A, then close.

        Ordered so motion stops before power is removed, and done while the
        operator is present -- a supervised sag beats letting the watchdog detach
        the arm a second after the process is gone with nobody watching.

        NOT AN EMERGENCY STOP. STP aborts motion and HOLDS with the joints still
        driven; DIS A detaches and a gravity-loaded arm falls. The rocker switch
        and the inline fuse are the only real stop.
        """
        self.stop_background()
        if self.is_open:
            for verb in ("STP", "DIS A"):
                try:
                    self.try_command(verb)
                except (LinkError, OSError) as exc:
                    log.error("%s during shutdown failed: %s", verb, exc)
        self.close()
