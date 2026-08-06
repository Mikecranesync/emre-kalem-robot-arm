#!/usr/bin/env python3
"""
arm-bridge.py  -  the localhost serial bridge for the FactoryLM arm console.

Part of the Emre Kalem 6-axis arm project. Companion files:
    Software/factorylm_arm_controller/factorylm_arm_controller.ino   (the firmware)
    Software/arm-console/arm-console.html                            (the GUI)
    Software/arm-console/joint-limits.csv                            (the limits data)
    Documentation/SERIAL-PROTOCOL.md                                 (the wire spec)
    Documentation/ARM-CONTROLLER-GUIDE.md                            (the plain-English runbook)

WHAT THIS IS
    A dumb pipe. It serves arm-console.html on http://127.0.0.1:8770 and forwards
    bytes to and from the Arduino over USB serial at 115200 8N1. That is the whole
    job. Start the platform launcher ("START ARM GUI.bat" on Windows or
    "START ARM GUI.command" on macOS) and leave its terminal window open while
    you use the arm.

WHAT THIS IS NOT
    It is NOT a safety device. Neither is the Arduino, the firmware, the browser
    E-STOP button, or the serial watchdog. The real emergency stop is the KCD1
    rocker switch and the inline fuse on the servo supply. A browser tab, a USB
    cable, or this Python process can all die in the middle of a move. The only
    thing that reliably stops the arm is removing servo power.

WHY THIS FILE DOES NOT SEND THE HEARTBEAT ITSELF
    The firmware's serial watchdog exists to notice that THE HOST DIED. The GUI
    sends "PNG" every 250 ms on its own timer, and this bridge just carries it.
    If the bridge generated the heartbeat instead, a crashed browser tab, a frozen
    JavaScript loop, or a closed laptop lid would all keep feeding the watchdog
    while the arm stayed driven - which turns the watchdog into decoration.
    Do not "fix" the missing heartbeat here. It is deliberate.

WHY THIS FILE DOES NOT SILENTLY RECONNECT
    Opening the port DTR-resets a genuine Uno, and the firmware keeps NOTHING
    across a reset: every joint goes back to detached, uncalibrated, 70-110.
    A bridge that reconnected behind the GUI's back would leave the GUI showing
    enabled joints and adopted angles for a board that has forgotten all of them.
    So a lost port is reported (GET /rx returns "open": false plus a plain-English
    "error") and the GUI is expected to redo the full connect handshake.

POWER DOCTRINE (unchanged, non-negotiable)
    USB powers the Uno. A separate regulated supply powers the servos.
    ONLY THE GROUNDS JOIN. Never connect external positive to Uno 5V, VIN, or
    the barrel jack.

WHY THE CONSOLE URL HAS A ONE-TIME CODE IN IT
    Binding to 127.0.0.1 keeps this bridge off the network, but it does NOT keep
    it away from software already on this laptop. A browser is happy to send a
    request to 127.0.0.1 from ANY page it has open. Before the code existed, any
    web page in any tab could POST /tx and drive the arm - silently, with no
    prompt, while the operator was reading something else. Loopback is not a
    permission boundary; it only means "not from another machine".

    So at every launch this file mints a fresh random code and prints the console
    URL containing it. A request that cannot show that code is refused. A page on
    another site cannot read the code, because:

      - the code lives in the console page's own URL, and the browser's
        same-origin rule stops foreign pages from reading either that URL or
        anything this bridge returns;
      - responses carry Access-Control-Allow-Origin ONLY for this bridge's own
        loopback origin - never "*";
      - a request whose Origin or Host is anything other than loopback is
        refused outright, which is what blocks the DNS-rebinding trick of
        pointing evil.example at 127.0.0.1.

    The code is per-launch and lives in RAM only. Restart the bridge and every
    old tab is refused until it is reloaded from the new link. That is working
    as intended, not a bug - see the plain-English note in
    Documentation/ARM-CONTROLLER-GUIDE.md section 9.

    THREE WAYS A REQUEST MAY CARRY THE CODE (any one is enough)
      1. ?t=<code> on the request URL          - explicit; what the GUI should do
      2. X-Arm-Token: <code> request header    - for curl and scripts
      3. Referer: http://127.0.0.1:8770/?t=<code>
         The browser attaches this by itself to every fetch the console page
         makes, because the page was served from that URL. It is what lets an
         unmodified arm-console.html keep working. A page from any other site
         gets its referrer stripped to a bare origin, so it can never borrow the
         code this way.

    None of this makes the arm safe. It stops a stray web page from reaching the
    USB port. The rocker switch is still the only real stop.

HTTP API  (see Documentation/SERIAL-PROTOCOL.md for the serial protocol)
    GET  /            -> serves arm-console.html from this same directory.
                         Without a valid code this REDIRECTS to the correct
                         /?t=<code> URL, so a bookmark or a stale tab heals
                         itself on refresh. Every other route below answers
                         403 with a plain-English "error" instead.
    GET  /ports       -> {"ports":[{"device":"COM5","description":"Arduino Uno (COM5)"}]}
    POST /open        {"port":"COM5"}   ("port" may be omitted or "auto" to auto-pick)
    POST /close       {}
    POST /flush       {}                drops every buffered RX line
    POST /tx          {"data":"STA\\n"} written verbatim; a bare "!" is supported
    GET  /rx          -> {"open":true,"lines":["OK STA N=6", ...]}

    Every response is JSON. The bridge NEVER sleeps or flushes on open - the GUI
    owns the 2000 ms post-reset wait and the flush, so the bridge transport and
    the Chrome Web Serial transport behave identically.

    The Web Serial route (opening arm-console.html straight from disk in Chrome
    or Edge) does not use this bridge at all, so none of the above applies to it.

Python 3.11+. Standard library only, plus pyserial.
"""

import collections
import hmac
import json
import os
import secrets
import sys
import threading
import traceback
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ---------------------------------------------------------------------------
# pyserial is the one dependency. Say exactly how to install it; never traceback.
# ---------------------------------------------------------------------------
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.stderr.write(
        "\n"
        "  This bridge needs the 'pyserial' package and it is not installed.\n"
        "\n"
        "  Install it by running the matching line in a terminal:\n"
        "\n"
        "      macOS/Linux: python3 -m pip install --user pyserial\n"
        "      Windows:     py -3 -m pip install --user pyserial\n"
        "\n"
        "  Then run the platform launcher again.\n"
        "\n"
        "  No Python at all? You can skip this bridge entirely: open\n"
        "  Software/arm-console/arm-console.html directly in Google Chrome\n"
        "  or Microsoft Edge and use the built-in Web Serial path instead.\n"
        "\n"
    )
    sys.exit(2)

# ---------------------------------------------------------------------------
# CONFIG
# ---------------------------------------------------------------------------
HOST = "127.0.0.1"          # loopback ONLY. Never 0.0.0.0, never "".
PORT = 8770                 # fixed; the .bat and arm-console.html agree on this
BAUD = 115200               # must match Serial.begin() in the firmware
READ_TIMEOUT = 0.05         # seconds; keeps the reader thread responsive
WRITE_TIMEOUT = 2.0         # seconds; a wedged write must not hang a request
RX_MAXLEN = 2000            # complete lines kept if the GUI stops polling
RX_BUF_LIMIT = 4096         # bytes of a partial line before we give up on it
BODY_MAX = 65536            # largest POST body worth reading; real ones are tiny

HERE = os.path.dirname(os.path.abspath(__file__))
HTML_PATH = os.path.join(HERE, "arm-console.html")
# Served so the console can re-apply the envelope automatically after a connect.
# Opening the port RESETS the Arduino, and the firmware keeps NOTHING across a
# reset - so without this the operator must re-pick the same file by hand every
# single time, and a reset they cannot see reads as "the shoulder stopped
# working". Read-only, and only ever this one fixed filename beside the bridge:
# no part of the request names it, so no request can reach any other file.
LIMITS_PATH = os.path.join(HERE, "joint-limits.csv")

# ---------------------------------------------------------------------------
# THE PER-LAUNCH ACCESS CODE
#
# Minted once, here, in RAM. Never written to disk, never reused across runs.
# The rationale is in the module docstring; the short version is that binding to
# loopback keeps other MACHINES out but does nothing about other PAGES on this
# one, and a page that can POST /tx can drive the arm.
#
# token_urlsafe(18) is 144 bits of os.urandom, URL-safe, ~24 characters - short
# enough to retype off the screen, far beyond guessing at HTTP speeds.
#
# CANONICAL_ORIGIN is the single origin the console is served from. GET / on any
# other accepted loopback spelling (localhost) is redirected here first, so the
# page's own fetches are always same-origin - which is what keeps the Referer
# fallback working and keeps CORS out of the picture entirely.
# ---------------------------------------------------------------------------
TOKEN = secrets.token_urlsafe(18)

CANONICAL_ORIGIN = "http://127.0.0.1:%d" % PORT
CONSOLE_URL = "%s/?t=%s" % (CANONICAL_ORIGIN, TOKEN)

# Both spellings of loopback are trusted. Anything else - including a name that
# merely RESOLVES to 127.0.0.1, which is the DNS-rebinding attack - is not.
ALLOWED_ORIGINS = (CANONICAL_ORIGIN, "http://localhost:%d" % PORT)
ALLOWED_HOSTS = ("127.0.0.1:%d" % PORT, "localhost:%d" % PORT)

# The ONE origin the page is allowed to be served from. "localhost" and "127.0.0.1"
# are different browser origins, so a page served as one and calling the other has
# every API call refused. Both are accepted for API calls (ALLOWED_HOSTS above), but
# the page itself is always redirected to this spelling.
CANONICAL_HOST = "127.0.0.1:%d" % PORT

# USB VID prefixes, best first. Genuine Arduino, then the usual clone chips.
VID_GENUINE = ("VID:PID=2341", "VID:PID=2A03")                   # Arduino LLC / SRL
VID_CLONE = ("VID:PID=1A86", "VID:PID=0403", "VID:PID=10C4")     # CH340, FTDI, CP210x


# ---------------------------------------------------------------------------
# PORT ENUMERATION + AUTO-PICK
#
# serial.tools.list_ports uses SetupAPI, which is the only enumeration that told
# the truth on this machine. PowerShell's [System.IO.Ports.SerialPort]::
# GetPortNames() returned a phantom COM5 from a stale registry entry while the
# board was unplugged. Never use it. Never hardcode COM5 either - the port
# number changes when the board is replugged.
# ---------------------------------------------------------------------------
def list_ports():
    out = []
    for p in serial.tools.list_ports.comports():
        out.append({
            "device": p.device,
            "description": p.description or p.device,
            "hwid": p.hwid or "",
        })
    out.sort(key=lambda d: d["device"])
    return out


def _rank(entry):
    hwid = entry.get("hwid", "").upper()
    desc = entry.get("description", "").upper()
    if any(v in hwid for v in VID_GENUINE):
        return 3
    if any(v in hwid for v in VID_CLONE):
        return 2
    if "ARDUINO" in desc or "CH340" in desc or "USB SERIAL" in desc or "USB-SERIAL" in desc:
        return 1
    return 0


def auto_pick(ports):
    """Return (device, None) or (None, plain-English reason)."""
    if not ports:
        return None, ("No serial ports were found. Plug the Arduino into a USB port "
                      "and press Refresh. If it still does not appear, the cable may "
                      "be charge-only - try a different USB cable.")
    best = max(_rank(p) for p in ports)
    top = [p for p in ports if _rank(p) == best]
    if len(top) == 1:
        return top[0]["device"], None
    if best == 0 and len(ports) == 1:
        return ports[0]["device"], None
    found = ", ".join("%s (%s)" % (p["device"], p["description"]) for p in top)
    return None, ("More than one board looks possible, so nothing was opened. "
                  "Pick the right one from the list yourself. Found: " + found)


# ---------------------------------------------------------------------------
# ERROR TEXT - the operator reads this, not a traceback.
# ---------------------------------------------------------------------------
def friendly_open_error(port, exc):
    text = str(exc)
    low = text.lower()
    if (isinstance(exc, PermissionError) or "access is denied" in low
            or "permission denied" in low or "permissionerror" in low):
        return ("Could not open %s. Almost always this means the Arduino IDE Serial "
                "Monitor is still open and is holding the port. Close it (or close the "
                "whole IDE) and try again. Only one program can hold the port at a time."
                % port)
    if (isinstance(exc, FileNotFoundError) or "filenotfounderror" in low
            or "cannot find the file" in low or "does not exist" in low
            or "no such file" in low):
        return ("Could not open %s. The operating system says there is no such port right now. "
                "Unplug and replug the USB cable, then press Refresh - the port number "
                "changes when the board is replugged." % port)
    return "Could not open %s. The operating system said: %s" % (port, text)


LOST_PORT_MSG = ("The board stopped responding and the port was closed. Usually the USB "
                 "cable came loose, or the Arduino IDE Serial Monitor took the port. "
                 "Reconnect and the console will start over from a fresh handshake.")

NO_TOKEN_MSG = ("This page did not show the access code that the arm bridge printed when "
                "it started, so nothing was sent to the board. Open the console from the "
                "link in the black FactoryLM Arm GUI window, or just press F5 on this "
                "page. A tab left over from an earlier run of the bridge carries an old "
                "code and has to be reloaded.")


# ---------------------------------------------------------------------------
# THE SERIAL LINK
#
# One port, one reader thread. Everything shared is guarded by self._lock.
# A generation counter retires the old reader thread the instant we close or
# reopen, so a stale thread can never push lines into a new session.
# ---------------------------------------------------------------------------
class SerialLink(object):

    def __init__(self):
        self._lock = threading.Lock()
        self._ser = None
        self._gen = 0
        self._lines = collections.deque(maxlen=RX_MAXLEN)
        self._buf = bytearray()
        self._error = None
        self._device = None

    # -- state ------------------------------------------------------------
    def status(self):
        with self._lock:
            return self._ser is not None, self._device, self._error

    # -- open -------------------------------------------------------------
    def open(self, device):
        self.close()
        ser = serial.Serial()
        ser.port = device
        ser.baudrate = BAUD
        ser.bytesize = serial.EIGHTBITS
        ser.parity = serial.PARITY_NONE
        ser.stopbits = serial.STOPBITS_ONE
        ser.timeout = READ_TIMEOUT
        ser.write_timeout = WRITE_TIMEOUT
        # DTR asserted BEFORE open. On a genuine Uno this is what resets the board,
        # which is exactly why the GUI waits 2000 ms and then flushes before it
        # sends its first VER. The bridge deliberately does neither.
        ser.dtr = True
        ser.rts = True
        try:
            ser.open()
        except Exception as exc:
            return False, friendly_open_error(device, exc)

        with self._lock:
            self._gen += 1
            gen = self._gen
            self._ser = ser
            self._device = device
            self._error = None
            self._lines.clear()
            self._buf = bytearray()

        t = threading.Thread(target=self._reader, args=(ser, gen),
                             name="arm-serial-reader", daemon=True)
        t.start()
        return True, None

    # -- close ------------------------------------------------------------
    def close(self):
        with self._lock:
            ser = self._ser
            self._ser = None
            self._device = None
            self._gen += 1          # retires the reader thread
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        return True

    # -- reader thread ----------------------------------------------------
    def _reader(self, ser, gen):
        while True:
            with self._lock:
                if gen != self._gen:
                    return          # we were closed or reopened; stand down
            try:
                waiting = ser.in_waiting
                data = ser.read(waiting if waiting else 1)
            except Exception:
                # A yanked USB cable raises here. If this thread died quietly the
                # GUI would see an empty /rx forever, which looks exactly like a
                # healthy idle board. Report it instead.
                self._fail(gen, LOST_PORT_MSG)
                try:
                    ser.close()
                except Exception:
                    pass
                return
            if not data:
                continue
            with self._lock:
                if gen != self._gen:
                    return
                self._buf.extend(data)
                # Split on newline, keep any incomplete tail buffered. The firmware
                # sends a 7-line STA block back to back, so lines DO straddle reads.
                while True:
                    idx = self._buf.find(b"\n")
                    if idx < 0:
                        break
                    raw = bytes(self._buf[:idx])
                    del self._buf[:idx + 1]
                    self._lines.append(raw.replace(b"\r", b"").decode("ascii", "replace"))
                if len(self._buf) > RX_BUF_LIMIT:
                    # No newline in 4 KB means this is not our firmware (wrong baud,
                    # wrong sketch, or line noise). Drop it rather than grow forever.
                    self._buf = bytearray()

    def _fail(self, gen, message):
        with self._lock:
            if gen != self._gen:
                return
            self._ser = None
            self._device = None
            self._error = message
            self._gen += 1

    # -- transmit ---------------------------------------------------------
    def tx(self, text):
        with self._lock:
            ser = self._ser
            err = self._error
        if ser is None:
            return False, err or "No serial port is open. Connect to the board first."
        try:
            # Verbatim ASCII. Nothing is appended, so a bare "!" (the realtime
            # e-stop byte, which must never queue behind a half-typed line) goes
            # out as exactly one byte. write() blocks until the bytes are gone or
            # write_timeout expires, so no extra flush is needed.
            ser.write(text.encode("ascii", "replace"))
        except Exception as exc:
            # A /tx that reported success while the byte went nowhere is how an
            # operator ends up believing they sent an e-stop. Never do that.
            self.close()
            with self._lock:
                self._error = LOST_PORT_MSG
            return False, "%s (%s)" % (LOST_PORT_MSG, exc)
        return True, None

    # -- receive ----------------------------------------------------------
    def drain(self):
        with self._lock:
            lines = list(self._lines)
            self._lines.clear()
            return (self._ser is not None), lines, self._error

    def flush(self):
        with self._lock:
            ser = self._ser
            self._lines.clear()
            self._buf = bytearray()
        if ser is not None:
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
        return True


LINK = SerialLink()


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------
MISSING_HTML = """<!doctype html>
<meta charset="utf-8"><title>arm-console.html not found</title>
<body style="font:16px system-ui;margin:40px;max-width:44em;color:#222">
<h1>The console page is missing</h1>
<p>The bridge is running fine, but it could not find the page it is supposed to
serve. It looked here:</p>
<pre style="background:#f4f4f5;padding:12px;border-radius:8px">%s</pre>
<p>Put <b>arm-console.html</b> in that folder and press F5.</p>
</body>"""


class BridgeServer(ThreadingHTTPServer):
    # http.server sets allow_reuse_address = 1, and on Windows SO_REUSEADDR lets a
    # SECOND process bind a port that is already in use. Two bridges would then
    # split the GUI's requests unpredictably while only one of them held the serial
    # port - baffling to debug. Turning it off makes the second launch fail loudly.
    allow_reuse_address = False
    daemon_threads = True


class Handler(BaseHTTPRequestHandler):

    protocol_version = "HTTP/1.1"
    server_version = "FactoryLM-ArmBridge/1.0"

    # The GUI polls about 20 times a second. The default access log would bury
    # the startup message within seconds.
    def log_message(self, fmt, *args):
        pass

    # -- helpers ----------------------------------------------------------
    #
    # Access-Control-Allow-Origin is echoed ONLY for this bridge's own loopback
    # origin, and only when the request actually carried one. It used to be "*",
    # which told every browser that any site on the internet was welcome to read
    # this bridge's replies. Never put "*" back.
    #
    # In normal use the console page and this bridge share an origin, so the
    # browser applies no CORS rules at all and these headers are simply unused.
    def _cors(self):
        origin = (self.headers.get("Origin") or "").strip()
        if origin.lower() in ALLOWED_ORIGINS:
            self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, X-Arm-Token")
        self.send_header("Vary", "Origin")
        self.send_header("Cache-Control", "no-store")

    def _send(self, code, body, ctype):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        # "same-origin" keeps the console page's own fetches carrying the full
        # URL - which is the Referer route the access code travels on - while
        # guaranteeing the code is never attached to a request to anywhere else.
        self.send_header("Referrer-Policy", "same-origin")
        self._cors()
        self.end_headers()
        try:
            self.wfile.write(body)
        except Exception:
            pass

    def _json(self, payload, code=200):
        self._send(code, json.dumps(payload), "application/json; charset=utf-8")

    def _body(self):
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            n = 0
        if n <= 0 or n > BODY_MAX:
            # Every legitimate request here is a few dozen bytes. Refusing to
            # even read an absurd one keeps a bad Content-Length from parking
            # this thread on a socket read.
            return {}
        try:
            return json.loads(self.rfile.read(n).decode("utf-8")) or {}
        except Exception:
            return {}

    # -- the access gate --------------------------------------------------
    #
    # Three independent checks, cheapest first. Every route runs behind them.
    # Each one exists for a different attacker:
    #
    #   Host    a page on evil.example whose DNS was rebound to 127.0.0.1. The
    #           packet really does arrive on loopback, so binding to 127.0.0.1
    #           does not stop it - but the browser still sends
    #           "Host: evil.example", and that is what this catches.
    #   Origin  an ordinary cross-site fetch or form POST from another tab.
    #   token   anything that cannot show it came from the console page this
    #           bridge itself served.
    #
    # Each returns a plain-English reason, or None to let the request pass. The
    # text matters: the console shows it verbatim, so it has to read like an
    # explanation and not like a stack trace.

    def _host_reason(self):
        host = (self.headers.get("Host") or "").strip().lower()
        if host in ALLOWED_HOSTS:
            return None
        return ("This request was addressed to '%s'. The arm bridge only answers to "
                "127.0.0.1, which means this computer and nothing else. A request "
                "arriving under any other name is refused." % (host or "no address"))

    def _origin_reason(self):
        # An absent Origin is normal: a same-origin GET does not send one. Only
        # a PRESENT and WRONG one is evidence of anything.
        origin = (self.headers.get("Origin") or "").strip()
        if origin and origin.lower() not in ALLOWED_ORIGINS:
            return ("This request came from %s, which is not the arm console. Only the "
                    "console page served by this bridge is allowed to use it." % origin)
        return None

    # DELIBERATELY NOT APPLIED TO "GET /".
    #
    # This header is a bonus check on the API routes, where it costs nothing: a
    # cross-site caller cannot obtain the access code anyway, so refusing it one
    # step earlier changes no outcome.
    #
    # On the page route it would be a liability. "GET /" needs no code, serves a
    # static page, and is already covered by Host and Origin - and it is reached
    # by a redirect from http://localhost:8770, which is a DIFFERENT SITE from
    # 127.0.0.1. A browser may well label that hop "cross-site", and refusing it
    # would block the one request the operator absolutely must be able to make:
    # opening the console. Never gate the page on this header.
    def _fetch_site_reason(self):
        site = (self.headers.get("Sec-Fetch-Site") or "").strip().lower()
        if site and site not in ("same-origin", "none"):
            return ("Your browser reported that this request came from a different site, "
                    "so it was refused. Only the arm console page served by this bridge "
                    "is allowed to use it.")
        return None

    # Any ONE of these three is enough. See the module docstring for why the
    # Referer route is not a loophole: a foreign page's referrer is stripped to
    # a bare origin, so it can never carry the code.
    def _presented_token(self, query):
        head = (self.headers.get("X-Arm-Token") or "").strip()
        if head:
            return head
        got = urllib.parse.parse_qs(query).get("t")
        if got and got[0]:
            return got[0]
        ref = (self.headers.get("Referer") or "").strip()
        if ref:
            parts = urllib.parse.urlsplit(ref)
            if ("%s://%s" % (parts.scheme, parts.netloc)).lower() in ALLOWED_ORIGINS:
                got = urllib.parse.parse_qs(parts.query).get("t")
                if got and got[0]:
                    return got[0]
        return None

    def _token_ok(self, query):
        got = self._presented_token(query)
        if got is None:
            return False
        # compare_digest, not ==, so the reply time cannot be used to guess the
        # code one character at a time. Compare BYTES: the str form raises
        # TypeError on any non-ASCII character, which would turn a refusal into
        # a 500 and hand an attacker a way to crash a request on purpose.
        return hmac.compare_digest(got.encode("utf-8"), TOKEN.encode("ascii"))

    def _deny(self, why):
        # 403 with a JSON body the console can read out loud. "ports" and "open"
        # are present so the GUI's existing error paths say something true
        # instead of "the bridge is not answering" - it answered, it refused.
        body = json.dumps({"ok": False, "open": False, "ports": [], "error": why}).encode("utf-8")
        self.send_response(403)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        # A refused POST still has its body sitting unread in the socket, and
        # this connection is keep-alive. Closing is the only way to be certain
        # the NEXT request on it is not parsed out of the middle of that body.
        # send_header("Connection", "close") is what tells the stdlib to close.
        self.send_header("Connection", "close")
        self.send_header("Referrer-Policy", "same-origin")
        self._cors()
        self.end_headers()
        try:
            self.wfile.write(body)
        except Exception:
            pass

    def _blocked(self, query, require_token=True):
        """Send the refusal and return True when the request must not proceed."""
        why = self._host_reason() or self._origin_reason()
        if why is None and require_token:
            # Sec-Fetch-Site and the code are both API-route checks only. See
            # _fetch_site_reason for why the page route must not use either.
            why = self._fetch_site_reason()
            if why is None and not self._token_ok(query):
                why = NO_TOKEN_MSG
        if why is None:
            return False
        self._deny(why)
        return True

    def _redirect_to_console(self):
        # Content-Length is explicit because protocol_version is HTTP/1.1 and the
        # connection is kept alive; a bodyless reply without it desynchronises
        # the socket and the browser hangs.
        self.send_response(302)
        self.send_header("Location", CONSOLE_URL)
        self.send_header("Content-Length", "0")
        self.send_header("Referrer-Policy", "same-origin")
        self._cors()
        self.end_headers()

    # -- routes -----------------------------------------------------------
    def do_OPTIONS(self):
        # A preflight cannot carry the code and does not need to: it grants
        # nothing on its own, and the real request behind it is still checked.
        parts = urllib.parse.urlsplit(self.path)
        if self._blocked(parts.query, require_token=False):
            return
        self.send_response(204)
        self._cors()
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_GET(self):
        parts = urllib.parse.urlsplit(self.path)
        path = parts.path.rstrip("/") or "/"

        if path in ("/", "/index.html", "/arm-console.html"):
            # The page itself. Host and Origin still apply, but a missing or
            # stale code REDIRECTS to the right URL rather than refusing: the
            # launcher opens a plain http://127.0.0.1:8770/, and a bookmark or a
            # tab left over from the previous bridge run must heal on refresh
            # instead of showing the operator a dead end.
            #
            # The redirect gives nothing away. A foreign page can read neither
            # the Location header nor the page it points at - both are blocked
            # by the same-origin rule.
            if self._blocked(parts.query, require_token=False):
                return
            # Canonicalise the HOST first, independently of the token.
            #
            # "localhost:8770" and "127.0.0.1:8770" are different browser origins.
            # If we serve the page to the "localhost" spelling just because it
            # happened to carry a valid token, every API call that page then makes
            # is refused by the origin check - and the GUI reports it as "no serial
            # ports found", i.e. a phantom hardware fault. Redirect to the canonical
            # origin BEFORE looking at the token so that can never happen.
            if self.headers.get("Host", "") != CANONICAL_HOST:
                self._redirect_to_console()
                return
            if not self._token_ok(parts.query):
                self._redirect_to_console()
                return
            try:
                with open(HTML_PATH, "rb") as fh:
                    self._send(200, fh.read(), "text/html; charset=utf-8")
            except OSError:
                self._send(404, MISSING_HTML % HTML_PATH, "text/html; charset=utf-8")
            return

        if self._blocked(parts.query):
            return

        if path == "/ports":
            try:
                self._json({"ports": list_ports()})
            except Exception as exc:
                self._json({"ports": [], "error": "Could not list serial ports: %s" % exc})
            return

        if path == "/limits":
            # Text, not a file download: the console runs it through exactly the
            # same parser and validator as the LOAD LIMITS FILE picker, so an
            # edited row is refused here for the same reasons and with the same
            # words. Missing file is reported, never silently treated as empty.
            try:
                with open(LIMITS_PATH, "r", encoding="utf-8") as fh:
                    self._json({"ok": True,
                                "name": os.path.basename(LIMITS_PATH),
                                "csv": fh.read()})
            except OSError as exc:
                self._json({"ok": False,
                            "error": "Could not read %s: %s" % (LIMITS_PATH, exc)})
            return

        if path == "/rx":
            is_open, lines, err = LINK.drain()
            out = {"open": is_open, "lines": lines}
            if err:
                out["error"] = err
            self._json(out)
            return

        self._json({"ok": False, "error": "Unknown path %s" % path}, 404)

    def do_POST(self):
        parts = urllib.parse.urlsplit(self.path)
        path = parts.path.rstrip("/") or "/"
        if self._blocked(parts.query):
            return
        data = self._body()

        if path == "/open":
            device = data.get("port")
            if not isinstance(device, str) or not device.strip() or device.strip().lower() == "auto":
                device, why = auto_pick(list_ports())
                if device is None:
                    self._json({"ok": False, "error": why})
                    return
            device = device.strip()
            ok, err = LINK.open(device)
            self._json({"ok": ok, "port": device} if ok else {"ok": False, "error": err})
            return

        if path == "/close":
            LINK.close()
            self._json({"ok": True})
            return

        if path == "/flush":
            LINK.flush()
            self._json({"ok": True})
            return

        if path == "/tx":
            payload = data.get("data")
            if not isinstance(payload, str):
                self._json({"ok": False, "error": 'Missing "data" string.'})
                return
            ok, err = LINK.tx(payload)
            self._json({"ok": ok} if ok else {"ok": False, "error": err})
            return

        self._json({"ok": False, "error": "Unknown path %s" % path}, 404)


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------
BANNER = """
=========================================================
 FACTORYLM ARM BRIDGE  -  running
=========================================================
 Open the console here:

   %s

 That long code at the end is this session's access code.
 It is new every time the bridge starts, and it stops any
 other web page on this laptop from reaching the arm.
 http://127.0.0.1:8770/ on its own also works - it just
 forwards you to the address above.

 LEAVE THIS WINDOW OPEN while you use the arm console.
 Closing it disconnects the board.

 Close the Arduino IDE Serial Monitor first - only one
 program can hold the USB port at a time.

 The real emergency stop is the rocker switch and the
 inline fuse. This bridge, the Arduino, and the on-screen
 E-STOP button are NOT safety devices.

 Press Ctrl-C here to stop.
---------------------------------------------------------
"""


def main(argv):
    requested = argv[1] if len(argv) > 1 else None

    try:
        httpd = BridgeServer((HOST, PORT), Handler)
    except OSError as exc:
        sys.stderr.write(
            "\n  Could not start on http://%s:%d\n"
            "  %s\n\n"
            "  Almost always this means another ARM CONSOLE window is already\n"
            "  running. Use that one, or close it and try again.\n\n" % (HOST, PORT, exc))
        return 1

    sys.stdout.write(BANNER % CONSOLE_URL)

    ports = []
    try:
        ports = list_ports()
    except Exception as exc:
        sys.stdout.write(" Could not list serial ports: %s\n" % exc)
    if ports:
        sys.stdout.write(" Serial ports found:\n")
        for p in ports:
            sys.stdout.write("   %-6s %s\n" % (p["device"], p["description"]))
    else:
        sys.stdout.write(" No serial ports found yet. Plug the Arduino in and press\n"
                         " Refresh in the console - the list is re-read every time.\n")

    # Optional convenience: "arm-bridge.py COM5" or "arm-bridge.py auto" opens a
    # port at startup. The GUI normally does this itself via POST /open.
    if requested:
        device = requested
        if device.strip().lower() == "auto":
            device, why = auto_pick(ports)
            if device is None:
                sys.stdout.write("\n %s\n" % why)
        if device:
            ok, err = LINK.open(device)
            sys.stdout.write("\n %s\n" % ("Opened %s at %d baud." % (device, BAUD) if ok else err))

    sys.stdout.write("\n")
    sys.stdout.flush()

    try:
        httpd.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    except Exception:
        traceback.print_exc()
        return 1
    finally:
        try:
            httpd.server_close()
        except Exception:
            pass
        LINK.close()
        sys.stdout.write("\n Serial port closed. Bridge stopped.\n"
                         " The board keeps whatever it was last commanded until it is\n"
                         " reset or its power is removed.\n\n")
        sys.stdout.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
