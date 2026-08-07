"""Decide whether a message from a phone is allowed to move a physical arm.

PURE. No files, no sockets, no environment, no clock, no imports outside
`dataclasses`/`typing`. `now` is an argument and the link state is an argument,
because the two things this layer must get right -- a window that expires, and a
daemon that stopped answering -- are precisely the two things that become
untestable the moment the code reads them itself. A suite that waits out a 300 s
TTL in wall-clock seconds is a suite nobody runs (`arm-sim/README.md` section 9).

WHAT THIS LAYER CANNOT KNOW, and never will:

  * Where the arm is. Nothing in this system observes an output shaft. The
    declared pose is the operator's estimate -- the same contract shape as
    `ENA <j> <adopt>`, where the firmware pre-loads a pulse before attaching and
    the number is always a human's guess.
  * Whether a human is at the bench, whether hands are clear of the claw, or
    whether the phone was in somebody's hand or face-down on a table. Arming is a
    deliberate act with a short expiry. It bounds how long a stale intention
    stays valid. It is NOT evidence of a person, and it is a WEAKER substitute
    for `arm-bench-safety` section 2's capture-a-frame-and-look-at-it rule -- not
    an equal one. That rule exists because the operator said "go ahead" and in
    the very next frame his hand was still cupped around the claw.
  * Whether a motor turned. Every outcome here is read off a board line, and a
    board ack proves the firmware accepted a command and nothing more. That cost
    a whole afternoon on D3, where the firmware provably drove a joint at
    29.2 deg/s for 3.5 s and nothing moved.

THE CENTRAL INVERSION -- EXPIRY SENDS NOTHING TO THE ARM.

The instinctive implementation of "the window expired, make it safe" is `EST` or
`DIS`. On this arm those DETACH every joint, and a gravity-loaded arm falls; no
pose in `arm-poses.csv` is self-supporting and a remote user cannot catch a
falling arm. So expiry is a state change in the BOT. `hold_arm.py` goes on doing
exactly what it was doing. `EST`, `DIS` and `CLR` are on a forbidden list that is
enforced in code (`_guard`) and asserted in the tests, and every refusal and
every expiry returns `lines == ()`.

REFUSE, NEVER QUEUE. A motion request outside the window is refused outright, not
buffered. A queued motion command asserts consent that has already expired: 90 s
later the operator may have walked away or put a hand in the claw, and the queue
fires anyway. A refusal costs one retyped message.

THE LINK STATE IS PASSED IN, and is duck-typed on three attributes:

    link.verdict      "LIVE" | "STALE" | "NO LINK"
    link.restarted    bool -- the daemon truncated its log and re-ENA'd
    link.log_window   str  -- log text observed since the last look

Phase 1's `arm_link.observe()` produces that object. This module deliberately
does not import it: policy must stay pure and importable with nothing on disk,
and the two halves meet in the bot (Phase 3). Missing attributes fail CLOSED --
a producer that hands over something malformed reads as `NO LINK` and refuses.

THE CALLER OWNS THE ALLOWLIST. `decide()` has no chat id and cannot check one.
It assumes the allowlist gate has already passed. Calling it before that gate has
run inverts the design (PRD section 8: the allowlist is the OUTERMOST gate,
before arming state, before parsing, before anything).

TWO CHOICES WITH A COST, named as choices rather than dressed up as derivations:

  1. `STALE` and `NO LINK` CLOSE the window, they do not merely refuse the turn.
     PRD section 5 lists "any refusal caused by a stale or dead daemon" as a
     closing event; this reads it strictly. Cost: a transient two-missed-poll
     blip makes the operator re-arm. That is the safe direction.
  2. A failed transition (`CL=1`, `ERR`, or a latch mid-move) voids the declared
     pose AND disarms. PRD section 9b only specifies the SUCCESS case ("the
     declared pose becomes the destination"). A clamp means the joint is not
     where you asked, so after a failure nobody knows where the arm is -- and the
     remedy this project already prescribes for not knowing is a human looking at
     the arm and typing a fresh `/arm <pose>`.

NOTHING IN THIS FILE HAS RUN AGAINST HARDWARE. A green test suite here is
evidence that the decision logic is modelled as written. It is not evidence about
a motor, a shaft, a gear, or an arm.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Iterable, Mapping, Sequence

# --------------------------------------------------------------------------
# Poses and transitions -- LIFTED VERBATIM from Software/tests/cycle_poses.py.
#
# Why lifted and not imported: `cycle_poses.py` imports cv2 and numpy at module
# top level, so importing it would drag OpenCV into a bot whose strongest true
# claim is that it adds zero dependencies. The bench tools are proven and the
# bench is disconnected, so they are not edited to suit this. The copies are held
# honest by a BEHAVIOURAL drift guard in tests/test_policy.py, which imports both
# and asserts the constants are equal -- not textually compared, which would
# false-fail on whitespace.
#
# TO LEAVE A POSE, REVERSE ITS OWN ENTRY PATH (arm-poses.csv, in capitals in the
# source). The endpoints are not the safety property: two poses can both be safe
# while the straight line between them drives the claw through the bench, the
# wire loom or the base housing. So a transition is a SEQUENCE OF PHASES and only
# the joints inside one phase move together. The order is not cosmetic --
# reversing it drops a claw that is still pointing down onto the table, or swings
# a wrist through the loom.
# --------------------------------------------------------------------------
PICK = {1: 8, 3: 36, 4: 140, 5: 165}
STORAGE = {1: 88, 3: 64, 4: 90, 5: 104}
TRANSIT_J1 = 40  # shoulder height used to cross between the two, claw clear of the mat

TO_STORAGE = (("lift", {1: TRANSIT_J1}),
              ("neutralise", {3: 64, 4: 90, 5: 104}),
              ("fold", {1: 88}))
TO_PICK = (("swing_out", {1: TRANSIT_J1}),
           ("aim", {3: 36, 4: 140, 5: 165}),
           ("descend", {1: 8}))

POSES = ("pick", "storage")

#: (declared start pose, requested destination) -> phase sequence.
#: Directional on purpose. There is no entry for a pose to itself, and no
#: interpolation between arbitrary poses exists or may be added here.
_PATHS = {("pick", "storage"): TO_STORAGE,
          ("storage", "pick"): TO_PICK}

# --------------------------------------------------------------------------
# The wire, and what may go on it
# --------------------------------------------------------------------------
#: The ONLY verbs this surface may emit. An allowlist of three beats a denylist
#: against a firmware that has fifteen verbs.
ALLOWED_VERBS = ("MOV", "STA", "STP")

#: The ONLY joints a `MOV` from this surface may name.
#:
#: This is the wire-level half of "no gripper, no J0". The DATA half already
#: holds -- neither recorded transition touches J6 or J0, and
#: `test_no_waypoint_touches_j0_or_the_gripper` pins that. But the data half is
#: the only half there was: `_guard` checked the verb and the line length and
#: nothing else, so a hand-built `Phase` or a future code path could put
#: `MOV 6 70` on the wire and no allowlist would stop it. An allowlist with a
#: hole in it for the joint is not an allowlist.
#:
#: J6 acks and ramps correctly and its fingers do not articulate -- the gear is
#: slipping on the motor shaft -- and `arm-bench-safety` forbids a gripper
#: command with fingers in the claw, which a phone can never rule out. J0's
#: servo is dead and appears in no pose. J2 does not exist on this arm.
#:
#: HARDENING, NOT A BUG THAT WAS FOUND. No path in this package constructs such
#: a line today. It is here so none can.
MOV_JOINTS = (1, 3, 4, 5)

#: Kept as its own list even though ALLOWED_VERBS already excludes them, because
#: this is the safety assertion the plan asks for and it should be greppable.
#: EST and the watchdog DETACH every joint and the arm falls. CLR clears a latch,
#: which is the daemon's job and not a remote user's. Nothing here opens a port.
FORBIDDEN_VERBS = ("EST", "DIS", "CLR")

#: Inbound firmware lines cap at 48 characters INCLUDING the terminator; longer
#: is discarded with `ERR E8 LINE`. So 47 characters of content.
MAX_LINE_CHARS = 47

#: `enable_all()` re-sends `ENA <j> <adopt>`, which snaps joints back to their
#: adopt angles. In a log window that reads exactly like the arm moving on its
#: own, and it means the declared pose is no longer true.
LATCH_MARKERS = ("LATCHED", "re-ENA")

#: The plan ships 300 s because the PRD recommends five minutes. IT IS MIKE'S
#: NUMBER, NOT A TUNING DEFAULT (PRD Q17-Q3, still unanswered). The bot passes
#: `ARM_TELEGRAM_TTL_S`; this module never reads an environment variable.
DEFAULT_TTL_S = 300.0

ACCEPTED_LIST = "/arm <pose>, /disarm, /status, /go storage, /go pick, /stop"

#: Words that name the gripper or the dead base servo. Checked BEFORE the
#: no-numbers rule so that "j6" and "j0" get the reason that explains them rather
#: than the generic number refusal.
_GRIPPER_WORDS = ("grip", "claw", "jaw", "j6")
_J0_WORDS = ("j0", "base", "turntable")

#: Every reason code `decide()` can return. The test table asserts it covers this
#: set exactly, so a reason added later cannot land untested.
REASONS = (
    # accepted
    "armed", "disarmed", "status", "motion_accepted", "stopped",
    # refused
    "empty_message", "gripper_refused", "j0_refused", "number_refused",
    "unknown_command", "unknown_pose", "not_armed", "window_expired",
    "link_stale", "link_absent", "daemon_restarted", "latched",
    "move_in_flight", "no_declared_pose", "wrong_declared_pose",
)

#: Every reason code `finish_motion()` can return.
OUTCOME_REASONS = ("motion_complete", "clamped", "board_error", "latched_during_move",
                   "restarted_during_move")


class ForbiddenLine(RuntimeError):
    """A protocol line was about to be emitted that this surface may not send.

    Raised, not returned. It means a code change produced a line the safety rules
    forbid -- a programmer error, and the loudest possible way to find out is at
    the moment of construction rather than at the bench.
    """


@dataclass(frozen=True)
class State:
    """Everything the bot remembers between messages. Frozen; `decide` returns a new one.

    `armed_until` is on the same clock as `decide`'s `now` -- the bot's
    `time.monotonic()`. `in_flight` holds the DESTINATION pose name while a
    transition's lines are being written, which is what makes a second request
    refusable rather than stackable.
    """

    armed_until: float | None = None
    declared_pose: str | None = None
    in_flight: str | None = None

    def armed(self, now: float) -> bool:
        return self.armed_until is not None and now < self.armed_until

    def remaining_s(self, now: float) -> float:
        if self.armed_until is None:
            return 0.0
        return max(0.0, self.armed_until - now)


@dataclass(frozen=True)
class Phase:
    """One phase of a transition. Only the joints inside a phase move together."""

    label: str
    targets: Mapping[int, int]
    lines: tuple[str, ...]


@dataclass(frozen=True)
class Decision:
    """What the bot should do about one message.

    `lines` is every protocol line this decision emits, in order. It is EMPTY on
    every refusal and on every expiry -- that emptiness is the assertion the
    whole phase turns on, so it is a tuple and not a list: the guard runs at
    construction, and a list could be appended to afterwards.

    THE CALLER MUST NOT FIRE `lines` BACK TO BACK. Use `phases`, and settle each
    phase before starting the next. Firing them straight through moves joints
    across a phase boundary, which is exactly the straight line between two safe
    poses that the phase ordering exists to prevent. `lines` is the flattened
    view, for auditing and for the emptiness assertions.

    `reply` is the BOT'S OWN copy. Where a line was emitted, the caller appends
    the board's verbatim reply beneath it -- this layer cannot quote a reply it
    has not seen, and must not paraphrase one.

    `events` records what the housekeeping pass fired (expiry, restart, latch,
    link verdict) so the audit row can show why a window closed.
    """

    accept: bool
    reason: str
    lines: tuple[str, ...]
    reply: str
    new_state: State
    phases: tuple[Phase, ...] = ()
    events: tuple[str, ...] = ()


def _guard(lines: Sequence[str]) -> tuple[str, ...]:
    """Reject any line this surface may not put on the wire. Raises, never returns bad."""
    for ln in lines:
        tokens = ln.split()
        verb = tokens[0].upper() if tokens else ""
        if verb in FORBIDDEN_VERBS:
            raise ForbiddenLine(
                f"{verb} is forbidden on this surface -- it detaches or clears a latch: {ln!r}")
        if verb not in ALLOWED_VERBS:
            raise ForbiddenLine(f"{verb!r} is not in ALLOWED_VERBS {ALLOWED_VERBS}: {ln!r}")
        if verb == "MOV":
            # The joint half of the allowlist. `STA` takes no arguments and
            # `STP` names no joint on this surface (the per-joint form is
            # refused upstream by the no-numbers rule), so only `MOV` carries
            # one worth checking.
            joint = tokens[1] if len(tokens) > 1 else ""
            if not joint.isdigit() or int(joint) not in MOV_JOINTS:
                raise ForbiddenLine(
                    f"{joint!r} is not in MOV_JOINTS {MOV_JOINTS}: {ln!r}. J6 acks "
                    "and does not articulate and a phone cannot see the claw; J0's "
                    "servo is dead.")
        if len(ln) > MAX_LINE_CHARS:
            raise ForbiddenLine(
                f"line is {len(ln)} chars; the firmware discards over "
                f"{MAX_LINE_CHARS} with ERR E8 LINE: {ln!r}")
    return tuple(lines)


def guard_line(line: str) -> str:
    """The single-line form of `_guard`, reachable from the bot's choke point.

    `bot._send()` writes lines `decide()` never sees -- the `STA` the settle
    loop invents is composed at that layer and never passes through a
    `Decision`. So the allowlist has to be callable from there, and it has to be
    THIS allowlist: a second, hand-rolled copy in `bot.py` is a copy that drifts,
    and the one it would drift away from is the one enforcing "no gripper, no
    J0". Raises `ForbiddenLine`; returns the line unchanged when it is allowed.
    """
    return _guard([line])[0]


def _decision(accept: bool, reason: str, reply: str, state: State,
              lines: Sequence[str] = (), phases: Sequence[Phase] = (),
              events: Sequence[str] = ()) -> Decision:
    """The single constructor. Every Decision goes through the guard, once."""
    if reason not in REASONS + OUTCOME_REASONS:
        raise ForbiddenLine(
            f"unlisted reason {reason!r} -- add it to REASONS and to the test table. "
            "An unlisted reason is a branch nothing exercises.")
    if phases and lines:
        raise ForbiddenLine("pass phases OR lines, not both -- lines is derived from phases")
    if phases:
        lines = [ln for ph in phases for ln in ph.lines]
    return Decision(accept=accept, reason=reason, lines=_guard(lines), reply=reply,
                    new_state=state, phases=tuple(phases), events=tuple(events))


def _build_phases(spec: Sequence[tuple[str, Mapping[int, int]]]) -> tuple[Phase, ...]:
    return tuple(Phase(label=label, targets=dict(targets),
                       lines=tuple(f"MOV {j} {v}" for j, v in targets.items()))
                 for label, targets in spec)


def _housekeep(state: State, now: float, link: Any) -> tuple[State, tuple[str, ...]]:
    """Apply everything that closes a window, BEFORE any command is dispatched.

    This returns state and events only. It never returns a Decision, and that is
    structural, not stylistic: every refusal has to be decided inside its own
    verb branch so that `/stop` reaches its branch on LIVE, STALE, NO LINK, a
    restart and a latch alike. A `/stop` refused for being unarmed, or for a
    stale link, would be refused at exactly the moment it was wanted.

    EXPIRY CLEARS PERMISSION. A LATCH CLEARS KNOWLEDGE. They are deliberately not
    the same call: nothing moved when a TTL ran out, so the operator's declared
    pose is still his best estimate -- but `enable_all()` snapping joints to
    their adopt angles means the declaration is no longer true.
    """
    events: list[str] = []
    verdict = getattr(link, "verdict", "NO LINK")
    restarted = bool(getattr(link, "restarted", False))
    window = getattr(link, "log_window", "") or ""

    if state.armed_until is not None and now >= state.armed_until:
        state = replace(state, armed_until=None)          # permission only
        events.append("expired")
    if restarted:
        # hold_arm.py:96 is `open(LOG, "w").close()` -- a restart truncates the
        # log AND re-runs `ENA <j> <adopt>`, so every joint snapped somewhere.
        state = replace(state, armed_until=None, declared_pose=None)
        events.append("restarted")
    if any(marker in window for marker in LATCH_MARKERS):
        state = replace(state, armed_until=None, declared_pose=None)
        events.append("latched")
    if verdict == "STALE":
        state = replace(state, armed_until=None)
        events.append("link_stale")
    elif verdict != "LIVE":
        # Anything that is not LIVE or STALE -- including a malformed producer --
        # is treated as NO LINK. Fail closed.
        state = replace(state, armed_until=None)
        events.append("link_absent")
    return state, tuple(events)


def _latch_lines(window: str) -> str:
    """The log lines that tripped the latch check, verbatim. Never paraphrased."""
    return "\n".join(ln for ln in (window or "").splitlines()
                     if any(m in ln for m in LATCH_MARKERS))


def decide(state: State, message: str, now: float, link: Any,
           ttl_s: float = DEFAULT_TTL_S) -> Decision:
    """Decide one message. Pure: same arguments, same Decision, every time.

    `link` is duck-typed on `.verdict` / `.restarted` / `.log_window` (see the
    module docstring). `now` is the caller's monotonic clock. The allowlist has
    already passed before this is called.
    """
    state, events = _housekeep(state, now, link)
    verdict = getattr(link, "verdict", "NO LINK")
    window = getattr(link, "log_window", "") or ""
    expired = "expired" in events

    def refuse(reason: str, reply: str) -> Decision:
        return _decision(False, reason, reply, state, events=events)

    text = (message or "").strip()
    if not text:
        return refuse("empty_message", f"Nothing to act on. Accepted: {ACCEPTED_LIST}")

    low = text.lower()

    # Gripper and J0 are checked before the no-numbers rule so that "j6" and "j0"
    # get the reason that explains them instead of the generic number refusal.
    if any(w in low for w in _GRIPPER_WORDS):
        return refuse(
            "gripper_refused",
            "No gripper command from here, ever. J6 acks and ramps correctly and "
            "its fingers do not articulate -- the gear is slipping on the motor "
            "shaft -- and a phone can never rule out fingers in the claw.")
    if any(w in low for w in _J0_WORDS):
        return refuse(
            "j0_refused",
            "J0 is a dead servo and appears in no pose. It stays detached.")

    # PRD section 9: everything else -- INCLUDING any message containing a number
    # -- is refused with the accepted list. One typed number goes straight
    # through the "leave a pose the way you entered it" rule.
    #
    # This catches `/stop 3` too, and that is deliberate rather than a
    # fallthrough: `STP <j>` is a different command with different semantics (on
    # a latched board it answers `ERR E6 ... JOINT=3`, not `E7`), so the
    # per-joint form is the one that produces the most confusing reply at the
    # worst possible moment. Cost: one retyped message.
    if any(ch.isdigit() for ch in low):
        return refuse(
            "number_refused",
            "Numbers are refused. Two safe poses can have an unsafe straight line "
            "between them, so only recorded paths run from here. Send exactly "
            f"/stop to stop. Accepted: {ACCEPTED_LIST}")

    parts = low.split()
    verb = parts[0].split("@", 1)[0].lstrip("/")   # tolerates /arm@somebot in a group
    arg = parts[1] if len(parts) > 1 else ""

    # ---------------------------------------------------------------- /stop
    # The sole command that works unarmed. A transition takes 5-15 s and can
    # outlive a TTL that expired while it ran; a /stop refused for being unarmed
    # is refused at exactly the moment it was wanted. It is safe to widen ONLY
    # because it can only ever reduce motion, and it does not generalise.
    #
    # STP HOLDS, WITH THE JOINTS DRIVEN. It is not an emergency stop, there is
    # no such thing here, and the copy below must never imply otherwise.
    if verb == "stop":
        stopped = replace(state, armed_until=None, declared_pose=None, in_flight=None)
        if verdict == "NO LINK":
            # NOT A DEVIATION FOR TIDINESS. hold_arm.py:177 consumes arm_cmd.txt
            # whenever it is non-empty, and :96 truncates the log at startup --
            # so a line written with no daemon present is a command QUEUED for
            # the next daemon that starts. That is section 6's prohibition
            # wearing a different hat, so nothing is sent. Reported to Mike as a
            # finding; see the phase report.
            return _decision(
                True, "stopped",
                "Nothing was sent: the daemon's log or status file is missing, and "
                "a line written now would sit in arm_cmd.txt until the next daemon "
                "starts and then run. Further commands are refused until you re-arm. "
                "This did not remove power. If you need the arm safe, put a hand "
                "under the forearm and use the rocker switch.",
                stopped, events=events)
        return _decision(
            True, "stopped", "STP sent -- the board's own reply follows.\n"
            "Motion stopped and further commands refused. This did not remove power. "
            "If you need the arm safe, put a hand under the forearm and use the "
            "rocker switch.",
            stopped, lines=["STP"], events=events)

    # -------------------------------------------------------------- /disarm
    if verb == "disarm":
        # in_flight is deliberately NOT cleared. If a transition's lines are
        # already written, closing the window does not recall them, and copy that
        # implied otherwise would be the exact shape of lie this surface exists
        # to avoid.
        closed = replace(state, armed_until=None)
        note = ("" if state.in_flight is None else
                f"\nA transition to {state.in_flight} was already sent and is not "
                "recalled by this. /stop is what sends STP.")
        return _decision(
            True, "disarmed",
            "Window closed. Nothing was sent to the arm -- the holder daemon is "
            "untouched and goes on doing exactly what it was." + note,
            closed, events=events)

    # -------------------------------------------------------------- /status
    if verb == "status":
        if verdict == "LIVE":
            return _decision(
                True, "status",
                "Link verdict: LIVE -- the status file was rewritten inside the "
                "freshness window. Sending STA; the board's own reply follows.",
                state, lines=["STA"], events=events)
        if verdict == "STALE":
            return _decision(
                True, "status",
                "Link verdict: STALE -- the status file has not been rewritten "
                "inside the freshness window. Motion is refused until it reads "
                "LIVE. Sending STA anyway: the daemon only writes that file when "
                "STA came back non-empty, so the command channel can still be "
                "alive when the status write is not.",
                state, lines=["STA"], events=events)
        return _decision(
            True, "status",
            "Link verdict: NO LINK -- the daemon's log or status file is missing. "
            "Nothing was sent: a line written now would sit in arm_cmd.txt until "
            "the next daemon starts and then run. This cannot tell a crashed "
            "daemon from a wedged one from a sleeping laptop.",
            state, events=events)

    # ----------------------------------------------------------------- /arm
    if verb == "arm":
        # BEFORE the pose check: a declaration made while a transition is still
        # out cannot be honest, whatever pose it names -- the arm is part-way
        # along a path. This is also the hole through which a second transition
        # could otherwise be smuggled past the one-move-at-a-time rule: /arm
        # would clear in_flight, and the next /go would be accepted while the
        # first path was still running. /stop clears in_flight on every path, so
        # refusing here strands nobody.
        if state.in_flight is not None:
            return refuse(
                "move_in_flight",
                f"Refused -- a transition to {state.in_flight} is still out. You "
                "cannot declare a start pose for an arm that is part-way along a "
                "path. /stop first, then look at the arm and arm again.")
        if arg not in POSES:
            return refuse(
                "unknown_pose",
                f"Name the pose the arm is in right now: {' or '.join('/arm ' + p for p in POSES)}.")
        if verdict == "STALE":
            return refuse(
                "link_stale",
                "Refused -- link verdict STALE. The daemon has not rewritten its "
                "status file inside the freshness window, so there is nothing to "
                "arm against. Try /status.")
        if verdict != "LIVE":
            return refuse(
                "link_absent",
                "Refused -- link verdict NO LINK. The daemon's log or status file "
                "is missing. Nothing was sent and no port was opened.")
        # /arm is the RECOVERY path after a latch or a restart: the remedy this
        # project prescribes for not knowing where the arm is, is a human looking
        # at it and declaring a pose. So a latch event does not block arming --
        # it is reported instead.
        # replace(), never a fresh State(): a fresh object silently resets every
        # field it does not mention, and in_flight is exactly the field whose
        # silent reset lets a second transition through. The guard above is what
        # makes this safe -- not the constructor.
        armed = replace(state, armed_until=now + ttl_s, declared_pose=arg)
        runnable = ", ".join(f"/go {dest}" for (src, dest) in _PATHS if src == arg) or "nothing"
        warn = ""
        if "latched" in events:
            warn = ("\nWARNING -- the log window shows a latch or a re-ENA. The "
                    "daemon re-enabled and joints snapped to their adopt angles. "
                    "Look at the arm before you send anything:\n" + _latch_lines(window))
        elif "restarted" in events:
            warn = ("\nWARNING -- the daemon restarted. That re-ran ENA on every "
                    "joint, so they snapped to their adopt angles. Look at the arm "
                    "before you send anything.")
        return _decision(
            True, "armed",
            f"Armed for {int(ttl_s)} s. Nothing was sent to the arm.\n"
            f"Declared start pose: {arg}. Nothing observed that -- it is your "
            "estimate, exactly like ENA <j> <adopt>.\n"
            f"Runnable from here: {runnable}.\n"
            "The window is extended by nothing -- not by activity, not by a "
            "completed move. One driver at a time: this cannot stop another "
            "program writing arm_cmd.txt, so do not run cycle_poses.py while "
            "this window is open." + warn,
            armed, events=events)

    # ------------------------------------------------------------------ /go
    if verb == "go":
        # Link first, so the reply names the real obstacle instead of the
        # unarmed state that the housekeeping pass just produced from it.
        if verdict == "STALE":
            return refuse(
                "link_stale",
                "Refused -- link verdict STALE. The daemon has not rewritten its "
                "status file inside the freshness window, and that loop's "
                "heartbeat is what feeds the 4000 ms watchdog. Nothing was sent. "
                "The window is closed; re-arm when /status reads LIVE.")
        if verdict != "LIVE":
            return refuse(
                "link_absent",
                "Refused -- link verdict NO LINK. The daemon's log or status file "
                "is missing. Nothing was sent and no port was opened.")
        if "restarted" in events:
            return refuse(
                "daemon_restarted",
                "Refused -- the daemon restarted: its log went backwards. That "
                "re-ran ENA on every joint, so they snapped to their adopt "
                "angles. The declared pose is void and the window is closed. "
                "Look at the arm, then /arm <pose> again.")
        if "latched" in events:
            return refuse(
                "latched",
                "Refused -- the log window shows a latch or a re-ENA. The daemon "
                "cleared a latch and re-enabled, which snaps joints to their "
                "adopt angles and reads exactly like the arm moving on its own. "
                "The declared pose is void and the window is closed. Log lines, "
                "verbatim:\n" + _latch_lines(window))
        if expired:
            return refuse(
                "window_expired",
                "Refused -- the armed window expired. NOTHING WAS SENT TO THE ARM: "
                "expiry is a state change here, not a command, because the verbs "
                "that sound safe (EST, DIS) detach every joint and the arm falls. "
                "The holder daemon goes on doing exactly what it was. Send "
                "/arm <pose> to open a new window.")
        if not state.armed(now):
            return refuse(
                "not_armed",
                "Refused -- not armed. Send /arm <pose> naming the pose the arm is "
                "in right now, then /go. Nothing was sent and nothing is queued.")
        if state.in_flight is not None:
            return refuse(
                "move_in_flight",
                f"Refused -- a transition to {state.in_flight} is already out. "
                "Requests are never queued: a queued motion command asserts "
                "consent that has already expired. Nothing was sent. /stop sends STP.")
        if arg not in POSES:
            return refuse(
                "unknown_pose",
                f"Unknown pose. Accepted: {', '.join('/go ' + p for p in POSES)}.")
        if state.declared_pose is None:
            return refuse(
                "no_declared_pose",
                "Refused -- no declared pose. Nothing here observes a shaft, so "
                "the arm cannot be asked where it is. Send /arm <pose> first.")
        spec = _PATHS.get((state.declared_pose, arg))
        if spec is None:
            return refuse(
                "wrong_declared_pose",
                f"Refused -- /go {arg} runs a recorded path that starts from the "
                f"other pose, and you declared {state.declared_pose} when you "
                "armed. Paths are directional and are never interpolated. If the "
                "arm is somewhere else, look at it and re-arm.")
        phases = _build_phases(spec)
        moving = replace(state, in_flight=arg)
        labels = ", ".join(ph.label for ph in phases)
        return _decision(
            True, "motion_accepted",
            f"Accepted: {state.declared_pose} -> {arg}, {len(phases)} phases "
            f"({labels}). Each phase settles before the next starts -- the "
            "ordering is the safety property, not the endpoints. Every board "
            "reply is quoted as it lands; a board ack is not motion. No speed is "
            "commanded: the daemon's own speeds are inherited. /stop sends STP.",
            moving, phases=phases, events=events)

    return refuse("unknown_command", f"Not a command. Accepted: {ACCEPTED_LIST}")


def finish_motion(state: State, replies: Iterable[str], log_window: str = "",
                  restarted: bool = False) -> Decision:
    """Judge a completed transition FROM ITS OWN REPLIES. Sends nothing, ever.

    NEVER from `arm_status.txt`. That file is rewritten only on the daemon's 5 s
    poll, so immediately after a `MOV` it still holds pre-move state -- and says
    `MOV=0`. A "done" read from it is a lie shaped exactly like success.

    `CL=1` IS A FAILED COMMAND, NOT A WARNING. The firmware clamped the request
    and the joint is not where you asked.

    On success the declared pose becomes the destination. The armed window is NOT
    extended -- a completed move buys no time.

    This judges only the replies it is handed. A reply that never arrived is the
    caller's problem to notice; this layer cannot tell silence from success.

    `restarted` IS CHECKED FIRST AND IT HAD TO BE ADDED. A daemon that restarts
    part-way through a transition truncates its log and re-runs
    `ENA <j> <adopt>` on every joint -- so the replies collected so far are all
    `CL=0`, the log window is a BRAND NEW log with no `LATCHED` and no `re-ENA`
    in it (`hold_arm.main()` logs plain `ENA`; only `enable_all()` logs
    `re-ENA`), and every other check here passes. Without this flag the audit
    said `motion_complete`, the declared pose advanced to the destination, and
    the window stayed open -- for a transition whose second half ran against an
    arm that had snapped back to five adopt angles. Found by probing, not by
    reading; see `test_a_daemon_restart_at_a_phase_boundary_...` in
    `tests/test_bot.py`.
    """
    if state.in_flight is None:
        raise ValueError("finish_motion called with no transition in flight")

    dest = state.in_flight
    lines = [r for r in replies]
    blob = "\n".join(lines)
    voided = State(armed_until=None, declared_pose=None, in_flight=None)

    if restarted:
        return _decision(
            False, "restarted_during_move",
            "The transition is NOT believed complete: the daemon's log was "
            "replaced, which means it restarted part-way through. A restart "
            "re-runs ENA <j> <adopt> on every joint, so joints snapped to their "
            "adopt angles with nobody asking -- and the rest of the recorded path "
            "was NOT sent, because that path assumes a start the arm no longer "
            "has. Nothing was sent to make it safe: the verbs that sound safe "
            "(EST, DIS) detach every joint and a loaded arm falls. The declared "
            "pose is void and the window is closed. Look at the arm, then "
            "/arm <pose> again. Board lines collected before the restart, and "
            "then the new log, both verbatim:\n" + blob + "\n" + (log_window or ""),
            voided)
    if any(m in (log_window or "") for m in LATCH_MARKERS):
        return _decision(
            False, "latched_during_move",
            "The transition is NOT believed complete: the log window shows a latch "
            "or a re-ENA, so joints snapped to their adopt angles part-way. The "
            "declared pose is void and the window is closed. Log lines, verbatim:\n"
            + _latch_lines(log_window),
            voided)
    if any("CL=1" in ln for ln in lines):
        clamped = "\n".join(ln for ln in lines if "CL=1" in ln)
        return _decision(
            False, "clamped",
            "FAILED -- the firmware clamped a waypoint, so that joint is not where "
            "it was asked to go and the arm is part-way along the path. The "
            "declared pose is void and the window is closed. Look at the arm, then "
            "/arm <pose> again. Board lines, verbatim:\n" + clamped,
            voided)
    if any(ln.strip().startswith("ERR") for ln in lines):
        errs = "\n".join(ln for ln in lines if ln.strip().startswith("ERR"))
        return _decision(
            False, "board_error",
            "FAILED -- the board refused a waypoint, so the arm is part-way along "
            "the path. The declared pose is void and the window is closed. Look at "
            "the arm, then /arm <pose> again. Board lines, verbatim:\n" + errs,
            voided)

    # armed_until is carried through UNCHANGED. A completed move extends nothing.
    done = replace(state, declared_pose=dest, in_flight=None)
    return _decision(
        True, "motion_complete",
        f"Every waypoint came back CL=0 and the log window is clean of LATCHED and "
        f"re-ENA, so the declared pose is now {dest}. That is bookkeeping about "
        "commands the board accepted -- nothing here observed a shaft. The window "
        "was not extended by this.\n" + blob,
        done)
