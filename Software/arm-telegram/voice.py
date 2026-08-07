"""The voice seam. It transcribes nothing, and that is the whole of it today.

`transcribe(audio_bytes, mime) -> str | None` is one function with one job: turn
audio into text, or admit it cannot. It returns `None`. There is no model here,
no download, no decoder, and no backend named -- PRD Q17-Q1 is unanswered and a
default picked quietly is a decision nobody made.

WHY A SEAM AT ALL, IF IT DOES NOTHING
    Because the shape is the part that has to be right, and the shape is: VOICE
    IS A TEXT SOURCE AND NOTHING MORE. Whatever this returns goes through the
    identical allowlist, arming, freshness and refusal path as a message somebody
    typed. It gets no privileges of its own, no shortcut, and no separate
    vocabulary. `bot.py` calls this and then joins the same road.

FAIL CLOSED. THIS IS THE RULE THAT IS CHEAP NOW AND EXPENSIVE LATER.
    An unrecognised transcription is REFUSED. It is never fuzzy-matched, never
    scored for similarity, never snapped to the nearest pose name.

    "pick", "quick" and "stick" are one bad microphone apart. So are "storage"
    and "store it". A near-miss that MOVES THE ARM is exactly the failure a
    phone-shaped interface invites, and the operator would have no way to tell
    the difference between "it heard me" and "it guessed, and guessed the pose I
    am not standing next to".

    The refusal comes for free, and that is worth stating plainly so nobody
    "improves" it: `bot.py` hands this string to `policy.decide()`, which knows
    exactly six commands and refuses everything else with the accepted list. So
    the day a backend lands, an unrecognised word already refuses -- as long as
    nobody adds a similarity score in between. DO NOT ADD ONE. `tests/test_bot.py`
    asserts this module imports no matcher and that near-misses refuse.

WHAT A BACKEND WILL HAVE TO CARRY WHEN IT ARRIVES
    Telegram delivers voice notes as OGG/Opus, so a backend needs a decoder AND a
    speech-to-text model: a dependency chain, and for most backends a model
    download. Every one of those gets a WRITTEN LICENCE NOTE, with its licence
    named, BEFORE it is installed -- Apache-2.0 or MIT only, following
    `Software/lerobot_robot_emre_arm/pyproject.toml`, which flags pyserial and
    numpy as off-list rather than passing them silently. Adding a dependency here
    is a stop-and-ask, not a decision an implementer makes on the way past.

NOTHING HERE HAS RUN AGAINST HARDWARE, AND NOTHING HERE HAS HEARD AUDIO.
"""

from __future__ import annotations

#: What the operator gets back when a voice note arrives. It says what to do
#: instead, because a refusal that does not tell you the next move is a refusal
#: you have to guess your way out of on a phone in a noisy room.
VOICE_DISABLED_REPLY = (
    "Voice is not enabled -- send a text command. Nothing was sent to the arm.\n"
    "The audio was fetched, handed to the transcriber, and discarded unheard; "
    "no recording is written to disk and none is kept.\n"
    "Accepted: /arm <pose>, /disarm, /status, /go storage, /go pick, /stop"
)


def transcribe(audio_bytes: bytes, mime: str) -> str | None:
    """Turn a voice note into a command string, or return `None`.

    RETURNS `None`. Always, today. `bot.py` treats `None` as "voice is not
    enabled", replies `VOICE_DISABLED_REPLY`, writes an audit row, and sends
    NOTHING to the arm.

    The arguments are real and are really passed -- `bot.py` downloads the OGG
    and hands over the bytes and the mime type it was given. That costs two Bot
    API calls to produce a refusal, which is the honest price of a seam that is
    exercised rather than described: the day a backend lands, ONE FUNCTION BODY
    changes and the fetch, the mime plumbing and the failure handling are already
    proven. A seam whose arguments are never constructed is a seam that does not
    work the first time somebody needs it.

    When a backend does land, the contract it must keep:

      * Return the transcription VERBATIM. Do not normalise it towards a known
        command, do not correct it, do not pick the closest match. Hand back what
        was heard and let `policy.decide()` refuse it.
      * Return `None` on any doubt -- a failed decode, a truncated download, a
        low-confidence result, a model that is not loaded. `None` refuses. There
        is no third answer and no "probably".
      * Never write the audio to disk, never log it, and never include it in an
        audit row. Duration is recorded; the recording is not.
    """
    return None
