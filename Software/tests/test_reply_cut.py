"""Pin the reply-cut rule, and pin every copy of it against the others.

This exists because the same six lines were duplicated into three tools and all
three carried the same safety-relevant defect: a reply truncated before its CL=
field read as CLEAN. The fix is only durable if a future edit to one copy fails
here rather than silently diverging.

    cd Software/tests && python -m pytest test_reply_cut.py -q
"""

from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from reply_cut import clamped, cut_reply

# The literal string a real pick->storage run produced on 2026-08-07. The
# daemon's heartbeat reply landed in front, and this command's own reply was cut
# off mid-field by the tail happening before the flush.
LIVE_CONTAMINATED = "OK PNG UP=2480971\n\n08:31:02 OK MOV J1 REQ=88 SET=88 C"

# A STA reply: hold_arm.log() timestamps only the FIRST line of a multi-line
# record, so the joint rows that follow are untimestamped continuations. A cut at
# the next NEWLINE would throw all of them away.
STA_MULTILINE = (
    "STA J0 EN=0 SET=90 TGT=90 MIN=70 MAX=110 CAL=0 DPS=30 MOV=0 JTO=0\n"
    "STA J1 EN=1 SET=88 TGT=88 MIN=0 MAX=91 CAL=1 DPS=5 MOV=0 JTO=0\n"
    "STA J3 EN=1 SET=64 TGT=64 MIN=0 MAX=66 CAL=1 DPS=5 MOV=0 JTO=0\n"
    "SYS ES=0 WD=0 WDMS=4000 MIR=INV UP=99 UNCAL=1\n"
    "OK STA N=6\n"
    "08:31:07 CMD PNG -> OK PNG UP=2481011"
)


def test_the_live_contaminated_reply_is_cut_at_the_daemon_timestamp():
    assert cut_reply(LIVE_CONTAMINATED) == "OK PNG UP=2480971"


def test_a_truncated_reply_is_unreadable_and_never_reads_as_clean():
    """The whole point. Before the fix this returned False -- i.e. 'no clamp'."""
    cut = cut_reply(LIVE_CONTAMINATED)
    assert clamped(cut) is None, "an unreadable reply must not read as clean"
    assert clamped(cut) is not False


def test_a_multi_line_sta_survives_the_cut_intact():
    out = cut_reply(STA_MULTILINE)
    assert out.count("STA J") == 3, "a newline cut would have kept only J0"
    assert out.endswith("OK STA N=6")
    assert "PNG" not in out


@pytest.mark.parametrize("reply,expected", [
    ("OK MOV J1 REQ=88 SET=88 CL=0", False),
    ("OK MOV J1 REQ=88 SET=88 CL=1", True),
    ("OK MOV J1 REQ=88 SET=88 C", None),
    ("", None),
    ("ERR E5 MOV JOINT=1", None),
])
def test_clamped_reads_three_states_not_two(reply, expected):
    assert clamped(reply) is expected


def test_a_clean_reply_is_untouched():
    r = "OK MOV J5 REQ=165 SET=165 CL=0"
    assert cut_reply(r) == r


def test_the_telegram_copy_agrees_with_this_one():
    """arm-telegram/arm_link.py keeps its own implementation on purpose -- the
    bot must stay standard-library-only and self-contained. That is exactly why
    it needs pinning: two implementations of one rule is how one goes stale."""
    tg = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      "arm-telegram")
    sys.path.insert(0, tg)
    import arm_link

    cutter = None
    for name in ("cut_reply", "_cut_reply", "cut_at_timestamp"):
        if hasattr(arm_link, name):
            cutter = getattr(arm_link, name)
            break
    if cutter is None:
        pytest.skip("arm_link exposes no module-level cut function to compare")

    for raw in (LIVE_CONTAMINATED, STA_MULTILINE, "OK MOV J5 REQ=165 SET=165 CL=0"):
        assert cutter(raw) == cut_reply(raw), f"implementations diverged on {raw!r}"
