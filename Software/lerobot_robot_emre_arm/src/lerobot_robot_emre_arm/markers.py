"""The observer interface: how a camera turns into the observed half of Option A.

The marker SYSTEM -- dictionary choice, the 13 stickers, the size chain, where
each one goes and why -- is designed and committed in Documentation/MARKER-SYSTEM.md
and Software/vision/markers.csv. This module does not restate it. It defines the
narrow contract the adapter needs, so that the detection slice can be built and
swapped without the robot class learning anything about cameras.

WHY THE OBSERVER IS NOT OPTIONAL POLISH
    It reads like a later nicety, and in build order it is. But it is also the
    only thing that unblocks unattended recording, and that link is easy to miss:

        `ENA <j> <adopt_deg>` takes a HUMAN'S BY-EYE ESTIMATE of where the shaft
        physically is. After any detach -- watchdog trip, e-stop, or this
        adapter's own idle park -- a gravity-loaded arm SAGS, and the next adopt
        angle must be freshly estimated against an arm that moved while nothing
        was commanding it.

    So nothing can re-enable a joint automatically today, which means a
    multi-episode `lerobot-record` run cannot be left alone. An observer is the
    only mechanism that could ever supply a trustworthy adopt angle. It is the
    prerequisite for the headline use case, not a garnish on it.

WHAT AN OBSERVER CAN AND CANNOT CLAIM TODAY
    Markers give a CHANGE in orientation. Turning that into an angle in the
    firmware's commanded-degree convention needs two things that do not exist:

      Pass 1  AXIS IDENTIFICATION. Drive each joint alone across its calibrated
              range, fit a rotation axis to the child tag's trajectory relative
              to the parent tag. This recovers the joint axes and the
              link-to-link transforms that the STLs cannot give -- there are 21
              individual parts in print orientation and no assembly file.
      Pass 2  DATUM CAPTURE at commanded home, so absolute angles have a
              reference at all.

    Until both have run, an implementation MUST NOT return OBS_SOURCE_MARKER,
    because `residual_deg` would then be measuring the observer's own
    uncalibrated mapping rather than the arm's tracking error.
"""

from __future__ import annotations

from typing import Any, Mapping, Protocol, runtime_checkable

from .observation import JointFix


@runtime_checkable
class MarkerObserver(Protocol):
    """Turns camera frames into per-joint observations.

    NINE CONSTRAINTS. An implementation that breaks any of them is wrong, and
    most of them are wrong in ways that look like they work:

    1. CONSUMES FRAMES IT IS GIVEN. It must not open its own capture. The camera
       is already open under LeRobot, a camera cannot be opened twice, and a
       second capture would not be time-aligned with the frame the dataset
       actually records.
    2. TAKES CAPTURE TIME, NOT CALL TIME. `capture_mono` is `time.monotonic()` at
       the moment each frame was GRABBED. LeRobot camera reads come from a
       background thread and return the most recent frame, so call time can lag
       capture by a whole frame interval -- using it silently under-reports
       staleness and inflates trust in an old fix.
    3. KEYED BY LOGICAL JOINT ID. Never by part name or marker id. Mapping
       marker -> link -> joint is the observer's job.
    4. NEVER RETURNS JOINT 2. D5 is not addressable. An observer that can see the
       second shoulder horn reports it as `pair_disagree_deg` on joint 1.
    5. DISTINGUISHES "COULD NOT SEE" FROM "DID NOT LOOK". A joint that was
       searched for and not found returns OBS_SOURCE_OCCLUDED. Omitting it means
       "never looked", and the mask must be able to tell those apart.
    6. ANGLES IN COMMANDED-DEGREE CONVENTION. See the module docstring -- this is
       the constraint that is unmet today.
    7. JOINT 1 PAIR DISAGREEMENT IS OPTIONAL AND LIVES ON THE j1 FIX. An observer
       that can see both shoulder horns sets `pair_disagree_deg` to the signed
       difference between the angles they imply. This is the only mechanism this
       project has ever had for measuring the shoulder mirror offset, which has
       never been measured.
    8. TIME BUDGET <= 10 ms. A full STA round trip already costs roughly 40 ms of
       wire time at 115200 (arithmetic from the reply's byte count, not a bench
       measurement), which caps the loop near 20 Hz. An observer that blocks for
       longer sets the frame rate, not the arm.
    9. NO SERIAL ACCESS. It must not know the port exists.
    """

    def observe(
        self,
        frames: Mapping[str, Any],
        capture_mono: Mapping[str, float],
    ) -> dict[int, JointFix]:
        """Return {joint_id: JointFix} for whatever this frame supports."""
        ...


class NullMarkerObserver:
    """Observes nothing, and says so honestly.

    This is the day-one implementation and the correct default. Every
    `observed_deg` / `residual_deg` / `obs_age_s` column comes out NaN with
    `obs_source = OBS_SOURCE_NONE`.

    It is NOT a stub waiting to be replaced by something that guesses. Returning
    `observed = commanded` would assert the arm is exactly where it was told to
    be, which is the precise fiction the Option A contract exists to destroy.
    Nothing observes this arm today; this class is what that fact looks like.
    """

    def observe(
        self,
        frames: Mapping[str, Any],
        capture_mono: Mapping[str, float],
    ) -> dict[int, JointFix]:
        return {}


class ArucoMarkerObserver:
    """DICT_4X4_50 detection against the sticker set in markers.csv.

    Designed but NOT built. The design is real and committed -- see
    Documentation/MARKER-SYSTEM.md and Software/vision/markers.csv, which size
    all 13 stickers from measured STL geometry rather than guesses. What is
    missing is everything that turns a detection into an angle.
    """

    def __init__(self, *args, **kwargs) -> None:
        raise NotImplementedError(
            "The marker observer is designed but not implemented. Five things are "
            "missing, and the first three are measurements rather than code:\n"
            "  1. CAMERA INTRINSICS. No calibration has been run. The design's "
            "     focal length is ASSUMED from a 60-degree HFOV; every marker "
            "     size and every detection-grade claim in MARKER-SYSTEM.md scales "
            "     linearly off it. Run a ChArUco board first.\n"
            "  2. AXIS IDENTIFICATION (Pass 1). The joint axes and link-to-link "
            "     transforms are unknown -- the STLs are separate parts in print "
            "     orientation with no assembly file. They must be fitted by "
            "     driving each joint alone and watching the child tag.\n"
            "  3. DATUM CAPTURE (Pass 2). Without a reference at commanded home, "
            "     `observed - commanded` has nothing to be a residual against.\n"
            "  4. THE STICKERS DO NOT PHYSICALLY EXIST. None have been printed "
            "     or applied.\n"
            "  5. A KNOWN NEGATIVE RESULT: at the ~550 mm standoff needed to "
            "     frame the arm, a single 1280x720 camera cannot detect the "
            "     forearm, wrist or gripper markers at all. That needs a second "
            "     close camera or 40 mm printed marker tabs -- 30 mm is not "
            "     enough.\n"
            "Passes 1 and 2 also both require driving an assembled arm, which "
            "needs the 3-5 A supply that is still outstanding.\n"
            "Use NullMarkerObserver until these are resolved."
        )
