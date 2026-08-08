"""The Option A observation contract, asserted.

These tests exist because `observation.py` makes promises that are invisible at
the call site and expensive to break:

  * `.pos` is the COMMANDED angle. It is never fused with, defaulted to, or
    substituted for anything a camera or a human observed.
  * `observed_deg` / `residual_deg` / `obs_source` / `obs_age_s` are four separate
    columns and stay separate. Collapsing any pair of them re-creates the single
    ambiguous "where is the joint" field that this whole schema exists to destroy.
  * an absent observation is NaN, never 0.0 and never the commanded angle.
  * a stale fix KEEPS its value and its age and LOSES its residual.
  * joint id 2 has no key at all.
  * the flat `observation.state` block boundaries in the module docstring are what
    a recorded dataset will actually contain.

The modules are loaded by file path -- see conftest.py for why that is necessary
and why it is not a shortcut.
"""

from __future__ import annotations

import math

import pytest

# --------------------------------------------------------------------------
# Joint id set
# --------------------------------------------------------------------------


def test_addressable_joint_ids_are_the_six_logical_joints(observation):
    assert observation.JOINT_IDS == (0, 1, 3, 4, 5, 6)
    assert isinstance(observation.JOINT_IDS, tuple)


def test_reserved_joint_is_2_and_is_not_addressable(observation):
    assert observation.RESERVED_JOINT_ID == 2
    assert observation.RESERVED_JOINT_ID not in observation.JOINT_IDS


def test_joint_key_refuses_the_reserved_joint(observation):
    with pytest.raises(ValueError, match="not"):
        observation.joint_key(observation.RESERVED_JOINT_ID, "pos")


def test_joint_key_refuses_an_unknown_joint(observation):
    with pytest.raises(ValueError, match="unknown joint id"):
        observation.joint_key(7, "pos")


def test_no_j2_key_exists_anywhere_in_the_schema(observation):
    for name in observation.OBSERVATION_STATE_NAMES + observation.ACTION_NAMES:
        assert not name.startswith("j2."), name


def test_joint_labels_cover_exactly_the_addressable_joints(observation):
    assert tuple(sorted(observation.JOINT_LABELS)) == observation.JOINT_IDS
    assert observation.RESERVED_JOINT_ID not in observation.JOINT_LABELS


def test_joint_labels_are_display_only_and_never_appear_in_a_key(observation):
    # A key is built from the id, so no human-readable label may leak into one.
    keys = set(observation.OBSERVATION_STATE_NAMES) | set(observation.ACTION_NAMES)
    for label in observation.JOINT_LABELS.values():
        word = label.split()[0].lower()
        assert not any(word in k.lower() for k in keys), label


# --------------------------------------------------------------------------
# The two LeRobot feature dicts
# --------------------------------------------------------------------------


def test_action_features_are_one_commanded_angle_per_logical_joint(observation):
    feats = observation.action_features()
    assert list(feats) == ["j0.pos", "j1.pos", "j3.pos", "j4.pos", "j5.pos", "j6.pos"]
    assert all(v is float for v in feats.values())
    assert len(feats) == len(observation.JOINT_IDS) == 6


def test_action_names_match_action_features(observation):
    assert observation.ACTION_NAMES == tuple(observation.action_features())


def test_the_shoulder_pair_cannot_be_commanded_separately(observation):
    # Six action keys, seven physical servos. Exposing D4 and D5 separately would
    # let a policy drive the pair apart -- the highest-consequence mechanical
    # failure available on this arm.
    assert len(observation.ACTION_NAMES) == 6
    assert "j2.pos" not in observation.ACTION_NAMES


def test_observation_features_declare_six_fields_per_joint_plus_the_tail(observation):
    feats = observation.observation_features()
    expected = len(observation.OBSERVATION_FIELDS) * len(observation.JOINT_IDS) + 1
    assert len(feats) == expected == 37
    assert all(v is float for v in feats.values())
    assert list(feats)[-1] == observation.PAIR_DISAGREE_KEY


def test_the_six_per_joint_fields_are_the_documented_ones(observation):
    assert observation.OBSERVATION_FIELDS == (
        "pos",
        "observed_deg",
        "residual_deg",
        "obs_source",
        "obs_age_s",
        "clamped",
    )


def test_every_joint_carries_all_six_fields_as_distinct_keys(observation):
    feats = observation.observation_features()
    for jid in observation.JOINT_IDS:
        keys = [observation.joint_key(jid, f) for f in observation.OBSERVATION_FIELDS]
        assert len(set(keys)) == 6, keys
        for k in keys:
            assert k in feats


def test_commanded_and_observed_are_separate_keys_for_every_joint(observation):
    # The single assertion this whole module exists for: there is no one key that
    # means "where the joint is". Commanded, observed and their difference are
    # three columns, and each is declared for every joint.
    feats = observation.observation_features()
    for jid in observation.JOINT_IDS:
        commanded = observation.joint_key(jid, "pos")
        observed = observation.joint_key(jid, "observed_deg")
        residual = observation.joint_key(jid, "residual_deg")
        assert commanded != observed != residual != commanded
        assert {commanded, observed, residual} <= set(feats)


def test_camera_features_are_tuples_and_come_after_the_scalars(observation):
    cams = {"top": (480, 640, 3), "wrist": (240, 320, 3)}
    feats = observation.observation_features(cams)
    assert feats["top"] == (480, 640, 3)
    assert feats["wrist"] == (240, 320, 3)
    scalars = [k for k, v in feats.items() if v is float]
    assert list(feats)[: len(scalars)] == scalars
    assert list(feats)[len(scalars):] == ["top", "wrist"]


def test_camera_keys_are_excluded_from_the_flat_state_names(observation):
    feats = observation.observation_features({"top": (480, 640, 3)})
    floats = tuple(k for k, v in feats.items() if v is float)
    assert floats == observation.OBSERVATION_STATE_NAMES
    assert "top" not in observation.OBSERVATION_STATE_NAMES


def test_the_scalar_schema_is_identical_with_and_without_cameras(observation):
    # Schema stability: adding a camera must not renumber the scalar block, or
    # `lerobot-record --resume` and dataset concatenation break.
    without = [k for k, v in observation.observation_features().items() if v is float]
    with_cams = [
        k for k, v in observation.observation_features({"top": (480, 640, 3)}).items()
        if v is float
    ]
    assert without == with_cams


# --------------------------------------------------------------------------
# The flat observation.state block boundaries
#
# The module docstring publishes these exact slices. Downstream code is told to
# derive them from `names` at load time, but the promise is still a promise.
# --------------------------------------------------------------------------


def test_state_names_are_field_major_with_the_documented_block_boundaries(observation):
    names = observation.OBSERVATION_STATE_NAMES
    assert len(names) == 37
    blocks = {
        "pos": (0, 6),
        "observed_deg": (6, 12),
        "residual_deg": (12, 18),
        "obs_source": (18, 24),
        "obs_age_s": (24, 30),
        "clamped": (30, 36),
    }
    for field, (lo, hi) in blocks.items():
        expected = [observation.joint_key(j, field) for j in observation.JOINT_IDS]
        assert list(names[lo:hi]) == expected, field
    assert names[36] == observation.PAIR_DISAGREE_KEY


def test_the_pair_disagreement_tail_belongs_to_joint_1_only(observation):
    assert observation.PAIR_DISAGREE_KEY == "j1.pair_disagree_deg"
    # It is a tail singleton, not a seventh per-joint field, so no other joint
    # has one and it does not make any six-wide block ragged.
    assert sum(
        1 for n in observation.OBSERVATION_STATE_NAMES if n.endswith("pair_disagree_deg")
    ) == 1


# --------------------------------------------------------------------------
# The observation source enum
# --------------------------------------------------------------------------


def test_observation_sources_are_distinct_and_labelled(observation):
    sources = (
        observation.OBS_SOURCE_NONE,
        observation.OBS_SOURCE_STALE,
        observation.OBS_SOURCE_OCCLUDED,
        observation.OBS_SOURCE_MARKER,
        observation.OBS_SOURCE_HUMAN,
    )
    assert len(set(sources)) == 5
    assert all(isinstance(s, float) for s in sources)
    assert set(observation.OBS_SOURCE_LABELS) == set(sources)


def test_never_looked_and_looked_and_could_not_see_are_different_facts(observation):
    assert observation.OBS_SOURCE_NONE != observation.OBS_SOURCE_OCCLUDED


# --------------------------------------------------------------------------
# build_observation -- the unobserved case, which is the normal case
# --------------------------------------------------------------------------

COMMANDED = {0: 64.0, 1: 1.0, 3: 33.0, 4: 90.0, 5: 104.0, 6: 90.0}


def test_with_no_observer_the_commanded_angle_is_carried_through_unchanged(observation):
    obs = observation.build_observation(COMMANDED, {}, None, now_mono=100.0)
    for jid, cmd in COMMANDED.items():
        assert obs[observation.joint_key(jid, "pos")] == cmd


def test_with_no_observer_observed_residual_and_age_are_nan_not_zero(observation):
    # `observed = 0.0` would claim the joint was observed at zero degrees, and
    # `observed = commanded` would assert the arm is exactly where it was told to
    # be. Both are silently wrong; NaN is loudly wrong.
    obs = observation.build_observation(COMMANDED, {}, None, now_mono=100.0)
    for jid in observation.JOINT_IDS:
        for field in ("observed_deg", "residual_deg", "obs_age_s"):
            value = obs[observation.joint_key(jid, field)]
            assert math.isnan(value), (jid, field, value)
            assert value != 0.0


def test_with_no_observer_the_commanded_angle_is_not_laundered_into_observed(observation):
    obs = observation.build_observation(COMMANDED, {}, None, now_mono=100.0)
    for jid in observation.JOINT_IDS:
        assert math.isnan(obs[observation.joint_key(jid, "observed_deg")])


def test_with_no_observer_the_source_is_none_and_finite(observation):
    obs = observation.build_observation(COMMANDED, {}, None, now_mono=100.0)
    for jid in observation.JOINT_IDS:
        src = obs[observation.joint_key(jid, "obs_source")]
        assert src == observation.OBS_SOURCE_NONE
        assert not math.isnan(src)


def test_build_observation_emits_exactly_the_declared_scalar_keys(observation):
    declared = {k for k, v in observation.observation_features().items() if v is float}
    obs = observation.build_observation(COMMANDED, {}, None, now_mono=100.0)
    assert set(obs) == declared


def test_the_key_set_does_not_change_when_an_observer_appears(observation):
    fix = observation.JointFix(
        angle_deg=61.0, source=observation.OBS_SOURCE_MARKER, captured_mono=99.9
    )
    without = observation.build_observation(COMMANDED, {}, None, now_mono=100.0)
    with_fix = observation.build_observation(COMMANDED, {}, {0: fix}, now_mono=100.0)
    assert set(without) == set(with_fix)


def test_clamped_is_one_when_the_board_clamped_and_zero_otherwise(observation):
    obs = observation.build_observation(
        COMMANDED, {0: True, 3: False}, None, now_mono=100.0
    )
    assert obs["j0.clamped"] == 1.0
    assert obs["j3.clamped"] == 0.0
    # A joint missing from the mapping is not clamped, not unknown.
    assert obs["j5.clamped"] == 0.0


# --------------------------------------------------------------------------
# build_observation -- a fresh fix
# --------------------------------------------------------------------------


@pytest.mark.parametrize("source_name", ["OBS_SOURCE_MARKER", "OBS_SOURCE_HUMAN"])
def test_a_fresh_fix_publishes_observed_and_residual(observation, source_name):
    source = getattr(observation, source_name)
    fix = observation.JointFix(angle_deg=61.0, source=source, captured_mono=99.9)
    obs = observation.build_observation(
        COMMANDED, {}, {0: fix}, now_mono=100.0, max_fix_age_s=0.25
    )
    assert obs["j0.pos"] == 64.0                       # commanded, untouched
    assert obs["j0.observed_deg"] == 61.0
    assert obs["j0.residual_deg"] == pytest.approx(61.0 - 64.0)
    assert obs["j0.obs_source"] == source
    assert obs["j0.obs_age_s"] == pytest.approx(0.1)


def test_a_fresh_fix_on_one_joint_leaves_the_others_unobserved(observation):
    fix = observation.JointFix(
        angle_deg=61.0, source=observation.OBS_SOURCE_MARKER, captured_mono=99.9
    )
    obs = observation.build_observation(COMMANDED, {}, {0: fix}, now_mono=100.0)
    for jid in (1, 3, 4, 5, 6):
        assert obs[observation.joint_key(jid, "obs_source")] == observation.OBS_SOURCE_NONE
        assert math.isnan(obs[observation.joint_key(jid, "observed_deg")])


def test_the_residual_is_observed_minus_commanded_in_that_order(observation):
    # Sign matters: a positive residual means the joint is FURTHER ALONG than it
    # was told to be. Flipping it would invert every correction built on top.
    fix = observation.JointFix(
        angle_deg=70.0, source=observation.OBS_SOURCE_MARKER, captured_mono=100.0
    )
    obs = observation.build_observation(COMMANDED, {}, {0: fix}, now_mono=100.0)
    assert obs["j0.residual_deg"] == pytest.approx(70.0 - 64.0)
    assert obs["j0.residual_deg"] > 0


def test_a_fix_exactly_at_the_age_limit_still_publishes_a_residual(observation):
    # The comparison is `age <= max_fix_age_s`; the boundary is inclusive.
    fix = observation.JointFix(
        angle_deg=61.0, source=observation.OBS_SOURCE_MARKER, captured_mono=99.75
    )
    obs = observation.build_observation(
        COMMANDED, {}, {0: fix}, now_mono=100.0, max_fix_age_s=0.25
    )
    assert obs["j0.obs_age_s"] == pytest.approx(0.25)
    assert obs["j0.obs_source"] == observation.OBS_SOURCE_MARKER
    assert not math.isnan(obs["j0.residual_deg"])


# --------------------------------------------------------------------------
# build_observation -- staleness
# --------------------------------------------------------------------------


@pytest.mark.parametrize("source_name", ["OBS_SOURCE_MARKER", "OBS_SOURCE_HUMAN"])
def test_a_stale_fix_keeps_its_value_and_age_but_loses_its_residual(
    observation, source_name
):
    # The three halves of one rule, asserted together so the pairing cannot rot:
    # the observed value is still informative, the age is still informative, and
    # a residual computed across a stale gap is not -- so it is refused.
    fix = observation.JointFix(
        angle_deg=61.0,
        source=getattr(observation, source_name),
        captured_mono=95.0,
    )
    obs = observation.build_observation(
        COMMANDED, {}, {0: fix}, now_mono=100.0, max_fix_age_s=0.25
    )
    assert obs["j0.observed_deg"] == 61.0                    # kept
    assert obs["j0.obs_age_s"] == pytest.approx(5.0)         # kept
    assert math.isnan(obs["j0.residual_deg"])                # lost
    assert obs["j0.obs_source"] == observation.OBS_SOURCE_STALE   # demoted


def test_a_stale_residual_is_nan_rather_than_a_stale_number(observation):
    fix = observation.JointFix(
        angle_deg=61.0, source=observation.OBS_SOURCE_MARKER, captured_mono=0.0
    )
    obs = observation.build_observation(COMMANDED, {}, {0: fix}, now_mono=100.0)
    assert obs["j0.residual_deg"] != 61.0 - 64.0
    assert math.isnan(obs["j0.residual_deg"])


def test_staleness_demotes_the_source_but_never_erases_the_observation(observation):
    fix = observation.JointFix(
        angle_deg=12.5, source=observation.OBS_SOURCE_HUMAN, captured_mono=0.0
    )
    obs = observation.build_observation(COMMANDED, {}, {3: fix}, now_mono=100.0)
    assert obs["j3.obs_source"] == observation.OBS_SOURCE_STALE
    assert obs["j3.observed_deg"] == 12.5
    assert obs["j3.obs_source"] != observation.OBS_SOURCE_NONE


# --------------------------------------------------------------------------
# build_observation -- occlusion and the "no observer configured" fix
# --------------------------------------------------------------------------


def test_an_occluded_fix_reports_occluded_with_no_value_and_no_residual(observation):
    fix = observation.JointFix(
        angle_deg=math.nan,
        source=observation.OBS_SOURCE_OCCLUDED,
        captured_mono=99.9,
    )
    obs = observation.build_observation(COMMANDED, {}, {0: fix}, now_mono=100.0)
    assert obs["j0.obs_source"] == observation.OBS_SOURCE_OCCLUDED
    assert math.isnan(obs["j0.observed_deg"])
    assert math.isnan(obs["j0.residual_deg"])
    assert obs["j0.obs_age_s"] == pytest.approx(0.1)


def test_an_old_occluded_fix_stays_occluded_and_is_not_demoted_to_stale(observation):
    # Staleness demotion applies only where there is a value to be stale about.
    # "I looked and could not see it" does not become "I saw it a while ago".
    fix = observation.JointFix(
        angle_deg=math.nan,
        source=observation.OBS_SOURCE_OCCLUDED,
        captured_mono=0.0,
    )
    obs = observation.build_observation(COMMANDED, {}, {0: fix}, now_mono=100.0)
    assert obs["j0.obs_source"] == observation.OBS_SOURCE_OCCLUDED
    assert obs["j0.obs_age_s"] == pytest.approx(100.0)


def test_a_fix_reporting_none_yields_no_value_but_a_real_age(observation):
    fix = observation.JointFix(
        angle_deg=61.0, source=observation.OBS_SOURCE_NONE, captured_mono=99.5
    )
    obs = observation.build_observation(COMMANDED, {}, {0: fix}, now_mono=100.0)
    assert obs["j0.obs_source"] == observation.OBS_SOURCE_NONE
    assert math.isnan(obs["j0.observed_deg"])
    assert math.isnan(obs["j0.residual_deg"])
    assert obs["j0.obs_age_s"] == pytest.approx(0.5)


def test_the_source_column_is_finite_in_every_state(observation):
    # The mask guarantee: a consumer always has a finite answer to "may I trust
    # this row?", whatever happened upstream.
    states = [
        None,
        {0: observation.JointFix(61.0, observation.OBS_SOURCE_MARKER, 99.9)},
        {0: observation.JointFix(61.0, observation.OBS_SOURCE_MARKER, 0.0)},
        {0: observation.JointFix(61.0, observation.OBS_SOURCE_HUMAN, 99.9)},
        {0: observation.JointFix(math.nan, observation.OBS_SOURCE_OCCLUDED, 99.9)},
        {0: observation.JointFix(math.nan, observation.OBS_SOURCE_NONE, 99.9)},
    ]
    for fixes in states:
        obs = observation.build_observation(COMMANDED, {}, fixes, now_mono=100.0)
        for jid in observation.JOINT_IDS:
            src = obs[observation.joint_key(jid, "obs_source")]
            assert not math.isnan(src), (fixes, jid)


# --------------------------------------------------------------------------
# build_observation -- the reserved joint at assembly time
# --------------------------------------------------------------------------


def test_a_fix_for_the_reserved_joint_produces_no_key(observation):
    # An observer that sees D5 has nowhere to report it as its own joint --
    # unaddressable by construction means unaddressable at the assembly layer too.
    fix = observation.JointFix(
        angle_deg=120.0, source=observation.OBS_SOURCE_MARKER, captured_mono=99.9
    )
    obs = observation.build_observation(
        COMMANDED, {}, {observation.RESERVED_JOINT_ID: fix}, now_mono=100.0
    )
    assert not any(k.startswith("j2.") for k in obs)
    assert len(obs) == 37


def test_a_commanded_entry_for_the_reserved_joint_is_ignored(observation):
    commanded = dict(COMMANDED)
    commanded[observation.RESERVED_JOINT_ID] = 120.0
    obs = observation.build_observation(commanded, {}, None, now_mono=100.0)
    assert not any(k.startswith("j2.") for k in obs)


# --------------------------------------------------------------------------
# build_observation -- the pair-disagreement tail
# --------------------------------------------------------------------------


def test_pair_disagreement_is_nan_until_a_two_marker_observer_exists(observation):
    obs = observation.build_observation(COMMANDED, {}, None, now_mono=100.0)
    assert math.isnan(obs[observation.PAIR_DISAGREE_KEY])


def test_pair_disagreement_is_read_from_joint_1s_fix_only(observation):
    fix = observation.JointFix(
        angle_deg=45.0,
        source=observation.OBS_SOURCE_MARKER,
        captured_mono=99.9,
        pair_disagree_deg=3.5,
    )
    obs = observation.build_observation(COMMANDED, {}, {1: fix}, now_mono=100.0)
    assert obs[observation.PAIR_DISAGREE_KEY] == pytest.approx(3.5)


def test_pair_disagreement_reported_on_another_joint_is_discarded(observation):
    fix = observation.JointFix(
        angle_deg=45.0,
        source=observation.OBS_SOURCE_MARKER,
        captured_mono=99.9,
        pair_disagree_deg=3.5,
    )
    obs = observation.build_observation(COMMANDED, {}, {3: fix}, now_mono=100.0)
    assert math.isnan(obs[observation.PAIR_DISAGREE_KEY])


def test_a_joint_1_fix_without_a_disagreement_leaves_the_tail_nan(observation):
    fix = observation.JointFix(
        angle_deg=45.0, source=observation.OBS_SOURCE_MARKER, captured_mono=99.9
    )
    obs = observation.build_observation(COMMANDED, {}, {1: fix}, now_mono=100.0)
    assert math.isnan(obs[observation.PAIR_DISAGREE_KEY])


def test_pair_disagreement_of_zero_is_kept_as_zero_not_turned_into_nan(observation):
    # A genuine two-marker reading of 0 means "the horns agree" and is a real
    # result. It must not be confused with "nobody has looked".
    fix = observation.JointFix(
        angle_deg=45.0,
        source=observation.OBS_SOURCE_MARKER,
        captured_mono=99.9,
        pair_disagree_deg=0.0,
    )
    obs = observation.build_observation(COMMANDED, {}, {1: fix}, now_mono=100.0)
    assert obs[observation.PAIR_DISAGREE_KEY] == 0.0


# --------------------------------------------------------------------------
# A disabled joint, as far as this layer can see it
#
# The rule that a DISABLED joint's `SET=` is bookkeeping rather than state lives
# in `emre_arm.get_observation()`, which imports lerobot and is not importable
# here. What IS reachable is its consequence: that layer substitutes NaN for a
# disabled joint's commanded angle, and these tests pin what happens to a NaN
# commanded angle once it arrives.
# --------------------------------------------------------------------------


def test_a_nan_commanded_angle_stays_nan_and_is_not_defaulted(observation):
    commanded = dict(COMMANDED)
    commanded[6] = math.nan
    obs = observation.build_observation(commanded, {}, None, now_mono=100.0)
    assert math.isnan(obs["j6.pos"])
    assert obs["j6.pos"] != 90.0     # the firmware's boot seed, not a command


def test_a_commanded_map_missing_a_joint_is_a_hard_error_not_a_silent_nan(observation):
    # Stricter than it looks, and worth knowing: `build_observation` demands an
    # entry for EVERY addressable joint. A caller that OMITS a disabled joint
    # instead of substituting NaN gets a KeyError, not a quietly incomplete row.
    incomplete = {j: 90.0 for j in observation.JOINT_IDS if j != 6}
    with pytest.raises(KeyError):
        observation.build_observation(incomplete, {}, None, now_mono=100.0)


def test_a_nan_commanded_angle_poisons_its_residual_rather_than_hiding_it(observation):
    # A residual against an unknown command is meaningless, and must present as
    # meaningless rather than as the observed angle.
    commanded = dict(COMMANDED)
    commanded[0] = math.nan
    fix = observation.JointFix(
        angle_deg=61.0, source=observation.OBS_SOURCE_MARKER, captured_mono=99.9
    )
    obs = observation.build_observation(commanded, {}, {0: fix}, now_mono=100.0)
    assert obs["j0.observed_deg"] == 61.0
    assert math.isnan(obs["j0.residual_deg"])
    assert obs["j0.obs_source"] == observation.OBS_SOURCE_MARKER


# --------------------------------------------------------------------------
# JointFix
# --------------------------------------------------------------------------


def test_jointfix_is_frozen(observation):
    fix = observation.JointFix(1.0, observation.OBS_SOURCE_MARKER, 0.0)
    with pytest.raises(Exception):
        fix.angle_deg = 2.0


def test_jointfix_defaults_pair_disagreement_to_absent(observation):
    fix = observation.JointFix(1.0, observation.OBS_SOURCE_MARKER, 0.0)
    assert fix.pair_disagree_deg is None


def test_observation_module_imports_nothing_outside_the_standard_library(observation):
    # The claim that makes board-free testing possible in the first place. The
    # stronger form of this check -- that nothing heavy is in sys.modules after a
    # load -- lives in test_isolation.py; this one names the file that broke it.
    import inspect

    imports = [
        line.strip()
        for line in inspect.getsource(observation).splitlines()
        if line.strip().startswith(("import ", "from "))
    ]
    assert imports, "expected at least the stdlib imports"
    for line in imports:
        for heavy in ("lerobot", "cv2", "numpy", "serial", "torch"):
            assert heavy not in line, line
