#!/usr/bin/env python3
"""Config-consistency tests for the multi-arm (ur16e / ur3e) support.

These are pure file/geometry checks — no ROS graph, no hardware, no YOLO — so
they run in milliseconds and are safe to put in front of the slow integration
suites:

    python3 -m pytest test/test_robot_profiles.py -v

What they protect:
  - every arm in PROFILES has a complete config set in both packages
  - no configured pose sits outside its own arm's reach envelope (the failure
    mode that makes a UR16e cell "work" right up until the arm stalls mid-place)
  - the E-Pick tool offset is present in every URDF, on every arm
  - the two cells agree on the things that are model-driven, not arm-driven
"""

import math
import os

import pytest
import yaml

from ament_index_python.packages import get_package_share_directory

from recycle_bot.robot_profile import PROFILES, RobotProfile


def _share(pkg):
    return get_package_share_directory(pkg)


def _robot_config_dir(ur_type):
    return os.path.join(_share("recycle_bot"), "config", ur_type)


def _moveit_config_dir(ur_type):
    return os.path.join(_share("recycle_bot_moveit_config"), "config", ur_type)


def _load(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _distance(position):
    return math.sqrt(sum(float(v) * float(v) for v in position))


ALL_ARMS = sorted(PROFILES)


@pytest.mark.parametrize("ur_type", ALL_ARMS)
def test_application_config_present(ur_type):
    """Each arm needs its own calibration, sorting sequence and cell geometry."""
    for filename in ("calibration.yaml", "sorting_sequence.yaml", "cell.yaml"):
        path = os.path.join(_robot_config_dir(ur_type), filename)
        assert os.path.exists(path), f"missing {ur_type} config: {path}"


@pytest.mark.parametrize("ur_type", ALL_ARMS)
def test_moveit_config_present(ur_type):
    """Each arm needs its own URDF, SRDF, joint limits and Pilz limits."""
    expected = (
        f"{ur_type}.urdf.xacro",
        f"{ur_type}.srdf",
        "joint_limits.yaml",
        "pilz_cartesian_limits.yaml",
        "initial_positions.yaml",
    )
    for filename in expected:
        path = os.path.join(_moveit_config_dir(ur_type), filename)
        assert os.path.exists(path), f"missing {ur_type} MoveIt config: {path}"


@pytest.mark.parametrize("ur_type", ALL_ARMS)
def test_configured_poses_within_reach(ur_type):
    """No pose may sit outside its own arm's reach envelope.

    This is the regression guard for the port: copying a pose from the UR16e's
    cell into the UR3e's config is the single easiest mistake to make, and
    without this check it surfaces as an opaque MoveIt planning failure partway
    through a pick-place cycle rather than as a config error.
    """
    prof = PROFILES[ur_type]
    data = _load(os.path.join(_robot_config_dir(ur_type), "sorting_sequence.yaml"))

    poses = []
    if data.get("neutral_pose"):
        poses.append(("neutral_pose", data["neutral_pose"]["position"]))
    for name, bin_data in (data.get("bins") or {}).items():
        poses.append((f"bins.{name}", bin_data["position"]))

    assert poses, f"{ur_type} sorting_sequence.yaml defines no poses at all"

    over = [
        (name, _distance(pos))
        for name, pos in poses
        if _distance(pos) > prof.planning_reach_m
    ]
    assert not over, (
        f"{ur_type} pose(s) outside the {prof.planning_reach_m:.3f} m planning "
        f"envelope: " + ", ".join(f"{n} at {d:.3f} m" for n, d in over)
    )


@pytest.mark.parametrize("ur_type", ALL_ARMS)
def test_every_routed_bin_exists(ur_type):
    """bin_routing and default_bin must only name bins that are actually defined."""
    data = _load(os.path.join(_robot_config_dir(ur_type), "sorting_sequence.yaml"))
    bins = set((data.get("bins") or {}).keys())
    routing = data.get("bin_routing") or {}

    unknown = {label: b for label, b in routing.items() if b not in bins}
    assert not unknown, f"{ur_type} routes labels to undefined bins: {unknown}"

    default_bin = data.get("default_bin")
    assert default_bin in bins, (
        f"{ur_type} default_bin '{default_bin}' is not defined under 'bins'"
    )


@pytest.mark.parametrize("ur_type", ALL_ARMS)
def test_gripper_tool_offset_present(ur_type):
    """The E-Pick's 150 mm flange->tool0 offset must be in every arm's URDF.

    It is a hand-applied deviation from stock ur_description, so regenerating a
    URDF from xacro silently drops it — and losing it moves every pick 150 mm
    off target without any error.
    """
    path = os.path.join(_moveit_config_dir(ur_type), f"{ur_type}.urdf.xacro")
    with open(path, "r") as f:
        urdf = f.read()

    marker = 'xyz="0.150 0 0"'
    assert marker in urdf, (
        f"{ur_type}.urdf.xacro is missing the E-Pick tool offset ({marker}) on "
        "the flange-tool0 joint"
    )


@pytest.mark.parametrize("ur_type", ALL_ARMS)
def test_planning_group_is_ur_arm(ur_type):
    """rec_bot_control hardcodes get_planning_component("ur_arm")."""
    path = os.path.join(_moveit_config_dir(ur_type), f"{ur_type}.srdf")
    with open(path, "r") as f:
        srdf = f.read()
    assert '<group name="ur_arm">' in srdf, (
        f"{ur_type}.srdf must define the group 'ur_arm' — upstream calls it "
        "'ur_manipulator', which rec_bot_control will not find"
    )


def test_bin_routing_identical_across_arms():
    """Routing is a property of the YOLO model's classes, not of the arm.

    If the two cells disagree, one of them was updated after a model swap and the
    other was not — which shows up as objects going to the wrong bin on one arm
    only.
    """
    routings = {}
    for ur_type in ALL_ARMS:
        data = _load(os.path.join(_robot_config_dir(ur_type), "sorting_sequence.yaml"))
        routings[ur_type] = data.get("bin_routing") or {}

    reference_arm = ALL_ARMS[0]
    reference = routings[reference_arm]
    for ur_type, routing in routings.items():
        assert routing == reference, (
            f"bin_routing differs between {reference_arm} and {ur_type}; both must "
            f"match the active YOLO model's class list.\n"
            f"  only in {reference_arm}: {set(reference) - set(routing)}\n"
            f"  only in {ur_type}: {set(routing) - set(reference)}"
        )


def test_profiles_are_ordered_by_reach():
    """Sanity check on the profile table itself.

    A transposed reach/payload pair would silently widen the UR3e's envelope,
    defeating every reach check above.
    """
    assert PROFILES["ur3e"].max_reach_m < PROFILES["ur16e"].max_reach_m
    assert PROFILES["ur3e"].payload_kg < PROFILES["ur16e"].payload_kg
    for ur_type, prof in PROFILES.items():
        assert isinstance(prof, RobotProfile)
        assert 0.0 < prof.reach_derate <= 1.0, f"{ur_type} derate out of range"
        assert prof.planning_reach_m < prof.max_reach_m
