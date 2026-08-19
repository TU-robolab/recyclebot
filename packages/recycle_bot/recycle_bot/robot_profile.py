#!/usr/bin/env python3
"""Single source of truth for which UR arm the stack is driving.

Everything that differs between the UR16e and the UR3e is resolved through this
module: the MoveIt configuration, the per-robot YAML directory under
recycle_bot/config/<ur_type>/, and the reach envelope used to reject poses the
arm physically cannot get to.

Imported from two very different contexts, so it deliberately keeps the MoveIt
import lazy:
  - launch files (``build_moveit_config``), which run before any node starts
  - runtime nodes (``profile``, ``config_path``), which must not pull in
    moveit_configs_utils just to read a YAML path
"""

import os

from ament_index_python.packages import get_package_share_directory

MOVEIT_CONFIG_PACKAGE = "recycle_bot_moveit_config"

DEFAULT_UR_TYPE = "ur16e"


class RobotProfile:
    """Physical envelope of one UR arm.

    Reach is measured from the SHOULDER, not from base_link, and against the
    arm's real geometry rather than the datasheet number. Both corrections came
    from measuring the UR3e cell on 2026-08-18, where three of five poses the arm
    had physically just achieved were rejected by the original check:

      * Origin. The working envelope is centred on the shoulder joint, which
        sits ``shoulder_height_m`` above base_link. Measuring from base_link
        overestimates the distance to anything high up — a pose at z = 0.65 read
        as 0.674 m from base_link but only 0.529 m from the shoulder.

      * Radius. ``max_reach_m`` (the datasheet "reach") is the horizontal working
        radius, NOT the maximum flange distance. The UR3e is sold as 500 mm but
        reaches 600 mm flange-to-shoulder, and was measured at 529 mm in this
        cell. Using the datasheet figure rejected valid poses.

    ``max_tool_reach_m`` is therefore the real quantity: the furthest tool0 can
    get from the shoulder, sampled over joint space from this arm's own URDF.
    tool0 is what MoveIt plans to and what sorting_sequence.yaml poses mean, so
    it already includes the E-Pick's 150 mm offset — change the end effector and
    this must be re-derived. test_reach_envelope_matches_urdf does that
    re-derivation and fails if the constant drifts.

    ``planning_reach_m`` derates it, since a pose at full extension is reachable
    at exactly one orientation and useless in practice.
    """

    def __init__(
        self,
        ur_type,
        max_reach_m,
        payload_kg,
        dashboard_model,
        shoulder_height_m,
        max_tool_reach_m,
        reach_derate=0.85,
    ):
        self.ur_type = ur_type
        self.max_reach_m = max_reach_m
        self.payload_kg = payload_kg
        self.shoulder_height_m = shoulder_height_m
        self.max_tool_reach_m = max_tool_reach_m
        # What this arm's dashboard server returns for "get robot model". It is
        # the product FAMILY, not the variant: a UR3e and a CB3 UR3 both answer
        # "UR3". That is still enough to separate a UR3e from a UR16e, which is
        # the dangerous confusion; e-Series vs CB3 is settled by the kinematic
        # calibration instead (see calibration_check.py).
        self.dashboard_model = dashboard_model
        self.reach_derate = reach_derate

    @property
    def planning_reach_m(self):
        """Envelope the reach check enforces, measured from the shoulder."""
        return self.max_tool_reach_m * self.reach_derate

    @property
    def urdf_path(self):
        return f"config/{self.ur_type}/{self.ur_type}.urdf.xacro"

    @property
    def srdf_path(self):
        return f"config/{self.ur_type}/{self.ur_type}.srdf"

    @property
    def joint_limits_path(self):
        return f"config/{self.ur_type}/joint_limits.yaml"

    @property
    def pilz_cartesian_limits_path(self):
        return f"config/{self.ur_type}/pilz_cartesian_limits.yaml"

    def __repr__(self):
        return (
            f"RobotProfile({self.ur_type}, datasheet_reach={self.max_reach_m}m, "
            f"max_tool_reach={self.max_tool_reach_m:.3f}m, "
            f"planning_reach={self.planning_reach_m:.3f}m from shoulder "
            f"@z={self.shoulder_height_m:.3f}m, payload={self.payload_kg}kg)"
        )


# Reach and payload from the UR datasheets. Adding an arm here plus a
# config/<ur_type>/ directory in both recycle_bot and recycle_bot_moveit_config
# is the whole port — no code changes needed.
# shoulder_height_m and max_tool_reach_m are derived from each arm's URDF by
# sampling joint space; see test_reach_envelope_matches_urdf, which recomputes
# them and fails if these constants go stale. They are stored rather than
# computed at startup because the sampling is far too slow to run per launch.
PROFILES = {
    "ur16e": RobotProfile(
        "ur16e", max_reach_m=0.900, payload_kg=16.0, dashboard_model="UR16",
        shoulder_height_m=0.18070, max_tool_reach_m=1.1476,
    ),
    "ur3e": RobotProfile(
        "ur3e", max_reach_m=0.500, payload_kg=3.0, dashboard_model="UR3",
        shoulder_height_m=0.15185, max_tool_reach_m=0.7318,
    ),
}


def resolve_ur_type(explicit=None):
    """Resolve the target arm from an explicit value, then $UR_TYPE, then default.

    Raises ValueError on an unknown arm rather than silently falling back — a
    typo'd ur_type that quietly launched a UR16e against a UR3e cell would drive
    the arm into its own workspace limits on the first motion.
    """
    ur_type = explicit or os.environ.get("UR_TYPE") or DEFAULT_UR_TYPE
    ur_type = ur_type.strip().lower()
    if ur_type not in PROFILES:
        raise ValueError(
            f"Unknown ur_type '{ur_type}'. Known arms: {sorted(PROFILES)}. "
            f"Add a RobotProfile in {__file__} plus matching config/<ur_type>/ "
            "directories in the recycle_bot and recycle_bot_moveit_config packages."
        )
    return ur_type


def profile(ur_type=None):
    """RobotProfile for the given (or resolved) arm."""
    return PROFILES[resolve_ur_type(ur_type)]


# Fallback used only when neither a per-arm nor a generic IP is configured.
DEFAULT_ROBOT_IP = "192.168.1.102"


def robot_ip(ur_type=None):
    """IP address for an arm's controller.

    Resolution order, most specific first:

      1. ``<UR_TYPE>_ROBOT_IP``  e.g. UR3E_ROBOT_IP=192.168.1.102
      2. ``REMOTE_IP``           the single-robot variable export_env.sh has
                                 always written; still correct when the cell has
                                 one arm at a time, including when two arms share
                                 an address because they are swapped in and out
      3. ``DEFAULT_ROBOT_IP``

    Per-arm variables matter once two arms are on the network simultaneously: a
    single REMOTE_IP cannot describe both, and pointing a UR16e configuration at
    a UR3e is exactly the mistake verify_robot_model() exists to stop.
    """
    ur_type = resolve_ur_type(ur_type)
    specific = os.environ.get(f"{ur_type.upper()}_ROBOT_IP")
    if specific:
        return specific.strip()
    generic = os.environ.get("REMOTE_IP")
    if generic:
        return generic.strip()
    return DEFAULT_ROBOT_IP


def config_path(ur_type, filename):
    """Absolute path to a per-robot YAML in the installed recycle_bot share dir.

    Falls back to the shared (robot-independent) config/ location when no
    per-robot copy exists, so files that genuinely do not vary by arm need not be
    duplicated into every robot directory.
    """
    share = get_package_share_directory("recycle_bot")
    specific = os.path.join(share, "config", resolve_ur_type(ur_type), filename)
    if os.path.exists(specific):
        return specific
    return os.path.join(share, "config", filename)


def kinematics_params_file(ur_type=None):
    """Path to this arm's teach-pendant kinematic calibration, or None.

    my_robot_calibration.yaml is unique to one physical robot, so it is never
    shared between arms and never invented. Returns None when the file is absent
    so callers can decide whether to warn (bring-up) or refuse (hardware runs);
    ur_robot_driver falls back to ur_description's nominal kinematics.
    """
    ur_type = resolve_ur_type(ur_type)
    path = os.path.join(
        get_package_share_directory("recycle_bot"),
        "config",
        ur_type,
        "my_robot_calibration.yaml",
    )
    return path if os.path.exists(path) else None


def build_moveit_config(ur_type=None):
    """Build the MoveIt configuration for the resolved arm.

    Replaces the block that was copy-pasted into all six launch files. Note that
    joint limits and Pilz Cartesian limits are passed explicitly: they now live
    under config/<ur_type>/, so MoveItConfigsBuilder's convention-based
    auto-discovery of config/joint_limits.yaml would silently find nothing and
    fall back to the URDF's own limits.
    """
    from moveit_configs_utils import MoveItConfigsBuilder

    prof = profile(ur_type)
    moveit_config_share = get_package_share_directory(MOVEIT_CONFIG_PACKAGE)

    return (
        MoveItConfigsBuilder(
            robot_name=prof.ur_type, package_name=MOVEIT_CONFIG_PACKAGE
        )
        .robot_description(file_path=prof.urdf_path)
        .robot_description_semantic(file_path=prof.srdf_path)
        .joint_limits(file_path=prof.joint_limits_path)
        .pilz_cartesian_limits(file_path=prof.pilz_cartesian_limits_path)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=os.path.join(moveit_config_share, "config", "moveit_cpp.yaml")
        )
        .to_moveit_configs()
    )
