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
  - the E-Pick is modelled for collision, and exempt only from wrist_3_link
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


def _distance(position, shoulder_height_m=0.0):
    """Distance from the shoulder joint, matching rec_bot_control.reach_distance.

    The working envelope is centred on the shoulder, not on base_link. Measuring
    from base_link rejects poses the arm can reach — see RobotProfile's docstring
    for the measured evidence.
    """
    x, y, z = (float(v) for v in position)
    dz = z - shoulder_height_m
    return math.sqrt(x * x + y * y + dz * dz)


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
        (name, _distance(pos, prof.shoulder_height_m))
        for name, pos in poses
        if _distance(pos, prof.shoulder_height_m) > prof.planning_reach_m
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
def test_gripper_collision_geometry_present(ur_type):
    """The E-Pick must be modelled for collision, and only exempt from wrist_3.

    Without the "epick" link MoveIt believes the flange is bare and will fold the
    gripper into the arm. Like the tool0 offset, it is a hand edit that a URDF
    regeneration silently drops. The SRDF side guards the opposite mistake: the
    Setup Assistant marking epick "Never" against the upper arm or forearm,
    which random sampling shows it hits in ~11% of states each.
    """
    import xml.etree.ElementTree as ET

    root = ET.parse(
        os.path.join(_moveit_config_dir(ur_type), f"{ur_type}.urdf.xacro")
    ).getroot()

    link = root.find("link[@name='epick']")
    assert link is not None, f"{ur_type}.urdf.xacro has no 'epick' link"
    collisions = link.findall("collision")
    assert collisions, f"{ur_type} epick link has no collision geometry"

    joint = next(
        (j for j in root.findall("joint") if j.find("child").get("link") == "epick"),
        None,
    )
    assert joint is not None and joint.find("parent").get("link") == "flange", (
        f"{ur_type} epick must be fixed to the flange"
    )

    # The model must stop short of tool0 (x = 0.150 in the flange frame): the
    # cup lip touches the object, so modelling it would reject every pick's final
    # LIN descent. Each cylinder is pitched onto flange X, so it spans
    # origin_x +/- length/2.
    far_end = max(
        float(c.find("origin").get("xyz").split()[0])
        + float(c.find("geometry/cylinder").get("length")) / 2.0
        for c in collisions
    )
    assert far_end < 0.150, (
        f"{ur_type} epick geometry reaches x={far_end:.3f}, at or past tool0 (0.150)"
    )

    srdf = ET.parse(
        os.path.join(_moveit_config_dir(ur_type), f"{ur_type}.srdf")
    ).getroot()
    exempt = {
        (d.get("link2") if d.get("link1") == "epick" else d.get("link1"))
        for d in srdf.findall("disable_collisions")
        if "epick" in (d.get("link1"), d.get("link2"))
    }
    assert exempt == {"wrist_3_link"}, (
        f"{ur_type}.srdf exempts epick from {sorted(exempt)}; only wrist_3_link "
        "(which it is bolted to) may be disabled"
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


@pytest.mark.parametrize("ur_type", ALL_ARMS)
def test_calibration_matches_its_arm(ur_type):
    """A present my_robot_calibration.yaml must belong to the arm it is filed under.

    Skipped when absent: an arm in bring-up legitimately has no calibration yet,
    and the launch files already warn and fall back to nominal kinematics. But
    once the file exists it must be the right one — copying another arm's export
    shifts every computed pose by a fixed offset, which looks like a camera
    calibration error rather than a kinematics one.
    """
    from recycle_bot.calibration_check import check_calibration

    path = os.path.join(_robot_config_dir(ur_type), "my_robot_calibration.yaml")
    if not os.path.exists(path):
        pytest.skip(
            f"no calibration for {ur_type} yet — export it from the teach pendant "
            f"to {path}"
        )

    problems, info = check_calibration(path, ur_type)
    assert not problems, (
        f"{ur_type} calibration at {path} is not usable:\n  "
        + "\n  ".join(problems)
    )


def test_calibration_checker_rejects_the_wrong_arm():
    """The checker itself must catch a cross-filed calibration.

    Guards the guard: if arm identification silently stopped working, the test
    above would pass on any file and the whole check would be decorative.
    """
    from recycle_bot.calibration_check import check_calibration

    donor = None
    for ur_type in ALL_ARMS:
        candidate = os.path.join(_robot_config_dir(ur_type), "my_robot_calibration.yaml")
        if os.path.exists(candidate):
            donor = (ur_type, candidate)
            break
    if donor is None:
        pytest.skip("no calibration file available to cross-check with")

    donor_arm, donor_path = donor
    others = [a for a in ALL_ARMS if a != donor_arm]
    if not others:
        pytest.skip("only one arm configured; nothing to cross-check against")

    for other in others:
        problems, info = check_calibration(donor_path, other)
        assert problems, (
            f"checker accepted {donor_arm}'s calibration as {other}'s — "
            "arm identification is not working"
        )
        assert info.get("matched_arm") == donor_arm, (
            f"checker failed to identify the file as {donor_arm} "
            f"(said {info.get('matched_arm')!r})"
        )


def test_no_circular_package_dependencies():
    """The repo's packages must order topologically.

    colcon refuses to build at all when two packages depend on each other, with
    an error that names the cycle but not the offending <depend> tag. The live
    case: recycle_bot needs recycle_bot_moveit_config to resolve the MoveIt
    config, so recycle_bot_moveit_config must stay a leaf and must not depend
    back on recycle_bot.

    Reads package.xml from the source tree rather than the install space, since
    a cycle prevents the install space from existing in the first place.
    """
    import collections
    import pathlib
    import xml.etree.ElementTree as ET

    # test/ -> test_suite/ -> repo root
    repo = pathlib.Path(__file__).resolve().parents[2]
    manifests = list((repo / "packages").glob("*/package.xml"))
    manifests.append(repo / "test_suite" / "package.xml")
    manifests = [m for m in manifests if m.exists()]
    if not manifests:
        pytest.skip("source tree not available (running from an install space)")

    graph = {}
    for manifest in manifests:
        root = ET.parse(manifest).getroot()
        name = root.find("name").text.strip()
        deps = set()
        for tag in ("depend", "exec_depend", "build_depend", "buildtool_depend"):
            for d in root.findall(tag):
                if d.text:
                    deps.add(d.text.strip())
        graph[name] = deps

    local = set(graph)
    graph = {p: (d & local) for p, d in graph.items()}

    indegree = {p: len(graph[p]) for p in graph}
    ready = collections.deque(p for p in graph if indegree[p] == 0)
    ordered = []
    while ready:
        p = ready.popleft()
        ordered.append(p)
        for q in graph:
            if p in graph[q]:
                indegree[q] -= 1
                if indegree[q] == 0:
                    ready.append(q)

    unresolved = sorted(set(graph) - set(ordered))
    assert not unresolved, (
        "circular package dependency — colcon cannot order these:\n  "
        + "\n  ".join(f"{p} -> {sorted(graph[p])}" for p in unresolved)
    )


class _FakeDashboard:
    """Minimal stand-in for a UR dashboard server, so these tests need no robot.

    Speaks just enough of the protocol: a greeting banner, then a canned reply to
    "get robot model".
    """

    def __init__(self, model):
        import socket
        import threading

        self.model = model
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.listen(1)
        self.port = self._sock.getsockname()[1]
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _serve(self):
        try:
            conn, _ = self._sock.accept()
        except OSError:
            return
        with conn:
            try:
                conn.sendall(b"Connected: Universal Robots Dashboard Server\n")
                conn.recv(4096)
                conn.sendall(self.model.encode() + b"\n")
            except OSError:
                pass

    def close(self):
        self._sock.close()


@pytest.mark.parametrize(
    "connected_model,expected_ur_type,should_pass",
    [
        ("UR3", "ur3e", True),
        ("UR16", "ur16e", True),
        ("UR3", "ur16e", False),   # the live hazard: ur16e config, ur3e on the wire
        ("UR16", "ur3e", False),
        ("UR5", "ur3e", False),    # a model no profile claims
    ],
)
def test_robot_model_matching(connected_model, expected_ur_type, should_pass):
    """Model string -> ur_type resolution, without touching the network."""
    from recycle_bot.robot_identity import model_to_ur_types

    matches = model_to_ur_types(connected_model)
    assert (expected_ur_type in matches) == should_pass, (
        f"model '{connected_model}' resolved to {matches}; expected "
        f"{expected_ur_type} to {'match' if should_pass else 'not match'}"
    )


def test_query_robot_model_reads_the_dashboard():
    """The dashboard client speaks the protocol correctly against a fake server."""
    from recycle_bot.robot_identity import query_robot_model

    server = _FakeDashboard("UR3")
    try:
        model = query_robot_model("127.0.0.1", timeout=5, port=server.port)
    finally:
        server.close()
    assert model == "UR3"


def test_unreachable_robot_warns_rather_than_blocking():
    """A controller we cannot reach must not block a launch.

    The driver reports a genuine connection failure far better than a timed-out
    pre-flight check would, and failing here on a transient network blip would
    make the check something people routinely disable.
    """
    from recycle_bot.robot_identity import verify_robot_model

    # 127.0.0.1 with nothing listening on the dashboard port: refused instantly.
    ok, message = verify_robot_model("ur3e", ip="127.0.0.1", timeout=1, strict=False)
    assert ok, "an unreachable controller must not fail the check"
    assert "could not reach" in message


def test_every_profile_has_a_distinct_dashboard_model():
    """Two arms sharing a model string would make the check unable to separate them.

    Not fatal on its own — ur3e and ur3 would legitimately collide if both were
    configured — but it must be a deliberate choice, so assert the current set is
    unambiguous.
    """
    seen = {}
    for ur_type, prof in PROFILES.items():
        seen.setdefault(prof.dashboard_model, []).append(ur_type)
    ambiguous = {m: arms for m, arms in seen.items() if len(arms) > 1}
    assert not ambiguous, (
        f"these arms cannot be told apart by dashboard model: {ambiguous}. "
        "verify_robot_model will accept either one for the other."
    )


def test_robot_ip_resolution_order(monkeypatch):
    """Per-arm variable beats REMOTE_IP beats the built-in default."""
    from recycle_bot import robot_profile

    monkeypatch.delenv("UR3E_ROBOT_IP", raising=False)
    monkeypatch.delenv("UR16E_ROBOT_IP", raising=False)
    monkeypatch.delenv("REMOTE_IP", raising=False)
    assert robot_profile.robot_ip("ur3e") == robot_profile.DEFAULT_ROBOT_IP

    monkeypatch.setenv("REMOTE_IP", "10.0.0.5")
    assert robot_profile.robot_ip("ur3e") == "10.0.0.5"
    assert robot_profile.robot_ip("ur16e") == "10.0.0.5"

    monkeypatch.setenv("UR3E_ROBOT_IP", "10.0.0.9")
    assert robot_profile.robot_ip("ur3e") == "10.0.0.9"
    assert robot_profile.robot_ip("ur16e") == "10.0.0.5", (
        "a per-arm override must not leak onto another arm"
    )

    # An empty per-arm value (how docker-compose passes an unset override)
    # must fall through rather than resolve to an empty address.
    monkeypatch.setenv("UR3E_ROBOT_IP", "")
    assert robot_profile.robot_ip("ur3e") == "10.0.0.5"


@pytest.mark.parametrize("ur_type", ALL_ARMS)
def test_moveit_config_builds_for_each_arm(ur_type):
    """build_moveit_config() must actually construct, and pick this arm's files.

    Two failures this catches, both of which reached a launch before being
    noticed:

    1. MoveItConfigsBuilder.__init__ parses .setup_assistant and does
       urdf_config["package"] with no guard, but only when
       config/<robot_name>.urdf.xacro is absent. Moving the URDFs into per-arm
       subdirectories made that path always taken, turning a missing 'package'
       key into KeyError before any launch argument was read.

    2. .setup_assistant can only name one arm's URDF. Every launch overrides it
       explicitly, but if that override were ever dropped, the non-default arm
       would silently load the other arm's description — same joint names, wrong
       link lengths, no error.
    """
    pytest.importorskip(
        "moveit_configs_utils", reason="MoveIt Python utils not available"
    )
    from recycle_bot.robot_profile import build_moveit_config

    config = build_moveit_config(ur_type)

    description = config.robot_description["robot_description"]
    assert f'name="{ur_type}"' in description, (
        f"build_moveit_config({ur_type!r}) returned a description for a "
        f"different robot — check the explicit .robot_description() override"
    )

    assert '0.150 0 0' in description, (
        f"{ur_type} description is missing the E-Pick tool offset"
    )
    assert 'link name="epick"' in description, (
        f"{ur_type} description is missing the E-Pick collision link"
    )

    semantic = config.robot_description_semantic["robot_description_semantic"]
    assert '<group name="ur_arm">' in semantic

    # Joint limits must come from config/<ur_type>/joint_limits.yaml. They are
    # loaded explicitly because the per-arm layout defeats MoveItConfigsBuilder's
    # convention-based discovery, which would silently fall back to the URDF's
    # own limits instead.
    limits = config.joint_limits["robot_description_planning"]["joint_limits"]
    assert "wrist_1_joint" in limits, f"{ur_type} joint limits did not load"


def test_arms_get_distinct_joint_limits():
    """The per-arm joint limit files must actually differ where the arms do.

    A silent fallback to shared or URDF-default limits would make every arm look
    identical here, which is exactly what the explicit .joint_limits() call in
    build_moveit_config exists to prevent.
    """
    pytest.importorskip(
        "moveit_configs_utils", reason="MoveIt Python utils not available"
    )
    from recycle_bot.robot_profile import build_moveit_config

    if len(ALL_ARMS) < 2:
        pytest.skip("only one arm configured")

    wrist_speeds = {}
    for ur_type in ALL_ARMS:
        limits = build_moveit_config(ur_type).joint_limits[
            "robot_description_planning"
        ]["joint_limits"]
        wrist_speeds[ur_type] = limits["wrist_1_joint"]["max_velocity"]

    # UR3e wrists run at 360 deg/s against the UR16e's 180 deg/s.
    assert len(set(wrist_speeds.values())) > 1, (
        f"all arms report the same wrist_1 max_velocity {wrist_speeds} — "
        "per-arm joint limits are not being loaded"
    )


def test_forward_kinematics_matches_the_real_robot():
    """FK from the URDF must agree with what the controller actually reported.

    Frozen from a live UR3e reading on 2026-08-18 while measuring the cell. The
    controller's PolyScope TCP is zero, so its reported pose is the flange; this
    asserts our URDF-derived flange lands in the same place.

    Guards the 150 mm trap: if the flange/tool0 chain in the URDF were altered —
    the E-Pick offset dropped by a URDF regeneration, say — this catches it
    against real hardware rather than against another copy of the same URDF.
    """
    import math

    from recycle_bot.kinematics import link_poses

    joints = {
        "shoulder_pan_joint": -2.18478,
        "shoulder_lift_joint": -0.86409,
        "elbow_joint": -1.68174,
        "wrist_1_joint": -2.19123,
        "wrist_2_joint": -1.57698,
        "wrist_3_joint": +0.00326,
    }
    # Controller-reported TCP (== flange, TCP offset zero), converted from the
    # UR "base" frame to base_link by negating x and y.
    reported_flange = (0.16646, 0.01297, 0.54795)

    frames = link_poses(joints, ur_type="ur3e")

    flange_error_mm = math.dist(frames["flange"], reported_flange) * 1000.0
    assert flange_error_mm < 15.0, (
        f"URDF flange is {flange_error_mm:.1f} mm from what the robot reported; "
        "expected a few mm of nominal-vs-calibrated kinematics error only"
    )

    # tool0 must sit exactly the E-Pick offset beyond the flange.
    offset_mm = math.dist(frames["flange"], frames["tool0"]) * 1000.0
    assert abs(offset_mm - 150.0) < 1.0, (
        f"flange->tool0 is {offset_mm:.1f} mm, expected 150 mm (E-Pick TCP offset)"
    )


def test_tool_offset_verified_against_a_physical_surface():
    """The 150 mm E-Pick offset must be the RIGHT value, not merely present.

    test_gripper_tool_offset_present only checks the URDF says 150 mm, and
    test_forward_kinematics_matches_the_real_robot only checks the flange. Both
    would still pass if 150 mm were simply the wrong number for the gripper
    actually bolted on.

    This closes that gap with a measurement taken on 2026-08-18 with the E-Pick
    mounted and its cup tip resting on the table: FK must put tool0 on the table
    surface, whose height was measured independently by touching it with the bare
    flange.

    If the end effector is changed without updating the URDF, this fails.
    """
    from recycle_bot.kinematics import link_poses

    joints = {
        "shoulder_pan_joint": -1.76011,
        "shoulder_lift_joint": -2.21096,
        "elbow_joint": -1.41399,
        "wrist_1_joint": +5.17531,
        "wrist_2_joint": +1.58632,
        "wrist_3_joint": +1.01959,
    }
    TABLE_TOP_Z = 0.00196  # measured with the bare flange, base_link frame

    tool0 = link_poses(joints, ur_type="ur3e")["tool0"]
    error_mm = abs(tool0[2] - TABLE_TOP_Z) * 1000.0
    assert error_mm < 10.0, (
        f"with the cup tip resting on the table, FK puts tool0 at z={tool0[2]:.5f}, "
        f"{error_mm:.1f} mm from the measured table top {TABLE_TOP_Z}. The "
        f"flange->tool0 offset in the URDF does not match the fitted gripper."
    )

    # The cup must point down in this pose; a sign error would put tool0 300 mm
    # off while leaving the magnitude correct.
    flange = link_poses(joints, ur_type="ur3e")["flange"]
    assert tool0[2] < flange[2], (
        "tool0 is above the flange in a tool-down pose — the offset direction is "
        "inverted"
    )


@pytest.mark.parametrize("ur_type", ALL_ARMS)
def test_forward_kinematics_chain_resolves(ur_type):
    """Every arm's URDF must expose a base_link -> tool0 chain FK can walk."""
    from recycle_bot.kinematics import link_poses

    zero = {name: 0.0 for name in (
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")}
    frames = link_poses(zero, ur_type=ur_type)
    for link in ("shoulder_link", "forearm_link", "wrist_3_link", "flange", "tool0"):
        assert link in frames, f"{ur_type}: FK chain is missing {link}"


def test_reach_envelope_matches_urdf():
    """The stored envelope constants must still match the arms' actual geometry.

    shoulder_height_m and max_tool_reach_m are hardcoded in PROFILES because
    deriving them means sampling joint space, which is far too slow to do per
    launch. That makes them capable of going stale — a regenerated URDF, a
    different end effector, a new arm — so re-derive them here and fail if they
    have drifted.

    This matters because the constants are what the reach check enforces. A stale
    max_tool_reach_m either rejects valid poses or waves through unreachable ones.
    """
    import random

    from recycle_bot.kinematics import link_poses

    names = (
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    )

    for ur_type, prof in sorted(PROFILES.items()):
        zero = link_poses({n: 0.0 for n in names}, ur_type=ur_type)
        shoulder_z = zero["shoulder_link"][2]
        assert abs(shoulder_z - prof.shoulder_height_m) < 1e-4, (
            f"{ur_type}: shoulder_height_m is {prof.shoulder_height_m}, but the "
            f"URDF puts the shoulder at {shoulder_z:.5f}"
        )

        # Seeded so a failure is reproducible rather than intermittent.
        rng = random.Random(0)
        furthest = 0.0
        for _ in range(4000):
            q = {n: rng.uniform(-math.pi, math.pi) for n in names}
            tool = link_poses(q, ur_type=ur_type)["tool0"]
            furthest = max(furthest, _distance(tool, shoulder_z))

        # Sampling under-estimates the true maximum, so the stored constant
        # should be at least what we found and not wildly beyond it.
        assert furthest <= prof.max_tool_reach_m + 1e-3, (
            f"{ur_type}: sampling reached {furthest:.4f} m, beyond the stored "
            f"max_tool_reach_m of {prof.max_tool_reach_m} — the constant is stale "
            f"and the reach check will reject valid poses"
        )
        assert furthest > prof.max_tool_reach_m * 0.95, (
            f"{ur_type}: stored max_tool_reach_m {prof.max_tool_reach_m} is much "
            f"larger than the sampled {furthest:.4f} m — the check is too "
            f"permissive and will wave through unreachable poses"
        )


def test_reach_check_still_catches_a_cross_arm_pose():
    """The looser envelope must not stop catching the failure it exists for.

    Widening the envelope (datasheet reach -> real tool reach, base -> shoulder)
    risks making the guard useless. Assert the original hazard still trips it:
    the UR16e's own poses, evaluated against the UR3e.
    """
    if not {"ur16e", "ur3e"} <= set(PROFILES):
        pytest.skip("needs both arms configured")

    ur16e_poses = _load(
        os.path.join(_robot_config_dir("ur16e"), "sorting_sequence.yaml")
    )
    ur3e = PROFILES["ur3e"]

    checked = []
    if ur16e_poses.get("neutral_pose"):
        checked.append(("neutral_pose", ur16e_poses["neutral_pose"]["position"]))
    for name, data in (ur16e_poses.get("bins") or {}).items():
        checked.append((f"bins.{name}", data["position"]))

    assert checked, "ur16e config defines no poses to cross-check"

    # The property that matters is that the CONFIG is rejected, not that every
    # individual pose is. validate_configured_poses raises if any pose offends,
    # so one is enough to block startup.
    #
    # Not every UR16e pose has to fail: bins.hdpe at [0.436, -0.055, 0.576] sits
    # 0.611 m from the UR3e's shoulder, inside its 0.732 m tool envelope, so it
    # may well be reachable. A pose from another cell that happens to land in
    # range is not a reach error — it is simply in the wrong place, which is a
    # different problem and not one a reach check can see.
    offenders = [
        name for name, pos in checked
        if _distance(pos, ur3e.shoulder_height_m) > ur3e.planning_reach_m
    ]
    assert offenders, (
        "no UR16e pose trips the UR3e reach check — the envelope has been "
        "widened so far that a cross-arm config would start up cleanly"
    )
    # And it should be the clear majority, or the guard is barely working.
    assert len(offenders) >= len(checked) // 2, (
        f"only {len(offenders)} of {len(checked)} UR16e poses trip the UR3e "
        f"check; expected most of them"
    )


@pytest.mark.parametrize("ur_type", ALL_ARMS)
def test_poses_outside_the_inner_dead_zone(ur_type):
    """No configured pose may sit inside the arm's inner dead zone.

    A UR arm cannot reach straight down close to its own base. The outer reach
    check cannot catch this: a pose in the dead zone is CLOSE to the shoulder, so
    it passes every distance test while being unreachable. A bin taught there
    would pass startup validation and then fail on every single place.
    """
    data = _load(os.path.join(_robot_config_dir(ur_type), "sorting_sequence.yaml"))
    inner = float(data.get("min_reach_radius_m", 0.0))
    if inner <= 0.0:
        pytest.skip(f"{ur_type} has no dead-zone radius configured")

    checked = []
    if data.get("neutral_pose"):
        checked.append(("neutral_pose", data["neutral_pose"]["position"]))
    for name, b in (data.get("bins") or {}).items():
        checked.append((f"bins.{name}", b["position"]))

    inside = [
        (n, math.hypot(float(p[0]), float(p[1])))
        for n, p in checked
        if math.hypot(float(p[0]), float(p[1])) < inner
    ]
    assert not inside, (
        f"{ur_type} pose(s) inside the {inner:.3f} m dead zone: "
        + ", ".join(f"{n} at r={r:.3f} m" for n, r in inside)
    )


def test_dead_zone_is_between_the_measured_bounds():
    """The UR3e dead-zone radius must stay inside its empirical bracket.

    It is not derivable — forward kinematics alone says 0.066 m, ignoring
    self-collision and the table. It was measured by jogging: r = 0.167 m could
    not be reached at table height, r = 0.226 m could. A value outside that
    bracket is either discarding reachable objects or letting through
    unreachable ones.
    """
    data = _load(os.path.join(_robot_config_dir("ur3e"), "sorting_sequence.yaml"))
    inner = float(data.get("min_reach_radius_m", 0.0))
    assert 0.167 <= inner <= 0.226, (
        f"ur3e min_reach_radius_m is {inner}, outside the measured bracket "
        "[0.167, 0.226]. Re-measure before moving it out of that range."
    )


def test_cartesian_limits_scale_with_arm_size():
    """A smaller arm must not carry a larger arm's Pilz Cartesian limits.

    Pilz LIN plans a straight Cartesian line, runs IK along it, and rejects the
    whole plan if any joint exceeds its limits. The joint acceleration a given
    tool acceleration demands scales with the Jacobian — on a shorter arm the
    same tool motion sweeps a proportionally larger joint angle.

    Copying the UR16e's values to the UR3e broke the Step 3 pick descent on the
    first simulated run (elbow wanted 6.73 rad/s^2 against a 6.28 limit), so
    assert the smaller arm's translational limits really are smaller.
    """
    import yaml as _yaml

    limits = {}
    for ur_type in ALL_ARMS:
        path = os.path.join(_moveit_config_dir(ur_type), "pilz_cartesian_limits.yaml")
        with open(path) as f:
            limits[ur_type] = _yaml.safe_load(f)["cartesian_limits"]

    if not {"ur16e", "ur3e"} <= set(limits):
        pytest.skip("needs both arms configured")

    small, large = limits["ur3e"], limits["ur16e"]
    for key in ("max_trans_vel", "max_trans_acc"):
        assert small[key] < large[key], (
            f"ur3e {key} ({small[key]}) is not below ur16e's ({large[key]}). "
            "The UR3e reaches 0.5 m against the UR16e's 0.9 m; identical "
            "Cartesian limits demand far more joint acceleration on the smaller "
            "arm and make Pilz LIN reject valid short moves."
        )
    # deceleration is negative, so a gentler limit is the LARGER value
    assert small["max_trans_dec"] > large["max_trans_dec"], (
        f"ur3e max_trans_dec ({small['max_trans_dec']}) should be gentler "
        f"(less negative) than ur16e's ({large['max_trans_dec']})"
    )


ACTIVE_MODEL = "Minimuell.pt"


def _model_class_names(model_path):
    """Class names from a YOLO .pt without importing torch or ultralytics.

    A .pt is a zip holding a pickle. Reading the names out directly keeps this
    test in the fast, dependency-free suite instead of requiring the full
    inference stack just to compare a list of strings.
    """
    import struct
    import zipfile

    with zipfile.ZipFile(model_path) as z:
        entry = [n for n in z.namelist() if n.endswith("data.pkl")]
        if not entry:
            return None
        raw = z.read(entry[0])

    start = raw.find(b"names")
    if start < 0:
        return None
    window, found, i = raw[start:start + 900], [], 0
    while i < len(window) - 5:
        op = window[i]
        if op == 0x58:  # BINUNICODE
            (n,) = struct.unpack("<I", window[i + 1:i + 5])
            if 0 < n < 64:
                try:
                    found.append(window[i + 5:i + 5 + n].decode())
                    i += 5 + n
                    continue
                except UnicodeDecodeError:
                    pass
            i += 1
        elif op == 0x8C:  # SHORT_BINUNICODE
            n = window[i + 1]
            try:
                found.append(window[i + 2:i + 2 + n].decode())
                i += 2 + n
                continue
            except UnicodeDecodeError:
                i += 1
        else:
            i += 1

    stop = {"end2end", "args", "task", "model", "yaml", "nc", "ch", "stride", "inplace"}
    classes = []
    for token in found:
        if token == "names":
            continue
        if token in stop:
            break
        classes.append(token)
    return classes or None


@pytest.mark.parametrize("ur_type", ALL_ARMS)
def test_every_model_class_is_routed(ur_type):
    """Every class the active YOLO model can emit must have a bin_routing rule.

    Labels come from model.names at load time, so swapping the model silently
    changes what routing has to cover — with no error when they disagree. An
    unrouted class does not fail; it falls through to default_bin, so mis-sorted
    objects are the only symptom.

    This bit for real: switching to Minimuell.pt left 7 of its 9 classes
    unrouted, because the previous routing was written for a model that shared
    none of the same labels.

    Exact string matching also makes spelling load-bearing. Minimuell mixes
    'non-food_bottle_pet' (hyphen) with 'non_food_can_metal' (underscore), and
    an earlier model in this repo spells it 'bootle'. Comparing against the
    model's own names is the only reliable check.
    """
    model = os.path.join(_share("recycle_bot"), "pkg_resources", ACTIVE_MODEL)
    if not os.path.exists(model):
        pytest.skip(f"{ACTIVE_MODEL} not installed at {model}")

    classes = _model_class_names(model)
    if not classes:
        pytest.skip(f"could not read class names from {ACTIVE_MODEL}")

    data = _load(os.path.join(_robot_config_dir(ur_type), "sorting_sequence.yaml"))
    routing = data.get("bin_routing") or {}
    unrouted = [c for c in classes if c not in routing]
    assert not unrouted, (
        f"{ur_type}: {len(unrouted)} of {len(classes)} classes in {ACTIVE_MODEL} "
        f"have no bin_routing rule and would fall through to "
        f"'{data.get('default_bin')}': {unrouted}"
    )


@pytest.mark.parametrize("ur_type", ALL_ARMS)
def test_no_routing_rules_for_classes_the_model_cannot_emit(ur_type):
    """bin_routing must not carry rules for labels the active model never emits.

    Dead rules are not harmful at runtime, but they describe a model that is not
    loaded — which is how a stale routing table survives a model swap unnoticed.
    """
    model = os.path.join(_share("recycle_bot"), "pkg_resources", ACTIVE_MODEL)
    if not os.path.exists(model):
        pytest.skip(f"{ACTIVE_MODEL} not installed")
    classes = _model_class_names(model)
    if not classes:
        pytest.skip("could not read class names")

    data = _load(os.path.join(_robot_config_dir(ur_type), "sorting_sequence.yaml"))
    routing = data.get("bin_routing") or {}
    dead = sorted(set(routing) - set(classes))
    assert not dead, (
        f"{ur_type}: bin_routing has {len(dead)} rule(s) for labels "
        f"{ACTIVE_MODEL} cannot emit: {dead}"
    )


def test_active_model_matches_the_vision_node():
    """ACTIVE_MODEL here must match the filename hardcoded in rec_bot_vision.

    Without this, the routing tests above could happily validate against a model
    the vision node does not load, and pass while production mis-sorts.

    Locates the module via find_spec and reads it as TEXT rather than importing
    it. Importing rec_bot_vision pulls in torch, cv2 and ultralytics, which would
    drag the whole inference stack into a suite whose entire value is running in
    milliseconds without it.
    """
    import importlib.util

    spec = importlib.util.find_spec("recycle_bot.rec_bot_vision")
    if spec is None or not spec.origin:
        pytest.skip("cannot locate rec_bot_vision source")

    with open(spec.origin) as f:
        text = f.read()

    assert f'"{ACTIVE_MODEL}"' in text, (
        f"rec_bot_vision.py does not load {ACTIVE_MODEL}, so the routing tests "
        f"are validating against the wrong model. Update ACTIVE_MODEL in this "
        f"file whenever the model in rec_bot_vision.py changes."
    )


def test_profiles_are_ordered_by_reach():
    """Sanity check on the profile table itself.

    A transposed reach/payload pair would silently widen the UR3e's envelope,
    defeating every reach check above.
    """
    assert PROFILES["ur3e"].max_reach_m < PROFILES["ur16e"].max_reach_m
    assert PROFILES["ur3e"].max_tool_reach_m < PROFILES["ur16e"].max_tool_reach_m
    assert PROFILES["ur3e"].payload_kg < PROFILES["ur16e"].payload_kg
    for ur_type, prof in PROFILES.items():
        assert isinstance(prof, RobotProfile)
        assert 0.0 < prof.reach_derate <= 1.0, f"{ur_type} derate out of range"
        assert prof.planning_reach_m < prof.max_tool_reach_m
