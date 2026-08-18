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
