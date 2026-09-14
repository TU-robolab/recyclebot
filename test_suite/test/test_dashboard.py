#!/usr/bin/env python3
"""Fast checks for the operator dashboard's log parsing — no ROS graph needed.

    python3 -m pytest test/test_dashboard.py -v

The dashboard knows what the robot is doing only by reading the pipeline's log
messages (rec_bot_control publishes no task state of its own). Two things can
break that silently, and both are covered here:
  - the parsing itself (queue mirroring, success/failure verdicts, hints);
  - a node rewording a message the dashboard matches on. test_source_messages_*
    reads the installed node sources and fails if a matched phrase is gone.
"""

import importlib.util
import os

import pytest

from ament_index_python.packages import get_package_share_directory

from recycle_bot.dashboard import (
    DETECTION_FILTER_DEFAULTS,
    MODES,
    ActivityTracker,
    arm_title,
    build_launch_command,
    detection_ignored_reason,
    load_routing,
    match_hint,
    pretty_name,
)
from recycle_bot.robot_profile import PROFILES

CONTROL = "moveit_py"


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def tracker(clock):
    return ActivityTracker(clock=clock, wall_clock=lambda: 0.0)


def queue_item(tracker, label, bin_name):
    tracker.feed_log(CONTROL, 20, f"Routing label '{label}' → bin '{bin_name}' (mapped)")
    tracker.feed_log(CONTROL, 20, f"Queued sorting task for label '{label}'.")


def step(tracker, text):
    # exactly as rec_bot_control logs it, colour codes included
    tracker.feed_log(CONTROL, 20, f"\033[94m {text}\033[0m")


# -----------------------------------------------------------------------------
# task lifecycle
# -----------------------------------------------------------------------------

def test_successful_task_is_counted_per_bin(tracker, clock):
    queue_item(tracker, "food_can_metal", "gelber_sack")
    assert [q["label"] for q in tracker.snapshot()["queue"]] == ["food can metal"]

    tracker.set_busy(True)
    step(tracker, "Step 1/10: Moving to neutral (safe start)")
    step(tracker, "Step 4/10: Gripping object")
    snap = tracker.snapshot()
    assert snap["queue"] == []
    assert snap["task"]["label"] == "food can metal"
    assert snap["task"]["bin"] == "Gelber Sack"
    assert snap["task"]["step_text"] == "Picking up the item"
    assert 0 < snap["task"]["progress"] < 1

    tracker.feed_log(CONTROL, 20, "Sorting task completed")
    tracker.set_busy(False)
    clock.t += 10
    tracker.tick()
    snap = tracker.snapshot()
    assert snap["task"] is None
    assert snap["counts"] == {"sorted": 1, "failed": 0, "bins": {"Gelber Sack": 1}}


def test_task_ending_without_completion_is_a_failure(tracker, clock):
    queue_item(tracker, "food_jar_glass", "glas_papier")
    tracker.set_busy(True)
    step(tracker, "Step 1/10: Moving to neutral (safe start)")
    step(tracker, "Step 2/10: Moving to pre-pick (approach)")
    tracker.feed_log(CONTROL, 40, "Trajectory execution failed.")
    tracker.set_busy(False)

    tracker.tick()                       # still inside the grace window
    assert tracker.snapshot()["counts"]["failed"] == 0

    clock.t += ActivityTracker.TASK_END_GRACE_S + 0.1
    tracker.tick()
    snap = tracker.snapshot()
    assert snap["counts"]["failed"] == 1
    assert snap["counts"]["sorted"] == 0
    assert "Could not sort the food jar glass" in snap["events"][0]["text"]
    assert "protective stop" in snap["events"][0]["text"]   # reason carried through


def test_completion_arriving_after_busy_false_still_counts(tracker, clock):
    """robot_busy and /rosout are separate topics with no ordering guarantee."""
    queue_item(tracker, "unbekannt", "general_waste")
    tracker.set_busy(True)
    step(tracker, "Step 1/10: Moving to neutral (safe start)")
    tracker.set_busy(False)
    clock.t += 0.5
    tracker.feed_log(CONTROL, 20, "Sorting task completed")
    clock.t += 10
    tracker.tick()
    assert tracker.snapshot()["counts"]["sorted"] == 1
    assert tracker.snapshot()["counts"]["failed"] == 0


def test_duplicate_detection_is_not_queued(tracker):
    tracker.feed_log(CONTROL, 20, "Routing label 'food_can_metal' → bin 'gelber_sack' (mapped)")
    tracker.feed_log(CONTROL, 20, "Skipping duplicate detection for label 'food_can_metal' "
                                  "(existing task within 0.05 m)")
    assert tracker.snapshot()["queue"] == []


def test_stale_drop_pops_the_front_of_the_queue(tracker):
    queue_item(tracker, "food_packet_pp", "gelber_sack")
    queue_item(tracker, "food_jar_glass", "glas_papier")
    tracker.feed_log(CONTROL, 30, "Dropping stale task (61s old > 60s limit)")
    step(tracker, "Step 1/10: Moving to neutral (safe start)")
    assert tracker.snapshot()["task"]["label"] == "food jar glass"


def test_default_routed_label_keeps_its_bin(tracker):
    tracker.feed_log(CONTROL, 20, "Routing label 'bootle' → bin 'general_waste' (default)")
    tracker.feed_log(CONTROL, 20, "Queued sorting task for label 'bootle'.")
    assert tracker.snapshot()["queue"] == [{"label": "bootle", "bin": "General Waste"}]


def test_control_ready(tracker):
    assert not tracker.control_ready
    tracker.feed_log(CONTROL, 20, "ur3e sorter node initialized")
    assert tracker.control_ready


# -----------------------------------------------------------------------------
# launch gate
# -----------------------------------------------------------------------------

def test_gate_opens_counts_down_and_closes(tracker, clock):
    tracker.feed_log("launch_gate", 20, "Waiting 10.0s for UR driver to initialize...")
    assert not tracker.snapshot()["gate"]["open"]
    tracker.feed_log("launch_gate", 20, ">>> Enable External Control URCap on teach pendant <<<")
    tracker.feed_log("launch_gate", 20, "Or wait 60.0s for auto-continue...")
    clock.t += 15
    gate = tracker.snapshot()["gate"]
    assert gate["open"] and gate["seconds_left"] == 45

    tracker.feed_log("launch_gate", 20, "Gate triggered via service call, continuing...")
    assert not tracker.snapshot()["gate"]["open"]


def test_gate_timeout_closes_and_warns(tracker):
    tracker.feed_log("launch_gate", 20, ">>> Enable External Control URCap on teach pendant <<<")
    tracker.feed_log("launch_gate", 20, "Timeout (60.0s) reached, continuing...")
    snap = tracker.snapshot()
    assert not snap["gate"]["open"]
    assert [h["key"] for h in snap["hints"]] == ["gate_timeout"]


# -----------------------------------------------------------------------------
# hints
# -----------------------------------------------------------------------------

@pytest.mark.parametrize("line,key,severity", [
    ("RuntimeError: WRONG ROBOT: the arm at 192.168.1.102 reports model 'UR3'", "wrong_robot", "error"),
    ("[realsense2_camera_node-1] [WARN] No RealSense devices were found!", "no_camera", "error"),
    ("Waiting for serial bridge to subscribe before activating gripper...", "gripper_waiting", "warn"),
    ("[ERROR] [rec_bot_vision-5]: process has died [pid 42, exit code 1, cmd 'x'].",
     "died:rec_bot_vision", "error"),
    ("[ERROR] [rviz2-9]: process has died [pid 43, exit code -6, cmd 'x'].", "died:rviz2", "info"),
    ("Detection 'food_can_metal' at 0.612 m is outside the ur3e's 0.450 m reach envelope; skipping",
     "item_too_far", "info"),
    ("Grasp not confirmed after retry; releasing and returning to neutral", "grasp_failed", "warn"),
])
def test_hints(line, key, severity):
    hint = match_hint(line)
    assert hint is not None and hint.key == key and hint.severity == severity


def test_ordinary_lines_raise_no_hint():
    assert match_hint("Queued sorting task for label 'food_can_metal'.") is None
    assert match_hint("[INFO] [launch_gate-3]: process has finished cleanly [pid 7]") is None


def test_repeated_hint_collapses_and_info_expires(tracker, clock):
    line = "Detection 'x' at 0.6 m is outside the ur3e's 0.450 m reach envelope; skipping"
    for _ in range(5):
        tracker.feed_log(CONTROL, 30, line)
    assert len(tracker.snapshot()["hints"]) == 1
    assert len(tracker.snapshot()["events"]) == 1

    clock.t += ActivityTracker.HINT_TTL_S["info"] + 1
    tracker.tick()
    assert tracker.snapshot()["hints"] == []


def test_error_hints_persist_until_dismissed(tracker, clock):
    tracker.feed_launch_line("WRONG ROBOT: the arm at 10.0.0.1 reports model 'UR16'")
    clock.t += 3600
    tracker.tick()
    assert [h["key"] for h in tracker.snapshot()["hints"]] == ["wrong_robot"]
    tracker.dismiss_hint("wrong_robot")
    assert tracker.snapshot()["hints"] == []


# -----------------------------------------------------------------------------
# launch commands and display helpers
# -----------------------------------------------------------------------------

def test_launch_commands():
    assert build_launch_command(MODES["sim"], "ur3e", rviz=True) == [
        "ros2", "launch", "recycle_bot", "rec_bot_fake.launch.py", "ur_type:=ur3e"]
    assert build_launch_command(MODES["real"], "ur16e", rviz=False) == [
        "ros2", "launch", "recycle_bot", "rec_bot.launch.py", "ur_type:=ur16e",
        "launch_rviz:=false"]
    assert MODES["real"].real_robot and not MODES["sim"].real_robot


@pytest.mark.parametrize("mode", sorted(MODES))
def test_launch_files_exist_and_take_the_args_we_pass(mode):
    m = MODES[mode]
    path = os.path.join(get_package_share_directory("recycle_bot"), "launch", m.launch_file)
    with open(path) as f:
        src = f.read()
    assert '"ur_type"' in src
    if m.has_rviz_arg:
        assert '"launch_rviz"' in src


def test_display_helpers():
    assert arm_title("ur16e") == "UR16e"
    assert arm_title("ur3e") == "UR3e"
    assert pretty_name("non-food_bottle_pet") == "non food bottle pet"
    assert pretty_name(None) == ""


# -----------------------------------------------------------------------------
# detection filter: the page must agree with rec_bot_core on what gets picked
# -----------------------------------------------------------------------------

def test_detection_ignored_reason():
    flt = {"min_confidence": 0.75, "min_depth_m": 0.3, "max_depth_m": 1.5}
    assert detection_ignored_reason(0.80, 0.6, flt) is None
    assert "not sure enough" in detection_ignored_reason(0.52, 0.6, flt)
    assert "75%" in detection_ignored_reason(0.52, 0.6, flt)
    assert "too close" in detection_ignored_reason(0.9, 0.1, flt)
    assert "too far" in detection_ignored_reason(0.9, 2.0, flt)


def test_filter_defaults_match_rec_bot_core():
    src = _module_source("recycle_bot", "rec_bot_core")
    for key, value in DETECTION_FILTER_DEFAULTS.items():
        assert f'"{key}": {value}' in src, (
            f"rec_bot_core's default {key} is no longer {value}; update "
            "DETECTION_FILTER_DEFAULTS in dashboard.py")


@pytest.mark.parametrize("ur_type", sorted(PROFILES))
def test_every_arm_has_routing_and_filter(ur_type):
    bin_routing, default_bin, flt = load_routing(ur_type)
    assert bin_routing and default_bin
    assert 0.0 < flt["min_confidence"] <= 1.0


# -----------------------------------------------------------------------------
# the messages the dashboard parses still exist where they are printed
# -----------------------------------------------------------------------------

def _module_source(package, module):
    spec = importlib.util.find_spec(package)
    path = os.path.join(spec.submodule_search_locations[0], f"{module}.py")
    with open(path) as f:
        return f.read()


SOURCE_PHRASES = {
    ("recycle_bot", "rec_bot_control"): [
        "Routing label '", "' → bin '", "Queued sorting task for label '",
        "Skipping duplicate detection for label", "Dropping stale task",
        "Sorting task completed", "sorter node initialized",
        "Step 1/10:", "Step 1b:", "Step 5b:", "Step 10/10:",
        "Grasp not confirmed after retry", "reach envelope; skipping",
        "the arm cannot reach into", "Trajectory execution failed",
        "Gripper service not available", "configured pose(s) are outside",
    ],
    ("recycle_bot", "launch_gate"): [
        "Enable External Control URCap", "s for auto-continue", "Gate triggered",
        ") reached, continuing",
    ],
    ("recycle_bot", "robot_identity"): ["WRONG ROBOT"],
    ("grip_command_package", "gripper_node"): [
        "Waiting for serial bridge to subscribe", "Gripper not activated",
        "object detected",
    ],
}


@pytest.mark.parametrize("package,module", sorted(SOURCE_PHRASES))
def test_source_messages_still_match(package, module):
    src = _module_source(package, module)
    missing = [p for p in SOURCE_PHRASES[(package, module)] if p not in src]
    assert not missing, (
        f"{module}.py no longer prints {missing}; update the matching rule in "
        "recycle_bot/dashboard.py or the dashboard silently stops reporting it")
