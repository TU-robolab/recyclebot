#!/usr/bin/env python3
"""Operator dashboard: a web page that starts, watches and stops the robot.

Serves one page on http://localhost:8080 (start_recyclebot.sh at the repo root
is the one-click host side that gets the container up and opens it). From the
page a non-technical operator can:

  * start one of three run modes — simulation, camera check, real robot — for
    either arm, and stop it again;
  * watch the live camera image with the YOLO detections drawn on it, and see
    which bin each detected item is headed for;
  * follow what the robot is doing in plain language, and get a readable hint
    when something goes wrong (camera unplugged, wrong arm, gripper offline...).

It is a passive observer of the pipeline. Everything it knows comes from what the
existing nodes already publish — /rosout, /rec_bot/camera_image,
/object_detections, /rec_bot/robot_busy, the UR driver's status topics. It never
publishes a detection or a motion goal. The only services it calls are
/launch_gate (the call the README has the operator type by hand) and the UR
dashboard's /dashboard_client/stop when Stop is pressed on the real robot.

The launch runs as a child `ros2 launch` in its own session, so Stop can signal
the whole process group the way Ctrl+C in a terminal would.

Everything above the "ROS side" marker is plain Python that needs no running
graph, so the log parsing is unit-tested in isolation
(test_suite/test/test_dashboard.py). That test also checks each message this
file parses still exists in the node that prints it.
"""

import json
import os
import re
import signal
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional
from urllib.parse import parse_qs, urlparse


# =============================================================================
# Run modes
# =============================================================================

@dataclass(frozen=True)
class Mode:
    key: str
    title: str
    description: str
    launch_file: str
    real_robot: bool      # drives real hardware: needs the operator's confirmation
    uses_robot: bool      # has an arm (real or mock) and a gripper
    has_rviz_arg: bool    # launch file accepts launch_rviz:=


MODES = {
    m.key: m for m in (
        Mode("sim", "Simulation",
             "A virtual robot and a simulated camera. Nothing moves — safe for "
             "practice and testing.",
             "rec_bot_fake.launch.py",
             real_robot=False, uses_robot=True, has_rviz_arg=False),
        Mode("camera", "Camera check",
             "Shows what the camera and the object recognition see. The robot "
             "does not move.",
             "rec_bot_vision_only.launch.py",
             real_robot=False, uses_robot=False, has_rviz_arg=True),
        Mode("real", "Real robot",
             "Runs the real robot, camera and gripper. The robot WILL move.",
             "rec_bot.launch.py",
             real_robot=True, uses_robot=True, has_rviz_arg=True),
    )
}


def build_launch_command(mode: Mode, ur_type: str, rviz: bool):
    cmd = ["ros2", "launch", "recycle_bot", mode.launch_file, f"ur_type:={ur_type}"]
    if mode.has_rviz_arg:
        cmd.append(f"launch_rviz:={'true' if rviz else 'false'}")
    return cmd


def arm_title(ur_type: str) -> str:
    """'ur16e' -> 'UR16e'."""
    return ur_type[:-1].upper() + ur_type[-1] if ur_type.endswith("e") else ur_type.upper()


def pretty_name(raw: Optional[str]) -> str:
    """YOLO labels and bin names are identifiers; show them as words.

    Display only — routing must keep using the raw label, whose spelling is
    load-bearing (see bin_routing in sorting_sequence.yaml).
    """
    if not raw:
        return ""
    return re.sub(r"[-_]+", " ", raw).strip()


def pretty_bin(raw: Optional[str]) -> str:
    """Bins are names of places ('gelber_sack' -> 'Gelber Sack')."""
    return pretty_name(raw).title()


# Mirrors rec_bot_core.load_filter_config's defaults. /object_detections is raw
# YOLO output, *before* rec_bot_core applies these thresholds — without this the
# page would show "food can → yellow bag" for an item the robot is never going to
# touch, which is exactly the confusion it exists to remove.
DETECTION_FILTER_DEFAULTS = {"min_confidence": 0.75, "min_depth_m": 0.3, "max_depth_m": 1.5}


def detection_ignored_reason(conf: float, depth_m: float, flt: dict) -> Optional[str]:
    """Why rec_bot_core will drop this detection, or None if it passes."""
    if conf < flt["min_confidence"]:
        return f"not sure enough (needs {flt['min_confidence']:.0%})"
    if depth_m < flt["min_depth_m"]:
        return "too close to the camera"
    if depth_m > flt["max_depth_m"]:
        return "too far from the camera"
    return None


# =============================================================================
# Log lines -> operator hints
# =============================================================================

ANSI_RE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


def strip_ansi(text: str) -> str:
    return ANSI_RE.sub("", text)


@dataclass(frozen=True)
class Hint:
    key: str
    severity: str   # "error" | "warn" | "info"
    text: str


# First match wins. Patterns are taken verbatim from the messages the nodes and
# drivers actually print; when one of those messages is reworded, the matching
# rule here silently stops firing, so keep the two in step.
HINT_RULES = [
    (re.compile(r"WRONG ROBOT"),
     Hint("wrong_robot", "error",
          "The connected robot is not the arm you selected. Choose the right arm "
          "(check the label on the robot) and press Start again.")),
    (re.compile(r"Failed to connect to robot"),
     Hint("robot_unreachable", "error",
          "Cannot reach the robot over the network. Check that the robot is "
          "switched on and its network cable is plugged in.")),
    (re.compile(r"No RealSense devices were found"),
     Hint("no_camera", "error",
          "The camera was not found. Check the camera's USB cable, then press "
          "Stop and Start again.")),
    (re.compile(r"configured pose\(s\) are outside"),
     Hint("config_reach", "error",
          "A saved robot position is outside the arm's reach. The configuration "
          "needs a technician.")),
    (re.compile(r"Waiting for serial bridge to subscribe"),
     Hint("gripper_waiting", "warn",
          "The gripper is not connected yet. Make sure the External Control "
          "program is running on the teach pendant.")),
    (re.compile(r"Gripper service not available|Gripper \w+ call timed out"),
     Hint("gripper_down", "error",
          "The gripper is not responding.")),
    (re.compile(r"Grasp not confirmed after retry"),
     Hint("grasp_failed", "warn",
          "The gripper could not hold an item, so it let go. The robot tries "
          "again if the item is still in view.")),
    (re.compile(r"Trajectory execution failed"),
     Hint("motion_failed", "warn",
          "A robot movement did not complete. On the real robot, check the "
          "teach pendant for a protective stop.")),
    (re.compile(r"Planning failed|Joint planning failed"),
     Hint("planning_failed", "warn",
          "The robot could not find a safe path for a movement. If this keeps "
          "happening, check nothing is blocking the robot.")),
    (re.compile(r"reach envelope; skipping"),
     Hint("item_too_far", "info",
          "An item is out of the robot's reach. Move it closer to the robot.")),
    (re.compile(r"the arm cannot reach into"),
     Hint("item_too_close", "info",
          "An item is too close to the robot's base. Move it further out.")),
    (re.compile(r"Timeout \([\d.]+s\) reached, continuing"),
     Hint("gate_timeout", "warn",
          "Continued without seeing the pendant program start. If the robot "
          "does not move, check External Control is running on the pendant.")),
    (re.compile(r"no my_robot_calibration\.yaml"),
     Hint("no_kinematics", "info",
          "This arm has no factory calibration file, so picks may be slightly "
          "off. A technician can add it.")),
    (re.compile(r"[Cc]an(?:no|')t open display|could not connect to display"),
     Hint("no_display", "info",
          "The 3D view could not open on this screen. Run the start script again "
          "(it repairs screen access), or leave the 3D view switched off.")),
]

# `ros2 launch` prints this when a child exits non-zero:
#   [ERROR] [rec_bot_vision-5]: process has died [pid 123, exit code 1, cmd ...]
PROCESS_DIED_RE = re.compile(r"\[ERROR\] \[([A-Za-z0-9_.]+?)(?:-\d+)?\]: process has died")

COMPONENT_NAMES = {
    "rec_bot_vision": "object recognition",
    "rec_bot_core": "position calculation",
    "rec_bot_control": "robot control",
    "rec_bot_viz": "detection display",
    "realsense2_camera_node": "camera",
    "fake_rgbd_publisher": "simulated camera",
    "gripper_node": "gripper",
    "mock_gripper_service": "simulated gripper",
    "ros2_control_node": "robot driver",
    "rviz2": "3D view",
}


def match_hint(text: str) -> Optional[Hint]:
    for pattern, hint in HINT_RULES:
        if pattern.search(text):
            return hint
    died = PROCESS_DIED_RE.search(text)
    if died:
        proc = died.group(1)
        if proc == "rviz2":
            return Hint("died:rviz2", "info",
                        "The 3D view closed or could not open. The robot keeps "
                        "working without it.")
        friendly = COMPONENT_NAMES.get(proc, f"'{proc}' component")
        return Hint(f"died:{proc}", "error",
                    f"The {friendly} stopped unexpectedly. Press Stop, then Start "
                    "again. If it keeps happening, call a technician.")
    return None


# =============================================================================
# Log lines -> what the robot is doing
# =============================================================================

# Friendly text per pick-place step, keyed by the step id in
# rec_bot_control._run_pick_place ("Step 4/10: ...", "Step 5b: ...").
STEP_ORDER = ["1", "1b", "2", "3", "4", "5", "5b", "6", "7", "8", "9", "10"]
STEP_TEXT = {
    "1": "Getting ready",
    "1b": "Moving next to the camera",
    "2": "Moving above the item",
    "3": "Reaching down to the item",
    "4": "Picking up the item",
    "5": "Lifting the item",
    "5b": "Checking the item is held",
    "6": "Carrying the item",
    "7": "Moving above the bin",
    "8": "Lowering into the bin",
    "9": "Letting go",
    "10": "Returning to the start position",
}

ROUTING_RE = re.compile(r"Routing label '(.+)' → bin '(.+)' \((?:mapped|default)\)")
QUEUED_RE = re.compile(r"Queued sorting task for label '(.+)'\.")
DUPLICATE_RE = re.compile(r"Skipping duplicate detection for label")
STALE_RE = re.compile(r"Dropping stale task")
STEP_RE = re.compile(r"Step (\d+[a-z]?)(?:/\d+)?: ")
COMPLETED_RE = re.compile(r"Sorting task completed")
CONTROL_READY_RE = re.compile(r"sorter node initialized")
GATE_OPEN_RE = re.compile(r"Enable External Control URCap")
GATE_WAIT_RE = re.compile(r"Or wait ([\d.]+)s for auto-continue")
GATE_CLOSED_RE = re.compile(r"Gate triggered|Timeout \([\d.]+s\) reached, continuing")

LEVEL_ERROR = 40  # rcl_interfaces/Log.ERROR


class ActivityTracker:
    """Turns the pipeline's log messages into what an operator wants to know.

    rec_bot_control publishes no queue or task state of its own, but it logs
    every transition, and /rosout preserves each node's order. This mirrors the
    control node's FIFO from those lines:

        Routing label 'L' → bin 'B'   (always precedes the verdict below)
        Queued sorting task for label 'L'.   -> append (L, B)
        Skipping duplicate detection ...     -> discard the pending route
        Dropping stale task ...              -> pop the front
        Step 1/10: ...                       -> pop the front; it is now running
        Sorting task completed               -> success

    A task that ends any other way is a failure. Control publishes robot_busy
    False after its last log line, but the two arrive on different topics with no
    ordering guarantee, so the verdict waits TASK_END_GRACE_S for a late
    "completed". The mirror is display-only; if it ever drifts, entries age out
    after QUEUE_ENTRY_TTL_S (control itself expires tasks at 60 s).
    """

    TASK_END_GRACE_S = 2.0
    QUEUE_ENTRY_TTL_S = 90.0
    HINT_TTL_S = {"info": 60.0, "warn": 300.0, "error": None}  # None: until reset
    MAX_EVENTS = 40

    def __init__(self, clock=time.monotonic, wall_clock=time.time):
        self._clock = clock
        self._wall = wall_clock
        self.reset()

    def reset(self):
        self.sorted_count = 0
        self.failed_count = 0
        self.bin_counts = {}
        self.events = deque(maxlen=self.MAX_EVENTS)
        self.hints = {}
        self.queue = deque()
        self.current = None
        self.busy = False
        self.control_ready = False
        self.gate_open = False
        self.gate_deadline = None
        self._pending_route = None

    # -- inputs ---------------------------------------------------------------

    def feed_log(self, logger: str, level: int, text: str):
        text = strip_ansi(text).strip()
        hint = match_hint(text)
        if hint:
            self.add_hint(hint)

        if logger == "launch_gate":
            self._on_gate(text)
            return

        if CONTROL_READY_RE.search(text):
            self.control_ready = True
        elif m := ROUTING_RE.search(text):
            self._pending_route = (m.group(1), m.group(2))
        elif m := QUEUED_RE.search(text):
            label = m.group(1)
            bin_name = None
            if self._pending_route and self._pending_route[0] == label:
                bin_name = self._pending_route[1]
            self._pending_route = None
            self.queue.append({"label": label, "bin": bin_name, "t": self._clock()})
            self.note("info", f"Found a {pretty_name(label)}"
                        + (f" — it goes to {pretty_bin(bin_name)}" if bin_name else ""))
        elif DUPLICATE_RE.search(text):
            self._pending_route = None
        elif STALE_RE.search(text):
            if self.queue:
                dropped = self.queue.popleft()
                self.note("info", f"Skipped the {pretty_name(dropped['label'])}: it "
                            "waited too long without being seen again")
        elif m := STEP_RE.search(text):
            self._on_step(m.group(1))
        elif COMPLETED_RE.search(text):
            self._finish(success=True)
        elif level >= LEVEL_ERROR and self.current is not None:
            self.current["error"] = text

    def feed_launch_line(self, line: str):
        """Lines from `ros2 launch` stdout: only launch-level failures matter here.

        Node log lines also reach stdout (output="screen"), but those are parsed
        from /rosout, which carries the level and logger name. The hint table is
        applied here too because some failures (WRONG ROBOT, a crashed process)
        never reach /rosout; duplicates collapse on the hint key.
        """
        hint = match_hint(strip_ansi(line))
        if hint:
            self.add_hint(hint)

    def set_busy(self, busy: bool):
        self.busy = busy
        if self.current is not None:
            self.current["ending_at"] = None if busy else self._clock()

    def add_hint(self, hint: Hint):
        now = self._clock()
        entry = self.hints.get(hint.key)
        if entry is None:
            self.hints[hint.key] = {"hint": hint, "first": now, "last": now, "count": 1}
            self.note(hint.severity, hint.text)
        else:
            entry["last"] = now
            entry["count"] += 1

    def dismiss_hint(self, key: str):
        self.hints.pop(key, None)

    # -- periodic -------------------------------------------------------------

    def tick(self):
        now = self._clock()
        cur = self.current
        if cur is not None and cur.get("ending_at") is not None \
                and now - cur["ending_at"] > self.TASK_END_GRACE_S:
            self._finish(success=False)
        while self.queue and now - self.queue[0]["t"] > self.QUEUE_ENTRY_TTL_S:
            self.queue.popleft()
        for key, entry in list(self.hints.items()):
            ttl = self.HINT_TTL_S.get(entry["hint"].severity)
            if ttl is not None and now - entry["last"] > ttl:
                del self.hints[key]
        if self.gate_deadline is not None and now > self.gate_deadline + 5.0:
            # the gate's own "Timeout reached" line was missed; don't show a
            # countdown stuck at zero
            self.gate_open = False
            self.gate_deadline = None

    # -- outputs --------------------------------------------------------------

    def snapshot(self):
        now = self._clock()
        task = None
        if self.current is not None:
            step = self.current["step"]
            idx = STEP_ORDER.index(step) if step in STEP_ORDER else 0
            task = {
                "label": pretty_name(self.current["label"]) or "item",
                "bin": pretty_bin(self.current["bin"]) or "its bin",
                "step_text": STEP_TEXT.get(step, "Working"),
                "progress": (idx + 1) / len(STEP_ORDER),
            }
        return {
            "task": task,
            "queue": [{"label": pretty_name(q["label"]), "bin": pretty_bin(q["bin"])}
                      for q in self.queue],
            "counts": {
                "sorted": self.sorted_count,
                "failed": self.failed_count,
                "bins": {pretty_bin(k): v for k, v in sorted(self.bin_counts.items())},
            },
            "events": list(reversed(self.events)),
            "hints": sorted(
                ({"key": e["hint"].key, "severity": e["hint"].severity,
                  "text": e["hint"].text, "count": e["count"],
                  "age_s": round(now - e["last"])}
                 for e in self.hints.values()),
                key=lambda h: ({"error": 0, "warn": 1, "info": 2}[h["severity"]], h["age_s"]),
            ),
            "gate": {
                "open": self.gate_open,
                "seconds_left": (max(0, round(self.gate_deadline - now))
                                 if self.gate_open and self.gate_deadline else None),
            },
        }

    # -- internals ------------------------------------------------------------

    def _on_gate(self, text: str):
        if GATE_OPEN_RE.search(text):
            self.gate_open = True
        elif m := GATE_WAIT_RE.search(text):
            self.gate_open = True
            self.gate_deadline = self._clock() + float(m.group(1))
        elif GATE_CLOSED_RE.search(text):
            self.gate_open = False
            self.gate_deadline = None

    def _on_step(self, step: str):
        if step == "1":
            if self.current is not None:
                self._finish(success=False)
            task = self.queue.popleft() if self.queue else {"label": None, "bin": None}
            self.current = {"label": task["label"], "bin": task["bin"], "step": step,
                            "error": None, "ending_at": None}
        elif self.current is not None:
            self.current["step"] = step

    def _finish(self, success: bool):
        cur = self.current
        if cur is None:
            return
        self.current = None
        name = pretty_name(cur["label"]) or "item"
        if success:
            self.sorted_count += 1
            if cur["bin"]:
                self.bin_counts[cur["bin"]] = self.bin_counts.get(cur["bin"], 0) + 1
            self.note("success", f"Sorted the {name}"
                        + (f" into {pretty_bin(cur['bin'])}" if cur["bin"] else ""))
        else:
            self.failed_count += 1
            reason = ""
            if cur["error"]:
                hint = match_hint(cur["error"])
                reason = f": {hint.text}" if hint else f" ({cur['error']})"
            self.note("error", f"Could not sort the {name}{reason}")

    def note(self, severity: str, text: str):
        self.events.append({"t": self._wall(), "severity": severity, "text": text})


# =============================================================================
# The `ros2 launch` child process
# =============================================================================

class LaunchSupervisor:
    """Runs one `ros2 launch` at a time and stops it like Ctrl+C would.

    start_new_session puts the launch and every node it spawns in their own
    process group, so Stop reaches all of them — and so a Ctrl+C aimed at the
    dashboard's own terminal does not tear the robot down behind its back.
    """

    STOP_ESCALATION = ((signal.SIGINT, 20.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0))

    def __init__(self, on_line, on_exit, log_dir=None):
        self._on_line = on_line
        self._on_exit = on_exit
        self._log_dir = log_dir
        self._lock = threading.Lock()
        self._proc = None
        self.stopping = False
        self.mode = None
        self.ur_type = None
        self.started_at = None
        self.log_path = None
        self.lines = deque(maxlen=2000)   # (seq, text)
        self.seq = 0
        self._lines_lock = threading.Lock()  # pump thread appends, HTTP threads read

    @property
    def running(self) -> bool:
        with self._lock:
            return self._proc is not None

    def start(self, cmd, mode, ur_type):
        with self._lock:
            if self._proc is not None:
                raise RuntimeError("already running")
            env = dict(os.environ, PYTHONUNBUFFERED="1", RCUTILS_COLORIZED_OUTPUT="0")
            log_file = None
            if self._log_dir:
                try:
                    os.makedirs(self._log_dir, exist_ok=True)
                    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
                    self.log_path = os.path.join(
                        self._log_dir, f"{stamp}_{mode}_{ur_type}.log")
                    log_file = open(self.log_path, "w", buffering=1)
                except OSError:
                    self.log_path = None
            self._proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, text=True, bufsize=1, errors="replace",
                start_new_session=True, env=env,
            )
            self.stopping = False
            self.mode = mode
            self.ur_type = ur_type
            self.started_at = time.time()
            with self._lines_lock:
                self.lines.clear()   # seq keeps counting; the page resets on started_at
            self._append(f"$ {' '.join(cmd)}", log_file)
            threading.Thread(target=self._pump, args=(self._proc, log_file),
                             daemon=True).start()

    def stop(self, pre_stop=None, blocking=False):
        with self._lock:
            proc = self._proc
            if proc is None or self.stopping:
                return
            self.stopping = True
        if blocking:
            self._stop_proc(proc, pre_stop)
        else:
            threading.Thread(target=self._stop_proc, args=(proc, pre_stop),
                             daemon=True).start()

    def lines_after(self, seq: int):
        with self._lines_lock:
            return [(s, t) for s, t in self.lines if s > seq]

    def _append(self, text, log_file):
        with self._lines_lock:
            self.seq += 1
            self.lines.append((self.seq, text))
        if log_file:
            try:
                log_file.write(text + "\n")
            except (OSError, ValueError):
                pass

    def _pump(self, proc, log_file):
        for raw in proc.stdout:
            line = strip_ansi(raw.rstrip("\n"))
            self._append(line, log_file)
            try:
                self._on_line(line)
            except Exception:  # a parsing bug must never stall the pipe
                pass
        code = proc.wait()
        self._append(f"[dashboard] launch exited with code {code}", log_file)
        if log_file:
            log_file.close()
        with self._lock:
            requested = self.stopping
            self._proc = None
            self.stopping = False
        self._on_exit(code, requested)

    def _stop_proc(self, proc, pre_stop):
        if pre_stop is not None:
            try:
                pre_stop()
            except Exception:
                pass
        pgid = proc.pid  # session leader: its pid is the group id
        for sig, wait_s in self.STOP_ESCALATION:
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                return
            deadline = time.monotonic() + wait_s
            while time.monotonic() < deadline:
                if proc.poll() is not None and not _group_alive(pgid):
                    return
                time.sleep(0.2)


def _group_alive(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


# =============================================================================
# ROS side
# =============================================================================

import cv2  # noqa: E402
import yaml  # noqa: E402

import rclpy  # noqa: E402
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from rclpy.signals import SignalHandlerOptions  # noqa: E402

from cv_bridge import CvBridge  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from rcl_interfaces.msg import Log  # noqa: E402
from sensor_msgs.msg import Image, JointState  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from vision_msgs.msg import Detection3DArray  # noqa: E402

from recycle_bot.rec_bot_viz import _class_color_bgr  # noqa: E402
from recycle_bot.robot_profile import PROFILES, config_path, resolve_ur_type  # noqa: E402

try:
    from ur_dashboard_msgs.msg import RobotMode, SafetyMode
except ImportError:  # the driver ships it; absent only in a stripped-down image
    RobotMode = SafetyMode = None

try:
    from grip_interface.srv import GripCommand
except ImportError:
    GripCommand = None


# How long a published detection stays drawn. Vision re-publishes a stationary
# object only once its dedup window (2 s) expires, so anything shorter flickers.
DETECTION_HOLD_S = 3.0
STREAM_MAX_FPS = 15.0   # rendering a frame costs ~4 ms at 640x480
STREAM_MAX_WIDTH = 960
# Right after our own launch exits its nodes linger in the DDS graph cache — up
# to the participant lease (20 s in Fast DDS) for any that had to be SIGKILLed.
# Don't mistake them for someone else's run.
EXTERNAL_RUN_GRACE_S = 25.0
PIPELINE_NODES = {"moveit_py", "cobot_control", "rec_bot_vision", "rec_bot_core",
                  "rec_bot_viz", "launch_gate", "controller_manager"}

SAFETY_TEXT = {
    3: "Protective stop: the robot bumped into something or was pushed. Clear "
       "the area, then unlock the protective stop on the teach pendant.",
    5: "Safeguard stop: a safety input stopped the robot (for example a door or "
       "light curtain). Clear it, then reset it on the teach pendant.",
    6: "Emergency stop is pressed. When it is safe, release the E-stop and "
       "restart the robot on the teach pendant.",
    7: "Emergency stop is pressed. When it is safe, release the E-stop and "
       "restart the robot on the teach pendant.",
    8: "The robot reports a safety violation. Check the teach pendant and call "
       "a technician.",
    9: "The robot reports a safety fault. Check the teach pendant and call a "
       "technician.",
    12: "Safeguard stop: a safety input stopped the robot. Clear it, then reset "
        "it on the teach pendant.",
    13: "The enabling device on the teach pendant was released.",
}
ROBOT_MODE_TEXT = {
    3: "The robot arm is powered off. Turn it on at the teach pendant.",
    5: "The robot's brakes are not released. Release them on the teach pendant "
       "(press ON, then START).",
    1: "The robot is waiting for a safety confirmation on the teach pendant.",
    2: "The robot is still booting.",
}


def load_routing(ur_type: str):
    """(bin_routing, default_bin, detection_filter) for this arm's cell.

    Read from the same installed YAML the pipeline nodes use, so the page labels
    each detection with the bin — and the filter verdict — the robot will act on.
    """
    bin_routing, default_bin = {}, None
    flt = dict(DETECTION_FILTER_DEFAULTS)
    try:
        with open(config_path(ur_type, "sorting_sequence.yaml")) as f:
            cfg = yaml.safe_load(f) or {}
        bin_routing, default_bin = dict(cfg.get("bin_routing") or {}), cfg.get("default_bin")
    except (OSError, yaml.YAMLError):
        pass
    try:
        with open(config_path(ur_type, "calibration.yaml")) as f:
            cfg = (yaml.safe_load(f) or {}).get("detection_filter") or {}
        flt.update({k: float(cfg[k]) for k in flt if k in cfg})
    except (OSError, yaml.YAMLError, ValueError, AttributeError):
        pass
    return bin_routing, default_bin, flt


class Dashboard(Node):
    def __init__(self):
        super().__init__("rec_bot_dashboard")

        self.port = int(self.declare_parameter("port", 8080).value)
        # Loopback by default: anyone who can load this page can start the real
        # robot. Binding to 0.0.0.0 (for a tablet at the cell) is a deliberate
        # choice for a trusted network, not a default.
        self.bind_address = str(self.declare_parameter("bind_address", "127.0.0.1").value)
        self.default_ur_type = resolve_ur_type(
            self.declare_parameter("ur_type", "").value or None)

        share = get_package_share_directory("recycle_bot")
        self.web_dir = os.path.join(share, "web")
        log_dir = os.path.expanduser("~/logs/dashboard") \
            if os.path.isdir(os.path.expanduser("~/logs")) else None

        self._lock = threading.Lock()
        self.tracker = ActivityTracker()
        self.launch = LaunchSupervisor(self._on_launch_line, self._on_launch_exit, log_dir)
        self.last_exit = None
        self._last_exit_mono = 0.0
        self._routing = ({}, None, dict(DETECTION_FILTER_DEFAULTS))

        self._bridge = CvBridge()
        self._frame_msg = None
        self._frame_t = 0.0
        self._frame_dt = 0.0
        self._frame_event = threading.Event()
        self._detections = []
        self._detections_t = 0.0
        self._joint_t = 0.0
        self._gripper_status = None
        self._gripper_t = 0.0
        self._program_running = None
        self._safety_mode = None
        self._robot_mode = None
        self._graph_nodes = set()
        self._gate_requested = False

        # MJPEG: one encoder thread, any number of viewers waiting on _jpeg_cond
        self._jpeg = None
        self._jpeg_seq = 0
        self._jpeg_cond = threading.Condition()
        self._stream_clients = 0
        self.shutting_down = threading.Event()

        best_effort = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                 reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
        reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # TRANSIENT_LOCAL is load-bearing. A freshly launched node logs its first
        # lines ("sorter node initialized", the gate prompt) before DDS discovery
        # has matched its /rosout publisher to this subscription; a VOLATILE
        # subscriber silently loses them, and the page sits at "Starting up…"
        # forever. /rosout publishers keep ~10 s of history for exactly this.
        self.create_subscription(Log, "/rosout", self._on_rosout,
                                 QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1000,
                                            reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Own callback group: in the node's default (mutually exclusive) group
        # camera frames queue behind every /rosout line and the video halves.
        self.create_subscription(Image, "/rec_bot/camera_image", self._on_image, best_effort,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(Detection3DArray, "/object_detections",
                                 self._on_detections, reliable)
        self.create_subscription(Bool, "/rec_bot/robot_busy", self._on_busy, latched)
        self.create_subscription(JointState, "/joint_states", self._on_joints, best_effort)
        self.create_subscription(String, "/object_detection/status",
                                 self._on_gripper_status, reliable)
        self.create_subscription(Bool, "/io_and_status_controller/robot_program_running",
                                 self._on_program_running, latched)
        if SafetyMode is not None:
            self.create_subscription(SafetyMode, "/io_and_status_controller/safety_mode",
                                     self._on_safety_mode, latched)
            self.create_subscription(RobotMode, "/io_and_status_controller/robot_mode",
                                     self._on_robot_mode, latched)

        self._gate_client = self.create_client(Trigger, "/launch_gate")
        self._ur_stop_client = self.create_client(Trigger, "/dashboard_client/stop")
        self._gripper_client = (self.create_client(GripCommand, "/gripper_action")
                                if GripCommand is not None else None)

        self.create_timer(1.0, self._tick)
        threading.Thread(target=self._encode_loop, daemon=True).start()

    # -- ROS callbacks --------------------------------------------------------

    def _on_rosout(self, msg: Log):
        if msg.name == self.get_name() or self.launch.stopping:
            return
        with self._lock:
            self.tracker.feed_log(msg.name, msg.level, msg.msg)

    def _on_image(self, msg: Image):
        now = time.monotonic()
        with self._lock:
            if self._frame_msg is not None:
                # smoothed interval, not smoothed 1/interval: frames arrive in
                # bursts, and averaging rates overweights the short gaps
                dt = now - self._frame_t
                self._frame_dt = dt if self._frame_dt == 0.0 else 0.9 * self._frame_dt + 0.1 * dt
            self._frame_msg = msg
            self._frame_t = now
        self._frame_event.set()

    def _on_detections(self, msg: Detection3DArray):
        dets = []
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            dets.append({
                "label": hyp.class_id, "conf": float(hyp.score),
                "cx": det.bbox.center.position.x, "cy": det.bbox.center.position.y,
                "depth": det.bbox.center.position.z,   # avg depth, see rec_bot_vision
                "w": det.bbox.size.x, "h": det.bbox.size.y,
            })
        with self._lock:
            self._detections = dets
            self._detections_t = time.monotonic()

    def _on_busy(self, msg: Bool):
        with self._lock:
            self.tracker.set_busy(msg.data)

    def _on_joints(self, _msg):
        self._joint_t = time.monotonic()

    def _on_gripper_status(self, msg: String):
        with self._lock:
            self._gripper_status = msg.data
            self._gripper_t = time.monotonic()

    def _on_program_running(self, msg: Bool):
        self._program_running = msg.data

    def _on_safety_mode(self, msg):
        self._safety_mode = int(msg.mode)

    def _on_robot_mode(self, msg):
        self._robot_mode = int(msg.mode)

    # A hint stops being true once the thing it complains about is working.
    RESOLVED_BY = {
        "camera": ("no_camera",),
        "robot": ("robot_unreachable",),
        "gripper": ("gripper_waiting", "gripper_down"),
        "pendant": ("gate_timeout",),
    }

    def _tick(self):
        try:
            self._graph_nodes = set(self.get_node_names())
        except Exception:
            pass
        state = self.state()
        ok = {light["key"] for light in state["lights"] if light["state"] == "ok"}
        with self._lock:
            for key in ok:
                for hint_key in self.RESOLVED_BY.get(key, ()):
                    self.tracker.dismiss_hint(hint_key)
            self.tracker.tick()
            gate_open = self.tracker.gate_open
        if not gate_open:
            self._gate_requested = False
        # The gate exists to wait for External Control; once the driver reports
        # the program running there is nothing left to wait for.
        elif (self._program_running and not self._gate_requested
              and self.launch.mode == "real" and self._gate_client.service_is_ready()):
            self._gate_requested = True
            self._gate_client.call_async(Trigger.Request())
            with self._lock:
                self.tracker.note("success", "External Control is running on the "
                                    "pendant — continuing start-up")

    # -- launch ---------------------------------------------------------------

    def _on_launch_line(self, line: str):
        # During a requested Stop, nodes interrupted by SIGINT exit non-zero and
        # launch reports each as "process has died"; that is not a problem to
        # show the operator. Same for the errors they log on the way down.
        if self.launch.stopping:
            return
        with self._lock:
            self.tracker.feed_launch_line(line)

    def _on_launch_exit(self, code: int, requested: bool):
        unexpected = not requested
        with self._lock:
            if requested:
                for key in [k for k in self.tracker.hints if k.startswith("died:")]:
                    self.tracker.dismiss_hint(key)
            self.last_exit = {"code": code, "unexpected": unexpected,
                              "mode": self.launch.mode}
            self._last_exit_mono = time.monotonic()
            self.tracker.gate_open = False
            self.tracker.current = None
            self.tracker.queue.clear()
            self.tracker.note("error" if unexpected else "info",
                                "Stopped because of a problem" if unexpected else "Stopped")
        self._program_running = None
        self._safety_mode = None
        self._robot_mode = None

    def start(self, mode_key: str, ur_type: str, rviz: bool, confirmed: bool):
        mode = MODES.get(mode_key)
        if mode is None:
            raise ValueError(f"unknown mode '{mode_key}'")
        if ur_type not in PROFILES:
            raise ValueError(f"unknown arm '{ur_type}'")
        if mode.real_robot and not confirmed:
            raise ValueError("starting the real robot needs the safety confirmation")
        if self.launch.running:
            raise ValueError("already running — press Stop first")
        if self._external_run():
            raise ValueError("robot software is already running outside the dashboard")
        with self._lock:
            self.tracker.reset()
            self.last_exit = None
            self._detections = []
            self._frame_msg = None
            self._frame_dt = 0.0
            self._routing = load_routing(ur_type)
        self._program_running = self._safety_mode = self._robot_mode = None
        self._gate_requested = False
        cmd = build_launch_command(mode, ur_type, rviz)
        self.get_logger().info(f"starting: {' '.join(cmd)}")
        self.launch.start(cmd, mode_key, ur_type)

    def stop(self, blocking=False):
        pre_stop = self._stop_ur_program if self.launch.mode == "real" else None
        self.launch.stop(pre_stop=pre_stop, blocking=blocking)

    def _stop_ur_program(self):
        """Stop External Control through the UR dashboard before tearing down.

        Killing the driver alone leaves the program waiting for a reconnect;
        stopping it first halts the arm promptly and leaves the pendant in a
        clean state. Best effort — the launch is shut down either way.
        """
        if not self._ur_stop_client.service_is_ready():
            return
        future = self._ur_stop_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)

    def continue_gate(self) -> bool:
        if not self._gate_client.service_is_ready():
            return False
        self._gate_requested = True
        self._gate_client.call_async(Trigger.Request())
        return True

    def _external_run(self) -> bool:
        if self.launch.running:
            return False
        if time.monotonic() - self._last_exit_mono < EXTERNAL_RUN_GRACE_S:
            return False
        return bool(self._graph_nodes & PIPELINE_NODES)

    # -- camera stream --------------------------------------------------------

    def _encode_loop(self):
        """Encode the newest frame whenever one arrives, at most STREAM_MAX_FPS.

        Woken by _on_image rather than polling: a poll-then-sleep loop beats
        against the camera's own period and drops every other frame.
        """
        next_ok = 0.0
        while not self.shutting_down.is_set():
            if self._stream_clients == 0:
                time.sleep(0.2)
                continue
            if not self._frame_event.wait(0.5):
                continue
            self._frame_event.clear()
            delay = next_ok - time.monotonic()
            if delay > 0:
                time.sleep(delay)   # newer frames replace this one meanwhile
            t0 = time.monotonic()
            next_ok = t0 + 1.0 / STREAM_MAX_FPS
            with self._lock:
                msg = self._frame_msg
                dets = list(self._detections) \
                    if t0 - self._detections_t < DETECTION_HOLD_S else []
                routing = self._routing
            if msg is None:
                continue
            try:
                jpeg = self._render(msg, dets, routing)
            except Exception as exc:
                self.get_logger().warn(f"frame encode failed: {exc}",
                                       throttle_duration_sec=10.0)
                continue
            with self._jpeg_cond:
                self._jpeg = jpeg
                self._jpeg_seq += 1
                self._jpeg_cond.notify_all()

    def _render(self, msg, dets, routing):
        bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h0, w0 = bgr.shape[:2]
        scale = min(1.0, STREAM_MAX_WIDTH / float(w0))
        if scale < 1.0:
            bgr = cv2.resize(bgr, (int(w0 * scale), int(h0 * scale)),
                             interpolation=cv2.INTER_AREA)
        bin_routing, default_bin, flt = routing
        for d in dets:
            cx, cy = d["cx"] * scale, d["cy"] * scale
            w, h = d["w"] * scale, d["h"] * scale
            x1, y1 = int(cx - w / 2), int(cy - h / 2)
            x2, y2 = int(cx + w / 2), int(cy + h / 2)
            text = f"{pretty_name(d['label'])} {d['conf']:.0%}"
            if detection_ignored_reason(d["conf"], d["depth"], flt):
                # grey and thin: seen, but the robot will leave it alone
                color, thickness = (128, 128, 128), 1
                text += " (ignored)"
            else:
                color, thickness = _class_color_bgr(d["label"]), 3
                bin_name = bin_routing.get(d["label"], default_bin)
                if bin_name:
                    text += f" > {pretty_bin(bin_name)}"
            cv2.rectangle(bgr, (x1, y1), (x2, y2), color, thickness)
            (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            ty = max(y1, th + base + 6)
            cv2.rectangle(bgr, (x1, ty - th - base - 6), (x1 + tw + 8, ty), color, cv2.FILLED)
            cv2.putText(bgr, text, (x1 + 4, ty - base - 3), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 1, cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if not ok:
            raise RuntimeError("imencode failed")
        return buf.tobytes()

    def wait_frame(self, after_seq: int, timeout: float):
        with self._jpeg_cond:
            if self._jpeg_seq <= after_seq:
                self._jpeg_cond.wait(timeout)
            if self._jpeg_seq <= after_seq:
                return None, after_seq
            return self._jpeg, self._jpeg_seq

    def stream_client(self, delta: int):
        with self._jpeg_cond:
            self._stream_clients += delta

    # -- state for the page ---------------------------------------------------

    def state(self):
        now = time.monotonic()
        running = self.launch.running
        mode = MODES.get(self.launch.mode) if running else None
        with self._lock:
            snap = self.tracker.snapshot()
            busy = self.tracker.busy
            control_ready = self.tracker.control_ready
            gripper_status = self._gripper_status
            gripper_age = now - self._gripper_t
            frame_age = now - self._frame_t if self._frame_msg is not None else None
            dets = self._detections if now - self._detections_t < DETECTION_HOLD_S else []
            bin_routing, default_bin, flt = self._routing
            last_exit = dict(self.last_exit) if self.last_exit else None

        camera_ok = frame_age is not None and frame_age < 2.0
        joints_ok = now - self._joint_t < 1.5
        external = self._external_run()

        # The driver publishes safety and robot mode on change only, so their age
        # says nothing. Joint states keep flowing through a protective or
        # emergency stop, which makes them the "driver still connected" signal.
        safety = None
        if mode is not None and mode.real_robot and joints_ok:
            if self._safety_mode in SAFETY_TEXT:
                safety = SAFETY_TEXT[self._safety_mode]
            elif self._robot_mode in ROBOT_MODE_TEXT:
                safety = ROBOT_MODE_TEXT[self._robot_mode]

        lights = self._lights(mode, running, camera_ok, joints_ok,
                              gripper_status, gripper_age)
        phase, headline, detail = self._phase(
            mode, running, camera_ok, busy, control_ready, snap, last_exit, external)

        return {
            "process_running": running,
            "stopping": self.launch.stopping,
            "phase": phase,
            "headline": headline,
            "detail": detail,
            "safety": safety,
            "mode": self.launch.mode if running else None,
            "ur_type": self.launch.ur_type if running else None,
            # kept after the run ends: the page keys its log view on it, and the
            # output of a run that just failed is the one a technician wants
            "started_at": self.launch.started_at,
            "log_path": self.launch.log_path,
            "log_seq": self.launch.seq,
            "busy": busy,
            "camera": {"ok": camera_ok,
                       "paused": busy and camera_ok,
                       "hz": round(1.0 / self._frame_dt, 1) if camera_ok and self._frame_dt else 0.0},
            "lights": lights,
            "gate": {**snap["gate"],
                     "program_running": self._program_running,
                     "can_continue": self._gate_client.service_is_ready()},
            "task": snap["task"],
            "queue": snap["queue"],
            "counts": snap["counts"],
            "events": snap["events"],
            "hints": snap["hints"],
            "detections": [
                {"label": pretty_name(d["label"]), "conf": round(d["conf"], 2),
                 "bin": pretty_bin(bin_routing.get(d["label"], default_bin)),
                 "ignored": detection_ignored_reason(d["conf"], d["depth"], flt)}
                for d in dets
            ],
            "external_run": external,
            "last_exit": last_exit,
        }

    def _lights(self, mode, running, camera_ok, joints_ok, gripper_status, gripper_age):
        def light(key, label, state, text):
            return {"key": key, "label": label, "state": state, "text": text}

        uses_robot = mode is not None and mode.uses_robot
        sim = mode is not None and not mode.real_robot
        lights = []

        if not running:
            robot = light("robot", "Robot arm", "off", "Off")
        elif not uses_robot:
            robot = light("robot", "Robot arm", "off", "Not used in this mode")
        elif joints_ok:
            robot = light("robot", "Robot arm", "ok", "Simulated" if sim else "Connected")
        else:
            robot = light("robot", "Robot arm", "wait", "Connecting…")
        lights.append(robot)

        if mode is not None and mode.real_robot:
            if self._program_running is True:
                lights.append(light("pendant", "Pendant program", "ok",
                                    "External Control running"))
            elif self._program_running is False:
                lights.append(light("pendant", "Pendant program", "wait",
                                    "Not running — press Play"))
            else:
                lights.append(light("pendant", "Pendant program", "wait", "Waiting…"))

        if camera_ok:
            lights.append(light("camera", "Camera", "ok",
                                "Simulated" if mode and mode.key == "sim" else "Live"))
        else:
            lights.append(light("camera", "Camera", "wait" if running else "off",
                                "Starting…" if running else "Off"))

        if "rec_bot_vision" in self._graph_nodes and running:
            lights.append(light("vision", "Object recognition", "ok", "Running"))
        else:
            lights.append(light("vision", "Object recognition",
                                "wait" if running else "off",
                                "Loading…" if running else "Off"))

        if not running:
            lights.append(light("gripper", "Gripper", "off", "Off"))
        elif not uses_robot:
            lights.append(light("gripper", "Gripper", "off", "Not used in this mode"))
        elif gripper_status and "not activated" in gripper_status:
            lights.append(light("gripper", "Gripper", "bad", "Not activated"))
        elif gripper_age < 5.0 or (self._gripper_client is not None
                                   and self._gripper_client.service_is_ready()):
            text = "Holding an item" if gripper_status and gripper_status.startswith(
                "object detected") and gripper_age < 5.0 else "Ready"
            lights.append(light("gripper", "Gripper", "ok",
                                f"{text} (simulated)" if sim else text))
        else:
            lights.append(light("gripper", "Gripper", "wait", "Connecting…"))
        return lights

    def _phase(self, mode, running, camera_ok, busy, control_ready, snap, last_exit, external):
        if not running:
            if last_exit and last_exit["unexpected"]:
                return ("failed", "Stopped because of a problem",
                        "See the messages below. Press Start to try again.")
            if external:
                return ("external", "Robot software is already running elsewhere",
                        "It was started outside this dashboard, for example from a "
                        "terminal. Stop it there before starting from here.")
            return ("stopped", "Ready to start",
                    "Choose what to run, then press Start.")
        if self.launch.stopping:
            return ("stopping", "Stopping…", "Shutting everything down safely.")

        if mode.key == "camera":
            if camera_ok:
                return ("watching", "Camera check running",
                        "The robot does not move in this mode. Put items under the "
                        "camera to see what is recognised.")
            return ("starting", "Starting the camera…", "This takes a few seconds.")

        if mode.real_robot and snap["gate"]["open"]:
            return ("waiting_pendant", "Start the program on the teach pendant",
                    "On the teach pendant, open the External Control program and "
                    "press Play. The dashboard continues by itself once it sees the "
                    "program running.")
        if not control_ready:
            return ("starting", "Starting up…",
                    "Loading the robot controller and object recognition. This takes "
                    "about half a minute.")
        if snap["task"]:
            t = snap["task"]
            return ("sorting", f"Sorting: {t['label']} → {t['bin']}", t["step_text"])
        if busy:
            return ("moving", "Robot moving to its start position",
                    "Keep clear of the robot.")
        return ("watching", "Watching for items",
                "Place items under the camera. The robot picks them up one by one.")


# =============================================================================
# HTTP
# =============================================================================

LOOPBACK_HOSTS = {"localhost", "127.0.0.1", "[::1]", "::1"}
STATIC_TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript",
                ".css": "text/css", ".svg": "image/svg+xml"}


def make_handler(app: Dashboard):
    class Handler(BaseHTTPRequestHandler):
        server_version = "RecycleBotDashboard/1"

        def log_message(self, fmt, *args):  # keep the console for the launch output
            pass

        # -- guards -----------------------------------------------------------

        def _host_ok(self) -> bool:
            # Blocks DNS-rebinding: a hostile page that re-points its own domain
            # at 127.0.0.1 still sends its own name in Host.
            if app.bind_address not in ("127.0.0.1", "localhost", "::1"):
                return True
            host = self.headers.get("Host") or ""
            if host.startswith("["):          # [::1]:8080
                host = host[:host.find("]") + 1]
            else:                             # localhost:8080
                host = host.rsplit(":", 1)[0]
            return host in LOOPBACK_HOSTS

        def _send_json(self, obj, status=HTTPStatus.OK):
            body = json.dumps(obj).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        # -- GET --------------------------------------------------------------

        def do_GET(self):
            if not self._host_ok():
                return self.send_error(HTTPStatus.FORBIDDEN)
            url = urlparse(self.path)
            if url.path == "/api/state":
                return self._send_json(app.state())
            if url.path == "/api/config":
                return self._send_json({
                    "modes": [{"key": m.key, "title": m.title, "description": m.description,
                               "real_robot": m.real_robot, "has_rviz": m.has_rviz_arg}
                              for m in MODES.values()],
                    "arms": [{"key": k, "title": arm_title(k),
                              "reach_mm": round(p.max_reach_m * 1000),
                              "payload_kg": p.payload_kg}
                             for k, p in sorted(PROFILES.items())],
                    "default_ur_type": app.default_ur_type,
                })
            if url.path == "/api/log":
                after = int((parse_qs(url.query).get("after") or ["0"])[0] or 0)
                lines = app.launch.lines_after(after)
                return self._send_json({"lines": lines, "seq": app.launch.seq})
            if url.path == "/stream.mjpg":
                return self._stream()
            name = "index.html" if url.path in ("/", "/index.html") else url.path.lstrip("/")
            return self._static(name)

        def _static(self, name):
            path = os.path.realpath(os.path.join(app.web_dir, name))
            if not path.startswith(os.path.realpath(app.web_dir) + os.sep) \
                    or not os.path.isfile(path):
                return self.send_error(HTTPStatus.NOT_FOUND)
            with open(path, "rb") as f:
                body = f.read()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type",
                             STATIC_TYPES.get(os.path.splitext(name)[1], "application/octet-stream"))
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _stream(self):
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            app.stream_client(+1)
            seq = 0
            try:
                while not app.shutting_down.is_set():
                    jpeg, seq = app.wait_frame(seq, timeout=2.0)
                    if jpeg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: %d\r\n\r\n" % len(jpeg))
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                app.stream_client(-1)

        # -- POST -------------------------------------------------------------

        def do_POST(self):
            # The custom header makes every cross-origin POST a CORS preflight,
            # which this server never answers — so another website open in the
            # operator's browser cannot press Start on their behalf.
            if not self._host_ok() or self.headers.get("X-RecycleBot") != "1":
                return self.send_error(HTTPStatus.FORBIDDEN)
            try:
                length = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(length) or b"{}") if length else {}
            except (ValueError, json.JSONDecodeError):
                return self._send_json({"error": "bad request"}, HTTPStatus.BAD_REQUEST)

            path = urlparse(self.path).path
            try:
                if path == "/api/start":
                    app.start(str(body.get("mode", "")), str(body.get("ur_type", "")),
                              bool(body.get("rviz", False)), bool(body.get("confirmed", False)))
                elif path == "/api/stop":
                    app.stop()
                elif path == "/api/continue":
                    if not app.continue_gate():
                        raise ValueError("the robot is not waiting for the pendant")
                elif path == "/api/dismiss":
                    with app._lock:
                        app.tracker.dismiss_hint(str(body.get("key", "")))
                else:
                    return self.send_error(HTTPStatus.NOT_FOUND)
            except ValueError as exc:
                return self._send_json({"error": str(exc)}, HTTPStatus.CONFLICT)
            except Exception as exc:
                app.get_logger().error(f"{path} failed: {exc}")
                return self._send_json({"error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)
            return self._send_json({"ok": True})

    return Handler


class _Server(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True    # open camera streams must not block shutdown


def main(args=None):
    # Our own handlers, not rclpy's: shutdown has to stop the robot launch before
    # the ROS context goes away, and rclpy's SIGINT handler would tear the
    # context down first.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    app = Dashboard()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(app)
    threading.Thread(target=executor.spin, daemon=True).start()

    try:
        server = _Server((app.bind_address, app.port), make_handler(app))
    except OSError as exc:
        app.get_logger().fatal(
            f"cannot listen on {app.bind_address}:{app.port}: {exc}. Is the "
            "dashboard already running?")
        app.destroy_node()
        rclpy.try_shutdown()
        raise SystemExit(1)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    app.get_logger().info(
        f"RecycleBot dashboard on http://{'localhost' if app.bind_address == '127.0.0.1' else app.bind_address}:{app.port}")

    done = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: done.set())
    signal.signal(signal.SIGTERM, lambda *_: done.set())
    done.wait()

    app.get_logger().info("shutting down — stopping the robot software first")
    app.shutting_down.set()
    app.stop(blocking=True)
    server.shutdown()
    executor.shutdown()
    app.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
