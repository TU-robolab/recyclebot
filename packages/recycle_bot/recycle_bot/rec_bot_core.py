#!/usr/bin/env python3
import os
import time
import yaml
from threading import Lock

# ros imports
import rclpy
import rclpy.time
import tf2_ros
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from realsense2_camera_msgs.msg import RGBD
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Pose, Quaternion, TransformStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

from recycle_bot.fallback_pick import (
    DEPTH_SCALE_BY_ENCODING,
    FALLBACK_LABEL,
    FallbackPickError,
    Pinhole,
    RigidTransform,
    measure_pick_height,
    parse_config as parse_fallback_config,
    sample_depth,
)
from recycle_bot.robot_profile import config_path, profile, resolve_ur_type

# A fallback pick measures height from the latest depth frame; older than this
# and the camera has probably stopped, so the reading would describe the past.
FALLBACK_MAX_FRAME_AGE_S = 2.0

class RecBotCore(Node):

    def __init__(self):
        super().__init__("rec_bot_core")

        # which UR arm this cell is running. Selects config/<ur_type>/ for both
        # the camera transform and the detection filter — the camera sits at a
        # different height on each arm's cell, so loading the wrong one silently
        # mis-projects every detection.
        self.ur_type = resolve_ur_type(
            self.declare_parameter("ur_type", "").value or None
        )
        self.profile = profile(self.ur_type)
        self.get_logger().info(f"rec_bot_core starting for {self.profile}")

        # RGBD data (protected by rgbd_lock)
        self.rgbd_lock = Lock()  # protects: last_depth_image, last_camera_info, last_depth_info, last_rgbd_time
        self.last_depth_image = None
        self.last_camera_info = None
        self.last_depth_info = None
        self.last_rgbd_time = 0.0

        # detection filtering config (loaded from YAML with defaults)
        self.load_filter_config()

        # Fallback pick (see recycle_bot/fallback_pick.py): /fallback_pick picks
        # whatever sits on a configured spot, measuring only its height — for
        # when recognition fails. Needs "base" -> camera TF to find the spot in
        # the depth image, and the robot-busy latch so it never measures while
        # the arm is in view.
        self.fallback = self.load_fallback_config()
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.robot_busy = False
        busy_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool, "/rec_bot/robot_busy", self._robot_busy_callback, busy_qos
        )
        self.create_service(Trigger, "/fallback_pick", self.fallback_pick_callback)

        # setup ROS quality of service for camera frames
        qos_camera_feed = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,  # keep only the latest frames
            depth=5,  # buffer up to 5 frames
            reliability=ReliabilityPolicy.BEST_EFFORT, # Drop frames if necessary for speed
            durability=DurabilityPolicy.VOLATILE  # no need to keep old frames
        )

        # setup ROS quality of service for detected poses
        qos_detected_objects = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,  # store recent messages
            depth=10,  # buffer up to 10 detections
            reliability=ReliabilityPolicy.RELIABLE,  # ensure all detections arrive
            durability=DurabilityPolicy.VOLATILE  # no need to retain past detections
        )

        self.image_sub = self.create_subscription(
            RGBD,
            "/camera/camera/rgbd",
            self.image_callback,
            qos_camera_feed, 
            callback_group=ReentrantCallbackGroup()
        )

        self.subscription = self.create_subscription(
            Detection3DArray,
            'object_detections',
            self.detection_callback,
            qos_detected_objects  # RELIABLE to match vision publisher
        )

        self.detected_object_pub = self.create_publisher(
            Detection3D,
            "/vision/detected_object",
            qos_detected_objects
        )

        self.publish_camera_static_transform()

    """ 
    thread-safe image callback, only 1 thread can update 
    last image at a time, with statement ensures lock lifecycle
    is automated (creation and release)
    image type is realsense2_msgs.msg (RGBD) -> cv image etc. 
    """
    def image_callback(self, msg):
        with self.rgbd_lock:
            self.last_depth_image = msg.depth
            self.last_camera_info = msg.rgb_camera_info
            self.last_depth_info  = msg.depth_camera_info
            self.last_rgbd_time = time.monotonic()

    def _robot_busy_callback(self, msg: Bool):
        self.robot_busy = msg.data

    def load_fallback_config(self):
        """fallback_pick from sorting_sequence.yaml, or None (service then refuses)."""
        yaml_path = config_path(self.ur_type, "sorting_sequence.yaml")
        try:
            with open(yaml_path, 'r') as file:
                cfg = parse_fallback_config(yaml.safe_load(file))
        except Exception as e:
            self.get_logger().error(f"Fallback pick disabled: bad fallback_pick in {yaml_path}: {e}")
            return None
        if cfg is None:
            self.get_logger().info(f"Fallback pick disabled: no fallback_pick in {yaml_path}")
        else:
            self.get_logger().info(
                f"Fallback pick spot: ({cfg.x:.3f}, {cfg.y:.3f}) in base; "
                "call /fallback_pick to pick from it"
            )
        return cfg

    def fallback_pick_callback(self, request, response):
        try:
            response.message = self.fallback_pick()
            response.success = True
        except FallbackPickError as e:
            detail = f" ({e.detail})" if e.detail else ""
            self.get_logger().warn(f"Fallback pick refused: {e}{detail}")
            response.success = False
            response.message = str(e)
        return response

    def fallback_pick(self) -> str:
        """Measure the height on the fallback spot and send a pick there to control.

        The pick goes out on /vision/detected_object like any detection, but in
        the "base" frame with the configured x/y kept exactly — only z is
        measured. Control then applies its usual reach check, duplicate-task
        rejection and routing (the label has no bin_routing rule, so it goes to
        default_bin).
        """
        if self.fallback is None:
            raise FallbackPickError(
                f"No fallback spot is configured for the {self.ur_type}. A "
                "technician can add fallback_pick to sorting_sequence.yaml.")
        # Same reason vision gates capture on this: with the arm in view the
        # depth at the spot is the arm, and the pick would aim at thin air.
        if self.robot_busy:
            raise FallbackPickError("The robot is moving. Wait until it stops, then try again.")

        with self.rgbd_lock:
            depth_msg = self.last_depth_image
            camera_info = self.last_camera_info
            frame_age = time.monotonic() - self.last_rgbd_time
        if depth_msg is None or camera_info is None or frame_age > FALLBACK_MAX_FRAME_AGE_S:
            raise FallbackPickError("No camera image. Check that the camera is running.")

        scale = DEPTH_SCALE_BY_ENCODING.get(depth_msg.encoding)
        if scale is None:
            raise FallbackPickError(f"Unsupported depth encoding '{depth_msg.encoding}'.")

        frame_id = depth_msg.header.frame_id
        try:
            tf = self.tf_buffer.lookup_transform(frame_id, "base", rclpy.time.Time())
        except tf2_ros.TransformException as e:
            raise FallbackPickError(
                f"Cannot place the spot in the camera image yet (no transform "
                f"'base' -> '{frame_id}': {e}). Is the robot driver running?")

        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        reading = measure_pick_height(
            self.fallback,
            RigidTransform.from_msg(tf.transform),
            Pinhole.from_camera_info(camera_info),
            lambda u, v, half_px: sample_depth(depth, u, v, half_px, scale),
            self.min_depth_m,
            self.max_depth_m,
        )

        x, y, z = self.fallback.x, self.fallback.y, reading.z
        self.get_logger().info(
            f"Fallback pick: surface at z={z:.3f} m on the spot ({x:.3f}, {y:.3f}) in base "
            f"(depth {reading.depth_m:.3f} m around pixel "
            f"({reading.pixel[0]:.0f}, {reading.pixel[1]:.0f}))"
        )

        out = Detection3D()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base"
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = FALLBACK_LABEL
        # score stays 0.0: there is no recognition behind this pick, and nothing
        # downstream of core filters on confidence
        hypothesis.pose.pose.position.x = x
        hypothesis.pose.pose.position.y = y
        hypothesis.pose.pose.position.z = z
        hypothesis.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        out.results.append(hypothesis)
        self.detected_object_pub.publish(out)

        return f"Picking at x={x:.3f}, y={y:.3f}, z={z:.3f} m (base frame)"

    def load_filter_config(self):
        """
        Load detection filter config from YAML with defaults.

        Filter thresholds:
        - min_confidence: minimum detection confidence (0.0-1.0)
        - min_depth_m: minimum valid depth in meters (rejects too close)
        - max_depth_m: maximum valid depth in meters (rejects too far)

        RealSense D415 range: 0.3m - 10m (optimal 0.5m - 3m)
        """
        defaults = {
            "min_confidence": 0.75,
            "min_depth_m": 0.3,
            "max_depth_m": 1.5
        }

        yaml_path = config_path(self.ur_type, "calibration.yaml")
        try:
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)

            config = data.get("detection_filter", {})
            self.min_confidence = config.get("min_confidence", defaults["min_confidence"])
            self.min_depth_m = config.get("min_depth_m", defaults["min_depth_m"])
            self.max_depth_m = config.get("max_depth_m", defaults["max_depth_m"])

            self.get_logger().info(
                f"Detection filter: confidence>={self.min_confidence}, "
                f"depth in [{self.min_depth_m}, {self.max_depth_m}]m"
            )

        except Exception as e:
            self.get_logger().warn(f"Failed to load filter config: {e}, using defaults")
            self.min_confidence = defaults["min_confidence"]
            self.min_depth_m = defaults["min_depth_m"]
            self.max_depth_m = defaults["max_depth_m"]

    def detection_callback(self, msg: Detection3DArray):
        for detection in msg.detections:
            self.process_detection(detection)

    def process_detection(self, detection: Detection3D):
        # bbox.center.position:
        #   x, y = pixel coordinates of bbox center
        #   z = average valid depth in meters (0 if no valid depth)
        u = int(detection.bbox.center.position.x)
        v = int(detection.bbox.center.position.y)
        z = detection.bbox.center.position.z  # avg depth in meters from vision node
        #z = 0.6

        # get confidence and class label from first hypothesis (if available)
        confidence = 0.0
        label = ""
        if detection.results:
            confidence = detection.results[0].hypothesis.score
            label = detection.results[0].hypothesis.class_id

        self.get_logger().info(
            f"Detection: (u={u}, v={v}), depth={z:.3f}m, conf={confidence:.2f}, label='{label}'"
        )

        # filter by confidence
        if confidence < self.min_confidence:
            self.get_logger().warn(f"Low confidence ({confidence:.2f} < {self.min_confidence}), skipping")
            return

        # filter by depth range
        if z == 0.0:
            self.get_logger().warn("No valid depth for detection, skipping")
            return

        if z < self.min_depth_m:
            self.get_logger().warn(f"Object too close ({z:.3f}m < {self.min_depth_m}m), skipping")
            return

        if z > self.max_depth_m:
            self.get_logger().warn(f"Object too far ({z:.3f}m > {self.max_depth_m}m), skipping")
            return

        # keep lock on camera info only as long as necessary
        with self.rgbd_lock:
            if self.last_camera_info is None:
                self.get_logger().info("No camera info available")
                return
            if self.last_depth_image is None:
                self.get_logger().info("No depth image available for frame_id")
                return

            camera_info = self.last_camera_info
            frame_id = self.last_depth_image.header.frame_id

        self.get_logger().info(f"point candidate (u, v, z): ({u}, {v}, {z:.3f}m)")

        # get correct model to process the camera from pixel space into world
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info)

        # project pixel to 3D point in camera space
        #
        # RealSense provides Z-depth (perpendicular to image plane), not ray distance.
        # we scale the ray to reach the Z-depth plane:
        #
        #        camera
        #           ·
        #          /|
        #         / |
        #        /  | Z-depth (what RealSense gives)
        #       / θ |
        #      /    |
        #     ·-----+ point at (x, y, z)
        #      \
        #       ray (unit vector)
        #
        # scale = z / ray[2] = z / cos(θ)
        #
        ray = camera_model.projectPixelTo3dRay((u, v))  # unit vector
        scale = z / ray[2]  # ray[2] ≈ 1.0 for center, < 1.0 for edges
        x = ray[0] * scale
        y = ray[1] * scale
        # z remains as the original Z-depth

        # build the projected pose in the camera frame
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        #pose.position.z = 0.16  # fixed height above table to avoid grasping issues, can be tuned based on testing

        # Orientation is intentionally left as identity here.
        # The pick orientation (face-down) is set in base frame by rec_bot_control
        # after the camera→base TF transform.
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # publish a Detection3D so the label rides along with the pose:
        #   header.frame_id → camera frame (control TF-transforms to base)
        #   results[0].hypothesis.class_id → label (used for bin routing)
        #   results[0].pose.pose → projected 3D pose
        out = Detection3D()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = frame_id  # usually "camera_link" or similar

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = label
        hypothesis.hypothesis.score = confidence
        hypothesis.pose.pose = pose
        out.results.append(hypothesis)

        self.get_logger().info(
            f"publishing detection: label='{label}', pos=({x:.3f}, {y:.3f}, {z:.3f})"
        )
        self.detected_object_pub.publish(out)

    def load_camera_transform(self):
        """
        Load camera transform from YAML config with defaults.

        Returns dict with keys: parent_frame, child_frame, translation, rotation
        """
        # fallback: keep in sync with config/calibration.yaml (a wrong camera TF
        # breaks every downstream 3D projection, so a load failure is logged as
        # an error below)
        defaults = {
            "parent_frame": "base_link",
            "child_frame": "camera_link",
            "translation": [0.35, -0.29, 0.61],
            "rotation": [-0.5, 0.5, 0.5, 0.5]
        }

        yaml_path = config_path(self.ur_type, "calibration.yaml")
        try:
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)

            config = data.get("camera_transform", {})
            result = {
                "parent_frame": config.get("parent_frame", defaults["parent_frame"]),
                "child_frame": config.get("child_frame", defaults["child_frame"]),
                "translation": config.get("translation", defaults["translation"]),
                "rotation": config.get("rotation", defaults["rotation"])
            }
            self.get_logger().info(f"Loaded camera transform from config: {result['parent_frame']} -> {result['child_frame']}")
            return result

        except Exception as e:
            self.get_logger().error(
                f"Failed to load camera transform from config: {e}; using hardcoded "
                "defaults — verify these match the physical camera mount!"
            )
            return defaults

    def publish_camera_static_transform(self):
        """Publish static transform from base to camera using config or defaults."""
        config = self.load_camera_transform()

        # Keep the broadcaster alive on self: static TF uses a latched
        # (TRANSIENT_LOCAL) publisher, and if the broadcaster is garbage-collected
        # the latched transform is lost for late-joining subscribers (e.g. the
        # control node, whose MoveItPy init takes several seconds).
        self._static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        static_br = self._static_broadcaster
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = config["parent_frame"]
        transform.child_frame_id = config["child_frame"]
        transform.transform.translation.x = config["translation"][0]
        transform.transform.translation.y = config["translation"][1]
        transform.transform.translation.z = config["translation"][2]
        transform.transform.rotation.x = config["rotation"][0]
        transform.transform.rotation.y = config["rotation"][1]
        transform.transform.rotation.z = config["rotation"][2]
        transform.transform.rotation.w = config["rotation"][3]
        static_br.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    rec_bot_core = RecBotCore()

    try:
        executor.add_node(rec_bot_core)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rec_bot_core.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
