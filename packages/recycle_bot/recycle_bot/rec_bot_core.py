#!/usr/bin/env python3
import os
import yaml
from threading import Lock

# ros imports
import rclpy
import tf2_ros
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from realsense2_camera_msgs.msg import RGBD
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler

import numpy as np

class RecBotCore(Node):

    def __init__(self):
        super().__init__("rec_bot_core")
        self.get_logger().info("Hello world from the Python node rec_bot_core")

        # RGBD data (protected by rgbd_lock)
        self.bridge = CvBridge()
        self.rgbd_lock = Lock()  # protects: last_depth_image, last_camera_info, last_depth_info
        self.last_depth_image = None
        self.last_camera_info = None
        self.last_depth_info = None

        # detection filtering config (loaded from YAML with defaults)
        self.load_filter_config()

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
            PoseStamped,
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
            "min_confidence": 0.5,
            "min_depth_m": 0.3,
            "max_depth_m": 1.5
        }

        yaml_path = os.path.join(get_package_share_directory("recycle_bot"), "config", "calibration.yaml")
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

        # get confidence from first hypothesis (if available)
        confidence = 0.0
        if detection.results:
            confidence = detection.results[0].hypothesis.score

        self.get_logger().info(f"Detection: (u={u}, v={v}), depth={z:.3f}m, conf={confidence:.2f}")

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

        # create PoseStamped
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id  # usually "camera_link" or similar

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # Tool pointing down = aligned with -Z → euler (roll=pi, pitch=0, yaw=0)
        q = quaternion_from_euler(np.pi, 0, 0)  # facing downward
        pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.detected_object_pub.publish(pose)

    def load_camera_transform(self):
        """
        Load camera transform from YAML config with defaults.

        Returns dict with keys: parent_frame, child_frame, translation, rotation
        """
        # defaults (original hardcoded values)
        defaults = {
            "parent_frame": "base",
            "child_frame": "camera_link",
            "translation": [-0.384, 0.286, 0.624],
            "rotation": [-0.999, 0.045, 0.0, 0.0]
        }

        yaml_path = os.path.join(get_package_share_directory("recycle_bot"), "config", "calibration.yaml")
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
            self.get_logger().warn(f"Failed to load camera transform from config: {e}, using defaults")
            return defaults

    def publish_camera_static_transform(self):
        """Publish static transform from base to camera using config or defaults."""
        config = self.load_camera_transform()

        static_br = tf2_ros.StaticTransformBroadcaster(self)
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
