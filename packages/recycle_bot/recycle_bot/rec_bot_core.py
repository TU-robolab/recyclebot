#!/usr/bin/env python3
from threading import Lock

# ros imports
import rclpy
import tf2_ros
from rclpy.node import Node

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

        # image configuration
        self.bridge = CvBridge()
        self.image_lock = Lock()
        self.last_depth_image = None
        self.last_camera_info = None
        self.last_depth_info  = None

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
            10  # or another qos
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
        with self.image_lock:
            self.last_depth_image = msg.depth
            self.last_camera_info = msg.rgb_camera_info
            self.last_depth_info  = msg.depth_camera_info
        
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

        self.get_logger().info(f"Detection center: (u={u}, v={v}), depth={z:.3f}m")

        if z == 0.0:
            self.get_logger().warn("No valid depth for detection, skipping")
            return

        # keep lock on camera info only as long as necessary
        with self.image_lock:
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
        #        camera
        #           ·
        #          /|\
        #         / | \
        #        /  |  \   ray = unit vector from camera through pixel (u, v)
        #       /   |z  \
        #      /    |    \
        #     /     ↓     \
        #    ·------·------·  image plane at depth z
        #          (x, y, z)
        #
        ray = camera_model.projectPixelTo3dRay((u, v))  # unit vector
        x = ray[0] * z
        y = ray[1] * z
        z = ray[2] * z

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

    def publish_camera_static_transform(self):
        static_br = tf2_ros.StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "base"
        transform.child_frame_id = "camera_link" # change to published by system if needed
        transform.transform.translation.x = -0.38384216583910713
        transform.transform.translation.y = 0.2863018787024152
        transform.transform.translation.z = 0.6239699702309053
        transform.transform.rotation.x = -0.9989747131951828
        transform.transform.rotation.y = 0.04527164772325851
        transform.transform.rotation.z = -1.2163534954626416e-05
        transform.transform.rotation.w = 1.2691403055540589e-05
        static_br.sendTransform(transform)

                               
        

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    rec_bot_core = RecBotCore()

    try:
        executor.spin(rec_bot_core)
    except KeyboardInterrupt:
        pass
    finally:
        rec_bot_core.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
