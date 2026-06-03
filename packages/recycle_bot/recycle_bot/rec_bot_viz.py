#!/usr/bin/env python3
import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

from builtin_interfaces.msg import Duration
from realsense2_camera_msgs.msg import RGBD
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel


def _class_color_bgr(label: str):
    """Deterministic BGR color per class label, always bright enough to see."""
    h = hash(label) & 0xFFFFFF
    r = max((h >> 16) & 0xFF, 80)
    g = max((h >> 8) & 0xFF, 80)
    b = max(h & 0xFF, 80)
    return (b, g, r)


def _class_color_rgb_float(label: str):
    """Deterministic RGB float color for RViz markers."""
    h = hash(label) & 0xFFFFFF
    r = max((h >> 16) & 0xFF, 80) / 255.0
    g = max((h >> 8) & 0xFF, 80) / 255.0
    b = max(h & 0xFF, 80) / 255.0
    return r, g, b


class DetectionVizNode(Node):
    def __init__(self):
        super().__init__("rec_bot_viz")

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._last_rgb: Image | None = None
        self._last_camera_info = None

        qos_camera = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_detections = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            RGBD,
            "/camera/camera/rgbd",
            self._rgbd_callback,
            qos_camera,
            callback_group=ReentrantCallbackGroup(),
        )
        self.create_subscription(
            Detection3DArray,
            "object_detections",
            self._detection_callback,
            qos_detections,
        )

        self._raw_image_pub = self.create_publisher(Image, "/rec_bot/camera_image", 10)
        self._image_pub = self.create_publisher(Image, "/rec_bot/detection_image", 10)
        self._marker_pub = self.create_publisher(MarkerArray, "/rec_bot/detection_markers", 10)

        self.get_logger().info(
            "rec_bot_viz ready — "
            "/rec_bot/camera_image | /rec_bot/detection_image | /rec_bot/detection_markers"
        )

    def _rgbd_callback(self, msg: RGBD):
        with self._lock:
            self._last_rgb = msg.rgb
            self._last_camera_info = msg.rgb_camera_info
        self._raw_image_pub.publish(msg.rgb)

    def _detection_callback(self, msg: Detection3DArray):
        if not msg.detections:
            return

        with self._lock:
            rgb_msg = self._last_rgb
            camera_info = self._last_camera_info

        # build camera model for 3D projection (mirrors rec_bot_core approach)
        camera_model: PinholeCameraModel | None = None
        if camera_info is not None:
            camera_model = PinholeCameraModel()
            camera_model.fromCameraInfo(camera_info)

        # --- console output (same style as /capture_detections service response) ---
        lines = [f"{len(msg.detections)} detection(s):"]
        for i, det in enumerate(msg.detections):
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            depth = det.bbox.center.position.z
            w = det.bbox.size.x
            h = det.bbox.size.y
            label = det.results[0].hypothesis.class_id if det.results else "unknown"
            conf = det.results[0].hypothesis.score if det.results else 0.0

            coord_str = "3D=N/A"
            if camera_model is not None and depth > 0.0:
                ray = camera_model.projectPixelTo3dRay((cx, cy))
                scale = depth / ray[2]
                x3d, y3d, z3d = ray[0] * scale, ray[1] * scale, depth
                coord_str = f"3D=(x={x3d:.3f} y={y3d:.3f} z={z3d:.3f})m"

            lines.append(
                f"  [{i}] {label:<20s}  conf={conf:.2f}"
                f"  bbox=(cx={cx:.0f} cy={cy:.0f} w={w:.0f} h={h:.0f})px"
                f"  depth={depth:.3f}m  {coord_str}"
            )

        self.get_logger().info("\n".join(lines))

        # --- annotated image ---
        if rgb_msg is not None:
            self._publish_annotated_image(rgb_msg, msg.detections)

        # --- 3D markers ---
        if camera_model is not None and camera_info is not None:
            self._publish_markers(
                msg.detections, camera_model, camera_info.header.frame_id, msg.header.stamp
            )

    def _publish_annotated_image(self, rgb_msg: Image, detections):
        try:
            bgr = self._bridge.imgmsg_to_cv2(rgb_msg, rgb_msg.encoding)
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        if rgb_msg.encoding == "rgb8":
            bgr = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR)

        for det in detections:
            cx = int(det.bbox.center.position.x)
            cy = int(det.bbox.center.position.y)
            w = int(det.bbox.size.x)
            h = int(det.bbox.size.y)
            label = det.results[0].hypothesis.class_id if det.results else "unknown"
            conf = det.results[0].hypothesis.score if det.results else 0.0

            x1, y1 = cx - w // 2, cy - h // 2
            x2, y2 = cx + w // 2, cy + h // 2
            color = _class_color_bgr(label)

            cv2.rectangle(bgr, (x1, y1), (x2, y2), color, 2)

            text = f"{label} {conf:.0%}"
            (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(bgr, (x1, y1 - th - baseline - 4), (x1 + tw + 4, y1), color, cv2.FILLED)
            cv2.putText(
                bgr, text, (x1 + 2, y1 - baseline - 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
            )

        out_msg = self._bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
        out_msg.header = rgb_msg.header
        self._image_pub.publish(out_msg)

    def _publish_markers(self, detections, camera_model: PinholeCameraModel, frame_id: str, stamp):
        marker_array = MarkerArray()
        lifetime = Duration(sec=0, nanosec=500_000_000)  # 0.5 s — auto-clears stale markers

        for i, det in enumerate(detections):
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            depth = det.bbox.center.position.z
            label = det.results[0].hypothesis.class_id if det.results else "unknown"
            conf = det.results[0].hypothesis.score if det.results else 0.0

            if depth <= 0.0:
                continue

            ray = camera_model.projectPixelTo3dRay((cx, cy))
            scale = depth / ray[2]
            x3d, y3d, z3d = ray[0] * scale, ray[1] * scale, depth

            r, g, b = _class_color_rgb_float(label)

            sphere = Marker()
            sphere.header.frame_id = frame_id
            sphere.header.stamp = stamp
            sphere.ns = "detections"
            sphere.id = i * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x3d
            sphere.pose.position.y = y3d
            sphere.pose.position.z = z3d
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.05
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = r, g, b, 0.9
            sphere.lifetime = lifetime
            marker_array.markers.append(sphere)

            label_marker = Marker()
            label_marker.header.frame_id = frame_id
            label_marker.header.stamp = stamp
            label_marker.ns = "detection_labels"
            label_marker.id = i * 2 + 1
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD
            label_marker.pose.position.x = x3d
            label_marker.pose.position.y = y3d
            label_marker.pose.position.z = z3d + 0.08
            label_marker.pose.orientation.w = 1.0
            label_marker.scale.z = 0.04
            label_marker.color.r = label_marker.color.g = label_marker.color.b = 1.0
            label_marker.color.a = 1.0
            label_marker.text = f"{label} ({conf:.0%})"
            label_marker.lifetime = lifetime
            marker_array.markers.append(label_marker)

        self._marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = DetectionVizNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
