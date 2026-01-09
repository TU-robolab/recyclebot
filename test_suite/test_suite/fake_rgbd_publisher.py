#!/usr/bin/env python3
"""
Fake RGBD publisher for testing vision node without real camera hardware.
Publishes synthetic RGBD images to /camera/camera/rgbd topic.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from realsense2_camera_msgs.msg import RGBD
from cv_bridge import CvBridge
import numpy as np
import cv2


class FakeRGBDPublisher(Node):
    def __init__(self):
        super().__init__('fake_rgbd_publisher')

        # Setup QoS profile matching what the vision node expects
        qos_camera_feed = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Create publisher
        self.publisher = self.create_publisher(
            RGBD,
            '/camera/camera/rgbd',
            qos_camera_feed
        )

        # Create timer to publish at 5 Hz (slower than real camera for testing)
        self.timer = self.create_timer(0.2, self.publish_fake_rgbd)

        self.bridge = CvBridge()
        self.frame_count = 0

        self.get_logger().info('Fake RGBD Publisher started - publishing to /camera/camera/rgbd')

    def create_fake_image(self):
        """Create a fake RGB image with some shapes that YOLO might detect"""
        # Create a 1280x720 image (typical RealSense resolution)
        width, height = 1280, 720
        img = np.ones((height, width, 3), dtype=np.uint8) * 200  # Light gray background

        # Add some colorful shapes to simulate objects
        # Draw some rectangles (simulating bottles/boxes)
        cv2.rectangle(img, (200, 200), (350, 500), (0, 0, 255), -1)  # Red rectangle
        cv2.rectangle(img, (500, 150), (650, 400), (0, 255, 0), -1)  # Green rectangle
        cv2.rectangle(img, (800, 250), (950, 550), (255, 0, 0), -1)  # Blue rectangle

        # Add some circles (simulating cups/bottles)
        cv2.circle(img, (400, 600), 60, (255, 255, 0), -1)  # Cyan circle
        cv2.circle(img, (900, 600), 50, (255, 0, 255), -1)  # Magenta circle

        # Add some text
        cv2.putText(img, f'Frame {self.frame_count}', (50, 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

        return img

    def create_fake_depth(self):
        """Create a fake depth image"""
        # Create a 1280x720 depth image
        width, height = 1280, 720

        # Create gradient depth (closer at top, farther at bottom)
        depth = np.zeros((height, width), dtype=np.uint16)
        for i in range(height):
            depth[i, :] = int(500 + (i / height) * 2000)  # Range from 500mm to 2500mm

        return depth

    def create_camera_info(self):
        """Create fake camera info"""
        camera_info = CameraInfo()
        camera_info.width = 1280
        camera_info.height = 720

        # Typical RealSense D435 intrinsics (approximate)
        fx = 900.0
        fy = 900.0
        cx = 640.0
        cy = 360.0

        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        camera_info.distortion_model = "plumb_bob"
        camera_info.header.frame_id = "camera_color_optical_frame"

        return camera_info

    def publish_fake_rgbd(self):
        """Publish a fake RGBD message"""
        # Create RGBD message
        rgbd_msg = RGBD()

        # Set timestamp
        current_time = self.get_clock().now().to_msg()
        rgbd_msg.header.stamp = current_time
        rgbd_msg.header.frame_id = "camera_color_optical_frame"

        # Create fake RGB image
        rgb_img = self.create_fake_image()
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_img, encoding="bgr8")
        rgb_msg.header = rgbd_msg.header
        rgbd_msg.rgb = rgb_msg

        # Create fake depth image
        depth_img = self.create_fake_depth()
        depth_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding="passthrough")
        depth_msg.header = rgbd_msg.header
        rgbd_msg.depth = depth_msg

        # Create camera info
        camera_info = self.create_camera_info()
        camera_info.header = rgbd_msg.header
        rgbd_msg.rgb_camera_info = camera_info
        rgbd_msg.depth_camera_info = camera_info

        # Publish
        self.publisher.publish(rgbd_msg)

        self.frame_count += 1
        if self.frame_count % 25 == 0:
            self.get_logger().info(f'Published {self.frame_count} fake RGBD frames')


def main(args=None):
    rclpy.init(args=args)
    node = FakeRGBDPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()