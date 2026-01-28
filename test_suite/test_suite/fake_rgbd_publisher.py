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

        # Create timer to publish at 6 Hz
        self.timer = self.create_timer(1.0/6.0, self.publish_fake_rgbd)

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
        """
        Create a fake depth image matching RealSense D415 characteristics.

        D415 specs:
        - Range: 0.3m to ~3m
        - Format: Z16 (16-bit unsigned integer depth values)
        - Resolution: 1280x720 @ 6fps

        DEPTH SCALE:
        - RealSense cameras use a depth_scale factor (typically 0.001 for D415)
        - Raw Z16 value * depth_scale = depth in meters
        - With depth_scale=0.001: raw value 1000 = 1.0 meter
        - This publisher stores values directly in MILLIMETERS (depth_scale=1.0)
        - So raw value 1000 = 1000mm = 1.0 meter
        - To match real D415 with depth_scale=0.001, divide values by 1000
        """
        width, height = 1280, 720

        # Initialize with table depth (from calibration: ~0.624m)
        depth = np.full((height, width), 624, dtype=np.uint16)

        # Add depth values for the objects matching the RGB image positions
        # Objects closer to camera have lower depth values (5-25 cm above table)

        # Red rectangle (200, 200, 350, 500) - at 574mm (5 cm above table)
        depth[200:500, 200:350] = 574

        # Green rectangle (500, 150, 650, 400) - at 554mm (7 cm above table)
        depth[150:400, 500:650] = 554

        # Blue rectangle (800, 250, 950, 550) - at 524mm (10 cm above table)
        depth[250:550, 800:950] = 524

        # Cyan circle at (400, 600) radius 60 - at 474mm (15 cm above table)
        y, x = np.ogrid[:height, :width]
        mask_cyan = (x - 400)**2 + (y - 600)**2 <= 60**2
        depth[mask_cyan] = 474

        # Magenta circle at (900, 600) radius 50 - at 374mm (25 cm above table)
        mask_magenta = (x - 900)**2 + (y - 600)**2 <= 50**2
        depth[mask_magenta] = 374

        # Add realistic noise to simulate sensor noise (±2mm standard deviation)
        noise = np.random.normal(0, 2, (height, width)).astype(np.int16)
        depth = np.clip(depth.astype(np.int32) + noise, 300, 3000).astype(np.uint16)

        # Add some invalid depth regions (0 value) to simulate areas where depth cannot be measured
        # Typically happens at edges or reflective surfaces
        invalid_mask = np.random.random((height, width)) < 0.02  # 2% invalid pixels
        depth[invalid_mask] = 0

        return depth

    def create_camera_info(self):
        """Create fake camera info matching RealSense D415 specifications"""
        camera_info = CameraInfo()
        camera_info.width = 1280
        camera_info.height = 720

        # RealSense D415 intrinsics for 1280x720 (approximate typical values)
        # D415 has a narrower FOV than D435
        fx = 920.0  # focal length in x
        fy = 920.0  # focal length in y
        cx = 640.0  # principal point x (image center)
        cy = 360.0  # principal point y (image center)

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
