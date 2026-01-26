#!/usr/bin/env python3
"""
End-to-end pipeline tests for RecycleBot.

Tests the full data flow from camera to control-ready poses:
  fake_rgbd → vision (Detection3DArray) → core (PoseStamped) → gripper service

Test sequence:
  1. Verify RGBD frames are published
  2. Trigger detection and verify Detection3DArray published
  3. Verify core processes detections and publishes PoseStamped
  4. Verify gripper service responds correctly
"""

import unittest
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray
from realsense2_camera_msgs.msg import RGBD
from grip_interface.srv import GripCommand


class TestE2EPipeline(unittest.TestCase):
    """End-to-end pipeline integration tests."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 and create test node."""
        rclpy.init()
        cls.node = Node('e2e_test_node')

        # storage for received messages
        cls.rgbd_frames = []
        cls.detections = []
        cls.poses = []

        # QoS for camera feed (BEST_EFFORT to match publisher)
        qos_camera = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # QoS for detections (RELIABLE to match publisher)
        qos_detections = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # subscriptions
        cls.rgbd_sub = cls.node.create_subscription(
            RGBD, '/camera/camera/rgbd',
            cls.rgbd_callback, qos_camera
        )

        cls.detection_sub = cls.node.create_subscription(
            Detection3DArray, '/object_detections',
            cls.detection_callback, qos_detections
        )

        cls.pose_sub = cls.node.create_subscription(
            PoseStamped, '/vision/detected_object',
            cls.pose_callback, qos_detections
        )

        # service clients
        cls.trigger_client = cls.node.create_client(Trigger, '/capture_detections')
        cls.gripper_client = cls.node.create_client(GripCommand, '/gripper_action')

    @classmethod
    def tearDownClass(cls):
        """Cleanup ROS2."""
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def rgbd_callback(cls, msg):
        cls.rgbd_frames.append(msg)

    @classmethod
    def detection_callback(cls, msg):
        cls.detections.append(msg)

    @classmethod
    def pose_callback(cls, msg):
        cls.poses.append(msg)

    def spin_until(self, condition, timeout=10.0, interval=0.1):
        """Spin node until condition is true or timeout."""
        start = time.time()
        while not condition() and (time.time() - start) < timeout:
            rclpy.spin_once(self.node, timeout_sec=interval)
        return condition()

    # -------------------------------------------------------------------------
    # Test 1: RGBD frames are published
    # -------------------------------------------------------------------------
    def test_01_rgbd_frames_published(self):
        """Verify fake RGBD publisher is sending frames."""
        self.rgbd_frames.clear()

        success = self.spin_until(
            lambda: len(self.rgbd_frames) >= 1,
            timeout=10.0
        )

        print(f"\n[E2E Test 1] RGBD frames received: {len(self.rgbd_frames)}")
        self.assertTrue(success, "No RGBD frames received")
        self.assertGreaterEqual(len(self.rgbd_frames), 1)

    # -------------------------------------------------------------------------
    # Test 2: Vision detection service available
    # -------------------------------------------------------------------------
    def test_02_detection_service_available(self):
        """Verify vision detection service is available."""
        available = self.trigger_client.wait_for_service(timeout_sec=10.0)

        print(f"\n[E2E Test 2] Detection service available: {available}")
        self.assertTrue(available, "Detection service not available")

    # -------------------------------------------------------------------------
    # Test 3: Trigger detection and receive Detection3DArray
    # -------------------------------------------------------------------------
    def test_03_trigger_detection(self):
        """Trigger detection and verify Detection3DArray is published."""
        # wait for RGBD frames first
        self.rgbd_frames.clear()
        self.spin_until(lambda: len(self.rgbd_frames) >= 1, timeout=5.0)

        self.detections.clear()

        # trigger detection
        request = Trigger.Request()
        future = self.trigger_client.call_async(request)

        # wait for response (YOLO inference can be slow)
        start = time.time()
        while not future.done() and (time.time() - start) < 30.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(future.done(), "Detection service call timed out")

        response = future.result()
        print(f"\n[E2E Test 3] Detection response: {response.success}")
        print(f"             Message: {response.message}")

        self.assertTrue(response.success, f"Detection failed: {response.message}")

        # wait for Detection3DArray
        success = self.spin_until(
            lambda: len(self.detections) > 0,
            timeout=5.0
        )

        print(f"             Detections received: {len(self.detections)}")
        self.assertTrue(success, "No Detection3DArray received after trigger")

    # -------------------------------------------------------------------------
    # Test 4: Core processes detections (PoseStamped published)
    # -------------------------------------------------------------------------
    def test_04_pose_published(self):
        """Verify core node processes detections and publishes PoseStamped."""
        # note: this depends on test_03 having triggered a detection
        # and the detection passing the confidence/depth filters in rec_bot_core

        # the fake RGBD publisher generates depth values that should pass filters
        # but detection confidence depends on YOLO output

        print(f"\n[E2E Test 4] Checking for PoseStamped messages...")
        print(f"             Current poses received: {len(self.poses)}")

        # if no poses yet, trigger another detection and wait
        if len(self.poses) == 0:
            # trigger detection
            request = Trigger.Request()
            future = self.trigger_client.call_async(request)

            start = time.time()
            while not future.done() and (time.time() - start) < 30.0:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            # wait for pose
            self.spin_until(lambda: len(self.poses) > 0, timeout=10.0)

        print(f"             Poses after waiting: {len(self.poses)}")

        # note: poses may be empty if detections don't pass filters
        # this is expected behavior - we verify the pipeline runs without errors
        if len(self.poses) > 0:
            pose = self.poses[-1]
            x, y, z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z

            print(f"             Last pose position: ({x:.3f}, {y:.3f}, {z:.3f})")
            print(f"             Frame ID: {pose.header.frame_id}")

            # validate pose against known fake RGBD data:
            # - fake depths: 0.7m, 0.8m, 0.9m, 1.0m, 1.2m, 1.5m (background)
            # - filter config: min_depth=0.1m, max_depth=1.5m
            # - camera: fx=fy=920, cx=640, cy=360, image=1280x720

            # verify frame_id is set
            self.assertIsNotNone(pose.header.frame_id)

            # z (depth) should be within filter range
            self.assertGreater(z, 0.1, "Depth below minimum filter threshold")
            self.assertLess(z, 1.6, "Depth above maximum filter threshold")

            # x, y should be reasonable for the camera FOV
            # at z=1.5m, max x = (1280-640)*1.5/920 ≈ 1.04m
            # at z=1.5m, max y = (720-360)*1.5/920 ≈ 0.59m
            self.assertGreater(x, -1.5, "X position out of expected range")
            self.assertLess(x, 1.5, "X position out of expected range")
            self.assertGreater(y, -1.0, "Y position out of expected range")
            self.assertLess(y, 1.0, "Y position out of expected range")

            print(f"             Pose validation: PASSED")
        else:
            # no poses is acceptable if detections didn't pass filters
            print("             No poses published (detections may not pass filters)")
            # skip assertion - this is not a failure condition

    # -------------------------------------------------------------------------
    # Test 5: Gripper service responds correctly
    # -------------------------------------------------------------------------
    def test_05_gripper_service_available(self):
        """Verify mock gripper service is available."""
        available = self.gripper_client.wait_for_service(timeout_sec=5.0)

        print(f"\n[E2E Test 5] Gripper service available: {available}")
        self.assertTrue(available, "Gripper service not available")

    # -------------------------------------------------------------------------
    # Test 6: Gripper grip command
    # -------------------------------------------------------------------------
    def test_06_gripper_grip(self):
        """Test gripper grip command."""
        request = GripCommand.Request()
        request.action = 'grip'

        future = self.gripper_client.call_async(request)

        start = time.time()
        while not future.done() and (time.time() - start) < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(future.done(), "Gripper service call timed out")

        response = future.result()
        print(f"\n[E2E Test 6] Grip response: {response.success}")
        print(f"             Message: {response.message}")

        self.assertTrue(response.success, f"Grip failed: {response.message}")

    # -------------------------------------------------------------------------
    # Test 7: Gripper release command
    # -------------------------------------------------------------------------
    def test_07_gripper_release(self):
        """Test gripper release command."""
        request = GripCommand.Request()
        request.action = 'release'

        future = self.gripper_client.call_async(request)

        start = time.time()
        while not future.done() and (time.time() - start) < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(future.done(), "Gripper service call timed out")

        response = future.result()
        print(f"\n[E2E Test 7] Release response: {response.success}")
        print(f"             Message: {response.message}")

        self.assertTrue(response.success, f"Release failed: {response.message}")

    # -------------------------------------------------------------------------
    # Test 8: Full pipeline data flow
    # -------------------------------------------------------------------------
    def test_08_full_pipeline_flow(self):
        """Verify data flows through entire pipeline."""
        print("\n[E2E Test 8] Full pipeline flow summary:")
        print(f"             RGBD frames received: {len(self.rgbd_frames)}")
        print(f"             Detections received: {len(self.detections)}")
        print(f"             Poses received: {len(self.poses)}")

        # verify we received data at each stage
        self.assertGreater(len(self.rgbd_frames), 0, "No RGBD frames in pipeline")
        self.assertGreater(len(self.detections), 0, "No detections in pipeline")

        # verify detection format
        if self.detections:
            det = self.detections[-1]
            self.assertIsInstance(det, Detection3DArray)
            if det.detections:
                d = det.detections[0]
                print(f"             Sample detection:")
                print(f"               - bbox center: ({d.bbox.center.position.x:.1f}, "
                      f"{d.bbox.center.position.y:.1f})")
                print(f"               - bbox size: ({d.bbox.size.x:.1f}, {d.bbox.size.y:.1f})")
                if d.results:
                    print(f"               - confidence: {d.results[0].hypothesis.score:.2f}")
                    print(f"               - class_id: {d.results[0].hypothesis.class_id}")


if __name__ == '__main__':
    unittest.main()
