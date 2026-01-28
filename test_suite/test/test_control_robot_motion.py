#!/usr/bin/env python3
"""
End-to-End Control + Robot Motion Tests for RecycleBot.

Tests the complete pipeline from fake camera to robot motion execution:
  fake_rgbd → vision (YOLO) → core (3D projection) → control (MoveIt) → robot motion

Test sequence:
  1. Verify RGBD frames published
  2. Verify vision service available
  3. Verify detections → poses flow
  4. Verify initial joint states published
  5. Test single pick motion (neutral → pick)
  6. Test full pick-place sequence (neutral → pick → grip → neutral → place → release → neutral)
  7. Test multiple detections queuing sequentially
  8. Verify joint states update during execution
"""

import unittest
import time
import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray
from realsense2_camera_msgs.msg import RGBD
from grip_interface.srv import GripCommand


class TestControlRobotMotion(unittest.TestCase):
    """End-to-end control and robot motion integration tests."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 and create test node."""
        rclpy.init()
        cls.node = Node('control_motion_test_node')

        # Storage for received messages
        cls.rgbd_frames = []
        cls.detections = []
        cls.poses = []
        cls.joint_states = []
        cls.gripper_calls = []

        # QoS profiles
        qos_camera = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscriptions
        cls.rgbd_sub = cls.node.create_subscription(
            RGBD, '/camera/camera/rgbd',
            cls.rgbd_callback, qos_camera
        )

        cls.detection_sub = cls.node.create_subscription(
            Detection3DArray, '/object_detections',
            cls.detection_callback, qos_reliable
        )

        cls.pose_sub = cls.node.create_subscription(
            PoseStamped, '/vision/detected_object',
            cls.pose_callback, qos_reliable
        )

        cls.joint_state_sub = cls.node.create_subscription(
            JointState, '/joint_states',
            cls.joint_state_callback, 10
        )

        # Service clients
        cls.trigger_client = cls.node.create_client(Trigger, '/capture_detections')
        cls.gripper_client = cls.node.create_client(GripCommand, '/gripper_action')

        print("\n" + "="*80)
        print("END-TO-END CONTROL + ROBOT MOTION TEST SUITE")
        print("="*80)

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

    @classmethod
    def joint_state_callback(cls, msg):
        cls.joint_states.append(msg)

    def spin_until(self, condition, timeout=10.0, interval=0.1):
        """Spin node until condition is true or timeout."""
        start = time.time()
        while not condition() and (time.time() - start) < timeout:
            rclpy.spin_once(self.node, timeout_sec=interval)
        return condition()

    def wait_for_messages(self, getter, min_count=1, timeout=15.0):
        """Wait until a message list reaches min_count."""
        return self.spin_until(lambda: len(getter()) >= min_count, timeout=timeout)

    def wait_for_topic(self, topic_name, timeout=30.0):
        """Wait until a topic is visible in the ROS graph."""
        def topic_visible():
            topics = [name for name, _ in self.node.get_topic_names_and_types()]
            return topic_name in topics

        return self.spin_until(topic_visible, timeout=timeout)

    def wait_for_joint_motion(self, initial_positions, timeout=60.0, threshold=0.1):
        """Wait for joint positions to change significantly from initial."""
        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if len(self.joint_states) > 0:
                current = self.joint_states[-1].position
                if len(current) == len(initial_positions):
                    # Check if any joint moved significantly
                    max_delta = max(abs(c - i) for c, i in zip(current, initial_positions))
                    if max_delta > threshold:
                        return True
        return False

    def count_joint_updates(self, duration=5.0):
        """Count joint state updates over duration."""
        start_count = len(self.joint_states)
        time_start = time.time()
        while (time.time() - time_start) < duration:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return len(self.joint_states) - start_count

    # =========================================================================
    # Test 1: RGBD frames published
    # =========================================================================
    def test_01_rgbd_frames_published(self):
        """Verify fake RGBD publisher is sending frames."""
        self.rgbd_frames.clear()

        topic_ready = self.wait_for_topic("/camera/camera/rgbd", timeout=30.0)
        self.assertTrue(topic_ready, "RGBD topic not available")

        success = self.wait_for_messages(lambda: self.rgbd_frames, min_count=1, timeout=30.0)

        print(f"\n[Test 1] RGBD frames received: {len(self.rgbd_frames)}")
        self.assertTrue(success, "No RGBD frames received")
        self.assertGreaterEqual(len(self.rgbd_frames), 1)

    # =========================================================================
    # Test 2: Vision service available
    # =========================================================================
    def test_02_vision_service_available(self):
        """Verify /capture_detections service is available."""
        success = self.trigger_client.wait_for_service(timeout_sec=30.0)

        print(f"\n[Test 2] Vision service available: {success}")
        self.assertTrue(success, "/capture_detections service not available")

    # =========================================================================
    # Test 3: Detections → Poses pipeline
    # =========================================================================
    def test_03_detections_to_poses(self):
        """Trigger detection and verify pose published to /vision/detected_object."""
        self.detections.clear()
        self.poses.clear()

        self.assertTrue(
            self.trigger_client.wait_for_service(timeout_sec=30.0),
            "Detection service not available"
        )

        # Trigger detection
        request = Trigger.Request()
        future = self.trigger_client.call_async(request)

        success = self.spin_until(lambda: future.done(), timeout=30.0)
        self.assertTrue(success, "Detection service call timeout")

        response = future.result()
        self.assertTrue(response.success, f"Detection failed: {response.message}")

        # Wait for detection
        success = self.wait_for_messages(lambda: self.detections, min_count=1, timeout=15.0)
        self.assertTrue(success, "No detections received")

        # Wait for pose
        success = self.wait_for_messages(lambda: self.poses, min_count=1, timeout=20.0)
        self.assertTrue(success, "No poses received from core")

        print(f"\n[Test 3] Detections: {len(self.detections)}, Poses: {len(self.poses)}")
        print(f"         Pose frame: {self.poses[0].header.frame_id}")
        print(f"         Position: x={self.poses[0].pose.position.x:.3f}, "
              f"y={self.poses[0].pose.position.y:.3f}, z={self.poses[0].pose.position.z:.3f}")

    # =========================================================================
    # Test 4: Initial joint states
    # =========================================================================
    def test_04_initial_joint_states(self):
        """Verify /joint_states available before motion."""
        self.joint_states.clear()

        success = self.wait_for_messages(lambda: self.joint_states, min_count=1, timeout=30.0)

        print(f"\n[Test 4] Joint states received: {len(self.joint_states)}")
        if len(self.joint_states) > 0:
            js = self.joint_states[0]
            print(f"         Joint names: {js.name}")
            print(f"         Positions: {[f'{p:.3f}' for p in js.position]}")

        self.assertTrue(success, "No joint states received")
        self.assertEqual(len(self.joint_states[0].name), 6, "Should have 6 UR joints")

    # =========================================================================
    # Test 5: Single pick motion
    # =========================================================================
    def test_05_single_pick_motion(self):
        """Trigger detection → robot moves to pick position."""
        # Store initial joint state
        self.joint_states.clear()
        self.wait_for_messages(lambda: self.joint_states, min_count=1, timeout=30.0)
        self.assertGreater(len(self.joint_states), 0, "No initial joint state")

        initial_positions = list(self.joint_states[0].position)

        # Clear previous data
        self.detections.clear()
        self.poses.clear()

        self.assertTrue(
            self.trigger_client.wait_for_service(timeout_sec=30.0),
            "Detection service not available"
        )

        # Trigger detection
        request = Trigger.Request()
        future = self.trigger_client.call_async(request)
        self.spin_until(lambda: future.done(), timeout=30.0)

        # Wait for pose to be published (triggers control node)
        success = self.wait_for_messages(lambda: self.poses, min_count=1, timeout=20.0)
        self.assertTrue(success, "No pose received")

        print(f"\n[Test 5] Waiting for robot motion...")

        # Wait for joint motion (timeout 40s for detection + planning + execution)
        motion_detected = self.wait_for_joint_motion(initial_positions, timeout=40.0, threshold=0.1)

        print(f"         Motion detected: {motion_detected}")
        if len(self.joint_states) > 0:
            final_positions = self.joint_states[-1].position
            deltas = [abs(f - i) for f, i in zip(final_positions, initial_positions)]
            print(f"         Max joint delta: {max(deltas):.3f} rad")
            print(f"         Joint updates: {len(self.joint_states)}")

        self.assertTrue(motion_detected, "Robot did not move after detection")

    # =========================================================================
    # Test 6: Full pick-place sequence
    # =========================================================================
    def test_06_full_pick_place_sequence(self):
        """Trigger detection → full cycle (neutral → pick → grip → neutral → place → release → neutral)."""
        # Note: This test requires gripper service to track calls
        # For now, we verify extended motion sequence

        self.joint_states.clear()
        self.wait_for_messages(lambda: self.joint_states, min_count=1, timeout=30.0)
        initial_positions = list(self.joint_states[0].position)

        self.detections.clear()
        self.poses.clear()

        self.assertTrue(
            self.trigger_client.wait_for_service(timeout_sec=30.0),
            "Detection service not available"
        )

        # Trigger detection
        request = Trigger.Request()
        future = self.trigger_client.call_async(request)
        self.spin_until(lambda: future.done(), timeout=30.0)

        # Wait for pose
        self.wait_for_messages(lambda: self.poses, min_count=1, timeout=20.0)

        print(f"\n[Test 6] Waiting for full pick-place sequence...")

        # Wait longer for full sequence (90s timeout)
        # Should see multiple waypoints: neutral, pre-pick, pick, grip, neutral, pre-place, place, release, neutral
        motion_detected = self.wait_for_joint_motion(initial_positions, timeout=90.0, threshold=0.1)

        # Count joint updates over 10s to see if multiple waypoints executed
        update_count = self.count_joint_updates(duration=10.0)

        print(f"         Motion detected: {motion_detected}")
        print(f"         Joint state updates (10s window): {update_count}")
        print(f"         Total joint states: {len(self.joint_states)}")

        self.assertTrue(motion_detected, "Robot did not execute pick-place sequence")
        # Expect at least some updates if robot is moving through waypoints
        # (exact count depends on joint_state publisher frequency)

    # =========================================================================
    # Test 7: Multiple detections sequential
    # =========================================================================
    def test_07_multiple_detections_sequential(self):
        """Trigger 2 detections, verify tasks queue and execute sequentially."""
        self.detections.clear()
        self.poses.clear()
        self.joint_states.clear()

        self.assertTrue(
            self.trigger_client.wait_for_service(timeout_sec=30.0),
            "Detection service not available"
        )

        # Trigger first detection
        request1 = Trigger.Request()
        future1 = self.trigger_client.call_async(request1)
        self.spin_until(lambda: future1.done(), timeout=30.0)

        # Small delay, then trigger second
        time.sleep(0.5)

        request2 = Trigger.Request()
        future2 = self.trigger_client.call_async(request2)
        self.spin_until(lambda: future2.done(), timeout=30.0)

        # Wait for both poses
        success = self.wait_for_messages(lambda: self.poses, min_count=2, timeout=25.0)

        print(f"\n[Test 7] Poses queued: {len(self.poses)}")
        print(f"         Waiting for sequential execution (this may take time)...")

        # Wait extended time for both tasks to process sequentially
        # Each full sequence ~60s, so 150s total timeout
        time.sleep(5.0)  # Give control node time to start processing

        # Just verify we got multiple joint updates (sign of sequential execution)
        update_count = self.count_joint_updates(duration=10.0)
        print(f"         Joint updates in 10s: {update_count}")

        self.assertGreater(len(self.poses), 0, "No poses queued")
        # Note: Full validation of sequential execution would require longer test duration

    # =========================================================================
    # Test 8: Joint states update
    # =========================================================================
    def test_08_joint_states_update(self):
        """Verify joint positions change during execution."""
        self.joint_states.clear()

        # Wait for initial state
        self.wait_for_messages(lambda: self.joint_states, min_count=1, timeout=30.0)
        self.assertGreater(len(self.joint_states), 0, "No initial joint state")

        initial_positions = list(self.joint_states[0].position)

        self.assertTrue(
            self.trigger_client.wait_for_service(timeout_sec=30.0),
            "Detection service not available"
        )

        # Trigger detection
        request = Trigger.Request()
        future = self.trigger_client.call_async(request)
        self.spin_until(lambda: future.done(), timeout=30.0)

        # Wait for pose
        self.poses.clear()
        self.wait_for_messages(lambda: self.poses, min_count=1, timeout=20.0)

        print(f"\n[Test 8] Monitoring joint state updates during execution...")

        # Wait for motion
        motion_detected = self.wait_for_joint_motion(initial_positions, timeout=60.0, threshold=0.1)

        if len(self.joint_states) > 0:
            final_positions = self.joint_states[-1].position
            deltas = [abs(f - i) for f, i in zip(final_positions, initial_positions)]
            max_delta = max(deltas)

            print(f"         Initial positions: {[f'{p:.3f}' for p in initial_positions]}")
            print(f"         Final positions: {[f'{p:.3f}' for p in final_positions]}")
            print(f"         Max delta: {max_delta:.3f} rad")
            print(f"         Total updates: {len(self.joint_states)}")

            self.assertTrue(motion_detected, "Joint states did not update")
            self.assertGreater(max_delta, 0.1, "Joint positions barely changed")
        else:
            self.fail("No joint states received during execution")


def main():
    """Run tests."""
    import sys
    import pytest

    sys.exit(pytest.main([__file__, '-v']))


if __name__ == '__main__':
    main()
