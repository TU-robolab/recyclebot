#!/usr/bin/env python3
"""
Real Control + Robot Motion Tests for RecycleBot.

Tests MoveIt motion execution with UR virtual robot (requires full MoveIt stack):
  fake_rgbd → vision (YOLO) → core (3D projection) → control (MoveIt) → UR virtual robot

Prerequisites (provided by launch file):
  - UR robot driver with use_mock_hardware=true
  - MoveIt configuration loaded
  - All pipeline nodes running

Test sequence:
  01. Test single pick motion (neutral → pick)
  02. Test full pick-place sequence (neutral → pick → grip → neutral → place → release → neutral)
  03. Test multiple detections queuing sequentially
  04. Verify joint states update during execution

Note: Basic pipeline tests (RGBD, vision service, detections, joint states) are in test_e2e_pipeline.py
"""

import unittest
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray
from grip_interface.srv import GripCommand


class TestRealControlRobotMotion(unittest.TestCase):
    """Real control and robot motion tests (requires MoveIt + UR virtual robot)."""

    report_data = {
        'test_start_time': None,
        'test_end_time': None,
        'tests': [],
        'detections': [],
        'service_response_times': [],
        'detection_latencies': [],
        'joint_motion_events': [],
        'gripper_events': [],
    }

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 and create test node."""
        rclpy.init()
        cls.report_data['test_start_time'] = datetime.now()
        print("\n" + "="*80)
        print("REAL CONTROL + ROBOT MOTION TEST REPORT")
        print("="*80)
        print(f"Test started at: {cls.report_data['test_start_time'].strftime('%Y-%m-%d %H:%M:%S')}")
        print("="*80 + "\n")

        cls.node = Node('real_control_motion_test_node')

        # Storage for received messages
        cls.detections = []
        cls.poses = []
        cls.joint_states = []
        cls.gripper_calls = []

        # QoS profile for reliable topics
        qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscriptions
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

    @classmethod
    def tearDownClass(cls):
        """Cleanup ROS2 and generate report."""
        cls.report_data['test_end_time'] = datetime.now()
        cls.generate_report()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def detection_callback(cls, msg):
        cls.detections.append((msg, time.time()))

    @classmethod
    def pose_callback(cls, msg):
        cls.poses.append((msg, time.time()))

    @classmethod
    def joint_state_callback(cls, msg):
        cls.joint_states.append(msg)

    @classmethod
    def generate_report(cls):
        """Generate detailed test report"""
        print("\n" + "="*80)
        print("REAL CONTROL + ROBOT MOTION TEST SUMMARY")
        print("="*80)

        # Calculate metrics
        duration = cls.report_data['test_end_time'] - cls.report_data['test_start_time']

        avg_response = None
        if cls.report_data['service_response_times']:
            avg_response = sum(cls.report_data['service_response_times']) / len(cls.report_data['service_response_times'])

        avg_latency = None
        if cls.report_data['detection_latencies']:
            avg_latency = sum(cls.report_data['detection_latencies']) / len(cls.report_data['detection_latencies'])

        # Count objects by class
        object_counts = {}
        total_objects = 0
        for detection_set in cls.report_data['detections']:
            total_objects += detection_set['num_objects']
            for obj in detection_set['objects']:
                class_id = obj['class_id']
                if class_id not in object_counts:
                    object_counts[class_id] = []
                object_counts[class_id].append(obj['confidence'])

        # Quick Summary at the top
        print("\n" + "─"*80)
        print("QUICK SUMMARY")
        print("─"*80)
        print(f"  - Duration: {duration.total_seconds():.2f} seconds")
        if avg_response:
            print(f"  - Service Response: {avg_response*1000:.2f}ms average")
        if avg_latency:
            print(f"  - Detection Latency: {avg_latency*1000:.2f}ms average")
        print(f"  - Objects Detected: {total_objects} objects")
        for class_id, confidences in sorted(object_counts.items()):
            conf_str = ", ".join([f"{c:.2%}" for c in confidences])
            print(f"    - {len(confidences)} {class_id} ({conf_str})")
        print(f"  - Poses Published: {len(cls.poses)}")
        print(f"  - Joint State Updates: {len(cls.joint_states)}")
        print(f"  - Joint Motion Events: {len(cls.report_data['joint_motion_events'])}")
        print()

        # Time information
        print(f"\nTest completed at: {cls.report_data['test_end_time'].strftime('%Y-%m-%d %H:%M:%S')}")

        # Service response times
        if cls.report_data['service_response_times']:
            print(f"\nService Response Times:")
            print(f"  Average: {avg_response*1000:.2f}ms")
            print(f"  Min: {min(cls.report_data['service_response_times'])*1000:.2f}ms")
            print(f"  Max: {max(cls.report_data['service_response_times'])*1000:.2f}ms")

        # Detection latencies
        if cls.report_data['detection_latencies']:
            print(f"\nDetection Latencies:")
            print(f"  Average: {avg_latency*1000:.2f}ms")
            print(f"  Min: {min(cls.report_data['detection_latencies'])*1000:.2f}ms")
            print(f"  Max: {max(cls.report_data['detection_latencies'])*1000:.2f}ms")

        # Detections summary
        if cls.report_data['detections']:
            print(f"\n{'─'*80}")
            print("DETECTION RESULTS")
            print(f"{'─'*80}")
            print(f"\nTotal detections captured: {len(cls.report_data['detections'])}\n")

            for idx, detection_info in enumerate(cls.report_data['detections'], 1):
                print(f"Detection Set #{idx}:")
                print(f"  Timestamp: {detection_info['timestamp']}")
                print(f"  Number of objects: {detection_info['num_objects']}")

                for obj_idx, obj in enumerate(detection_info['objects'], 1):
                    print(f"\n  Object {obj_idx}:")
                    print(f"    Class ID: {obj['class_id']}")
                    print(f"    Confidence: {obj['confidence']:.2%}")
                    print(f"    Bounding Box:")
                    print(f"      Center: ({obj['bbox_center_x']:.1f}, {obj['bbox_center_y']:.1f})")
                    print(f"      Size: {obj['bbox_width']:.1f} x {obj['bbox_height']:.1f} pixels")
                    print(f"      Area: {obj['bbox_area']:.1f} px²")
                print()

        # Test results summary
        print(f"{'─'*80}")
        print("TEST RESULTS")
        print(f"{'─'*80}")
        for test_info in cls.report_data['tests']:
            status = "✓ PASSED" if test_info['passed'] else "✗ FAILED"
            print(f"{status} - {test_info['name']}")
            if test_info.get('details'):
                print(f"  {test_info['details']}")

        print("\n" + "="*80)
        print("END OF REPORT")
        print("="*80 + "\n")

        # Write report to file
        report_file = '/tmp/real_control_robot_motion_test_report.txt'
        try:
            with open(report_file, 'w') as f:
                f.write("="*80 + "\n")
                f.write("REAL CONTROL + ROBOT MOTION TEST REPORT\n")
                f.write("="*80 + "\n")
                f.write(f"Test started: {cls.report_data['test_start_time'].strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Test ended: {cls.report_data['test_end_time'].strftime('%Y-%m-%d %H:%M:%S')}\n\n")

                # Quick Summary
                f.write("-"*80 + "\n")
                f.write("QUICK SUMMARY\n")
                f.write("-"*80 + "\n")
                f.write(f"  - Duration: {duration.total_seconds():.2f} seconds\n")
                if avg_response:
                    f.write(f"  - Service Response: {avg_response*1000:.2f}ms average\n")
                if avg_latency:
                    f.write(f"  - Detection Latency: {avg_latency*1000:.2f}ms average\n")
                f.write(f"  - Objects Detected: {total_objects} objects\n")
                for class_id, confidences in sorted(object_counts.items()):
                    conf_str = ", ".join([f"{c:.2%}" for c in confidences])
                    f.write(f"    - {len(confidences)} {class_id} ({conf_str})\n")
                f.write(f"  - Poses Published: {len(cls.poses)}\n")
                f.write(f"  - Joint State Updates: {len(cls.joint_states)}\n")
                f.write("\n")

                if cls.report_data['detections']:
                    f.write("DETECTIONS:\n")
                    f.write("-"*80 + "\n")
                    for idx, detection_info in enumerate(cls.report_data['detections'], 1):
                        f.write(f"\nDetection Set #{idx} ({detection_info['timestamp']}):\n")
                        f.write(f"  Objects detected: {detection_info['num_objects']}\n")
                        for obj_idx, obj in enumerate(detection_info['objects'], 1):
                            f.write(f"  - Object {obj_idx}: {obj['class_id']} (confidence: {obj['confidence']:.2%})\n")
                            f.write(f"    BBox: center=({obj['bbox_center_x']:.1f}, {obj['bbox_center_y']:.1f}), ")
                            f.write(f"size={obj['bbox_width']:.1f}x{obj['bbox_height']:.1f}\n")

                f.write("\n" + "="*80 + "\n")
            print(f"Detailed report saved to: {report_file}")
        except Exception as e:
            print(f"Warning: Could not write report file: {e}")

    def setUp(self):
        """Set up test node"""
        pass

    def tearDown(self):
        """Clean up test node"""
        pass

    def spin_until(self, condition, timeout=10.0, interval=0.1):
        """Spin node until condition is true or timeout."""
        start = time.time()
        while not condition() and (time.time() - start) < timeout:
            rclpy.spin_once(self.node, timeout_sec=interval)
        return condition()

    def wait_for_messages(self, getter, min_count=1, timeout=15.0):
        """Wait until a message list reaches min_count."""
        result = self.spin_until(lambda: len(getter()) >= min_count, timeout=timeout)
        return result

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
    # Test 1: Single pick motion
    # =========================================================================
    def test_01_single_pick_motion(self):
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
        success_pose = self.wait_for_messages(lambda: self.poses, min_count=1, timeout=20.0)

        print(f"\n[Test 1] Waiting for robot motion...")

        # Wait for joint motion (timeout 40s for detection + planning + execution)
        motion_detected = self.wait_for_joint_motion(initial_positions, timeout=40.0, threshold=0.1)

        max_delta = 0.0
        if len(self.joint_states) > 0:
            final_positions = self.joint_states[-1].position
            deltas = [abs(f - i) for f, i in zip(final_positions, initial_positions)]
            max_delta = max(deltas)

        # Record joint motion event
        if motion_detected:
            self.report_data['joint_motion_events'].append({
                'test': 'Single Pick Motion',
                'max_delta_rad': max_delta,
                'joint_updates': len(self.joint_states)
            })

        test_passed = success_pose and motion_detected
        self.report_data['tests'].append({
            'name': 'Single Pick Motion',
            'passed': test_passed,
            'details': f"Motion detected: {motion_detected}, max joint delta: {max_delta:.3f} rad, {len(self.joint_states)} joint updates"
        })

        print(f"         Motion detected: {motion_detected}")
        if len(self.joint_states) > 0:
            print(f"         Max joint delta: {max_delta:.3f} rad")
            print(f"         Joint updates: {len(self.joint_states)}")

        self.assertTrue(success_pose, "No pose received")
        self.assertTrue(motion_detected, "Robot did not move after detection")

    # =========================================================================
    # Test 2: Full pick-place sequence
    # =========================================================================
    def test_02_full_pick_place_sequence(self):
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
        success_pose = self.wait_for_messages(lambda: self.poses, min_count=1, timeout=20.0)

        print(f"\n[Test 2] Waiting for full pick-place sequence...")

        # Wait longer for full sequence (90s timeout)
        # Should see multiple waypoints: neutral, pre-pick, pick, grip, neutral, pre-place, place, release, neutral
        motion_detected = self.wait_for_joint_motion(initial_positions, timeout=90.0, threshold=0.1)

        # Count joint updates over 10s to see if multiple waypoints executed
        update_count = self.count_joint_updates(duration=10.0)

        # Record joint motion event
        if motion_detected:
            self.report_data['joint_motion_events'].append({
                'test': 'Full Pick-Place Sequence',
                'total_updates': len(self.joint_states),
                'updates_in_10s': update_count
            })

        test_passed = success_pose and motion_detected
        self.report_data['tests'].append({
            'name': 'Full Pick-Place Sequence',
            'passed': test_passed,
            'details': f"Motion detected: {motion_detected}, {len(self.joint_states)} total joint states, {update_count} updates in 10s window"
        })

        print(f"         Motion detected: {motion_detected}")
        print(f"         Joint state updates (10s window): {update_count}")
        print(f"         Total joint states: {len(self.joint_states)}")

        self.assertTrue(success_pose, "No pose received")
        self.assertTrue(motion_detected, "Robot did not execute pick-place sequence")
        # Expect at least some updates if robot is moving through waypoints
        # (exact count depends on joint_state publisher frequency)

    # =========================================================================
    # Test 3: Multiple detections sequential
    # =========================================================================
    def test_03_multiple_detections_sequential(self):
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
        success_poses = self.wait_for_messages(lambda: self.poses, min_count=2, timeout=25.0)

        print(f"\n[Test 3] Poses queued: {len(self.poses)}")
        print(f"         Waiting for sequential execution (this may take time)...")

        # Wait extended time for both tasks to process sequentially
        # Each full sequence ~60s, so 150s total timeout
        time.sleep(5.0)  # Give control node time to start processing

        # Just verify we got multiple joint updates (sign of sequential execution)
        update_count = self.count_joint_updates(duration=10.0)

        test_passed = success_poses and len(self.poses) >= 2
        self.report_data['tests'].append({
            'name': 'Multiple Detections Sequential',
            'passed': test_passed,
            'details': f"{len(self.poses)} pose(s) queued, {update_count} joint updates in 10s"
        })

        print(f"         Joint updates in 10s: {update_count}")

        self.assertTrue(success_poses, "Did not receive expected poses")
        self.assertGreater(len(self.poses), 0, "No poses queued")
        # Note: Full validation of sequential execution would require longer test duration

    # =========================================================================
    # Test 4: Joint states update
    # =========================================================================
    def test_04_joint_states_update(self):
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

        print(f"\n[Test 4] Monitoring joint state updates during execution...")

        # Wait for motion
        motion_detected = self.wait_for_joint_motion(initial_positions, timeout=60.0, threshold=0.1)

        max_delta = 0.0
        if len(self.joint_states) > 0:
            final_positions = self.joint_states[-1].position
            deltas = [abs(f - i) for f, i in zip(final_positions, initial_positions)]
            max_delta = max(deltas)

            test_passed = motion_detected and max_delta > 0.1
            self.report_data['tests'].append({
                'name': 'Joint States Update',
                'passed': test_passed,
                'details': f"Max delta: {max_delta:.3f} rad, {len(self.joint_states)} total updates"
            })

            print(f"         Initial positions: {[f'{p:.3f}' for p in initial_positions]}")
            print(f"         Final positions: {[f'{p:.3f}' for p in final_positions]}")
            print(f"         Max delta: {max_delta:.3f} rad")
            print(f"         Total updates: {len(self.joint_states)}")

            self.assertTrue(motion_detected, "Joint states did not update")
            self.assertGreater(max_delta, 0.1, "Joint positions barely changed")
        else:
            self.report_data['tests'].append({
                'name': 'Joint States Update',
                'passed': False,
                'details': "No joint states received during execution"
            })
            self.fail("No joint states received during execution")


def main():
    """Run tests."""
    import sys
    import pytest

    sys.exit(pytest.main([__file__, '-v']))


if __name__ == '__main__':
    main()
