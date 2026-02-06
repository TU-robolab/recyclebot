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
import copy
from datetime import datetime, timezone
import numpy as np
import cv2
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time

from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray
from realsense2_camera_msgs.msg import RGBD
from grip_interface.srv import GripCommand
from cv_bridge import CvBridge
import tf2_ros


class TestRealControlRobotMotion(unittest.TestCase):
    """Real control and robot motion tests (requires MoveIt + UR virtual robot)."""

    report_data = {
        'test_start_time': None,
        'test_end_time': None,
        'tests': [],
        'detections': [],
        'service_response_times': [],
        'detection_latencies': [],
        'rgbd_frames': [],
        'depth_stats': {},
        'frame_visualizations': [],
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

        cls.node = Node('control_motion_test_node')
        cls.bridge = CvBridge()
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.node)

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

    @classmethod
    def tearDownClass(cls):
        """Cleanup ROS2 and generate report."""
        cls.report_data['test_end_time'] = datetime.now()

        # Save frame visualizations before generating report
        print("\n" + "="*80)
        print("SAVING FRAME VISUALIZATIONS")
        print("="*80)
        cls.save_frame_visualizations()

        cls.generate_report()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def rgbd_callback(cls, msg):
        cls.rgbd_frames.append((msg, time.time()))

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
    def save_frame_visualizations(cls, frames=1):
        """
        Save RGB and depth frame visualizations to files

        Args:
            frames: Can be:
                - int: Number of frames to save from the start (default: 1)
                - -1: Save all frames
                - list: List of specific frame indices to save, e.g., [0, 2, 5]
                - str: Range string like "1:3" to save frames 1 through 3 (inclusive)
        """
        if not cls.report_data['rgbd_frames']:
            return

        try:
            bridge = CvBridge()
            output_dir = '/tmp'
            total_frames = len(cls.report_data['rgbd_frames'])

            # Determine which frame indices to save
            if isinstance(frames, str):
                # Handle range string like "1:3"
                if ':' in frames:
                    start, end = frames.split(':')
                    start = int(start) if start else 0
                    end = int(end) if end else total_frames
                    frame_indices = list(range(start, min(end + 1, total_frames)))
                else:
                    frame_indices = [int(frames)]
            elif isinstance(frames, list):
                # Use provided list of indices
                frame_indices = [idx for idx in frames if 0 <= idx < total_frames]
            elif frames == -1:
                # Save all frames
                frame_indices = list(range(total_frames))
            else:
                # Save first N frames
                frame_indices = list(range(min(frames, total_frames)))

            print(f"Saving {len(frame_indices)} frame(s) out of {total_frames} available: {frame_indices}")

            for idx in frame_indices:
                rgbd_msg = cls.report_data['rgbd_frames'][idx]

                # Convert RGB image
                rgb_image = bridge.imgmsg_to_cv2(rgbd_msg.rgb, desired_encoding="bgr8")

                # Convert depth image
                depth_image = bridge.imgmsg_to_cv2(rgbd_msg.depth, desired_encoding="passthrough")

                # Save RGB image
                rgb_path = f"{output_dir}/control_motion_frame_{idx}_rgb.png"
                cv2.imwrite(rgb_path, rgb_image)

                # Create colorized depth visualization
                # Normalize depth to 0-255 for visualization (ignoring invalid pixels)
                # INVERT so close=255 (red/warm) and far=0 (blue/cool)
                valid_mask = depth_image > 0
                depth_normalized = np.zeros_like(depth_image, dtype=np.uint8)

                if np.any(valid_mask):
                    valid_depth = depth_image[valid_mask]
                    depth_min, depth_max = np.min(valid_depth), np.max(valid_depth)

                    # Normalize and INVERT: close objects → 255 (red), far objects → 0 (blue)
                    depth_normalized[valid_mask] = (
                        255 - ((depth_image[valid_mask] - depth_min) / (depth_max - depth_min) * 255)
                    ).astype(np.uint8)

                # Apply colormap (TURBO: red=255=close, blue=0=far)
                depth_colorized = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_TURBO)

                # Mark invalid pixels as black
                depth_colorized[~valid_mask] = [0, 0, 0]

                # Add depth scale legend to the depth image
                # Create a vertical color bar on the right side
                legend_width = 80
                legend_height = 300
                legend_x = depth_colorized.shape[1] - legend_width - 20
                legend_y = 50

                # Create gradient bar (inverted: top=0=far=blue, bottom=255=close=red)
                gradient = np.linspace(0, 255, legend_height, dtype=np.uint8)
                gradient_3d = np.tile(gradient[:, np.newaxis], (1, 30))
                gradient_colored = cv2.applyColorMap(gradient_3d, cv2.COLORMAP_TURBO)

                # Draw white background for legend
                cv2.rectangle(depth_colorized,
                             (legend_x - 10, legend_y - 10),
                             (legend_x + 70, legend_y + legend_height + 40),
                             (255, 255, 255), -1)
                cv2.rectangle(depth_colorized,
                             (legend_x - 10, legend_y - 10),
                             (legend_x + 70, legend_y + legend_height + 40),
                             (0, 0, 0), 2)

                # Place gradient on depth image
                depth_colorized[legend_y:legend_y+legend_height, legend_x:legend_x+30] = gradient_colored

                # Add depth labels
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.4
                thickness = 1

                # Max depth at top (far objects = blue/cool = 0 in gradient after inversion)
                cv2.putText(depth_colorized, f'{depth_max}mm',
                           (legend_x + 35, legend_y + 5),
                           font, font_scale, (0, 0, 0), thickness)
                cv2.putText(depth_colorized, '(far)',
                           (legend_x + 35, legend_y + 20),
                           font, font_scale - 0.1, (0, 0, 0), thickness)

                # Min depth at bottom (close objects = red/warm = 255 in gradient after inversion)
                cv2.putText(depth_colorized, f'{depth_min}mm',
                           (legend_x + 35, legend_y + legend_height - 5),
                           font, font_scale, (0, 0, 0), thickness)
                cv2.putText(depth_colorized, '(close)',
                           (legend_x + 35, legend_y + legend_height + 10),
                           font, font_scale - 0.1, (0, 0, 0), thickness)

                # Add title
                cv2.putText(depth_colorized, 'Depth',
                           (legend_x - 5, legend_y - 20),
                           font, font_scale + 0.1, (0, 0, 0), thickness + 1)

                # Save colorized depth
                depth_path = f"{output_dir}/control_motion_frame_{idx}_depth.png"
                cv2.imwrite(depth_path, depth_colorized)

                # Create side-by-side comparison
                w = rgb_image.shape[1]  # Get width for label positioning
                combined = np.hstack([rgb_image, depth_colorized])

                # Add labels
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(combined, 'RGB', (10, 30), font, 1, (255, 255, 255), 2)
                cv2.putText(combined, 'Depth (colorized)', (w + 10, 30), font, 1, (255, 255, 255), 2)

                combined_path = f"{output_dir}/control_motion_frame_{idx}_combined.png"
                cv2.imwrite(combined_path, combined)

                cls.report_data['frame_visualizations'].append({
                    'rgb': rgb_path,
                    'depth': depth_path,
                    'combined': combined_path
                })

                print(f"\nFrame {idx} visualizations saved:")
                print(f"  RGB: {rgb_path}")
                print(f"  Depth: {depth_path}")
                print(f"  Combined: {combined_path}")

        except Exception as e:
            print(f"Warning: Could not save frame visualizations: {e}")

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
        print(f"  - RGBD Frames: {len(cls.rgbd_frames)}")
        print(f"  - Poses Published: {len(cls.poses)}")
        print(f"  - Joint State Updates: {len(cls.joint_states)}")
        print(f"  - Joint Motion Events: {len(cls.report_data['joint_motion_events'])}")
        print()

        # Depth Statistics
        if cls.report_data['depth_stats']:
            stats = cls.report_data['depth_stats']
            print(f"\nDEPTH CHANNEL STATISTICS:")
            print(f"  Depth Range: {stats['min_mm']}mm - {stats['max_mm']}mm")
            print(f"  Mean Depth: {stats['mean_mm']:.1f}mm")
            print(f"  Std Deviation: {stats['std_mm']:.1f}mm")
            print(f"  Valid Pixels: {stats['valid_pixels']:,} ({100-stats['invalid_percentage']:.1f}%)")
            print(f"  Invalid Pixels: {stats['invalid_pixels']:,} ({stats['invalid_percentage']:.1f}%)")

        # Frame Visualizations
        if cls.report_data['frame_visualizations']:
            print(f"\nFRAME VISUALIZATIONS:")
            for idx, viz in enumerate(cls.report_data['frame_visualizations']):
                print(f"  Frame {idx}:")
                print(f"    RGB: {viz['rgb']}")
                print(f"    Depth (colorized): {viz['depth']}")
                print(f"    Combined: {viz['combined']}")

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
        report_file = '/tmp/control_robot_motion_test_report.txt'
        try:
            with open(report_file, 'w') as f:
                f.write("="*80 + "\n")
                f.write("CONTROL + ROBOT MOTION TEST REPORT\n")
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
                f.write(f"  - RGBD Frames: {len(cls.rgbd_frames)}\n")
                f.write(f"  - Poses Published: {len(cls.poses)}\n")
                f.write(f"  - Joint State Updates: {len(cls.joint_states)}\n")
                f.write("\n")

                # Depth statistics
                if cls.report_data['depth_stats']:
                    stats = cls.report_data['depth_stats']
                    f.write("-"*80 + "\n")
                    f.write("DEPTH CHANNEL STATISTICS\n")
                    f.write("-"*80 + "\n")
                    f.write(f"  Depth Range: {stats['min_mm']}mm - {stats['max_mm']}mm\n")
                    f.write(f"  Mean Depth: {stats['mean_mm']:.1f}mm\n")
                    f.write(f"  Std Deviation: {stats['std_mm']:.1f}mm\n")
                    f.write(f"  Valid Pixels: {stats['valid_pixels']:,} ({100-stats['invalid_percentage']:.1f}%)\n")
                    f.write(f"  Invalid Pixels: {stats['invalid_pixels']:,} ({stats['invalid_percentage']:.1f}%)\n")
                    f.write("\n")

                # Frame visualizations
                if cls.report_data['frame_visualizations']:
                    f.write("-"*80 + "\n")
                    f.write("FRAME VISUALIZATIONS\n")
                    f.write("-"*80 + "\n")
                    for idx, viz in enumerate(cls.report_data['frame_visualizations']):
                        f.write(f"Frame {idx}:\n")
                        f.write(f"  RGB: {viz['rgb']}\n")
                        f.write(f"  Depth (colorized): {viz['depth']}\n")
                        f.write(f"  Combined: {viz['combined']}\n")
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
    # Test 1: RGBD frames published
    # =========================================================================
    def test_01_rgbd_frames_published(self):
        """Verify fake RGBD publisher is sending frames."""
        self.rgbd_frames.clear()

        topic_ready = self.wait_for_topic("/camera/camera/rgbd", timeout=30.0)
        success = self.wait_for_messages(lambda: self.rgbd_frames, min_count=1, timeout=30.0) if topic_ready else False

        test_passed = True
        try:
            self.assertTrue(topic_ready, "RGBD topic not available")
            self.assertTrue(success, "No RGBD frames received")
            self.assertGreaterEqual(len(self.rgbd_frames), 1)
        except AssertionError as e:
            test_passed = False
            self.report_data['tests'].append({
                'name': 'RGBD Frames Published',
                'passed': False,
                'details': f"Failed: {str(e)}"
            })
            raise e

        self.report_data['tests'].append({
            'name': 'RGBD Frames Published',
            'passed': test_passed,
            'details': f"Received {len(self.rgbd_frames)} RGBD frame(s)"
        })

        print(f"\n[Test 1] RGBD frames received: {len(self.rgbd_frames)}")

    # =========================================================================
    # Test 2: Vision service available
    # =========================================================================
    def test_02_vision_service_available(self):
        """Verify /capture_detections service is available."""
        start_time = time.time()
        success = self.trigger_client.wait_for_service(timeout_sec=30.0)
        elapsed = time.time() - start_time

        self.report_data['tests'].append({
            'name': 'Vision Service Available',
            'passed': success,
            'details': f"Service became available in {elapsed:.2f}s" if success else f"Service not available after {elapsed:.2f}s"
        })

        print(f"\n[Test 2] Vision service available: {success} (took {elapsed:.2f}s)")
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
        service_start = time.time()
        future = self.trigger_client.call_async(request)

        success = self.spin_until(lambda: future.done(), timeout=30.0)
        self.assertTrue(success, "Detection service call timeout")

        response = future.result()
        service_elapsed = time.time() - service_start
        self.report_data['service_response_times'].append(service_elapsed)

        # Wait for detection
        success_detection = self.wait_for_messages(lambda: self.detections, min_count=1, timeout=15.0)

        # Wait for pose
        success_pose = self.wait_for_messages(lambda: self.poses, min_count=1, timeout=20.0)

        # Capture detection and pose metadata
        detection_info = None
        if len(self.detections) > 0:
            detection_array, detection_time = self.detections[0]
            detection_info = {
                'timestamp': datetime.fromtimestamp(detection_time).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                'num_objects': len(detection_array.detections),
                'objects': []
            }

            for detection in detection_array.detections:
                if len(detection.results) > 0:
                    result = detection.results[0]
                    obj_info = {
                        'class_id': result.hypothesis.class_id,
                        'confidence': result.hypothesis.score,
                        'bbox_center_x': detection.bbox.center.position.x,
                        'bbox_center_y': detection.bbox.center.position.y,
                        'bbox_width': detection.bbox.size.x,
                        'bbox_height': detection.bbox.size.y,
                        'bbox_area': detection.bbox.size.x * detection.bbox.size.y
                    }
                    detection_info['objects'].append(obj_info)

            self.report_data['detections'].append(detection_info)

        pose_info = None
        if len(self.poses) > 0:
            pose_stamped, pose_time = self.poses[0]
            pose_info = {
                'timestamp': datetime.fromtimestamp(pose_time).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                'frame_id': pose_stamped.header.frame_id,
                'position': {
                    'x': pose_stamped.pose.position.x,
                    'y': pose_stamped.pose.position.y,
                    'z': pose_stamped.pose.position.z
                },
                'orientation': {
                    'x': pose_stamped.pose.orientation.x,
                    'y': pose_stamped.pose.orientation.y,
                    'z': pose_stamped.pose.orientation.z,
                    'w': pose_stamped.pose.orientation.w
                }
            }

        # Capture report before assertions
        test_passed = response.success and success_detection and success_pose and len(self.detections) > 0 and len(self.poses) > 0
        details = f"{len(self.detections)} detection(s), {len(self.poses)} pose(s), service response: {service_elapsed*1000:.2f}ms"
        if detection_info and pose_info:
            details += f"\n  Detection: {detection_info['num_objects']} object(s)"
            if detection_info['objects']:
                obj = detection_info['objects'][0]
                details += f" - {obj['class_id']} ({obj['confidence']:.2%})"
            details += f"\n  Pose: frame={pose_info['frame_id']}, pos=({pose_info['position']['x']:.3f}, {pose_info['position']['y']:.3f}, {pose_info['position']['z']:.3f})"

        self.report_data['tests'].append({
            'name': 'Detections to Poses Pipeline',
            'passed': test_passed,
            'details': details
        })

        # Print results
        print(f"\n[Test 3] Detections: {len(self.detections)}, Poses: {len(self.poses)}")
        print(f"         Service response: {service_elapsed*1000:.2f}ms")
        if detection_info:
            print(f"         Detected {detection_info['num_objects']} object(s)")
            for idx, obj in enumerate(detection_info['objects'], 1):
                print(f"           Object {idx}: {obj['class_id']} ({obj['confidence']:.2%})")
        if pose_info:
            print(f"         Pose frame: {pose_info['frame_id']}")
            print(f"         Position: x={pose_info['position']['x']:.3f}, "
                  f"y={pose_info['position']['y']:.3f}, z={pose_info['position']['z']:.3f}")

        # Now do assertions (may raise)
        self.assertTrue(response.success, f"Detection failed: {response.message}")
        self.assertTrue(success_detection, "No detections received")
        self.assertTrue(success_pose, "No poses received from core")

    # =========================================================================
    # Test 4: Initial joint states
    # =========================================================================
    def test_04_initial_joint_states(self):
        """Verify /joint_states available before motion."""
        self.joint_states.clear()

        success = self.wait_for_messages(lambda: self.joint_states, min_count=1, timeout=30.0)
        num_joints = len(self.joint_states[0].name) if len(self.joint_states) > 0 else 0
        test_passed = success and len(self.joint_states) > 0 and num_joints == 6

        self.report_data['tests'].append({
            'name': 'Initial Joint States',
            'passed': test_passed,
            'details': f"Received {len(self.joint_states)} joint state message(s), {num_joints} joints"
        })

        print(f"\n[Test 4] Joint states received: {len(self.joint_states)}")
        if len(self.joint_states) > 0:
            js = self.joint_states[0]
            print(f"         Joint names: {js.name}")
            print(f"         Positions: {[f'{p:.3f}' for p in js.position]}")

        self.assertTrue(success, "No joint states received")
        if len(self.joint_states) > 0:
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
        success_pose = self.wait_for_messages(lambda: self.poses, min_count=1, timeout=20.0)

        print(f"\n[Test 5] Waiting for robot motion...")

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
        success_pose = self.wait_for_messages(lambda: self.poses, min_count=1, timeout=20.0)

        print(f"\n[Test 6] Waiting for full pick-place sequence...")

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
        success_poses = self.wait_for_messages(lambda: self.poses, min_count=2, timeout=25.0)

        print(f"\n[Test 7] Poses queued: {len(self.poses)}")
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
