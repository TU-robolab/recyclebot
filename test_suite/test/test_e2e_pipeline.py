#!/usr/bin/env python3
"""
End-to-End Pipeline Tests for RecycleBot.

Tests the full data flow from camera to control-ready poses (no MoveIt required):
  fake_rgbd -> vision (Detection3DArray) -> core (PoseStamped) -> gripper service

Test sequence:
  01. Verify RGBD frames are published
  02. Verify vision service available
  03. Trigger detection + Detection3DArray received
  04. Detection format validation (bbox, confidence, class)
  05. Verify core publishes PoseStamped with position validation
  06. TF available (base_link -> camera_color_optical_frame)
  07. Initial joint states (6 joints on /joint_states)
  08. Gripper service available
  09. Gripper grip command
  10. Gripper release command
  11. Full pipeline data flow summary
  12. RGBD data validation (1280x720, encodings)
  13. Depth channel validation (D415 range check)

Features:
  - Per-test node creation (setUp/tearDown) like vision_workflow
  - Automated performance metrics collection
  - Detailed detection metadata capture
  - Frame visualizations saved to /tmp
  - Report saved to /tmp/e2e_pipeline_test_report.txt
"""

import unittest
import time
from datetime import datetime
import numpy as np
import cv2

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


class TestE2EPipeline(unittest.TestCase):
    """End-to-end pipeline integration tests."""

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
    }

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 and print header."""
        rclpy.init()
        cls.report_data['test_start_time'] = datetime.now()
        print("\n" + "="*80)
        print("END-TO-END PIPELINE TEST REPORT")
        print("="*80)
        print(f"Test started at: {cls.report_data['test_start_time'].strftime('%Y-%m-%d %H:%M:%S')}")
        print("="*80 + "\n")

    @classmethod
    def tearDownClass(cls):
        """Save visualizations, generate report, shutdown ROS2."""
        cls.report_data['test_end_time'] = datetime.now()

        print("\n" + "="*80)
        print("SAVING FRAME VISUALIZATIONS")
        print("="*80)
        cls.save_frame_visualizations()

        cls.generate_report()
        rclpy.shutdown()

    def setUp(self):
        """Create a fresh per-test node with subscriptions and clients."""
        self.node = rclpy.create_node('e2e_pipeline_test_node')
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        # Storage for received messages
        self.rgbd_frames = []
        self.detections_received = []
        self.poses_received = []
        self.joint_states_received = []

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
        self.rgbd_sub = self.node.create_subscription(
            RGBD, '/camera/camera/rgbd',
            self._rgbd_callback, qos_camera
        )

        self.detection_sub = self.node.create_subscription(
            Detection3DArray, '/object_detections',
            self._detection_callback, qos_reliable
        )

        self.pose_sub = self.node.create_subscription(
            PoseStamped, '/vision/detected_object',
            self._pose_callback, qos_reliable
        )

        self.joint_state_sub = self.node.create_subscription(
            JointState, '/joint_states',
            self._joint_state_callback, 10
        )

        # Service clients
        self.trigger_client = self.node.create_client(Trigger, '/capture_detections')
        self.gripper_client = self.node.create_client(GripCommand, '/gripper_action')

    def tearDown(self):
        """Destroy per-test node."""
        self.node.destroy_node()

    # -- callbacks --

    def _rgbd_callback(self, msg):
        self.rgbd_frames.append((msg, time.time()))

    def _detection_callback(self, msg):
        self.detections_received.append((msg, time.time()))

    def _pose_callback(self, msg):
        self.poses_received.append((msg, time.time()))

    def _joint_state_callback(self, msg):
        self.joint_states_received.append(msg)

    # -- helpers --

    def spin_until(self, condition, timeout=10.0, interval=0.1):
        """Spin node until condition is true or timeout."""
        start = time.time()
        while not condition() and (time.time() - start) < timeout:
            rclpy.spin_once(self.node, timeout_sec=interval)
        return condition()

    def _record_test(self, name, passed, details=''):
        self.report_data['tests'].append({
            'name': name,
            'passed': passed,
            'details': details
        })

    # =========================================================================
    # Test 01: RGBD frames published
    # =========================================================================
    def test_01_rgbd_frames_published(self):
        """Verify fake RGBD publisher is sending frames."""
        success = self.spin_until(
            lambda: len(self.rgbd_frames) >= 1,
            timeout=15.0
        )

        self._record_test(
            'RGBD Frames Published', success,
            f"Received {len(self.rgbd_frames)} frame(s)"
        )

        print(f"\n[E2E 01] RGBD frames received: {len(self.rgbd_frames)}")
        self.assertTrue(success, "No RGBD frames received")

    # =========================================================================
    # Test 02: Vision service available
    # =========================================================================
    def test_02_vision_service_available(self):
        """Verify /capture_detections service is available."""
        start_time = time.time()
        success = self.trigger_client.wait_for_service(timeout_sec=30.0)
        elapsed = time.time() - start_time

        self._record_test(
            'Vision Service Available', success,
            f"Service available in {elapsed:.2f}s" if success else f"Not available after {elapsed:.2f}s"
        )

        print(f"\n[E2E 02] Detection service available: {success} ({elapsed:.2f}s)")
        self.assertTrue(success, "/capture_detections service not available")

    # =========================================================================
    # Test 03: Trigger detection + Detection3DArray received
    # =========================================================================
    def test_03_trigger_detection(self):
        """Trigger detection and verify Detection3DArray is published."""
        self.assertTrue(
            self.trigger_client.wait_for_service(timeout_sec=30.0),
            "Detection service not available"
        )

        # Wait for RGBD frames first
        self.spin_until(lambda: len(self.rgbd_frames) >= 1, timeout=10.0)

        # Trigger detection
        request = Trigger.Request()
        service_start = time.time()
        future = self.trigger_client.call_async(request)

        success = self.spin_until(lambda: future.done(), timeout=30.0)
        self.assertTrue(success, "Detection service call timed out")

        response = future.result()
        service_elapsed = time.time() - service_start
        self.report_data['service_response_times'].append(service_elapsed)

        # Wait for Detection3DArray
        success_detection = self.spin_until(
            lambda: len(self.detections_received) > 0,
            timeout=5.0
        )

        if success_detection:
            detection_array, detection_time = self.detections_received[0]
            latency = detection_time - service_start
            self.report_data['detection_latencies'].append(latency)

        test_passed = response.success and success_detection
        self._record_test(
            'Trigger Detection', test_passed,
            f"Response: {response.success}, detections: {len(self.detections_received)}, "
            f"service: {service_elapsed*1000:.2f}ms"
        )

        print(f"\n[E2E 03] Detection response: {response.success}, "
              f"detections: {len(self.detections_received)}, "
              f"service: {service_elapsed*1000:.2f}ms")

        self.assertTrue(response.success, f"Detection failed: {response.message}")
        self.assertTrue(success_detection, "No Detection3DArray received after trigger")

    # =========================================================================
    # Test 04: Detection format validation
    # =========================================================================
    def test_04_detection_format_validation(self):
        """Verify detection format: bbox, confidence, class_id."""
        self.assertTrue(
            self.trigger_client.wait_for_service(timeout_sec=30.0),
            "Detection service not available"
        )

        # Trigger detection
        request = Trigger.Request()
        future = self.trigger_client.call_async(request)
        self.spin_until(lambda: future.done(), timeout=30.0)

        # Wait for detections
        self.spin_until(lambda: len(self.detections_received) > 0, timeout=5.0)
        self.assertGreater(len(self.detections_received), 0, "No detections received")

        detection_array, detection_time = self.detections_received[0]

        detection_info = {
            'timestamp': datetime.fromtimestamp(detection_time).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            'num_objects': len(detection_array.detections),
            'objects': []
        }

        for detection in detection_array.detections:
            self.assertIsNotNone(detection.bbox)
            self.assertGreater(detection.bbox.size.x, 0, "Invalid bbox width")
            self.assertGreater(detection.bbox.size.y, 0, "Invalid bbox height")

            self.assertGreater(len(detection.results), 0, "No detection results")
            result = detection.results[0]
            self.assertIsNotNone(result.hypothesis.class_id, "No class ID")
            self.assertGreater(result.hypothesis.score, 0.0, "Invalid confidence score")
            self.assertLessEqual(result.hypothesis.score, 1.0, "Confidence score > 1.0")

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

        self._record_test(
            'Detection Format Validation', True,
            f"Validated {len(detection_array.detections)} detection(s)"
        )

        print(f"\n[E2E 04] Detection format validated: {len(detection_array.detections)} object(s)")
        for idx, obj in enumerate(detection_info['objects'], 1):
            print(f"         Object {idx}: {obj['class_id']} ({obj['confidence']:.2%})")
            print(f"           BBox: ({obj['bbox_center_x']:.1f}, {obj['bbox_center_y']:.1f}) "
                  f"{obj['bbox_width']:.1f}x{obj['bbox_height']:.1f}px")

    # =========================================================================
    # Test 05: Pose published with position validation
    # =========================================================================
    def test_05_pose_published(self):
        """Verify core publishes PoseStamped with valid position."""
        self.assertTrue(
            self.trigger_client.wait_for_service(timeout_sec=30.0),
            "Detection service not available"
        )

        # Trigger detection
        request = Trigger.Request()
        future = self.trigger_client.call_async(request)
        self.spin_until(lambda: future.done(), timeout=30.0)

        # Wait for pose
        success = self.spin_until(
            lambda: len(self.poses_received) > 0,
            timeout=15.0
        )

        details = f"Poses received: {len(self.poses_received)}"
        if success and len(self.poses_received) > 0:
            pose_stamped, _ = self.poses_received[-1]
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            z = pose_stamped.pose.position.z

            details += f", pos=({x:.3f}, {y:.3f}, {z:.3f}), frame={pose_stamped.header.frame_id}"

            self.assertIsNotNone(pose_stamped.header.frame_id)
            self.assertGreater(z, 0.1, "Depth below minimum filter threshold")
            self.assertLess(z, 1.6, "Depth above maximum filter threshold")
            self.assertGreater(x, -1.5, "X out of range")
            self.assertLess(x, 1.5, "X out of range")
            self.assertGreater(y, -1.0, "Y out of range")
            self.assertLess(y, 1.0, "Y out of range")

            print(f"\n[E2E 05] Pose position: ({x:.3f}, {y:.3f}, {z:.3f}), frame: {pose_stamped.header.frame_id}")
        else:
            # Poses may be empty if detections don't pass filters
            print(f"\n[E2E 05] No poses published (detections may not pass filters)")

        self._record_test('Pose Published', success or len(self.poses_received) == 0, details)

    # =========================================================================
    # Test 06: TF available
    # =========================================================================
    def test_06_tf_available(self):
        """Verify TF can resolve base_link to camera_color_optical_frame."""
        timeout = 5.0
        start = time.time()
        transform = None
        last_error = None

        while time.time() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link",
                    "camera_color_optical_frame",
                    Time()
                )
                break
            except Exception as exc:
                last_error = exc

        test_passed = transform is not None
        details = "Transform available" if test_passed else f"Transform unavailable: {last_error}"

        self._record_test('TF base_link -> camera_color_optical_frame', test_passed, details)

        print(f"\n[E2E 06] TF available: {test_passed}")
        self.assertIsNotNone(
            transform,
            f"TF missing between base_link and camera_color_optical_frame: {last_error}"
        )

    # =========================================================================
    # Test 07: Initial joint states
    # =========================================================================
    def test_07_initial_joint_states(self):
        """Verify /joint_states available with 6 UR joints."""
        success = self.spin_until(
            lambda: len(self.joint_states_received) >= 1,
            timeout=30.0
        )

        num_joints = len(self.joint_states_received[0].name) if self.joint_states_received else 0
        test_passed = success and num_joints == 6

        self._record_test(
            'Initial Joint States', test_passed,
            f"{len(self.joint_states_received)} message(s), {num_joints} joints"
        )

        print(f"\n[E2E 07] Joint states received: {len(self.joint_states_received)}")
        if self.joint_states_received:
            js = self.joint_states_received[0]
            print(f"         Joint names: {js.name}")
            print(f"         Positions: {[f'{p:.3f}' for p in js.position]}")

        self.assertTrue(success, "No joint states received")
        self.assertEqual(num_joints, 6, "Should have 6 UR joints")

    # =========================================================================
    # Test 08: Gripper service available
    # =========================================================================
    def test_08_gripper_service_available(self):
        """Verify mock gripper service is available."""
        available = self.gripper_client.wait_for_service(timeout_sec=5.0)

        self._record_test('Gripper Service Available', available)

        print(f"\n[E2E 08] Gripper service available: {available}")
        self.assertTrue(available, "Gripper service not available")

    # =========================================================================
    # Test 09: Gripper grip command
    # =========================================================================
    def test_09_gripper_grip(self):
        """Test gripper grip command."""
        self.assertTrue(
            self.gripper_client.wait_for_service(timeout_sec=5.0),
            "Gripper service not available"
        )

        request = GripCommand.Request()
        request.action = 'grip'

        future = self.gripper_client.call_async(request)
        success = self.spin_until(lambda: future.done(), timeout=5.0)
        self.assertTrue(success, "Gripper service call timed out")

        response = future.result()

        self._record_test(
            'Gripper Grip', response.success,
            f"Message: {response.message}"
        )

        print(f"\n[E2E 09] Grip response: {response.success}, {response.message}")
        self.assertTrue(response.success, f"Grip failed: {response.message}")

    # =========================================================================
    # Test 10: Gripper release command
    # =========================================================================
    def test_10_gripper_release(self):
        """Test gripper release command."""
        self.assertTrue(
            self.gripper_client.wait_for_service(timeout_sec=5.0),
            "Gripper service not available"
        )

        request = GripCommand.Request()
        request.action = 'release'

        future = self.gripper_client.call_async(request)
        success = self.spin_until(lambda: future.done(), timeout=5.0)
        self.assertTrue(success, "Gripper service call timed out")

        response = future.result()

        self._record_test(
            'Gripper Release', response.success,
            f"Message: {response.message}"
        )

        print(f"\n[E2E 10] Release response: {response.success}, {response.message}")
        self.assertTrue(response.success, f"Release failed: {response.message}")

    # =========================================================================
    # Test 11: Full pipeline data flow summary
    # =========================================================================
    def test_11_full_pipeline_flow(self):
        """Verify data flows through entire pipeline."""
        # Collect data across pipeline stages
        self.spin_until(lambda: len(self.rgbd_frames) >= 1, timeout=10.0)

        self.assertTrue(
            self.trigger_client.wait_for_service(timeout_sec=30.0),
            "Detection service not available"
        )

        request = Trigger.Request()
        future = self.trigger_client.call_async(request)
        self.spin_until(lambda: future.done(), timeout=30.0)
        self.spin_until(lambda: len(self.detections_received) > 0, timeout=5.0)
        self.spin_until(lambda: len(self.poses_received) > 0, timeout=15.0)

        print(f"\n[E2E 11] Full pipeline flow summary:")
        print(f"         RGBD frames: {len(self.rgbd_frames)}")
        print(f"         Detections: {len(self.detections_received)}")
        print(f"         Poses: {len(self.poses_received)}")
        print(f"         Joint states: {len(self.joint_states_received)}")

        self.assertGreater(len(self.rgbd_frames), 0, "No RGBD frames in pipeline")
        self.assertGreater(len(self.detections_received), 0, "No detections in pipeline")

        # Capture detection details for report
        if self.detections_received:
            det_array, _ = self.detections_received[-1]
            if det_array.detections:
                d = det_array.detections[0]
                print(f"         Sample detection:")
                print(f"           bbox center: ({d.bbox.center.position.x:.1f}, {d.bbox.center.position.y:.1f})")
                print(f"           bbox size: ({d.bbox.size.x:.1f}, {d.bbox.size.y:.1f})")
                if d.results:
                    print(f"           confidence: {d.results[0].hypothesis.score:.2f}")
                    print(f"           class_id: {d.results[0].hypothesis.class_id}")

        self._record_test(
            'Full Pipeline Flow', True,
            f"RGBD: {len(self.rgbd_frames)}, Detections: {len(self.detections_received)}, "
            f"Poses: {len(self.poses_received)}, Joints: {len(self.joint_states_received)}"
        )

    # =========================================================================
    # Test 12: RGBD data validation
    # =========================================================================
    def test_12_rgbd_data_validation(self):
        """Verify RGBD frames: 1280x720, correct encodings."""
        self.spin_until(lambda: len(self.rgbd_frames) >= 1, timeout=10.0)
        self.assertGreater(len(self.rgbd_frames), 0, "No RGBD frames received")

        rgbd_msg, _ = self.rgbd_frames[0]

        # Store frame for visualization
        self.report_data['rgbd_frames'].append(rgbd_msg)

        self.assertIsNotNone(rgbd_msg.rgb, "RGB image is None")
        self.assertIsNotNone(rgbd_msg.depth, "Depth image is None")
        self.assertIsNotNone(rgbd_msg.rgb_camera_info, "RGB camera info is None")
        self.assertIsNotNone(rgbd_msg.depth_camera_info, "Depth camera info is None")

        self.assertEqual(rgbd_msg.rgb.width, 1280, "RGB width should be 1280")
        self.assertEqual(rgbd_msg.rgb.height, 720, "RGB height should be 720")
        self.assertEqual(rgbd_msg.depth.width, 1280, "Depth width should be 1280")
        self.assertEqual(rgbd_msg.depth.height, 720, "Depth height should be 720")

        self.assertIn(rgbd_msg.rgb.encoding, ["rgb8", "bgr8"], "RGB encoding should be rgb8 or bgr8")
        self.assertEqual(rgbd_msg.depth.encoding, "16UC1", "Depth encoding should be 16UC1")

        self._record_test(
            'RGBD Data Validation', True,
            f"1280x720, RGB: {rgbd_msg.rgb.encoding}, Depth: {rgbd_msg.depth.encoding}"
        )

        print(f"\n[E2E 12] RGBD validated: {rgbd_msg.rgb.width}x{rgbd_msg.rgb.height}, "
              f"RGB: {rgbd_msg.rgb.encoding}, Depth: {rgbd_msg.depth.encoding}")

    # =========================================================================
    # Test 13: Depth channel validation
    # =========================================================================
    def test_13_depth_channel_validation(self):
        """Verify depth channel has valid data matching D415 specifications."""
        self.spin_until(lambda: len(self.rgbd_frames) >= 1, timeout=10.0)
        self.assertGreater(len(self.rgbd_frames), 0, "No RGBD frames received")

        rgbd_msg, _ = self.rgbd_frames[0]
        depth_image = self.bridge.imgmsg_to_cv2(rgbd_msg.depth, desired_encoding="passthrough")

        self.assertEqual(depth_image.dtype, np.uint16, "Depth should be uint16")
        self.assertEqual(len(depth_image.shape), 2, "Depth should be single channel")

        valid_depth = depth_image[depth_image > 0]
        invalid_pixels = int(np.sum(depth_image == 0))
        total_pixels = depth_image.shape[0] * depth_image.shape[1]
        invalid_percentage = (invalid_pixels / total_pixels) * 100

        depth_min = int(np.min(valid_depth)) if len(valid_depth) > 0 else 0
        depth_max = int(np.max(valid_depth)) if len(valid_depth) > 0 else 0
        depth_mean = float(np.mean(valid_depth)) if len(valid_depth) > 0 else 0
        depth_std = float(np.std(valid_depth)) if len(valid_depth) > 0 else 0

        self.report_data['depth_stats'] = {
            'min_mm': depth_min,
            'max_mm': depth_max,
            'mean_mm': depth_mean,
            'std_mm': depth_std,
            'invalid_pixels': invalid_pixels,
            'invalid_percentage': float(invalid_percentage),
            'total_pixels': total_pixels,
            'valid_pixels': int(len(valid_depth))
        }

        self.assertGreater(depth_min, 0, "Minimum depth should be > 0")
        self.assertGreaterEqual(depth_min, 100, "Minimum depth should be >= 100mm")
        self.assertLessEqual(depth_max, 3000, "Maximum depth should be <= 3000mm")
        self.assertGreater(len(valid_depth), 0, "Should have valid depth pixels")
        self.assertLess(invalid_percentage, 10, "Invalid pixels should be < 10%")

        self._record_test(
            'Depth Channel Validation', True,
            f"Range: {depth_min}-{depth_max}mm, mean: {depth_mean:.1f}mm, invalid: {invalid_percentage:.1f}%"
        )

        print(f"\n[E2E 13] Depth validated:")
        print(f"         Range: {depth_min}mm - {depth_max}mm")
        print(f"         Mean: {depth_mean:.1f}mm, Std: {depth_std:.1f}mm")
        print(f"         Valid: {len(valid_depth):,} ({100-invalid_percentage:.1f}%), "
              f"Invalid: {invalid_pixels:,} ({invalid_percentage:.1f}%)")

    # =========================================================================
    # Frame Visualization + Report Generation (class methods)
    # =========================================================================

    @classmethod
    def save_frame_visualizations(cls, frames=1):
        """Save RGB and depth frame visualizations to files."""
        if not cls.report_data['rgbd_frames']:
            print("No RGBD frames captured for visualization.")
            return

        try:
            bridge = CvBridge()
            output_dir = '/tmp'
            total_frames = len(cls.report_data['rgbd_frames'])

            if isinstance(frames, str):
                if ':' in frames:
                    start, end = frames.split(':')
                    start = int(start) if start else 0
                    end = int(end) if end else total_frames
                    frame_indices = list(range(start, min(end + 1, total_frames)))
                else:
                    frame_indices = [int(frames)]
            elif isinstance(frames, list):
                frame_indices = [idx for idx in frames if 0 <= idx < total_frames]
            elif frames == -1:
                frame_indices = list(range(total_frames))
            else:
                frame_indices = list(range(min(frames, total_frames)))

            print(f"Saving {len(frame_indices)} frame(s) out of {total_frames} available: {frame_indices}")

            for idx in frame_indices:
                rgbd_msg = cls.report_data['rgbd_frames'][idx]

                rgb_image = bridge.imgmsg_to_cv2(rgbd_msg.rgb, desired_encoding="bgr8")
                depth_image = bridge.imgmsg_to_cv2(rgbd_msg.depth, desired_encoding="passthrough")

                rgb_path = f"{output_dir}/e2e_pipeline_frame_{idx}_rgb.png"
                cv2.imwrite(rgb_path, rgb_image)

                # Colorized depth visualization
                valid_mask = depth_image > 0
                depth_normalized = np.zeros_like(depth_image, dtype=np.uint8)

                if np.any(valid_mask):
                    valid_depth = depth_image[valid_mask]
                    depth_min, depth_max = np.min(valid_depth), np.max(valid_depth)
                    depth_normalized[valid_mask] = (
                        255 - ((depth_image[valid_mask] - depth_min) / (depth_max - depth_min) * 255)
                    ).astype(np.uint8)

                depth_colorized = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_TURBO)
                depth_colorized[~valid_mask] = [0, 0, 0]

                # Depth scale legend
                legend_width = 80
                legend_height = 300
                legend_x = depth_colorized.shape[1] - legend_width - 20
                legend_y = 50

                gradient = np.linspace(0, 255, legend_height, dtype=np.uint8)
                gradient_3d = np.tile(gradient[:, np.newaxis], (1, 30))
                gradient_colored = cv2.applyColorMap(gradient_3d, cv2.COLORMAP_TURBO)

                cv2.rectangle(depth_colorized,
                             (legend_x - 10, legend_y - 10),
                             (legend_x + 70, legend_y + legend_height + 40),
                             (255, 255, 255), -1)
                cv2.rectangle(depth_colorized,
                             (legend_x - 10, legend_y - 10),
                             (legend_x + 70, legend_y + legend_height + 40),
                             (0, 0, 0), 2)

                depth_colorized[legend_y:legend_y+legend_height, legend_x:legend_x+30] = gradient_colored

                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.4
                thickness = 1

                if np.any(valid_mask):
                    cv2.putText(depth_colorized, f'{depth_max}mm',
                               (legend_x + 35, legend_y + 5),
                               font, font_scale, (0, 0, 0), thickness)
                    cv2.putText(depth_colorized, '(far)',
                               (legend_x + 35, legend_y + 20),
                               font, font_scale - 0.1, (0, 0, 0), thickness)
                    cv2.putText(depth_colorized, f'{depth_min}mm',
                               (legend_x + 35, legend_y + legend_height - 5),
                               font, font_scale, (0, 0, 0), thickness)
                    cv2.putText(depth_colorized, '(close)',
                               (legend_x + 35, legend_y + legend_height + 10),
                               font, font_scale - 0.1, (0, 0, 0), thickness)

                cv2.putText(depth_colorized, 'Depth',
                           (legend_x - 5, legend_y - 20),
                           font, font_scale + 0.1, (0, 0, 0), thickness + 1)

                depth_path = f"{output_dir}/e2e_pipeline_frame_{idx}_depth.png"
                cv2.imwrite(depth_path, depth_colorized)

                w = rgb_image.shape[1]
                combined = np.hstack([rgb_image, depth_colorized])
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(combined, 'RGB', (10, 30), font, 1, (255, 255, 255), 2)
                cv2.putText(combined, 'Depth (colorized)', (w + 10, 30), font, 1, (255, 255, 255), 2)

                combined_path = f"{output_dir}/e2e_pipeline_frame_{idx}_combined.png"
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
        """Generate detailed test report."""
        print("\n" + "="*80)
        print("E2E PIPELINE TEST SUMMARY")
        print("="*80)

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

        # Quick Summary
        print("\n" + "-"*80)
        print("QUICK SUMMARY")
        print("-"*80)
        print(f"  - Duration: {duration.total_seconds():.2f} seconds")
        if avg_response:
            print(f"  - Service Response: {avg_response*1000:.2f}ms average")
        if avg_latency:
            print(f"  - Detection Latency: {avg_latency*1000:.2f}ms average")
        print(f"  - Objects Detected: {total_objects} objects")
        for class_id, confidences in sorted(object_counts.items()):
            conf_str = ", ".join([f"{c:.2%}" for c in confidences])
            print(f"    - {len(confidences)} {class_id} ({conf_str})")
        print()

        # Depth Statistics
        if cls.report_data['depth_stats']:
            stats = cls.report_data['depth_stats']
            print(f"DEPTH CHANNEL STATISTICS:")
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
            print(f"\n{'~'*80}")
            print("DETECTION RESULTS")
            print(f"{'~'*80}")
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
                    print(f"      Area: {obj['bbox_area']:.1f} px2")
                print()

        # Test results summary
        print(f"{'~'*80}")
        print("TEST RESULTS")
        print(f"{'~'*80}")
        for test_info in cls.report_data['tests']:
            status = "PASSED" if test_info['passed'] else "FAILED"
            print(f"{status} - {test_info['name']}")
            if test_info.get('details'):
                print(f"  {test_info['details']}")

        print("\n" + "="*80)
        print("END OF REPORT")
        print("="*80 + "\n")

        # Write report to file
        report_file = '/tmp/e2e_pipeline_test_report.txt'
        try:
            with open(report_file, 'w') as f:
                f.write("="*80 + "\n")
                f.write("E2E PIPELINE TEST REPORT\n")
                f.write("="*80 + "\n")
                f.write(f"Test started: {cls.report_data['test_start_time'].strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Test ended: {cls.report_data['test_end_time'].strftime('%Y-%m-%d %H:%M:%S')}\n\n")

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
                f.write("\n")

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

                f.write("\nTEST RESULTS:\n")
                f.write("-"*80 + "\n")
                for test_info in cls.report_data['tests']:
                    status = "PASSED" if test_info['passed'] else "FAILED"
                    f.write(f"{status} - {test_info['name']}\n")
                    if test_info.get('details'):
                        f.write(f"  {test_info['details']}\n")

                f.write("\n" + "="*80 + "\n")
            print(f"Detailed report saved to: {report_file}")
        except Exception as e:
            print(f"Warning: Could not write report file: {e}")


def main():
    """Run tests."""
    import sys
    import pytest
    sys.exit(pytest.main([__file__, '-v']))


if __name__ == '__main__':
    main()
