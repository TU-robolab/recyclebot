#!/usr/bin/env python3
"""
Vision Workflow End-to-End Integration Tests

Tests the complete object detection pipeline:
    Fake RGBD Camera → Vision Detector → Object Detections

Test Coverage:
    1. Service Availability - /capture_detections service is accessible
    2. Detection Triggering - Service calls successfully trigger detections
    3. Topic Publishing - Detections published to /object_detections topic
    4. Detection Format - Output has valid structure and metadata

Features:
    - Automated performance metrics collection
    - Detailed detection metadata capture (bbox, confidence, class)
    - Real-time console reporting
    - Saved report file (/tmp/vision_workflow_test_report.txt)
    - Quick summary with key metrics

Usage:
    # Via automated script (recommended)
    bash scripts/run_vision_test.sh

    # Direct pytest
    python3 -m pytest test/test_vision_workflow.py -v -s

See README_VISION_TESTS.md for complete documentation.
"""

import unittest
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection3DArray
from realsense2_camera_msgs.msg import RGBD
from datetime import datetime
import os
import numpy as np
from cv_bridge import CvBridge
import cv2
import tf2_ros


class TestVisionWorkflow(unittest.TestCase):
    """Test the complete vision detection workflow"""

    report_data = {
        'test_start_time': None,
        'test_end_time': None,
        'tests': [],
        'detections': [],
        'service_response_times': [],
        'detection_latencies': [],
        'rgbd_frames': [],
        'depth_stats': {},
        'frame_visualizations': []
    }

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2"""
        rclpy.init()
        cls.report_data['test_start_time'] = datetime.now()
        print("\n" + "="*80)
        print("VISION WORKFLOW TEST REPORT")
        print("="*80)
        print(f"Test started at: {cls.report_data['test_start_time'].strftime('%Y-%m-%d %H:%M:%S')}")
        print("="*80 + "\n")

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 and generate report"""
        cls.report_data['test_end_time'] = datetime.now()

        # Save frame visualizations before generating report
        print("\n" + "="*80)
        print("SAVING FRAME VISUALIZATIONS")
        print("="*80)
        cls.save_frame_visualizations()

        cls.generate_report()
        rclpy.shutdown()

    def setUp(self):
        """Set up test node"""
        self.node = rclpy.create_node('test_vision_workflow')
        self.detections_received = []
        self.rgbd_frames = []
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        # Create subscription to detection topic (RELIABLE QoS to match vision publisher)
        qos_detections = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.detection_sub = self.node.create_subscription(
            Detection3DArray,
            '/object_detections',
            self.detection_callback,
            qos_detections
        )

        # Create subscription to RGBD camera topic for depth tests
        # Use BEST_EFFORT QoS to match the fake camera publisher
        qos_camera_feed = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.rgbd_sub = self.node.create_subscription(
            RGBD,
            '/camera/camera/rgbd',
            self.rgbd_callback,
            qos_camera_feed
        )

        # Create service client for triggering detections
        self.trigger_client = self.node.create_client(
            Trigger,
            '/capture_detections'
        )

    def tearDown(self):
        """Clean up test node"""
        self.node.destroy_node()

    def detection_callback(self, msg):
        """Store received detections"""
        self.detections_received.append((msg, time.time()))

    def rgbd_callback(self, msg):
        """Store received RGBD frames with timestamp"""
        self.rgbd_frames.append((msg, time.time()))

    def wait_for_service(self, timeout_sec=10.0):
        """Wait for the capture_detections service to be available"""
        start_time = time.time()
        while not self.trigger_client.wait_for_service(timeout_sec=1.0):
            if time.time() - start_time > timeout_sec:
                return False
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return True

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
                rgb_path = f"{output_dir}/rgbd_frame_{idx}_rgb.png"
                cv2.imwrite(rgb_path, rgb_image)

                # Create colorized depth visualization
                # Normalize depth to 0-255 for visualization (ignoring invalid pixels)
                # INVERT so close=255 (red/warm) and far=0 (blue/cool)
                # Filter: exclude invalid (0 and 65535)
                valid_mask = (depth_image > 0) & (depth_image < 65535)
                depth_normalized = np.zeros_like(depth_image, dtype=np.uint8)

                # Fixed depth range: 0.2m to 1.5m (200mm to 1500mm)
                depth_min, depth_max = 200, 1500

                if np.any(valid_mask):
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
                depth_path = f"{output_dir}/rgbd_frame_{idx}_depth.png"
                cv2.imwrite(depth_path, depth_colorized)

                # Create side-by-side comparison
                w = rgb_image.shape[1]  # Get width for label positioning
                combined = np.hstack([rgb_image, depth_colorized])

                # Add labels
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(combined, 'RGB', (10, 30), font, 1, (255, 255, 255), 2)
                cv2.putText(combined, 'Depth (colorized)', (w + 10, 30), font, 1, (255, 255, 255), 2)

                combined_path = f"{output_dir}/rgbd_frame_{idx}_combined.png"
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
        print("VISION WORKFLOW TEST SUMMARY")
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
        report_file = '/tmp/vision_workflow_test_report.txt'
        try:
            with open(report_file, 'w') as f:
                f.write("="*80 + "\n")
                f.write("VISION WORKFLOW TEST REPORT\n")
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

    def test_01_service_available(self):
        """Test that the capture_detections service is available"""
        start_time = time.time()
        result = self.wait_for_service(timeout_sec=15.0)
        elapsed = time.time() - start_time

        self.report_data['tests'].append({
            'name': 'Service Availability',
            'passed': result,
            'details': f"Service became available in {elapsed:.2f}s"
        })

        print(f"\n[Test 1] Service available: {result} (took {elapsed:.2f}s)")

        self.assertTrue(result, "Service /capture_detections not available after 15 seconds")

    def test_02_trigger_detection(self):
        """Test triggering a detection via service call"""
        # Wait for service
        self.assertTrue(self.wait_for_service(timeout_sec=15.0))

        # Wait for vision node to receive RGBD frames before triggering
        self.rgbd_frames.clear()
        timeout = 10.0
        start_time = time.time()
        while len(self.rgbd_frames) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.fail("No RGBD frames received - vision node may not be subscribed")

        print(f"         RGBD frames received: {len(self.rgbd_frames)}")

        # Create request
        request = Trigger.Request()

        # Call service
        service_start = time.time()
        future = self.trigger_client.call_async(request)

        # Wait for response
        timeout = 30.0  # YOLO inference can take time on CPU
        start_time = time.time()
        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.fail("Service call timeout after 30 seconds")

        response = future.result()
        service_elapsed = time.time() - service_start
        self.report_data['service_response_times'].append(service_elapsed)

        # Verify response - log the message if it fails
        test_passed = response.success and "detection" in response.message.lower()

        self.report_data['tests'].append({
            'name': 'Trigger Detection Service',
            'passed': test_passed,
            'details': f"Response time: {service_elapsed*1000:.2f}ms, Message: {response.message}"
        })

        print(f"\n[Test 2] Service response: {response.success} ({service_elapsed*1000:.2f}ms)")
        print(f"         Message: {response.message}")

        self.assertTrue(response.success, f"Service call failed: {response.message}")
        self.assertIn("detection", response.message.lower(),
                     f"Unexpected message: {response.message}")

    def test_03_detections_published(self):
        """Test that detections are published to /object_detections topic"""
        # Wait for service and trigger detection
        self.assertTrue(self.wait_for_service(timeout_sec=15.0))

        # Clear previous detections
        self.detections_received.clear()

        # Trigger detection
        request = Trigger.Request()
        service_call_time = time.time()
        future = self.trigger_client.call_async(request)

        # Wait for service response
        timeout = 30.0
        start_time = time.time()
        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.fail("Service call timeout")

        # Wait for detections to be published
        # The vision node processes detections at 10Hz (every 0.1s)
        timeout = 5.0
        start_time = time.time()
        while len(self.detections_received) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.fail("No detections received after 5 seconds")

        # Calculate latency
        detection_array, detection_time = self.detections_received[0]
        latency = detection_time - service_call_time
        self.report_data['detection_latencies'].append(latency)

        # Verify detections
        test_passed = (len(self.detections_received) > 0 and
                      isinstance(detection_array, Detection3DArray) and
                      len(detection_array.detections) > 0)

        self.report_data['tests'].append({
            'name': 'Detections Published',
            'passed': test_passed,
            'details': f"Received {len(detection_array.detections)} detections, latency: {latency*1000:.2f}ms"
        })

        print(f"\n[Test 3] Detections published: {len(detection_array.detections)} objects")
        print(f"         Latency: {latency*1000:.2f}ms")

        self.assertGreater(len(self.detections_received), 0,
                          "No detection messages received")
        self.assertIsInstance(detection_array, Detection3DArray)
        self.assertGreater(len(detection_array.detections), 0,
                          "Detection array is empty")

    def test_04_detection_format(self):
        """Test that detections have the correct format"""
        # Trigger and wait for detections
        self.assertTrue(self.wait_for_service(timeout_sec=15.0))
        self.detections_received.clear()

        request = Trigger.Request()
        future = self.trigger_client.call_async(request)

        # Wait for response
        timeout = 30.0
        start_time = time.time()
        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.fail("Service call timeout")

        # Wait for detections
        timeout = 5.0
        start_time = time.time()
        while len(self.detections_received) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.fail("No detections received")

        # Verify detection format and capture detailed information
        detection_array, detection_time = self.detections_received[0]

        detection_info = {
            'timestamp': datetime.fromtimestamp(detection_time).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            'num_objects': len(detection_array.detections),
            'objects': []
        }

        test_passed = True
        for detection in detection_array.detections:
            # Check bounding box
            self.assertIsNotNone(detection.bbox)
            self.assertGreater(detection.bbox.size.x, 0, "Invalid bbox width")
            self.assertGreater(detection.bbox.size.y, 0, "Invalid bbox height")

            # Check results (class and confidence)
            self.assertGreater(len(detection.results), 0, "No detection results")
            result = detection.results[0]
            self.assertIsNotNone(result.hypothesis.class_id, "No class ID")
            self.assertGreater(result.hypothesis.score, 0.0, "Invalid confidence score")
            self.assertLessEqual(result.hypothesis.score, 1.0, "Confidence score > 1.0")

            # Capture detection details
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

        self.report_data['tests'].append({
            'name': 'Detection Format & Details',
            'passed': test_passed,
            'details': f"Validated {len(detection_array.detections)} detections with proper format"
        })

        print(f"\n[Test 4] Detection format validated: {len(detection_array.detections)} objects")
        for idx, obj in enumerate(detection_info['objects'], 1):
            print(f"         Object {idx}: {obj['class_id']} ({obj['confidence']:.2%})")
            print(f"           BBox: ({obj['bbox_center_x']:.1f}, {obj['bbox_center_y']:.1f}) "
                  f"{obj['bbox_width']:.1f}x{obj['bbox_height']:.1f}px")

    def test_04b_tf_available(self):
        """Test that TF can resolve base_link to camera_color_optical_frame"""
        timeout = 5.0
        start_time = time.time()
        transform = None
        last_error = None

        while time.time() - start_time < timeout:
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
        details = "Transform available"
        if transform is None:
            details = f"Transform unavailable: {last_error}"

        self.report_data['tests'].append({
            'name': 'TF base_link -> camera_color_optical_frame',
            'passed': test_passed,
            'details': details
        })

        self.assertIsNotNone(
            transform,
            f"TF missing between base_link and camera_color_optical_frame: {last_error}"
        )

    def test_05_rgbd_data_available(self):
        """Test that RGBD frames are being published from the fake camera"""
        # Clear any previous frames
        self.rgbd_frames.clear()

        # Wait for RGBD frames to arrive
        timeout = 5.0
        start_time = time.time()
        while len(self.rgbd_frames) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.fail("No RGBD frames received after 5 seconds")

        # Get the first RGBD frame
        rgbd_msg, frame_time = self.rgbd_frames[0]

        # Store frame for visualization BEFORE assertions (so we get it even if test fails)
        self.report_data['rgbd_frames'].append(rgbd_msg)

        # Verify the message structure
        test_passed = True
        try:
            self.assertIsNotNone(rgbd_msg.rgb, "RGB image is None")
            self.assertIsNotNone(rgbd_msg.depth, "Depth image is None")
            self.assertIsNotNone(rgbd_msg.rgb_camera_info, "RGB camera info is None")
            self.assertIsNotNone(rgbd_msg.depth_camera_info, "Depth camera info is None")

            # Check image dimensions
            self.assertEqual(rgbd_msg.rgb.width, 1280, "RGB width should be 1280")
            self.assertEqual(rgbd_msg.rgb.height, 720, "RGB height should be 720")
            self.assertEqual(rgbd_msg.depth.width, 1280, "Depth width should be 1280")
            self.assertEqual(rgbd_msg.depth.height, 720, "Depth height should be 720")

            # Check encodings (RealSense outputs rgb8)
            self.assertIn(rgbd_msg.rgb.encoding, ["rgb8", "bgr8"], "RGB encoding should be rgb8 or bgr8")
            self.assertEqual(rgbd_msg.depth.encoding, "16UC1", "Depth encoding should be 16UC1")

        except AssertionError as e:
            test_passed = False
            raise e

        self.report_data['tests'].append({
            'name': 'RGBD Data Available',
            'passed': test_passed,
            'details': f"RGBD frames publishing at 1280x720, RGB: {rgbd_msg.rgb.encoding}, Depth: {rgbd_msg.depth.encoding}"
        })

        print(f"\n[Test 5] RGBD data available: {len(self.rgbd_frames)} frame(s) received")
        print(f"         Resolution: {rgbd_msg.rgb.width}x{rgbd_msg.rgb.height}")
        print(f"         RGB encoding: {rgbd_msg.rgb.encoding}, Depth encoding: {rgbd_msg.depth.encoding}")

    def test_06_depth_channel_validation(self):
        """Test that depth channel has valid data matching D415 specifications"""
        # Get an RGBD frame (reuse from previous test if available)
        if len(self.rgbd_frames) == 0:
            self.rgbd_frames.clear()
            timeout = 5.0
            start_time = time.time()
            while len(self.rgbd_frames) == 0:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if time.time() - start_time > timeout:
                    self.fail("No RGBD frames received")

        rgbd_msg, _frame_time = self.rgbd_frames[0]

        # Convert depth image to numpy array
        depth_image = self.bridge.imgmsg_to_cv2(rgbd_msg.depth, desired_encoding="passthrough")

        # Verify it's 16-bit unsigned
        self.assertEqual(depth_image.dtype, np.uint16, "Depth should be uint16")
        self.assertEqual(len(depth_image.shape), 2, "Depth should be single channel")

        # Calculate depth statistics
        valid_depth = depth_image[depth_image > 0]  # Exclude invalid pixels (0 values)
        invalid_pixels = np.sum(depth_image == 0)
        total_pixels = depth_image.shape[0] * depth_image.shape[1]
        invalid_percentage = (invalid_pixels / total_pixels) * 100

        depth_min = np.min(valid_depth) if len(valid_depth) > 0 else 0
        depth_max = np.max(valid_depth) if len(valid_depth) > 0 else 0
        depth_mean = np.mean(valid_depth) if len(valid_depth) > 0 else 0
        depth_std = np.std(valid_depth) if len(valid_depth) > 0 else 0

        # Store statistics for report
        self.report_data['depth_stats'] = {
            'min_mm': int(depth_min),
            'max_mm': int(depth_max),
            'mean_mm': float(depth_mean),
            'std_mm': float(depth_std),
            'invalid_pixels': int(invalid_pixels),
            'invalid_percentage': float(invalid_percentage),
            'total_pixels': int(total_pixels),
            'valid_pixels': int(len(valid_depth))
        }

        # Verify depth is within D415 working range (300mm to 3000mm)
        test_passed = True
        try:
            self.assertGreater(depth_min, 0, "Minimum depth should be > 0")
            # D415 min ~300mm, D435 min ~200mm, allow some noise below spec
            self.assertGreaterEqual(depth_min, 100, "Minimum depth should be >= 100mm (allowing sensor noise)")
            self.assertLessEqual(depth_max, 3000, "Maximum depth should be <= 3000mm (D415 max range)")
            self.assertGreater(len(valid_depth), 0, "Should have some valid depth pixels")
            self.assertLess(invalid_percentage, 10, "Invalid pixels should be < 10%")
        except AssertionError as e:
            test_passed = False
            raise e

        self.report_data['tests'].append({
            'name': 'Depth Channel Validation',
            'passed': test_passed,
            'details': f"Depth range: {depth_min}-{depth_max}mm, mean: {depth_mean:.1f}mm, invalid: {invalid_percentage:.1f}%"
        })

        print(f"\n[Test 6] Depth channel validated:")
        print(f"         Range: {depth_min}mm - {depth_max}mm (D415 spec: 300-3000mm)")
        print(f"         Mean: {depth_mean:.1f}mm, Std Dev: {depth_std:.1f}mm")
        print(f"         Valid pixels: {len(valid_depth):,} ({100-invalid_percentage:.1f}%)")
        print(f"         Invalid pixels: {invalid_pixels:,} ({invalid_percentage:.1f}%)")


def main():
    """Run tests"""
    import sys
    import pytest

    # Run pytest with this file
    sys.exit(pytest.main([__file__, '-v']))


if __name__ == '__main__':
    main()
