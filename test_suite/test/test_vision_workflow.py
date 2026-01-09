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
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray
from datetime import datetime
import os


class TestVisionWorkflow(unittest.TestCase):
    """Test the complete vision detection workflow"""

    report_data = {
        'test_start_time': None,
        'test_end_time': None,
        'tests': [],
        'detections': [],
        'service_response_times': [],
        'detection_latencies': []
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
        cls.generate_report()
        rclpy.shutdown()

    def setUp(self):
        """Set up test node"""
        self.node = rclpy.create_node('test_vision_workflow')
        self.detections_received = []

        # Create subscription to detection topic
        self.detection_sub = self.node.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.detection_callback,
            10
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

    def wait_for_service(self, timeout_sec=10.0):
        """Wait for the capture_detections service to be available"""
        start_time = time.time()
        while not self.trigger_client.wait_for_service(timeout_sec=1.0):
            if time.time() - start_time > timeout_sec:
                return False
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return True

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

        # Time information
        print(f"Test completed at: {cls.report_data['test_end_time'].strftime('%Y-%m-%d %H:%M:%S')}")

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

        # Add extra delay to ensure vision node and YOLO model are fully ready
        # YOLO model loading can take several seconds
        time.sleep(5.0)

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
                      isinstance(detection_array, Detection2DArray) and
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
        self.assertIsInstance(detection_array, Detection2DArray)
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
            self.assertGreater(detection.bbox.size_x, 0, "Invalid bbox width")
            self.assertGreater(detection.bbox.size_y, 0, "Invalid bbox height")

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
                'bbox_width': detection.bbox.size_x,
                'bbox_height': detection.bbox.size_y,
                'bbox_area': detection.bbox.size_x * detection.bbox.size_y
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


def main():
    """Run tests"""
    import sys
    import pytest

    # Run pytest with this file
    sys.exit(pytest.main([__file__, '-v']))


if __name__ == '__main__':
    main()
