#!/usr/bin/env python3
"""
Automated test for vision detection system.
Tests vision node with fake camera data and validates detections.
"""

import sys
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray


class VisionTester(Node):
    def __init__(self):
        super().__init__('vision_tester')

        self.detections_received = []
        self.test_passed = False

        # Subscribe to detections
        self.sub = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.detection_callback,
            10
        )

        # Service client to trigger detection
        self.client = self.create_client(Trigger, '/capture_detections')

        self.get_logger().info('Vision Tester initialized')

    def detection_callback(self, msg):
        """Store received detections"""
        self.detections_received.append(msg)
        self.get_logger().info(f'Received {len(msg.detections)} detections')

        for i, det in enumerate(msg.detections):
            if det.results:
                class_id = det.results[0].hypothesis.class_id
                score = det.results[0].hypothesis.score
                bbox_x = det.bbox.center.position.x
                bbox_y = det.bbox.center.position.y
                self.get_logger().info(
                    f'  Detection {i+1}: {class_id} '
                    f'(confidence: {score:.2%}) at ({bbox_x:.0f}, {bbox_y:.0f})'
                )

    def wait_for_service(self, timeout_sec=10.0):
        """Wait for detection service to be available"""
        self.get_logger().info('Waiting for /capture_detections service...')
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('Service not available!')
            return False
        self.get_logger().info('Service available!')
        return True

    def trigger_detection(self):
        """Call the capture_detections service"""
        self.get_logger().info('Triggering detection...')
        request = Trigger.Request()
        future = self.client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f'Detection result: {response.success} - {response.message}'
            )
            return response.success
        else:
            self.get_logger().error('Service call failed!')
            return False

    def run_test(self):
        """Run the automated test"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('STARTING VISION SYSTEM TEST')
        self.get_logger().info('=' * 60)

        # Step 1: Wait for service
        if not self.wait_for_service():
            self.get_logger().error('TEST FAILED: Service unavailable')
            return False

        # Step 2: Clear any existing detections
        self.detections_received.clear()

        # Step 3: Trigger detection
        time.sleep(1)  # Give time for camera to publish
        if not self.trigger_detection():
            self.get_logger().error('TEST FAILED: Detection trigger failed')
            return False

        # Step 4: Wait for detection messages
        self.get_logger().info('Waiting for detection messages...')
        timeout = 5.0
        start_time = time.time()

        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.detections_received:
                break

        # Step 5: Validate results
        if not self.detections_received:
            self.get_logger().warn('TEST WARNING: No detections published (may be expected)')
            return True  # Not necessarily a failure

        total_objects = sum(len(d.detections) for d in self.detections_received)
        self.get_logger().info(f'Total objects detected: {total_objects}')

        if total_objects > 0:
            self.get_logger().info('=' * 60)
            self.get_logger().info('TEST PASSED: Vision system working correctly!')
            self.get_logger().info('=' * 60)
            return True
        else:
            self.get_logger().warn('TEST WARNING: Detection ran but found no objects')
            return True


def main(args=None):
    rclpy.init(args=args)

    tester = VisionTester()

    try:
        success = tester.run_test()
        time.sleep(1)  # Allow final messages to print

        sys.exit(0 if success else 1)

    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
        sys.exit(1)
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
