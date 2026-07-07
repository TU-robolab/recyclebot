#!/usr/bin/env python3
"""
Mock gripper service for E2E testing.

Simulates gripper behavior by returning success for grip/release actions
without requiring real hardware.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from grip_interface.srv import GripCommand


class MockGripperService(Node):
    def __init__(self):
        super().__init__('mock_gripper_service')

        self.srv = self.create_service(
            GripCommand,
            '/gripper_action',
            self.handle_grip_command
        )

        # track gripper state for realistic simulation
        self.is_gripping = False

        # Mimic the real gripper node's 1 Hz vacuum-sensor status so the control
        # node's grasp verification (confirm_grasp) is exercised in tests too.
        self.status_pub = self.create_publisher(String, '/object_detection/status', 10)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Mock gripper service started on /gripper_action')

    def publish_status(self):
        msg = String()
        msg.data = 'object detected' if self.is_gripping else 'no object detected'
        self.status_pub.publish(msg)

    def handle_grip_command(self, request, response):
        action = request.action.lower()

        if action == 'grip':
            if self.is_gripping:
                response.success = True
                response.message = 'Already gripping'
            else:
                self.is_gripping = True
                response.success = True
                response.message = 'Grip successful (mock)'
            self.get_logger().info(f'Grip command: {response.message}')

        elif action == 'release':
            if not self.is_gripping:
                response.success = True
                response.message = 'Already released'
            else:
                self.is_gripping = False
                response.success = True
                response.message = 'Release successful (mock)'
            self.get_logger().info(f'Release command: {response.message}')

        else:
            response.success = False
            response.message = f'Unknown action: {action}'
            self.get_logger().warn(f'Unknown gripper action: {action}')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockGripperService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
