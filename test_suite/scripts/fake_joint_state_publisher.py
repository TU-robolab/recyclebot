#!/usr/bin/env python3
"""
Fake joint state publisher for testing robot control without hardware.
Publishes joint states that simulate a UR16e robot.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class FakeJointStatePublisher(Node):
    def __init__(self):
        super().__init__('fake_joint_state_publisher')

        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz

        # UR16e joint names
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Start at home position (all zeros)
        self.positions = [0.0] * 6
        self.velocities = [0.0] * 6
        self.efforts = [0.0] * 6

        # Simulate slow oscillation to show "movement"
        self.time = 0.0

        self.get_logger().info('Fake Joint State Publisher started')
        self.get_logger().info(f'Publishing joint states for: {", ".join(self.joint_names)}')

    def publish_joint_states(self):
        """Publish fake joint states with slight oscillation"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.name = self.joint_names

        # Add slight sinusoidal motion to make it more realistic
        self.time += 0.1
        oscillation = 0.1 * math.sin(self.time * 0.5)

        # Home position with slight oscillation
        msg.position = [
            -1.57 + oscillation,  # shoulder_pan
            -1.57,                # shoulder_lift
            1.57,                 # elbow
            -1.57,                # wrist_1
            1.57,                 # wrist_2
            0.0                   # wrist_3
        ]

        msg.velocity = [0.01 * math.cos(self.time * 0.5)] + [0.0] * 5
        msg.effort = [0.0] * 6

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeJointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()