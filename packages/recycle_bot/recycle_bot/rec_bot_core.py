#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class RecBotCore(Node):

    def __init__(self):
        super().__init__("rec_bot_core")
        self.get_logger().info("Hello world from the Python node rec_bot_core")


def main(args=None):
    rclpy.init(args=args)

    rec_bot_core = RecBotCore()

    try:
        rclpy.spin(rec_bot_core)
    except KeyboardInterrupt:
        pass

    rec_bot_core.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
