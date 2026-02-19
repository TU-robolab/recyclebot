#!/usr/bin/env python3
"""Launch gate node: waits for a service call or timeout before exiting.

Used by real-hardware launch files to give the operator time to enable
External Control URCap on the UR teach pendant.

Sequence:
    1. Wait for UR driver to initialize (initial_wait_sec, default 10s)
    2. Then wait for either:
       - ros2 service call /launch_gate std_srvs/srv/Trigger
       - timeout expires (timeout_sec, default 30s)
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class LaunchGate(Node):
    def __init__(self):
        super().__init__("launch_gate")

        self.declare_parameter("timeout_sec", 30.0)
        self.declare_parameter("initial_wait_sec", 10.0)
        self.timeout = self.get_parameter("timeout_sec").value
        self.initial_wait = self.get_parameter("initial_wait_sec").value
        self._gate_started = False

        self.get_logger().info(
            f"Waiting {self.initial_wait}s for UR driver to initialize..."
        )

        # Stage 1: wait for driver init, then start the actual gate
        self._init_timer = self.create_timer(self.initial_wait, self._start_gate)

    def _start_gate(self):
        # one-shot: cancel the init timer after first fire
        self._init_timer.cancel()

        if self._gate_started:
            return
        self._gate_started = True

        self.get_logger().info(">>> Enable External Control URCap on teach pendant <<<")
        self.get_logger().info(
            "Then call: ros2 service call /launch_gate std_srvs/srv/Trigger"
        )
        self.get_logger().info(
            f"Or wait {self.timeout}s for auto-continue..."
        )

        self.create_timer(self.timeout, self._on_timeout)
        self.create_service(Trigger, "/launch_gate", self._on_trigger)

    def _on_timeout(self):
        self.get_logger().info(f"Timeout ({self.timeout}s) reached, continuing...")
        raise SystemExit(0)

    def _on_trigger(self, req, res):
        self.get_logger().info("Gate triggered via service call, continuing...")
        res.success = True
        res.message = "ok"
        self.create_timer(0.1, lambda: (_ for _ in ()).throw(SystemExit(0)))
        return res


def main(args=None):
    rclpy.init(args=args)
    node = LaunchGate()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
