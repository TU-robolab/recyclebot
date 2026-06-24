import rclpy
from rclpy.node import Node
from grip_interface.srv import GripCommand
from std_msgs.msg import UInt8MultiArray, String

class GripperNode(Node):     
    def __init__(self):
        super().__init__('gripper_node')

        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

        # Service
        self.srv = self.create_service(GripCommand, 'gripper_action', self.execute_command)

        # Publisher für Gripper-Befehle
        self.publisher = self.create_publisher(UInt8MultiArray, '/serial/com1/inject/output', 10)

        # Publisher für Objekterkennung
        self.detection_publisher = self.create_publisher(String, '/object_detection/status', 10)

        # Subscriber zum Empfangen von Inspektionsdaten
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            '/serial/com1/inspect/input',
            self.inspect_callback,
            10
        )

        # Timer für zyklische Abfrage
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.query_sensor)

        # Gripper initialisieren (closed-loop).
        # The activation message must not be published until the serial bridge
        # (remote_serial) has actually subscribed to the topic. With the default
        # VOLATILE QoS, anything published before discovery completes is silently
        # dropped, which left the gripper un-activated intermittently.
        #
        # Instead of firing once, we re-send the activation command on a timer
        # until the gripper's own status (decoded in inspect_callback) reports it
        # is activated. query_sensor() polls that status every second, so the two
        # timers form a request/confirm loop that self-corrects.
        self.gripper_activated = False
        self._init_attempts = 0
        self._init_timer = self.create_timer(1.0, self._try_initialize_gripper)

    def _try_initialize_gripper(self):
        if self.gripper_activated:
            self.get_logger().info("Gripper activation confirmed.")
            self._init_timer.cancel()
            return

        self._init_attempts += 1
        if self.publisher.get_subscription_count() < 1:
            self.get_logger().info(
                "Waiting for serial bridge to subscribe before activating gripper..."
            )
            return

        init_msg = UInt8MultiArray()
        init_msg.data = [9, 6, 3, 232, 9, 0, 14, 162]
        self.publisher.publish(init_msg)
        self.get_logger().info(
            f"Activation command sent (attempt {self._init_attempts}); "
            "awaiting status confirmation..."
        )
        if self._init_attempts % 10 == 0:
            self.get_logger().warn(
                f"Gripper still not confirmed activated after {self._init_attempts} attempts."
            )

    def execute_command(self, request, response):
        try:
            msg = UInt8MultiArray()
            if request.action == "grip":
                msg.data = [9, 6, 3, 233, 0, 99, 25, 27]
                response.message = "Gripper closed."
            elif request.action == "release":
                msg.data = [9, 6, 3, 233, 0, 101, 153, 25]
                response.message = "Gripper opened."
            else:
                response.success = False
                response.message = "Invalid action. Use 'grip' or 'release'."
                return response
            
            self.publisher.publish(msg)
            response.success = True
        except Exception as e:
            response.success = False
            response.message = f"Error executing command: {e}"

        return response

    def query_sensor(self):
        """Sends a query to the sensor at regular intervals."""
        sensor_msg = UInt8MultiArray()
        sensor_msg.data = [9, 4, 7, 208, 0, 2, 112, 14]
        self.publisher.publish(sensor_msg)

        if self.debug:
            self.get_logger().info("Sensorabfrage gesendet.")

    def inspect_callback(self, msg):
        """Processes the incoming inspection data."""
        try:
            if len(msg.data) >= 4:
                if msg.data[1] != 4:
                    self.get_logger().debug("Message ignored because its not a read message.")
                    return

                value = msg.data[3]
                result = String()
                if value == 185:
                    result.data = "object detected"
                    self.gripper_activated = True
                elif value == 249:
                    result.data = "no object detected"
                    self.gripper_activated = True
                elif value == 57:
                    result.data = "unknown object detection. Regulating towards requested vacuum/pressure"
                    self.gripper_activated = True
                elif value == 121:
                    result.data = "object detected. Minimum vacuum reached"
                    self.gripper_activated = True
                elif value == 192:
                    result.data = "Gripper not activated (0xC0)"
                    # Status explicitly reports the gripper is not activated; keep
                    # the init loop re-sending the activation command.
                    self.gripper_activated = False
                else:
                    result.data = f"unknown value: {value}"
                self.detection_publisher.publish(result)
                if self.debug:
                    self.get_logger().info(f"Inspection result: {result.data}")
            else:
                self.get_logger().warn("Inspection message too short.")
        except Exception as e:
            self.get_logger().error(f"Errors when processing the inspection data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
