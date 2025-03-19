import rclpy
from rclpy.node import Node
from grip_interface.srv import GripCommand
from std_msgs.msg import UInt8MultiArray, String
import time

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

        self.get_logger().info("GripperNode_v4 is ready to receive commands.")

        # Gripper initialisieren
        init_msg = UInt8MultiArray()
        init_msg.data = [9, 6, 3, 232, 9, 0, 14, 162]
        self.publisher.publish(init_msg)
        self.get_logger().info("Gripper initialized.")
        time.sleep(1) 

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
                elif value == 249:
                    result.data = "no object detected"
                elif value == 57:
                    result.data = "unknown object detection. Regulating towards requested vacuum/pressure"
                elif value == 121:
                    result.data = "object detected. Minimum vacuum reached"
                elif value == 192:
                    result.data = "Gripper not activated (0xC0)"
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
