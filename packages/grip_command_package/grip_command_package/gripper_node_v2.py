import rclpy
from rclpy.node import Node
from grip_interface.srv import GripCommand
from std_msgs.msg import UInt8MultiArray

class GripperNode(Node):     
    def __init__(self):
        super().__init__('gripper_node')
        self.srv = self.create_service(GripCommand, 'gripper_action', self.execute_command)
        self.publisher = self.create_publisher(UInt8MultiArray, '/serial/com1/inject/output', 10)
        self.get_logger().info("GripperNode_v2 is ready to receive commands.")

    def execute_command(self, request, response):
        try:
            msg = UInt8MultiArray()
            if request.action == "grip":  # Close gripper
                msg.data = [9, 6, 3, 233, 0, 99, 25, 27]
                response.message = "Gripper closed."
            elif request.action == "release":  # Open gripper
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

def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
