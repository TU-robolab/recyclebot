import rclpy
from rclpy.node import Node
from grip_interface.srv import GripCommand
import subprocess

class GripperNode(Node):     
    def __init__(self):
        super().__init__('gripper_node')
        self.srv = self.create_service(GripCommand, 'gripper_action', self.execute_command)
        self.get_logger().info("GripperNode2 is ready to receive commands.")

    def execute_command(self, request, response):
        try:
            if request.action == "grip":  # If the request is True
                # Close gripper
                subprocess.run(['bash', '-c', "printf '\\x09\\x06\\x03\\xE8\\x09\\x00\\x0E\\xA2' > /tmp/ttyUR"], check=True)
                subprocess.run(['bash', '-c', "printf '\\x09\\x06\\x03\\xE9\\x00\\x63\\x19\\x1B' > /tmp/ttyUR"], check=True)
                response.message = "Gripper closed."
            if request.action == "release":  # If the request is False
                # Open gripper
                subprocess.run(['bash', '-c', "printf '\\x09\\x06\\x03\\xE9\\x00\\x65\\x99\\x19' > /tmp/ttyUR"], check=True)
                response.message = "Gripper opened."
            
            response.success = True
        except subprocess.CalledProcessError as e:
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
