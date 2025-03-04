import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from moveit_commander import MoveGroupCommander
import tf2_ros
import yaml
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from collections import deque

class cobot_control(Node):
    def __init__(self):
        super().__init__('cobot_control')
        
        # Load sorting sequence from YAML file
        self.sorting_sequence = self.load_sorting_sequence()
        self.sequence_index = 0
        self.task_queue = deque()
        self.executing_task = False

        # MoveIt2 Interface
        self.move_group = MoveGroupCommander('manipulator')
        
        # TF2 Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Vision subscriber (detects object availability and poses)
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(PoseStamped, '/vision/detected_object', self.vision_callback, qos_profile)
        
        # Timer for checking and processing tasks
        self.create_timer(1.0, self.process_tasks)

        self.get_logger().info("UR16e sorter node initialized!")

    def load_sorting_sequence(self):
        yaml_path = os.path.join(os.path.dirname(__file__), 'sorting_sequence.yaml')
        try:
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                return data.get('sorting_sequence', [])
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML: {e}")
            return []

    def vision_callback(self, msg: PoseStamped):
        """Handles incoming object pose from the vision system and queues it."""
        try:
            transform = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, rclpy.time.Time())
            transformed_pose = tf2_ros.do_transform_pose(msg, transform)
            target_pose = self.get_next_sorting_pose()
            self.task_queue.append((transformed_pose, target_pose))
            self.get_logger().info("Queued new sorting task.")
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")

    def get_next_sorting_pose(self):
        """Returns the next target pose from the predefined sequence, cycling if needed."""
        if not self.sorting_sequence:
            self.get_logger().error("No sorting sequence available!")
            return None
        target_pose = self.sorting_sequence[self.sequence_index]
        self.sequence_index = (self.sequence_index + 1) % len(self.sorting_sequence)
        return target_pose

    def process_tasks(self):
        """Processes pending sorting tasks if the robot is idle."""
        if self.executing_task or not self.task_queue:
            return
        
        self.executing_task = True
        pick_pose, place_pose = self.task_queue.popleft()
        
        if self.move_to_pose(pick_pose):
            self.get_logger().info("Pick successful, moving to place.")
            if self.move_to_pose(place_pose):
                self.get_logger().info("Sorting task completed.")
        self.executing_task = False

    def move_to_pose(self, pose: PoseStamped):
        """Plans and executes a Cartesian motion to the given pose."""
        self.move_group.set_pose_target(pose.pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success


def main():
    rclpy.init()
    node = cobot_control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
