# System Imports
import yaml
import os

from collections import deque

# ROS2 imports
import rclpy
import tf2_ros


from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from moveit_commander import MoveGroupCommander

class cobot_control(Node):
    def __init__(self):
        super().__init__("cobot_control")
        
        # Load sorting sequence from YAML file
        self.sorting_sequence = self.load_sorting_sequence()
        self.sequence_index = 0
        # implemented as thread-safe deque, for now we use FIFO
        self.task_queue = deque() 
        self.executing_task = False

        # MoveIt2 Interface
        self.move_group = MoveGroupCommander("manipulator")
        
        # TF2 transform Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # to be used for vision subscriber (detects object availability and poses)
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(PoseStamped, "/vision/detected_object", self.vision_callback, qos_profile)
        
        # Timer for checking and processing tasks
        self.create_timer(1.0, self.process_tasks)

        self.get_logger().info("UR16e sorter node initialized!")

    def load_sorting_sequence(self):
        yaml_path = os.path.join(os.path.dirname(__file__), "sorting_sequence.yaml")
        try:
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                return data.get("sorting_sequence", [])
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML: {e}")
            return []

    def vision_callback(self, msg: PoseStamped):
        """Handles incoming object pose from the vision system and queues it."""
        try:
            # convert from camera transform to robot base-link reference
            transform = self.tf_buffer.lookup_transform("base_link", msg.header.frame_id, rclpy.time.Time())
            transformed_pose = tf2_ros.do_transform_pose(msg, transform)

            # retrieve target bin location (base-link reference)
            target_pose = self.get_next_sorting_pose()

            # add sorting task to FIFO queue
            self.task_queue.append((transformed_pose, target_pose))
            self.get_logger().info("Queued new sorting task.")
        except Exception as e:
            self.get_logger().warn(f"Not able to process detected trash: {e}")

    def get_next_sorting_pose(self):
        """Returns the next target pose from the predefined sorting sequence, cycles sequence if needed."""
        if not self.sorting_sequence:
            self.get_logger().error("No sorting sequence available!")
            return None
        
        target_pose = self.sorting_sequence[self.sequence_index]
        self.sequence_index = (self.sequence_index + 1) % len(self.sorting_sequence)
        
        # TODO convert from return YAML value into posetamped datatype
        return target_pose

    def process_tasks(self):
        """Processes pending sorting tasks if the robot is idle."""
        if self.executing_task or not self.task_queue:
            return
        
        self.executing_task = True
        # each task execution goes from pick -> neutral -> place
        """ neutral_pose = self.create_pose(
            [
                {"target_bin_1": {"position": [393.43, -247.56, 1.24], "orientation": [0.187876, -0.6345103, -0.7318231, 0.1628935]}}
            ]
        )
        """
        pick_pose, place_pose = self.task_queue.popleft() # FIFO order
        

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

    def print_current_status(self):
        """queries and prints robot"s state"""
        current_pose = self.move_group.get_current_pose().pose
        print("Current End-Effector Pose:", current_pose)
        
    def create_pose(self, location , element_idx=0):
        """ 
         element_index tells you whcih element you would like
         location based on format from YAML file, eg for current YAML, input would be:
         [
            {
                "target_bin_1": {
                    "position": [393.43, -247.56, 1.24],
                    "orientation": [0.187876, -0.6345103, -0.7318231, 0.1628935]
                }
            }
         ]
        """

        pose = PoseStamped()
        
        # Header configuration
        pose.header.stamp = rclpy.Time.now()
        pose.header.frame_id = "base_link"
        
        location_name = list(location[element_idx].keys())[0]
        # Position coordinates
        pose.pose.position.x = location[location_name]["position"][0]
        pose.pose.position.y = location[location_name]["position"][1]
        pose.pose.position.z = location[location_name]["position"][2]
        
        # Orientation quaternion 
        pose.pose.orientation.x = location[location_name]["orientation"][0]
        pose.pose.orientation.y = location[location_name]["orientation"][1]
        pose.pose.orientation.z = location[location_name]["orientation"][2]
        pose.pose.orientation.w = location[location_name]["orientation"][3]
        
        return pose

def main():
    rclpy.init()
    ur_node = cobot_control()
    try:
        rclpy.spin(ur_node)
    except KeyboardInterrupt:
        pass

    ur_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
