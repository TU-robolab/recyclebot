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
from tf_transformations import quaternion_from_euler
from image_geometry import PinholeCameraModel
from moveit.planning import MoveItPy, PlanningComponent
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.msg import Constraints, OrientationConstraint
from moveit_configs_utils import MoveItConfigsBuilder
from std_msgs.msg import Bool, String
from realsense2_camera_msgs.msg import RGBD

class cobot_control(Node):
    def __init__(self):
        super().__init__("cobot_control")
        
        self.robot_description = None
        self.create_subscription(
            String,
            "/robot_description",
            self.robot_description_callback,
            10
        )

        # Load sorting sequence from YAML file
        self.sorting_sequence = self.load_sorting_sequence()
        self.sequence_index = 0
        # implemented as thread-safe deque, for now we use FIFO
        self.task_queue = deque() 
        self.executing_task = False

        # MoveIt2 Interface
        # wait for robot description to be available
        while self.robot_description is None:
            self.get_logger().info("Waiting for robot description...")
            rclpy.spin_once(self, timeout_sec=1.0)

        moveit_config = MoveItConfigsBuilder("ur16e", package_name="ur_moveit_config")
        moveit_config.robot_description(self.robot_description)
        moveit_config.to_moveit_configs()

        self.moveit= MoveItPy(node_name="ur_manipulator", config_dict=moveit_config)
        self.arm = PlanningComponent("ur_manipulator", self.moveit)

        # TF2 transform Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # to be used for vision subscriber (detects object availability and poses)
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(PoseStamped, "/vision/detected_object", self.vision_callback, qos_profile)
        
        # Timer for checking and processing tasks
        self.create_timer(1.0, self.process_tasks)

        self.get_logger().info("UR16e sorter node initialized!")

    def robot_description_callback(self, msg):
        self.robot_description = msg.data

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

    def move_cartesian(self, waypoints):
        """Move end effector through Cartesian waypoints"""
        try:
            # Set start state to current state
            self.arm.set_start_state_to_current_state()
            
            # Create Cartesian constraints
            constraints = Constraints()
            ocm = OrientationConstraint()
            ocm.orientation = waypoints[0].orientation
            ocm.link_name = "tool0"
            ocm.absolute_x_axis_tolerance = 0.1
            ocm.absolute_y_axis_tolerance = 0.1
            ocm.absolute_z_axis_tolerance = 0.1
            ocm.weight = 1.0
            constraints.orientation_constraints.append(ocm)
            
            # Plan Cartesian path
            plan_result = self.arm.plan(
                goal_constraints=[constraints],
                cartesian=True,
                waypoints=waypoints,
                max_step=0.01,
                jump_threshold=0.0
            )
            
            if plan_result:
                self.get_logger().info("Executing Cartesian path")
                self.arm.execute()
            else:
                self.get_logger().error("Cartesian planning failed")

        except Exception as e:
            self.get_logger().error(f"Error in Cartesian motion: {str(e)}")

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

def create_waypoint_pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    """Create Pose message with Euler angles"""
    q = quaternion_from_euler(roll, pitch, yaw)
    return Pose(
        position=Point(x=x, y=y, z=z),
        orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    )

def main():
    rclpy.init()

    # Create Cartesian waypoints
    waypoints = [
        create_waypoint_pose(0.4, 0.2, 0.5),  # Home position
        create_waypoint_pose(0.5, 0.2, 0.5),  # X+0.1
        create_waypoint_pose(0.5, 0.3, 0.5),  # Y+0.1
        create_waypoint_pose(0.5, 0.3, 0.6)   # Z+0.1
    ]

    ur_node = cobot_control()

    ur_node.move_cartesian(waypoints)

    try:
        rclpy.spin(ur_node)
    except KeyboardInterrupt:
        pass

    ur_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
