# System Imports
import yaml
import os
import math

from collections import deque

# ROS2 imports
import rclpy
import tf2_ros



from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf_transformations import quaternion_from_euler
from image_geometry import PinholeCameraModel
from moveit.planning import MoveItPy
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.msg import Constraints, OrientationConstraint
from moveit_configs_utils import MoveItConfigsBuilder
from std_msgs.msg import Bool, String


class cobot_control(Node):
    def __init__(self):
        super().__init__("cobot_control")
        
        # self.robot_description = None
        # self.create_subscription(
        #     String,
        #     "/robot_description",
        #     self.robot_description_callback,
        #     10
        # )

        # setup ROS quality of service for moves
        # qos_moves_objects = QoSProfile(
        #     history=HistoryPolicy.KEEP_LAST,  # store recent messages
        #     depth=10,  # buffer up to 10 movements
        #     reliability=ReliabilityPolicy.RELIABLE,  # ensure all detections arrive
        #     durability=DurabilityPolicy.VOLATILE  # no need to retain past detections
        # )
        
        # Load sorting sequence from YAML file
        self.sorting_sequence = self.load_sorting_sequence()
        self.sequence_index = 0
        # implemented as thread-safe deque, for now we use FIFO
        self.task_queue = deque() 
        self.executing_task = False

        # MoveIt2 Interface
        # wait for robot description to be available
        # while self.robot_description is None:
        #     self.get_logger().info("Waiting for robot description...")
        #     rclpy.spin_once(self, timeout_sec=1.0)

        # moveit_config = MoveItConfigsBuilder("ur16e", package_name="ur_moveit_config")
        # tmp_yaml_path = os.path.join(os.path.expanduser("~"), "ros2_ws/src/recycle_bot/pkg_resources", "moveit_params.yaml" )

        # #moveit_config.robot_description(self.robot_description)
        # moveit_config.moveit_cpp(tmp_yaml_path)
        # moveit_config.to_moveit_configs()

        self.moveit = MoveItPy(node_name="ur_moveit")
        self.arm = self.moveit.get_planning_component("ur_arm")
        self.velocity_scaling = 0.2
        self.acceleration_scaling = 0.2

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
        
        target_pose_obj = self.sorting_sequence[self.sequence_index]
        self.sequence_index = (self.sequence_index + 1) % len(self.sorting_sequence)
        
        # convert from return YAML value into posetamped datatype
        return self.create_pose(target_pose_obj)

    def process_tasks(self):
        """Processes pending sorting tasks if the robot is idle."""
        if self.executing_task or not self.task_queue:
            return
        
        self.executing_task = True
        # each task execution goes from pick -> neutral -> place
        """ neutral_pose = self.create_pose(

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
            normalized_waypoints = [self.normalize_pose_orientation(p) for p in waypoints]

            # Create Cartesian constraints
            constraints = Constraints()
            ocm = OrientationConstraint()
            ocm.orientation = normalized_waypoints[0].orientation
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
                waypoints=normalized_waypoints,
                max_step=0.01,
                jump_threshold=0.0
            )

            if not plan_result or not getattr(plan_result, "success", False):
                status = getattr(plan_result, "status", "unknown")
                self.get_logger().error(f"Cartesian planning failed: {status}")
                return False

            self.get_logger().info("Executing Cartesian path")
            self.execute_trajectory(plan_result.trajectory)
            return True
        except Exception as e:
            self.get_logger().error(f"Error in Cartesian motion: {str(e)}")
            return False

    def move_to_pose(self, pose: PoseStamped):
        """Plans and executes a Cartesian motion to the given pose."""
        pose_to_plan = self.normalize_pose_stamped(pose)

        if pose_to_plan is None:
            return False

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=pose_to_plan, pose_link="tool0")
        plan_result = self.arm.plan()

        if not plan_result or not getattr(plan_result, "success", False):
            status = getattr(plan_result, "status", "unknown")
            self.get_logger().error(f"Planning failed: {status}")
            return False

        self.get_logger().info("Executing planned pose")
        self.execute_trajectory(plan_result.trajectory)
        return True

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
        
        # header configuration
        pose.header.stamp = rclpy.Time.now()
        pose.header.frame_id = "base_link"
        
        location_name = list(location[element_idx].keys())[0]
        # position coordinates
        pose.pose.position.x = location[element_idx][location_name]["position"][0]
        pose.pose.position.y = location[element_idx][location_name]["position"][1]
        pose.pose.position.z = location[element_idx][location_name]["position"][2]
        
        # orientation quaternion 
        pose.pose.orientation.x = location[element_idx][location_name]["orientation"][0]
        pose.pose.orientation.y = location[element_idx][location_name]["orientation"][1]
        pose.pose.orientation.z = location[element_idx][location_name]["orientation"][2]
        pose.pose.orientation.w = location[element_idx][location_name]["orientation"][3]
        
        return pose

    def execute_trajectory(self, trajectory):
        """Apply TOTG time parameterization and execute trajectory using scaled controller."""
        trajectory_retimed = trajectory.apply_totg_time_parameterization(
            velocity_scaling_factor=self.velocity_scaling,
            acceleration_scaling_factor=self.acceleration_scaling
        )

        if not trajectory_retimed:
            self.get_logger().warn("Time parameterization failed, executing raw trajectory")

        self.moveit.execute(trajectory, controllers=["scaled_joint_trajectory_controller"])

    def normalize_pose_orientation(self, pose):
        """Return a Pose with normalized quaternion orientation."""
        orientation = pose.orientation
        norm = math.sqrt(
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )

        if norm <= 0.0:
            raise ValueError("Invalid quaternion (norm=0)")

        pose.orientation.x = orientation.x / norm
        pose.orientation.y = orientation.y / norm
        pose.orientation.z = orientation.z / norm
        pose.orientation.w = orientation.w / norm
        return pose

    def normalize_pose_stamped(self, pose_stamped: PoseStamped):
        """Return a PoseStamped with normalized orientation, logging if invalid."""
        normalized_pose = PoseStamped()
        normalized_pose.header.frame_id = pose_stamped.header.frame_id or "base_link"
        normalized_pose.header.stamp = self.get_clock().now().to_msg()

        normalized_pose.pose.position = pose_stamped.pose.position
        normalized_pose.pose.orientation = pose_stamped.pose.orientation

        try:
            normalized_pose.pose = self.normalize_pose_orientation(normalized_pose.pose)
            return normalized_pose
        except ValueError as exc:
            self.get_logger().error(f"{exc}")
            return None

def create_waypoint_pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    """Create Pose message with Euler angles"""
    q = quaternion_from_euler(roll, pitch, yaw)
    return Pose(
        position=Point(x=x, y=y, z=z),
        orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    )

def main():
    rclpy.init()

    ur_node = cobot_control()

    target_pose = PoseStamped()
    target_pose.header.frame_id = "base"
    target_pose.header.stamp = ur_node.get_clock().now().to_msg()
    target_pose.pose.position.x = -0.38384216583910713
    target_pose.pose.position.y = 0.2863018787024152
    target_pose.pose.position.z = 0.6239699702309053
    target_pose.pose.orientation.x = -0.9989747131951828
    target_pose.pose.orientation.y = 0.04527164772325851
    target_pose.pose.orientation.z = -1.2163534954626416e-05
    target_pose.pose.orientation.w = 1.2691403055540589e-05

    if ur_node.move_to_pose(target_pose):
        ur_node.get_logger().info("Target pose executed")

    try:
        rclpy.spin(ur_node)
    except KeyboardInterrupt:
        pass

    ur_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
