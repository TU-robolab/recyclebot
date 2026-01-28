# System Imports
import yaml
import os
import math

from collections import deque
from threading import Event

# ROS2 imports
import rclpy
import rclpy.duration
import tf2_ros

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from tf_transformations import quaternion_from_euler
from image_geometry import PinholeCameraModel
from moveit.planning import MoveItPy, PlanningSceneMonitor
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.msg import Constraints, OrientationConstraint, CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from moveit_configs_utils import MoveItConfigsBuilder
from std_msgs.msg import Bool, String
from grip_interface.srv import GripCommand


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
        
        # load config from YAML file
        self.sorting_sequence, self.neutral_pose, self.cycle = self.load_config()
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

        # add collision objects to planning scene
        self.setup_collision_objects()

        # TF2 transform Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # timeout configuration (seconds)
        self.tf_timeout_sec = 1.0
        self.planning_timeout_sec = 10.0

        # to be used for vision subscriber (detects object availability and poses)
        # use RELIABLE to match rec_bot_core publisher
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=10)
        self.create_subscription(PoseStamped, "/vision/detected_object", self.vision_callback, qos_profile)

        # gripper service client
        self.gripper_client = self.create_client(GripCommand, '/gripper_action')
        self.gripper_timeout_sec = 5.0

        # Timer for checking and processing tasks
        self.create_timer(1.0, self.process_tasks)

        self.get_logger().info("UR16e sorter node initialized!")

    def robot_description_callback(self, msg):
        self.robot_description = msg.data

    def setup_collision_objects(
        self,
        table_size=(1.2, 0.8, 0.05),
        table_position=(0.0, 0.0, -0.025),
        camera_size=(0.10, 0.03, 0.03),
        camera_position=(-0.384, 0.286, 0.624)
    ):
        """
        Add collision objects to planning scene for safe motion planning.

        Args:
            table_size: (x, y, z) dimensions in meters, default 1.2x0.8x0.05m
            table_position: (x, y, z) center position relative to base_link
            camera_size: (x, y, z) dimensions in meters, default ~D415 with margin
            camera_position: (x, y, z) position from rec_bot_core.py static transform

        Collision geometry:
        - table: box underneath robot base where UR16e is mounted
        - camera: box at camera mount position (RealSense D415)
        """
        planning_scene_monitor = self.moveit.get_planning_scene_monitor()

        with planning_scene_monitor.read_write() as scene:
            # table collision object
            #
            #   top view:
            #        ┌─────────────────────┐
            #        │                     │
            #        │    table (1.2m)     │
            #        │         ·──────────── UR base at center
            #        │                     │
            #        └─────────────────────┘
            #              0.8m
            #
            #   side view:
            #        ════════════ base_link (z=0)
            #        ┌──────────┐
            #        │  table   │ 0.05m thick
            #        └──────────┘ z = -0.025m (center)
            #
            table = CollisionObject()
            table.header.frame_id = "base_link"
            table.header.stamp = self.get_clock().now().to_msg()
            table.id = "table"
            table.operation = CollisionObject.ADD

            table_box = SolidPrimitive()
            table_box.type = SolidPrimitive.BOX
            table_box.dimensions = list(table_size)

            table_pose = Pose()
            table_pose.position.x = table_position[0]
            table_pose.position.y = table_position[1]
            table_pose.position.z = table_position[2]
            table_pose.orientation.w = 1.0

            table.primitives.append(table_box)
            table.primitive_poses.append(table_pose)

            scene.apply_collision_object(table)
            self.get_logger().info("Added table collision object")

            # camera collision object (RealSense D415: ~99mm x 25mm x 25mm)
            #
            #   side view:
            #                    ┌───┐ camera
            #                    │   │
            #        ────────────┼───┼──────── z = 0.624m
            #                    │   │
            #                    └───┘
            #                      │
            #        ═════════════╧════════════ base_link
            #              x = -0.384m
            #
            camera = CollisionObject()
            camera.header.frame_id = "base_link"
            camera.header.stamp = self.get_clock().now().to_msg()
            camera.id = "camera"
            camera.operation = CollisionObject.ADD

            camera_box = SolidPrimitive()
            camera_box.type = SolidPrimitive.BOX
            camera_box.dimensions = list(camera_size)

            camera_pose = Pose()
            camera_pose.position.x = camera_position[0]
            camera_pose.position.y = camera_position[1]
            camera_pose.position.z = camera_position[2]
            camera_pose.orientation.w = 1.0

            camera.primitives.append(camera_box)
            camera.primitive_poses.append(camera_pose)

            scene.apply_collision_object(camera)
            self.get_logger().info("Added camera collision object")

    def load_config(self):
        """Load sorting sequence, neutral pose, and cycle setting from YAML config."""
        yaml_path = os.path.join(os.path.dirname(__file__), "sorting_sequence.yaml")
        try:
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)

            sorting_sequence = data.get("sorting_sequence", [])
            cycle = data.get("cycle", True)  # default to cycling for backwards compatibility
            self.approach_height_m = float(data.get("approach_height_m", 0.10))
            size = data.get("grasped_object_size", [0.10, 0.10, 0.10])
            self.grasped_object_size = [float(size[0]), float(size[1]), float(size[2])]

            # load neutral pose
            neutral_data = data.get("neutral_pose", None)
            neutral_pose = None
            if neutral_data:
                neutral_pose = self.create_pose_from_dict(neutral_data)
                self.get_logger().info("Neutral pose loaded from config")
            else:
                self.get_logger().warn("No neutral_pose in config, skipping neutral movements")

            return sorting_sequence, neutral_pose, cycle

        except Exception as e:
            self.get_logger().error(f"Failed to load YAML: {e}")
            self.approach_height_m = 0.10
            self.grasped_object_size = [0.10, 0.10, 0.10]
            return [], None, True

    def create_pose_from_dict(self, pose_dict):
        """Create PoseStamped from dict with position and orientation keys."""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"

        pose.pose.position.x = float(pose_dict["position"][0])
        pose.pose.position.y = float(pose_dict["position"][1])
        pose.pose.position.z = float(pose_dict["position"][2])

        pose.pose.orientation.x = float(pose_dict["orientation"][0])
        pose.pose.orientation.y = float(pose_dict["orientation"][1])
        pose.pose.orientation.z = float(pose_dict["orientation"][2])
        pose.pose.orientation.w = float(pose_dict["orientation"][3])

        return pose

    def vision_callback(self, msg: PoseStamped):
        """Handles incoming object pose from the vision system and queues it."""
        try:
            # convert from camera frame to robot base_link reference
            transformed_pose = self.tf_buffer.transform(
                msg,
                "base_link",
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec),
            )

            # retrieve target bin location (base-link reference)
            target_pose = self.get_next_sorting_pose()

            if target_pose is None:
                self.get_logger().error("Cannot queue task: no sorting sequence configured")
                return

            # add sorting task to FIFO queue
            self.task_queue.append((transformed_pose, target_pose))
            self.get_logger().info("Queued new sorting task.")
        except tf2_ros.LookupException:
            self.get_logger().warn(f"TF lookup failed: frame '{msg.header.frame_id}' not found")
        except tf2_ros.ExtrapolationException:
            self.get_logger().warn("TF extrapolation error: transform not available yet")
        except Exception as e:
            self.get_logger().warn(f"Failed to process detected object: {e}")

    def get_next_sorting_pose(self):
        """Returns the next target pose from the predefined sorting sequence, cycles if configured."""
        if not self.sorting_sequence:
            self.get_logger().error("No sorting sequence available!")
            return None

        if self.sequence_index >= len(self.sorting_sequence):
            if self.cycle:
                self.sequence_index = 0  # wrap around
            else:
                self.get_logger().warn("Sorting sequence exhausted and cycle=false")
                return None

        target_pose_obj = self.sorting_sequence[self.sequence_index]
        self.sequence_index += 1

        # convert from return YAML value into posetamped datatype
        return self.create_pose(target_pose_obj)

    def gripper_action(self, action: str) -> bool:
        """
        Call gripper service to grip or release.

        Args:
            action: "grip" or "release"

        Returns:
            True if successful, False otherwise
        """
        if not self.gripper_client.wait_for_service(timeout_sec=self.gripper_timeout_sec):
            self.get_logger().error("Gripper service not available")
            return False

        request = GripCommand.Request()
        request.action = action

        # Avoid spinning here so other callbacks (TF, vision, timers) keep running.
        future = self.gripper_client.call_async(request)
        done_event = Event()
        future.add_done_callback(lambda _fut: done_event.set())

        if not done_event.wait(timeout=self.gripper_timeout_sec):
            self.get_logger().error(f"Gripper {action} call timed out")
            return False

        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Gripper {action} call failed: {exc}")
            return False

        if result is None:
            self.get_logger().error(f"Gripper {action} call returned no result")
            return False
        if result.success:
            self.get_logger().info(f"Gripper {action}: {result.message}")
        else:
            self.get_logger().error(f"Gripper {action} failed: {result.message}")

        return result.success

    def move_to_neutral(self) -> bool:
        """Move to neutral pose if configured."""
        if self.neutral_pose is None:
            return True  # skip if not configured

        # refresh timestamp
        self.neutral_pose.header.stamp = self.get_clock().now().to_msg()

        if not self.move_to_pose(self.neutral_pose):
            self.get_logger().error("Failed to reach neutral pose")
            return False
        return True

    def process_tasks(self):
        """
        Processes pending sorting tasks if the robot is idle.

        Task sequence:
            1. move to neutral (safe start)
            2. move to pre-pick pose (approach)
            3. move to pick pose
            4. grip object
            5. move to neutral (safe transit with object)
            6. move to pre-place pose (approach)
            7. move to place pose
            8. release object
            9. move to neutral (safe end)

                 neutral ←──────────────────────┐
                    │                           │
                    ▼                           │
                  pre-pick ─► pick ─► grip ─► neutral ─► pre-place ─► place ─► release
        """
        if self.executing_task or not self.task_queue:
            return

        self.executing_task = True
        pick_pose, place_pose = self.task_queue.popleft()

        # 1. start from neutral
        if not self.move_to_neutral():
            self.get_logger().error("Failed to reach neutral, aborting task")
            self.executing_task = False
            return

        # 2. move to pre-pick (approach)
        pre_pick = self.offset_pose_z(pick_pose, self.approach_height_m)
        if pre_pick is not None and not self.move_to_pose(pre_pick):
            self.get_logger().error("Failed to reach pre-pick pose, returning to neutral")
            self.move_to_neutral()
            self.executing_task = False
            return

        # 3. move to pick
        if not self.move_to_pose(pick_pose):
            self.get_logger().error("Failed to reach pick pose, returning to neutral")
            self.move_to_neutral()
            self.executing_task = False
            return

        # 4. grip
        if not self.gripper_action("grip"):
            self.get_logger().error("Failed to grip object, returning to neutral")
            self.move_to_neutral()
            self.executing_task = False
            return
        self.attach_object_to_tool()

        self.get_logger().info("Object gripped, lifting to neutral")

        # 4. lift to neutral (safe transit with object)
        if not self.move_to_neutral():
            self.get_logger().error("Failed to lift to neutral, releasing object")
            self.gripper_action("release")
            self.executing_task = False
            return

        # 6. move to pre-place (approach)
        pre_place = self.offset_pose_z(place_pose, self.approach_height_m)
        if pre_place is not None and not self.move_to_pose(pre_place):
            self.get_logger().error("Failed to reach pre-place pose, releasing and returning to neutral")
            self.gripper_action("release")
            self.move_to_neutral()
            self.executing_task = False
            return

        # 7. move to place
        if not self.move_to_pose(place_pose):
            self.get_logger().error("Failed to reach place pose, releasing and returning to neutral")
            self.gripper_action("release")
            self.move_to_neutral()
            self.executing_task = False
            return

        # 8. release
        if not self.gripper_action("release"):
            self.get_logger().warn("Failed to release object")
        else:
            self.detach_object_from_tool()

        self.get_logger().info("Object released, returning to neutral")

        # 9. return to neutral
        self.move_to_neutral()

        self.get_logger().info("Sorting task completed")
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

    def move_to_pose(self, pose: PoseStamped) -> bool:
        """
        Plans and executes motion to the given pose.

        Note: planning timeout is configured in OMPL settings (moveit_cpp.yaml)
        """
        try:
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

        except Exception as e:
            self.get_logger().error(f"Motion planning error: {e}")
            return False

    def create_pose(self, location, element_idx=0):
        """ 
         element_index tells you which element you would like
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
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
        
        if isinstance(location, dict):
            location_name = next(iter(location.keys()))
            location_data = location[location_name]
        else:
            location_name = list(location[element_idx].keys())[0]
            location_data = location[element_idx][location_name]

        # position coordinates
        pose.pose.position.x = location_data["position"][0]
        pose.pose.position.y = location_data["position"][1]
        pose.pose.position.z = location_data["position"][2]
        
        # orientation quaternion 
        pose.pose.orientation.x = location_data["orientation"][0]
        pose.pose.orientation.y = location_data["orientation"][1]
        pose.pose.orientation.z = location_data["orientation"][2]
        pose.pose.orientation.w = location_data["orientation"][3]
        
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

    def attach_object_to_tool(self):
        """Attach a simple collision object to the tool for safer planning."""
        try:
            aco = AttachedCollisionObject()
            aco.link_name = "tool0"
            aco.touch_links = ["tool0"]

            aco.object = CollisionObject()
            aco.object.id = "grasped_object"
            aco.object.header.frame_id = "tool0"
            aco.object.operation = CollisionObject.ADD

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = list(self.grasped_object_size)

            box_pose = Pose()
            box_pose.position.z = self.grasped_object_size[2] / 2.0
            box_pose.orientation.w = 1.0

            aco.object.primitives.append(box)
            aco.object.primitive_poses.append(box_pose)

            planning_scene_monitor = self.moveit.get_planning_scene_monitor()
            with planning_scene_monitor.read_write() as scene:
                scene.apply_attached_collision_object(aco)
        except Exception as exc:
            self.get_logger().warn(f"Failed to attach collision object: {exc}")

    def detach_object_from_tool(self):
        """Detach the collision object from the tool."""
        try:
            planning_scene_monitor = self.moveit.get_planning_scene_monitor()
            with planning_scene_monitor.read_write() as scene:
                scene.remove_attached_collision_object("grasped_object")
        except Exception as exc:
            self.get_logger().warn(f"Failed to detach collision object: {exc}")

    def offset_pose_z(self, pose_stamped: PoseStamped, dz: float):
        """Return a PoseStamped offset in Z by dz (meters)."""
        if pose_stamped is None:
            return None
        offset_pose = PoseStamped()
        offset_pose.header.frame_id = pose_stamped.header.frame_id or "base_link"
        offset_pose.header.stamp = self.get_clock().now().to_msg()
        offset_pose.pose.position = pose_stamped.pose.position
        offset_pose.pose.orientation = pose_stamped.pose.orientation
        offset_pose.pose.position.z += dz
        return offset_pose

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

def create_waypoint_pose(x,y,z,roll=None,pitch=None,yaw=None,quaternion=None):
    """Create Pose message with either Euler angles or quaternion orientation."""
    if quaternion is not None and any(value is not None for value in (roll, pitch, yaw)):
        raise ValueError("Provide either Euler angles or a quaternion, not both")
    
    pose = Pose(position=Point(x=x, y=y, z=z))
    
    if quaternion is not None:
        qx, qy, qz, qw = quaternion
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 0.0:
            raise ValueError("Invalid quaternion (norm=0)")
        pose.orientation = Quaternion(
            x=qx / norm,
            y=qy / norm,
            z=qz / norm,
            w=qw / norm,
        )
    else:
        roll = 0.0 if roll is None else roll
        pitch = 0.0 if pitch is None else pitch
        yaw = 0.0 if yaw is None else yaw
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose

def main():
    rclpy.init()

    ur_node = cobot_control()

    pose = create_waypoint_pose(
        x=0.116,
        y=-0.468,
        z=0.874,
        quaternion=(0.330, -0.646, 0.606, 0.324),
    )

    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link"
    target_pose.header.stamp = ur_node.get_clock().now().to_msg()
    target_pose.pose = pose
    target_pose = ur_node.normalize_pose_stamped(target_pose)
    
    if ur_node.move_to_pose(target_pose):
        ur_node.get_logger().info("Target pose executed")

    executor = MultiThreadedExecutor()

    try:
        executor.add_node(ur_node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ur_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
