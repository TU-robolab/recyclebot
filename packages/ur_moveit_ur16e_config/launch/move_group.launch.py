from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os

#from moveit_configs_utils.launch_utils import generate_move_group_launch
#from moveit_configs_utils.launch_utils import load_yaml
import yaml
import launch

yaml_path = "/home/ur16e/ros2_ws/src/ur_moveit_ur16e_config/config/ompl_planning.yaml"

ompl_config = yaml.safe_load(open(yaml_path, 'r'))



def generate_launch_description():
    # Create MoveIt config object for the UR16e robot
    moveit_config = MoveItConfigsBuilder("ur16e", package_name="ur_moveit_ur16e_config").to_moveit_configs()

    # Optional: print the planning pipelines for debugging
    print(moveit_config.planning_pipelines)

    # Create the Move Group launch description using the MoveIt config
    #move_group_ld = generate_move_group_launch(moveit_config)

    # Add nodes manually if needed (example placeholder)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        ompl_config,
        moveit_config.joint_limits,
        ]
    )

   
    return launch.LaunchDescription([move_group_node])
