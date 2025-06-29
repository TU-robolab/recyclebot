# launch/rec_bot_smoke.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node, DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import os

def generate_launch_description():
    declare_type = DeclareLaunchArgument(
        'ur_type', default_value='ur16e',
        description='UR model (ur3e, ur5e, ur10e, ur16e …)'
    )

    # Build a dict of the exact MoveItCpp params for this robot
    moveit_cfg = (
        MoveItConfigsBuilder(package_name='ur_moveit_config', robot_name=ur_type)
        .moveit_cpp()        # generates moveit_cpp.yaml + plugin YAML
        .to_dict()           # returns a Python dict ready for Node.parameters
    )

    ur_type = LaunchConfiguration('ur_type') 
    ur_pkg_share = get_package_share_directory("ur_moveit_config")

    return LaunchDescription([
        Node(
            package="recycle_bot",
            executable="rec_bot_smoke",
            name="smoke",
            output="screen",
            parameters=[
                moveit_cfg,                     #   ←──── feeds MoveItPy
                {'ur_type': ur_type}            #   optional custom param
            ]            
            # parameters=[
            #     # robot + kinematics
            #     #os.path.join(ur_pkg_share, "config", "kinematics.yaml"),
            #     # planning-pipeline config
            #     os.path.join(get_package_share_directory("recycle_bot"), "config", "ompl_minimal.yaml"),
            # ],
        ),
    ])

