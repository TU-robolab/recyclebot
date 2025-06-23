# launch/rec_bot_smoke.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    ur_pkg_share = get_package_share_directory("ur_moveit_config")

    return LaunchDescription([
        Node(
            package="recycle_bot",
            executable="rec_bot_smoke.py",
            name="smoke",
            output="screen",
            parameters=[
                # robot + kinematics
                os.path.join(ur_pkg_share, "config", "kinematics.yaml"),
                # planning-pipeline config
                os.path.join(get_package_share_directory("recycle_bot"), "config", "ompl_minimal.yaml"),
            ],
        ),
    ])