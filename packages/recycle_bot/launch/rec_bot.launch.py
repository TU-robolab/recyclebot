import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur16e", package_name="ur16e_moveit_config"
        )
        .robot_description(file_path="config/ur16e.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("ur16e_moveit_config"),
                "config",
                "moveit_cpp.yaml",
            )
        )
        .to_moveit_configs()
    )

    moveit_exec_file = DeclareLaunchArgument(
        "moveit_exec_file",
        default_value="rec_bot_control",
        description="Python executable to run from recycle_bot package",
    )

    moveit_py_node = Node(
        name="moveit_py",
        package="recycle_bot",
        executable=LaunchConfiguration("moveit_exec_file"),
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    realsense_node = Node(
        package="realsense2_camera",
        executable="rs_launch.py",
        namespace="camera",
        parameters=[{"pointcloud.enable": True}],
    )

    grip_command_launch = Node(
        package="grip_command_package",
        executable="master.launch.py",
        output="both",
    )

    return LaunchDescription(
        [
            moveit_exec_file,
            moveit_py_node,
            realsense_node,
            grip_command_launch,
        ]
    )

