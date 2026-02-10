import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
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

    # =========================================================================
    # 1. UR Robot Driver (real hardware)
    # =========================================================================
    ur_robot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ur_robot_driver"), "/launch/ur_control.launch.py"
        ]),
        launch_arguments={
            "ur_type": "ur16e",
            "robot_ip": "192.168.1.102",
            "kinematics_params_file": os.path.join(
                get_package_share_directory("recycle_bot"),
                "config",
                "my_robot_calibration.yaml",
            ),
            "launch_rviz": "false",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
        }.items()
    )

    # =========================================================================
    # 2. Vision Detection Node (YOLO)
    # =========================================================================
    vision_node = Node(
        package="recycle_bot",
        executable="rec_bot_vision",
        name="rec_bot_vision",
        output="screen",
    )

    # =========================================================================
    # 3. Core Processing Node (3D projection + TF)
    # =========================================================================
    core_node = Node(
        package="recycle_bot",
        executable="rec_bot_core",
        name="rec_bot_core",
        output="screen",
    )

    # =========================================================================
    # 4. Control Node (MoveIt planning + execution)
    # =========================================================================
    control_node = Node(
        name="moveit_py",
        package="recycle_bot",
        executable="rec_bot_control",
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    # =========================================================================
    # 5. RealSense Camera
    # =========================================================================
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={"pointcloud.enable": "true"}.items(),
    )

    # =========================================================================
    # 6. Gripper
    # =========================================================================
    grip_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("grip_command_package"),
                "launch",
                "master.launch.py",
            )
        )
    )

    return LaunchDescription(
        [
            ur_robot_driver_launch,
            realsense_launch,
            vision_node,
            core_node,
            control_node,
            grip_launch,
        ]
    )
