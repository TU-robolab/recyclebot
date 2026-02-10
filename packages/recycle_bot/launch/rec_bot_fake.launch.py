import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
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
    # 1. UR Robot Driver (mock hardware)
    # =========================================================================
    ur_robot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ur_robot_driver"), "/launch/ur_control.launch.py"
        ]),
        launch_arguments={
            "ur_type": "ur16e",
            "robot_ip": "192.168.1.102",
            "use_mock_hardware": "true",
            "mock_sensor_commands": "true",
            "launch_rviz": "false",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
        }.items()
    )

    # =========================================================================
    # 2. Fake RGBD Publisher (simulates RealSense camera)
    # =========================================================================
    fake_camera_node = Node(
        package="test_suite",
        executable="fake_rgbd_publisher",
        name="fake_rgbd_publisher",
        output="screen",
    )

    # =========================================================================
    # 3. Vision Detection Node (YOLO)
    # =========================================================================
    vision_node = Node(
        package="recycle_bot",
        executable="rec_bot_vision",
        name="rec_bot_vision",
        output="screen",
    )

    # =========================================================================
    # 4. Core Processing Node (3D projection + TF)
    # =========================================================================
    core_node = Node(
        package="recycle_bot",
        executable="rec_bot_core",
        name="rec_bot_core",
        output="screen",
    )

    # =========================================================================
    # 5. Control Node (MoveIt planning + execution)
    # =========================================================================
    control_node = Node(
        name="moveit_py",
        package="recycle_bot",
        executable="rec_bot_control",
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    # =========================================================================
    # 6. Mock Gripper Service
    # =========================================================================
    mock_gripper_node = Node(
        package="test_suite",
        executable="mock_gripper_service",
        name="mock_gripper_service",
        output="screen",
    )

    return LaunchDescription(
        [
            ur_robot_driver_launch,
            fake_camera_node,
            vision_node,
            core_node,
            control_node,
            mock_gripper_node,
        ]
    )
