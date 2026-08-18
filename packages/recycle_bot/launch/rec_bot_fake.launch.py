import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from recycle_bot.robot_profile import (
    DEFAULT_UR_TYPE,
    build_moveit_config,
    kinematics_params_file,
    resolve_ur_type,
)


def launch_setup(context, *args, **kwargs):
    """Build the launch actions once ur_type is a concrete string.

    See rec_bot.launch.py for why this needs OpaqueFunction rather than a plain
    LaunchConfiguration.
    """
    ur_type = resolve_ur_type(LaunchConfiguration("ur_type").perform(context))
    moveit_config = build_moveit_config(ur_type)

    # =========================================================================
    # 0. Kill leftover ROS processes to avoid controller conflicts
    # =========================================================================
    cleanup = ExecuteProcess(
        cmd=["bash", "-c",
             "pkill -INT -f 'ros2_control_node|controller_manager' 2>/dev/null; sleep 1; echo '[cleanup] Done'"],
        output="screen",
    )

    # =========================================================================
    # 1. UR Robot Driver (mock hardware)
    # =========================================================================
    ur_robot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ur_robot_driver"), "/launch/ur_control.launch.py"
        ]),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": os.environ.get("REMOTE_IP", "192.168.1.102"),  # unused with mock hardware
            # Nominal ur_description kinematics are fine here: mock hardware has
            # no real arm whose calibration could differ from nominal.
            **({"kinematics_params_file": kinematics_params_file(ur_type)}
               if kinematics_params_file(ur_type) else {}),
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
        parameters=[{"ur_type": ur_type}],
    )

    # =========================================================================
    # 4. Core Processing Node (3D projection + TF)
    # =========================================================================
    core_node = Node(
        package="recycle_bot",
        executable="rec_bot_core",
        name="rec_bot_core",
        output="screen",
        parameters=[{"ur_type": ur_type}],
    )

    # =========================================================================
    # 5. Control Node (MoveIt planning + execution)
    # =========================================================================
    control_node = Node(
        name="moveit_py",
        package="recycle_bot",
        executable="rec_bot_control",
        output="both",
        parameters=[moveit_config.to_dict(), {"ur_type": ur_type}],
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

    # =========================================================================
    # 7. Detection Visualization Node
    # =========================================================================
    viz_node = Node(
        package="recycle_bot",
        executable="rec_bot_viz",
        name="rec_bot_viz",
        output="screen",
        parameters=[{"ur_type": ur_type}],
    )

    # Start everything else only after cleanup finishes
    start_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[
                ur_robot_driver_launch,
                fake_camera_node,
                vision_node,
                core_node,
                control_node,
                mock_gripper_node,
                viz_node,
            ],
        )
    )

    return [cleanup, start_after_cleanup]


def generate_launch_description():
    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value=DEFAULT_UR_TYPE,
        description="Which UR arm to simulate (ur16e, ur3e). Selects the MoveIt "
                    "config and the recycle_bot config/<ur_type>/ directory.",
    )
    return LaunchDescription([ur_type_arg, OpaqueFunction(function=launch_setup)])
