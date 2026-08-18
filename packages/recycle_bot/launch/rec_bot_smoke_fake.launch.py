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
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from recycle_bot.robot_profile import (
    DEFAULT_UR_TYPE,
    build_moveit_config,
    kinematics_params_file,
    resolve_ur_type,
)


def launch_setup(context, *args, **kwargs):
    """Build the launch actions once ur_type is a concrete string.

    See rec_bot.launch.py for why this needs OpaqueFunction.
    """
    ur_type = resolve_ur_type(LaunchConfiguration("ur_type").perform(context))
    moveit_config = build_moveit_config(ur_type)

    # Kill leftover ROS processes to avoid controller conflicts
    cleanup = ExecuteProcess(
        cmd=["bash", "-c",
             "pkill -INT -f 'ros2_control_node|controller_manager' 2>/dev/null; sleep 1; echo '[cleanup] Done'"],
        output="screen",
    )

    # UR Robot Driver with virtual robot (fake hardware)
    ur_robot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ur_robot_driver"), "/launch/ur_control.launch.py"
        ]),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": os.environ.get("REMOTE_IP", "192.168.1.102"),
            # Nominal ur_description kinematics are fine here: mock hardware
            # has no real arm whose calibration could differ from nominal.
            **({"kinematics_params_file": kinematics_params_file(ur_type)}
               if kinematics_params_file(ur_type) else {}),
            "use_mock_hardware": "true",
            "mock_sensor_commands": "true",
            "launch_rviz": "false",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
        }.items()
    )

    moveit_py_node = Node(
        name="moveit_py",
        package="recycle_bot",
        executable=LaunchConfiguration("moveit_exec_file"),
        output="both",
        parameters=[moveit_config.to_dict(), {"ur_type": ur_type}],
    )

    # Start everything else only after cleanup finishes
    start_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[
                ur_robot_driver_launch,
                moveit_py_node,
            ],
        )
    )

    return [cleanup, start_after_cleanup]


def generate_launch_description():
    moveit_exec_file = DeclareLaunchArgument(
        "moveit_exec_file",
        default_value="rec_bot_smoke",
        description="Python API smoke file name",
    )

    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value=DEFAULT_UR_TYPE,
        description="Which UR arm to simulate (ur16e, ur3e). Selects the MoveIt "
                    "config and the recycle_bot config/<ur_type>/ directory.",
    )

    return LaunchDescription(
        [
            moveit_exec_file,
            ur_type_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
