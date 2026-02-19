import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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

    # Launch argument: seconds to wait for teach pendant before auto-continuing
    wait_timeout_arg = DeclareLaunchArgument(
        "wait_timeout",
        default_value="30.0",
        description="Seconds to wait for External Control URCap before launching remaining nodes",
    )

    # =========================================================================
    # Stage 1: Kill leftover ROS processes to avoid controller conflicts
    # =========================================================================
    cleanup = ExecuteProcess(
        cmd=["bash", "-c",
             "pkill -INT -f 'ros2_control_node|controller_manager' 2>/dev/null; sleep 1; echo '[cleanup] Done'"],
        output="screen",
    )

    # =========================================================================
    # Stage 2a: UR Robot Driver (real hardware)
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
            "use_tool_communication": "true",
            "tool_device_name": "/tmp/ttyUR",
            "tool_voltage": "24",
            "tool_parity": "0",
            "tool_baud_rate": "115200",
            "tool_stop_bits": "1",
            "tool_rx_idle_chars": "1.5",
            "tool_tx_idle_chars": "3.5",
        }.items()
    )

    # =========================================================================
    # Stage 2b: Wait gate — operator enables External Control URCap
    #   Waits for:
    #     ros2 service call /launch_gate std_srvs/srv/Trigger   (instant)
    #   OR timeout (default wait_timeout seconds, configurable via launch arg)
    # =========================================================================
    wait_gate = Node(
        package="recycle_bot",
        executable="launch_gate",
        name="launch_gate",
        output="screen",
        parameters=[{"timeout_sec": LaunchConfiguration("wait_timeout")}],
    )

    # =========================================================================
    # Stage 3: Remaining nodes (after gate)
    # =========================================================================
    vision_node = Node(
        package="recycle_bot",
        executable="rec_bot_vision",
        name="rec_bot_vision",
        output="screen",
    )

    core_node = Node(
        package="recycle_bot",
        executable="rec_bot_core",
        name="rec_bot_core",
        output="screen",
    )

    control_node = Node(
        name="moveit_py",
        package="recycle_bot",
        executable="rec_bot_control",
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={
            "enable_rgbd": "true",
            "enable_sync": "true",
            "align_depth.enable": "true",
            "enable_color": "true",
            "enable_depth": "true",
        }.items(),
    )

    grip_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("grip_command_package"),
                "launch",
                "master.launch.py",
            )
        )
    )

    # Stage 2: after cleanup → start UR driver + wait gate
    start_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[
                ur_robot_driver_launch,
                wait_gate,
            ],
        )
    )

    # Stage 3: after gate exits → start remaining nodes
    start_after_gate = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_gate,
            on_exit=[
                realsense_launch,
                vision_node,
                core_node,
                control_node,
                grip_launch,
            ],
        )
    )

    return LaunchDescription(
        [
            wait_timeout_arg,
            cleanup,
            start_after_cleanup,
            start_after_gate,
        ]
    )
