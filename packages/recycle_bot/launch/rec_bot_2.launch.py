import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
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
        .robot_description_semantic(file_path="config/ur16e.srdf")
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
    # Stage 1: Kill leftover ROS processes to avoid controller conflicts
    # =========================================================================
    cleanup = ExecuteProcess(
        cmd=["bash", "-c",
             "pkill -INT -f 'ros2_control_node|controller_manager' 2>/dev/null; sleep 1; echo '[cleanup] Done'"],
        output="screen",
    )

    # Robot IP: single-sourced from the environment (.env / export_env.sh);
    # falls back to the lab default.
    robot_ip = os.environ.get("REMOTE_IP", "192.168.1.102")

    # =========================================================================
    # Stage 2: UR Robot Driver (real hardware)
    # =========================================================================
    ur_robot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ur_robot_driver"), "/launch/ur_control.launch.py"
        ]),
        launch_arguments={
            "ur_type": "ur16e",
            "robot_ip": robot_ip,
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
            "headless_mode": "true",
        }.items()
    )

    # =========================================================================
    # Stage 3: Trigger robot program + start all remaining nodes after 5s
    # =========================================================================
    start_nodes = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg=">>> Waiting for External Control program (headless mode)..."),
            # Headless mode: the driver sends the External Control URScript to
            # the robot itself at startup — no Polyscope program is loaded, so
            # /dashboard_client/play has nothing to play and always answers
            # "Failed to execute: play" even while the script is running.
            # Instead, confirm via the driver's robot_program_running flag and,
            # if it is not up yet, nudge it with resend_robot_program (the
            # documented headless-mode recovery call).
            ExecuteProcess(
                cmd=["bash", "-c",
                     "for i in $(seq 1 20); do "
                     "if timeout 10 ros2 topic echo --once /io_and_status_controller/robot_program_running 2>/dev/null "
                     "| grep -q 'data: true'; then "
                     "echo '[program] External Control program running'; exit 0; fi; "
                     "echo \"[program] not running yet (attempt $i/20); calling resend_robot_program\"; "
                     "timeout 10 ros2 service call /io_and_status_controller/resend_robot_program std_srvs/srv/Trigger 2>&1 | tail -1; "
                     "sleep 3; "
                     "done; "
                     "echo '[program] WARNING: could not confirm program running - check remote mode / External Control'; "
                     "exit 0"],
                output='screen',
            ),
            IncludeLaunchDescription(
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
            ),
            Node(
                package="recycle_bot",
                executable="rec_bot_vision",
                name="rec_bot_vision",
                output="screen",
                # Auto-capture detections on an interval (seconds); 0 disables it.
                parameters=[{"auto_capture_period_s": 3.0}],
            ),
            Node(
                package="recycle_bot",
                executable="rec_bot_core",
                name="rec_bot_core",
                output="screen",
            ),
            Node(
                name="moveit_py",
                package="recycle_bot",
                executable="rec_bot_control",
                output="both",
                parameters=[moveit_config.to_dict()],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("grip_command_package"),
                        "launch",
                        "master.launch.py",
                    )
                )
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                parameters=[moveit_config.to_dict()],
                output="screen",
            ),
            Node(
                package="recycle_bot",
                executable="rec_bot_viz",
                name="rec_bot_viz",
                output="screen",
            ),
        ]
    )

    # After cleanup → start UR driver and the timer
    start_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[
                ur_robot_driver_launch,
                start_nodes,
            ],
        )
    )

    return LaunchDescription([
        cleanup,
        start_after_cleanup,
    ])
