import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from recycle_bot.robot_identity import verify_robot_model
from recycle_bot.robot_profile import (
    DEFAULT_UR_TYPE,
    build_moveit_config,
    kinematics_params_file,
    resolve_ur_type,
)
from recycle_bot.robot_profile import robot_ip as resolve_robot_ip


LAUNCH_TAG = "rec_bot_2.launch"


def launch_setup(context, *args, **kwargs):
    """Build the launch actions once ur_type is a concrete string.

    See rec_bot.launch.py for why this needs OpaqueFunction.
    """
    ur_type = resolve_ur_type(LaunchConfiguration("ur_type").perform(context))
    moveit_config = build_moveit_config(ur_type)

    # =========================================================================
    # Stage 1: Kill leftover ROS processes to avoid controller conflicts
    # =========================================================================
    cleanup = ExecuteProcess(
        cmd=["bash", "-c",
             "pkill -INT -f 'ros2_control_node|controller_manager' 2>/dev/null; sleep 1; echo '[cleanup] Done'"],
        output="screen",
    )

    # Robot IP: resolved per-arm (UR3E_ROBOT_IP / UR16E_ROBOT_IP), falling back
    # to the single-robot REMOTE_IP that export_env.sh writes.
    robot_ip = resolve_robot_ip(ur_type)

    # Pre-flight: refuse to drive the wrong arm.
    #
    # ur_type picks the URDF, limits and cell geometry; robot_ip picks which
    # controller receives the trajectories. Nothing else ties those together, so
    # a stale REMOTE_IP or a swapped arm would send this configuration's motion
    # to a different robot. One read-only dashboard query closes that gap.
    # An unreachable controller only warns — the driver reports that better.
    if LaunchConfiguration("verify_robot").perform(context).lower() != "false":
        ok, detail = verify_robot_model(ur_type, ip=robot_ip, strict=False)
        if not ok:
            raise RuntimeError(
                f"{detail}\n\n  Pass verify_robot:=false to bypass this check."
            )
        print(f"[{LAUNCH_TAG}] robot check: {detail}")

    # =========================================================================
    # Stage 2: UR Robot Driver (real hardware)
    # =========================================================================
    ur_robot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ur_robot_driver"), "/launch/ur_control.launch.py"
        ]),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            # Per-arm teach-pendant calibration; omitted when absent so the
            # driver falls back to nominal ur_description kinematics.
            **({"kinematics_params_file": kinematics_params_file(ur_type)}
               if kinematics_params_file(ur_type) else {}),
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
                parameters=[{"auto_capture_period_s": 3.0, "ur_type": ur_type}],
            ),
            Node(
                package="recycle_bot",
                executable="rec_bot_core",
                name="rec_bot_core",
                output="screen",
                parameters=[{"ur_type": ur_type}],
            ),
            Node(
                name="moveit_py",
                package="recycle_bot",
                executable="rec_bot_control",
                output="both",
                parameters=[moveit_config.to_dict(), {"ur_type": ur_type}],
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
                parameters=[{"ur_type": ur_type}],
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

    return [cleanup, start_after_cleanup]


def generate_launch_description():
    verify_robot_arg = DeclareLaunchArgument(
        "verify_robot",
        default_value="true",
        description="Query the robot's dashboard server and abort if the "
                    "connected arm does not match ur_type.",
    )

    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value=DEFAULT_UR_TYPE,
        description="Which UR arm to drive (ur16e, ur3e). Selects the MoveIt "
                    "config and the recycle_bot config/<ur_type>/ directory.",
    )
    return LaunchDescription([ur_type_arg, verify_robot_arg, OpaqueFunction(function=launch_setup)])
