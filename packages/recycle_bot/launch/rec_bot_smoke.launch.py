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

from recycle_bot.robot_identity import verify_robot_model
from recycle_bot.robot_profile import (
    DEFAULT_UR_TYPE,
    build_moveit_config,
    kinematics_params_file,
    resolve_ur_type,
)
from recycle_bot.robot_profile import robot_ip as resolve_robot_ip


def launch_setup(context, *args, **kwargs):
    """Build the launch actions once ur_type is a concrete string.

    See rec_bot.launch.py for why this needs OpaqueFunction.
    """
    ur_type = resolve_ur_type(LaunchConfiguration("ur_type").perform(context))
    moveit_config = build_moveit_config(ur_type)

    # Robot IP: resolved per-arm (UR3E_ROBOT_IP / UR16E_ROBOT_IP), falling back
    # to the single-robot REMOTE_IP that export_env.sh writes.
    robot_ip = resolve_robot_ip(ur_type)

    # Pre-flight: refuse to drive the wrong arm. This launch moves real hardware,
    # so a ur_type/robot_ip mismatch would send one arm's trajectories to
    # another. See robot_identity.py. Unreachable controllers only warn.
    if LaunchConfiguration("verify_robot").perform(context).lower() != "false":
        ok, detail = verify_robot_model(ur_type, ip=robot_ip, strict=False)
        if not ok:
            raise RuntimeError(
                f"{detail}\n\n  Pass verify_robot:=false to bypass this check."
            )
        print(f"[rec_bot_smoke.launch] robot check: {detail}")

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
        }.items()
    )

    # =========================================================================
    # Stage 2b: Wait gate — operator enables External Control URCap
    # =========================================================================
    wait_gate = Node(
        package="recycle_bot",
        executable="launch_gate",
        name="launch_gate",
        output="screen",
        parameters=[{"timeout_sec": LaunchConfiguration("wait_timeout")}],
    )

    # =========================================================================
    # Stage 3: Smoke test node (after gate)
    # =========================================================================
    moveit_py_node = Node(
        name="moveit_py",
        package="recycle_bot",
        executable=LaunchConfiguration("moveit_exec_file"),
        output="both",
        parameters=[moveit_config.to_dict(), {"ur_type": ur_type}],
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

    # Stage 3: after gate exits → start smoke node
    start_after_gate = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_gate,
            on_exit=[
                moveit_py_node,
            ],
        )
    )

    return [cleanup, start_after_cleanup, start_after_gate]


def generate_launch_description():
    moveit_exec_file = DeclareLaunchArgument(
        "moveit_exec_file",
        default_value="rec_bot_smoke",
        description="Python API smoke file name",
    )

    wait_timeout_arg = DeclareLaunchArgument(
        "wait_timeout",
        default_value="30.0",
        description="Seconds to wait for External Control URCap before launching remaining nodes",
    )

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

    return LaunchDescription(
        [
            moveit_exec_file,
            wait_timeout_arg,
            ur_type_arg,
            verify_robot_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
