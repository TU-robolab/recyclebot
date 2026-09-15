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


LAUNCH_TAG = "rec_bot.launch"


def launch_setup(context, *args, **kwargs):
    """Build the launch actions once ur_type is a concrete string.

    MoveItConfigsBuilder loads the xacro and YAML eagerly, so it needs the actual
    arm name — not a LaunchConfiguration substitution that only resolves later.
    OpaqueFunction defers this whole body until the launch context exists, which
    is what makes `ros2 launch ... ur_type:=ur3e` work.
    """
    ur_type = resolve_ur_type(LaunchConfiguration("ur_type").perform(context))
    moveit_config = build_moveit_config(ur_type)

    # RViz layout: full pipeline (robot + planning scene + vision overlays).
    # An empty rviz_config means "let RViz use its built-in default", which is
    # what these launches did before — a bare grid, no robot, no camera. Passing
    # no -d at all is how you ask RViz for that, so build the argument list
    # rather than substituting a path.
    rviz_config = LaunchConfiguration("rviz_config").perform(context).strip()
    rviz_args = ["-d", rviz_config] if rviz_config else []
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context).lower() != "false"
    headless = LaunchConfiguration("headless_mode").perform(context).lower() != "false"

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
    driver_args = {
        "ur_type": ur_type,
        "robot_ip": robot_ip,
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
        "headless_mode": "true" if headless else "false",
    }

    # Per-robot kinematic calibration exported from that arm's teach pendant.
    # It is unique per physical robot, so it cannot be shared or invented. When
    # the file is absent the driver falls back to ur_description's nominal
    # kinematics, which is fine for bring-up but leaves centimetre-scale FK error
    # on real hardware — hence the loud warning rather than a silent default.
    kinematics_file = kinematics_params_file(ur_type)
    if kinematics_file:
        driver_args["kinematics_params_file"] = kinematics_file
    else:
        print(
            f"\n[rec_bot.launch] WARNING: no my_robot_calibration.yaml for "
            f"'{ur_type}'.\n"
            f"  Expected: recycle_bot/config/{ur_type}/my_robot_calibration.yaml\n"
            f"  Falling back to nominal ur_description kinematics. Export the\n"
            f"  calibration from this arm's teach pendant before trusting any\n"
            f"  measured pose.\n"
        )

    ur_robot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ur_robot_driver"), "/launch/ur_control.launch.py"
        ]),
        launch_arguments=driver_args.items()
    )

    # =========================================================================
    # Stage 2b (headless_mode:=true, the default): the driver sends the External
    # Control URScript to the robot itself, so nobody has to open and Play a
    # program on the pendant — the pendant only has to be in Remote Control.
    # There is no gate: the remaining nodes start after a short delay, and
    # rec_bot_control's own startup wait holds its first move until the
    # trajectory controller is active.
    #
    # No Polyscope program is loaded, so /dashboard_client/play has nothing to
    # play. Instead confirm the script via the driver's robot_program_running
    # flag and, if it is not up yet, re-send it with resend_robot_program (the
    # documented headless-mode recovery call).
    # =========================================================================
    program_check = ExecuteProcess(
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
        output="screen",
    )

    # =========================================================================
    # Stage 2b (headless_mode:=false): Wait gate — operator enables External
    # Control URCap on the pendant.
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
    # ur_type is passed to every recycle_bot node so they all resolve the same
    # config/<ur_type>/ directory. Leaving it off one node would mix two cells'
    # calibration in a single run.
    vision_node = Node(
        package="recycle_bot",
        executable="rec_bot_vision",
        name="rec_bot_vision",
        output="screen",
        parameters=[{"ur_type": ur_type}],
    )

    core_node = Node(
        package="recycle_bot",
        executable="rec_bot_core",
        name="rec_bot_core",
        output="screen",
        parameters=[{"ur_type": ur_type}],
    )

    control_node = Node(
        name="moveit_py",
        package="recycle_bot",
        executable="rec_bot_control",
        output="both",
        parameters=[moveit_config.to_dict(), {"ur_type": ur_type}],
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=rviz_args,
        parameters=[moveit_config.to_dict()],
        output="screen",
    )

    viz_node = Node(
        package="recycle_bot",
        executable="rec_bot_viz",
        name="rec_bot_viz",
        output="screen",
        parameters=[{"ur_type": ur_type}],
    )

    remaining_nodes = [
        realsense_launch,
        vision_node,
        core_node,
        control_node,
        grip_launch,
        viz_node,
        *([rviz_node] if launch_rviz else []),
    ]

    if headless:
        # Stage 2: after cleanup → UR driver; 5 s later the program check and
        # everything else
        start_after_cleanup = RegisterEventHandler(
            OnProcessExit(
                target_action=cleanup,
                on_exit=[
                    ur_robot_driver_launch,
                    TimerAction(
                        period=5.0,
                        actions=[
                            LogInfo(msg=">>> Headless mode: starting the External "
                                        "Control program from the driver..."),
                            program_check,
                            *remaining_nodes,
                        ],
                    ),
                ],
            )
        )
        return [cleanup, start_after_cleanup]

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
            on_exit=remaining_nodes,
        )
    )

    return [cleanup, start_after_cleanup, start_after_gate]


def generate_launch_description():
    # Headless by default: the driver starts the robot program itself, so the
    # operator never has to open and Play External Control on the pendant (the
    # pendant must be in Remote Control mode). false = the pendant program +
    # launch gate flow.
    headless_mode_arg = DeclareLaunchArgument(
        "headless_mode",
        default_value="true",
        description="Driver sends the robot program itself (pendant in Remote "
                    "Control). false: open External Control on the pendant and "
                    "press Play, released through the launch gate.",
    )

    # Launch argument (headless_mode:=false only): seconds to wait for the teach
    # pendant before auto-continuing.
    # 60 s gives the operator time to walk to the pendant; call the /launch_gate
    # service to continue immediately.
    wait_timeout_arg = DeclareLaunchArgument(
        "wait_timeout",
        default_value="60.0",
        description="Seconds to wait for External Control URCap before launching remaining nodes",
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(
            get_package_share_directory("recycle_bot"), "config", "rec_bot.rviz"
        ),
        description="RViz layout file. Pass an empty string for RViz's own default.",
    )

    # The operator dashboard shows the camera and detections itself and passes
    # launch_rviz:=false unless asked for the 3D view.
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Start RViz.",
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
            headless_mode_arg,
            wait_timeout_arg,
            ur_type_arg,
            verify_robot_arg,
            rviz_config_arg,
            launch_rviz_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
