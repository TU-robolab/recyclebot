import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
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
    # 0. Kill leftover ROS processes to avoid controller conflicts
    # =========================================================================
    cleanup = ExecuteProcess(
        cmd=["bash", "-c",
             "pkill -INT -f 'ros2_control_node|controller_manager' 2>/dev/null; sleep 1; echo '[cleanup] Done'"],
        output="screen",
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
        launch_arguments={
            "enable_rgbd": "true",
            "enable_sync": "true",
            "align_depth.enable": "true",
            "enable_color": "true",
            "enable_depth": "true",
        }.items(),
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

    # Start everything else only after cleanup finishes
    start_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[
                ur_robot_driver_launch,
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
            cleanup,
            start_after_cleanup,
        ]
    )
