import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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

    moveit_exec_file = DeclareLaunchArgument(
        "moveit_exec_file",
        default_value="rec_bot_smoke",
        description="Python API smoke file name",
    )

    # Kill leftover ROS processes to avoid controller conflicts
    cleanup = ExecuteProcess(
        cmd=["bash", "-c",
             "pkill -INT -f 'ros2_control_node|controller_manager' 2>/dev/null; sleep 1; echo '[cleanup] Done'"],
        output="screen",
    )

    # UR Robot Driver with real hardware
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

    moveit_py_node = Node(
        name="moveit_py",
        package="recycle_bot",
        executable=LaunchConfiguration("moveit_exec_file"),
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    # Publish world -> base_link identity transform. The UR driver's RSP
    # may also publish this (from the URDF), but MoveIt needs it available
    # immediately — this ensures 'world' exists in TF before planning starts.
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "world",
            "--child-frame-id", "base",
        ],
    )

    # Start everything else only after cleanup finishes
    start_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[
                ur_robot_driver_launch,
                moveit_py_node,
                # static_tf,
            ],
        )
    )

    return LaunchDescription(
        [
            moveit_exec_file,
            cleanup,
            start_after_cleanup,
        ]
    )
