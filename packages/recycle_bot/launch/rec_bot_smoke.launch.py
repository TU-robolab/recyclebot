import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur16e", package_name="ur16e_moveit_config"
        )
        .robot_description(file_path="config/ur16e.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("ur16e_moveit_config")
            + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
    )

    moveit_exec_file = DeclareLaunchArgument(
        "moveit_exec_file",
        default_value="rec_bot_smoke",
        description="Python API smoke file name",
    )

    moveit_py_node = Node(
        name="moveit_py",
        package="recycle_bot",
        executable=LaunchConfiguration("moveit_exec_file"),
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    # rviz_config_file = os.path.join(
    #     get_package_share_directory("ur16e_moveit_config"),
    #     "config",
    #     "moveit.rviz",
    # )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #     ],
    # )

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

    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="log",
    #     parameters=[moveit_config.robot_description],
    # )

    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("ur16e_moveit_config"),
    #     "config",
    #     "ros2_controllers.yaml",
    # )
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[ros2_controllers_path],
    #     remappings=[
    #         ("/controller_manager/robot_description", "/robot_description"),
    #     ],
    #     output="log",
    # )

    # load_controllers = []
    # for controller in [
    #     "ur_arm_controller",
    #     "joint_state_broadcaster",
    # ]:
    #     load_controllers += [
    #         ExecuteProcess(
    #             cmd=["ros2 run controller_manager spawner {}".format(controller)],
    #             shell=True,
    #             output="log",
    #         )
    #     ]

    return LaunchDescription(
        [
            moveit_exec_file,
            moveit_py_node,
            static_tf,
            # robot_state_publisher,
            # ros2_control_node,
            # rviz_node,
            # static_tf,
        ]
        # + load_controllers
    )
