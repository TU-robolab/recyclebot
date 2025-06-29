# launch/rec_bot_smoke.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
    # resolve the chosen UR type at runtime
    ur_type_value = LaunchConfiguration("ur_type").perform(context)

    # build MoveItCpp parameters for that robot
    moveit_cfg = (
        MoveItConfigsBuilder(
            package_name="ur_moveit_config",
            robot_name=ur_type_value,
        )
        .moveit_cpp()
        .to_dict()
    )

    smoke_node = Node(
        package="recycle_bot",
        executable="rec_bot_smoke",
        name="smoke",
        output="screen",
        parameters=[
            moveit_cfg,                # ← feeds MoveItPy
            {"ur_type": ur_type_value}
        ],
    )
    return [smoke_node]


def generate_launch_description():
    declare_type = DeclareLaunchArgument(
        "ur_type",
        default_value="ur16e",
        description="UR model (ur3e, ur5e, ur10e, ur16e …)",
    )

    return LaunchDescription(
        [
            declare_type,
            OpaqueFunction(function=launch_setup),
        ]
    )
