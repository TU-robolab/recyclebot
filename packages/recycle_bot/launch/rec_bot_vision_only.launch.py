import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_fake_camera = LaunchConfiguration("use_fake_camera")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_fake_camera",
            default_value="false",
            description="Use synthetic RGBD frames instead of a real RealSense camera",
        ),

        # =====================================================================
        # Camera source — fake or real, mutually exclusive
        # =====================================================================
        Node(
            package="test_suite",
            executable="fake_rgbd_publisher",
            name="fake_rgbd_publisher",
            output="screen",
            condition=IfCondition(use_fake_camera),
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
            condition=UnlessCondition(use_fake_camera),
        ),

        # =====================================================================
        # Vision pipeline
        # =====================================================================
        Node(
            package="recycle_bot",
            executable="rec_bot_vision",
            name="rec_bot_vision",
            output="screen",
        ),
        Node(
            package="recycle_bot",
            executable="rec_bot_core",
            name="rec_bot_core",
            output="screen",
        ),

        # =====================================================================
        # Visualization — annotated image, 3D markers, raw camera passthrough
        # =====================================================================
        Node(
            package="recycle_bot",
            executable="rec_bot_viz",
            name="rec_bot_viz",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                os.path.join(
                    get_package_share_directory("recycle_bot"),
                    "config",
                    "vision_debug.rviz",
                ),
            ],
            output="screen",
        ),
    ])
