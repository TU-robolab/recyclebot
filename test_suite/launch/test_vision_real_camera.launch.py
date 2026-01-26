#!/usr/bin/env python3
"""
Launch file for vision workflow testing with real RealSense camera.

Runs the same tests as test_vision_workflow.py but with actual hardware
instead of fake_rgbd_publisher.

Outputs:
  - /tmp/vision_workflow_test_report.txt
  - /tmp/rgbd_frame_*_rgb.png
  - /tmp/rgbd_frame_*_depth.png
  - /tmp/rgbd_frame_*_combined.png
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for real camera vision test."""

    # RealSense camera launch with RGBD enabled
    # From README: ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true
    #              enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'enable_rgbd': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
        }.items()
    )

    # vision detector node
    vision_node = Node(
        package='recycle_bot',
        executable='rec_bot_vision',
        name='rec_bot_vision',
        output='screen',
    )

    # test script path
    test_script = os.path.join(
        os.path.dirname(__file__),
        '..',
        'test',
        'test_vision_workflow.py'
    )

    # wait longer for real camera initialization (camera startup + YOLO model loading)
    run_test = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', '-m', 'pytest', test_script, '-v', '-s', '--tb=short'],
                output='screen',
                shell=False,
                on_exit=[Shutdown()]
            )
        ]
    )

    return LaunchDescription([
        realsense_launch,
        vision_node,
        run_test,
    ])
