#!/usr/bin/env python3
"""
Launch file for automated vision workflow testing.
Starts fake RGBD publisher, vision node, and runs integration tests.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, Shutdown
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for vision workflow test"""

    # Fake RGBD publisher node
    fake_camera_node = Node(
        package='test_suite',
        executable='fake_rgbd_publisher',
        name='fake_rgbd_publisher',
        output='screen',
    )

    # Vision detector node (from recycle_bot package)
    vision_node = Node(
        package='recycle_bot',
        executable='rec_bot_vision',
        name='vision_detector',
        output='screen',
    )

    # Wait for nodes to start, then run pytest
    # Using ExecuteProcess to run pytest on the test file
    test_script = os.path.join(
        os.path.dirname(__file__),
        '..',
        'test',
        'test_vision_workflow.py'
    )

    run_test = TimerAction(
        period=10.0,  # Wait 10 seconds for nodes to initialize
        actions=[
            ExecuteProcess(
                cmd=['python3', '-m', 'pytest', test_script, '-v', '--tb=short'],
                output='screen',
                shell=False,
                on_exit=[Shutdown()]
            )
        ]
    )

    return LaunchDescription([
        fake_camera_node,
        vision_node,
        run_test,
    ])
