#!/usr/bin/env python3
"""
Launch file for end-to-end pipeline testing.

Tests the full data flow:
  fake_rgbd → vision → core → (mock gripper available)

Without requiring MoveIt robot simulation.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for E2E pipeline test."""

    # fake RGBD publisher - simulates RealSense camera
    fake_camera_node = Node(
        package='test_suite',
        executable='fake_rgbd_publisher',
        name='fake_rgbd_publisher',
        output='screen',
    )

    # vision detector - runs YOLO, publishes Detection3DArray
    vision_node = Node(
        package='recycle_bot',
        executable='rec_bot_vision',
        name='rec_bot_vision',
        output='screen',
    )

    # core processor - filters detections, projects to 3D, publishes PoseStamped
    core_node = Node(
        package='recycle_bot',
        executable='rec_bot_core',
        name='rec_bot_core',
        output='screen',
    )

    # mock gripper service - simulates gripper without hardware
    mock_gripper_node = Node(
        package='test_suite',
        executable='mock_gripper_service',
        name='mock_gripper_service',
        output='screen',
    )

    # test script path
    test_script = os.path.join(
        os.path.dirname(__file__),
        '..',
        'test',
        'test_e2e_pipeline.py'
    )

    # wait for nodes to initialize (YOLO model loading takes time)
    run_test = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', '-m', 'pytest', test_script, '-v', '-s', '--tb=short'],
                output='screen',
                shell=False
            )
        ]
    )

    return LaunchDescription([
        fake_camera_node,
        vision_node,
        core_node,
        mock_gripper_node,
        run_test,
    ])
