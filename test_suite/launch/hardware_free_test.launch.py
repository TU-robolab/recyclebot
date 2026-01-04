#!/usr/bin/env python3
"""
Hardware-free test launch file.
Launches all fake nodes needed for testing without real hardware.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='vision',
        description='Test mode: vision, robot, gripper, or full'
    )

    test_mode = LaunchConfiguration('test_mode')

    return LaunchDescription([
        test_mode_arg,

        LogInfo(msg=['Starting hardware-free test mode: ', test_mode]),

        # Fake RGBD Camera Publisher (for vision tests)
        Node(
            package='',  # Will run as python script
            executable='python3',
            name='fake_rgbd_publisher',
            arguments=['/home/joao/fake_rgbd_publisher.py'],
            output='screen',
            parameters=[{'use_sim_time': False}],
            condition=lambda context: context.launch_configurations['test_mode'] in ['vision', 'full']
        ),

        # Fake Joint State Publisher (for robot tests)
        Node(
            package='',
            executable='python3',
            name='fake_joint_state_publisher',
            arguments=['/home/joao/test_suite/scripts/fake_joint_state_publisher.py'],
            output='screen',
            condition=lambda context: context.launch_configurations['test_mode'] in ['robot', 'full']
        ),

        # Vision Detector Node (for vision tests)
        Node(
            package='recycle_bot',
            executable='rec_bot_vision',
            name='vision_detector',
            output='screen',
            condition=lambda context: context.launch_configurations['test_mode'] in ['vision', 'full']
        ),

        # Gripper Node (for gripper tests)
        Node(
            package='grip_command_package',
            executable='gripper_node',
            name='gripper_node',
            output='screen',
            parameters=[{'debug': True}],
            condition=lambda context: context.launch_configurations['test_mode'] in ['gripper', 'full']
        ),

        LogInfo(msg='All test nodes launched successfully'),
    ])
