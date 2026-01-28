#!/usr/bin/env python3
"""
Launch file for end-to-end control + robot motion testing.

Tests complete pipeline with UR virtual robot (fake hardware):
  fake_rgbd → vision → core → control → MoveIt → UR virtual robot

Components:
  - fake_rgbd_publisher: Synthetic camera data
  - rec_bot_vision: YOLO detection
  - rec_bot_core: 3D projection
  - UR robot driver: Virtual robot (use_mock_hardware=true)
  - rec_bot_control: MoveIt planning + control
  - mock_gripper_service: Simulated gripper
  - pytest: Test execution after 30s initialization

Usage:
  ros2 launch test_suite test_control_robot_motion.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Generate launch description for control + robot motion testing."""

    # =========================================================================
    # 1. Fake RGBD Camera
    # =========================================================================
    fake_camera = Node(
        package='test_suite',
        executable='fake_rgbd_publisher',
        name='fake_rgbd_publisher',
        output='screen'
    )

    # =========================================================================
    # 2. Vision Detection Node (YOLO)
    # =========================================================================
    vision_node = Node(
        package='recycle_bot',
        executable='rec_bot_vision',
        name='rec_bot_vision',
        output='screen'
    )

    # =========================================================================
    # 3. Core Processing Node (3D projection)
    # =========================================================================
    core_node = Node(
        package='recycle_bot',
        executable='rec_bot_core',
        name='rec_bot_core',
        output='screen'
    )

    # =========================================================================
    # 4. UR Robot Driver with VIRTUAL ROBOT (fake hardware)
    # =========================================================================
    # The UR virtual robot provides:
    # - Joint states on /joint_states
    # - FollowJointTrajectory action server on /scaled_joint_trajectory_controller/follow_joint_trajectory
    # - ros2_control integration with fake_hardware plugin
    ur_robot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ur_robot_driver"), "/launch/ur_control.launch.py"
        ]),
        launch_arguments={
            "ur_type": "ur16e",
            "robot_ip": "192.168.1.102",  # Not used with mock hardware, but required parameter
            "use_mock_hardware": "true",  # KEY: Enable virtual robot
            "mock_sensor_commands": "true",  # KEY: Enable mock sensors
            "launch_rviz": "false",  # Disable RViz for automated testing
            "initial_joint_controller": "scaled_joint_trajectory_controller",
        }.items()
    )

    # =========================================================================
    # 5. MoveIt Configuration
    # =========================================================================
    moveit_config = MoveItConfigsBuilder(
        robot_name="ur16e",
        package_name="ur16e_moveit_config"
    ).to_moveit_configs()

    # =========================================================================
    # 6. Control Node (MoveIt + control logic)
    # =========================================================================
    control_node = Node(
        package='recycle_bot',
        executable='rec_bot_control',
        name='rec_bot_control',
        parameters=[moveit_config.to_dict()],
        output='screen'
    )

    # =========================================================================
    # 7. Mock Gripper Service
    # =========================================================================
    mock_gripper = Node(
        package='test_suite',
        executable='mock_gripper_service',
        name='mock_gripper_service',
        output='screen'
    )

    # =========================================================================
    # 8. Test Execution (delayed 30s for YOLO + MoveIt + UR driver warmup)
    # =========================================================================
    test_script = os.path.join(
        os.path.dirname(__file__),
        '..',
        'test',
        'test_control_robot_motion.py'
    )

    run_test = TimerAction(
        period=30.0,  # YOLO (~10s) + MoveIt (~10s) + UR driver (~10s)
        actions=[
            ExecuteProcess(
                cmd=['python3', '-m', 'pytest', test_script, '-v', '-s', '--tb=short'],
                output='screen',
                shell=False,
                on_exit=[Shutdown()]
            )
        ]
    )

    # =========================================================================
    # Launch Description
    # =========================================================================
    return LaunchDescription([
        fake_camera,
        vision_node,
        core_node,
        ur_robot_driver_launch,  # UR virtual robot
        control_node,
        mock_gripper,
        run_test
    ])
