#!/usr/bin/env python3
"""
Launch file for real control + robot motion testing (requires MoveIt + UR virtual robot).

Tests MoveIt motion execution with UR virtual robot (fake hardware):
  fake_rgbd → vision → core → control → MoveIt → UR virtual robot

Components:
  - fake_rgbd_publisher: Synthetic camera data
  - rec_bot_vision: YOLO detection
  - rec_bot_core: 3D projection
  - UR robot driver: Virtual robot (use_mock_hardware=true)
  - rec_bot_control: MoveIt planning + control
  - mock_gripper_service: Simulated gripper
  - pytest: Test execution after 30s initialization

Note: For basic pipeline tests without MoveIt, use test_e2e_pipeline.launch.py

Usage:
  ros2 launch test_suite test_real_control_robot_motion.launch.py
  ros2 launch test_suite test_real_control_robot_motion.launch.py ur_type:=ur3e
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from recycle_bot.robot_profile import (
    DEFAULT_UR_TYPE,
    build_moveit_config,
    resolve_ur_type,
)


def launch_setup(context, *args, **kwargs):
    """Build the launch actions once ur_type is a concrete string.

    See recycle_bot's rec_bot.launch.py for why this needs OpaqueFunction.
    """
    ur_type = resolve_ur_type(LaunchConfiguration("ur_type").perform(context))

    debug_no_collision_objects = DeclareLaunchArgument(
        "debug_no_collision_objects",
        default_value="false",
        description="Disable collision objects in rec_bot_control for debugging",
    )
    debug_motion_log = DeclareLaunchArgument(
        "debug_motion_log",
        default_value="true",
        description="Enable verbose motion debug logs in rec_bot_control",
    )

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
        parameters=[{"ur_type": ur_type}],
        output='screen'
    )

    # =========================================================================
    # 3. Core Processing Node (3D projection)
    # =========================================================================
    core_node = Node(
        package='recycle_bot',
        executable='rec_bot_core',
        name='rec_bot_core',
        parameters=[{"ur_type": ur_type}],
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
            "ur_type": ur_type,
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
    moveit_config = build_moveit_config(ur_type)

    # =========================================================================
    # 6. Control Node (MoveIt + control logic)
    # =========================================================================
    control_node = Node(
        package='recycle_bot',
        executable='rec_bot_control',
        name='rec_bot_control',
        parameters=[
            moveit_config.to_dict(),
            {"ur_type": ur_type},
            {"debug_no_collision_objects": LaunchConfiguration("debug_no_collision_objects")},
            {"debug_motion_log": LaunchConfiguration("debug_motion_log")},
        ],
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
        'test_real_control_robot_motion.py'
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
    return [
        debug_no_collision_objects,
        debug_motion_log,
        fake_camera,
        vision_node,
        core_node,
        ur_robot_driver_launch,  # UR virtual robot
        control_node,
        mock_gripper,
        run_test
    ]


def generate_launch_description():
    """Generate launch description for real control + robot motion testing."""
    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value=DEFAULT_UR_TYPE,
        description="Which UR arm to test against (ur16e, ur3e).",
    )
    return LaunchDescription([ur_type_arg, OpaqueFunction(function=launch_setup)])
