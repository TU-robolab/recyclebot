from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument("robot_ip", default_value="192.168.1.102"),
        DeclareLaunchArgument("debug", default_value="false"),
        DeclareLaunchArgument("sim_mode", default_value="false"),
        
        #ur robot driver
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ur_robot_driver"), "/launch/ur16e.launch.py"
        ]),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "ur_type": "ur16e",
            "launch_rviz": LaunchConfiguration("sim_mode"),
            "use_tool_communication" : "true",
            "use_mock_hardware" : LaunchConfiguration("sim_mode"),
            "mock_sensor_commands" : LaunchConfiguration("sim_mode"),
            "tool_voltage" : "24",
            "tool_parity" : "0",
            "tool_baud_rate" :  "115200",
            "tool_stop_bits" : "1",
            "tool_rx_idle_chars" : "1.5",
            "tool_tx_idle_chars" : "3.5",
            "tool_device_name" : "/tmp/ttyUR",
        }.items(),
        ),

        #serial interface
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='remote_serial',
                    executable='remote_serial_standalone',
                    name='remote_serial',
                    output='screen',
                    parameters=[
                        {'serial_is_remote': False},
                        {'serial_prefix': '/serial/com1'},
                        {'serial_dev_name': '/tmp/ttyUR'},
                        {'serial_baud_rate': 115200},
                        {'serial_data': 8},
                        {'serial_parity': False},
                        {'serial_stop': 1},
                        {'serial_flow_control': True}
                    ]
                ),
            ]
        ),

        Node(
            package='grip_command_package',
            executable='gripper_node_v4',
            name='gripper_node_v4',
            output='screen',
            parameters=[
                {'debug': LaunchConfiguration('debug')},
            ]
        ),  
    ])
