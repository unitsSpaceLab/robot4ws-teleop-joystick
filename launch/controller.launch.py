#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('robot4ws_teleop_joystick')

    # Parameters files
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # Launch arguments
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    arg_teleop_joystick_ns = DeclareLaunchArgument(
        "ns_joystick", default_value=TextSubstitution(text="Archimede"),
        description="Namespace of the teleop_joystick and joy nodes"
    )

    # Robot4WS Logitech controller node
    controller_node = Node(
        package='robot4ws_teleop_joystick',
        executable='robot4ws_teleop_logi_controller_node',
        namespace=LaunchConfiguration('ns_joystick'),
        name='robot4ws_logitech_controller',
        parameters=[params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        # remappings=[('/cmd_vel', '/Archimede/cmd_vel')],
        output='screen'
    )

    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        namespace=LaunchConfiguration('ns_joystick'),
        name='logitech_joystick_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.01,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    return LaunchDescription([
        arg_use_sim_time,
        arg_teleop_joystick_ns,
        controller_node,
        joy_node
    ])
