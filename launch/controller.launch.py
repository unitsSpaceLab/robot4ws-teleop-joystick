#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Get package directory
    pkg_share = get_package_share_directory('robot4ws_teleop_joystick')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')
    
    # Robot4WS Logitech controller node
    controller_node = Node(
        package='robot4ws_teleop_joystick',
        executable='robot4ws_teleop_logi_controller_node',
        name='robot4ws_logiteck_controller',
        parameters=[params_file],
        remappings=[('/cmd_vel', '/Archimede/cmd_vel')],
        output='screen'
    )
    
    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='logiteck_joystick_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.01
        }],
        output='screen'
    )
    
    return LaunchDescription([
        controller_node,
        joy_node
    ])