#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/HerkuleX',
            description='Serial port for HerkuleX'),

        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for serial communication'),

        Node(
            package='herkulex',
            executable='herkulex_node',
            name='herkulex_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'total_axis': 2 #1 [2 axis_id]: 1, 2
            }]
        )
    ]
)
