#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
	# herkulex_pantilt
    herkulex_pantilt_node = Node(
        package='herkulex',
        executable='herkulex_pantilt_node',
        name='herkulex_pantilt_node',
        output='screen'
    )


    return LaunchDescription(
    [
    	herkulex_pantilt_node,
        
       IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('herkulex'), '/launch/herkulex.launch.py']),
		),
		
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('herkulex_description'), '/launch/herkulex_pantilt_description.launch.py']),
		),
    ]
)
