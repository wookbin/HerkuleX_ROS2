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
    
	# HerkuleX 0602
    herkulex_0602_ex_node = Node(
        package='herkulex',
        executable='herkulex_0602_ex_node',
        name='herkulex_0602_ex_node',
        output='screen'
    )


    return LaunchDescription(
    [
    	herkulex_0602_ex_node,
        
       IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('herkulex'), '/launch/herkulex.launch.py']),
		),
		
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('herkulex_description'), '/launch/herkulex_0602ex_description.launch.py']),
		),
    ]
)
