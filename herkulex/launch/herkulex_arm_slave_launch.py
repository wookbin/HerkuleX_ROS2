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
    
	# HerkuleX arm
    herkulex_arm_slave_node = Node(
        package='herkulex',
        executable='herkulex_arm_slave_node',
        name='herkulex_arm_slave_node',
        output='screen'
    )


    return LaunchDescription(
    [
    	herkulex_arm_slave_node,
        
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0', #'/dev/HerkuleX'
            description='Serial port for HerkuleX'),

        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for serial communication'),

        Node(
            package='herkulex',
            executable='herkulex_node',
            name='herkulex_slave_node',
            namespace='slave',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'total_axis': 7 #1 #2 [2 axis_id]: 1, 2
        }]),
   
		
		#IncludeLaunchDescription(
		#PythonLaunchDescriptionSource(
		#	[get_package_share_directory('herkulex_description'), '/launch/herkulex_arm_description.launch.py']),
		#),
    ]
)
