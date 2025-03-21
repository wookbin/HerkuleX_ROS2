import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Declare use_sim_time argument
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value="false")

    # Path to RViz configuration file
    rviz_config_dir = os.path.join(
        get_package_share_directory('herkulex_description'),
        'rviz',
        'rviz_pantilt_settings.rviz'
    )

    # Robot State Publisher Node
    herkulex_description_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('herkulex_description'),
                        'urdf',
                        'herkulex_pantilt.xacro',
                    ]),
                ]),
        }]
    )
    
    # Joint State Publisher Node
    joint_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Joint State Publisher GUI Node
    joint_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz Node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Add all actions to the launch description
    ld.add_action(declare_use_sim_time)
    ld.add_action(herkulex_description_node)
    #ld.add_action(joint_node)
    ld.add_action(joint_gui_node)
    ld.add_action(rviz2_node)

    return ld

