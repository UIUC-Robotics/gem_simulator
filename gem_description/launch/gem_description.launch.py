#!/usr/bin/env python3

"""
GEM Robot Description Launch File (ROS 2)
Publishes the robot description and state
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='/',
        description='Namespace for the robot'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='Start RViz visualization'
    )
    
    # Get launch configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')
    
    # Get URDF via xacro
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('gem_description'),
            'urdf',
            'gem.urdf.xacro'
        ])
    ])
    
    # Joint state publisher node (for visualization without simulation)
    # Publishes joint states for all joints in the URDF
    # When robot_description is provided, it automatically publishes all joints
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(start_rviz)
    )
    
    # Robot state publisher node
    # Subscribes to joint_states and publishes TF transforms for all links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'publish_frequency': 30.0,
            'use_sim_time': use_sim_time
        }]
    )
    
    # RViz config file path (default to gem_launch config)
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare('gem_description'),
        'rviz',
        'gem.rviz'
    ])
    
    # RViz node
    # Note: If custom rviz_config is provided, user should pass it via command line
    # For now, we'll use the default config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_rviz)
    )
    
    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        start_rviz_arg,
        joint_state_publisher,  # Start joint_state_publisher first
        robot_state_publisher,   # Then robot_state_publisher
        rviz_node,
    ])

