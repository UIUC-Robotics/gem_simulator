#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Include Gazebo launch with empty world
    # Use built-in empty.sdf world - equivalent to: gz sim empty.sdf
    from launch.substitutions import TextSubstitution
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments=[
            ('gz_args', TextSubstitution(text='-r -v 1 empty.sdf'))
        ]
    )
    
    # Bridge clock topic
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    # Include robot description launch
    # This publishes the robot_description topic from the URDF
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gem_description'),
                'launch',
                'gem_description.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Spawn entity node - spawns the vehicle model in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_model',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'gem',
            '-allow_renaming', 'true',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Controller spawner node
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='controller_spawner',
        arguments=[
            'joint_state_broadcaster',
            'left_steering_controller',
            'right_steering_controller',
            'left_front_wheel_controller',
            'right_front_wheel_controller',
            'left_rear_wheel_controller',
            'right_rear_wheel_controller'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Ackermann controller node
    ackermann_params_file = PathJoinSubstitution([
        FindPackageShare('gem_gazebo'),
        'config',
        'gem_ackermann_control_params_ros2.yaml'
    ])
    
    ackermann_controller = Node(
        package='gem_gazebo',
        executable='gem_control.py',
        name='ackermann_controller',
        output='screen',
        parameters=[
            {'cmd_timeout': '0.5'},
            {'use_sim_time': use_sim_time},
            ackermann_params_file
        ]
    )
    
    # Start the test_controllers.py script
    test_controllers_node = Node(
        package='gem_gazebo',
        executable='test_controllers.py',
        name='controller_tester',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Delay description launch to ensure Gazebo is ready
    # Wait 2 seconds after Gazebo starts before publishing robot description
    delayed_description = TimerAction(
        period=2.0,
        actions=[description_launch]
    )
    
    # Delay spawn entity to ensure robot_description topic is available
    # Wait 1 second after description launch (3 seconds total from Gazebo start)
    delayed_spawn_entity = TimerAction(
        period=3.0,
        actions=[spawn_entity]
    )
    
    # Delay controllers to ensure entity is spawned
    # Wait 1 second after spawn entity (4 seconds total from Gazebo start)
    controllers_group = GroupAction([
        controller_spawner,
        ackermann_controller
    ])
    
    delayed_controllers = TimerAction(
        period=4.0,
        actions=[controllers_group]
    )
    
    # Delay test_controllers to ensure controllers are loaded
    # Wait 5 seconds after controllers start (9 seconds total from Gazebo start)
    delayed_test_controllers = TimerAction(
        period=9.0,
        actions=[test_controllers_node]
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        
        # Launches and nodes
        gazebo_launch,
        clock_bridge,
        delayed_description,
        delayed_spawn_entity,
        delayed_controllers,
        delayed_test_controllers,
    ]) 