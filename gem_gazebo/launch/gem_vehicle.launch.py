#!/usr/bin/env python3

"""
GEM Vehicle Launch File (ROS 2)
Spawns the GEM vehicle and starts controllers
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='/',
        description='Namespace for the vehicle'
    )
    
    cmd_timeout_arg = DeclareLaunchArgument(
        'cmd_timeout',
        default_value='0.5',
        description='Command timeout in seconds'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='Initial x position'
    )
    
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='-98.0',
        description='Initial y position'
    )
    
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.3',
        description='Initial z position'
    )
    
    roll_arg = DeclareLaunchArgument(
        'roll',
        default_value='0.0',
        description='Initial roll orientation'
    )
    
    pitch_arg = DeclareLaunchArgument(
        'pitch',
        default_value='0.0',
        description='Initial pitch orientation'
    )
    
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial yaw orientation'
    )
    
    # Get launch configurations
    namespace = LaunchConfiguration('namespace')
    cmd_timeout = LaunchConfiguration('cmd_timeout')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    
    # Include robot description launch
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gem_description'),
                'launch',
                'gem_description.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'start_rviz': 'false',
        }.items()
    )
    
    # Spawn entity node
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_model',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'gem',
            '-allow_renaming', 'true',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
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
            {'cmd_timeout': cmd_timeout},
            {'use_sim_time': use_sim_time},
            ackermann_params_file
        ]
    )

    # Delay gem_control until after spawn + controller_spawner (model and controllers must be up)
    delayed_ackermann = TimerAction(
        period=2.0,
        actions=[ackermann_controller]
    )
    
    # Group nodes with namespace: spawn first, then load controllers, then start gem_control after delay
    vehicle_group = GroupAction([
        PushRosNamespace(namespace),
        spawn_entity,
        controller_spawner,
        delayed_ackermann,
    ])
    
    return LaunchDescription([
        # Arguments
        namespace_arg,
        cmd_timeout_arg,
        use_sim_time_arg,
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        
        # Launches and nodes
        description_launch,
        vehicle_group,
    ])

