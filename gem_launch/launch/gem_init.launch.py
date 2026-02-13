#!/usr/bin/env python3

"""
GEM Simulator Main Launch File (ROS 2)
Launches Gazebo, spawns the GEM vehicle, and starts RViz
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='smaller_track.world',
        description='World filename to load from gem_gazebo/worlds/ (e.g. silverstone.world, default: smaller_track.world)'
    )
    
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='Initial x position'
    )
    
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='-3.0',
        description='Initial y position'
    )
    
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial yaw orientation'
    )
    
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Start RViz visualization'
    )


    
    # Get launch configurations
    world_name = LaunchConfiguration('world_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')
    

    from launch.substitutions import TextSubstitution
    
    # Set GZ_SIM_RESOURCE_PATH so model://oval_track in oval_track.world can be found
    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([FindPackageShare('gem_gazebo'), 'models'])
    )
    set_gz_version = SetEnvironmentVariable(
        name='GZ_SIM_VERSION',
        value='8'
    )
    # Full path to world file so Gazebo can find it when user passes e.g. world_name:=silverstone.world
    world_path = PathJoinSubstitution([FindPackageShare('gem_gazebo'), 'worlds', world_name])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': ['-r -v1 ', world_path],
        'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Bridge params
    bridge_params = PathJoinSubstitution([
        FindPackageShare('gem_gazebo'),
        'config',
        'gz_bridge.yaml'
    ])
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='bridge_node',
        name='gz_bridge',
        parameters=[{
            'config_file': bridge_params
        }],
        output='screen'
    )
    ros_gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge',
        arguments=["/camera/image_raw", "/depth_camera/image_raw"],
        output='screen'
    )
    
    # Include vehicle launch
    vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gem_gazebo'),
                'launch',
                'gem_vehicle.launch.py'
            ])
        ]),
        launch_arguments={
            'x': x,
            'y': y,
            'yaw': yaw,
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Delay vehicle launch to ensure Gazebo is ready
    # Wait 3 seconds after Gazebo starts before spawning entity
    delayed_vehicle_launch = TimerAction(
        period=3.0,
        actions=[vehicle_launch]
    )
    
    # RViz node
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('gem_launch'),
        'config_rviz',
        'gem_ros2.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_rviz)
    )

    # Static world -> odom TF (identity)
    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Publish odom -> base_footprint TF from /odom (connects odom to robot)
    odom_to_tf_node = Node(
        package='gem_gazebo',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        world_name_arg,
        x_arg,
        y_arg,
        yaw_arg,
        start_rviz_arg,
        
        # Environment variables
        set_gz_resource_path,
        set_gz_version,

        
        # Launches
        gazebo_launch,
        ros_gz_bridge,
        ros_gz_image_bridge,
        static_tf_world_odom,
        odom_to_tf_node,
        delayed_vehicle_launch,
        rviz_node,
    ])

