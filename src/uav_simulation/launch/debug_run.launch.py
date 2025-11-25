from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_uav_simulation = FindPackageShare('uav_simulation')
    pkg_uav_bringup = FindPackageShare('uav_bringup')

    return LaunchDescription([
        # Include the main simulation launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_uav_simulation, 'launch', 'simulate.launch.py'])
            ),
            launch_arguments={'world': 'tunnel_world'}.items(),
        ),

        # Launch the Tunnel Navigation Node with DEBUG logging
        Node(
            package='uav_bringup',
            executable='tunnel_nav',
            name='tunnel_nav',
            output='screen',
            arguments=['--ros-args', '--log-level', 'debug'],
            parameters=[{'use_sim_time': True}]
        )
    ])
