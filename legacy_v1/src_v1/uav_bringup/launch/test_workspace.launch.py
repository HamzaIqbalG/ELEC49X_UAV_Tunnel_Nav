from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Simple test node to verify ROS 2 workspace integration
        Node(
            package='uav_bringup',
            executable='test_node',
            name='test_node',
            output='screen'
        ),
    ])

