from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get the package share directory
    pkg_share = FindPackageShare(package='uav_simulation').find('uav_simulation')
    
    # World file path - simpletunnel (SDF format for Gazebo Fortress)
    world_file = os.path.join(pkg_share, 'worlds', 'simpletunnel.sdf')
    
    # Model path
    model_path = os.path.join(pkg_share, 'models')
    
    # Gazebo Fortress (Ignition Gazebo 6) uses 'ign gazebo' command
    gazebo_args = [
        world_file,
    ]
    
    # Set environment variables for Gazebo Fortress
    env = os.environ.copy()
    # For Ignition Gazebo, use IGN_GAZEBO_RESOURCE_PATH
    if 'IGN_GAZEBO_RESOURCE_PATH' in env:
        env['IGN_GAZEBO_RESOURCE_PATH'] = f"{model_path}:{env['IGN_GAZEBO_RESOURCE_PATH']}"
    else:
        env['IGN_GAZEBO_RESOURCE_PATH'] = model_path
    
    return LaunchDescription([
        # Launch Gazebo Fortress with simpletunnel world
        ExecuteProcess(
            cmd=['ign', 'gazebo'] + gazebo_args,
            output='screen',
            env=env
        ),
    ])
