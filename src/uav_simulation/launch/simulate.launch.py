from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get the package share directory
    pkg_share = FindPackageShare(package='uav_simulation').find('uav_simulation')
    
    # World file path
    world_file = os.path.join(pkg_share, 'worlds', 'tunnel_world.world')
    
    # Model path
    model_path = os.path.join(pkg_share, 'models')
    
    # Gazebo launch arguments
    gazebo_args = [
        world_file,
        '--verbose',
    ]
    
    # Set GAZEBO_MODEL_PATH
    # Note: ALSA/audio warnings are harmless in WSL environments - Gazebo will disable audio automatically
    env = {'GAZEBO_MODEL_PATH': model_path}
    
    return LaunchDescription([
        # Launch Gazebo (world only, no drone)
        ExecuteProcess(
            cmd=['gazebo'] + gazebo_args,
            output='screen',
            additional_env=env
        ),
    ])
