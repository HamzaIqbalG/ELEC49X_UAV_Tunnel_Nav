from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def launch_setup(context, *args, **kwargs):
    # Get the package share directory
    pkg_share = FindPackageShare(package='uav_simulation').find('uav_simulation')
    
    # Model path
    model_path = os.path.join(pkg_share, 'models')
    
    # Get world name from launch argument
    world_name = context.launch_configurations.get('world', 'tunnel_world')
    
    # Determine world file path based on name
    world_file = os.path.join(pkg_share, 'worlds', f'{world_name}.sdf')
    
    # Model file path
    model_file = os.path.join(model_path, 'basic_drone', 'model.sdf')
    
    # Gazebo Fortress uses 'ign gazebo' command
    gazebo_args = [world_file]
    
    # Set environment variables for Gazebo Fortress
    env = os.environ.copy()
    if 'IGN_GAZEBO_RESOURCE_PATH' in env:
        env['IGN_GAZEBO_RESOURCE_PATH'] = f"{model_path}:{env['IGN_GAZEBO_RESOURCE_PATH']}"
    else:
        env['IGN_GAZEBO_RESOURCE_PATH'] = model_path
    
    # Gazebo topic patterns - these will be created when drone spawns
    # Note: Topic names may vary, we'll use a pattern that should work
    lidar_gz_topic = f"/world/{world_name}/model/basic_drone/link/lidar_link/sensor/lidar_sensor/scan"
    imu_gz_topic = f"/world/{world_name}/model/basic_drone/link/imu_link/sensor/imu_sensor/imu"
    
    # ROS 2 topic names
    lidar_ros_topic = "/uav/scan"
    imu_ros_topic = "/uav/imu"
    
    return [
        # Launch Gazebo Fortress
        ExecuteProcess(
            cmd=['ign', 'gazebo'] + gazebo_args,
            output='screen',
            env=env
        ),
        
        # Spawn the drone model
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_drone',
                    arguments=[
                        '-world', world_name,
                        '-file', model_file,
                        '-name', 'basic_drone',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.5'
                    ],
                    output='screen'
                ),
            ]
        ),
        
        # Bridge LiDAR - wait longer for sensor to initialize
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='lidar_bridge',
                    arguments=[
                        # Format: <gz_topic>@<ros_topic>@<ros_type>[<gz_type>
                        f'{lidar_gz_topic}@{lidar_ros_topic}@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'
                    ],
                    output='screen'
                ),
            ]
        ),
        
        # Bridge IMU
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='imu_bridge',
                    arguments=[
                        f'{imu_gz_topic}@{imu_ros_topic}@sensor_msgs/msg/Imu[ignition.msgs.IMU'
                    ],
                    output='screen'
                ),
            ]
        ),
    ]


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='tunnel_world',
        description='World file to load: tunnel_world or simpletunnel'
    )
    
    return LaunchDescription([
        world_arg,
        OpaqueFunction(function=launch_setup)
    ])

