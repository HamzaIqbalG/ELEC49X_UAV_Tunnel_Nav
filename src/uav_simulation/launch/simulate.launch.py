from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def launch_setup(context, *args, **kwargs):
    # Get the package share directory
    pkg_share = FindPackageShare(package='uav_simulation').find('uav_simulation')
    
    # Paths
    world_name = context.launch_configurations.get('world', 'tunnel_world')
    world_file = os.path.join(pkg_share, 'worlds', f'{world_name}.sdf')
    
    # PX4 Paths
    home_dir = os.environ.get('HOME')
    workspace_dir = os.path.join(home_dir, 'ELEC49X_UAV_Tunnel_Nav')
    
    # Check workspace first, then home
    if os.path.exists(os.path.join(workspace_dir, 'PX4-Autopilot')):
        px4_dir = os.path.join(workspace_dir, 'PX4-Autopilot')
    else:
        px4_dir = os.path.join(home_dir, 'PX4-Autopilot')
        
    px4_build_dir = os.path.join(px4_dir, 'build/px4_sitl_default')
    px4_bin = os.path.join(px4_build_dir, 'bin/px4')
    
    # Check if PX4 is built
    if not os.path.exists(px4_bin):
        return [
            LogInfo(msg="ERROR: PX4 binary not found. Please run ./setup_px4.sh first.")
        ]

    # Gazebo Environment Variables
    model_path = os.path.join(pkg_share, 'models')
    px4_models = os.path.join(px4_dir, 'Tools/simulation/gz/models')
    
    env = os.environ.copy()
    
    # Add our models and PX4 models to Gazebo path
    if 'IGN_GAZEBO_RESOURCE_PATH' in env:
        env['IGN_GAZEBO_RESOURCE_PATH'] = f"{model_path}:{px4_models}:{env['IGN_GAZEBO_RESOURCE_PATH']}"
    else:
        env['IGN_GAZEBO_RESOURCE_PATH'] = f"{model_path}:{px4_models}"
        
    # PX4 Environment Variables
    env['PX4_SIM_MODEL'] = 'gz_x500'  # Use standard x500 parameters (physics matches our custom model)
    env['PX4_GZ_MODEL_NAME'] = 'x500_lidar_custom'
    env['PX4_SOURCE_DIR'] = px4_dir
    
    # Gazebo Arguments
    gazebo_args = [
        '-r',  # Run immediately
        world_file
    ]
    
    return [
        # 1. Launch MicroXRCEAgent (ROS 2 <-> PX4 Bridge)
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen'
        ),
        
        # 2. Launch Gazebo Fortress
        ExecuteProcess(
            cmd=['ign', 'gazebo'] + gazebo_args,
            output='screen',
            env=env
        ),
        
        # 3. Launch PX4 SITL
        ExecuteProcess(
            cmd=[
                px4_bin,
                '-i', '0',
                '-d',
                os.path.join(px4_build_dir, 'etc'),
                '-s', os.path.join(pkg_share, 'config', 'custom_start.sh')
            ],
            output='screen',
            env=env
        ),
        
        # 4. Spawn the drone model (x500_lidar_custom)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_drone',
                    arguments=[
                        '-world', world_name,
                        '-file', os.path.join(model_path, 'x500_lidar_custom', 'model.sdf'),
                        '-name', 'x500_lidar_custom',
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '0.5'
                    ],
                    output='screen'
                ),
            ]
        ),
        
        # 5. Bridge Lidar and Pose (for TF)
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='ros_gz_bridge',
                    arguments=[
                        # Bridge LiDAR: Gazebo /scan -> ROS /scan
                        '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                        # Bridge Pose: Gazebo /model/pose -> ROS /tf
                        '/model/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                        # Bridge Ground Truth: Gazebo Odometry -> ROS Odometry
                        # We map Gazebo's /model/x500_lidar_custom/odometry to /odom
                        f'/model/{context.launch_configurations.get("model", "x500_lidar_custom")}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'
                    ],
                    output='screen',
                    parameters=[{'qos_overrides./model/pose.publisher.reliability': 'best_effort'}] 
                ),
                # Static TF for LiDAR (base_link -> lidar_link)
                # Assuming base_link is at 0,0,0 and lidar is at 0,0,0.1
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='lidar_tf_publisher',
                    arguments=['0', '0', '0.1', '0', '0', '0', 'x500_lidar_custom', 'x500_lidar_custom/lidar_link/lidar'],
                    output='screen'
                )
            ]
        ),
        
        # 6. Rviz2 Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'config', 'view_uav.rviz')],
            output='screen'
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='tunnel_world',
            description='World file to load'
        ),
        OpaqueFunction(function=launch_setup)
    ])
