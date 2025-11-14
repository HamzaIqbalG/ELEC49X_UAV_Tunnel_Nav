from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
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
    # Both worlds use .sdf format for Gazebo Fortress
    world_file = os.path.join(pkg_share, 'worlds', f'{world_name}.sdf')
    
    # Model file path - use X3 quadcopter (pre-tuned)
    model_name = context.launch_configurations.get('model', 'x3_quadcopter')
    model_file = os.path.join(model_path, model_name, 'model.sdf')
    
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
    
    # Gazebo topic patterns for sensors
    # Format: /world/<world_name>/model/<model_name>/link/<link_name>/sensor/<sensor_name>/<topic>
    # Note: X3 model may not have these sensors, so we'll make them optional
    lidar_gz_topic = f"/world/{world_name}/model/{model_name}/link/lidar_link/sensor/lidar_sensor/scan"
    imu_gz_topic = f"/world/{world_name}/model/{model_name}/link/imu_link/sensor/imu_sensor/imu"
    
    # Gazebo topic for motor speed commands (X3 model uses motor speed commands)
    # X3 model with robotNamespace=X3 subscribes to: /X3/gazebo/command/motor_speed
    # For Gazebo Fortress: The bridge creates a ROS topic with the same name
    motor_speed_gz_topic = f"/X3/gazebo/command/motor_speed"
    
    # ROS 2 topic names for sensors (bridged from Gazebo)
    lidar_ros_topic = "/uav/scan"
    imu_ros_topic = "/uav/imu"
    
    return [
        # Launch Gazebo Fortress
        ExecuteProcess(
            cmd=['ign', 'gazebo'] + gazebo_args,
            output='screen',
            env=env
        ),
        
        # Spawn the drone model using ros_gz_sim (wait for Gazebo to start)
        TimerAction(
            period=3.0,  # Wait 3 seconds for Gazebo to initialize
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_drone',
                    arguments=[
                        '-world', world_name,
                        '-file', model_file,
                        '-name', 'x3_quadcopter',  # Spawn name (can be different from model internal name)
                        '-x', '0.0',
                        '-y', '0.0',
                        '-z', '2.0'
                    ],
                    output='screen'
                ),
            ]
        ),
        
        # Bridge LiDAR from Gazebo Transport to ROS 2
        # Format: <gz_topic>@<ros_type>[<gz_type>
        # [ means Gazebo -> ROS direction (one-way)
        # Note: The bridge will create a ROS topic with the same name as the Gazebo topic
        # We'll need to check the actual topic name Gazebo uses
        TimerAction(
            period=8.0,  # Wait 8 seconds for drone to spawn and sensors to fully initialize
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='lidar_bridge',
                    arguments=[
                        # Bridge format: <gz_topic>@<ros_type>[<gz_type>
                        # This bridges the Gazebo topic to a ROS topic with the same name
                        f'{lidar_gz_topic}@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'
                    ],
                    output='screen'
                ),
            ]
        ),
        
        # Bridge IMU from Gazebo Transport to ROS 2
        TimerAction(
            period=8.0,  # Wait 8 seconds for drone to spawn and sensors to fully initialize
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='imu_bridge',
                    arguments=[
                        # Bridge format: <gz_topic>@<ros_type>[<gz_type>
                        f'{imu_gz_topic}@sensor_msgs/msg/Imu[ignition.msgs.IMU'
                    ],
                    output='screen'
                ),
            ]
        ),
        
        # Bridge motor speed commands from ROS 2 to Gazebo (X3 model uses motor speeds)
        # ] means ROS -> Gazebo direction (one-way)
        # For Gazebo Fortress: use actuator_msgs/msg/Actuators which bridges to ignition.msgs.Actuators
        TimerAction(
            period=8.0,  # Wait 8 seconds for drone to spawn
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='motor_speed_bridge',
                    arguments=[
                        # Bridge format: <gz_topic>@<ros_type>]<gz_type>
                        # actuator_msgs/msg/Actuators bridges correctly to ignition.msgs.Actuators
                        # The ROS topic name matches the Gazebo topic name
                        f'{motor_speed_gz_topic}@actuator_msgs/msg/Actuators]ignition.msgs.Actuators'
                    ],
                    output='screen'
                ),
            ]
        ),
        
        # Optional: Motor speed controller (automatic hover)
        # This publishes motor speed commands for hover
        # Only enable if controller is enabled AND manual control is NOT enabled
        TimerAction(
            period=10.0,  # Wait 10 seconds for everything to initialize
            actions=[
                Node(
                    package='uav_bringup',
                    executable='motor_speed_controller',
                    name='motor_speed_controller',
                    condition=IfCondition(
                        # Check both conditions: controller enabled AND manual control disabled
                        PythonExpression([
                            "'", context.launch_configurations.get('enable_controller', 'false'), "' == 'true'",
                            " and ",
                            "'", context.launch_configurations.get('manual_control', 'false'), "' == 'false'"
                        ])
                    ) if context.launch_configurations.get('enable_controller', 'false') == 'true' and context.launch_configurations.get('manual_control', 'false') == 'false' else None,
                    output='screen'
                ),
            ]
        ),
        
        # Optional: Manual control (keyboard control - overrides automatic controller)
        # NOTE: Keyboard input is captured in the terminal where ros2 launch is run
        TimerAction(
            period=10.0,  # Wait 10 seconds for everything to initialize
            actions=[
                Node(
                    package='uav_bringup',
                    executable='manual_control',
                    name='manual_control',
                    condition=IfCondition(
                        context.launch_configurations.get('manual_control', 'false')
                    ),
                    output='screen',
                    emulate_tty=True  # Ensure stdin is connected for keyboard input
                ),
            ]
        ),
    ]


def generate_launch_description():
    # Declare launch argument for world selection
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='tunnel_world',
        description='World file to load: tunnel_world or simpletunnel'
    )
    
    # Declare launch argument for model selection
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='x3_quadcopter',
        description='Drone model to use: x3_quadcopter (pre-tuned)'
    )
    
    # Declare launch argument for enabling motor speed controller
    controller_arg = DeclareLaunchArgument(
        'enable_controller',
        default_value='false',
        description='Enable motor speed controller to make drone hover'
    )
    
    # Declare launch argument for manual control mode
    manual_control_arg = DeclareLaunchArgument(
        'manual_control',
        default_value='false',
        description='Enable manual keyboard control (overrides automatic controller)'
    )
    
    return LaunchDescription([
        world_arg,
        model_arg,
        controller_arg,
        manual_control_arg,
        OpaqueFunction(function=launch_setup)
    ])
