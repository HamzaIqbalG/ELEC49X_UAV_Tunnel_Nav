import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _append_env_path(new_path: str, current_value: str) -> str:
    return new_path if not current_value else new_path + os.pathsep + current_value


def generate_launch_description() -> LaunchDescription:
    gz_share = get_package_share_directory("uav_tunnel_gz")
    nav_share = get_package_share_directory("uav_tunnel_nav")
    as2_share = get_package_share_directory("as2_gazebo_assets")

    world_path = os.path.join(gz_share, "worlds")
    models_path = os.path.join(gz_share, "models")
    sim_config = os.path.join(gz_share, "config", "as2_tunnel.yaml")
    rviz_config = os.path.join(nav_share, "rviz", "uav_tunnel.rviz")
    ekf_config = os.path.join(nav_share, "config", "ekf.yaml")

    current_resource = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = _append_env_path(world_path, current_resource)
    resource_path = _append_env_path(models_path, resource_path)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo without the GUI.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz2 for LiDAR visualization.",
            ),
            DeclareLaunchArgument(
                "static_tf",
                default_value="true",
                description="Publish a static world->base_link TF for RViz.",
            ),
            DeclareLaunchArgument(
                "use_gt_vio",
                default_value="false",
                description="Publish /uav0/vio/odom from ground truth (sim-only).",
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(as2_share, "launch", "launch_simulation.py")
                ),
                launch_arguments={
                    "simulation_config_file": sim_config,
                    "headless": LaunchConfiguration("headless"),
                    "use_sim_time": "true",
                    "run_on_start": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(as2_share, "launch", "drone_bridges.py")
                ),
                launch_arguments={
                    "simulation_config_file": sim_config,
                    "namespace": "uav0",
                }.items(),
            ),
            Node(
                package="uav_tunnel_nav",
                executable="tunnel_navigator",
                parameters=[
                    {
                        "use_sim_time": True,
                        "cmd_vel_topic": "/gz/uav0/cmd_vel",
                        "scan_topic": "/uav0/sensor_measurements/lidar/scan",
                        "enable_arm": True,
                        "arm_topic": "/gz/uav0/arm",
                        "arm_seconds": 1.0,
                    }
                ],
                output="screen",
            ),
            Node(
                package="uav_tunnel_nav",
                executable="scan_reframe",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": "/uav0/sensor_measurements/lidar/scan",
                        "output_topic": "/scan_fixed",
                        "frame_id": "base_link",
                    }
                ],
                output="screen",
            ),
            Node(
                package="uav_tunnel_nav",
                executable="vio_watchdog",
                parameters=[
                    {
                        "use_sim_time": True,
                        "vio_topic": "/uav0/vio/odom",
                        "startup_timeout_sec": 3.0,
                        "timeout_sec": 0.5,
                    }
                ],
                output="screen",
            ),
            Node(
                package="uav_tunnel_nav",
                executable="gt_to_vio_odom",
                parameters=[
                    {
                        "use_sim_time": True,
                        "pose_topic": "/uav0/ground_truth/pose",
                        "twist_topic": "/uav0/ground_truth/twist",
                        "odom_topic": "/uav0/vio/odom",
                        "frame_id": "odom",
                        "child_frame_id": "base_link",
                    }
                ],
                condition=IfCondition(LaunchConfiguration("use_gt_vio")),
                output="screen",
            ),
            Node(
                package="uav_tunnel_nav",
                executable="imu_reframe",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": "/uav0/sensor_measurements/imu",
                        "output_topic": "/uav0/imu/reframed",
                        "frame_id": "base_link",
                    }
                ],
                output="screen",
            ),
            Node(
                package="uav_tunnel_nav",
                executable="baro_to_pose",
                parameters=[
                    {
                        "use_sim_time": True,
                        "baro_topic": "/uav0/sensor_measurements/air_pressure",
                        "pose_topic": "/uav0/baro/pose",
                        "frame_id": "odom",
                    }
                ],
                output="screen",
            ),
            Node(
                package="uav_tunnel_nav",
                executable="mag_to_yaw",
                parameters=[
                    {
                        "use_sim_time": True,
                        "mag_topic": "/uav0/sensor_measurements/magnetometer",
                        "imu_topic": "/uav0/sensor_measurements/imu",
                        "pose_topic": "/uav0/mag/yaw",
                        "frame_id": "odom",
                    }
                ],
                output="screen",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                parameters=[ekf_config],
                remappings=[("odometry/filtered", "/uav0/basic_odom")],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "1", "world", "odom"],
                condition=IfCondition(LaunchConfiguration("static_tf")),
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
