import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _append_env_path(new_path: str, env_var: str) -> str:
    current = os.environ.get(env_var, "")
    return new_path if not current else new_path + os.pathsep + current


def _launch_gz(context, world_path):
    headless = LaunchConfiguration("headless").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )
    cmd = ["gz", "sim", "-r"]
    if headless:
        cmd.append("-s")
    cmd.append(world_path)
    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description() -> LaunchDescription:
    gz_share = get_package_share_directory("uav_tunnel_gz")
    nav_share = get_package_share_directory("uav_tunnel_nav")
    gz_prefix = get_package_prefix("uav_tunnel_gz")

    world_path = os.path.join(gz_share, "worlds", "tunnel.sdf")
    models_path = os.path.join(gz_share, "models")
    plugin_path = os.path.join(gz_prefix, "lib")
    rviz_config = os.path.join(nav_share, "rviz", "uav_tunnel.rviz")

    resource_path = _append_env_path(models_path, "GZ_SIM_RESOURCE_PATH")
    system_plugin_path = _append_env_path(
        plugin_path, "GZ_SIM_SYSTEM_PLUGIN_PATH"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="Run Gazebo without the GUI (recommended for WSL containers).",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Launch RViz2 for LiDAR visualization.",
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", system_plugin_path),
            OpaqueFunction(function=_launch_gz, args=[world_path]),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                    "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
                ],
                output="screen",
            ),
            Node(
                package="uav_tunnel_nav",
                executable="tunnel_navigator",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="uav_tunnel_nav",
                executable="scan_reframe",
                parameters=[
                    {"use_sim_time": True, "frame_id": "base_link"},
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "1", "world", "base_link"],
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
