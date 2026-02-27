"""Launch ArduPilot SITL + Gazebo tunnel simulation with LiDAR nav."""

import os
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WORLD_NAME = "tunnel"
MODEL_NAME = "iris_tunnel"


def _prepend_env(new: str, current: str) -> str:
    return new if not current else new + os.pathsep + current


def _render_bridge_config(template_path: str) -> str:
    """Replace {{ world_name }} / {{ model_name }} in the bridge yaml."""
    with open(template_path) as f:
        text = f.read()
    text = text.replace("{{ world_name }}", WORLD_NAME)
    text = text.replace("{{ model_name }}", MODEL_NAME)
    tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".yaml", mode="w")
    tmp.write(text)
    tmp.close()
    return tmp.name


def _find_sim_vehicle() -> str:
    """Locate sim_vehicle.py on PATH or in common ArduPilot install paths."""
    found = shutil.which("sim_vehicle.py")
    if found:
        return found
    for candidate in [
        "/ardu_ws/src/ardupilot/Tools/autotest/sim_vehicle.py",
        os.path.expanduser("~/ardupilot/Tools/autotest/sim_vehicle.py"),
    ]:
        if os.path.isfile(candidate):
            return candidate
    return ""


def generate_launch_description() -> LaunchDescription:
    gz_share = get_package_share_directory("uav_tunnel_gz")
    nav_share = get_package_share_directory("uav_tunnel_nav")

    world_path = os.path.join(gz_share, "worlds")
    models_path = os.path.join(gz_share, "models")
    world_sdf = os.path.join(world_path, "tunnel.sdf")
    model_sdf = os.path.join(models_path, "iris_tunnel", "model.sdf")
    rviz_config = os.path.join(nav_share, "rviz", "uav_tunnel.rviz")
    slam_config = os.path.join(nav_share, "config", "slam_toolbox.yaml")
    bridge_template = os.path.join(nav_share, "config", "iris_bridge.yaml")
    tunnel_parm = os.path.join(nav_share, "config", "gazebo-iris-tunnel.parm")

    sim_vehicle = _find_sim_vehicle()

    cur_res = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = _prepend_env(models_path, _prepend_env(world_path, cur_res))

    bridge_config_file = _render_bridge_config(bridge_template)

    return LaunchDescription(
        [
            # ── Arguments ────────────────────────────────────────────
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_slam", default_value="true"),
            DeclareLaunchArgument("use_nav", default_value="true",
                                 description="Run tunnel navigator + ArduPilot control."),
            DeclareLaunchArgument(
                "use_sitl",
                default_value="true" if sim_vehicle else "false",
                description="Launch ArduPilot SITL (requires sim_vehicle.py).",
            ),

            # ── Environment ──────────────────────────────────────────
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),

            # ── Gazebo server (gz sim = Garden) ──────────────────────
            ExecuteProcess(
                cmd=["gz", "sim", "-v4", "-s", "-r", world_sdf],
                output="screen",
            ),

            # ── Gazebo GUI (shown when headless:=false) ──────────────
            ExecuteProcess(
                cmd=["gz", "sim", "-v4", "-g"],
                output="screen",
                condition=UnlessCondition(LaunchConfiguration("headless")),
            ),

            # ── Spawn drone via gz service ───────────────────────────
            ExecuteProcess(
                cmd=[
                    "gz", "service",
                    "-s", f"/world/{WORLD_NAME}/create",
                    "--reqtype", "gz.msgs.EntityFactory",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "5000",
                    "--req",
                    f'sdf_filename: "{model_sdf}" '
                    f'name: "{MODEL_NAME}" '
                    f'pose: {{position: {{z: 0.2}}}}',
                ],
                output="screen",
            ),

            # ── ArduPilot SITL ───────────────────────────────────────
            ExecuteProcess(
                cmd=[
                    sim_vehicle or "echo",
                    "-v", "ArduCopter",
                    "--model", "json",
                    "--add-param-file", tunnel_parm,
                    "--enable-DDS",
                    "-I0",
                    "--no-rebuild",
                    "--no-mavproxy",
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_sitl")),
            ),

            # ── ROS <-> Gazebo bridge ────────────────────────────────
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[{"config_file": bridge_config_file}],
                output="screen",
            ),

            # ── LiDAR scan reframe (frame_id -> base_link) ──────────
            Node(
                package="uav_tunnel_nav",
                executable="scan_reframe",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": "/scan",
                        "output_topic": "/scan_fixed",
                        "frame_id": "base_link",
                    }
                ],
                output="screen",
            ),

            # ── Relay ArduPilot /ap/tf -> /tf ────────────────────────
            Node(
                package="topic_tools",
                executable="relay",
                arguments=["/ap/tf", "/tf"],
                output="screen",
            ),

            # ── SLAM (optional) ──────────────────────────────────────
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[slam_config],
                remappings=[("/scan", "/scan_fixed")],
                condition=IfCondition(LaunchConfiguration("use_slam")),
                output="screen",
            ),

            # ── Static TF: map -> odom (fallback when SLAM is off) ──
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "1", "map", "odom"],
                condition=UnlessCondition(LaunchConfiguration("use_slam")),
                output="screen",
            ),

            # ── ArduPilot MAVLink control ─────────────────────────────
            Node(
                package="uav_tunnel_nav",
                executable="ardupilot_control",
                parameters=[
                    {
                        "use_sim_time": True,
                        "connection": "tcp:127.0.0.1:5760",
                        "target_altitude": 1.0,
                        "takeoff_speed": 0.5,
                        "cmd_vel_topic": "/cmd_vel",
                    }
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_nav")),
            ),

            # ── LiDAR wall-following navigator ───────────────────────
            Node(
                package="uav_tunnel_nav",
                executable="tunnel_navigator",
                parameters=[
                    {
                        "use_sim_time": True,
                        "scan_topic": "/scan",
                        "cmd_vel_topic": "/cmd_vel",
                        "takeoff_seconds": 0.0,
                        "forward_speed": 0.4,
                        "turn_speed": 0.4,
                        "safe_distance": 1.5,
                        "side_clearance": 0.8,
                        "center_gain": 0.6,
                        "center_deadband": 0.1,
                    }
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_nav")),
            ),

            # ── RViz ─────────────────────────────────────────────────
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
