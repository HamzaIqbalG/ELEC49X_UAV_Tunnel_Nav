from setuptools import setup

package_name = "uav_tunnel_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/bringup.launch.py",
                "launch/bringup_as2.launch.py",
                "launch/bringup_ardupilot.launch.py",
            ],
        ),
        (
            f"share/{package_name}/config",
            [
                "config/ekf.yaml",
                "config/slam_toolbox.yaml",
                "config/iris_bridge.yaml",
                "config/gazebo-iris-tunnel.parm",
            ],
        ),
        (f"share/{package_name}/rviz", ["rviz/uav_tunnel.rviz"]),
    ],
    install_requires=["setuptools", "pymavlink"],
    zip_safe=True,
    maintainer="UAV Tunnel",
    maintainer_email="dev@example.com",
    description="ROS 2 tunnel navigation for a Gazebo Fortress UAV demo.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tunnel_navigator = uav_tunnel_nav.tunnel_navigator:main",
            "scan_reframe = uav_tunnel_nav.scan_reframe:main",
            "basic_odometry = uav_tunnel_nav.basic_odometry:main",
            "baro_to_pose = uav_tunnel_nav.baro_to_pose:main",
            "mag_to_yaw = uav_tunnel_nav.mag_to_yaw:main",
            "imu_reframe = uav_tunnel_nav.imu_reframe:main",
            "vio_watchdog = uav_tunnel_nav.vio_watchdog:main",
            "gt_to_vio_odom = uav_tunnel_nav.gt_to_vio_odom:main",
            "ardupilot_control = uav_tunnel_nav.ardupilot_control:main",
            "external_nav_bridge = uav_tunnel_nav.external_nav_bridge:main",
        ],
    },
)
