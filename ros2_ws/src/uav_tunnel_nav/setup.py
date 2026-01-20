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
            ["launch/bringup.launch.py", "launch/bringup_as2.launch.py"],
        ),
        (f"share/{package_name}/rviz", ["rviz/uav_tunnel.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UAV Tunnel",
    maintainer_email="dev@example.com",
    description="ROS 2 tunnel navigation for a Gazebo Fortress UAV demo.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tunnel_navigator = uav_tunnel_nav.tunnel_navigator:main",
            "scan_reframe = uav_tunnel_nav.scan_reframe:main",
        ],
    },
)
