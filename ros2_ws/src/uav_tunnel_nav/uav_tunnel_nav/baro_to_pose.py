import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure


class BaroToPose(Node):
    def __init__(self) -> None:
        super().__init__("baro_to_pose")
        self.declare_parameter("baro_topic", "/uav0/sensor_measurements/air_pressure")
        self.declare_parameter("pose_topic", "/uav0/baro/pose")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("reference_pressure", 0.0)
        self.declare_parameter("z_offset", 0.0)
        self.declare_parameter("z_covariance", 0.5)
        self.declare_parameter("non_z_covariance", 1e6)

        self.baro_topic = (
            self.get_parameter("baro_topic").get_parameter_value().string_value
        )
        self.pose_topic = (
            self.get_parameter("pose_topic").get_parameter_value().string_value
        )
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.reference_pressure = (
            self.get_parameter("reference_pressure").get_parameter_value().double_value
        )
        self.z_offset = self.get_parameter("z_offset").get_parameter_value().double_value
        self.z_covariance = (
            self.get_parameter("z_covariance").get_parameter_value().double_value
        )
        self.non_z_covariance = (
            self.get_parameter("non_z_covariance").get_parameter_value().double_value
        )

        self.pressure_ref: Optional[float] = (
            self.reference_pressure if self.reference_pressure > 0.0 else None
        )

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.pose_topic, 10
        )
        self.create_subscription(
            FluidPressure, self.baro_topic, self.on_baro, qos_profile_sensor_data
        )

    def on_baro(self, msg: FluidPressure) -> None:
        if msg.fluid_pressure <= 0.0:
            return
        if self.pressure_ref is None:
            self.pressure_ref = msg.fluid_pressure

        altitude = self._pressure_to_altitude(msg.fluid_pressure) + self.z_offset

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.frame_id
        pose.pose.pose.position.z = altitude
        pose.pose.pose.orientation.w = 1.0

        cov = [0.0] * 36
        cov[0] = self.non_z_covariance
        cov[7] = self.non_z_covariance
        cov[14] = max(1e-6, self.z_covariance)
        cov[21] = self.non_z_covariance
        cov[28] = self.non_z_covariance
        cov[35] = self.non_z_covariance
        pose.pose.covariance = cov

        self.pose_pub.publish(pose)

    def _pressure_to_altitude(self, pressure: float) -> float:
        if self.pressure_ref is None or pressure <= 0.0:
            return 0.0
        return 44330.0 * (1.0 - (pressure / self.pressure_ref) ** (1.0 / 5.255))


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = BaroToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
