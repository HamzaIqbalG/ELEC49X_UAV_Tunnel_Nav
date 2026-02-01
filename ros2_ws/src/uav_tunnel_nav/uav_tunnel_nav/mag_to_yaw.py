import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField


class MagToYaw(Node):
    def __init__(self) -> None:
        super().__init__("mag_to_yaw")
        self.declare_parameter("mag_topic", "/uav0/sensor_measurements/magnetometer")
        self.declare_parameter("imu_topic", "/uav0/sensor_measurements/imu")
        self.declare_parameter("pose_topic", "/uav0/mag/yaw")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("yaw_covariance", 0.3)
        self.declare_parameter("non_yaw_covariance", 1e6)
        self.declare_parameter("use_tilt_compensation", True)

        self.mag_topic = (
            self.get_parameter("mag_topic").get_parameter_value().string_value
        )
        self.imu_topic = (
            self.get_parameter("imu_topic").get_parameter_value().string_value
        )
        self.pose_topic = (
            self.get_parameter("pose_topic").get_parameter_value().string_value
        )
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.yaw_covariance = (
            self.get_parameter("yaw_covariance").get_parameter_value().double_value
        )
        self.non_yaw_covariance = (
            self.get_parameter("non_yaw_covariance").get_parameter_value().double_value
        )
        self.use_tilt_compensation = (
            self.get_parameter("use_tilt_compensation").get_parameter_value().bool_value
        )

        self.last_imu: Optional[Imu] = None

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.pose_topic, 10
        )
        self.create_subscription(
            Imu, self.imu_topic, self.on_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            MagneticField, self.mag_topic, self.on_mag, qos_profile_sensor_data
        )

    def on_imu(self, msg: Imu) -> None:
        self.last_imu = msg

    def on_mag(self, msg: MagneticField) -> None:
        mag_x = msg.magnetic_field.x
        mag_y = msg.magnetic_field.y
        mag_z = msg.magnetic_field.z

        if self.use_tilt_compensation and self.last_imu is not None:
            q = self.last_imu.orientation
            mag_x, mag_y, mag_z = self._rotate_vector((mag_x, mag_y, mag_z), (q.x, q.y, q.z, q.w))

        yaw = math.atan2(mag_y, mag_x)
        qx, qy, qz, qw = self._rpy_to_quat(0.0, 0.0, yaw)

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.frame_id
        pose.pose.pose.orientation.x = qx
        pose.pose.pose.orientation.y = qy
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw

        cov = [0.0] * 36
        cov[0] = self.non_yaw_covariance
        cov[7] = self.non_yaw_covariance
        cov[14] = self.non_yaw_covariance
        cov[21] = self.non_yaw_covariance
        cov[28] = self.non_yaw_covariance
        cov[35] = max(1e-6, self.yaw_covariance)
        pose.pose.covariance = cov

        self.pose_pub.publish(pose)

    @staticmethod
    def _rotate_vector(
        vec: Tuple[float, float, float], quat: Tuple[float, float, float, float]
    ) -> Tuple[float, float, float]:
        vx, vy, vz = vec
        qx, qy, qz, qw = quat
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        vpx = vx + qw * tx + (qy * tz - qz * ty)
        vpy = vy + qw * ty + (qz * tx - qx * tz)
        vpz = vz + qw * tz + (qx * ty - qy * tx)
        return vpx, vpy, vpz

    @staticmethod
    def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = MagToYaw()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
