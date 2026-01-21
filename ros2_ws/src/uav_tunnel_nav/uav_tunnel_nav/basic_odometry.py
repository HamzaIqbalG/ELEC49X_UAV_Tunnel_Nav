import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, Imu, LaserScan, MagneticField
from tf2_ros import TransformBroadcaster


class BasicOdometry(Node):
    def __init__(self) -> None:
        super().__init__("basic_odometry")
        self.declare_parameter("imu_topic", "/uav0/sensor_measurements/imu")
        self.declare_parameter("mag_topic", "/uav0/sensor_measurements/magnetometer")
        self.declare_parameter("baro_topic", "/uav0/sensor_measurements/air_pressure")
        self.declare_parameter("lidar_topic", "/uav0/sensor_measurements/lidar/scan")
        self.declare_parameter("odom_topic", "/uav0/basic_odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("corridor_width", 5.0)
        self.declare_parameter("end_wall_x", 15.0)
        self.declare_parameter("publish_tf", True)

        self.imu_topic = (
            self.get_parameter("imu_topic").get_parameter_value().string_value
        )
        self.mag_topic = (
            self.get_parameter("mag_topic").get_parameter_value().string_value
        )
        self.baro_topic = (
            self.get_parameter("baro_topic").get_parameter_value().string_value
        )
        self.lidar_topic = (
            self.get_parameter("lidar_topic").get_parameter_value().string_value
        )
        self.odom_topic = (
            self.get_parameter("odom_topic").get_parameter_value().string_value
        )
        self.odom_frame = (
            self.get_parameter("odom_frame").get_parameter_value().string_value
        )
        self.base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )
        self.corridor_width = (
            self.get_parameter("corridor_width").get_parameter_value().double_value
        )
        self.end_wall_x = (
            self.get_parameter("end_wall_x").get_parameter_value().double_value
        )
        self.publish_tf = (
            self.get_parameter("publish_tf").get_parameter_value().bool_value
        )

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(Imu, self.imu_topic, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(
            MagneticField, self.mag_topic, self.on_mag, qos_profile_sensor_data
        )
        self.create_subscription(
            FluidPressure, self.baro_topic, self.on_baro, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, self.lidar_topic, self.on_scan, qos_profile_sensor_data
        )

        self.timer = self.create_timer(0.1, self.on_timer)

        self.last_imu: Optional[Imu] = None
        self.last_mag: Optional[MagneticField] = None
        self.last_scan: Optional[LaserScan] = None
        self.pressure_ref: Optional[float] = None

        self.position = [0.0, 0.0, 0.0]
        self.last_position = [0.0, 0.0, 0.0]
        self.last_time = time.monotonic()

    def on_imu(self, msg: Imu) -> None:
        self.last_imu = msg

    def on_mag(self, msg: MagneticField) -> None:
        self.last_mag = msg

    def on_baro(self, msg: FluidPressure) -> None:
        if self.pressure_ref is None and msg.fluid_pressure > 0.0:
            self.pressure_ref = msg.fluid_pressure
        self.position[2] = self._pressure_to_altitude(msg.fluid_pressure)

    def on_scan(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def on_timer(self) -> None:
        if self.last_imu is None or self.last_scan is None:
            return

        front_range = self._min_range_by_angle(self.last_scan, -0.2, 0.2)
        left_range = self._min_range_by_angle(self.last_scan, 1.1, 2.0)
        right_range = self._min_range_by_angle(self.last_scan, -2.0, -1.1)

        if math.isfinite(front_range):
            self.position[0] = self.end_wall_x - front_range

        if math.isfinite(left_range) and math.isfinite(right_range):
            self.position[1] = (right_range - left_range) * 0.5

        roll, pitch, yaw = self._quat_to_rpy(self.last_imu.orientation)
        if self.last_mag is not None:
            yaw = math.atan2(
                self.last_mag.magnetic_field.y, self.last_mag.magnetic_field.x
            )
        qx, qy, qz, qw = self._rpy_to_quat(roll, pitch, yaw)

        now = time.monotonic()
        dt = max(1e-3, now - self.last_time)
        vx = (self.position[0] - self.last_position[0]) / dt
        vy = (self.position[1] - self.last_position[1]) / dt
        vz = (self.position[2] - self.last_position[2]) / dt

        self.last_position = list(self.position)
        self.last_time = now

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = self.position[2]
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz

        self.odom_pub.publish(odom)

        if self.tf_broadcaster:
            tf_msg = TransformStamped()
            tf_msg.header = odom.header
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.position[0]
            tf_msg.transform.translation.y = self.position[1]
            tf_msg.transform.translation.z = self.position[2]
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)

    def _pressure_to_altitude(self, pressure: float) -> float:
        if self.pressure_ref is None or pressure <= 0.0:
            return self.position[2]
        return 44330.0 * (1.0 - (pressure / self.pressure_ref) ** (1.0 / 5.255))

    @staticmethod
    def _min_range_by_angle(scan: LaserScan, start_angle: float, end_angle: float) -> float:
        if scan.angle_increment == 0.0:
            return float("inf")
        start_idx = int((start_angle - scan.angle_min) / scan.angle_increment)
        end_idx = int((end_angle - scan.angle_min) / scan.angle_increment)
        if end_idx < start_idx:
            start_idx, end_idx = end_idx, start_idx
        start_idx = max(0, start_idx)
        end_idx = min(len(scan.ranges), end_idx)
        valid = [r for r in scan.ranges[start_idx:end_idx] if math.isfinite(r)]
        return min(valid) if valid else float("inf")

    @staticmethod
    def _quat_to_rpy(q) -> tuple[float, float, float]:
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

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
    node = BasicOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
