import math
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, Imu, MagneticField
from tf2_ros import TransformBroadcaster


class BasicOdometry(Node):
    def __init__(self) -> None:
        super().__init__("basic_odometry")
        self.declare_parameter("imu_topic", "/uav0/sensor_measurements/imu")
        self.declare_parameter("mag_topic", "/uav0/sensor_measurements/magnetometer")
        self.declare_parameter("baro_topic", "/uav0/sensor_measurements/air_pressure")
        self.declare_parameter("vio_odom_topic", "/uav0/vio/odom")
        self.declare_parameter("odom_topic", "/uav0/basic_odom")
        self.declare_parameter("stats_topic", "/uav0/basic_odom/stats")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("vio_position_weight", 0.6)
        self.declare_parameter("vio_velocity_weight", 0.3)
        self.declare_parameter("baro_weight", 0.5)
        self.declare_parameter("mag_weight", 0.4)
        self.declare_parameter("use_gravity_comp", True)

        self.imu_topic = (
            self.get_parameter("imu_topic").get_parameter_value().string_value
        )
        self.mag_topic = (
            self.get_parameter("mag_topic").get_parameter_value().string_value
        )
        self.baro_topic = (
            self.get_parameter("baro_topic").get_parameter_value().string_value
        )
        self.vio_odom_topic = (
            self.get_parameter("vio_odom_topic").get_parameter_value().string_value
        )
        self.odom_topic = (
            self.get_parameter("odom_topic").get_parameter_value().string_value
        )
        self.stats_topic = (
            self.get_parameter("stats_topic").get_parameter_value().string_value
        )
        self.odom_frame = (
            self.get_parameter("odom_frame").get_parameter_value().string_value
        )
        self.base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )
        self.publish_tf = (
            self.get_parameter("publish_tf").get_parameter_value().bool_value
        )
        self.vio_position_weight = (
            self.get_parameter("vio_position_weight").get_parameter_value().double_value
        )
        self.vio_velocity_weight = (
            self.get_parameter("vio_velocity_weight").get_parameter_value().double_value
        )
        self.baro_weight = (
            self.get_parameter("baro_weight").get_parameter_value().double_value
        )
        self.mag_weight = (
            self.get_parameter("mag_weight").get_parameter_value().double_value
        )
        self.use_gravity_comp = (
            self.get_parameter("use_gravity_comp").get_parameter_value().bool_value
        )

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.stats_pub = self.create_publisher(DiagnosticArray, self.stats_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(Imu, self.imu_topic, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(
            MagneticField, self.mag_topic, self.on_mag, qos_profile_sensor_data
        )
        self.create_subscription(
            FluidPressure, self.baro_topic, self.on_baro, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, self.vio_odom_topic, self.on_vio, qos_profile_sensor_data
        )

        self.timer = self.create_timer(0.1, self.on_timer)

        self.last_imu: Optional[Imu] = None
        self.last_mag: Optional[MagneticField] = None
        self.last_vio: Optional[Odometry] = None
        self.pressure_ref: Optional[float] = None
        self.last_baro_z: Optional[float] = None

        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.last_position = [0.0, 0.0, 0.0]
        self.last_time = None

    def on_imu(self, msg: Imu) -> None:
        self.last_imu = msg

    def on_mag(self, msg: MagneticField) -> None:
        self.last_mag = msg

    def on_baro(self, msg: FluidPressure) -> None:
        if self.pressure_ref is None and msg.fluid_pressure > 0.0:
            self.pressure_ref = msg.fluid_pressure
        self.last_baro_z = self._pressure_to_altitude(msg.fluid_pressure)

    def on_vio(self, msg: Odometry) -> None:
        self.last_vio = msg

    def on_timer(self) -> None:
        if self.last_imu is None:
            return

        roll, pitch, yaw = self._quat_to_rpy(self.last_imu.orientation)
        if self.last_mag is not None:
            mag_yaw = math.atan2(
                self.last_mag.magnetic_field.y, self.last_mag.magnetic_field.x
            )
            yaw = self._blend_angle(yaw, mag_yaw, self.mag_weight)
        qx, qy, qz, qw = self._rpy_to_quat(roll, pitch, yaw)

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time is None:
            self.last_time = now
            return
        dt = max(1e-3, now - self.last_time)
        self.last_time = now

        acc_world = self._rotate_vector(
            (self.last_imu.linear_acceleration.x,
             self.last_imu.linear_acceleration.y,
             self.last_imu.linear_acceleration.z),
            (qx, qy, qz, qw),
        )
        ax, ay, az = acc_world
        if self.use_gravity_comp:
            az -= 9.80665

        self.velocity[0] += ax * dt
        self.velocity[1] += ay * dt
        self.velocity[2] += az * dt

        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt
        self.position[2] += self.velocity[2] * dt

        if self.last_baro_z is not None:
            self.position[2] = self._blend(
                self.position[2], self.last_baro_z, self.baro_weight
            )

        if self.last_vio is not None:
            vio_pos = self.last_vio.pose.pose.position
            self.position[0] = self._blend(
                self.position[0], vio_pos.x, self.vio_position_weight
            )
            self.position[1] = self._blend(
                self.position[1], vio_pos.y, self.vio_position_weight
            )
            self.position[2] = self._blend(
                self.position[2], vio_pos.z, self.vio_position_weight
            )
            vio_vel = self.last_vio.twist.twist.linear
            self.velocity[0] = self._blend(
                self.velocity[0], vio_vel.x, self.vio_velocity_weight
            )
            self.velocity[1] = self._blend(
                self.velocity[1], vio_vel.y, self.vio_velocity_weight
            )
            self.velocity[2] = self._blend(
                self.velocity[2], vio_vel.z, self.vio_velocity_weight
            )

        self.last_position = list(self.position)

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
        odom.twist.twist.linear.x = self.velocity[0]
        odom.twist.twist.linear.y = self.velocity[1]
        odom.twist.twist.linear.z = self.velocity[2]

        self.odom_pub.publish(odom)
        self.stats_pub.publish(self._build_stats(roll, pitch, yaw))

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
    def _blend(current: float, measurement: float, weight: float) -> float:
        weight = max(0.0, min(1.0, weight))
        return (1.0 - weight) * current + weight * measurement

    @staticmethod
    def _blend_angle(a: float, b: float, weight: float) -> float:
        weight = max(0.0, min(1.0, weight))
        delta = math.atan2(math.sin(b - a), math.cos(b - a))
        return a + weight * delta

    @staticmethod
    def _rotate_vector(vec, quat) -> tuple[float, float, float]:
        vx, vy, vz = vec
        qx, qy, qz, qw = quat
        # Quaternion rotation: v' = q * v * q^-1
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        vpx = vx + qw * tx + (qy * tz - qz * ty)
        vpy = vy + qw * ty + (qz * tx - qx * tz)
        vpz = vz + qw * tz + (qx * ty - qy * tx)
        return vpx, vpy, vpz

    def _build_stats(self, roll: float, pitch: float, yaw: float) -> DiagnosticArray:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "basic_odometry"
        status.level = DiagnosticStatus.OK
        status.message = "basic odometry running"
        status.values = [
            KeyValue(key="pos_x", value=f"{self.position[0]:.3f}"),
            KeyValue(key="pos_y", value=f"{self.position[1]:.3f}"),
            KeyValue(key="pos_z", value=f"{self.position[2]:.3f}"),
            KeyValue(key="vel_x", value=f"{self.velocity[0]:.3f}"),
            KeyValue(key="vel_y", value=f"{self.velocity[1]:.3f}"),
            KeyValue(key="vel_z", value=f"{self.velocity[2]:.3f}"),
            KeyValue(key="roll", value=f"{roll:.3f}"),
            KeyValue(key="pitch", value=f"{pitch:.3f}"),
            KeyValue(key="yaw", value=f"{yaw:.3f}"),
            KeyValue(key="baro_z", value=f"{self.last_baro_z or 0.0:.3f}"),
            KeyValue(key="vio_ok", value=str(self.last_vio is not None)),
            KeyValue(key="mag_ok", value=str(self.last_mag is not None)),
            KeyValue(key="baro_ok", value=str(self.last_baro_z is not None)),
        ]
        msg.status.append(status)
        return msg

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
