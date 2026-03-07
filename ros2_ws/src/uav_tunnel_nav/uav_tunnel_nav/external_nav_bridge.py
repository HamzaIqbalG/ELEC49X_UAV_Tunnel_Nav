"""External navigation bridge: SLAM TF -> ArduPilot VISION_POSITION_ESTIMATE.

Reads the SLAM-produced map->base_link transform from the ROS 2 TF tree
and forwards it to ArduPilot SITL as MAVLink VISION_POSITION_ESTIMATE
messages, enabling GPS-denied EKF3 position/yaw estimation.

Frame convention:
  ROS TF uses ENU/FLU (+x East/Forward, +y North/Left, +z Up).
  ArduPilot VISION_POSITION_ESTIMATE uses NED (+x North, +y East, +z Down).
  Conversion: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
              roll_ned = roll_flu, pitch_ned = -pitch_flu, yaw_ned = -yaw_flu + pi/2
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped

from pymavlink import mavutil


def _quat_to_euler(qx: float, qy: float, qz: float, qw: float):
    """Return (roll, pitch, yaw) in radians from a quaternion."""
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def _enu_to_ned(x_enu: float, y_enu: float, z_enu: float):
    """Convert ENU position to NED position."""
    return y_enu, x_enu, -z_enu


def _enu_euler_to_ned_euler(roll_enu: float, pitch_enu: float, yaw_enu: float):
    """Convert ENU Euler angles to NED Euler angles.

    In ENU, yaw=0 points East. In NED, yaw=0 points North.
    The offset is -pi/2 (or equivalently, NED yaw = pi/2 - ENU yaw).
    """
    roll_ned = roll_enu
    pitch_ned = -pitch_enu
    yaw_ned = -(yaw_enu - math.pi / 2.0)
    # Normalise to (-pi, pi]
    yaw_ned = (yaw_ned + math.pi) % (2 * math.pi) - math.pi
    return roll_ned, pitch_ned, yaw_ned


class ExternalNavBridge(Node):
    """ROS 2 node: publishes SLAM pose as MAVLink VISION_POSITION_ESTIMATE."""

    def __init__(self) -> None:
        super().__init__("external_nav_bridge")

        self.declare_parameter("connection", "tcp:127.0.0.1:5762")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        # Covariance (m^2 for position, rad^2 for angles). Conservative start values.
        self.declare_parameter("pos_covariance", 0.1)
        self.declare_parameter("yaw_covariance", 0.05)

        self._conn_str = self.get_parameter("connection").get_parameter_value().string_value
        self._rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self._map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self._base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self._pos_cov = self.get_parameter("pos_covariance").get_parameter_value().double_value
        self._yaw_cov = self.get_parameter("yaw_covariance").get_parameter_value().double_value

        self._mav = None
        self._mav_lock = threading.Lock()
        self._msgs_sent = 0

        # TF listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Connect to ArduPilot in background so TF setup is not blocked
        threading.Thread(target=self._connect_loop, daemon=True).start()

        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            f"ExternalNavBridge starting — MAVLink at {self._conn_str}, "
            f"{self._rate:.0f} Hz, frames: {self._map_frame} -> {self._base_frame}"
        )

    # ── MAVLink connection ────────────────────────────────────────────

    def _connect_loop(self) -> None:
        while rclpy.ok() and self._mav is None:
            try:
                self.get_logger().info(f"Connecting to ArduPilot at {self._conn_str} ...")
                conn = mavutil.mavlink_connection(self._conn_str)
                conn.wait_heartbeat(timeout=30)
                with self._mav_lock:
                    self._mav = conn
                self.get_logger().info(
                    f"ExternalNavBridge connected (sysid={conn.target_system} "
                    f"compid={conn.target_component})"
                )
            except Exception as exc:
                self.get_logger().warn(f"ExternalNavBridge connect failed: {exc} — retry in 3 s")
                time.sleep(3)

    # ── Main tick ────────────────────────────────────────────────────

    def _tick(self) -> None:
        with self._mav_lock:
            mav = self._mav
        if mav is None:
            return

        # Look up the map -> base_link transform
        try:
            tf: TransformStamped = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            if self._msgs_sent == 0:
                self.get_logger().warn(
                    f"TF {self._map_frame}->{self._base_frame} not yet available: {exc}"
                )
            return

        t = tf.transform.translation
        r = tf.transform.rotation

        # Convert ROS quaternion to Euler in ENU frame
        roll_enu, pitch_enu, yaw_enu = _quat_to_euler(r.x, r.y, r.z, r.w)

        # Convert ENU position to NED
        x_ned, y_ned, z_ned = _enu_to_ned(t.x, t.y, t.z)

        # Convert ENU Euler to NED Euler
        roll_ned, pitch_ned, yaw_ned = _enu_euler_to_ned_euler(roll_enu, pitch_enu, yaw_enu)

        # Timestamp in microseconds (use ROS time)
        usec = int(self.get_clock().now().nanoseconds / 1000)

        # Build covariance array (upper-triangular, 21 elements):
        # ArduPilot uses 6 DOF: x, y, z, roll, pitch, yaw
        cov = [0.0] * 21
        cov[0] = self._pos_cov   # x var
        cov[6] = self._pos_cov   # y var
        cov[11] = self._pos_cov  # z var
        cov[15] = self._yaw_cov  # roll var
        cov[18] = self._yaw_cov  # pitch var
        cov[20] = self._yaw_cov  # yaw var

        try:
            mav.mav.vision_position_estimate_send(
                usec,
                x_ned,
                y_ned,
                z_ned,
                roll_ned,
                pitch_ned,
                yaw_ned,
                cov,
                0,  # reset_counter
            )
            self._msgs_sent += 1

            if self._msgs_sent == 1:
                self.get_logger().info(
                    f"First VISION_POSITION_ESTIMATE sent: "
                    f"pos=({x_ned:.2f}, {y_ned:.2f}, {z_ned:.2f}) m  "
                    f"yaw={math.degrees(yaw_ned):.1f} deg"
                )
            elif self._msgs_sent % 200 == 0:
                self.get_logger().info(
                    f"VPE #{self._msgs_sent}: "
                    f"pos=({x_ned:.2f}, {y_ned:.2f}, {z_ned:.2f}) m  "
                    f"yaw={math.degrees(yaw_ned):.1f} deg"
                )
        except Exception as exc:
            self.get_logger().warn(f"MAVLink send error: {exc}")
            with self._mav_lock:
                self._mav = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExternalNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
