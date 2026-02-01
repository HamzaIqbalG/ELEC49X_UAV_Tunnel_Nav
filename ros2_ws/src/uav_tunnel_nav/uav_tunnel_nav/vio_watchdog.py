import sys
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class VioWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("vio_watchdog")
        self.declare_parameter("vio_topic", "/uav0/vio/odom")
        self.declare_parameter("startup_timeout_sec", 3.0)
        self.declare_parameter("timeout_sec", 0.5)

        self.vio_topic = (
            self.get_parameter("vio_topic").get_parameter_value().string_value
        )
        self.startup_timeout = (
            self.get_parameter("startup_timeout_sec").get_parameter_value().double_value
        )
        self.timeout_sec = (
            self.get_parameter("timeout_sec").get_parameter_value().double_value
        )

        self.last_msg_time: Optional[float] = None
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.failed = False

        self.create_subscription(
            Odometry, self.vio_topic, self.on_vio, qos_profile_sensor_data
        )
        self.create_timer(0.2, self.on_timer)

    def on_vio(self, msg: Odometry) -> None:
        self.last_msg_time = self.get_clock().now().nanoseconds * 1e-9

    def on_timer(self) -> None:
        if self.failed:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_msg_time is None:
            if now - self.start_time > self.startup_timeout:
                self._fail("VIO odometry not received within startup timeout.")
            return
        if now - self.last_msg_time > self.timeout_sec:
            self._fail("VIO odometry stopped publishing.")

    def _fail(self, reason: str) -> None:
        self.failed = True
        self.get_logger().fatal(
            f"{reason} Topic: {self.vio_topic}. EKF cannot correct drift without VIO."
        )
        rclpy.shutdown()
        sys.exit(2)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = VioWatchdog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
