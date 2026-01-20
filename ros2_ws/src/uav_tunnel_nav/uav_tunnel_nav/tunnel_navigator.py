import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class TunnelNavigator(Node):
    def __init__(self) -> None:
        super().__init__("tunnel_navigator")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("enable_arm", False)
        self.declare_parameter("arm_topic", "/gz/uav0/arm")
        self.declare_parameter("arm_seconds", 1.0)
        self.declare_parameter("takeoff_speed", 0.6)
        self.declare_parameter("takeoff_seconds", 2.5)
        self.declare_parameter("forward_speed", 0.6)
        self.declare_parameter("turn_speed", 0.6)
        self.declare_parameter("safe_distance", 1.2)
        self.declare_parameter("side_clearance", 0.6)
        self.declare_parameter("center_gain", 0.8)
        self.declare_parameter("center_deadband", 0.05)

        self.cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self.scan_topic = (
            self.get_parameter("scan_topic").get_parameter_value().string_value
        )
        self.enable_arm = (
            self.get_parameter("enable_arm").get_parameter_value().bool_value
        )
        self.arm_topic = (
            self.get_parameter("arm_topic").get_parameter_value().string_value
        )
        self.arm_seconds = (
            self.get_parameter("arm_seconds").get_parameter_value().double_value
        )
        self.takeoff_speed = (
            self.get_parameter("takeoff_speed").get_parameter_value().double_value
        )
        self.takeoff_seconds = (
            self.get_parameter("takeoff_seconds").get_parameter_value().double_value
        )
        self.forward_speed = (
            self.get_parameter("forward_speed").get_parameter_value().double_value
        )
        self.turn_speed = (
            self.get_parameter("turn_speed").get_parameter_value().double_value
        )
        self.safe_distance = (
            self.get_parameter("safe_distance").get_parameter_value().double_value
        )
        self.side_clearance = (
            self.get_parameter("side_clearance").get_parameter_value().double_value
        )
        self.center_gain = (
            self.get_parameter("center_gain").get_parameter_value().double_value
        )
        self.center_deadband = (
            self.get_parameter("center_deadband").get_parameter_value().double_value
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.arm_pub = (
            self.create_publisher(Bool, self.arm_topic, 10)
            if self.enable_arm
            else None
        )
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.on_scan, 10
        )
        self.timer = self.create_timer(0.1, self.on_timer)

        self.last_scan = None
        self.start_time = time.monotonic()

    def on_scan(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def on_timer(self) -> None:
        if self.last_scan is None:
            return

        cmd = Twist()
        elapsed = time.monotonic() - self.start_time
        if self.enable_arm and elapsed < self.arm_seconds:
            self.arm_pub.publish(Bool(data=True))
            return

        takeoff_elapsed = elapsed - self.arm_seconds if self.enable_arm else elapsed
        if takeoff_elapsed < self.takeoff_seconds:
            cmd.linear.z = self.takeoff_speed
            self.cmd_pub.publish(cmd)
            return

        ranges = self.last_scan.ranges
        count = len(ranges)
        if count == 0:
            return

        if self.last_scan.angle_increment > 0.0:
            front_min = self._min_range_by_angle(self.last_scan, -0.2, 0.2)
            right_min = self._min_range_by_angle(self.last_scan, -2.0, -1.1)
            left_min = self._min_range_by_angle(self.last_scan, 1.1, 2.0)
        else:
            window = max(1, int(count * 0.02))
            center = count // 2
            front_min = self._min_range(ranges, center - window, center + window)
            right_min = self._min_range(ranges, int(count * 0.05), int(count * 0.25))
            left_min = self._min_range(ranges, int(count * 0.75), int(count * 0.95))

        if front_min < self.safe_distance:
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed if left_min > right_min else -self.turn_speed
        else:
            cmd.linear.x = self.forward_speed
            if math.isfinite(left_min) and math.isfinite(right_min):
                error = left_min - right_min
                if abs(error) >= self.center_deadband:
                    cmd.angular.z = max(
                        -self.turn_speed,
                        min(self.turn_speed, self.center_gain * error),
                    )
            if left_min < self.side_clearance:
                cmd.angular.z = -self.turn_speed
            elif right_min < self.side_clearance:
                cmd.angular.z = self.turn_speed

        self.cmd_pub.publish(cmd)

    @staticmethod
    def _min_range(ranges, start, end) -> float:
        start = max(0, start)
        end = min(len(ranges), end)
        valid = [r for r in ranges[start:end] if math.isfinite(r)]
        return min(valid) if valid else float("inf")

    @staticmethod
    def _min_range_by_angle(scan: LaserScan, start_angle: float, end_angle: float) -> float:
        start_idx = int((start_angle - scan.angle_min) / scan.angle_increment)
        end_idx = int((end_angle - scan.angle_min) / scan.angle_increment)
        if end_idx < start_idx:
            start_idx, end_idx = end_idx, start_idx
        return TunnelNavigator._min_range(scan.ranges, start_idx, end_idx)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TunnelNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
