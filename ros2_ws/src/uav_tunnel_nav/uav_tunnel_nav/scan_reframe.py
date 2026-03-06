from typing import Optional
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanReframe(Node):
    def __init__(self) -> None:
        super().__init__("scan_reframe")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_fixed")
        self.declare_parameter("frame_id", "base_link")

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.frame_id = self.get_parameter(
            "frame_id"
        ).get_parameter_value().string_value

        self._log_path = Path("/ros2_ws/src/uav_tunnel_nav/debug-967540.log")

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(LaserScan, output_topic, reliable_qos)
        self.sub = self.create_subscription(
            LaserScan, input_topic, self.on_scan, qos_profile_sensor_data
        )

    # #region agent log
    def _agent_log(self, hypothesis_id: str, msg: str, data: dict) -> None:
        try:
            payload = {
                "sessionId": "967540",
                "runId": "pre-fix",
                "hypothesisId": hypothesis_id,
                "location": "scan_reframe.py",
                "message": msg,
                "data": data,
                "timestamp": int(time.time() * 1000),
            }
            with self._log_path.open("a") as f:
                f.write(json.dumps(payload) + "\n")
        except Exception:
            pass
    # #endregion

    def on_scan(self, msg: LaserScan) -> None:
        out = LaserScan()
        out.header = msg.header
        out.header.frame_id = self.frame_id
        if out.header.stamp.sec == 0 and out.header.stamp.nanosec == 0:
            out.header.stamp = self.get_clock().now().to_msg()
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = msg.ranges
        out.intensities = msg.intensities
        self._agent_log(
            "H3",
            "SCAN_RECEIVED",
            {
                "ranges_len": len(msg.ranges),
                "stamp_sec": msg.header.stamp.sec,
            },
        )
        self.pub.publish(out)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = ScanReframe()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
