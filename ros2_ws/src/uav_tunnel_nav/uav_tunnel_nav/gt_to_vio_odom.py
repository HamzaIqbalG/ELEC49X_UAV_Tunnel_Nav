from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class GroundTruthToVio(Node):
    def __init__(self) -> None:
        super().__init__("gt_to_vio_odom")
        self.declare_parameter("pose_topic", "/uav0/ground_truth/pose")
        self.declare_parameter("twist_topic", "/uav0/ground_truth/twist")
        self.declare_parameter("odom_topic", "/uav0/vio/odom")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("pose_covariance", 0.02)
        self.declare_parameter("twist_covariance", 0.05)

        self.pose_topic = (
            self.get_parameter("pose_topic").get_parameter_value().string_value
        )
        self.twist_topic = (
            self.get_parameter("twist_topic").get_parameter_value().string_value
        )
        self.odom_topic = (
            self.get_parameter("odom_topic").get_parameter_value().string_value
        )
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.child_frame_id = (
            self.get_parameter("child_frame_id").get_parameter_value().string_value
        )
        self.pose_covariance = (
            self.get_parameter("pose_covariance").get_parameter_value().double_value
        )
        self.twist_covariance = (
            self.get_parameter("twist_covariance").get_parameter_value().double_value
        )

        self.last_twist: Optional[TwistStamped] = None
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        self.create_subscription(
            PoseStamped, self.pose_topic, self.on_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            TwistStamped, self.twist_topic, self.on_twist, qos_profile_sensor_data
        )

    def on_twist(self, msg: TwistStamped) -> None:
        self.last_twist = msg

    def on_pose(self, msg: PoseStamped) -> None:
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position = msg.pose.position
        odom.pose.pose.orientation = msg.pose.orientation

        if self.last_twist is not None:
            odom.twist.twist = self.last_twist.twist

        odom.pose.covariance = self._diag_covariance(self.pose_covariance)
        odom.twist.covariance = self._diag_covariance(self.twist_covariance)
        self.odom_pub.publish(odom)

    @staticmethod
    def _diag_covariance(value: float) -> list[float]:
        cov = [0.0] * 36
        value = max(1e-9, value)
        cov[0] = value
        cov[7] = value
        cov[14] = value
        cov[21] = value
        cov[28] = value
        cov[35] = value
        return cov


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = GroundTruthToVio()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
