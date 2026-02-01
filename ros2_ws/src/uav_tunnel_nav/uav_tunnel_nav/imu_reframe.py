from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuReframe(Node):
    def __init__(self) -> None:
        super().__init__("imu_reframe")
        self.declare_parameter("input_topic", "/uav0/sensor_measurements/imu")
        self.declare_parameter("output_topic", "/uav0/imu/reframed")
        self.declare_parameter("frame_id", "base_link")

        self.input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        self.output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self.pub = self.create_publisher(Imu, self.output_topic, 10)
        self.create_subscription(
            Imu, self.input_topic, self.on_imu, qos_profile_sensor_data
        )

    def on_imu(self, msg: Imu) -> None:
        reframed = Imu()
        reframed.header.stamp = msg.header.stamp
        reframed.header.frame_id = self.frame_id
        reframed.orientation = msg.orientation
        reframed.orientation_covariance = msg.orientation_covariance
        reframed.angular_velocity = msg.angular_velocity
        reframed.angular_velocity_covariance = msg.angular_velocity_covariance
        reframed.linear_acceleration = msg.linear_acceleration
        reframed.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.pub.publish(reframed)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = ImuReframe()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
