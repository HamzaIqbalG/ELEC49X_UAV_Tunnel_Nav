import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)


class WallFollowerPX4(Node):
    def __init__(self):
        super().__init__('wall_follower_px4')

        self.target_dist = 1.0        # meters from left wall
        self.forward_speed = 0.5      # m/s
        self.kp = 1.2                 # wall-follow gain
        self.takeoff_altitude = -2.0  # meters (NED, negative is UP)

        self.left_dist = 10.0
        self.counter = 0

        self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.traj_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )


        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('PX4 Wall Follower started')


    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = 10.0

        def idx(deg):
            return int((np.radians(deg) - msg.angle_min) / msg.angle_increment)

        i1 = max(0, idx(80))
        i2 = min(len(ranges) - 1, idx(100))
        self.left_dist = np.mean(ranges[i1:i2])

    #timer heartbeat
    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # After ~1 second of streaming, arm + offboard
        if self.counter == 20:
            self.arm()
            self.set_offboard_mode()

        self.counter += 1

    #offboard control
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_pub.publish(msg)

    #set trajectory
    def publish_trajectory_setpoint(self):
        error = self.target_dist - self.left_dist
        vy = float(self.kp * error)

        sp = TrajectorySetpoint()

        # Velocity control (NED frame)
        sp.velocity = [
            float(self.forward_speed),  # X (forward)
            vy,                  # Y (left/right)
            0.0                  # Z (hold altitude)
        ]

        # Position Z for takeoff / altitude hold
        sp.position = [float('nan'), float('nan'), float(self.takeoff_altitude)]

        sp.yaw = float(0.0)
        sp.timestamp = self.get_clock().now().nanoseconds // 1000

        self.traj_pub.publish(sp)

    
    def arm(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=float(1.0)
        )
        self.get_logger().info('Vehicle armed')

    def set_offboard_mode(self):
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=float(1.0),
            param2=float(6.0)  # OFFBOARD
        )
        self.get_logger().info('OFFBOARD mode set')

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = int(command)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerPX4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
