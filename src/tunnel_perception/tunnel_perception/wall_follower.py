import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        
        # QoS Profile to match Gazebo/Sensor Best Effort
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Listen to LiDAR data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Topic name
            self.lidar_callback,
            qos_profile)
            
        # Publish velocity commands (Linear X = Forward, Angular Z = Turn)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- Parameters ---
        self.target_dist = 1.0  # Target distance from wall (meters)
        self.forward_speed = 0.5 # m/s
        self.kp = 1.0 # Proportional gain for steering
        
        self.get_logger().info('Wall Follower Node Started')

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        
        # Filter out "infinite" or "zero" (invalid) readings
        ranges[ranges == float('inf')] = 10.0
        ranges[ranges == 0] = 10.0
        
        # Helper to find index for a specific angle
        def get_index_for_angle(angle_deg):
            angle_rad = np.radians(angle_deg)
            return int((angle_rad - msg.angle_min) / msg.angle_increment)

        # Calculate indices for Left (90 deg) and Right (-90 deg)
        left_idx = get_index_for_angle(90)
        right_idx = get_index_for_angle(-90)
        
        # Safety clamp indices to array bounds
        left_idx = max(0, min(left_idx, len(ranges)-1))
        right_idx = max(0, min(right_idx, len(ranges)-1))

        # Average a few points to reduce noise
        window = 10 
        left_dist = np.mean(ranges[left_idx-window : left_idx+window])
        right_dist = np.mean(ranges[right_idx-window : right_idx+window])

        self.get_logger().info(f'Left: {left_dist:.2f}m | Right: {right_dist:.2f}m')

        # Control Logic: Error = Left - Right
        error = left_dist - right_dist
        
        # Calculate steering (P-Controller)
        angular_z = self.kp * error
        
        # Cap the turning speed
        angular_z = max(-1.0, min(1.0, angular_z))

        # Publish Command
        cmd = Twist()
        cmd.linear.x = float(self.forward_speed)
        cmd.angular.z = float(angular_z) # <--- THE FIX
        
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
