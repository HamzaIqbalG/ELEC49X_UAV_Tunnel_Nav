#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

class TunnelNav : public rclcpp::Node
{
public:
	TunnelNav() : Node("tunnel_nav")
	{
		rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
		qos_profile.best_effort();
		qos_profile.durability_volatile();

		// Publishers
		offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos_profile);
		trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos_profile);
		vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_profile);
        
        // Visual Odometry Publisher (for GPS-denied flight)
        vehicle_visual_odometry_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", qos_profile);

        // Subscribers
        vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos_profile, 
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                nav_state_ = msg->nav_state;
                arming_state_ = msg->arming_state;
            });

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, 
            std::bind(&TunnelNav::scan_callback, this, std::placeholders::_1));
            
        // Odometry Subscriber (Ground Truth from Gazebo)
        // Topic matches the bridge output in launch file
        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/x500_lidar_custom/odometry", 10,
            std::bind(&TunnelNav::odometry_callback, this, std::placeholders::_1));

		// Control Loop Timer (20Hz)
		timer_ = this->create_wall_timer(50ms, std::bind(&TunnelNav::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Tunnel Navigation Node Started");
	}

private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_received_ = true;
        // Forward Gazebo Odometry to PX4 VehicleVisualOdometry
        px4_msgs::msg::VehicleOdometry visual_odom{};
        
        visual_odom.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        visual_odom.timestamp_sample = visual_odom.timestamp;
        
        // Position (NED)
        // Gazebo is ENU (East-North-Up). PX4 is NED (North-East-Down).
        // Standard conversion:
        // X (PX4 North) = Y (Gazebo North)
        // Y (PX4 East)  = X (Gazebo East)
        // Z (PX4 Down)  = -Z (Gazebo Up)
        // Wait, Gazebo world frame: X=Forward/East? Y=Left/North? 
        // In our tunnel world:
        // X is along the tunnel?
        // Let's assume standard ENU -> NED conversion for now.
        // X_ned = Y_enu
        // Y_ned = X_enu
        // Z_ned = -Z_enu
        
        // Actually, for Visual Odometry, PX4 expects it in the Local Frame (FRD or NED).
        // If we send it as "Local Frame", we need to match the frame convention.
        
        visual_odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        visual_odom.position = {
            (float)msg->pose.pose.position.y,
            (float)msg->pose.pose.position.x,
            -(float)msg->pose.pose.position.z
        };
        
        // Orientation (Quaternion)
        // ENU to NED rotation needed?
        // Q_ned = [y, x, -z, w] ? No, that's not right.
        // Simple mapping:
        // q_ned.w = q_enu.w
        // q_ned.x = q_enu.y
        // q_ned.y = q_enu.x
        // q_ned.z = -q_enu.z
        
        visual_odom.q = {
            (float)msg->pose.pose.orientation.w,
            (float)msg->pose.pose.orientation.y,
            (float)msg->pose.pose.orientation.x,
            -(float)msg->pose.pose.orientation.z
        };
        
        // Velocity (NED)
        visual_odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
        visual_odom.velocity = {
            (float)msg->twist.twist.linear.y,
            (float)msg->twist.twist.linear.x,
            -(float)msg->twist.twist.linear.z
        };
        
        visual_odom.angular_velocity = {
            (float)msg->twist.twist.angular.y,
            (float)msg->twist.twist.angular.x,
            -(float)msg->twist.twist.angular.z
        };
        
        vehicle_visual_odometry_publisher_->publish(visual_odom);

        RCLCPP_INFO(this->get_logger(), "OdomIn: [x:%.2f, y:%.2f, z:%.2f] -> VisOdomOut: [n:%.2f, e:%.2f, d:%.2f]", 
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
            visual_odom.position[0], visual_odom.position[1], visual_odom.position[2]);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    // ... existing scan_callback ...

    {
        // Simple sector logic
        // LaserScan angles are usually -PI to PI. 0 is forward.
        // Left Sector: 45 to 135 degrees (approx 0.78 to 2.35 rad)
        // Right Sector: -45 to -135 degrees (approx -0.78 to -2.35 rad)
        // Front Sector: -20 to 20 degrees

        float min_left = 100.0;
        float min_right = 100.0;
        float min_front = 100.0;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float range = msg->ranges[i];
            if (range < msg->range_min || range > msg->range_max) continue;

            float angle = msg->angle_min + i * msg->angle_increment;

            // Normalize angle to -PI to PI if needed (standard ROS scan is usually already normalized)
            
            if (angle > 0.78 && angle < 2.35) {
                if (range < min_left) min_left = range;
            } else if (angle < -0.78 && angle > -2.35) {
                if (range < min_right) min_right = range;
            } else if (angle > -0.35 && angle < 0.35) {
                if (range < min_front) min_front = range;
            }
        }

        // Calculate Error (Positive = Too far Right, need to move Left)
        // If Left is 2m and Right is 4m, Error = 2 - 4 = -2. 
        // We want vy to be positive (move right)? No.
        // Coordinate system: Y is Right (in Body NED? No, Body FRD).
        // PX4 Body Frame: X Forward, Y Right, Z Down.
        // If we are too close to Left (min_left < min_right), we want to move Right (+Y).
        // Error = min_right - min_left.
        // If Right=4, Left=2 -> Error = 2. We want +Y velocity.
        
        float error = min_right - min_left;
        
        // PID Control for Lateral Velocity (vy)
        // Simple P-controller for now
        float kp = 1.0; 
        float vy_target = kp * error;

        // Clamp velocity
        vy_target = std::max(-1.0f, std::min(1.0f, vy_target));
        
        RCLCPP_DEBUG(this->get_logger(), "Scan Analysis: L=%.2f, R=%.2f, F=%.2f | Error=%.2f | Vy_target=%.2f", 
            min_left, min_right, min_front, error, vy_target);
        
        // Forward Velocity (vx)
        float vx_target = 0.5; // Slow forward speed
        
        // Safety Stop
        if (min_front < 1.5) {
            vx_target = 0.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Obstacle Ahead! Stopping.");
        }

        // Store for logging
        last_min_left_ = min_left;
        last_min_right_ = min_right;
        last_min_front_ = min_front;
        last_error_ = error;

        // Update commands
        velocity_command_[0] = vx_target;
        velocity_command_[1] = vy_target;
        velocity_command_[2] = 0.0; // Hold altitude handled by position setpoint usually, but in velocity mode we might drift?
        // Actually for velocity control in PX4, Z velocity 0 means hold altitude if in Velocity control mode? 
        // Or we should use Position control for Z and Velocity for XY.
        // For simplicity, let's try pure velocity control first. Z=0 means maintain current altitude rate (0).
        // Better: Use TrajectorySetpoint with position for Z and velocity for XY? 
        // Let's stick to pure velocity for now, but we need to ensure we took off first.
    }

	void timer_callback()
	{
        // 0. Always publish offboard control mode and setpoints (required @ >2Hz)
		publish_offboard_control_mode();

        // Logic to decide what setpoint to send
        bool takeoff_complete = (offboard_setpoint_counter_ >= 200); // Allow more time for takeoff
        
        if (!takeoff_complete) {
            // Takeoff phase: Hold position at 1.5m
            publish_trajectory_setpoint(0.0, 0.0, -1.5, true); 
        } else {
            // Navigation phase: Velocity control
            publish_trajectory_setpoint(velocity_command_[0], velocity_command_[1], 0.0, false);
        }

        // 1. Check for Odometry
        if (!odom_received_) {
            if (offboard_setpoint_counter_ % 20 == 0) {
                RCLCPP_WARN(this->get_logger(), "Waiting for Odometry data...");
            }
            return; // Don't attempt to arm yet
        }

        // 2. Arming and Mode Switching Logic
        // We use the counter to delay initial attempts slightly, then retry periodically
        
        if (offboard_setpoint_counter_ > 10) {
            // Retry Arming if not armed
            if (arming_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
                 if (offboard_setpoint_counter_ % 20 == 0) { // Retry every 1s (20 * 50ms)
                    this->arm();
                    RCLCPP_INFO(this->get_logger(), "Attempting to ARM...");
                 }
            }
            
            // Retry Offboard Mode if not in Offboard
            // Nav state 14 is OFFBOARD
            if (nav_state_ != 14) {
                if (offboard_setpoint_counter_ % 20 == 0) {
                    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                    RCLCPP_INFO(this->get_logger(), "Attempting to switch to OFFBOARD...");
                }
            }
        }

		// Log status every second
		if (offboard_setpoint_counter_ % 20 == 0) {
			RCLCPP_INFO(this->get_logger(), "Status: [Nav:%d, Arm:%d] | Scan: [L:%.2f, R:%.2f, F:%.2f] | Err: %.2f | Cmd: [vx:%.2f, vy:%.2f]", 
                nav_state_, arming_state_, 
                last_min_left_, last_min_right_, last_min_front_, 
                last_error_, 
                velocity_command_[0], velocity_command_[1]);
		}

		// Increment counter
		if (offboard_setpoint_counter_ < 500) {
			offboard_setpoint_counter_++;
		}
	}

	void arm()
	{
		publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
		RCLCPP_INFO(this->get_logger(), "Arm command sent");
	}

	void publish_offboard_control_mode()
	{
		px4_msgs::msg::OffboardControlMode msg{};
		msg.position = (offboard_setpoint_counter_ < 200); // True during takeoff
		msg.velocity = (offboard_setpoint_counter_ >= 200); // True during nav
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		offboard_control_mode_publisher_->publish(msg);
	}

	void publish_trajectory_setpoint(float x, float y, float z, bool is_position)
	{
		px4_msgs::msg::TrajectorySetpoint msg{};
        
        // Initialize with NAN to ignore unused fields
        msg.position = {NAN, NAN, NAN};
        msg.velocity = {NAN, NAN, NAN};
        msg.acceleration = {NAN, NAN, NAN};
        msg.jerk = {NAN, NAN, NAN};
        msg.yaw = NAN;
        msg.yawspeed = NAN;

        if (is_position) {
            msg.position = {x, y, z};
            msg.yaw = 0.0; // Face East
        } else {
            // Velocity control
            msg.velocity = {x, y, NAN};
            msg.position = {NAN, NAN, -1.5}; // Hold 1.5m height (Hybrid control)
            msg.yaw = 0.0;
        }
        
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		trajectory_setpoint_publisher_->publish(msg);
	}

	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
	{
		px4_msgs::msg::VehicleCommand msg{};
		msg.param1 = param1;
		msg.param2 = param2;
		msg.command = command;
		msg.target_system = 1;
		msg.target_component = 1;
		msg.source_system = 1;
		msg.source_component = 1;
		msg.from_external = true;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		vehicle_command_publisher_->publish(msg);
	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_visual_odometry_publisher_;
    
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;

	uint64_t offboard_setpoint_counter_ = 0;
    uint8_t nav_state_ = 0;
    uint8_t arming_state_ = 0;
    bool odom_received_ = false;
    
    float velocity_command_[3] = {0.0, 0.0, 0.0};
    
    // Debugging variables
    float last_min_left_ = 0.0;
    float last_min_right_ = 0.0;
    float last_min_front_ = 0.0;
    float last_error_ = 0.0;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TunnelNav>());
	rclcpp::shutdown();
	return 0;
}
