#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		// Publishers
		offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        // Subscribers
        vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", 10, 
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                nav_state_ = msg->nav_state;
                arming_state_ = msg->arming_state;
            });

		// Timer
		timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Offboard Control Node Started");
	}

private:
	void timer_callback()
	{
		if (offboard_setpoint_counter_ == 10) {
			// Change to Offboard mode after 10 setpoints
			this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

			// Arm the vehicle
			this->arm();
		}

		// Publish offboard control mode
		publish_offboard_control_mode();

		// Publish trajectory setpoint (Hover at 5m)
		publish_trajectory_setpoint();

		// Stop the counter after reaching 11
		if (offboard_setpoint_counter_ < 11) {
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
		msg.position = true;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		offboard_control_mode_publisher_->publish(msg);
	}

	void publish_trajectory_setpoint()
	{
		px4_msgs::msg::TrajectorySetpoint msg{};
		msg.position = {0.0, 0.0, -5.0}; // NED coordinates (Negative Z is Up)
		msg.yaw = -3.14; // Face South
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
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;

	std::atomic<uint64_t> timestamp_;
	uint64_t offboard_setpoint_counter_ = 0;
    uint8_t nav_state_ = 0;
    uint8_t arming_state_ = 0;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());
	rclcpp::shutdown();
	return 0;
}
