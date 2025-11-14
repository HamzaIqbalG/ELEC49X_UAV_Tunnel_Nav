#include <rclcpp/rclcpp.hpp>
#include <actuator_msgs/msg/actuators.hpp>
#include <chrono>

using namespace std::chrono_literals;

/**
 * Motor speed controller for X3 quadcopter in Gazebo Fortress
 * 
 * For Gazebo Fortress (Ignition Gazebo 6) with ROS 2 Humble:
 * - X3 model uses motor speed commands via ignition.msgs.Actuators
 * - Topic: /X3/gazebo/command/motor_speed
 * - Message: actuator_msgs/msg/Actuators (bridges to ignition.msgs.Actuators)
 * - The velocity field contains motor speeds in rad/s
 */
class MotorSpeedController : public rclcpp::Node
{
public:
  MotorSpeedController()
  : Node("motor_speed_controller")
  {
    // Declare ROS parameter for motor speed (can be changed at runtime)
    this->declare_parameter<double>("hover_speed", 600.0);
    
    // Publisher for motor speed commands
    // Publish to the Gazebo topic name - the bridge will handle conversion
    // The bridge creates a ROS topic with the same name as the Gazebo topic
    motor_publisher_ = this->create_publisher<actuator_msgs::msg::Actuators>(
      "/X3/gazebo/command/motor_speed", 10);

    // Create a timer to publish commands at 20 Hz
    timer_ = this->create_wall_timer(
      50ms,  // 20 Hz
      std::bind(&MotorSpeedController::publish_cmd, this));

    // Get initial parameter value
    hover_speed_ = this->get_parameter("hover_speed").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Motor Speed Controller started");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /X3/gazebo/command/motor_speed");
    RCLCPP_INFO(this->get_logger(), "Using actuator_msgs/msg/Actuators (bridges to ignition.msgs.Actuators)");
    RCLCPP_INFO(this->get_logger(), "Initial hover speed: %.1f rad/s", hover_speed_);
    RCLCPP_INFO(this->get_logger(), "To change speed at runtime: ros2 param set /motor_speed_controller hover_speed <value>");
  }

private:
  void publish_cmd()
  {
    // Get current parameter value (allows runtime changes)
    hover_speed_ = this->get_parameter("hover_speed").as_double();
    
    auto msg = actuator_msgs::msg::Actuators();
    
    // Set all 4 motors to same speed for hover
    // Speed can be adjusted at runtime via ROS parameter
    msg.velocity = {hover_speed_, hover_speed_, hover_speed_, hover_speed_};
    
    motor_publisher_->publish(msg);
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Publishing motor speeds: [%.1f, %.1f, %.1f, %.1f] rad/s",
                         hover_speed_, hover_speed_, hover_speed_, hover_speed_);
  }

  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double hover_speed_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorSpeedController>());
  rclcpp::shutdown();
  return 0;
}
