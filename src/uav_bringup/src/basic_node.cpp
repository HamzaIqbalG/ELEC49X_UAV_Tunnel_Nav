#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>

class BasicUAVNode : public rclcpp::Node
{
public:
  BasicUAVNode()
  : Node("basic_uav_node")
  {
    // Subscribe to LiDAR data
    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/uav/scan", 10,
      std::bind(&BasicUAVNode::lidar_callback, this, std::placeholders::_1));

    // Subscribe to IMU data
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/uav/imu", 10,
      std::bind(&BasicUAVNode::imu_callback, this, std::placeholders::_1));

    // Note: X3 model uses motor speed commands, not velocity commands
    // Use motor_speed_controller for controlling the drone

    RCLCPP_INFO(this->get_logger(), "Basic UAV Node started");
    RCLCPP_INFO(this->get_logger(), "Subscribed to /uav/scan and /uav/imu");
  }

private:
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Received LiDAR scan with %zu ranges", msg->ranges.size());
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr /* msg */)
  {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Received IMU data");
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasicUAVNode>());
  rclcpp::shutdown();
  return 0;
}

