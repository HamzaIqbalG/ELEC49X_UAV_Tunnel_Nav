#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class TestNode : public rclcpp::Node
{
public:
  TestNode()
  : Node("test_node"), count_(0)
  {
    // Publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("test_topic", 10);
    
    // Subscriber
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "test_topic", 10,
      std::bind(&TestNode::topic_callback, this, std::placeholders::_1));

    // Timer to publish messages
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&TestNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Test Node started");
    RCLCPP_INFO(this->get_logger(), "Publishing to: test_topic");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: test_topic");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello from test_node! Count: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestNode>());
  rclcpp::shutdown();
  return 0;
}

