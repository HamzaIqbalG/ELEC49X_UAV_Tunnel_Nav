#include <rclcpp/rclcpp.hpp>
#include <actuator_msgs/msg/actuators.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

/**
 * Manual control node for X3 quadcopter
 * 
 * Reads arrow keys and converts them to motor speed commands
 * Arrow keys control:
 *   Up Arrow    - Increase altitude (all motors increase)
 *   Down Arrow  - Decrease altitude (all motors decrease)
 *   Left Arrow  - Rotate left (left motors decrease, right motors increase)
 *   Right Arrow - Rotate right (right motors decrease, left motors increase)
 *   W/A/S/D     - Forward/Left/Backward/Right movement (pitch/roll adjustments)
 *   Space       - Emergency stop (zero all motors)
 *   Q           - Quit manual control
 * 
 * Motor layout (X3):
 *   Motor 0: Front Left  (CCW)
 *   Motor 1: Front Right (CCW)
 *   Motor 2: Back Right  (CW)
 *   Motor 3: Back Left   (CW)
 */
class ManualControl : public rclcpp::Node
{
public:
  ManualControl()
  : Node("manual_control"), 
    base_speed_(662.0),  // Base hover speed
    active_(false),
    running_(true)
  {
    // Declare parameters
    this->declare_parameter<double>("base_speed", 662.0);
    this->declare_parameter<double>("speed_increment", 20.0);  // How much to change per keypress
    this->declare_parameter<double>("max_speed", 800.0);
    this->declare_parameter<double>("min_speed", 400.0);
    
    base_speed_ = this->get_parameter("base_speed").as_double();
    speed_increment_ = this->get_parameter("speed_increment").as_double();
    max_speed_ = this->get_parameter("max_speed").as_double();
    min_speed_ = this->get_parameter("min_speed").as_double();
    
    // Publisher for motor speed commands
    rclcpp::QoS qos_profile(50);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
    
    motor_publisher_ = this->create_publisher<actuator_msgs::msg::Actuators>(
      "/X3/gazebo/command/motor_speed", qos_profile);
    
    // Initialize motor speeds to base hover speed
    current_speeds_ = {base_speed_, base_speed_, base_speed_, base_speed_};
    
    // Create timer to publish commands at 50 Hz
    timer_ = this->create_wall_timer(
      20ms,  // 50 Hz
      std::bind(&ManualControl::publish_cmd, this));
    
    // Start keyboard input thread
    keyboard_thread_ = std::thread(&ManualControl::keyboard_loop, this);
    
    RCLCPP_INFO(this->get_logger(), "\n========================================");
    RCLCPP_INFO(this->get_logger(), "  MANUAL CONTROL NODE STARTED");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "IMPORTANT: Keyboard input is captured in THIS terminal");
    RCLCPP_INFO(this->get_logger(), "Make sure this terminal window has focus!");
    RCLCPP_INFO(this->get_logger(), "Base speed: %.1f rad/s", base_speed_);
    RCLCPP_INFO(this->get_logger(), "Speed increment: %.1f rad/s", speed_increment_);
    RCLCPP_INFO(this->get_logger(), "\n=== Manual Control Instructions ===");
    RCLCPP_INFO(this->get_logger(), "Arrow Keys:");
    RCLCPP_INFO(this->get_logger(), "  ↑ Up    - Increase altitude");
    RCLCPP_INFO(this->get_logger(), "  ↓ Down  - Decrease altitude");
    RCLCPP_INFO(this->get_logger(), "  ← Left  - Rotate left (yaw)");
    RCLCPP_INFO(this->get_logger(), "  → Right - Rotate right (yaw)");
    RCLCPP_INFO(this->get_logger(), "WASD Keys:");
    RCLCPP_INFO(this->get_logger(), "  W - Pitch forward (move forward)");
    RCLCPP_INFO(this->get_logger(), "  S - Pitch backward (move backward)");
    RCLCPP_INFO(this->get_logger(), "  A - Roll left (move left)");
    RCLCPP_INFO(this->get_logger(), "  D - Roll right (move right)");
    RCLCPP_INFO(this->get_logger(), "Other:");
    RCLCPP_INFO(this->get_logger(), "  Space - Emergency stop (zero motors)");
    RCLCPP_INFO(this->get_logger(), "  R     - Reset to hover speed");
    RCLCPP_INFO(this->get_logger(), "  Q     - Quit manual control");
    RCLCPP_INFO(this->get_logger(), "\n>>> Press any key in THIS terminal to start manual control...");
  }
  
  ~ManualControl()
  {
    running_ = false;
    if (keyboard_thread_.joinable()) {
      keyboard_thread_.join();
    }
    // Send zero commands on exit
    send_zero_commands();
  }

private:
  void publish_cmd()
  {
    auto msg = actuator_msgs::msg::Actuators();
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.velocity = current_speeds_;
    
    motor_publisher_->publish(msg);
    
    if (active_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                           "Manual: [%.0f, %.0f, %.0f, %.0f] rad/s",
                           current_speeds_[0], current_speeds_[1], 
                           current_speeds_[2], current_speeds_[3]);
    }
  }
  
  void keyboard_loop()
  {
    // Set terminal to raw mode for immediate key reading
    struct termios old_termios, new_termios;
    tcgetattr(STDIN_FILENO, &old_termios);
    new_termios = old_termios;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    
    // Set stdin to non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    
    char key;
    while (running_ && rclcpp::ok()) {
      if (read(STDIN_FILENO, &key, 1) > 0) {
        handle_key(key);
      }
      std::this_thread::sleep_for(10ms);
    }
    
    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &old_termios);
    fcntl(STDIN_FILENO, F_SETFL, flags);
  }
  
  void handle_key(char key)
  {
    if (!active_ && key != '\n' && key != ' ') {
      active_ = true;
      RCLCPP_INFO(this->get_logger(), "=== MANUAL CONTROL ACTIVE ===");
    }
    
    switch (key) {
      case 27:  // ESC or arrow key prefix
        // Check for arrow keys (ESC [ A/B/C/D)
        char seq[2];
        if (read(STDIN_FILENO, &seq[0], 1) > 0 && seq[0] == '[') {
          if (read(STDIN_FILENO, &seq[1], 1) > 0) {
            handle_arrow_key(seq[1]);
          }
        }
        break;
        
      case 'w':
      case 'W':
        // Pitch forward (move forward)
        adjust_pitch_forward();
        break;
        
      case 's':
      case 'S':
        // Pitch backward (move backward)
        adjust_pitch_backward();
        break;
        
      case 'a':
      case 'A':
        // Roll left (move left)
        adjust_roll_left();
        break;
        
      case 'd':
      case 'D':
        // Roll right (move right)
        adjust_roll_right();
        break;
        
      case ' ':
        // Emergency stop
        send_zero_commands();
        RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP - All motors zeroed!");
        break;
        
      case 'r':
      case 'R':
        // Reset to hover
        reset_to_hover();
        RCLCPP_INFO(this->get_logger(), "Reset to hover speed: %.1f rad/s", base_speed_);
        break;
        
      case 'q':
      case 'Q':
        // Quit
        RCLCPP_INFO(this->get_logger(), "Exiting manual control...");
        active_ = false;
        reset_to_hover();
        break;
    }
  }
  
  void handle_arrow_key(char key)
  {
    switch (key) {
      case 'A':  // Up arrow
        increase_altitude();
        break;
      case 'B':  // Down arrow
        decrease_altitude();
        break;
      case 'D':  // Left arrow
        rotate_left();
        break;
      case 'C':  // Right arrow
        rotate_right();
        break;
    }
  }
  
  void increase_altitude()
  {
    // Increase all motors equally
    for (size_t i = 0; i < 4; i++) {
      current_speeds_[i] = std::min(current_speeds_[i] + speed_increment_, max_speed_);
    }
    RCLCPP_INFO(this->get_logger(), "↑ Altitude increase");
  }
  
  void decrease_altitude()
  {
    // Decrease all motors equally
    for (size_t i = 0; i < 4; i++) {
      current_speeds_[i] = std::max(current_speeds_[i] - speed_increment_, min_speed_);
    }
    RCLCPP_INFO(this->get_logger(), "↓ Altitude decrease");
  }
  
  void rotate_left()
  {
    // Left motors decrease, right motors increase (yaw left)
    // Motors 0,3 decrease; Motors 1,2 increase
    current_speeds_[0] = std::max(current_speeds_[0] - speed_increment_, min_speed_);
    current_speeds_[3] = std::max(current_speeds_[3] - speed_increment_, min_speed_);
    current_speeds_[1] = std::min(current_speeds_[1] + speed_increment_, max_speed_);
    current_speeds_[2] = std::min(current_speeds_[2] + speed_increment_, max_speed_);
    RCLCPP_INFO(this->get_logger(), "← Rotate left");
  }
  
  void rotate_right()
  {
    // Right motors decrease, left motors increase (yaw right)
    // Motors 1,2 decrease; Motors 0,3 increase
    current_speeds_[1] = std::max(current_speeds_[1] - speed_increment_, min_speed_);
    current_speeds_[2] = std::max(current_speeds_[2] - speed_increment_, min_speed_);
    current_speeds_[0] = std::min(current_speeds_[0] + speed_increment_, max_speed_);
    current_speeds_[3] = std::min(current_speeds_[3] + speed_increment_, max_speed_);
    RCLCPP_INFO(this->get_logger(), "→ Rotate right");
  }
  
  void adjust_pitch_forward()
  {
    // Pitch forward: back motors increase, front motors decrease
    current_speeds_[2] = std::min(current_speeds_[2] + speed_increment_, max_speed_);
    current_speeds_[3] = std::min(current_speeds_[3] + speed_increment_, max_speed_);
    current_speeds_[0] = std::max(current_speeds_[0] - speed_increment_, min_speed_);
    current_speeds_[1] = std::max(current_speeds_[1] - speed_increment_, min_speed_);
    RCLCPP_INFO(this->get_logger(), "W - Pitch forward");
  }
  
  void adjust_pitch_backward()
  {
    // Pitch backward: front motors increase, back motors decrease
    current_speeds_[0] = std::min(current_speeds_[0] + speed_increment_, max_speed_);
    current_speeds_[1] = std::min(current_speeds_[1] + speed_increment_, max_speed_);
    current_speeds_[2] = std::max(current_speeds_[2] - speed_increment_, min_speed_);
    current_speeds_[3] = std::max(current_speeds_[3] - speed_increment_, min_speed_);
    RCLCPP_INFO(this->get_logger(), "S - Pitch backward");
  }
  
  void adjust_roll_left()
  {
    // Roll left: right motors increase, left motors decrease
    current_speeds_[1] = std::min(current_speeds_[1] + speed_increment_, max_speed_);
    current_speeds_[2] = std::min(current_speeds_[2] + speed_increment_, max_speed_);
    current_speeds_[0] = std::max(current_speeds_[0] - speed_increment_, min_speed_);
    current_speeds_[3] = std::max(current_speeds_[3] - speed_increment_, min_speed_);
    RCLCPP_INFO(this->get_logger(), "A - Roll left");
  }
  
  void adjust_roll_right()
  {
    // Roll right: left motors increase, right motors decrease
    current_speeds_[0] = std::min(current_speeds_[0] + speed_increment_, max_speed_);
    current_speeds_[3] = std::min(current_speeds_[3] + speed_increment_, max_speed_);
    current_speeds_[1] = std::max(current_speeds_[1] - speed_increment_, min_speed_);
    current_speeds_[2] = std::max(current_speeds_[2] - speed_increment_, min_speed_);
    RCLCPP_INFO(this->get_logger(), "D - Roll right");
  }
  
  void send_zero_commands()
  {
    current_speeds_ = {0.0, 0.0, 0.0, 0.0};
  }
  
  void reset_to_hover()
  {
    current_speeds_ = {base_speed_, base_speed_, base_speed_, base_speed_};
  }
  
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread keyboard_thread_;
  
  std::vector<double> current_speeds_;
  double base_speed_;
  double speed_increment_;
  double max_speed_;
  double min_speed_;
  bool active_;
  bool running_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Note: This node needs to run in a terminal that can receive keyboard input
  auto node = std::make_shared<ManualControl>();
  
  // Run in a separate thread to allow keyboard input
  std::thread spin_thread([node]() {
    rclcpp::spin(node);
  });
  
  // Keep main thread for keyboard input
  spin_thread.join();
  
  rclcpp::shutdown();
  return 0;
}

