#include "kimchi_navigation/localization_movement_controller.hpp"

#include "rclcpp/rclcpp.hpp"


  LocalizationMovementController::LocalizationMovementController()
    : Node("localization_movement_controller")
  {
    // Publisher for velocity commands
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Parameters
    declare_parameter("linear_speed", 0.1);
    declare_parameter("angular_speed", 0.3);
    declare_parameter("movement_pattern", "rotate_and_advance");

    linear_speed_ = get_parameter("linear_speed").as_double();
    angular_speed_ = get_parameter("angular_speed").as_double();
    pattern_ = get_parameter("movement_pattern").as_string();

    // Start time for movement patterns
    start_time_ = now();

    // Timer for executing movement pattern
    timer_ = create_wall_timer(100ms, std::bind(&LocalizationMovementController::executeMovementPattern, this));

    RCLCPP_ERROR(get_logger(), "Starting %s movement pattern for localization", pattern_.c_str());
  }

  LocalizationMovementController::~LocalizationMovementController()
  {
    // Make sure to stop the robot before shutting down
    auto stop_cmd = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel_pub_->publish(std::move(stop_cmd));
  }

  void LocalizationMovementController::executeMovementPattern()
  {
    auto current_time = now();
    double elapsed = (current_time - start_time_).seconds();

    auto cmd = std::make_unique<geometry_msgs::msg::Twist>();

    if (pattern_ == "rotate") {
      // Simple rotation in place
      cmd->angular.z = angular_speed_;

    } else if (pattern_ == "spiral") {
      // Spiral pattern: increasing radius
      double factor = std::min(1.0, elapsed / 30.0);  // Increase radius over 30 seconds
      cmd->linear.x = linear_speed_ * factor;
      cmd->angular.z = angular_speed_ * (1.0 - 0.5 * factor);

    } else if (pattern_ == "zigzag") {
      // Zigzag pattern
      double phase_duration = 3.0;
      int phase = static_cast<int>(elapsed / phase_duration) % 4;

      if (phase == 0) {
        cmd->linear.x = linear_speed_;
      } else if (phase == 1) {
        cmd->angular.z = angular_speed_;
      } else if (phase == 2) {
        cmd->linear.x = linear_speed_;
      } else {
        cmd->angular.z = -angular_speed_;
      }

    } else if (pattern_ == "rotate_and_advance") {
      // Rotate a bit, then move forward, repeat
      double phase_duration = 4.0;
      int phase = static_cast<int>(elapsed / phase_duration) % 2;
      double phase_time = std::fmod(elapsed, phase_duration);

      if (phase == 0) {
        // First rotate for 3 seconds
        int random_phase_time_limit = std::rand() % 4;
        if (phase_time < random_phase_time_limit) {
          cmd->angular.z = angular_speed_;
        }
      } else {
        // Then move forward for 1 second
        if (phase_time < 2.0) {
          cmd->linear.x = linear_speed_;
        }
      }
    }

    cmd_vel_pub_->publish(std::move(cmd));
  }

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocalizationMovementController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
