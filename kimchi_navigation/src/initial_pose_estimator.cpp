#include "rclcpp/rclcpp.hpp"
#include <std_srvs/srv/empty.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include <chrono>
#include <memory>
#include <vector>

// #include "kimchi_navigation/localization_movement_controller.hpp"

using namespace std::chrono_literals;

class AutoInitialPoseEstimator : public rclcpp::Node
{
public:
  AutoInitialPoseEstimator() : Node("auto_initial_pose_estimator")
  {
    global_loc_client_ = create_client<std_srvs::srv::Empty>("/reinitialize_global_localization");
    particle_sub_ = create_subscription<nav2_msgs::msg::ParticleCloud>(
      "/particle_cloud", rclcpp::SystemDefaultsQoS(), std::bind(&AutoInitialPoseEstimator::particleCallback, this, std::placeholders::_1));
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", rclcpp::SystemDefaultsQoS());

    declare_parameter("convergence_threshold", 0.3);
    convergence_threshold_ = get_parameter("convergence_threshold").as_double();

    declare_parameter("min_samples_before_convergence", 2);
    min_samples_ = get_parameter("min_samples_before_convergence").as_int();

    localization_started_ = false;
    convergence_counter_ = 0;
    converged_ = false;

    timer_ = create_wall_timer(1s, std::bind(&AutoInitialPoseEstimator::statusCheck, this));

    RCLCPP_ERROR(get_logger(), "Auto Initial Pose Estimator started. Waiting for AMCL to be ready...");
  }

private:

  void statusCheck()
  {
    if (!localization_started_) {
      RCLCPP_ERROR(get_logger(), "Triggering global localization...");
      triggerGlobalLocalization();
    } else if (converged_) {
      RCLCPP_ERROR(get_logger(), "Initial pose has been estimated and published.");
      timer_->cancel();
      // movement_controller_->shutdown();
    }
  }

  void triggerGlobalLocalization()
  {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    global_loc_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
        try {
          future.get();
          RCLCPP_ERROR(get_logger(), "Global localization triggered. Moving robot to improve localization...");
          localization_started_ = true;
          // Here you would send movement commands to the robot
          // You could implement a simple exploration pattern or just rotate in place
        } catch (const std::exception &e) {
          RCLCPP_ERROR(get_logger(), "Global localization service call failed: %s", e.what());
        }
      });
  }

  void particleCallback(const nav2_msgs::msg::ParticleCloud::SharedPtr msg)
  {
    RCLCPP_ERROR(get_logger(), "*************Processing Particle Callback");

    if (!localization_started_ || converged_) {
      return;
    }

    // Extract particle positions
    std::vector<double> positions_x;
    std::vector<double> positions_y;
    std::vector<double> orientations;

    for (const auto& particle : msg->particles) {
      positions_x.push_back(particle.pose.position.x);
      positions_y.push_back(particle.pose.position.y);
      // Convert quaternion to angle (simplified - assumes rotation around z-axis)
      orientations.push_back(2.0 * std::acos(particle.pose.orientation.w));
    }

    // Calculate mean and standard deviations
    double mean_x = 0.0, mean_y = 0.0, mean_theta = 0.0;
    double std_x = 0.0, std_y = 0.0, std_theta = 0.0;

    // Calculate means
    for (size_t i = 0; i < positions_x.size(); ++i) {
      mean_x += positions_x[i];
      mean_y += positions_y[i];
      mean_theta += orientations[i];
    }
    mean_x /= positions_x.size();
    mean_y /= positions_y.size();
    mean_theta /= orientations.size();

    // Calculate standard deviations
    for (size_t i = 0; i < positions_x.size(); ++i) {
      std_x += std::pow(positions_x[i] - mean_x, 2);
      std_y += std::pow(positions_y[i] - mean_y, 2);
      std_theta += std::pow(orientations[i] - mean_theta, 2);
    }
    std_x = std::sqrt(std_x / positions_x.size());
    std_y = std::sqrt(std_y / positions_y.size());
    std_theta = std::sqrt(std_theta / orientations.size());

    RCLCPP_ERROR(get_logger(), "Position STD: (%f, %f), Orientation STD: %f", std_x, std_y, std_theta);

    // Check if the distribution has converged
    if (std_x < convergence_threshold_ && std_y < convergence_threshold_) {
      convergence_counter_++;
      RCLCPP_ERROR(get_logger(), "Potential convergence detected (%d/%d)", convergence_counter_, min_samples_);

      if (convergence_counter_ >= min_samples_) {
        publishInitialPose(mean_x, mean_y, mean_theta);
        converged_ = true;
      }
    } else {
      convergence_counter_ = 0;
    }
  }

  void publishInitialPose(double x, double y, double theta)
  {
    auto msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
    msg->header.stamp = now();
    msg->header.frame_id = "map";

    // Set position
    msg->pose.pose.position.x = x;
    msg->pose.pose.position.y = y;
    msg->pose.pose.position.z = 0.0;

    // Set orientation (convert angle to quaternion)
    msg->pose.pose.orientation.w = std::cos(theta / 2.0);
    msg->pose.pose.orientation.z = std::sin(theta / 2.0);
    msg->pose.pose.orientation.x = 0.0;
    msg->pose.pose.orientation.y = 0.0;

    // Set covariance (diagonal elements represent uncertainty)
    // Position x, y, z then rotation x, y, z
    for (size_t i = 0; i < 36; ++i) {
      msg->pose.covariance[i] = 0.0;
    }
    msg->pose.covariance[0] = 0.1;   // x
    msg->pose.covariance[7] = 0.1;   // y
    msg->pose.covariance[35] = 0.1;  // rotation about z

    pose_pub_->publish(std::move(msg));
    RCLCPP_ERROR(get_logger(), "Published initial pose: x=%f, y=%f, theta=%f", x, y, theta);
  }

  // Service client for global localization
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr global_loc_client_;

  // Subscription to particle cloud
  rclcpp::Subscription<nav2_msgs::msg::ParticleCloud>::SharedPtr particle_sub_;


  // Publisher for initial pose
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

  // Timer for status checking
  rclcpp::TimerBase::SharedPtr timer_;

  // ROS Parameters
  double convergence_threshold_;
  int min_samples_;

  // State variables
  bool localization_started_;
  bool converged_;
  int convergence_counter_;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoInitialPoseEstimator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
