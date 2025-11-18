#include "kimchi_navigation/global_localization.hpp"

// Helper to keep angles in the [0, 2PI] range
double NormalizeAngle(double angle) {
    const double TWO_PI = 2.0 * M_PI;

    // remainder() normalizes to [-PI, PI]
    double normalized = std::remainder(angle, TWO_PI);

    // If the result is negative, shift it into the [0, 2*PI] range
    if (normalized < 0.0) {
        normalized += TWO_PI;
    }

    return normalized;
}

GlobalLocalizationServer::GlobalLocalizationServer()
    : node_(new rclcpp::Node("global_localization")) {
  using namespace std::placeholders;

  auto qos_profile =
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_t{
          RMW_QOS_POLICY_HISTORY_KEEP_LAST,
          10,  // depth
          RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
          RMW_QOS_POLICY_DURABILITY_VOLATILE, RMW_QOS_DEADLINE_DEFAULT,
          RMW_QOS_LIFESPAN_DEFAULT, RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
          RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
          false  // avoid_ros_namespace_conventions
      }));

  action_server_ = rclcpp_action::create_server<GlobalLocalization>(
      node_, "global_localization",
      std::bind(&GlobalLocalizationServer::handleGoal, this, _1, _2),
      std::bind(&GlobalLocalizationServer::handleCancel, this, _1),
      std::bind(&GlobalLocalizationServer::handleAccepted, this, _1));

  // Declare parameters for convergence thresholds
  node_->declare_parameter<double>("position_covariance_threshold",
                                  0.25);  // meters^2
  node_->declare_parameter<double>("orientation_covariance_threshold",
                                  0.025);  // radians^2

  // Get the configured thresholds
  pos_uncertainty_threashold_ =
      node_->get_parameter("position_covariance_threshold").as_double();
  orientation_uncertainty_threashold_ =
      node_->get_parameter("orientation_covariance_threshold").as_double();

  initial_pose_publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", qos_profile);

  command_robot_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", qos_profile);

  // Create a subscription to the /amcl_pose topic
  // This is AMCL's output, which we will monitor for convergence.
  amcl_pose_subscription_ =
      node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/amcl_pose", 10,
          std::bind(&GlobalLocalizationServer::AmclPoseCallback, this,
                    std::placeholders::_1));

  lidar_subscriber_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 1,
            std::bind(&GlobalLocalizationServer::LidarCallback, this, std::placeholders::_1));

  scape_maneuver_ = std::make_unique<ScapeManeuver>(node_);
}

rclcpp_action::GoalResponse GlobalLocalizationServer::handleGoal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const GlobalLocalization::Goal> goal) {
  (void)uuid;
  inital_pose_estimate_x_ = goal->pose_estimate.x;
  inital_pose_estimate_y_ = goal->pose_estimate.y;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalLocalizationServer::handleCancel(
    const std::shared_ptr<GoalHandleGlobalLocalization> goal_handle) {
  (void)goal_handle;

  StopRobot();

  return rclcpp_action::CancelResponse::ACCEPT;
}

void GlobalLocalizationServer::handleAccepted(
    const std::shared_ptr<GoalHandleGlobalLocalization> goal_handle) {
  using namespace std::placeholders;

  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  std::thread{std::bind(&GlobalLocalizationServer::execute, this, _1),
              goal_handle}
      .detach();
}

void GlobalLocalizationServer::cleanup() {
  // Always stop the robot first
  StopRobot();

  // Reset state flags
  robot_localized_ = false;
  initial_pose_published_ = false;
  localization_state_ = LocalizationState::LOOKING_FOR_OBSTACLES;
  obstacles_.clear();
}

void GlobalLocalizationServer::RotateRobot() {
  geometry_msgs::msg::Twist robot_rotation;
  robot_rotation.angular.z = 1.0;

  command_robot_pub_->publish(robot_rotation);
}

void GlobalLocalizationServer::StopRobot() {
  geometry_msgs::msg::Twist robot_rotation;
  robot_rotation.angular.z = 0.0;

  command_robot_pub_->publish(robot_rotation);
}

void GlobalLocalizationServer::RobotLocalized() {
  robot_localized_ = true;
  StopRobot();
}

void GlobalLocalizationServer::execute(
    const std::shared_ptr<GoalHandleGlobalLocalization> goal_handle) {
  //  Reset state for new goal
  robot_localized_ = false;
  initial_pose_published_ = false;

  // Initialize goal execution
  PublishInitialPoseWithHighVariance();

  // Rate of 20hz
  rclcpp::Rate loop_rate(20);
  auto feedback = std::make_shared<GlobalLocalization::Feedback>();
  auto result = std::make_shared<GlobalLocalization::Result>();

  while (!robot_localized_) {
    // Check for cancellation first
    if (goal_handle->is_canceling()) {
      // Handles cleanup
      cleanup();

      result->localized_pose = current_pose_;
      result->localized = false;
      goal_handle->canceled(result);
      return;
    }

    switch (localization_state_)
    {
      case LocalizationState::LOOKING_FOR_OBSTACLES:
        CheckForObstacles();
        if (obstacles_.size() == 0 && lidar_reading_ != nullptr){
          localization_state_ = LocalizationState::LOCALIZING;
        } else if (obstacles_.size() > 0) {
          localization_state_ = LocalizationState::SCAPING_MANEUVER;
        }
        break;
      case LocalizationState::SCAPING_MANEUVER:
        scape_maneuver_->InitializeScapeManeuver(obstacles_);

        localization_state_ = LocalizationState::LOOKING_FOR_OBSTACLES;
        obstacles_.clear();
        break;
      case LocalizationState::LOCALIZING:
        obstacles_.clear();
        RotateRobot();
        feedback->pose_feedback = current_pose_;
        feedback->current_uncertainty[0] = current_position_uncertainty_;
        feedback->current_uncertainty[1] = current_orientation_uncertainty_;
        goal_handle->publish_feedback(feedback);
        break;
      default:
        break;
    }

    loop_rate.sleep();
  }


  // Check if we exited due to successful localization
  if (rclcpp::ok() && robot_localized_) {
    // Handles cleanup
    cleanup();

    result->localized_pose = current_pose_;
    result->localized = true;
    goal_handle->succeed(result);
  } else {
    // Handles cleanup
    cleanup();

    result->localized_pose = current_pose_;
    result->localized = false;
    goal_handle->abort(result);
  }
}

void GlobalLocalizationServer::PublishInitialPoseWithHighVariance() {
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_estimate;
  initial_pose_estimate.header.frame_id = "map";
  initial_pose_estimate.pose.pose.position.x = inital_pose_estimate_x_;
  initial_pose_estimate.pose.pose.position.y = inital_pose_estimate_y_;

  // Sets high covariance for x and y position
  initial_pose_estimate.pose.covariance[0] = 3.0;
  initial_pose_estimate.pose.covariance[7] = 3.0;
  // Sets really high covariance for yaw
  // We assume that the user is not going to point the
  // direction of the robot
  initial_pose_estimate.pose.covariance[35] = 99.0;

  initial_pose_publisher_->publish(initial_pose_estimate);
  initial_pose_published_ = true;
}

void GlobalLocalizationServer::AmclPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  if (!initial_pose_published_) return;

  // --- Convergence Detection Logic ---
  // We'll check the covariance matrix to determine if AMCL has converged.
  // Lower covariance values indicate higher confidence in the pose estimate.

  // Get the diagonal elements of the covariance matrix for position (x, y)
  // and orientation (yaw).
  // The covariance matrix is a 6x6 matrix flattened into a 36-element array.
  // Indices:
  // 0: x-x, 1: x-y, ..., 5: x-yaw
  // 6: y-x, 7: y-y, ..., 11: y-yaw
  // ...
  // 30: yaw-x, ..., 35: yaw-yaw
  double var_x = msg->pose.covariance[0];     // Variance in X position
  double var_y = msg->pose.covariance[7];     // Variance in Y position
  double var_yaw = msg->pose.covariance[35];  // Variance in Yaw orientation

  // Calculate a combined position covariance (e.g., sum of squares)
  current_position_uncertainty_ = std::sqrt(var_x + var_y);
  // For orientation, we directly use the yaw variance
  current_orientation_uncertainty_ = std::sqrt(var_yaw);

  current_pose_ = (*msg);

  // Check if uncertainty of the pose is lower than the threshold
  if (current_position_uncertainty_ < pos_uncertainty_threashold_ &&
      current_orientation_uncertainty_ < orientation_uncertainty_threashold_) {
    RCLCPP_INFO(node_->get_logger(),
                "[GlobalLocalizationServer] Robot localized");
    RobotLocalized();
  }
}

void GlobalLocalizationServer::CheckForObstacles() {
  RCLCPP_INFO(node_->get_logger(), "[GlobalLocalizationServer] Checking if obstacle near.");
  std::lock_guard<std::mutex> lock(mutex_);
  // Wait for one laser scan.
  if (lidar_reading_ == nullptr)
  {
    RCLCPP_INFO(node_->get_logger(), "[GlobalLocalizationServer] No lidar reading.");
    return;
  }

  double min_range = lidar_reading_->range_max;
  int min_index = -1;

  for (size_t i = 0; i < lidar_reading_->ranges.size(); ++i) {
    if (std::isfinite(lidar_reading_->ranges[i]) &&
        lidar_reading_->ranges[i] > lidar_reading_->range_min &&
        lidar_reading_->ranges[i] < min_range &&
        lidar_reading_->ranges[i] < kSafetyDistance) {

      double obstacle_angle = lidar_reading_->angle_min + i * lidar_reading_->angle_increment;
      obstacle_angle = NormalizeAngle(obstacle_angle);

      // If it's the first obstacle found in a sector, then add the sector to
      // the obstacles position vector.
      if (obstacle_angle < (M_PI / 2.0)) {
        if(std::find(obstacles_.begin(), obstacles_.end(), ObstaclesPosition::SECTOR_1) == obstacles_.end()) {
          obstacles_.push_back(ObstaclesPosition::SECTOR_1);
          RCLCPP_INFO(node_->get_logger(), "[GlobalLocalizationServer] Obstacle encountered on sector 1");
        }
      } else if (obstacle_angle < M_PI) {
        if(std::find(obstacles_.begin(), obstacles_.end(), ObstaclesPosition::SECTOR_2) == obstacles_.end()) {
          obstacles_.push_back(ObstaclesPosition::SECTOR_2);
          RCLCPP_INFO(node_->get_logger(), "[GlobalLocalizationServer] Obstacle encountered on sector 2");
        }
      } else if (obstacle_angle < (3 * M_PI / 4.0)) {
        if(std::find(obstacles_.begin(), obstacles_.end(), ObstaclesPosition::SECTOR_3) == obstacles_.end()) {
          obstacles_.push_back(ObstaclesPosition::SECTOR_3);
          RCLCPP_INFO(node_->get_logger(), "[GlobalLocalizationServer] Obstacle encountered on sector 3");
        }
      } else {
        if(std::find(obstacles_.begin(), obstacles_.end(), ObstaclesPosition::SECTOR_4) == obstacles_.end()) {
          obstacles_.push_back(ObstaclesPosition::SECTOR_4);
          RCLCPP_INFO(node_->get_logger(), "[GlobalLocalizationServer] Obstacle encountered on sector 4");
        }
      }
    }
  }
}

void GlobalLocalizationServer::LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  lidar_reading_ = msg;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalLocalizationServer>();

  std::shared_ptr<GlobalLocalizationServer> global_localization_action = std::make_shared<GlobalLocalizationServer>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    4);
  executor.add_node(global_localization_action->getNode());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
