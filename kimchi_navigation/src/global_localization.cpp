#include "kimchi_navigation/global_localization.hpp"

GlobalLocalizationServer::GlobalLocalizationServer()
    : Node("global_localization") {
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

  this->action_server_ = rclcpp_action::create_server<GlobalLocalization>(
      this, "global_localization",
      std::bind(&GlobalLocalizationServer::handleGoal, this, _1, _2),
      std::bind(&GlobalLocalizationServer::handleCancel, this, _1),
      std::bind(&GlobalLocalizationServer::handleAccepted, this, _1));

  // Declare parameters for convergence thresholds
  this->declare_parameter<double>("position_covariance_threshold",
                                  0.5);  // meters^2
  this->declare_parameter<double>("orientation_covariance_threshold",
                                  0.05);  // radians^2

  // Get the configured thresholds
  pos_uncertainty_threashold_ =
      this->get_parameter("position_covariance_threshold").as_double();
  orientation_uncertainty_threashold_ =
      this->get_parameter("orientation_covariance_threshold").as_double();

  initial_pose_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", qos_profile);

  command_robot_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", qos_profile);

  // Create a subscription to the /amcl_pose topic
  // This is AMCL's output, which we will monitor for convergence.
  amcl_pose_subscription_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/amcl_pose", 10,
          std::bind(&GlobalLocalizationServer::AmclPoseCallback, this,
                    std::placeholders::_1));
}

rclcpp_action::GoalResponse GlobalLocalizationServer::handleGoal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const GlobalLocalization::Goal> goal) {
  RCLCPP_DEBUG(this->get_logger(), "Handling goal");
  (void)uuid;
  inital_pose_estimate_ = goal->pose_estimate;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalLocalizationServer::handleCancel(
    const std::shared_ptr<GoalHandleGlobalLocalization> goal_handle) {
  RCLCPP_DEBUG(this->get_logger(), "Handling Cancel");
  (void)goal_handle;

  StopRobot();

  return rclcpp_action::CancelResponse::ACCEPT;
}

void GlobalLocalizationServer::handleAccepted(
    const std::shared_ptr<GoalHandleGlobalLocalization> goal_handle) {
  RCLCPP_DEBUG(this->get_logger(), "Handling Accepted");
  using namespace std::placeholders;

  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  std::thread{std::bind(&GlobalLocalizationServer::execute, this, _1),
              goal_handle}
      .detach();
}

void GlobalLocalizationServer::cleanup() {
  RCLCPP_DEBUG(this->get_logger(), "Performing cleanup");

  // Always stop the robot first
  StopRobot();

  // Reset state flags
  robot_localized_ = false;
  initial_pose_published_ = false;
}

void GlobalLocalizationServer::RotateRobot() {
  geometry_msgs::msg::Twist robot_rotation;
  robot_rotation.angular.z = 1.0;

  command_robot_pub_->publish(robot_rotation);
  RCLCPP_INFO(this->get_logger(), "Rotating robot");
}

void GlobalLocalizationServer::StopRobot() {
  geometry_msgs::msg::Twist robot_rotation;
  robot_rotation.angular.z = 0.0;

  command_robot_pub_->publish(robot_rotation);
  RCLCPP_INFO(this->get_logger(), "Stoping robot");
}

void GlobalLocalizationServer::RobotLocalized() {
  RCLCPP_INFO(this->get_logger(), "Robot localization completed");
  robot_localized_ = true;
  StopRobot();
}

void GlobalLocalizationServer::execute(
    const std::shared_ptr<GoalHandleGlobalLocalization> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  //  Reset state for new goal
  robot_localized_ = false;
  initial_pose_published_ = false;

  // Initialize goal execution
  PublishInitialPoseWithHighVariance();
  RotateRobot();

  // Rate of 20hz
  rclcpp::Rate loop_rate(20);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<GlobalLocalization::Feedback>();
  auto result = std::make_shared<GlobalLocalization::Result>();

  while (!robot_localized_ && rclcpp::ok()) {
    // Check for cancellation first
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal cancellation requested");

      // Handles cleanup
      cleanup();

      result->localized_pose = current_pose_;
      result->localized = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    feedback->pose_feedback = current_pose_;
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  // Check if we exited due to successful localization
  if (rclcpp::ok() && robot_localized_) {
    RCLCPP_INFO(this->get_logger(), "Robot successfully localized");

    // Handles cleanup
    cleanup();

    result->localized_pose = current_pose_;
    result->localized = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
  } else {
    RCLCPP_WARN(this->get_logger(), "Node shutting down during goal execution");

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
  // TODO(@lola): Is it necessary to take into account the orientation?
  initial_pose_estimate.pose.pose.position.x = inital_pose_estimate_.position.x;
  initial_pose_estimate.pose.pose.position.y = inital_pose_estimate_.position.y;

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
  RCLCPP_DEBUG(this->get_logger(), "Localizing Robot ");

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
  double cov_xx = msg->pose.covariance[0];        // Variance in X position
  double cov_yy = msg->pose.covariance[7];        // Variance in Y position
  double cov_yaw_yaw = msg->pose.covariance[35];  // Variance in Yaw orientation

  // Calculate a combined position covariance (e.g., sum of squares)
  double position_uncertainty = std::sqrt(cov_xx + cov_yy);
  // For orientation, we directly use the yaw variance
  double orientation_uncertainty = std::sqrt(cov_yaw_yaw);

  current_pose_ = (*msg);

  // Check if uncertainty of the pose is lower than the threshold
  if (position_uncertainty < pos_uncertainty_threashold_ &&
      orientation_uncertainty < orientation_uncertainty_threashold_) {
    RCLCPP_INFO(this->get_logger(), "Robot localized");
    RobotLocalized();
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalLocalizationServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
