#include "kimchi_state/navigation_manager.h"

#include <chrono>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <memory>
#include <nav2_lifecycle_manager/lifecycle_manager_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

NavigationManager::NavigationManager(std::shared_ptr<rclcpp::Node> node,
                                     std::shared_ptr<MissionObserver> observer)
    : node_(node), mission_observer_(observer), current_goal_(nullptr) {
  RCLCPP_INFO(node_->get_logger(), "NavigationManager initialized.");
  active_slam_toolbox_node_client_ =
      node_->create_client<lifecycle_msgs::srv::ChangeState>(
          "/slam_toolbox/change_state");

  client_localization_ =
      std::make_unique<nav2_lifecycle_manager::LifecycleManagerClient>(
          "lifecycle_manager_localization", node_);

  navigate_to_pose_action_client_ptr_ =
      rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
}

void NavigationManager::startSlam() {
  RCLCPP_INFO(node_->get_logger(), "Waiting for slam_toolbox service");
  active_slam_toolbox_node_client_->wait_for_service();
  RCLCPP_INFO(node_->get_logger(), "Finished waiting for slam_toolbox service");
  auto new_request =
      std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  new_request->transition.id = 3;  // Activate
  active_slam_toolbox_node_client_->async_send_request(new_request);
  RCLCPP_INFO(node_->get_logger(), "Starting SLAM");
}

void NavigationManager::stopSlam() {
  active_slam_toolbox_node_client_->wait_for_service();
  auto new_request =
      std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  new_request->transition.id = 4;  // Deactivate
  // TODO(arilow): Handle the response by passing a callback to
  // async_send_request
  active_slam_toolbox_node_client_->async_send_request(new_request);
}

void NavigationManager::startNavigation() {
  std::chrono::milliseconds wait_duration(100);

  // If is_active() returns TIMEOUT it means the lifecycle manager is not
  // configured yet. Waiting for it to be configured is a must before calling
  // the startup service.
  while (client_localization_->is_active(std::chrono::nanoseconds(100000)) ==
         nav2_lifecycle_manager::SystemStatus::TIMEOUT) {
    RCLCPP_INFO(node_->get_logger(),
                "Waiting for lifecycle_manager_localization to be configured");
    std::this_thread::sleep_for(wait_duration);
  }

  std::thread startup_loc_thread(
      std::bind(&nav2_lifecycle_manager::LifecycleManagerClient::startup,
                client_localization_.get(), std::placeholders::_1),
      wait_duration  // Direct argument instead of placeholder
  );

  startup_loc_thread.detach();
}

void NavigationManager::stopNavigation() {}

void NavigationManager::addGoalToMission(const Point2D& point) {
  RCLCPP_INFO(node_->get_logger(), "Adding point to path: (%f, %f)", point.x,
              point.y);
  goals_.push(point);
  onNewGoal();
}

void NavigationManager::goToNextGoal() {
  using namespace std::placeholders;
  if (goals_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "No goals to navigate to.");
    return;
  }

  auto navigation_goal = NavigateToPose::Goal();
  navigation_goal.pose.header.frame_id = "map";
  navigation_goal.pose.header.stamp = node_->now();
  navigation_goal.pose.pose.position.x = goals_.front().x;
  navigation_goal.pose.pose.position.y = goals_.front().y;
  navigation_goal.pose.pose.orientation.w = 1.0;  // Default orientation
  navigation_goal.pose.pose.orientation.x = 0.0;
  navigation_goal.pose.pose.orientation.y = 0.0;
  navigation_goal.pose.pose.orientation.z = 0.0;

  auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(
      &NavigationManager::navigateToPoseGoalResponseCallback, this, _1);
  send_goal_options.result_callback =
      std::bind(&NavigationManager::navigateToPoseResultCallback, this, _1);
  navigate_to_pose_action_client_ptr_->async_send_goal(navigation_goal,
                                                       send_goal_options);

  current_goal_ = std::make_unique<Point2D>(goals_.front());
  goals_.pop();
}

void NavigationManager::onNewGoal() {
  if (current_goal_ != nullptr) {
    RCLCPP_INFO(node_->get_logger(), "Already navigating to a goal: (%f, %f)",
                current_goal_->x, current_goal_->y);
    return;
  }
  goToNextGoal();
}

void NavigationManager::navigateToPoseGoalResponseCallback(
    GoalHandleNavigateToPose::SharedPtr goal_handle) {
  RCLCPP_INFO(node_->get_logger(),
              "NavigationManager::navigateToPoseGoalResponseCallback");

  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Goal was rejected by navigateToPose server");
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "Goal accepted by navigateToPose server, waiting for result");
    mission_observer_->onNavigatingToGoal(*current_goal_);
  }
}

void NavigationManager::navigateToPoseResultCallback(
    const GoalHandleNavigateToPose::WrappedResult& result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_ERROR(node_->get_logger(),
                   "navigateToPoseResultCallback: Goal reached!");

      if (goals_.empty()) {
        mission_observer_->onMissionFinished();
      } else {
        mission_observer_->onGoalReached(*current_goal_);
      }
      current_goal_.reset();  // Clear the current goal after success

      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(),
                   "navigateToPoseResultCallback: Goal was aborted. Error "
                   "code: %i. Message: %s",
                   result.result->error_code, result.result->error_msg.c_str());
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(),
                   "navigateToPoseResultCallback: Goal was canceled. Error "
                   "code: %i. Message: %s",
                   result.result->error_code, result.result->error_msg.c_str());
      return;
    default:
      RCLCPP_ERROR(node_->get_logger(),
                   "navigateToPoseResultCallback: Unknown result code");
      return;
  }
}
