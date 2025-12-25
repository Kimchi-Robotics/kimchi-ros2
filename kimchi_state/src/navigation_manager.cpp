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
  RCLCPP_INFO(node_->get_logger(),
              "[NavigationManager] NavigationManager initialized.");

  slam_toolbox_client_ =
      std::make_unique<nav2_lifecycle_manager::LifecycleManagerClient>(
          "slam_toolbox_lifecycle_manager", node_);

  client_localization_ =
      std::make_unique<nav2_lifecycle_manager::LifecycleManagerClient>(
          "lifecycle_manager_localization", node_);

  client_navigation_ =
      std::make_unique<nav2_lifecycle_manager::LifecycleManagerClient>(
          "lifecycle_manager_navigation", node_);

  global_localization_action_client_ptr_ =
      rclcpp_action::create_client<GlobalLocalization>(node_,
                                                       "global_localization");

  navigate_to_pose_action_client_ptr_ =
      rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
}

void NavigationManager::startSlam() {
  std::chrono::seconds wait_duration(1);

  RCLCPP_INFO(node_->get_logger(),
              "[NavigationManager] Waiting for slam_toolbox service");
  // If is_active() returns TIMEOUT it means the lifecycle manager is not
  // configured yet. Waiting for it to be configured is a must before calling
  // the startup service.
  while (slam_toolbox_client_->is_active(wait_duration) ==
         nav2_lifecycle_manager::SystemStatus::TIMEOUT) {
    RCLCPP_INFO(node_->get_logger(),
                "[NavigationManager] Waiting for "
                "slam_toolbox_lifecycle_manager to be configured");
  }

  std::thread activate_slam_thread([this]() {
    slam_toolbox_client_->resume();
    mission_observer_->onSlamStarted();
  });
  activate_slam_thread.detach();
}

void NavigationManager::stopSlam() {
  std::thread stop_slam_thread([this]() { slam_toolbox_client_->pause(); });
  stop_slam_thread.detach();
}

void NavigationManager::startLocalization() {
  RCLCPP_ERROR(node_->get_logger(), "[Navigation Manager] Start Localization");

  std::chrono::seconds wait_duration(1);

  auto client_loc_status = client_localization_->is_active(wait_duration);

  if (client_loc_status == nav2_lifecycle_manager::SystemStatus::ACTIVE) {
    RCLCPP_INFO(node_->get_logger(),
                "[NavigationManager] Localization already active.");
    return;
  } else if (client_loc_status ==
             nav2_lifecycle_manager::SystemStatus::TIMEOUT) {
    while (client_localization_->is_active(wait_duration) ==
           nav2_lifecycle_manager::SystemStatus::TIMEOUT) {
      RCLCPP_INFO(node_->get_logger(),
                  "[NavigationManager] Waiting for "
                  "lifecycle_manager_localization to be configured");
    }
  }

  std::thread startup_loc_thread([this]() {
    client_localization_->startup();
    mission_observer_->onNav2LocalizationStarted();
  });
  startup_loc_thread.detach();
}

void NavigationManager::stopLocalization() {}

void NavigationManager::startNavigation() {
  RCLCPP_ERROR(node_->get_logger(), "[Navigation Manager] Start Navigation");

  std::chrono::seconds wait_duration(1);

  auto client_nav_status = client_navigation_->is_active(wait_duration);

  if (client_nav_status == nav2_lifecycle_manager::SystemStatus::ACTIVE) {
    RCLCPP_INFO(node_->get_logger(),
                "[NavigationManager] Navigation already active.");
    return;
  } else if (client_nav_status ==
             nav2_lifecycle_manager::SystemStatus::TIMEOUT) {
    while (client_navigation_->is_active(wait_duration) ==
           nav2_lifecycle_manager::SystemStatus::TIMEOUT) {
      RCLCPP_INFO(node_->get_logger(),
                  "[NavigationManager] Waiting for "
                  "lifecycle_manager_navigation to be configured");
    }
  }

  std::thread startup_nav_thread([this]() { client_navigation_->startup(); });

  startup_nav_thread.detach();
}

void NavigationManager::startLocating(const Point2D& point) {
  using namespace std::placeholders;

  RCLCPP_DEBUG(node_->get_logger(),
               "[NavigationManager] Goal received: (%f, %f)", point.x, point.y);

  auto localize_goal = GlobalLocalization::Goal();
  localize_goal.pose_estimate.x = point.x;
  localize_goal.pose_estimate.y = point.y;

  auto goal_handle_future =
      global_localization_action_client_ptr_->async_send_goal(localize_goal);
  if (goal_handle_future.wait_for(std::chrono::seconds(5)) !=
      std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[NavigationManager] LocalizationGoal was not accepted by "
                 "server in time.");
    mission_observer_->onLocalizationCancelled();
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "[NavigationManager] LocalizationGoal was rejected by server.");
    mission_observer_->onLocalizationCancelled();
  }

  auto result_future =
      global_localization_action_client_ptr_->async_get_result(goal_handle);
  mission_observer_->onLocalizationStarted();
  RCLCPP_INFO(node_->get_logger(),
              "[Worker Thread] Goal accepted. Waiting for result for up to %d "
              "seconds...",
              kMaxGlobalLocalizationWaitTimeSeconds);

  // Block and wait for the result for a maximum of 30 seconds.
  std::future_status status = result_future.wait_for(
      std::chrono::seconds(kMaxGlobalLocalizationWaitTimeSeconds));

  if (status == std::future_status::ready) {
    // Result was received within the 30-second window
    auto result = result_future.get();
    RCLCPP_INFO(node_->get_logger(), "[NavigationManager] Robot localized!");
    mission_observer_->onLocalizationSucceded();
  } else {
    // The future timed out after 30 seconds
    RCLCPP_WARN(node_->get_logger(),
                "[ERROR NavigationManager] Robot not localized in 1 minute.");

    RCLCPP_INFO(node_->get_logger(),
                "[Worker Thread] Sending cancel request to the server.");
    global_localization_action_client_ptr_->async_cancel_goal(goal_handle);
    mission_observer_->onLocalizationCancelled();
  }
}

void NavigationManager::addGoalToMission(const Point2D& point) {
  RCLCPP_DEBUG(node_->get_logger(),
               "[NavigationManager] Adding point to path: (%f, %f)", point.x,
               point.y);
  goals_.push(point);
  onNewGoal();
}

void NavigationManager::cancelMission() {
  RCLCPP_INFO(node_->get_logger(), "Cancelling current mission.");
  while (!goals_.empty()) {
    goals_.pop();
  }

  if (current_goal_ != nullptr) {
    cancelCurrentGoal();
  }
  mission_observer_->onMissionFinished();
}

void NavigationManager::cancelCurrentGoal() {
  if (current_goal_ == nullptr) {
    RCLCPP_INFO(node_->get_logger(), "No current goal to cancel.");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Cancelling current goal at point: (%f, %f)",
              current_goal_->x, current_goal_->y);
  navigate_to_pose_action_client_ptr_->async_cancel_all_goals();
}

void NavigationManager::goToNextGoal() {
  using namespace std::placeholders;
  if (goals_.empty()) {
    RCLCPP_DEBUG(node_->get_logger(),
                 "[NavigationManager] No goals to navigate to.");
    return;
  }

  if (!navigate_to_pose_action_client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "/navigate_to_pose action server not available after waiting");
    while (!goals_.empty()) goals_.pop();
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
    RCLCPP_DEBUG(node_->get_logger(),
                 "[NavigationManager] Already navigating to a goal: (%f, %f)",
                 current_goal_->x, current_goal_->y);
    return;
  }
  goToNextGoal();
}

void NavigationManager::navigateToPoseGoalResponseCallback(
    GoalHandleNavigateToPose::SharedPtr goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "[NavigationManager] Goal was rejected by navigateToPose server");
    while (!goals_.empty()) { goals_.pop(); }
    mission_observer_->onMissionFinished();
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "[NavigationManager] Goal accepted by navigateToPose server, "
                "waiting for result");
    mission_observer_->onNavigatingToGoal(*current_goal_);
  }
}

void NavigationManager::navigateToPoseResultCallback(
    const GoalHandleNavigateToPose::WrappedResult& result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(
          node_->get_logger(),
          "[NavigationManager] navigateToPoseResultCallback: Goal reached!");

      if (goals_.empty()) {
        mission_observer_->onMissionFinished();
      } else {
        mission_observer_->onGoalReached(*current_goal_);
      }
      current_goal_.reset();  // Clear the current goal after success

      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_DEBUG(node_->get_logger(),
                   "[NavigationManager] navigateToPoseResultCallback: Goal was "
                   "aborted. Error "
                   "code: %i. Message: %s",
                   result.result->error_code, result.result->error_msg.c_str());
      if (goals_.empty()) {
        mission_observer_->onMissionFinished();
      } else {
        mission_observer_->onGoalReached(*current_goal_);
      }
      current_goal_.reset();  // Clear the current goal after success

      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(node_->get_logger(),
                   "[NavigationManager] navigateToPoseResultCallback: Goal was "
                   "canceled. Error "
                   "code: %i. Message: %s",
                   result.result->error_code, result.result->error_msg.c_str());
      if (goals_.empty()) {
        mission_observer_->onMissionFinished();
      } else {
        mission_observer_->onGoalCancelled(*current_goal_);
      }
      current_goal_.reset();  // Clear the current goal after success

      return;
    default:
      RCLCPP_DEBUG(node_->get_logger(),
                   "[NavigationManager] navigateToPoseResultCallback: Unknown "
                   "result code");
      return;
  }
}
