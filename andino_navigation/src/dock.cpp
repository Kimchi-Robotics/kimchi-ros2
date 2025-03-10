#include "behaviors/dock.hpp"

#include "nav2_behaviors/timed_behavior.hpp"

namespace andino_navigation {

Dock::Dock()
    : nav2_behaviors::TimedBehavior<DockAction>(), state_(State::WAITING) {}

Dock::~Dock() {}

void Dock::onConfigure() {
  auto node = node_.lock();
  navigate_to_pose_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          node, "/navigate_to_pose");
}

nav2_behaviors::Status Dock::onRun(
    const std::shared_ptr<const DockActionGoal> command) {
  auto node = node_.lock();
  RCLCPP_INFO(node->get_logger(), "Dock::onRun() called");

  if (!navigate_to_pose_client_->wait_for_action_server()) {
    RCLCPP_ERROR(node->get_logger(),
                 "Action server not available after waiting");
    return nav2_behaviors::Status{nav2_behaviors::Status::FAILED};
  }

  using namespace std::placeholders;
  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = node->get_clock()->now();

  goal_msg.pose.pose.position.x = command->dock_pose.x;
  goal_msg.pose.pose.position.y = command->dock_pose.y;
  goal_msg.pose.pose.position.z = 0;
  goal_msg.pose.pose.orientation.x = 0;
  goal_msg.pose.pose.orientation.y = 0;
  goal_msg.pose.pose.orientation.z = 0;
  goal_msg.pose.pose.orientation.w = 1;

  auto send_goal_options = rclcpp_action::Client<
      nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&Dock::goalResponseCallback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&Dock::goalFeedbackCallback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&Dock::goalResultCallback, this, _1);
  navigate_to_pose_client_->async_send_goal(goal_msg, send_goal_options);

  state_ = State::RUNNING;
  return nav2_behaviors::Status{nav2_behaviors::Status::SUCCEEDED};
}

nav2_behaviors::Status Dock::onCycleUpdate() {
  auto node = node_.lock();
  auto return_status = nav2_behaviors::Status{nav2_behaviors::Status::RUNNING};

  switch (state_) {
    case State::RUNNING:
      RCLCPP_INFO(node->get_logger(), "Dock::onCycleUpdate() RUNNING");
      return return_status;
      break;
    case State::REJECTED:
      RCLCPP_INFO(node->get_logger(),
                  "Dock::onCycleUpdate() goal rejected by /navigate_to_pose "
                  "action server");
      return_status = nav2_behaviors::Status::FAILED;
      break;
    case State::CANCELED:
      RCLCPP_INFO(node->get_logger(), "Dock::onCycleUpdate() goal canceled");
      return_status = nav2_behaviors::Status::FAILED;
      break;
    case State::ABORTED:
      RCLCPP_INFO(node->get_logger(), "Dock::onCycleUpdate() goal aborted");
      return_status = nav2_behaviors::Status::FAILED;
      break;
    case State::SUCCEEDED:
      RCLCPP_INFO(node->get_logger(), "Dock::onCycleUpdate() Goal reached");
      return_status = nav2_behaviors::Status::SUCCEEDED;
      break;
    default:
      RCLCPP_ERROR(node->get_logger(), "Dock::onCycleUpdate() Unknown state");
      return nav2_behaviors::Status{nav2_behaviors::Status::FAILED};
  }

  state_ = State::WAITING;
  return return_status;
}

void Dock::goalResponseCallback(
    const GoalHandleNavigateToPose::SharedPtr& goal_handle) {
  auto node = node_.lock();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    state_ = State::REJECTED;
  } else {
    RCLCPP_INFO(node->get_logger(),
                "Goal accepted by server, waiting for result");
    state_ = State::RUNNING;
  }
  return;
}

void Dock::goalFeedbackCallback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
  auto node = node_.lock();

  std::stringstream ss;
  ss << "Current pose is x: " << feedback->current_pose.pose.position.x
     << " y: " << feedback->current_pose.pose.position.y
     << " theta: " << feedback->current_pose.pose.orientation.z;
  RCLCPP_INFO(node->get_logger(), ss.str().c_str());
  return;
}

void Dock::goalResultCallback(
    const GoalHandleNavigateToPose::WrappedResult& result) {
  auto node = node_.lock();
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      state_ = State::SUCCEEDED;
      RCLCPP_INFO(node->get_logger(), "Goal reached");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      state_ = State::ABORTED;
      RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      state_ = State::CANCELED;
      RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(node->get_logger(), "Unknown result code");
      return;
  }

  return;
}

}  // namespace andino_navigation

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(andino_navigation::Dock, nav2_core::Behavior)
