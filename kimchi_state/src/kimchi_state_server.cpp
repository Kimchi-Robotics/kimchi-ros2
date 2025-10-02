#include "kimchi_state/kimchi_state_server.h"

#include <chrono>
#include <filesystem>
#include <functional>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/wait_for_message.hpp"
#include <std_srvs/srv/trigger.hpp>

#include <kimchi_state/map_info.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

namespace {
std::string toString(RobotState robot_state) {
  switch (robot_state) {
    case RobotState::NO_MAP:
      return "NO_MAP";
    case RobotState::MAPPING_WITH_EXPLORATION:
      return "MAPPING_WITH_EXPLORATION";
    case RobotState::MAPPING_WITH_TELEOP:
      return "MAPPING_WITH_TELEOP";
    case RobotState::NAVIGATING:
      return "NAVIGATING";
    case RobotState::LOCATING:
      return "LOCATING";
    case RobotState::TELEOP:
      return "TELEOP";
    case RobotState::IDLE:
      return "IDLE";
    case RobotState::LOST:
      return "LOST";
    case RobotState::RECOVERING:
      return "RECOVERING";
    case RobotState::GOAL_REACHED:
      return "GOAL_REACHED";
    case RobotState::CHARGING:
      return "CHARGING";
  }
  return "UNKNOWN_STATE";
}
}  // namespace

std::shared_ptr<KimchiStateServer> KimchiStateServer::Create(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) {
  auto output =
      std::shared_ptr<KimchiStateServer>(new KimchiStateServer(options));
  output->initialize();
  return output;
}

void KimchiStateServer::onNav2LocalizationStarted() {
  RCLCPP_INFO(node_->get_logger(), "[KimchiStateServer] Localization started");
  if (state_ == RobotState::MAPPING_WITH_EXPLORATION ||
      state_ == RobotState::MAPPING_WITH_TELEOP) {
   startLocating(current_robot_position_);
  } else {
    changeState(RobotState::LOST);
  }
}

void KimchiStateServer::onSlamStarted() {
  RCLCPP_INFO(node_->get_logger(), "[KimchiStateServer] SLAM started");
  changeState(RobotState::MAPPING_WITH_TELEOP);
}

void KimchiStateServer::onLocalizationStarted()
{
  RCLCPP_INFO(node_->get_logger(), "[KimchiStateServer] Robot localizing.");
  changeState(RobotState::LOCATING);
}

void KimchiStateServer::onLocalizationCancelled() {
  RCLCPP_ERROR(node_->get_logger(),"[KimchiStateServer] Localization action cancelled.");
  changeState(RobotState::LOST);
}

void KimchiStateServer::onLocalizationSucceded() {
  RCLCPP_ERROR(node_->get_logger(), "[KimchiStateServer] Robot localized correclty.");
  changeState(RobotState::IDLE);
}

void KimchiStateServer::onNavigatingToGoal(const Point2D &point) {
  changeState(RobotState::NAVIGATING);
  RCLCPP_INFO(node_->get_logger(), "[KimchiStateServer] Navigating to goal at point: (%f, %f)",
              point.x, point.y);
}

void KimchiStateServer::onGoalReached(const Point2D &point) {
  changeState(RobotState::GOAL_REACHED);
  RCLCPP_DEBUG(node_->get_logger(),
               "[KimchiStateServer] Goal reached at point: (%f, %f)", point.x,
               point.y);
}

void KimchiStateServer::onGoalCancelled(const Point2D &point) {
  changeState(RobotState::GOAL_REACHED);
  RCLCPP_INFO(node_->get_logger(), "[KimchiStateServer] Goal cancelled: (%f, %f)", point.x,
              point.y);
}

void KimchiStateServer::onMissionFinished() {
  changeState(RobotState::IDLE);
  RCLCPP_INFO(node_->get_logger(), "[KimchiStateServer] Mission finished");
}

void KimchiStateServer::initialize() {
  navigation_manager_ =
      std::make_unique<NavigationManager>(node_, shared_from_this());
  // Create a QoS profile with best effort for sharing the state of the robot.
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10),
                         qos_profile);
  qos.best_effort();

  // Publish state.
  state_publisher_ =
      node_->create_publisher<kimchi_interfaces::msg::RobotState>(
          "/kimchi_state_server/state", qos);
  state_publisher_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&KimchiStateServer::statePublisherTimerCallback, this));

  // Subscribe to the map info service
  get_map_info_client_ = node_->create_client<kimchi_interfaces::srv::MapInfo>(
      "/kimchi_map/get_map_info");

  // Subscribe to the save map service
  save_map_client_ =
      node_->create_client<nav2_msgs::srv::SaveMap>("/map_saver/save_map");

  start_slam_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/kimchi_state_server/start_slam",
      std::bind(&KimchiStateServer::startSlamCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  start_navigation_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/kimchi_state_server/start_navigation",
      std::bind(&KimchiStateServer::startNavigationCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  add_goal_to_mission_service_ =
      node_->create_service<kimchi_interfaces::srv::ProccessSelectedPosition>(
          "/kimchi_state_server/add_goal_to_mission",
          std::bind(&KimchiStateServer::addGoalToMissionCallback, this,
                    std::placeholders::_1, std::placeholders::_2));

  start_locating_service_ =
      node_->create_service<kimchi_interfaces::srv::ProccessSelectedPosition>(
          "/kimchi_state_server/localize",
          std::bind(&KimchiStateServer::initialPoseCallback, this,
                    std::placeholders::_1, std::placeholders::_2));

  send_command_service_ =
      node_->create_service<kimchi_interfaces::srv::KimchiStateServerCommand>(
          "/kimchi_state_server/send_command",
          std::bind(&KimchiStateServer::sendCommandCallback, this,
                    std::placeholders::_1, std::placeholders::_2));

  joint_states_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&KimchiStateServer::jointStatesCallback, this,
                std::placeholders::_1));

  // Initialize tf2 buffer and listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Create a timer to periodically check the transform
  tf_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&KimchiStateServer::getRobotPositionFromTF, this));

  // Call the map info service
  callGetMapInfoService();
}

KimchiStateServer::KimchiStateServer(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : node_(new rclcpp::Node("kimchi_state_server", options)),
      navigation_manager_(nullptr),
      state_(RobotState::NO_MAP),
      current_robot_position_{0.0, 0.0} {}

void KimchiStateServer::statePublisherTimerCallback() {
  auto message = kimchi_interfaces::msg::RobotState();
  message.state = static_cast<uint32_t>(state_.load());
  state_publisher_->publish(message);
}

void KimchiStateServer::callGetMapInfoService() {
  while (!get_map_info_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[KimchiStateServer] Interrupted while waiting for the "
                   "service. Exiting.");
      return;
    }
    RCLCPP_INFO(node_->get_logger(),
                "[KimchiStateServer] Service not available, waiting again...");
  }

  auto request = std::make_shared<kimchi_interfaces::srv::MapInfo::Request>();
  request->str_place_holder = "May you share the map info please?";

  auto future = get_map_info_client_->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(),
                                         future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = future.get();
    if (result->success) {
      map_info_ = std::make_unique<MapInfo>(result->resolution, result->origin,
                                            result->map_image);
      auto set_map_filename_future = SetMapFileName();
      rclcpp::spin_until_future_complete(node_->get_node_base_interface(),
                                         set_map_filename_future);
      startNavigation();

    } else {
      changeState(RobotState::NO_MAP);
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service /kimchi_map/get_map_info");
  }
}

void KimchiStateServer::startSlamCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  navigation_manager_->startSlam();
  response->success = true;
}

void KimchiStateServer::initialPoseCallback(
    const kimchi_interfaces::srv::ProccessSelectedPosition::Request::SharedPtr
        request,
    kimchi_interfaces::srv::ProccessSelectedPosition::Response::SharedPtr
        response) {
  if (state_ != RobotState::LOST) {
    response->success = false;

    return;
  }

  startLocating(Point2D(request->goal.x, request->goal.y));
  response->success = true;

  return;
}

void KimchiStateServer::startLocating(const Point2D& point) {
  std::thread locate_thread([this, point]() {
    navigation_manager_->startLocating(point);
  });
  locate_thread.detach();
}

void KimchiStateServer::startNavigationCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  if (state_ == RobotState::NO_MAP) {
    response->success = false;
    response->message = "There's no map available. Can't start navigation.";
    return;
  }

  response->success = true;

  if (state_ == RobotState::MAPPING_WITH_TELEOP) {
    std::thread map_saved_callback_thread([this]() {
      auto save_map_future = saveMap();
      navigation_manager_->stopSlam();
      auto set_map_filename_future = SetMapFileName();
      save_map_future.wait();
      set_map_filename_future.wait();
      startNavigation();
    });
    map_saved_callback_thread.detach();
    return;
  }

  startNavigation();
}

void KimchiStateServer::startNavigation() {
  navigation_manager_->startNavigation();
}

void KimchiStateServer::sendCommandCallback(
    const kimchi_interfaces::srv::KimchiStateServerCommand::Request::SharedPtr
        request,
    kimchi_interfaces::srv::KimchiStateServerCommand::Response::SharedPtr
        response) {
  if (request->command == "continue_path") {
    navigation_manager_->goToNextGoal();
    response->success = true;
  } else if (request->command == "cancel_navigation_goal") {
    navigation_manager_->cancelCurrentGoal();
    response->success = true;
  } else if (request->command == "cancel_navigation_mission") {
    navigation_manager_->cancelMission();
  } else {
    response->success = false;
    response->msg = "Unknown command: " + request->command;
  }
}

void KimchiStateServer::addGoalToMissionCallback(
    const kimchi_interfaces::srv::ProccessSelectedPosition::Request::SharedPtr
        request,
    kimchi_interfaces::srv::ProccessSelectedPosition::Response::SharedPtr
        response) {
  navigation_manager_->addGoalToMission(
      Point2D(request->goal.x, request->goal.y));
  response->success = true;
}

std::shared_future<nav2_msgs::srv::SaveMap::Response::SharedPtr>
KimchiStateServer::saveMap() {
  save_map_client_->wait_for_service();
  auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();

  request->map_topic = "/map";
  request->map_url = "kimchi_map";
  request->image_format = "png";
  request->map_mode = "trinary";
  request->free_thresh = 0.25;
  request->occupied_thresh = 0.65;

  auto future = save_map_client_->async_send_request(request);
  return future.share();  // Return the future to allow waiting for completion
}

std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
KimchiStateServer::SetMapFileName() {
  // TODO(Arilow): Instead of hardcoding the map file name, we should make it a
  // parameter.
  auto map_server_param_client =
      std::make_shared<rclcpp::AsyncParametersClient>(node_, "/map_server");
  if (map_server_param_client->wait_for_service(std::chrono::seconds(1))) {
    auto future = map_server_param_client->set_parameters({rclcpp::Parameter(
        "yaml_filename",
        std::filesystem::absolute("kimchi_map.yaml").c_str())});
    // Handle future result...
    return future;
  }
  RCLCPP_ERROR(node_->get_logger(),
               "[KimchiStateServer] Failed to set map file name, map_server service not available");
  return std::shared_future<
      std::vector<rcl_interfaces::msg::SetParametersResult>>{};
}

void KimchiStateServer::changeState(RobotState new_state) {
  state_ = new_state;
  RCLCPP_INFO(node_->get_logger(), "State changed to: %s",
              toString(state_).c_str());
}

// TODO(lneumarkt): Implement behavior based on bumper and button states
void KimchiStateServer::jointStatesCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg) {

  for (size_t i = 0; i < msg->name.size(); ++i) {
    const std::string& joint_name = msg->name[i];
    
    // For all three joints, we consider a position of non-zero as "pressed"
    if (joint_name == "button_joint") {
      button_state_ = msg->position[i] > 0.01;
      RCLCPP_DEBUG(node_->get_logger(), "Button state: %s", 
                  button_state_ ? "PRESSED" : "NOT_PRESSED");
      if (state_ == RobotState::GOAL_REACHED && button_state_) {
        navigation_manager_->goToNextGoal();
      }
    }
    else if (joint_name == "left_bumper_joint") {
      left_bumper_state_ = msg->position[i] > 0.01;
      RCLCPP_DEBUG(node_->get_logger(), "Left bumper state: %s", 
                  left_bumper_state_ ? "PRESSED" : "NOT_PRESSED");
    }
    else if (joint_name == "right_bumper_joint") {
      right_bumper_state_ = msg->position[i] > 0.01;
      RCLCPP_DEBUG(node_->get_logger(), "Right bumper state: %s", 
                  right_bumper_state_ ? "PRESSED" : "NOT_PRESSED");
    }
  }
}

void KimchiStateServer::getRobotPositionFromTF() {
    std::string target_frame = "map";
    std::string source_frame = "base_link";
    
    try {
        // Look up the transform from base_link to map
        geometry_msgs::msg::TransformStamped transform_stamped = 
            tf_buffer_->lookupTransform(
                target_frame, 
                source_frame,
                tf2::TimePointZero);  // Get the latest available transform
        
        // Extract position
        current_robot_position_.x = transform_stamped.transform.translation.x;
        current_robot_position_.y = transform_stamped.transform.translation.y;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(node_->get_logger(), 
            "Could not transform %s to %s: %s", 
            source_frame.c_str(), target_frame.c_str(), ex.what());
    }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<KimchiStateServer> kimchi_state_server =
      KimchiStateServer::Create(rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    4);
  executor.add_node(kimchi_state_server->getNode());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
