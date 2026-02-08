#include "kimchi_state/bumper_obstacle_publisher.h"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/string.hpp>

BumperObstaclePublisher::BumperObstaclePublisher(
    std::string global_costmap_frame, std::string local_costmap_frame,
    std::string left_bumper_frame, std::string right_bumper_frame,
    std::shared_ptr<rclcpp::Node> node, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : global_costmap_frame_(global_costmap_frame),
      local_costmap_frame_(local_costmap_frame),
      left_bumper_frame_(left_bumper_frame),
      right_bumper_frame_(right_bumper_frame),
      node_(node),
      tf_buffer_(tf_buffer) {
  gc_virtual_layer_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/global_costmap/virtual_layer/shapes", 10);
  gc_clear_virtual_layer_client_ = node_->create_client<std_srvs::srv::Trigger>(
      "/global_costmap/virtual_layer/clear_all");
  lc_virtual_layer_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/local_costmap/virtual_layer/shapes", 10);
  lc_clear_virtual_layer_client_ = node_->create_client<std_srvs::srv::Trigger>(
      "/local_costmap/virtual_layer/clear_all");

}

void BumperObstaclePublisher::markObstacles(BumperSide side) {
  switch (side) {
    case BumperSide::LEFT:
      markObstacles({left_bumper_frame_});
      break;
    case BumperSide::RIGHT:
      markObstacles({right_bumper_frame_});
      break;
    case BumperSide::BOTH:
      markObstacles({left_bumper_frame_, right_bumper_frame_});
      break;
  }
}

void BumperObstaclePublisher::markObstacles(std::vector<std::string> bumper_frames) {
  try {
    std::vector<geometry_msgs::msg::PointStamped> points;
    for (const auto& bumper_frame : bumper_frames) {
      geometry_msgs::msg::TransformStamped transform;
      transform = tf_buffer_->lookupTransform("base_link", bumper_frame,
                                              tf2::TimePointZero);

      double x = transform.transform.translation.x;
      double y = transform.transform.translation.y;

      geometry_msgs::msg::PointStamped point;
      point.point.x = x;
      point.point.y = y;
      point.point.z = 0.1f;  // height
      points.push_back(point);
    }


    // Get transforms to local and global costmap frames
    auto tf_to_odom = tf_buffer_->lookupTransform(
        local_costmap_frame_, "base_link", tf2::TimePointZero);
    auto tf_to_map = tf_buffer_->lookupTransform(
        global_costmap_frame_, "base_link", tf2::TimePointZero);

    // // Transform points to local and global costmap frames.
    for (const auto& pt : points) {
      geometry_msgs::msg::PointStamped p_odom, p_map;
      tf2::doTransform(pt, p_odom, tf_to_odom);
      tf2::doTransform(pt, p_map, tf_to_map);

      std_msgs::msg::String lc_msg;
      lc_msg.data = getObstacleMessage(p_odom, 0.08);

      std_msgs::msg::String gc_msg;
      gc_msg.data = getObstacleMessage(p_map, 0.05);

      lc_virtual_layer_pub_->publish(lc_msg);
      gc_virtual_layer_pub_->publish(gc_msg);
    }

    setClearingTimer();
 
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[KimchiStateServer] Failed to get transform: %s", ex.what());
  }
}

std::string BumperObstaclePublisher::getObstacleMessage(
    const geometry_msgs::msg::PointStamped& point, double radius) {
  std::string obstacle_msg;
  obstacle_msg += "CIRCLE(" + std::to_string(point.point.x) + " " +
                  std::to_string(point.point.y) + " " +
                  std::to_string(radius) + ") [COST:254]";
  return obstacle_msg;
}

void BumperObstaclePublisher::setClearingTimer() {
  // Set up a timer to clear the obstacles after 60 seconds
  if (!clear_obstacles_timer_ || clear_obstacles_timer_->is_canceled()) {
    RCLCPP_INFO(node_->get_logger(), "[KimchiStateServer] Setting timer for clearing obstacles");

    clear_obstacles_timer_ = node_->create_wall_timer(
        std::chrono::seconds(60),
        [this]() {
          RCLCPP_INFO(node_->get_logger(), "[KimchiStateServer] Clearing obstacles");
          auto gc_request = std::make_shared<std_srvs::srv::Trigger::Request>();
          auto lc_request = std::make_shared<std_srvs::srv::Trigger::Request>();
          gc_clear_virtual_layer_client_->async_send_request(gc_request);
          lc_clear_virtual_layer_client_->async_send_request(lc_request);
          clear_obstacles_timer_->cancel();
        });
  } else {
    RCLCPP_INFO(node_->get_logger(), "[KimchiStateServer] Resetting timer for clearing obstacles");
    clear_obstacles_timer_->reset();
  }
}
