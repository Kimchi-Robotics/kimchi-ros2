#pragma once

#include <tf2_ros/buffer.h>

#include <array>
#include <memory>
#include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

/**
 * Class to publish bumper obstacles as PointCloud2 messages.
 *
 * IMPORTANT NOTE: In theory, it should be enough to mark the obstacle in the in
 * one frame for both the global and local costmaps. But in practice, if not
 * marking the obstacle in the same frame of the costmap, the obstacle is not
 * considered. This is why the constructor receives frames of both local and
 * global costmap.
 */
class BumperObstaclePublisher {
 public:
  enum class BumperSide { LEFT, RIGHT, BOTH };

  BumperObstaclePublisher(std::string global_costmap_frame,
                          std::string local_costmap_frame,
                          std::string left_bumper_frame,
                          std::string right_bumper_frame,
                          std::shared_ptr<rclcpp::Node> node,
                          std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  /**
   * Publishes an obstacle as a PointCloud2 message in front of the robot.
   */
  void markObstacles(BumperSide side);

 private:
  void markObstacles(std::vector<std::string> bumper_frames);

  std::string getObstacleMessage(
    const geometry_msgs::msg::PointStamped& point, double radius);
  void setClearingTimer();

  std::string global_costmap_frame_;
  std::string local_costmap_frame_;
  std::string left_bumper_frame_;
  std::string right_bumper_frame_;

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // Global costmap virtual layer
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gc_virtual_layer_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gc_clear_virtual_layer_client_;


  // Local costmap virtual layer
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lc_virtual_layer_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr lc_clear_virtual_layer_client_;


  rclcpp::TimerBase::SharedPtr clear_obstacles_timer_;
};
