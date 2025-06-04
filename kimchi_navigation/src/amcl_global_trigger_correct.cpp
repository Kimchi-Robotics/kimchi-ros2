#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h> // For creating quaternions
#include <memory>
#include <chrono>

/*
 * @brief A ROS2 C++ node that publishes a PoseWithCovarianceStamped message
 * to /initial_pose with a very high covariance to trigger AMCL's global re-initialization.
 */
class AmclGlobalTriggerCorrect : public rclcpp::Node
{
public:
    /*
     * @brief Constructor for the AmclGlobalTriggerCorrect node.
     */
    AmclGlobalTriggerCorrect()
        : Node("amcl_global_trigger_correct_node")
    {
        auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_t{
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            1, // depth
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            RMW_QOS_POLICY_DURABILITY_VOLATILE,
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false // avoid_ros_namespace_conventions
        }));
        // Create a publisher for the /initial_pose topic
        initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            qos_profile); // QoS depth of 1 is sufficient for a one-shot trigger

        // Use a one-shot timer to publish the message shortly after startup
        // This ensures the publisher is ready before sending the message.
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), // Wait 500ms
            std::bind(&AmclGlobalTriggerCorrect::publish_trigger_and_shutdown, this)
        );

        RCLCPP_INFO(this->get_logger(), "AMCL Global Trigger (Correct Method) Node started.");
        RCLCPP_INFO(this->get_logger(), "Will send high-covariance /initial_pose in 500ms to trigger global localization...");
    }

private:
    /*
     * @brief Callback function for the one-shot timer. Publishes the trigger and shuts down.
     */
    void publish_trigger_and_shutdown()
    {
        // Cancel the timer so it only runs once
        timer_->cancel();

        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg;
        initial_pose_msg.header.stamp = this->now();
        initial_pose_msg.header.frame_id = "map"; // Always publish in the 'map' frame

        // Set the pose to the map origin (or any known point, doesn't matter much with high covariance)
        initial_pose_msg.pose.pose.position.x = 0.0;
        initial_pose_msg.pose.pose.position.y = 0.0;
        initial_pose_msg.pose.pose.position.z = 0.0;

        // Set orientation to no rotation (identity quaternion)
        tf2::Quaternion q;
        q.setRPY(0, 0, 0); // Roll, Pitch, Yaw
        initial_pose_msg.pose.pose.orientation.x = q.x();
        initial_pose_msg.pose.pose.orientation.y = q.y();
        initial_pose_msg.pose.pose.orientation.z = q.z();
        initial_pose_msg.pose.pose.orientation.w = q.w();

        // --- Set very large covariance values to signal global initialization ---
        // These values represent high uncertainty in position (x, y) and orientation (yaw).
        // Common large values are 0.25 (for small uncertainty) to 9999.0 for very high uncertainty.
        // We'll use 0.5 * 0.5 = 0.25 (m^2) for small and 999999.0 for global
        double big_covariance = 999999.0;
        double orientation_covariance = 999999.0; // Large value for orientation uncertainty

        // Initialize all covariance elements to a small value first
        for (int i = 0; i < 36; ++i) {
            initial_pose_msg.pose.covariance[i] = 0.0; // Start with zeros
        }

        // Set large values on the diagonal for position (x, y, z) and orientation (roll, pitch, yaw)
        // Position diagonal: [0](x-x), [7](y-y), [14](z-z)
        // Orientation diagonal: [21](roll-roll), [28](pitch-pitch), [35](yaw-yaw)
        initial_pose_msg.pose.covariance[0] = big_covariance;    // Variance in x
        initial_pose_msg.pose.covariance[7] = big_covariance;    // Variance in y
        initial_pose_msg.pose.covariance[14] = big_covariance;   // Variance in z
        initial_pose_msg.pose.covariance[21] = big_covariance;   // Variance in roll
        initial_pose_msg.pose.covariance[28] = big_covariance;   // Variance in pitch
        initial_pose_msg.pose.covariance[35] = orientation_covariance; // Variance in yaw

        initial_pose_publisher_->publish(initial_pose_msg);
        RCLCPP_INFO(this->get_logger(), "Published high-covariance /initial_pose to trigger AMCL global localization.");
        RCLCPP_INFO(this->get_logger(), "Shutting down AmclGlobalTriggerCorrect node.");
        // rclcpp::shutdown(); // Shut down this node after sending the trigger
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

/*
 * @brief Main function to initialize and run the ROS2 node.
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AmclGlobalTriggerCorrect>());
    return 0;
}
