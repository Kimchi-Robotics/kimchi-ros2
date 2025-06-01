#include <memory>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

/**
 * @brief A ROS2 C++ node that subscribes to /amcl_pose,
 * detects when AMCL has converged (based on covariance),
 * and then publishes that pose once to /initial_pose for Nav2.
 */
class AmclInitialPoseTrigger : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the AmclInitialPoseTrigger node.
     */
    AmclInitialPoseTrigger()
        : Node("amcl_initial_pose_trigger_node"),
          has_published_initial_pose_(false) // Flag to ensure we publish only once
    {
        auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_t{
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            10, // depth
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            RMW_QOS_POLICY_DURABILITY_VOLATILE,
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false // avoid_ros_namespace_conventions
        }));
        // Create a subscription to the /amcl_pose topic
        // This is AMCL's output, which we will monitor for convergence.
        amcl_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose",
            qos_profile, // QoS history depth
            std::bind(&AmclInitialPoseTrigger::amclPoseCallback, this, std::placeholders::_1));

        // Create a publisher for the /initial_pose topic
        // Nav2 subscribes to this topic to get its starting pose.
        initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            qos_profile); // QoS history depth

        // Declare parameters for convergence thresholds
        this->declare_parameter<double>("position_covariance_threshold", 0.5); // meters^2
        this->declare_parameter<double>("orientation_covariance_threshold", 0.05); // radians^2

        RCLCPP_INFO(this->get_logger(), "AMCL Initial Pose Trigger Node has started.");
        RCLCPP_INFO(this->get_logger(), "Waiting for AMCL to converge...");
    }

private:
    /**
     * @brief Callback function for received /amcl_pose messages.
     * @param msg The received PoseWithCovarianceStamped message from AMCL.
     */
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // If we've already published the initial pose, do nothing further.
        if (has_published_initial_pose_) {
            RCLCPP_ERROR(this->get_logger(), "Published intial pose");
            return;
        }

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

        double cov_xx = msg->pose.covariance[0];  // Variance in X position
        double cov_yy = msg->pose.covariance[7];  // Variance in Y position
        double cov_yaw_yaw = msg->pose.covariance[35]; // Variance in Yaw orientation

        // Get the configured thresholds
        double pos_cov_threshold = this->get_parameter("position_covariance_threshold").as_double();
        double orient_cov_threshold = this->get_parameter("orientation_covariance_threshold").as_double();

        // Calculate a combined position covariance (e.g., sum of squares)
        double position_uncertainty = std::sqrt(cov_xx + cov_yy);
        // For orientation, we directly use the yaw variance
        double orientation_uncertainty = std::sqrt(cov_yaw_yaw);

        RCLCPP_ERROR(this->get_logger(),
                     "Current AMCL Covariance: Pos_unc: %.4f (thresh: %.4f), Orient_unc: %.4f (thresh: %.4f)",
                     position_uncertainty, pos_cov_threshold, orientation_uncertainty, orient_cov_threshold);

        // Check if both position and orientation uncertainties are below their respective thresholds
        if (position_uncertainty < pos_cov_threshold &&
            orientation_uncertainty < orient_cov_threshold)
        {
            RCLCPP_INFO(this->get_logger(),
                        "AMCL has converged! Publishing initial pose to /initial_pose.");
            RCLCPP_INFO(this->get_logger(),
                        "  Converged Pose: (x=%.2f, y=%.2f, yaw=%.2f rad)",
                        msg->pose.pose.position.x, msg->pose.pose.position.y,
                        get_yaw_from_quaternion(msg->pose.pose.orientation));

            // Publish the converged pose to /initial_pose
            initial_pose_publisher_->publish(*msg);

            // Set the flag to true so we don't publish again
            has_published_initial_pose_ = true;

            // Optionally, you might want to shut down this node after publishing
            // to avoid consuming resources, or keep it alive for debugging/re-triggering.
            // rclcpp::shutdown(); // Uncomment if you want the node to exit after publishing
        } else {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 10000, // Log every 5 seconds
                                 "AMCL not yet converged. Position uncertainty: %.4f (threshold: %.4f), Orientation uncertainty: %.4f (threshold: %.4f)",
                                 position_uncertainty, pos_cov_threshold, orientation_uncertainty, orient_cov_threshold);
        }
    }

    /**
     * @brief Helper function to convert a quaternion to yaw (Z-axis rotation).
     * @param q The quaternion.
     * @return The yaw angle in radians.
     */
    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
    {
        // This is a simplified conversion for 2D (yaw-only)
        // For full 3D, you'd use a proper quaternion to Euler conversion library
        return std::atan2(2.0 * (q.z * q.w + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
    bool has_published_initial_pose_;
};

/**
 * @brief Main function to initialize and run the ROS2 node.
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // Initialize ROS2
    // Create an instance of the AmclInitialPoseTrigger node and spin it
    rclcpp::spin(std::make_shared<AmclInitialPoseTrigger>());
    rclcpp::shutdown(); // Shutdown ROS2
    return 0;
}
