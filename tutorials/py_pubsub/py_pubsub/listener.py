import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """Minimal example of listener node"""

    def __init__(self):
        """Constructor"""
        super().__init__("minimal_subscriber")
        self._subscription = self.create_subscription(String, "chatter", self.listener_callback, 10)

    def listener_callback(self, msg):
        """Subscription callback

        Args:
        ----
            msg (msg.String): Received message

        """
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    """Main entrypoint"""
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
